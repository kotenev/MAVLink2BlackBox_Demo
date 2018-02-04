
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
        public class POSITION_TARGET_LOCAL_NED : GroundControl.POSITION_TARGET_LOCAL_NED
        {
            /**
            *Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or
            *	0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
            *	the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
            *	2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
            *	bit 11: yaw, bit 12: yaw rat*/
            public ushort type_mask
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public uint time_boot_ms //Timestamp in milliseconds since system boot
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  2);}
            }

            public float x //X Position in NED frame in meters
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 6);}
            }

            public float y //Y Position in NED frame in meters
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 10);}
            }

            public float z //Z Position in NED frame in meters (note, altitude is negative in NED)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 14);}
            }

            public float vx //X velocity in NED frame in meter / s
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 18);}
            }

            public float vy //Y velocity in NED frame in meter / s
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 22);}
            }

            public float vz //Z velocity in NED frame in meter / s
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 26);}
            }

            public float afx //X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 30);}
            }

            public float afy //Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 34);}
            }

            public float afz //Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 38);}
            }

            public float yaw //yaw setpoint in rad
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 42);}
            }

            public float yaw_rate //yaw rate setpoint in rad/s
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 46);}
            }

            /**
            *Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED
            *	=*/
            public MAV_FRAME coordinate_frame
            {
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 400);}
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
                        case MAV_CMD.MAV_CMD_NAV_ALTITUDE_WAIT:
                            id = 17;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_VTOL_TAKEOFF:
                            id = 18;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_VTOL_LAND:
                            id = 19;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_GUIDED_ENABLE:
                            id = 20;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_DELAY:
                            id = 21;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_PAYLOAD_PLACE:
                            id = 22;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_LAST:
                            id = 23;
                            break;
                        case MAV_CMD.MAV_CMD_CONDITION_DELAY:
                            id = 24;
                            break;
                        case MAV_CMD.MAV_CMD_CONDITION_CHANGE_ALT:
                            id = 25;
                            break;
                        case MAV_CMD.MAV_CMD_CONDITION_DISTANCE:
                            id = 26;
                            break;
                        case MAV_CMD.MAV_CMD_CONDITION_YAW:
                            id = 27;
                            break;
                        case MAV_CMD.MAV_CMD_CONDITION_LAST:
                            id = 28;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_MODE:
                            id = 29;
                            break;
                        case MAV_CMD.MAV_CMD_DO_JUMP:
                            id = 30;
                            break;
                        case MAV_CMD.MAV_CMD_DO_CHANGE_SPEED:
                            id = 31;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_HOME:
                            id = 32;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_PARAMETER:
                            id = 33;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_RELAY:
                            id = 34;
                            break;
                        case MAV_CMD.MAV_CMD_DO_REPEAT_RELAY:
                            id = 35;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_SERVO:
                            id = 36;
                            break;
                        case MAV_CMD.MAV_CMD_DO_REPEAT_SERVO:
                            id = 37;
                            break;
                        case MAV_CMD.MAV_CMD_DO_FLIGHTTERMINATION:
                            id = 38;
                            break;
                        case MAV_CMD.MAV_CMD_DO_CHANGE_ALTITUDE:
                            id = 39;
                            break;
                        case MAV_CMD.MAV_CMD_DO_LAND_START:
                            id = 40;
                            break;
                        case MAV_CMD.MAV_CMD_DO_RALLY_LAND:
                            id = 41;
                            break;
                        case MAV_CMD.MAV_CMD_DO_GO_AROUND:
                            id = 42;
                            break;
                        case MAV_CMD.MAV_CMD_DO_REPOSITION:
                            id = 43;
                            break;
                        case MAV_CMD.MAV_CMD_DO_PAUSE_CONTINUE:
                            id = 44;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_REVERSE:
                            id = 45;
                            break;
                        case MAV_CMD.MAV_CMD_DO_CONTROL_VIDEO:
                            id = 46;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_ROI:
                            id = 47;
                            break;
                        case MAV_CMD.MAV_CMD_DO_DIGICAM_CONFIGURE:
                            id = 48;
                            break;
                        case MAV_CMD.MAV_CMD_DO_DIGICAM_CONTROL:
                            id = 49;
                            break;
                        case MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE:
                            id = 50;
                            break;
                        case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL:
                            id = 51;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST:
                            id = 52;
                            break;
                        case MAV_CMD.MAV_CMD_DO_FENCE_ENABLE:
                            id = 53;
                            break;
                        case MAV_CMD.MAV_CMD_DO_PARACHUTE:
                            id = 54;
                            break;
                        case MAV_CMD.MAV_CMD_DO_MOTOR_TEST:
                            id = 55;
                            break;
                        case MAV_CMD.MAV_CMD_DO_INVERTED_FLIGHT:
                            id = 56;
                            break;
                        case MAV_CMD.MAV_CMD_DO_GRIPPER:
                            id = 57;
                            break;
                        case MAV_CMD.MAV_CMD_DO_AUTOTUNE_ENABLE:
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
                        case MAV_CMD.MAV_CMD_POWER_OFF_INITIATED:
                            id = 130;
                            break;
                        case MAV_CMD.MAV_CMD_SOLO_BTN_FLY_CLICK:
                            id = 131;
                            break;
                        case MAV_CMD.MAV_CMD_SOLO_BTN_FLY_HOLD:
                            id = 132;
                            break;
                        case MAV_CMD.MAV_CMD_SOLO_BTN_PAUSE_CLICK:
                            id = 133;
                            break;
                        case MAV_CMD.MAV_CMD_DO_START_MAG_CAL:
                            id = 134;
                            break;
                        case MAV_CMD.MAV_CMD_DO_ACCEPT_MAG_CAL:
                            id = 135;
                            break;
                        case MAV_CMD.MAV_CMD_DO_CANCEL_MAG_CAL:
                            id = 136;
                            break;
                        case MAV_CMD.MAV_CMD_SET_FACTORY_TEST_MODE:
                            id = 137;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SEND_BANNER:
                            id = 138;
                            break;
                        case MAV_CMD.MAV_CMD_ACCELCAL_VEHICLE_POS:
                            id = 139;
                            break;
                        case MAV_CMD.MAV_CMD_GIMBAL_RESET:
                            id = 140;
                            break;
                        case MAV_CMD.MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS:
                            id = 141;
                            break;
                        case MAV_CMD.MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION:
                            id = 142;
                            break;
                        case MAV_CMD.MAV_CMD_GIMBAL_FULL_RESET:
                            id = 143;
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
                        case MAV_CMD.MAV_CMD_NAV_ALTITUDE_WAIT:
                            id = 17;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_VTOL_TAKEOFF:
                            id = 18;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_VTOL_LAND:
                            id = 19;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_GUIDED_ENABLE:
                            id = 20;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_DELAY:
                            id = 21;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_PAYLOAD_PLACE:
                            id = 22;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_LAST:
                            id = 23;
                            break;
                        case MAV_CMD.MAV_CMD_CONDITION_DELAY:
                            id = 24;
                            break;
                        case MAV_CMD.MAV_CMD_CONDITION_CHANGE_ALT:
                            id = 25;
                            break;
                        case MAV_CMD.MAV_CMD_CONDITION_DISTANCE:
                            id = 26;
                            break;
                        case MAV_CMD.MAV_CMD_CONDITION_YAW:
                            id = 27;
                            break;
                        case MAV_CMD.MAV_CMD_CONDITION_LAST:
                            id = 28;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_MODE:
                            id = 29;
                            break;
                        case MAV_CMD.MAV_CMD_DO_JUMP:
                            id = 30;
                            break;
                        case MAV_CMD.MAV_CMD_DO_CHANGE_SPEED:
                            id = 31;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_HOME:
                            id = 32;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_PARAMETER:
                            id = 33;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_RELAY:
                            id = 34;
                            break;
                        case MAV_CMD.MAV_CMD_DO_REPEAT_RELAY:
                            id = 35;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_SERVO:
                            id = 36;
                            break;
                        case MAV_CMD.MAV_CMD_DO_REPEAT_SERVO:
                            id = 37;
                            break;
                        case MAV_CMD.MAV_CMD_DO_FLIGHTTERMINATION:
                            id = 38;
                            break;
                        case MAV_CMD.MAV_CMD_DO_CHANGE_ALTITUDE:
                            id = 39;
                            break;
                        case MAV_CMD.MAV_CMD_DO_LAND_START:
                            id = 40;
                            break;
                        case MAV_CMD.MAV_CMD_DO_RALLY_LAND:
                            id = 41;
                            break;
                        case MAV_CMD.MAV_CMD_DO_GO_AROUND:
                            id = 42;
                            break;
                        case MAV_CMD.MAV_CMD_DO_REPOSITION:
                            id = 43;
                            break;
                        case MAV_CMD.MAV_CMD_DO_PAUSE_CONTINUE:
                            id = 44;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_REVERSE:
                            id = 45;
                            break;
                        case MAV_CMD.MAV_CMD_DO_CONTROL_VIDEO:
                            id = 46;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_ROI:
                            id = 47;
                            break;
                        case MAV_CMD.MAV_CMD_DO_DIGICAM_CONFIGURE:
                            id = 48;
                            break;
                        case MAV_CMD.MAV_CMD_DO_DIGICAM_CONTROL:
                            id = 49;
                            break;
                        case MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE:
                            id = 50;
                            break;
                        case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL:
                            id = 51;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST:
                            id = 52;
                            break;
                        case MAV_CMD.MAV_CMD_DO_FENCE_ENABLE:
                            id = 53;
                            break;
                        case MAV_CMD.MAV_CMD_DO_PARACHUTE:
                            id = 54;
                            break;
                        case MAV_CMD.MAV_CMD_DO_MOTOR_TEST:
                            id = 55;
                            break;
                        case MAV_CMD.MAV_CMD_DO_INVERTED_FLIGHT:
                            id = 56;
                            break;
                        case MAV_CMD.MAV_CMD_DO_GRIPPER:
                            id = 57;
                            break;
                        case MAV_CMD.MAV_CMD_DO_AUTOTUNE_ENABLE:
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
                        case MAV_CMD.MAV_CMD_POWER_OFF_INITIATED:
                            id = 130;
                            break;
                        case MAV_CMD.MAV_CMD_SOLO_BTN_FLY_CLICK:
                            id = 131;
                            break;
                        case MAV_CMD.MAV_CMD_SOLO_BTN_FLY_HOLD:
                            id = 132;
                            break;
                        case MAV_CMD.MAV_CMD_SOLO_BTN_PAUSE_CLICK:
                            id = 133;
                            break;
                        case MAV_CMD.MAV_CMD_DO_START_MAG_CAL:
                            id = 134;
                            break;
                        case MAV_CMD.MAV_CMD_DO_ACCEPT_MAG_CAL:
                            id = 135;
                            break;
                        case MAV_CMD.MAV_CMD_DO_CANCEL_MAG_CAL:
                            id = 136;
                            break;
                        case MAV_CMD.MAV_CMD_SET_FACTORY_TEST_MODE:
                            id = 137;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SEND_BANNER:
                            id = 138;
                            break;
                        case MAV_CMD.MAV_CMD_ACCELCAL_VEHICLE_POS:
                            id = 139;
                            break;
                        case MAV_CMD.MAV_CMD_GIMBAL_RESET:
                            id = 140;
                            break;
                        case MAV_CMD.MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS:
                            id = 141;
                            break;
                        case MAV_CMD.MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION:
                            id = 142;
                            break;
                        case MAV_CMD.MAV_CMD_GIMBAL_FULL_RESET:
                            id = 143;
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
                        case MAV_CMD.MAV_CMD_NAV_ALTITUDE_WAIT:
                            id = 17;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_VTOL_TAKEOFF:
                            id = 18;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_VTOL_LAND:
                            id = 19;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_GUIDED_ENABLE:
                            id = 20;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_DELAY:
                            id = 21;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_PAYLOAD_PLACE:
                            id = 22;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_LAST:
                            id = 23;
                            break;
                        case MAV_CMD.MAV_CMD_CONDITION_DELAY:
                            id = 24;
                            break;
                        case MAV_CMD.MAV_CMD_CONDITION_CHANGE_ALT:
                            id = 25;
                            break;
                        case MAV_CMD.MAV_CMD_CONDITION_DISTANCE:
                            id = 26;
                            break;
                        case MAV_CMD.MAV_CMD_CONDITION_YAW:
                            id = 27;
                            break;
                        case MAV_CMD.MAV_CMD_CONDITION_LAST:
                            id = 28;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_MODE:
                            id = 29;
                            break;
                        case MAV_CMD.MAV_CMD_DO_JUMP:
                            id = 30;
                            break;
                        case MAV_CMD.MAV_CMD_DO_CHANGE_SPEED:
                            id = 31;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_HOME:
                            id = 32;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_PARAMETER:
                            id = 33;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_RELAY:
                            id = 34;
                            break;
                        case MAV_CMD.MAV_CMD_DO_REPEAT_RELAY:
                            id = 35;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_SERVO:
                            id = 36;
                            break;
                        case MAV_CMD.MAV_CMD_DO_REPEAT_SERVO:
                            id = 37;
                            break;
                        case MAV_CMD.MAV_CMD_DO_FLIGHTTERMINATION:
                            id = 38;
                            break;
                        case MAV_CMD.MAV_CMD_DO_CHANGE_ALTITUDE:
                            id = 39;
                            break;
                        case MAV_CMD.MAV_CMD_DO_LAND_START:
                            id = 40;
                            break;
                        case MAV_CMD.MAV_CMD_DO_RALLY_LAND:
                            id = 41;
                            break;
                        case MAV_CMD.MAV_CMD_DO_GO_AROUND:
                            id = 42;
                            break;
                        case MAV_CMD.MAV_CMD_DO_REPOSITION:
                            id = 43;
                            break;
                        case MAV_CMD.MAV_CMD_DO_PAUSE_CONTINUE:
                            id = 44;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_REVERSE:
                            id = 45;
                            break;
                        case MAV_CMD.MAV_CMD_DO_CONTROL_VIDEO:
                            id = 46;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_ROI:
                            id = 47;
                            break;
                        case MAV_CMD.MAV_CMD_DO_DIGICAM_CONFIGURE:
                            id = 48;
                            break;
                        case MAV_CMD.MAV_CMD_DO_DIGICAM_CONTROL:
                            id = 49;
                            break;
                        case MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE:
                            id = 50;
                            break;
                        case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL:
                            id = 51;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST:
                            id = 52;
                            break;
                        case MAV_CMD.MAV_CMD_DO_FENCE_ENABLE:
                            id = 53;
                            break;
                        case MAV_CMD.MAV_CMD_DO_PARACHUTE:
                            id = 54;
                            break;
                        case MAV_CMD.MAV_CMD_DO_MOTOR_TEST:
                            id = 55;
                            break;
                        case MAV_CMD.MAV_CMD_DO_INVERTED_FLIGHT:
                            id = 56;
                            break;
                        case MAV_CMD.MAV_CMD_DO_GRIPPER:
                            id = 57;
                            break;
                        case MAV_CMD.MAV_CMD_DO_AUTOTUNE_ENABLE:
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
                        case MAV_CMD.MAV_CMD_POWER_OFF_INITIATED:
                            id = 130;
                            break;
                        case MAV_CMD.MAV_CMD_SOLO_BTN_FLY_CLICK:
                            id = 131;
                            break;
                        case MAV_CMD.MAV_CMD_SOLO_BTN_FLY_HOLD:
                            id = 132;
                            break;
                        case MAV_CMD.MAV_CMD_SOLO_BTN_PAUSE_CLICK:
                            id = 133;
                            break;
                        case MAV_CMD.MAV_CMD_DO_START_MAG_CAL:
                            id = 134;
                            break;
                        case MAV_CMD.MAV_CMD_DO_ACCEPT_MAG_CAL:
                            id = 135;
                            break;
                        case MAV_CMD.MAV_CMD_DO_CANCEL_MAG_CAL:
                            id = 136;
                            break;
                        case MAV_CMD.MAV_CMD_SET_FACTORY_TEST_MODE:
                            id = 137;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SEND_BANNER:
                            id = 138;
                            break;
                        case MAV_CMD.MAV_CMD_ACCELCAL_VEHICLE_POS:
                            id = 139;
                            break;
                        case MAV_CMD.MAV_CMD_GIMBAL_RESET:
                            id = 140;
                            break;
                        case MAV_CMD.MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS:
                            id = 141;
                            break;
                        case MAV_CMD.MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION:
                            id = 142;
                            break;
                        case MAV_CMD.MAV_CMD_GIMBAL_FULL_RESET:
                            id = 143;
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
                        case MAV_CMD.MAV_CMD_NAV_ALTITUDE_WAIT:
                            id = 17;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_VTOL_TAKEOFF:
                            id = 18;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_VTOL_LAND:
                            id = 19;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_GUIDED_ENABLE:
                            id = 20;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_DELAY:
                            id = 21;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_PAYLOAD_PLACE:
                            id = 22;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_LAST:
                            id = 23;
                            break;
                        case MAV_CMD.MAV_CMD_CONDITION_DELAY:
                            id = 24;
                            break;
                        case MAV_CMD.MAV_CMD_CONDITION_CHANGE_ALT:
                            id = 25;
                            break;
                        case MAV_CMD.MAV_CMD_CONDITION_DISTANCE:
                            id = 26;
                            break;
                        case MAV_CMD.MAV_CMD_CONDITION_YAW:
                            id = 27;
                            break;
                        case MAV_CMD.MAV_CMD_CONDITION_LAST:
                            id = 28;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_MODE:
                            id = 29;
                            break;
                        case MAV_CMD.MAV_CMD_DO_JUMP:
                            id = 30;
                            break;
                        case MAV_CMD.MAV_CMD_DO_CHANGE_SPEED:
                            id = 31;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_HOME:
                            id = 32;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_PARAMETER:
                            id = 33;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_RELAY:
                            id = 34;
                            break;
                        case MAV_CMD.MAV_CMD_DO_REPEAT_RELAY:
                            id = 35;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_SERVO:
                            id = 36;
                            break;
                        case MAV_CMD.MAV_CMD_DO_REPEAT_SERVO:
                            id = 37;
                            break;
                        case MAV_CMD.MAV_CMD_DO_FLIGHTTERMINATION:
                            id = 38;
                            break;
                        case MAV_CMD.MAV_CMD_DO_CHANGE_ALTITUDE:
                            id = 39;
                            break;
                        case MAV_CMD.MAV_CMD_DO_LAND_START:
                            id = 40;
                            break;
                        case MAV_CMD.MAV_CMD_DO_RALLY_LAND:
                            id = 41;
                            break;
                        case MAV_CMD.MAV_CMD_DO_GO_AROUND:
                            id = 42;
                            break;
                        case MAV_CMD.MAV_CMD_DO_REPOSITION:
                            id = 43;
                            break;
                        case MAV_CMD.MAV_CMD_DO_PAUSE_CONTINUE:
                            id = 44;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_REVERSE:
                            id = 45;
                            break;
                        case MAV_CMD.MAV_CMD_DO_CONTROL_VIDEO:
                            id = 46;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_ROI:
                            id = 47;
                            break;
                        case MAV_CMD.MAV_CMD_DO_DIGICAM_CONFIGURE:
                            id = 48;
                            break;
                        case MAV_CMD.MAV_CMD_DO_DIGICAM_CONTROL:
                            id = 49;
                            break;
                        case MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE:
                            id = 50;
                            break;
                        case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL:
                            id = 51;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST:
                            id = 52;
                            break;
                        case MAV_CMD.MAV_CMD_DO_FENCE_ENABLE:
                            id = 53;
                            break;
                        case MAV_CMD.MAV_CMD_DO_PARACHUTE:
                            id = 54;
                            break;
                        case MAV_CMD.MAV_CMD_DO_MOTOR_TEST:
                            id = 55;
                            break;
                        case MAV_CMD.MAV_CMD_DO_INVERTED_FLIGHT:
                            id = 56;
                            break;
                        case MAV_CMD.MAV_CMD_DO_GRIPPER:
                            id = 57;
                            break;
                        case MAV_CMD.MAV_CMD_DO_AUTOTUNE_ENABLE:
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
                        case MAV_CMD.MAV_CMD_POWER_OFF_INITIATED:
                            id = 130;
                            break;
                        case MAV_CMD.MAV_CMD_SOLO_BTN_FLY_CLICK:
                            id = 131;
                            break;
                        case MAV_CMD.MAV_CMD_SOLO_BTN_FLY_HOLD:
                            id = 132;
                            break;
                        case MAV_CMD.MAV_CMD_SOLO_BTN_PAUSE_CLICK:
                            id = 133;
                            break;
                        case MAV_CMD.MAV_CMD_DO_START_MAG_CAL:
                            id = 134;
                            break;
                        case MAV_CMD.MAV_CMD_DO_ACCEPT_MAG_CAL:
                            id = 135;
                            break;
                        case MAV_CMD.MAV_CMD_DO_CANCEL_MAG_CAL:
                            id = 136;
                            break;
                        case MAV_CMD.MAV_CMD_SET_FACTORY_TEST_MODE:
                            id = 137;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SEND_BANNER:
                            id = 138;
                            break;
                        case MAV_CMD.MAV_CMD_ACCELCAL_VEHICLE_POS:
                            id = 139;
                            break;
                        case MAV_CMD.MAV_CMD_GIMBAL_RESET:
                            id = 140;
                            break;
                        case MAV_CMD.MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS:
                            id = 141;
                            break;
                        case MAV_CMD.MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION:
                            id = 142;
                            break;
                        case MAV_CMD.MAV_CMD_GIMBAL_FULL_RESET:
                            id = 143;
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
                        case MAV_CMD.MAV_CMD_NAV_ALTITUDE_WAIT:
                            id = 17;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_VTOL_TAKEOFF:
                            id = 18;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_VTOL_LAND:
                            id = 19;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_GUIDED_ENABLE:
                            id = 20;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_DELAY:
                            id = 21;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_PAYLOAD_PLACE:
                            id = 22;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_LAST:
                            id = 23;
                            break;
                        case MAV_CMD.MAV_CMD_CONDITION_DELAY:
                            id = 24;
                            break;
                        case MAV_CMD.MAV_CMD_CONDITION_CHANGE_ALT:
                            id = 25;
                            break;
                        case MAV_CMD.MAV_CMD_CONDITION_DISTANCE:
                            id = 26;
                            break;
                        case MAV_CMD.MAV_CMD_CONDITION_YAW:
                            id = 27;
                            break;
                        case MAV_CMD.MAV_CMD_CONDITION_LAST:
                            id = 28;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_MODE:
                            id = 29;
                            break;
                        case MAV_CMD.MAV_CMD_DO_JUMP:
                            id = 30;
                            break;
                        case MAV_CMD.MAV_CMD_DO_CHANGE_SPEED:
                            id = 31;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_HOME:
                            id = 32;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_PARAMETER:
                            id = 33;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_RELAY:
                            id = 34;
                            break;
                        case MAV_CMD.MAV_CMD_DO_REPEAT_RELAY:
                            id = 35;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_SERVO:
                            id = 36;
                            break;
                        case MAV_CMD.MAV_CMD_DO_REPEAT_SERVO:
                            id = 37;
                            break;
                        case MAV_CMD.MAV_CMD_DO_FLIGHTTERMINATION:
                            id = 38;
                            break;
                        case MAV_CMD.MAV_CMD_DO_CHANGE_ALTITUDE:
                            id = 39;
                            break;
                        case MAV_CMD.MAV_CMD_DO_LAND_START:
                            id = 40;
                            break;
                        case MAV_CMD.MAV_CMD_DO_RALLY_LAND:
                            id = 41;
                            break;
                        case MAV_CMD.MAV_CMD_DO_GO_AROUND:
                            id = 42;
                            break;
                        case MAV_CMD.MAV_CMD_DO_REPOSITION:
                            id = 43;
                            break;
                        case MAV_CMD.MAV_CMD_DO_PAUSE_CONTINUE:
                            id = 44;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_REVERSE:
                            id = 45;
                            break;
                        case MAV_CMD.MAV_CMD_DO_CONTROL_VIDEO:
                            id = 46;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_ROI:
                            id = 47;
                            break;
                        case MAV_CMD.MAV_CMD_DO_DIGICAM_CONFIGURE:
                            id = 48;
                            break;
                        case MAV_CMD.MAV_CMD_DO_DIGICAM_CONTROL:
                            id = 49;
                            break;
                        case MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE:
                            id = 50;
                            break;
                        case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL:
                            id = 51;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST:
                            id = 52;
                            break;
                        case MAV_CMD.MAV_CMD_DO_FENCE_ENABLE:
                            id = 53;
                            break;
                        case MAV_CMD.MAV_CMD_DO_PARACHUTE:
                            id = 54;
                            break;
                        case MAV_CMD.MAV_CMD_DO_MOTOR_TEST:
                            id = 55;
                            break;
                        case MAV_CMD.MAV_CMD_DO_INVERTED_FLIGHT:
                            id = 56;
                            break;
                        case MAV_CMD.MAV_CMD_DO_GRIPPER:
                            id = 57;
                            break;
                        case MAV_CMD.MAV_CMD_DO_AUTOTUNE_ENABLE:
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
                        case MAV_CMD.MAV_CMD_POWER_OFF_INITIATED:
                            id = 130;
                            break;
                        case MAV_CMD.MAV_CMD_SOLO_BTN_FLY_CLICK:
                            id = 131;
                            break;
                        case MAV_CMD.MAV_CMD_SOLO_BTN_FLY_HOLD:
                            id = 132;
                            break;
                        case MAV_CMD.MAV_CMD_SOLO_BTN_PAUSE_CLICK:
                            id = 133;
                            break;
                        case MAV_CMD.MAV_CMD_DO_START_MAG_CAL:
                            id = 134;
                            break;
                        case MAV_CMD.MAV_CMD_DO_ACCEPT_MAG_CAL:
                            id = 135;
                            break;
                        case MAV_CMD.MAV_CMD_DO_CANCEL_MAG_CAL:
                            id = 136;
                            break;
                        case MAV_CMD.MAV_CMD_SET_FACTORY_TEST_MODE:
                            id = 137;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SEND_BANNER:
                            id = 138;
                            break;
                        case MAV_CMD.MAV_CMD_ACCELCAL_VEHICLE_POS:
                            id = 139;
                            break;
                        case MAV_CMD.MAV_CMD_GIMBAL_RESET:
                            id = 140;
                            break;
                        case MAV_CMD.MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS:
                            id = 141;
                            break;
                        case MAV_CMD.MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION:
                            id = 142;
                            break;
                        case MAV_CMD.MAV_CMD_GIMBAL_FULL_RESET:
                            id = 143;
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
        public class MANUAL_SETPOINT : GroundControl.MANUAL_SETPOINT
        {
            public uint time_boot_ms //Timestamp in milliseconds since system boot
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public float roll //Desired roll rate in radians per second
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 4);}
            }

            public float pitch //Desired pitch rate in radians per second
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float yaw //Desired yaw rate in radians per second
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float thrust //Collective thrust, normalized to 0 .. 1
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public byte mode_switch //Flight mode switch position, 0.. 255
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  20);}
            }

            public byte manual_override_switch //Override mode switch position, 0.. 255
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  21);}
            }
        }
        public class SET_ATTITUDE_TARGET : GroundControl.SET_ATTITUDE_TARGET
        {
            public uint time_boot_ms //Timestamp in milliseconds since system boot
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public byte target_system //System ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  4);}
            }

            public byte target_component //Component ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  5);}
            }

            /**
            *Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate,
            *	bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 6: reserved, bit 7: throttle, bit 8: attitud*/
            public byte type_mask
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  6);}
            }

            public float[] q //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
            {
                set {q_SET(value, 0)  ;}
            }
            public void q_SET(float[] src, int pos)  //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
            {
                for(int BYTE =  7, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }
            public float body_roll_rate //Body roll rate in radians per second
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 23);}
            }

            public float body_pitch_rate //Body roll rate in radians per second
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 27);}
            }

            public float body_yaw_rate //Body roll rate in radians per second
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 31);}
            }

            public float thrust //Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 35);}
            }
        }
        public class ATTITUDE_TARGET : GroundControl.ATTITUDE_TARGET
        {
            public uint time_boot_ms //Timestamp in milliseconds since system boot
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            /**
            *Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate,
            *	bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 7: reserved, bit 8: attitud*/
            public byte type_mask
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  4);}
            }

            public float[] q //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
            {
                set {q_SET(value, 0)  ;}
            }
            public void q_SET(float[] src, int pos)  //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
            {
                for(int BYTE =  5, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }
            public float body_roll_rate //Body roll rate in radians per second
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 21);}
            }

            public float body_pitch_rate //Body pitch rate in radians per second
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 25);}
            }

            public float body_yaw_rate //Body yaw rate in radians per second
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 29);}
            }

            public float thrust //Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 33);}
            }
        }
        public class SET_POSITION_TARGET_LOCAL_NED : GroundControl.SET_POSITION_TARGET_LOCAL_NED
        {
            /**
            *Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or
            *	0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
            *	the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
            *	2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
            *	bit 11: yaw, bit 12: yaw rat*/
            public ushort type_mask
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public uint time_boot_ms //Timestamp in milliseconds since system boot
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  2);}
            }

            public byte target_system //System ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  6);}
            }

            public byte target_component //Component ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  7);}
            }

            public float x //X Position in NED frame in meters
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float y //Y Position in NED frame in meters
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float z //Z Position in NED frame in meters (note, altitude is negative in NED)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float vx //X velocity in NED frame in meter / s
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float vy //Y velocity in NED frame in meter / s
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float vz //Z velocity in NED frame in meter / s
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }

            public float afx //X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 32);}
            }

            public float afy //Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 36);}
            }

            public float afz //Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 40);}
            }

            public float yaw //yaw setpoint in rad
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 44);}
            }

            public float yaw_rate //yaw rate setpoint in rad/s
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 48);}
            }

            /**
            *Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED
            *	=*/
            public MAV_FRAME coordinate_frame
            {
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 416);}
            }
        }
        public class SET_POSITION_TARGET_GLOBAL_INT : GroundControl.SET_POSITION_TARGET_GLOBAL_INT
        {
            /**
            *Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or
            *	0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
            *	the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
            *	2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
            *	bit 11: yaw, bit 12: yaw rat*/
            public ushort type_mask
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            /**
            *Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow
            *	the system to compensate for the transport delay of the setpoint. This allows the system to compensate
            *	processing latency*/
            public uint time_boot_ms
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  2);}
            }

            public byte target_system //System ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  6);}
            }

            public byte target_component //Component ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  7);}
            }

            public int lat_int //X Position in WGS84 frame in 1e7 * meters
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  8);}
            }

            public int lon_int //Y Position in WGS84 frame in 1e7 * meters
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  12);}
            }

            public float alt //Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_IN
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float vx //X velocity in NED frame in meter / s
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float vy //Y velocity in NED frame in meter / s
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float vz //Z velocity in NED frame in meter / s
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }

            public float afx //X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 32);}
            }

            public float afy //Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 36);}
            }

            public float afz //Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 40);}
            }

            public float yaw //yaw setpoint in rad
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 44);}
            }

            public float yaw_rate //yaw rate setpoint in rad/s
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 48);}
            }

            /**
            *Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT
            *	= 1*/
            public MAV_FRAME coordinate_frame
            {
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 416);}
            }
        }
        public class POSITION_TARGET_GLOBAL_INT : GroundControl.POSITION_TARGET_GLOBAL_INT
        {
            /**
            *Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or
            *	0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
            *	the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
            *	2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
            *	bit 11: yaw, bit 12: yaw rat*/
            public ushort type_mask
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            /**
            *Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow
            *	the system to compensate for the transport delay of the setpoint. This allows the system to compensate
            *	processing latency*/
            public uint time_boot_ms
            {
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  2);}
            }

            public int lat_int //X Position in WGS84 frame in 1e7 * meters
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  6);}
            }

            public int lon_int //Y Position in WGS84 frame in 1e7 * meters
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  10);}
            }

            public float alt //Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_IN
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 14);}
            }

            public float vx //X velocity in NED frame in meter / s
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 18);}
            }

            public float vy //Y velocity in NED frame in meter / s
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 22);}
            }

            public float vz //Z velocity in NED frame in meter / s
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 26);}
            }

            public float afx //X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 30);}
            }

            public float afy //Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 34);}
            }

            public float afz //Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 38);}
            }

            public float yaw //yaw setpoint in rad
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 42);}
            }

            public float yaw_rate //yaw rate setpoint in rad/s
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 46);}
            }

            /**
            *Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT
            *	= 1*/
            public MAV_FRAME coordinate_frame
            {
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 400);}
            }
        }
        public class LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET : GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET
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

            public float roll //Roll
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float pitch //Pitch
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float yaw //Yaw
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }
        }
        public class HIL_STATE : GroundControl.HIL_STATE
        {
            public ulong time_usec //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public float roll //Roll angle (rad)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float pitch //Pitch angle (rad)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float yaw //Yaw angle (rad)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float rollspeed //Body frame roll / phi angular speed (rad/s)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float pitchspeed //Body frame pitch / theta angular speed (rad/s)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float yawspeed //Body frame yaw / psi angular speed (rad/s)
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }

            public int lat //Latitude, expressed as * 1E7
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  32);}
            }

            public int lon //Longitude, expressed as * 1E7
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  36);}
            }

            public int alt //Altitude in meters, expressed as * 1000 (millimeters)
            {
                set {  BitUtils.set_bytes((uint)(value), 4, data,  40);}
            }

            public short vx //Ground X Speed (Latitude), expressed as m/s * 100
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  44);}
            }

            public short vy //Ground Y Speed (Longitude), expressed as m/s * 100
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  46);}
            }

            public short vz //Ground Z Speed (Altitude), expressed as m/s * 100
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  48);}
            }

            public short xacc //X acceleration (mg)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  50);}
            }

            public short yacc //Y acceleration (mg)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  52);}
            }

            public short zacc //Z acceleration (mg)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  54);}
            }
        }
        public class HIL_CONTROLS : GroundControl.HIL_CONTROLS
        {
            public ulong time_usec //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public float roll_ailerons //Control output -1 .. 1
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float pitch_elevator //Control output -1 .. 1
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float yaw_rudder //Control output -1 .. 1
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float throttle //Throttle 0 .. 1
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float aux1 //Aux 1, -1 .. 1
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float aux2 //Aux 2, -1 .. 1
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }

            public float aux3 //Aux 3, -1 .. 1
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 32);}
            }

            public float aux4 //Aux 4, -1 .. 1
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 36);}
            }

            public byte nav_mode //Navigation mode (MAV_NAV_MODE)
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  40);}
            }

            public MAV_MODE mode //System mode (MAV_MODE)
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
                    BitUtils.set_bits(id, 4, data, 328);
                }
            }
        }
        public class HIL_RC_INPUTS_RAW : GroundControl.HIL_RC_INPUTS_RAW
        {
            public ushort chan1_raw //RC channel 1 value, in microseconds
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort chan2_raw //RC channel 2 value, in microseconds
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public ushort chan3_raw //RC channel 3 value, in microseconds
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  4);}
            }

            public ushort chan4_raw //RC channel 4 value, in microseconds
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  6);}
            }

            public ushort chan5_raw //RC channel 5 value, in microseconds
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  8);}
            }

            public ushort chan6_raw //RC channel 6 value, in microseconds
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  10);}
            }

            public ushort chan7_raw //RC channel 7 value, in microseconds
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  12);}
            }

            public ushort chan8_raw //RC channel 8 value, in microseconds
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  14);}
            }

            public ushort chan9_raw //RC channel 9 value, in microseconds
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  16);}
            }

            public ushort chan10_raw //RC channel 10 value, in microseconds
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  18);}
            }

            public ushort chan11_raw //RC channel 11 value, in microseconds
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  20);}
            }

            public ushort chan12_raw //RC channel 12 value, in microseconds
            {
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  22);}
            }

            public ulong time_usec //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  24);}
            }

            public byte rssi //Receive signal strength indicator, 0: 0%, 255: 100%
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  32);}
            }
        }
        public class HIL_ACTUATOR_CONTROLS : GroundControl.HIL_ACTUATOR_CONTROLS
        {
            public ulong time_usec //Timestamp (microseconds since UNIX epoch or microseconds since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public ulong flags //Flags as bitfield, reserved for future use.
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  8);}
            }

            public float[] controls //Control outputs -1 .. 1. Channel assignment depends on the simulated hardware.
            {
                set {controls_SET(value, 0)  ;}
            }
            public void controls_SET(float[] src, int pos)  //Control outputs -1 .. 1. Channel assignment depends on the simulated hardware.
            {
                for(int BYTE =  16, src_max = pos + 16; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }
            public MAV_MODE mode //System mode (MAV_MODE), includes arming state.
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
                    BitUtils.set_bits(id, 4, data, 640);
                }
            }
        }
        public class OPTICAL_FLOW : GroundControl.OPTICAL_FLOW
        {
            public ulong time_usec //Timestamp (UNIX)
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public byte sensor_id //Sensor ID
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  8);}
            }

            public short flow_x //Flow in pixels * 10 in x-sensor direction (dezi-pixels)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  9);}
            }

            public short flow_y //Flow in pixels * 10 in y-sensor direction (dezi-pixels)
            {
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  11);}
            }

            public float flow_comp_m_x //Flow in meters in x-sensor direction, angular-speed compensated
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 13);}
            }

            public float flow_comp_m_y //Flow in meters in y-sensor direction, angular-speed compensated
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 17);}
            }

            public byte quality //Optical flow quality / confidence. 0: bad, 255: maximum quality
            {
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  21);}
            }

            public float ground_distance //Ground distance in meters. Positive value: distance known. Negative value: Unknown distance
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 22);}
            }
            public void flow_rate_x_SET(float src, Inside ph)//Flow rate in radians/second about X axis
            {
                if(ph.field_bit != 208)insert_field(ph, 208, 0);
                BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src), 4, data, ph.BYTE);
            } public void flow_rate_y_SET(float src, Inside ph) //Flow rate in radians/second about Y axis
            {
                if(ph.field_bit != 209)insert_field(ph, 209, 0);
                BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src), 4, data, ph.BYTE);
            }
        }
        public class GLOBAL_VISION_POSITION_ESTIMATE : GroundControl.GLOBAL_VISION_POSITION_ESTIMATE
        {
            public ulong usec //Timestamp (microseconds, synced to UNIX time or since system boot)
            {
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public float x //Global X position
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float y //Global Y position
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float z //Global Z position
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float roll //Roll angle in rad
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float pitch //Pitch angle in rad
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float yaw //Yaw angle in rad
            {
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }
        }
        new class SENSOR_OFFSETS : GroundControl.SENSOR_OFFSETS
        {
            public short mag_ofs_x //magnetometer X offset
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  0, 2));}
            }

            public short mag_ofs_y //magnetometer Y offset
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  2, 2));}
            }

            public short mag_ofs_z //magnetometer Z offset
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  4, 2));}
            }

            public float mag_declination //magnetic declination (radians)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  6, 4)));}
            }

            public int raw_press //raw pressure from barometer
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  10, 4));}
            }

            public int raw_temp //raw temperature from barometer
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  14, 4));}
            }

            public float gyro_cal_x //gyro X calibration
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  18, 4)));}
            }

            public float gyro_cal_y //gyro Y calibration
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  22, 4)));}
            }

            public float gyro_cal_z //gyro Z calibration
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  26, 4)));}
            }

            public float accel_cal_x //accel X calibration
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  30, 4)));}
            }

            public float accel_cal_y //accel Y calibration
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  34, 4)));}
            }

            public float accel_cal_z //accel Z calibration
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  38, 4)));}
            }
        }
        new class SET_MAG_OFFSETS : GroundControl.SET_MAG_OFFSETS
        {
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public short mag_ofs_x //magnetometer X offset
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  2, 2));}
            }

            public short mag_ofs_y //magnetometer Y offset
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  4, 2));}
            }

            public short mag_ofs_z //magnetometer Z offset
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  6, 2));}
            }
        }
        new class MEMINFO : GroundControl.MEMINFO
        {
            public ushort brkval //heap top
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ushort freemem //free memory
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
            }
            public uint freemem32_TRY(Inside ph)//free memory (32 bit)
            {
                if(ph.field_bit !=  32 && !try_visit_field(ph, 32)) return 0;
                return (uint)((uint) BitUtils.get_bytes(data,  ph.BYTE, 4));
            }
        }
        new class AP_ADC : GroundControl.AP_ADC
        {
            public ushort adc1 //ADC output 1
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ushort adc2 //ADC output 2
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
            }

            public ushort adc3 //ADC output 3
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
            }

            public ushort adc4 //ADC output 4
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  6, 2));}
            }

            public ushort adc5 //ADC output 5
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  8, 2));}
            }

            public ushort adc6 //ADC output 6
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  10, 2));}
            }
        }
        new class DIGICAM_CONFIGURE : GroundControl.DIGICAM_CONFIGURE
        {
            public ushort shutter_speed //Divisor number e.g. 1000 means 1/1000 (0 means ignore)
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

            public byte mode //Mode enumeration from 1 to N P, TV, AV, M, Etc (0 means ignore)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            public byte aperture //F stop number x 10 e.g. 28 means 2.8 (0 means ignore)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
            }

            public byte iso //ISO enumeration from 1 to N e.g. 80, 100, 200, Etc (0 means ignore)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  6, 1));}
            }

            public byte exposure_type //Exposure type enumeration from 1 to N (0 means ignore)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  7, 1));}
            }

            /**
            *Command Identity (incremental loop: 0 to 255)A command sent multiple times will be executed or pooled
            *	just onc*/
            public byte command_id
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  8, 1));}
            }

            public byte engine_cut_off //Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  9, 1));}
            }

            public byte extra_param //Extra parameters enumeration (0 means ignore)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  10, 1));}
            }

            public float extra_value //Correspondent value to given extra_param
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  11, 4)));}
            }
        }
        new class DIGICAM_CONTROL : GroundControl.DIGICAM_CONTROL
        {
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public byte session //0: stop, 1: start or keep it up Session control e.g. show/hide lens
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }

            public byte zoom_pos //1 to N Zoom's absolute position (0 means ignore)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
            }

            public sbyte zoom_step //-100 to 100 Zooming step value to offset zoom from the current position
            {
                get {  return (sbyte)((sbyte) BitUtils.get_bytes(data,  4, 1));}
            }

            public byte focus_lock //0: unlock focus or keep unlocked, 1: lock focus or keep locked, 3: re-lock focus
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
            }

            public byte shot //0: ignore, 1: shot or start filming
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  6, 1));}
            }

            /**
            *Command Identity (incremental loop: 0 to 255)A command sent multiple times will be executed or pooled
            *	just onc*/
            public byte command_id
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  7, 1));}
            }

            public byte extra_param //Extra parameters enumeration (0 means ignore)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  8, 1));}
            }

            public float extra_value //Correspondent value to given extra_param
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  9, 4)));}
            }
        }
        new class MOUNT_CONFIGURE : GroundControl.MOUNT_CONFIGURE
        {
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public byte stab_roll //(1 = yes, 0 = no)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }

            public byte stab_pitch //(1 = yes, 0 = no)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
            }

            public byte stab_yaw //(1 = yes, 0 = no)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            public MAV_MOUNT_MODE mount_mode //mount operating mode (see MAV_MOUNT_MODE enum)
            {
                get {  return (MAV_MOUNT_MODE)(0 +  BitUtils.get_bits(data, 40, 3));}
            }
        }
        new class MOUNT_CONTROL : GroundControl.MOUNT_CONTROL
        {
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public int input_a //pitch(deg*100) or lat, depending on mount mode
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  2, 4));}
            }

            public int input_b //roll(deg*100) or lon depending on mount mode
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  6, 4));}
            }

            public int input_c //yaw(deg*100) or alt (in cm) depending on mount mode
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  10, 4));}
            }

            public byte save_position //if "1" it will save current trimmed position on EEPROM (just valid for NEUTRAL and LANDING)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  14, 1));}
            }
        }
        new class MOUNT_STATUS : GroundControl.MOUNT_STATUS
        {
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public int pointing_a //pitch(deg*100)
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  2, 4));}
            }

            public int pointing_b //roll(deg*100)
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  6, 4));}
            }

            public int pointing_c //yaw(deg*100)
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  10, 4));}
            }
        }
        new class FENCE_POINT : GroundControl.FENCE_POINT
        {
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public byte idx //point index (first point is 1, 0 is for return point)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }

            public byte count //total number of points (for sanity checking)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
            }

            public float lat //Latitude of point
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float lng //Longitude of point
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }
        }
        new class FENCE_FETCH_POINT : GroundControl.FENCE_FETCH_POINT
        {
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public byte idx //point index (first point is 1, 0 is for return point)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }
        }
        new class FENCE_STATUS : GroundControl.FENCE_STATUS
        {
            public ushort breach_count //number of fence breaches
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public uint breach_time //time of last breach in milliseconds since boot
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  2, 4));}
            }

            public byte breach_status //0 if currently inside fence, 1 if outside
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  6, 1));}
            }

            public FENCE_BREACH breach_type //last breach type (see FENCE_BREACH_* enum)
            {
                get {  return (FENCE_BREACH)(0 +  BitUtils.get_bits(data, 56, 3));}
            }
        }
        new class AHRS : GroundControl.AHRS
        {
            public float omegaIx //X gyro drift estimate rad/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  0, 4)));}
            }

            public float omegaIy //Y gyro drift estimate rad/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float omegaIz //Z gyro drift estimate rad/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float accel_weight //average accel_weight
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float renorm_val //average renormalisation value
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }

            public float error_rp //average error_roll_pitch value
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
            }

            public float error_yaw //average error_yaw value
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
            }
        }
        new class SIMSTATE : GroundControl.SIMSTATE
        {
            public float roll //Roll angle (rad)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  0, 4)));}
            }

            public float pitch //Pitch angle (rad)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float yaw //Yaw angle (rad)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float xacc //X acceleration m/s/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float yacc //Y acceleration m/s/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }

            public float zacc //Z acceleration m/s/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
            }

            public float xgyro //Angular speed around X axis rad/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
            }

            public float ygyro //Angular speed around Y axis rad/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
            }

            public float zgyro //Angular speed around Z axis rad/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  32, 4)));}
            }

            public int lat //Latitude in degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  36, 4));}
            }

            public int lng //Longitude in degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  40, 4));}
            }
        }
        new class HWSTATUS : GroundControl.HWSTATUS
        {
            public ushort Vcc //board voltage (mV)
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public byte I2Cerr //I2C error count
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }
        }
        new class RADIO : GroundControl.RADIO
        {
            public ushort rxerrors //receive errors
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ushort fixed_ //count of error corrected packets
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
            }

            public byte rssi //local signal strength
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            public byte remrssi //remote signal strength
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
            }

            public byte txbuf //how full the tx buffer is as a percentage
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  6, 1));}
            }

            public byte noise //background noise level
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  7, 1));}
            }

            public byte remnoise //remote background noise level
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  8, 1));}
            }
        }
        new class LIMITS_STATUS : GroundControl.LIMITS_STATUS
        {
            public ushort breach_count //number of fence breaches
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public uint last_trigger //time of last breach in milliseconds since boot
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  2, 4));}
            }

            public uint last_action //time of last recovery action in milliseconds since boot
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  6, 4));}
            }

            public uint last_recovery //time of last successful recovery in milliseconds since boot
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  10, 4));}
            }

            public uint last_clear //time of last all-clear in milliseconds since boot
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  14, 4));}
            }

            public LIMITS_STATE limits_state //state of AP_Limits, (see enum LimitState, LIMITS_STATE)
            {
                get {  return (LIMITS_STATE)(0 +  BitUtils.get_bits(data, 144, 3));}
            }

            public LIMIT_MODULE mods_enabled //AP_Limit_Module bitfield of enabled modules, (see enum moduleid or LIMIT_MODULE)
            {
                get
                {
                    switch((int)BitUtils.get_bits(data, 147, 2))
                    {
                        case 0:
                            return LIMIT_MODULE.LIMIT_GPSLOCK;
                        case 1:
                            return LIMIT_MODULE.LIMIT_GEOFENCE;
                        case 2:
                            return LIMIT_MODULE.LIMIT_ALTITUDE;
                    }
                    throw  new ArgumentException("Unknown enum ID ");
                }
            }

            public LIMIT_MODULE mods_required //AP_Limit_Module bitfield of required modules, (see enum moduleid or LIMIT_MODULE)
            {
                get
                {
                    switch((int)BitUtils.get_bits(data, 149, 2))
                    {
                        case 0:
                            return LIMIT_MODULE.LIMIT_GPSLOCK;
                        case 1:
                            return LIMIT_MODULE.LIMIT_GEOFENCE;
                        case 2:
                            return LIMIT_MODULE.LIMIT_ALTITUDE;
                    }
                    throw  new ArgumentException("Unknown enum ID ");
                }
            }

            public LIMIT_MODULE mods_triggered //AP_Limit_Module bitfield of triggered modules, (see enum moduleid or LIMIT_MODULE)
            {
                get
                {
                    switch((int)BitUtils.get_bits(data, 151, 2))
                    {
                        case 0:
                            return LIMIT_MODULE.LIMIT_GPSLOCK;
                        case 1:
                            return LIMIT_MODULE.LIMIT_GEOFENCE;
                        case 2:
                            return LIMIT_MODULE.LIMIT_ALTITUDE;
                    }
                    throw  new ArgumentException("Unknown enum ID ");
                }
            }
        }
        new class WIND : GroundControl.WIND
        {
            public float direction //wind direction that wind is coming from (degrees)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  0, 4)));}
            }

            public float speed //wind speed in ground plane (m/s)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float speed_z //vertical wind speed (m/s)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }
        }
        new class DATA16 : GroundControl.DATA16
        {
            public byte type //data type
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte len //data length
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public byte[] data_ //raw data
            {
                get {return data__GET(new byte[16], 0);}
            }
            public byte[]data__GET(byte[] dst_ch, int pos)  //raw data
            {
                for(int BYTE = 2, dst_max = pos + 16; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
        }
        new class DATA32 : GroundControl.DATA32
        {
            public byte type //data type
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte len //data length
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public byte[] data_ //raw data
            {
                get {return data__GET(new byte[32], 0);}
            }
            public byte[]data__GET(byte[] dst_ch, int pos)  //raw data
            {
                for(int BYTE = 2, dst_max = pos + 32; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
        }
        new class DATA64 : GroundControl.DATA64
        {
            public byte type //data type
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte len //data length
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public byte[] data_ //raw data
            {
                get {return data__GET(new byte[64], 0);}
            }
            public byte[]data__GET(byte[] dst_ch, int pos)  //raw data
            {
                for(int BYTE = 2, dst_max = pos + 64; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
        }
        new class DATA96 : GroundControl.DATA96
        {
            public byte type //data type
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte len //data length
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public byte[] data_ //raw data
            {
                get {return data__GET(new byte[96], 0);}
            }
            public byte[]data__GET(byte[] dst_ch, int pos)  //raw data
            {
                for(int BYTE = 2, dst_max = pos + 96; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
        }
        new class RANGEFINDER : GroundControl.RANGEFINDER
        {
            public float distance //distance in meters
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  0, 4)));}
            }

            public float voltage //raw voltage if available, zero otherwise
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }
        }
        new class AIRSPEED_AUTOCAL : GroundControl.AIRSPEED_AUTOCAL
        {
            public float vx //GPS velocity north m/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  0, 4)));}
            }

            public float vy //GPS velocity east m/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float vz //GPS velocity down m/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float diff_pressure //Differential pressure pascals
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float EAS2TAS //Estimated to true airspeed ratio
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }

            public float ratio //Airspeed ratio
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
            }

            public float state_x //EKF state x
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
            }

            public float state_y //EKF state y
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
            }

            public float state_z //EKF state z
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  32, 4)));}
            }

            public float Pax //EKF Pax
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  36, 4)));}
            }

            public float Pby //EKF Pby
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  40, 4)));}
            }

            public float Pcz //EKF Pcz
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  44, 4)));}
            }
        }
        new class RALLY_POINT : GroundControl.RALLY_POINT
        {
            public ushort land_dir //Heading to aim for when landing. In centi-degrees.
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

            public byte idx //point index (first point is 0)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            public byte count //total number of points (for sanity checking)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
            }

            public int lat //Latitude of point in degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  6, 4));}
            }

            public int lng //Longitude of point in degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  10, 4));}
            }

            public short alt //Transit / loiter altitude in meters relative to home
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  14, 2));}
            }

            public short break_alt //Break altitude in meters relative to home
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  16, 2));}
            }

            public RALLY_FLAGS flags //See RALLY_FLAGS enum for definition of the bitmask.
            {
                get {  return (RALLY_FLAGS)(1 +  BitUtils.get_bits(data, 144, 2));}
            }
        }
        new class RALLY_FETCH_POINT : GroundControl.RALLY_FETCH_POINT
        {
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public byte idx //point index (first point is 0)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }
        }
        new class COMPASSMOT_STATUS : GroundControl.COMPASSMOT_STATUS
        {
            public ushort throttle //throttle (percent*10)
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ushort interference //interference (percent)
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
            }

            public float current //current (Ampere)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float CompensationX //Motor Compensation X
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float CompensationY //Motor Compensation Y
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float CompensationZ //Motor Compensation Z
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }
        }
        new class AHRS2 : GroundControl.AHRS2
        {
            public float roll //Roll angle (rad)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  0, 4)));}
            }

            public float pitch //Pitch angle (rad)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float yaw //Yaw angle (rad)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float altitude //Altitude (MSL)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public int lat //Latitude in degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  16, 4));}
            }

            public int lng //Longitude in degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  20, 4));}
            }
        }
        new class CAMERA_STATUS : GroundControl.CAMERA_STATUS
        {
            public ushort img_idx //Image index
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ulong time_usec //Image timestamp (microseconds since UNIX epoch, according to camera clock)
            {
                get {  return (BitUtils.get_bytes(data,  2, 8));}
            }

            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  10, 1));}
            }

            public byte cam_idx //Camera ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  11, 1));}
            }

            public float p1 //Parameter 1 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float p2 //Parameter 2 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }

            public float p3 //Parameter 3 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
            }

            public float p4 //Parameter 4 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
            }

            public CAMERA_STATUS_TYPES event_id //See CAMERA_STATUS_TYPES enum for definition of the bitmask
            {
                get {  return (CAMERA_STATUS_TYPES)(0 +  BitUtils.get_bits(data, 224, 3));}
            }
        }
        new class CAMERA_FEEDBACK : GroundControl.CAMERA_FEEDBACK
        {
            public ushort img_idx //Image index
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            /**
            *Image timestamp (microseconds since UNIX epoch), as passed in by CAMERA_STATUS message (or autopilot if
            *	no CCB*/
            public ulong time_usec
            {
                get {  return (BitUtils.get_bytes(data,  2, 8));}
            }

            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  10, 1));}
            }

            public byte cam_idx //Camera ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  11, 1));}
            }

            public int lat //Latitude in (deg * 1E7)
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  12, 4));}
            }

            public int lng //Longitude in (deg * 1E7)
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  16, 4));}
            }

            public float alt_msl //Altitude Absolute (meters AMSL)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
            }

            public float alt_rel //Altitude Relative (meters above HOME location)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
            }

            public float roll //Camera Roll angle (earth frame, degrees, +-180)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
            }

            public float pitch //Camera Pitch angle (earth frame, degrees, +-180)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  32, 4)));}
            }

            public float yaw //Camera Yaw (earth frame, degrees, 0-360, true)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  36, 4)));}
            }

            public float foc_len //Focal Length (mm)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  40, 4)));}
            }

            public CAMERA_FEEDBACK_FLAGS flags //See CAMERA_FEEDBACK_FLAGS enum for definition of the bitmask
            {
                get {  return (CAMERA_FEEDBACK_FLAGS)(0 +  BitUtils.get_bits(data, 352, 3));}
            }
        }
        new class BATTERY2 : GroundControl.BATTERY2
        {
            public ushort voltage //voltage in millivolts
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public short current_battery //Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the curren
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  2, 2));}
            }
        }
        new class AHRS3 : GroundControl.AHRS3
        {
            public float roll //Roll angle (rad)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  0, 4)));}
            }

            public float pitch //Pitch angle (rad)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float yaw //Yaw angle (rad)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float altitude //Altitude (MSL)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public int lat //Latitude in degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  16, 4));}
            }

            public int lng //Longitude in degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  20, 4));}
            }

            public float v1 //test variable1
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
            }

            public float v2 //test variable2
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
            }

            public float v3 //test variable3
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  32, 4)));}
            }

            public float v4 //test variable4
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  36, 4)));}
            }
        }
        new class AUTOPILOT_VERSION_REQUEST : GroundControl.AUTOPILOT_VERSION_REQUEST
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
        new class REMOTE_LOG_DATA_BLOCK : GroundControl.REMOTE_LOG_DATA_BLOCK
        {
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public byte[] data_ //log data block
            {
                get {return data__GET(new byte[200], 0);}
            }
            public byte[]data__GET(byte[] dst_ch, int pos)  //log data block
            {
                for(int BYTE = 2, dst_max = pos + 200; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            public MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS seqno //log data block sequence number
            {
                get {  return (MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS)(2147483645 +  BitUtils.get_bits(data, 1616, 2));}
            }
        }
        new class REMOTE_LOG_BLOCK_STATUS : GroundControl.REMOTE_LOG_BLOCK_STATUS
        {
            public uint seqno //log data block sequence number
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
            }

            public MAV_REMOTE_LOG_DATA_BLOCK_STATUSES status //log data block status
            {
                get {  return (MAV_REMOTE_LOG_DATA_BLOCK_STATUSES)(0 +  BitUtils.get_bits(data, 48, 2));}
            }
        }
        new class LED_CONTROL : GroundControl.LED_CONTROL
        {
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public byte instance //Instance (LED instance to control or 255 for all LEDs)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }

            public byte pattern //Pattern (see LED_PATTERN_ENUM)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
            }

            public byte custom_len //Custom Byte Length
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            public byte[] custom_bytes //Custom Bytes
            {
                get {return custom_bytes_GET(new byte[24], 0);}
            }
            public byte[]custom_bytes_GET(byte[] dst_ch, int pos)  //Custom Bytes
            {
                for(int BYTE = 5, dst_max = pos + 24; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
        }
        new class MAG_CAL_PROGRESS : GroundControl.MAG_CAL_PROGRESS
        {
            public byte compass_id //Compass being calibrated
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte cal_mask //Bitmask of compasses being calibrated
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public byte attempt //Attempt number
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }

            public byte completion_pct //Completion percentage
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
            }

            public byte[] completion_mask //Bitmask of sphere sections (see http:en.wikipedia.org/wiki/Geodesic_grid)
            {
                get {return completion_mask_GET(new byte[10], 0);}
            }
            public byte[]completion_mask_GET(byte[] dst_ch, int pos)  //Bitmask of sphere sections (see http:en.wikipedia.org/wiki/Geodesic_grid)
            {
                for(int BYTE = 4, dst_max = pos + 10; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            public float direction_x //Body frame direction vector for display
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  14, 4)));}
            }

            public float direction_y //Body frame direction vector for display
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  18, 4)));}
            }

            public float direction_z //Body frame direction vector for display
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  22, 4)));}
            }

            public MAG_CAL_STATUS cal_status //Status (see MAG_CAL_STATUS enum)
            {
                get {  return (MAG_CAL_STATUS)(0 +  BitUtils.get_bits(data, 208, 3));}
            }
        }
        new class MAG_CAL_REPORT : GroundControl.MAG_CAL_REPORT
        {
            public byte compass_id //Compass being calibrated
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte cal_mask //Bitmask of compasses being calibrated
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public byte autosaved //0=requires a MAV_CMD_DO_ACCEPT_MAG_CAL, 1=saved to parameters
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }

            public float fitness //RMS milligauss residuals
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  3, 4)));}
            }

            public float ofs_x //X offset
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  7, 4)));}
            }

            public float ofs_y //Y offset
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  11, 4)));}
            }

            public float ofs_z //Z offset
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  15, 4)));}
            }

            public float diag_x //X diagonal (matrix 11)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  19, 4)));}
            }

            public float diag_y //Y diagonal (matrix 22)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  23, 4)));}
            }

            public float diag_z //Z diagonal (matrix 33)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  27, 4)));}
            }

            public float offdiag_x //X off-diagonal (matrix 12 and 21)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  31, 4)));}
            }

            public float offdiag_y //Y off-diagonal (matrix 13 and 31)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  35, 4)));}
            }

            public float offdiag_z //Z off-diagonal (matrix 32 and 23)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  39, 4)));}
            }

            public MAG_CAL_STATUS cal_status //Status (see MAG_CAL_STATUS enum)
            {
                get {  return (MAG_CAL_STATUS)(0 +  BitUtils.get_bits(data, 344, 3));}
            }
        }
        new class EKF_STATUS_REPORT : GroundControl.EKF_STATUS_REPORT
        {
            public float velocity_variance //Velocity variance
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  0, 4)));}
            }

            public float pos_horiz_variance //Horizontal Position variance
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float pos_vert_variance //Vertical Position variance
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float compass_variance //Compass variance
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float terrain_alt_variance //Terrain Altitude variance
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }

            public EKF_STATUS_FLAGS flags //Flags
            {
                get
                {
                    switch((int)BitUtils.get_bits(data, 160, 4))
                    {
                        case 0:
                            return EKF_STATUS_FLAGS.EKF_ATTITUDE;
                        case 1:
                            return EKF_STATUS_FLAGS.EKF_VELOCITY_HORIZ;
                        case 2:
                            return EKF_STATUS_FLAGS.EKF_VELOCITY_VERT;
                        case 3:
                            return EKF_STATUS_FLAGS.EKF_POS_HORIZ_REL;
                        case 4:
                            return EKF_STATUS_FLAGS.EKF_POS_HORIZ_ABS;
                        case 5:
                            return EKF_STATUS_FLAGS.EKF_POS_VERT_ABS;
                        case 6:
                            return EKF_STATUS_FLAGS.EKF_POS_VERT_AGL;
                        case 7:
                            return EKF_STATUS_FLAGS.EKF_CONST_POS_MODE;
                        case 8:
                            return EKF_STATUS_FLAGS.EKF_PRED_POS_HORIZ_REL;
                        case 9:
                            return EKF_STATUS_FLAGS.EKF_PRED_POS_HORIZ_ABS;
                    }
                    throw  new ArgumentException("Unknown enum ID ");
                }
            }
        }
        new class PID_TUNING : GroundControl.PID_TUNING
        {
            public float desired //desired rate (degrees/s)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  0, 4)));}
            }

            public float achieved //achieved rate (degrees/s)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float FF //FF component
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float P //P component
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float I //I component
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }

            public float D //D component
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
            }

            public PID_TUNING_AXIS axis //axis
            {
                get {  return (PID_TUNING_AXIS)(1 +  BitUtils.get_bits(data, 192, 3));}
            }
        }
        new class GIMBAL_REPORT : GroundControl.GIMBAL_REPORT
        {
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public float delta_time //Time since last update (seconds)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  2, 4)));}
            }

            public float delta_angle_x //Delta angle X (radians)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  6, 4)));}
            }

            public float delta_angle_y //Delta angle Y (radians)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  10, 4)));}
            }

            public float delta_angle_z //Delta angle X (radians)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  14, 4)));}
            }

            public float delta_velocity_x //Delta velocity X (m/s)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  18, 4)));}
            }

            public float delta_velocity_y //Delta velocity Y (m/s)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  22, 4)));}
            }

            public float delta_velocity_z //Delta velocity Z (m/s)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  26, 4)));}
            }

            public float joint_roll //Joint ROLL (radians)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  30, 4)));}
            }

            public float joint_el //Joint EL (radians)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  34, 4)));}
            }

            public float joint_az //Joint AZ (radians)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  38, 4)));}
            }
        }
        new class GIMBAL_CONTROL : GroundControl.GIMBAL_CONTROL
        {
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public float demanded_rate_x //Demanded angular rate X (rad/s)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  2, 4)));}
            }

            public float demanded_rate_y //Demanded angular rate Y (rad/s)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  6, 4)));}
            }

            public float demanded_rate_z //Demanded angular rate Z (rad/s)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  10, 4)));}
            }
        }
        new class GIMBAL_TORQUE_CMD_REPORT : GroundControl.GIMBAL_TORQUE_CMD_REPORT
        {
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public short rl_torque_cmd //Roll Torque Command
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  2, 2));}
            }

            public short el_torque_cmd //Elevation Torque Command
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  4, 2));}
            }

            public short az_torque_cmd //Azimuth Torque Command
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  6, 2));}
            }
        }
        new class GOPRO_HEARTBEAT : GroundControl.GOPRO_HEARTBEAT
        {
            public GOPRO_HEARTBEAT_STATUS status //Status
            {
                get {  return (GOPRO_HEARTBEAT_STATUS)(0 +  BitUtils.get_bits(data, 0, 3));}
            }

            public GOPRO_CAPTURE_MODE capture_mode //Current capture mode
            {
                get
                {
                    switch((int)BitUtils.get_bits(data, 3, 4))
                    {
                        case 0:
                            return GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_VIDEO;
                        case 1:
                            return GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_PHOTO;
                        case 2:
                            return GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_BURST;
                        case 3:
                            return GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_TIME_LAPSE;
                        case 4:
                            return GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_MULTI_SHOT;
                        case 5:
                            return GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_PLAYBACK;
                        case 6:
                            return GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_SETUP;
                        case 7:
                            return GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_UNKNOWN;
                    }
                    throw  new ArgumentException("Unknown enum ID ");
                }
            }

            public GOPRO_HEARTBEAT_FLAGS flags //additional status bits
            {
                get {  return (GOPRO_HEARTBEAT_FLAGS)(1 +  BitUtils.get_bits(data, 7, 1));}
            }
        }
        new class GOPRO_GET_REQUEST : GroundControl.GOPRO_GET_REQUEST
        {
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public GOPRO_COMMAND cmd_id //Command ID
            {
                get {  return (GOPRO_COMMAND)(0 +  BitUtils.get_bits(data, 16, 5));}
            }
        }
        new class GOPRO_GET_RESPONSE : GroundControl.GOPRO_GET_RESPONSE
        {
            public byte[] value //Value
            {
                get {return value_GET(new byte[4], 0);}
            }
            public byte[]value_GET(byte[] dst_ch, int pos)  //Value
            {
                for(int BYTE = 0, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            public GOPRO_COMMAND cmd_id //Command ID
            {
                get {  return (GOPRO_COMMAND)(0 +  BitUtils.get_bits(data, 32, 5));}
            }

            public GOPRO_REQUEST_STATUS status //Status
            {
                get {  return (GOPRO_REQUEST_STATUS)(0 +  BitUtils.get_bits(data, 37, 2));}
            }
        }
        new class GOPRO_SET_REQUEST : GroundControl.GOPRO_SET_REQUEST
        {
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public byte[] value //Value
            {
                get {return value_GET(new byte[4], 0);}
            }
            public byte[]value_GET(byte[] dst_ch, int pos)  //Value
            {
                for(int BYTE = 2, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            public GOPRO_COMMAND cmd_id //Command ID
            {
                get {  return (GOPRO_COMMAND)(0 +  BitUtils.get_bits(data, 48, 5));}
            }
        }
        new class GOPRO_SET_RESPONSE : GroundControl.GOPRO_SET_RESPONSE
        {
            public GOPRO_COMMAND cmd_id //Command ID
            {
                get {  return (GOPRO_COMMAND)(0 +  BitUtils.get_bits(data, 0, 5));}
            }

            public GOPRO_REQUEST_STATUS status //Status
            {
                get {  return (GOPRO_REQUEST_STATUS)(0 +  BitUtils.get_bits(data, 5, 2));}
            }
        }
        new class RPM : GroundControl.RPM
        {
            public float rpm1 //RPM Sensor1
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  0, 4)));}
            }

            public float rpm2 //RPM Sensor2
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
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
        new class UAVIONIX_ADSB_OUT_CFG : GroundControl.UAVIONIX_ADSB_OUT_CFG
        {
            public ushort stallSpeed //Aircraft stall speed in cm/s
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public uint ICAO //Vehicle address (24 bit)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  2, 4));}
            }

            public ADSB_EMITTER_TYPE emitterType //Transmitting vehicle type. See ADSB_EMITTER_TYPE enum
            {
                get {  return (ADSB_EMITTER_TYPE)(0 +  BitUtils.get_bits(data, 48, 5));}
            }

            public UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE aircraftSize //Aircraft length and width encoding (table 2-35 of DO-282B)
            {
                get {  return (UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE)(0 +  BitUtils.get_bits(data, 53, 5));}
            }

            public UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT gpsOffsetLat //GPS antenna lateral offset (table 2-36 of DO-282B)
            {
                get {  return (UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT)(0 +  BitUtils.get_bits(data, 58, 4));}
            }

            /**
            *GPS antenna longitudinal offset from nose [if non-zero, take position (in meters) divide by 2 and add
            *	one] (table 2-37 DO-282B*/
            public UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON gpsOffsetLon
            {
                get {  return (UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON)(0 +  BitUtils.get_bits(data, 62, 2));}
            }

            public UAVIONIX_ADSB_OUT_RF_SELECT rfSelect //ADS-B transponder reciever and transmit enable flags
            {
                get {  return (UAVIONIX_ADSB_OUT_RF_SELECT)(0 +  BitUtils.get_bits(data, 64, 2));}
            }
            public string callsign_TRY(Inside ph)//Vehicle identifier (8 characters, null terminated, valid characters are A-Z, 0-9, " " only)
            {
                if(ph.field_bit !=  66 && !try_visit_field(ph, 66)  ||  !try_visit_item(ph, 0)) return null;
                return new string(callsign_GET(ph, new char[ph.items], 0));
            }
            public char[]callsign_GET(Inside ph, char[] dst_ch, int pos) //Vehicle identifier (8 characters, null terminated, valid characters are A-Z, 0-9, " " only)
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int callsign_LEN(Inside ph)
            {
                return (ph.field_bit !=  66 && !try_visit_field(ph, 66)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            }
        }
        new class UAVIONIX_ADSB_OUT_DYNAMIC : GroundControl.UAVIONIX_ADSB_OUT_DYNAMIC
        {
            public ushort accuracyVert //Vertical accuracy in cm. If unknown set to UINT16_MAX
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ushort accuracyVel //Velocity accuracy in mm/s (m * 1E-3). If unknown set to UINT16_MAX
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
            }

            public ushort squawk //Mode A code (typically 1200 [0x04B0] for VFR)
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
            }

            public uint utcTime //UTC time in seconds since GPS epoch (Jan 6, 1980). If unknown set to UINT32_MAX
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  6, 4));}
            }

            public uint accuracyHor //Horizontal accuracy in mm (m * 1E-3). If unknown set to UINT32_MAX
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  10, 4));}
            }

            public int gpsLat //Latitude WGS84 (deg * 1E7). If unknown set to INT32_MAX
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  14, 4));}
            }

            public int gpsLon //Longitude WGS84 (deg * 1E7). If unknown set to INT32_MAX
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  18, 4));}
            }

            public int gpsAlt //Altitude in mm (m * 1E-3) UP +ve. WGS84 altitude. If unknown set to INT32_MAX
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  22, 4));}
            }

            public byte numSats //Number of satellites visible. If unknown set to UINT8_MAX
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  26, 1));}
            }

            /**
            *Barometric pressure altitude relative to a standard atmosphere of 1013.2 mBar and NOT bar corrected altitude
            *	(m * 1E-3). (up +ve). If unknown set to INT32_MA*/
            public int baroAltMSL
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  27, 4));}
            }

            public short velVert //GPS vertical speed in cm/s. If unknown set to INT16_MAX
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  31, 2));}
            }

            public short velNS //North-South velocity over ground in cm/s North +ve. If unknown set to INT16_MAX
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  33, 2));}
            }

            public short VelEW //East-West velocity over ground in cm/s East +ve. If unknown set to INT16_MAX
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  35, 2));}
            }

            public UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX gpsFix //0-1: no fix, 2: 2D fix, 3: 3D fix, 4: DGPS, 5: RTK
            {
                get {  return (UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX)(0 +  BitUtils.get_bits(data, 296, 3));}
            }

            public UAVIONIX_ADSB_EMERGENCY_STATUS emergencyStatus //Emergency status
            {
                get {  return (UAVIONIX_ADSB_EMERGENCY_STATUS)(0 +  BitUtils.get_bits(data, 299, 4));}
            }

            public UAVIONIX_ADSB_OUT_DYNAMIC_STATE state //ADS-B transponder dynamic input state flags
            {
                get
                {
                    switch((int)BitUtils.get_bits(data, 303, 3))
                    {
                        case 0:
                            return UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_INTENT_CHANGE;
                        case 1:
                            return UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_AUTOPILOT_ENABLED;
                        case 2:
                            return UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_NICBARO_CROSSCHECKED;
                        case 3:
                            return UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_ON_GROUND;
                        case 4:
                            return UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_IDENT;
                    }
                    throw  new ArgumentException("Unknown enum ID ");
                }
            }
        }
        new class UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT : GroundControl.UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT
        {
            public UAVIONIX_ADSB_RF_HEALTH rfHealth //ADS-B transponder messages
            {
                get
                {
                    switch((int)BitUtils.get_bits(data, 0, 3))
                    {
                        case 0:
                            return UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_INITIALIZING;
                        case 1:
                            return UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_OK;
                        case 2:
                            return UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_FAIL_TX;
                        case 3:
                            return UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_FAIL_RX;
                    }
                    throw  new ArgumentException("Unknown enum ID ");
                }
            }
        }
        new class DEVICE_OP_READ : GroundControl.DEVICE_OP_READ
        {
            public uint request_id //request ID - copied to reply
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
            }

            public byte bus //Bus number
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  6, 1));}
            }

            public byte address //Bus address
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  7, 1));}
            }

            public byte regstart //First register to read
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  8, 1));}
            }

            public byte count //count of registers to read
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  9, 1));}
            }

            public DEVICE_OP_BUSTYPE bustype //The bus type
            {
                get {  return (DEVICE_OP_BUSTYPE)(0 +  BitUtils.get_bits(data, 80, 2));}
            }
            public string busname_TRY(Inside ph)//Name of device on bus (for SPI)
            {
                if(ph.field_bit !=  82 && !try_visit_field(ph, 82)  ||  !try_visit_item(ph, 0)) return null;
                return new string(busname_GET(ph, new char[ph.items], 0));
            }
            public char[]busname_GET(Inside ph, char[] dst_ch, int pos) //Name of device on bus (for SPI)
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int busname_LEN(Inside ph)
            {
                return (ph.field_bit !=  82 && !try_visit_field(ph, 82)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            }
        }
        new class DEVICE_OP_READ_REPLY : GroundControl.DEVICE_OP_READ_REPLY
        {
            public uint request_id //request ID - copied from request
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public byte result //0 for success, anything else is failure code
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            public byte regstart //starting register
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
            }

            public byte count //count of bytes read
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  6, 1));}
            }

            public byte[] data_ //reply data
            {
                get {return data__GET(new byte[128], 0);}
            }
            public byte[]data__GET(byte[] dst_ch, int pos)  //reply data
            {
                for(int BYTE = 7, dst_max = pos + 128; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
        }
        new class DEVICE_OP_WRITE : GroundControl.DEVICE_OP_WRITE
        {
            public uint request_id //request ID - copied to reply
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
            }

            public byte bus //Bus number
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  6, 1));}
            }

            public byte address //Bus address
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  7, 1));}
            }

            public byte regstart //First register to write
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  8, 1));}
            }

            public byte count //count of registers to write
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  9, 1));}
            }

            public byte[] data_ //write data
            {
                get {return data__GET(new byte[128], 0);}
            }
            public byte[]data__GET(byte[] dst_ch, int pos)  //write data
            {
                for(int BYTE = 10, dst_max = pos + 128; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            public DEVICE_OP_BUSTYPE bustype //The bus type
            {
                get {  return (DEVICE_OP_BUSTYPE)(0 +  BitUtils.get_bits(data, 1104, 2));}
            }
            public string busname_TRY(Inside ph)//Name of device on bus (for SPI)
            {
                if(ph.field_bit !=  1106 && !try_visit_field(ph, 1106)  ||  !try_visit_item(ph, 0)) return null;
                return new string(busname_GET(ph, new char[ph.items], 0));
            }
            public char[]busname_GET(Inside ph, char[] dst_ch, int pos) //Name of device on bus (for SPI)
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int busname_LEN(Inside ph)
            {
                return (ph.field_bit !=  1106 && !try_visit_field(ph, 1106)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            }
        }
        new class DEVICE_OP_WRITE_REPLY : GroundControl.DEVICE_OP_WRITE_REPLY
        {
            public uint request_id //request ID - copied from request
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public byte result //0 for success, anything else is failure code
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }
        }
        new class ADAP_TUNING : GroundControl.ADAP_TUNING
        {
            public float desired //desired rate (degrees/s)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  0, 4)));}
            }

            public float achieved //achieved rate (degrees/s)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float error //error between model and vehicle
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float theta //theta estimated state predictor
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float omega //omega estimated state predictor
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }

            public float sigma //sigma estimated state predictor
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
            }

            public float theta_dot //theta derivative
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
            }

            public float omega_dot //omega derivative
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
            }

            public float sigma_dot //sigma derivative
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  32, 4)));}
            }

            public float f //projection operator value
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  36, 4)));}
            }

            public float f_dot //projection operator derivative
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  40, 4)));}
            }

            public float u //u adaptive controlled output command
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  44, 4)));}
            }

            public PID_TUNING_AXIS axis //axis
            {
                get {  return (PID_TUNING_AXIS)(1 +  BitUtils.get_bits(data, 384, 3));}
            }
        }
        new class VISION_POSITION_DELTA : GroundControl.VISION_POSITION_DELTA
        {
            public ulong time_usec //Timestamp (microseconds, synced to UNIX time or since system boot)
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
            }

            public ulong time_delta_usec //Time in microseconds since the last reported camera frame
            {
                get {  return (BitUtils.get_bytes(data,  8, 8));}
            }

            public float[] angle_delta //Defines a rotation vector in body frame that rotates the vehicle from the previous to the current orientatio
            {
                get {return angle_delta_GET(new float[3], 0);}
            }
            public float[]angle_delta_GET(float[] dst_ch, int pos)  //Defines a rotation vector in body frame that rotates the vehicle from the previous to the current orientatio
            {
                for(int BYTE = 16, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
            /**
            *Change in position in meters from previous to current frame rotated into body frame (0=forward, 1=right,
            *	2=down*/
            public float[] position_delta
            {
                get {return position_delta_GET(new float[3], 0);}
            }
            /**
            *Change in position in meters from previous to current frame rotated into body frame (0=forward, 1=right,
            *	2=down*/
            public float[]position_delta_GET(float[] dst_ch, int pos)
            {
                for(int BYTE = 28, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
            public float confidence //normalised confidence value from 0 to 100
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  40, 4)));}
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

            public void OnSENSOR_OFFSETSReceive_direct(Channel src, Inside ph, SENSOR_OFFSETS pack) {OnSENSOR_OFFSETSReceive(this, ph,  pack);}
            public event SENSOR_OFFSETSReceiveHandler OnSENSOR_OFFSETSReceive;
            public delegate void SENSOR_OFFSETSReceiveHandler(Channel src, Inside ph, SENSOR_OFFSETS pack);
            public void OnSET_MAG_OFFSETSReceive_direct(Channel src, Inside ph, SET_MAG_OFFSETS pack) {OnSET_MAG_OFFSETSReceive(this, ph,  pack);}
            public event SET_MAG_OFFSETSReceiveHandler OnSET_MAG_OFFSETSReceive;
            public delegate void SET_MAG_OFFSETSReceiveHandler(Channel src, Inside ph, SET_MAG_OFFSETS pack);
            public void OnMEMINFOReceive_direct(Channel src, Inside ph, MEMINFO pack) {OnMEMINFOReceive(this, ph,  pack);}
            public event MEMINFOReceiveHandler OnMEMINFOReceive;
            public delegate void MEMINFOReceiveHandler(Channel src, Inside ph, MEMINFO pack);
            public void OnAP_ADCReceive_direct(Channel src, Inside ph, AP_ADC pack) {OnAP_ADCReceive(this, ph,  pack);}
            public event AP_ADCReceiveHandler OnAP_ADCReceive;
            public delegate void AP_ADCReceiveHandler(Channel src, Inside ph, AP_ADC pack);
            public void OnDIGICAM_CONFIGUREReceive_direct(Channel src, Inside ph, DIGICAM_CONFIGURE pack) {OnDIGICAM_CONFIGUREReceive(this, ph,  pack);}
            public event DIGICAM_CONFIGUREReceiveHandler OnDIGICAM_CONFIGUREReceive;
            public delegate void DIGICAM_CONFIGUREReceiveHandler(Channel src, Inside ph, DIGICAM_CONFIGURE pack);
            public void OnDIGICAM_CONTROLReceive_direct(Channel src, Inside ph, DIGICAM_CONTROL pack) {OnDIGICAM_CONTROLReceive(this, ph,  pack);}
            public event DIGICAM_CONTROLReceiveHandler OnDIGICAM_CONTROLReceive;
            public delegate void DIGICAM_CONTROLReceiveHandler(Channel src, Inside ph, DIGICAM_CONTROL pack);
            public void OnMOUNT_CONFIGUREReceive_direct(Channel src, Inside ph, MOUNT_CONFIGURE pack) {OnMOUNT_CONFIGUREReceive(this, ph,  pack);}
            public event MOUNT_CONFIGUREReceiveHandler OnMOUNT_CONFIGUREReceive;
            public delegate void MOUNT_CONFIGUREReceiveHandler(Channel src, Inside ph, MOUNT_CONFIGURE pack);
            public void OnMOUNT_CONTROLReceive_direct(Channel src, Inside ph, MOUNT_CONTROL pack) {OnMOUNT_CONTROLReceive(this, ph,  pack);}
            public event MOUNT_CONTROLReceiveHandler OnMOUNT_CONTROLReceive;
            public delegate void MOUNT_CONTROLReceiveHandler(Channel src, Inside ph, MOUNT_CONTROL pack);
            public void OnMOUNT_STATUSReceive_direct(Channel src, Inside ph, MOUNT_STATUS pack) {OnMOUNT_STATUSReceive(this, ph,  pack);}
            public event MOUNT_STATUSReceiveHandler OnMOUNT_STATUSReceive;
            public delegate void MOUNT_STATUSReceiveHandler(Channel src, Inside ph, MOUNT_STATUS pack);
            public void OnFENCE_POINTReceive_direct(Channel src, Inside ph, FENCE_POINT pack) {OnFENCE_POINTReceive(this, ph,  pack);}
            public event FENCE_POINTReceiveHandler OnFENCE_POINTReceive;
            public delegate void FENCE_POINTReceiveHandler(Channel src, Inside ph, FENCE_POINT pack);
            public void OnFENCE_FETCH_POINTReceive_direct(Channel src, Inside ph, FENCE_FETCH_POINT pack) {OnFENCE_FETCH_POINTReceive(this, ph,  pack);}
            public event FENCE_FETCH_POINTReceiveHandler OnFENCE_FETCH_POINTReceive;
            public delegate void FENCE_FETCH_POINTReceiveHandler(Channel src, Inside ph, FENCE_FETCH_POINT pack);
            public void OnFENCE_STATUSReceive_direct(Channel src, Inside ph, FENCE_STATUS pack) {OnFENCE_STATUSReceive(this, ph,  pack);}
            public event FENCE_STATUSReceiveHandler OnFENCE_STATUSReceive;
            public delegate void FENCE_STATUSReceiveHandler(Channel src, Inside ph, FENCE_STATUS pack);
            public void OnAHRSReceive_direct(Channel src, Inside ph, AHRS pack) {OnAHRSReceive(this, ph,  pack);}
            public event AHRSReceiveHandler OnAHRSReceive;
            public delegate void AHRSReceiveHandler(Channel src, Inside ph, AHRS pack);
            public void OnSIMSTATEReceive_direct(Channel src, Inside ph, SIMSTATE pack) {OnSIMSTATEReceive(this, ph,  pack);}
            public event SIMSTATEReceiveHandler OnSIMSTATEReceive;
            public delegate void SIMSTATEReceiveHandler(Channel src, Inside ph, SIMSTATE pack);
            public void OnHWSTATUSReceive_direct(Channel src, Inside ph, HWSTATUS pack) {OnHWSTATUSReceive(this, ph,  pack);}
            public event HWSTATUSReceiveHandler OnHWSTATUSReceive;
            public delegate void HWSTATUSReceiveHandler(Channel src, Inside ph, HWSTATUS pack);
            public void OnRADIOReceive_direct(Channel src, Inside ph, RADIO pack) {OnRADIOReceive(this, ph,  pack);}
            public event RADIOReceiveHandler OnRADIOReceive;
            public delegate void RADIOReceiveHandler(Channel src, Inside ph, RADIO pack);
            public void OnLIMITS_STATUSReceive_direct(Channel src, Inside ph, LIMITS_STATUS pack) {OnLIMITS_STATUSReceive(this, ph,  pack);}
            public event LIMITS_STATUSReceiveHandler OnLIMITS_STATUSReceive;
            public delegate void LIMITS_STATUSReceiveHandler(Channel src, Inside ph, LIMITS_STATUS pack);
            public void OnWINDReceive_direct(Channel src, Inside ph, WIND pack) {OnWINDReceive(this, ph,  pack);}
            public event WINDReceiveHandler OnWINDReceive;
            public delegate void WINDReceiveHandler(Channel src, Inside ph, WIND pack);
            public void OnDATA16Receive_direct(Channel src, Inside ph, DATA16 pack) {OnDATA16Receive(this, ph,  pack);}
            public event DATA16ReceiveHandler OnDATA16Receive;
            public delegate void DATA16ReceiveHandler(Channel src, Inside ph, DATA16 pack);
            public void OnDATA32Receive_direct(Channel src, Inside ph, DATA32 pack) {OnDATA32Receive(this, ph,  pack);}
            public event DATA32ReceiveHandler OnDATA32Receive;
            public delegate void DATA32ReceiveHandler(Channel src, Inside ph, DATA32 pack);
            public void OnDATA64Receive_direct(Channel src, Inside ph, DATA64 pack) {OnDATA64Receive(this, ph,  pack);}
            public event DATA64ReceiveHandler OnDATA64Receive;
            public delegate void DATA64ReceiveHandler(Channel src, Inside ph, DATA64 pack);
            public void OnDATA96Receive_direct(Channel src, Inside ph, DATA96 pack) {OnDATA96Receive(this, ph,  pack);}
            public event DATA96ReceiveHandler OnDATA96Receive;
            public delegate void DATA96ReceiveHandler(Channel src, Inside ph, DATA96 pack);
            public void OnRANGEFINDERReceive_direct(Channel src, Inside ph, RANGEFINDER pack) {OnRANGEFINDERReceive(this, ph,  pack);}
            public event RANGEFINDERReceiveHandler OnRANGEFINDERReceive;
            public delegate void RANGEFINDERReceiveHandler(Channel src, Inside ph, RANGEFINDER pack);
            public void OnAIRSPEED_AUTOCALReceive_direct(Channel src, Inside ph, AIRSPEED_AUTOCAL pack) {OnAIRSPEED_AUTOCALReceive(this, ph,  pack);}
            public event AIRSPEED_AUTOCALReceiveHandler OnAIRSPEED_AUTOCALReceive;
            public delegate void AIRSPEED_AUTOCALReceiveHandler(Channel src, Inside ph, AIRSPEED_AUTOCAL pack);
            public void OnRALLY_POINTReceive_direct(Channel src, Inside ph, RALLY_POINT pack) {OnRALLY_POINTReceive(this, ph,  pack);}
            public event RALLY_POINTReceiveHandler OnRALLY_POINTReceive;
            public delegate void RALLY_POINTReceiveHandler(Channel src, Inside ph, RALLY_POINT pack);
            public void OnRALLY_FETCH_POINTReceive_direct(Channel src, Inside ph, RALLY_FETCH_POINT pack) {OnRALLY_FETCH_POINTReceive(this, ph,  pack);}
            public event RALLY_FETCH_POINTReceiveHandler OnRALLY_FETCH_POINTReceive;
            public delegate void RALLY_FETCH_POINTReceiveHandler(Channel src, Inside ph, RALLY_FETCH_POINT pack);
            public void OnCOMPASSMOT_STATUSReceive_direct(Channel src, Inside ph, COMPASSMOT_STATUS pack) {OnCOMPASSMOT_STATUSReceive(this, ph,  pack);}
            public event COMPASSMOT_STATUSReceiveHandler OnCOMPASSMOT_STATUSReceive;
            public delegate void COMPASSMOT_STATUSReceiveHandler(Channel src, Inside ph, COMPASSMOT_STATUS pack);
            public void OnAHRS2Receive_direct(Channel src, Inside ph, AHRS2 pack) {OnAHRS2Receive(this, ph,  pack);}
            public event AHRS2ReceiveHandler OnAHRS2Receive;
            public delegate void AHRS2ReceiveHandler(Channel src, Inside ph, AHRS2 pack);
            public void OnCAMERA_STATUSReceive_direct(Channel src, Inside ph, CAMERA_STATUS pack) {OnCAMERA_STATUSReceive(this, ph,  pack);}
            public event CAMERA_STATUSReceiveHandler OnCAMERA_STATUSReceive;
            public delegate void CAMERA_STATUSReceiveHandler(Channel src, Inside ph, CAMERA_STATUS pack);
            public void OnCAMERA_FEEDBACKReceive_direct(Channel src, Inside ph, CAMERA_FEEDBACK pack) {OnCAMERA_FEEDBACKReceive(this, ph,  pack);}
            public event CAMERA_FEEDBACKReceiveHandler OnCAMERA_FEEDBACKReceive;
            public delegate void CAMERA_FEEDBACKReceiveHandler(Channel src, Inside ph, CAMERA_FEEDBACK pack);
            public void OnBATTERY2Receive_direct(Channel src, Inside ph, BATTERY2 pack) {OnBATTERY2Receive(this, ph,  pack);}
            public event BATTERY2ReceiveHandler OnBATTERY2Receive;
            public delegate void BATTERY2ReceiveHandler(Channel src, Inside ph, BATTERY2 pack);
            public void OnAHRS3Receive_direct(Channel src, Inside ph, AHRS3 pack) {OnAHRS3Receive(this, ph,  pack);}
            public event AHRS3ReceiveHandler OnAHRS3Receive;
            public delegate void AHRS3ReceiveHandler(Channel src, Inside ph, AHRS3 pack);
            public void OnAUTOPILOT_VERSION_REQUESTReceive_direct(Channel src, Inside ph, AUTOPILOT_VERSION_REQUEST pack) {OnAUTOPILOT_VERSION_REQUESTReceive(this, ph,  pack);}
            public event AUTOPILOT_VERSION_REQUESTReceiveHandler OnAUTOPILOT_VERSION_REQUESTReceive;
            public delegate void AUTOPILOT_VERSION_REQUESTReceiveHandler(Channel src, Inside ph, AUTOPILOT_VERSION_REQUEST pack);
            public void OnREMOTE_LOG_DATA_BLOCKReceive_direct(Channel src, Inside ph, REMOTE_LOG_DATA_BLOCK pack) {OnREMOTE_LOG_DATA_BLOCKReceive(this, ph,  pack);}
            public event REMOTE_LOG_DATA_BLOCKReceiveHandler OnREMOTE_LOG_DATA_BLOCKReceive;
            public delegate void REMOTE_LOG_DATA_BLOCKReceiveHandler(Channel src, Inside ph, REMOTE_LOG_DATA_BLOCK pack);
            public void OnREMOTE_LOG_BLOCK_STATUSReceive_direct(Channel src, Inside ph, REMOTE_LOG_BLOCK_STATUS pack) {OnREMOTE_LOG_BLOCK_STATUSReceive(this, ph,  pack);}
            public event REMOTE_LOG_BLOCK_STATUSReceiveHandler OnREMOTE_LOG_BLOCK_STATUSReceive;
            public delegate void REMOTE_LOG_BLOCK_STATUSReceiveHandler(Channel src, Inside ph, REMOTE_LOG_BLOCK_STATUS pack);
            public void OnLED_CONTROLReceive_direct(Channel src, Inside ph, LED_CONTROL pack) {OnLED_CONTROLReceive(this, ph,  pack);}
            public event LED_CONTROLReceiveHandler OnLED_CONTROLReceive;
            public delegate void LED_CONTROLReceiveHandler(Channel src, Inside ph, LED_CONTROL pack);
            public void OnMAG_CAL_PROGRESSReceive_direct(Channel src, Inside ph, MAG_CAL_PROGRESS pack) {OnMAG_CAL_PROGRESSReceive(this, ph,  pack);}
            public event MAG_CAL_PROGRESSReceiveHandler OnMAG_CAL_PROGRESSReceive;
            public delegate void MAG_CAL_PROGRESSReceiveHandler(Channel src, Inside ph, MAG_CAL_PROGRESS pack);
            public void OnMAG_CAL_REPORTReceive_direct(Channel src, Inside ph, MAG_CAL_REPORT pack) {OnMAG_CAL_REPORTReceive(this, ph,  pack);}
            public event MAG_CAL_REPORTReceiveHandler OnMAG_CAL_REPORTReceive;
            public delegate void MAG_CAL_REPORTReceiveHandler(Channel src, Inside ph, MAG_CAL_REPORT pack);
            public void OnEKF_STATUS_REPORTReceive_direct(Channel src, Inside ph, EKF_STATUS_REPORT pack) {OnEKF_STATUS_REPORTReceive(this, ph,  pack);}
            public event EKF_STATUS_REPORTReceiveHandler OnEKF_STATUS_REPORTReceive;
            public delegate void EKF_STATUS_REPORTReceiveHandler(Channel src, Inside ph, EKF_STATUS_REPORT pack);
            public void OnPID_TUNINGReceive_direct(Channel src, Inside ph, PID_TUNING pack) {OnPID_TUNINGReceive(this, ph,  pack);}
            public event PID_TUNINGReceiveHandler OnPID_TUNINGReceive;
            public delegate void PID_TUNINGReceiveHandler(Channel src, Inside ph, PID_TUNING pack);
            public void OnGIMBAL_REPORTReceive_direct(Channel src, Inside ph, GIMBAL_REPORT pack) {OnGIMBAL_REPORTReceive(this, ph,  pack);}
            public event GIMBAL_REPORTReceiveHandler OnGIMBAL_REPORTReceive;
            public delegate void GIMBAL_REPORTReceiveHandler(Channel src, Inside ph, GIMBAL_REPORT pack);
            public void OnGIMBAL_CONTROLReceive_direct(Channel src, Inside ph, GIMBAL_CONTROL pack) {OnGIMBAL_CONTROLReceive(this, ph,  pack);}
            public event GIMBAL_CONTROLReceiveHandler OnGIMBAL_CONTROLReceive;
            public delegate void GIMBAL_CONTROLReceiveHandler(Channel src, Inside ph, GIMBAL_CONTROL pack);
            public void OnGIMBAL_TORQUE_CMD_REPORTReceive_direct(Channel src, Inside ph, GIMBAL_TORQUE_CMD_REPORT pack) {OnGIMBAL_TORQUE_CMD_REPORTReceive(this, ph,  pack);}
            public event GIMBAL_TORQUE_CMD_REPORTReceiveHandler OnGIMBAL_TORQUE_CMD_REPORTReceive;
            public delegate void GIMBAL_TORQUE_CMD_REPORTReceiveHandler(Channel src, Inside ph, GIMBAL_TORQUE_CMD_REPORT pack);
            public void OnGOPRO_HEARTBEATReceive_direct(Channel src, Inside ph, GOPRO_HEARTBEAT pack) {OnGOPRO_HEARTBEATReceive(this, ph,  pack);}
            public event GOPRO_HEARTBEATReceiveHandler OnGOPRO_HEARTBEATReceive;
            public delegate void GOPRO_HEARTBEATReceiveHandler(Channel src, Inside ph, GOPRO_HEARTBEAT pack);
            public void OnGOPRO_GET_REQUESTReceive_direct(Channel src, Inside ph, GOPRO_GET_REQUEST pack) {OnGOPRO_GET_REQUESTReceive(this, ph,  pack);}
            public event GOPRO_GET_REQUESTReceiveHandler OnGOPRO_GET_REQUESTReceive;
            public delegate void GOPRO_GET_REQUESTReceiveHandler(Channel src, Inside ph, GOPRO_GET_REQUEST pack);
            public void OnGOPRO_GET_RESPONSEReceive_direct(Channel src, Inside ph, GOPRO_GET_RESPONSE pack) {OnGOPRO_GET_RESPONSEReceive(this, ph,  pack);}
            public event GOPRO_GET_RESPONSEReceiveHandler OnGOPRO_GET_RESPONSEReceive;
            public delegate void GOPRO_GET_RESPONSEReceiveHandler(Channel src, Inside ph, GOPRO_GET_RESPONSE pack);
            public void OnGOPRO_SET_REQUESTReceive_direct(Channel src, Inside ph, GOPRO_SET_REQUEST pack) {OnGOPRO_SET_REQUESTReceive(this, ph,  pack);}
            public event GOPRO_SET_REQUESTReceiveHandler OnGOPRO_SET_REQUESTReceive;
            public delegate void GOPRO_SET_REQUESTReceiveHandler(Channel src, Inside ph, GOPRO_SET_REQUEST pack);
            public void OnGOPRO_SET_RESPONSEReceive_direct(Channel src, Inside ph, GOPRO_SET_RESPONSE pack) {OnGOPRO_SET_RESPONSEReceive(this, ph,  pack);}
            public event GOPRO_SET_RESPONSEReceiveHandler OnGOPRO_SET_RESPONSEReceive;
            public delegate void GOPRO_SET_RESPONSEReceiveHandler(Channel src, Inside ph, GOPRO_SET_RESPONSE pack);
            public void OnRPMReceive_direct(Channel src, Inside ph, RPM pack) {OnRPMReceive(this, ph,  pack);}
            public event RPMReceiveHandler OnRPMReceive;
            public delegate void RPMReceiveHandler(Channel src, Inside ph, RPM pack);
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
            public void OnUAVIONIX_ADSB_OUT_CFGReceive_direct(Channel src, Inside ph, UAVIONIX_ADSB_OUT_CFG pack) {OnUAVIONIX_ADSB_OUT_CFGReceive(this, ph,  pack);}
            public event UAVIONIX_ADSB_OUT_CFGReceiveHandler OnUAVIONIX_ADSB_OUT_CFGReceive;
            public delegate void UAVIONIX_ADSB_OUT_CFGReceiveHandler(Channel src, Inside ph, UAVIONIX_ADSB_OUT_CFG pack);
            public void OnUAVIONIX_ADSB_OUT_DYNAMICReceive_direct(Channel src, Inside ph, UAVIONIX_ADSB_OUT_DYNAMIC pack) {OnUAVIONIX_ADSB_OUT_DYNAMICReceive(this, ph,  pack);}
            public event UAVIONIX_ADSB_OUT_DYNAMICReceiveHandler OnUAVIONIX_ADSB_OUT_DYNAMICReceive;
            public delegate void UAVIONIX_ADSB_OUT_DYNAMICReceiveHandler(Channel src, Inside ph, UAVIONIX_ADSB_OUT_DYNAMIC pack);
            public void OnUAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORTReceive_direct(Channel src, Inside ph, UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT pack) {OnUAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORTReceive(this, ph,  pack);}
            public event UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORTReceiveHandler OnUAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORTReceive;
            public delegate void UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORTReceiveHandler(Channel src, Inside ph, UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT pack);
            public void OnDEVICE_OP_READReceive_direct(Channel src, Inside ph, DEVICE_OP_READ pack) {OnDEVICE_OP_READReceive(this, ph,  pack);}
            public event DEVICE_OP_READReceiveHandler OnDEVICE_OP_READReceive;
            public delegate void DEVICE_OP_READReceiveHandler(Channel src, Inside ph, DEVICE_OP_READ pack);
            public void OnDEVICE_OP_READ_REPLYReceive_direct(Channel src, Inside ph, DEVICE_OP_READ_REPLY pack) {OnDEVICE_OP_READ_REPLYReceive(this, ph,  pack);}
            public event DEVICE_OP_READ_REPLYReceiveHandler OnDEVICE_OP_READ_REPLYReceive;
            public delegate void DEVICE_OP_READ_REPLYReceiveHandler(Channel src, Inside ph, DEVICE_OP_READ_REPLY pack);
            public void OnDEVICE_OP_WRITEReceive_direct(Channel src, Inside ph, DEVICE_OP_WRITE pack) {OnDEVICE_OP_WRITEReceive(this, ph,  pack);}
            public event DEVICE_OP_WRITEReceiveHandler OnDEVICE_OP_WRITEReceive;
            public delegate void DEVICE_OP_WRITEReceiveHandler(Channel src, Inside ph, DEVICE_OP_WRITE pack);
            public void OnDEVICE_OP_WRITE_REPLYReceive_direct(Channel src, Inside ph, DEVICE_OP_WRITE_REPLY pack) {OnDEVICE_OP_WRITE_REPLYReceive(this, ph,  pack);}
            public event DEVICE_OP_WRITE_REPLYReceiveHandler OnDEVICE_OP_WRITE_REPLYReceive;
            public delegate void DEVICE_OP_WRITE_REPLYReceiveHandler(Channel src, Inside ph, DEVICE_OP_WRITE_REPLY pack);
            public void OnADAP_TUNINGReceive_direct(Channel src, Inside ph, ADAP_TUNING pack) {OnADAP_TUNINGReceive(this, ph,  pack);}
            public event ADAP_TUNINGReceiveHandler OnADAP_TUNINGReceive;
            public delegate void ADAP_TUNINGReceiveHandler(Channel src, Inside ph, ADAP_TUNING pack);
            public void OnVISION_POSITION_DELTAReceive_direct(Channel src, Inside ph, VISION_POSITION_DELTA pack) {OnVISION_POSITION_DELTAReceive(this, ph,  pack);}
            public event VISION_POSITION_DELTAReceiveHandler OnVISION_POSITION_DELTAReceive;
            public delegate void VISION_POSITION_DELTAReceiveHandler(Channel src, Inside ph, VISION_POSITION_DELTA pack);

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
                    case 3:
                        if(pack == null) return new POSITION_TARGET_LOCAL_NED();
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
                    case 81:
                        if(pack == null) return new MANUAL_SETPOINT();
                        break;
                    case 82:
                        if(pack == null) return new SET_ATTITUDE_TARGET();
                        break;
                    case 83:
                        if(pack == null) return new ATTITUDE_TARGET();
                        break;
                    case 84:
                        if(pack == null) return new SET_POSITION_TARGET_LOCAL_NED();
                        break;
                    case 86:
                        if(pack == null) return new SET_POSITION_TARGET_GLOBAL_INT();
                        break;
                    case 87:
                        if(pack == null) return new POSITION_TARGET_GLOBAL_INT();
                        break;
                    case 89:
                        if(pack == null) return new LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
                        break;
                    case 90:
                        if(pack == null) return new HIL_STATE();
                        break;
                    case 91:
                        if(pack == null) return new HIL_CONTROLS();
                        break;
                    case 92:
                        if(pack == null) return new HIL_RC_INPUTS_RAW();
                        break;
                    case 93:
                        if(pack == null) return new HIL_ACTUATOR_CONTROLS();
                        break;
                    case 100:
                        if(pack == null) return new OPTICAL_FLOW();
                        break;
                    case 101:
                        if(pack == null) return new GLOBAL_VISION_POSITION_ESTIMATE();
                        break;
                    case 150:
                        if(pack == null) return new SENSOR_OFFSETS();
                        OnSENSOR_OFFSETSReceive(this, ph, (SENSOR_OFFSETS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 151:
                        if(pack == null) return new SET_MAG_OFFSETS();
                        OnSET_MAG_OFFSETSReceive(this, ph, (SET_MAG_OFFSETS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 152:
                        if(pack == null) return new MEMINFO();
                        OnMEMINFOReceive(this, ph, (MEMINFO) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 153:
                        if(pack == null) return new AP_ADC();
                        OnAP_ADCReceive(this, ph, (AP_ADC) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 154:
                        if(pack == null) return new DIGICAM_CONFIGURE();
                        OnDIGICAM_CONFIGUREReceive(this, ph, (DIGICAM_CONFIGURE) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 155:
                        if(pack == null) return new DIGICAM_CONTROL();
                        OnDIGICAM_CONTROLReceive(this, ph, (DIGICAM_CONTROL) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 156:
                        if(pack == null) return new MOUNT_CONFIGURE();
                        OnMOUNT_CONFIGUREReceive(this, ph, (MOUNT_CONFIGURE) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 157:
                        if(pack == null) return new MOUNT_CONTROL();
                        OnMOUNT_CONTROLReceive(this, ph, (MOUNT_CONTROL) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 158:
                        if(pack == null) return new MOUNT_STATUS();
                        OnMOUNT_STATUSReceive(this, ph, (MOUNT_STATUS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 160:
                        if(pack == null) return new FENCE_POINT();
                        OnFENCE_POINTReceive(this, ph, (FENCE_POINT) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 161:
                        if(pack == null) return new FENCE_FETCH_POINT();
                        OnFENCE_FETCH_POINTReceive(this, ph, (FENCE_FETCH_POINT) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 162:
                        if(pack == null) return new FENCE_STATUS();
                        OnFENCE_STATUSReceive(this, ph, (FENCE_STATUS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 163:
                        if(pack == null) return new AHRS();
                        OnAHRSReceive(this, ph, (AHRS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 164:
                        if(pack == null) return new SIMSTATE();
                        OnSIMSTATEReceive(this, ph, (SIMSTATE) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 165:
                        if(pack == null) return new HWSTATUS();
                        OnHWSTATUSReceive(this, ph, (HWSTATUS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 166:
                        if(pack == null) return new RADIO();
                        OnRADIOReceive(this, ph, (RADIO) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 167:
                        if(pack == null) return new LIMITS_STATUS();
                        OnLIMITS_STATUSReceive(this, ph, (LIMITS_STATUS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 168:
                        if(pack == null) return new WIND();
                        OnWINDReceive(this, ph, (WIND) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 169:
                        if(pack == null) return new DATA16();
                        OnDATA16Receive(this, ph, (DATA16) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 170:
                        if(pack == null) return new DATA32();
                        OnDATA32Receive(this, ph, (DATA32) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 171:
                        if(pack == null) return new DATA64();
                        OnDATA64Receive(this, ph, (DATA64) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 172:
                        if(pack == null) return new DATA96();
                        OnDATA96Receive(this, ph, (DATA96) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 173:
                        if(pack == null) return new RANGEFINDER();
                        OnRANGEFINDERReceive(this, ph, (RANGEFINDER) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 174:
                        if(pack == null) return new AIRSPEED_AUTOCAL();
                        OnAIRSPEED_AUTOCALReceive(this, ph, (AIRSPEED_AUTOCAL) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 175:
                        if(pack == null) return new RALLY_POINT();
                        OnRALLY_POINTReceive(this, ph, (RALLY_POINT) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 176:
                        if(pack == null) return new RALLY_FETCH_POINT();
                        OnRALLY_FETCH_POINTReceive(this, ph, (RALLY_FETCH_POINT) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 177:
                        if(pack == null) return new COMPASSMOT_STATUS();
                        OnCOMPASSMOT_STATUSReceive(this, ph, (COMPASSMOT_STATUS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 178:
                        if(pack == null) return new AHRS2();
                        OnAHRS2Receive(this, ph, (AHRS2) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 179:
                        if(pack == null) return new CAMERA_STATUS();
                        OnCAMERA_STATUSReceive(this, ph, (CAMERA_STATUS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 180:
                        if(pack == null) return new CAMERA_FEEDBACK();
                        OnCAMERA_FEEDBACKReceive(this, ph, (CAMERA_FEEDBACK) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 181:
                        if(pack == null) return new BATTERY2();
                        OnBATTERY2Receive(this, ph, (BATTERY2) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 182:
                        if(pack == null) return new AHRS3();
                        OnAHRS3Receive(this, ph, (AHRS3) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 183:
                        if(pack == null) return new AUTOPILOT_VERSION_REQUEST();
                        OnAUTOPILOT_VERSION_REQUESTReceive(this, ph, (AUTOPILOT_VERSION_REQUEST) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 184:
                        if(pack == null) return new REMOTE_LOG_DATA_BLOCK();
                        OnREMOTE_LOG_DATA_BLOCKReceive(this, ph, (REMOTE_LOG_DATA_BLOCK) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 185:
                        if(pack == null) return new REMOTE_LOG_BLOCK_STATUS();
                        OnREMOTE_LOG_BLOCK_STATUSReceive(this, ph, (REMOTE_LOG_BLOCK_STATUS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 186:
                        if(pack == null) return new LED_CONTROL();
                        OnLED_CONTROLReceive(this, ph, (LED_CONTROL) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 191:
                        if(pack == null) return new MAG_CAL_PROGRESS();
                        OnMAG_CAL_PROGRESSReceive(this, ph, (MAG_CAL_PROGRESS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 192:
                        if(pack == null) return new MAG_CAL_REPORT();
                        OnMAG_CAL_REPORTReceive(this, ph, (MAG_CAL_REPORT) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 193:
                        if(pack == null) return new EKF_STATUS_REPORT();
                        OnEKF_STATUS_REPORTReceive(this, ph, (EKF_STATUS_REPORT) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 194:
                        if(pack == null) return new PID_TUNING();
                        OnPID_TUNINGReceive(this, ph, (PID_TUNING) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 200:
                        if(pack == null) return new GIMBAL_REPORT();
                        OnGIMBAL_REPORTReceive(this, ph, (GIMBAL_REPORT) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 201:
                        if(pack == null) return new GIMBAL_CONTROL();
                        OnGIMBAL_CONTROLReceive(this, ph, (GIMBAL_CONTROL) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 214:
                        if(pack == null) return new GIMBAL_TORQUE_CMD_REPORT();
                        OnGIMBAL_TORQUE_CMD_REPORTReceive(this, ph, (GIMBAL_TORQUE_CMD_REPORT) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 215:
                        if(pack == null) return new GOPRO_HEARTBEAT();
                        OnGOPRO_HEARTBEATReceive(this, ph, (GOPRO_HEARTBEAT) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 216:
                        if(pack == null) return new GOPRO_GET_REQUEST();
                        OnGOPRO_GET_REQUESTReceive(this, ph, (GOPRO_GET_REQUEST) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 217:
                        if(pack == null) return new GOPRO_GET_RESPONSE();
                        OnGOPRO_GET_RESPONSEReceive(this, ph, (GOPRO_GET_RESPONSE) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 218:
                        if(pack == null) return new GOPRO_SET_REQUEST();
                        OnGOPRO_SET_REQUESTReceive(this, ph, (GOPRO_SET_REQUEST) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 219:
                        if(pack == null) return new GOPRO_SET_RESPONSE();
                        OnGOPRO_SET_RESPONSEReceive(this, ph, (GOPRO_SET_RESPONSE) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 226:
                        if(pack == null) return new RPM();
                        OnRPMReceive(this, ph, (RPM) pack);//no any host channels can receive this pack. Handle it with test channel handler
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
                    case 10001:
                        if(pack == null) return new UAVIONIX_ADSB_OUT_CFG();
                        OnUAVIONIX_ADSB_OUT_CFGReceive(this, ph, (UAVIONIX_ADSB_OUT_CFG) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 10002:
                        if(pack == null) return new UAVIONIX_ADSB_OUT_DYNAMIC();
                        OnUAVIONIX_ADSB_OUT_DYNAMICReceive(this, ph, (UAVIONIX_ADSB_OUT_DYNAMIC) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 10003:
                        if(pack == null) return new UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT();
                        OnUAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORTReceive(this, ph, (UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 11000:
                        if(pack == null) return new DEVICE_OP_READ();
                        OnDEVICE_OP_READReceive(this, ph, (DEVICE_OP_READ) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 11001:
                        if(pack == null) return new DEVICE_OP_READ_REPLY();
                        OnDEVICE_OP_READ_REPLYReceive(this, ph, (DEVICE_OP_READ_REPLY) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 11002:
                        if(pack == null) return new DEVICE_OP_WRITE();
                        OnDEVICE_OP_WRITEReceive(this, ph, (DEVICE_OP_WRITE) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 11003:
                        if(pack == null) return new DEVICE_OP_WRITE_REPLY();
                        OnDEVICE_OP_WRITE_REPLYReceive(this, ph, (DEVICE_OP_WRITE_REPLY) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 11010:
                        if(pack == null) return new ADAP_TUNING();
                        OnADAP_TUNINGReceive(this, ph, (ADAP_TUNING) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 11011:
                        if(pack == null) return new VISION_POSITION_DELTA();
                        OnVISION_POSITION_DELTAReceive(this, ph, (VISION_POSITION_DELTA) pack);//no any host channels can receive this pack. Handle it with test channel handler
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
                Debug.Assert(pack.mavlink_version == (byte)(byte)19);
                Debug.Assert(pack.system_status == (MAV_STATE)MAV_STATE.MAV_STATE_ACTIVE);
                Debug.Assert(pack.autopilot == (MAV_AUTOPILOT)MAV_AUTOPILOT.MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY);
                Debug.Assert(pack.type == (MAV_TYPE)MAV_TYPE.MAV_TYPE_VTOL_RESERVED4);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED);
                Debug.Assert(pack.custom_mode == (uint)1652389697U);
            };
            HEARTBEAT p0 = new HEARTBEAT();
            PH.setPack(p0);
            p0.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED;
            p0.system_status = (MAV_STATE)MAV_STATE.MAV_STATE_ACTIVE;
            p0.custom_mode = (uint)1652389697U;
            p0.type = (MAV_TYPE)MAV_TYPE.MAV_TYPE_VTOL_RESERVED4;
            p0.mavlink_version = (byte)(byte)19;
            p0.autopilot = (MAV_AUTOPILOT)MAV_AUTOPILOT.MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY;
            SMP_TEST_CH.send(p0);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.onboard_control_sensors_health == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN);
                Debug.Assert(pack.current_battery == (short)(short) -31410);
                Debug.Assert(pack.errors_count4 == (ushort)(ushort)48263);
                Debug.Assert(pack.errors_count1 == (ushort)(ushort)57554);
                Debug.Assert(pack.errors_comm == (ushort)(ushort)34203);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte)49);
                Debug.Assert(pack.load == (ushort)(ushort)56543);
                Debug.Assert(pack.onboard_control_sensors_enabled == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN);
                Debug.Assert(pack.onboard_control_sensors_present == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2);
                Debug.Assert(pack.voltage_battery == (ushort)(ushort)11134);
                Debug.Assert(pack.errors_count2 == (ushort)(ushort)64380);
                Debug.Assert(pack.drop_rate_comm == (ushort)(ushort)2137);
                Debug.Assert(pack.errors_count3 == (ushort)(ushort)1053);
            };
            SYS_STATUS p1 = new SYS_STATUS();
            PH.setPack(p1);
            p1.voltage_battery = (ushort)(ushort)11134;
            p1.errors_count4 = (ushort)(ushort)48263;
            p1.errors_count3 = (ushort)(ushort)1053;
            p1.onboard_control_sensors_enabled = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN;
            p1.current_battery = (short)(short) -31410;
            p1.drop_rate_comm = (ushort)(ushort)2137;
            p1.errors_count1 = (ushort)(ushort)57554;
            p1.battery_remaining = (sbyte)(sbyte)49;
            p1.onboard_control_sensors_present = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2;
            p1.onboard_control_sensors_health = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN;
            p1.errors_comm = (ushort)(ushort)34203;
            p1.load = (ushort)(ushort)56543;
            p1.errors_count2 = (ushort)(ushort)64380;
            SMP_TEST_CH.send(p1);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYSTEM_TIMEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_unix_usec == (ulong)1378644306563677377L);
                Debug.Assert(pack.time_boot_ms == (uint)461559219U);
            };
            SYSTEM_TIME p2 = new SYSTEM_TIME();
            PH.setPack(p2);
            p2.time_unix_usec = (ulong)1378644306563677377L;
            p2.time_boot_ms = (uint)461559219U;
            SMP_TEST_CH.send(p2);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float) -1.478317E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)62038);
                Debug.Assert(pack.vx == (float)2.4553337E38F);
                Debug.Assert(pack.yaw == (float) -2.3374027E37F);
                Debug.Assert(pack.afz == (float) -2.6942404E38F);
                Debug.Assert(pack.yaw_rate == (float)1.9477774E38F);
                Debug.Assert(pack.vy == (float) -3.1037163E38F);
                Debug.Assert(pack.z == (float)2.1553628E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3387097275U);
                Debug.Assert(pack.afx == (float)1.6081078E38F);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_ENU);
                Debug.Assert(pack.afy == (float) -6.86386E37F);
                Debug.Assert(pack.vz == (float)2.8047108E38F);
                Debug.Assert(pack.x == (float) -1.2341486E38F);
            };
            POSITION_TARGET_LOCAL_NED p3 = new POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.afz = (float) -2.6942404E38F;
            p3.z = (float)2.1553628E38F;
            p3.yaw_rate = (float)1.9477774E38F;
            p3.afx = (float)1.6081078E38F;
            p3.vy = (float) -3.1037163E38F;
            p3.afy = (float) -6.86386E37F;
            p3.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_ENU;
            p3.time_boot_ms = (uint)3387097275U;
            p3.y = (float) -1.478317E38F;
            p3.yaw = (float) -2.3374027E37F;
            p3.x = (float) -1.2341486E38F;
            p3.vx = (float)2.4553337E38F;
            p3.vz = (float)2.8047108E38F;
            p3.type_mask = (ushort)(ushort)62038;
            SMP_TEST_CH.send(p3);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (uint)1315154914U);
                Debug.Assert(pack.target_system == (byte)(byte)23);
                Debug.Assert(pack.target_component == (byte)(byte)230);
                Debug.Assert(pack.time_usec == (ulong)2185172024271669485L);
            };
            PING p4 = new PING();
            PH.setPack(p4);
            p4.target_component = (byte)(byte)230;
            p4.seq = (uint)1315154914U;
            p4.target_system = (byte)(byte)23;
            p4.time_usec = (ulong)2185172024271669485L;
            SMP_TEST_CH.send(p4);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.passkey_LEN(ph) == 3);
                Debug.Assert(pack.passkey_TRY(ph).Equals("kre"));
                Debug.Assert(pack.version == (byte)(byte)71);
                Debug.Assert(pack.target_system == (byte)(byte)194);
                Debug.Assert(pack.control_request == (byte)(byte)143);
            };
            CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
            PH.setPack(p5);
            p5.passkey_SET("kre", PH) ;
            p5.version = (byte)(byte)71;
            p5.target_system = (byte)(byte)194;
            p5.control_request = (byte)(byte)143;
            SMP_TEST_CH.send(p5);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROL_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.control_request == (byte)(byte)153);
                Debug.Assert(pack.gcs_system_id == (byte)(byte)197);
                Debug.Assert(pack.ack == (byte)(byte)252);
            };
            CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
            PH.setPack(p6);
            p6.gcs_system_id = (byte)(byte)197;
            p6.control_request = (byte)(byte)153;
            p6.ack = (byte)(byte)252;
            SMP_TEST_CH.send(p6);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnAUTH_KEYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.key_LEN(ph) == 12);
                Debug.Assert(pack.key_TRY(ph).Equals("zrugyOCgxtlm"));
            };
            AUTH_KEY p7 = new AUTH_KEY();
            PH.setPack(p7);
            p7.key_SET("zrugyOCgxtlm", PH) ;
            SMP_TEST_CH.send(p7);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_MODEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)153);
                Debug.Assert(pack.base_mode == (MAV_MODE)MAV_MODE.MAV_MODE_MANUAL_DISARMED);
                Debug.Assert(pack.custom_mode == (uint)3167166408U);
            };
            SET_MODE p11 = new SET_MODE();
            PH.setPack(p11);
            p11.custom_mode = (uint)3167166408U;
            p11.base_mode = (MAV_MODE)MAV_MODE.MAV_MODE_MANUAL_DISARMED;
            p11.target_system = (byte)(byte)153;
            SMP_TEST_CH.send(p11);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)9);
                Debug.Assert(pack.param_id_LEN(ph) == 15);
                Debug.Assert(pack.param_id_TRY(ph).Equals("zjyhmacppfFejui"));
                Debug.Assert(pack.param_index == (short)(short) -8038);
                Debug.Assert(pack.target_component == (byte)(byte)52);
            };
            PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
            PH.setPack(p20);
            p20.target_component = (byte)(byte)52;
            p20.target_system = (byte)(byte)9;
            p20.param_id_SET("zjyhmacppfFejui", PH) ;
            p20.param_index = (short)(short) -8038;
            SMP_TEST_CH.send(p20);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)215);
                Debug.Assert(pack.target_component == (byte)(byte)157);
            };
            PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
            PH.setPack(p21);
            p21.target_component = (byte)(byte)157;
            p21.target_system = (byte)(byte)215;
            SMP_TEST_CH.send(p21);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_type == (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16);
                Debug.Assert(pack.param_count == (ushort)(ushort)18852);
                Debug.Assert(pack.param_index == (ushort)(ushort)15508);
                Debug.Assert(pack.param_value == (float)1.1483605E38F);
                Debug.Assert(pack.param_id_LEN(ph) == 12);
                Debug.Assert(pack.param_id_TRY(ph).Equals("ajissuzohcpR"));
            };
            PARAM_VALUE p22 = new PARAM_VALUE();
            PH.setPack(p22);
            p22.param_id_SET("ajissuzohcpR", PH) ;
            p22.param_count = (ushort)(ushort)18852;
            p22.param_value = (float)1.1483605E38F;
            p22.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16;
            p22.param_index = (ushort)(ushort)15508;
            SMP_TEST_CH.send(p22);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)19);
                Debug.Assert(pack.target_component == (byte)(byte)13);
                Debug.Assert(pack.param_value == (float)1.62832E38F);
                Debug.Assert(pack.param_id_LEN(ph) == 11);
                Debug.Assert(pack.param_id_TRY(ph).Equals("VmdWpqpjerH"));
                Debug.Assert(pack.param_type == (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT64);
            };
            PARAM_SET p23 = new PARAM_SET();
            PH.setPack(p23);
            p23.param_id_SET("VmdWpqpjerH", PH) ;
            p23.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT64;
            p23.target_system = (byte)(byte)19;
            p23.target_component = (byte)(byte)13;
            p23.param_value = (float)1.62832E38F;
            SMP_TEST_CH.send(p23);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RAW_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.eph == (ushort)(ushort)20457);
                Debug.Assert(pack.fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS);
                Debug.Assert(pack.hdg_acc_TRY(ph) == (uint)3090948739U);
                Debug.Assert(pack.alt_ellipsoid_TRY(ph) == (int)1660537742);
                Debug.Assert(pack.lat == (int) -655597386);
                Debug.Assert(pack.time_usec == (ulong)116596332801619830L);
                Debug.Assert(pack.cog == (ushort)(ushort)57609);
                Debug.Assert(pack.satellites_visible == (byte)(byte)243);
                Debug.Assert(pack.lon == (int) -1085940586);
                Debug.Assert(pack.alt == (int) -304524403);
                Debug.Assert(pack.epv == (ushort)(ushort)49824);
                Debug.Assert(pack.h_acc_TRY(ph) == (uint)1090011070U);
                Debug.Assert(pack.v_acc_TRY(ph) == (uint)821327004U);
                Debug.Assert(pack.vel == (ushort)(ushort)22012);
                Debug.Assert(pack.vel_acc_TRY(ph) == (uint)1831938835U);
            };
            GPS_RAW_INT p24 = new GPS_RAW_INT();
            PH.setPack(p24);
            p24.time_usec = (ulong)116596332801619830L;
            p24.v_acc_SET((uint)821327004U, PH) ;
            p24.alt_ellipsoid_SET((int)1660537742, PH) ;
            p24.alt = (int) -304524403;
            p24.lon = (int) -1085940586;
            p24.vel_acc_SET((uint)1831938835U, PH) ;
            p24.vel = (ushort)(ushort)22012;
            p24.epv = (ushort)(ushort)49824;
            p24.cog = (ushort)(ushort)57609;
            p24.satellites_visible = (byte)(byte)243;
            p24.lat = (int) -655597386;
            p24.eph = (ushort)(ushort)20457;
            p24.hdg_acc_SET((uint)3090948739U, PH) ;
            p24.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS;
            p24.h_acc_SET((uint)1090011070U, PH) ;
            SMP_TEST_CH.send(p24);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellites_visible == (byte)(byte)195);
                Debug.Assert(pack.satellite_used.SequenceEqual(new byte[] {(byte)11, (byte)186, (byte)117, (byte)255, (byte)162, (byte)34, (byte)54, (byte)68, (byte)70, (byte)20, (byte)219, (byte)219, (byte)41, (byte)31, (byte)254, (byte)200, (byte)175, (byte)4, (byte)19, (byte)119}));
                Debug.Assert(pack.satellite_snr.SequenceEqual(new byte[] {(byte)108, (byte)153, (byte)207, (byte)56, (byte)136, (byte)124, (byte)186, (byte)147, (byte)250, (byte)235, (byte)101, (byte)72, (byte)107, (byte)73, (byte)138, (byte)24, (byte)193, (byte)33, (byte)86, (byte)232}));
                Debug.Assert(pack.satellite_prn.SequenceEqual(new byte[] {(byte)248, (byte)105, (byte)175, (byte)93, (byte)51, (byte)103, (byte)82, (byte)32, (byte)154, (byte)129, (byte)72, (byte)88, (byte)110, (byte)192, (byte)95, (byte)239, (byte)241, (byte)208, (byte)54, (byte)221}));
                Debug.Assert(pack.satellite_azimuth.SequenceEqual(new byte[] {(byte)94, (byte)252, (byte)251, (byte)0, (byte)41, (byte)18, (byte)191, (byte)225, (byte)219, (byte)38, (byte)119, (byte)148, (byte)155, (byte)51, (byte)132, (byte)178, (byte)34, (byte)245, (byte)190, (byte)76}));
                Debug.Assert(pack.satellite_elevation.SequenceEqual(new byte[] {(byte)167, (byte)143, (byte)129, (byte)236, (byte)209, (byte)87, (byte)154, (byte)82, (byte)61, (byte)27, (byte)3, (byte)76, (byte)5, (byte)231, (byte)80, (byte)96, (byte)212, (byte)109, (byte)98, (byte)75}));
            };
            GPS_STATUS p25 = new GPS_STATUS();
            PH.setPack(p25);
            p25.satellite_azimuth_SET(new byte[] {(byte)94, (byte)252, (byte)251, (byte)0, (byte)41, (byte)18, (byte)191, (byte)225, (byte)219, (byte)38, (byte)119, (byte)148, (byte)155, (byte)51, (byte)132, (byte)178, (byte)34, (byte)245, (byte)190, (byte)76}, 0) ;
            p25.satellite_used_SET(new byte[] {(byte)11, (byte)186, (byte)117, (byte)255, (byte)162, (byte)34, (byte)54, (byte)68, (byte)70, (byte)20, (byte)219, (byte)219, (byte)41, (byte)31, (byte)254, (byte)200, (byte)175, (byte)4, (byte)19, (byte)119}, 0) ;
            p25.satellites_visible = (byte)(byte)195;
            p25.satellite_snr_SET(new byte[] {(byte)108, (byte)153, (byte)207, (byte)56, (byte)136, (byte)124, (byte)186, (byte)147, (byte)250, (byte)235, (byte)101, (byte)72, (byte)107, (byte)73, (byte)138, (byte)24, (byte)193, (byte)33, (byte)86, (byte)232}, 0) ;
            p25.satellite_prn_SET(new byte[] {(byte)248, (byte)105, (byte)175, (byte)93, (byte)51, (byte)103, (byte)82, (byte)32, (byte)154, (byte)129, (byte)72, (byte)88, (byte)110, (byte)192, (byte)95, (byte)239, (byte)241, (byte)208, (byte)54, (byte)221}, 0) ;
            p25.satellite_elevation_SET(new byte[] {(byte)167, (byte)143, (byte)129, (byte)236, (byte)209, (byte)87, (byte)154, (byte)82, (byte)61, (byte)27, (byte)3, (byte)76, (byte)5, (byte)231, (byte)80, (byte)96, (byte)212, (byte)109, (byte)98, (byte)75}, 0) ;
            SMP_TEST_CH.send(p25);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zmag == (short)(short)27093);
                Debug.Assert(pack.yacc == (short)(short)13404);
                Debug.Assert(pack.ygyro == (short)(short)22208);
                Debug.Assert(pack.xmag == (short)(short) -24489);
                Debug.Assert(pack.xacc == (short)(short) -5790);
                Debug.Assert(pack.xgyro == (short)(short) -589);
                Debug.Assert(pack.zgyro == (short)(short)7565);
                Debug.Assert(pack.zacc == (short)(short)14658);
                Debug.Assert(pack.time_boot_ms == (uint)1874777803U);
                Debug.Assert(pack.ymag == (short)(short)24238);
            };
            SCALED_IMU p26 = new SCALED_IMU();
            PH.setPack(p26);
            p26.yacc = (short)(short)13404;
            p26.zmag = (short)(short)27093;
            p26.ymag = (short)(short)24238;
            p26.zacc = (short)(short)14658;
            p26.zgyro = (short)(short)7565;
            p26.time_boot_ms = (uint)1874777803U;
            p26.xacc = (short)(short) -5790;
            p26.ygyro = (short)(short)22208;
            p26.xmag = (short)(short) -24489;
            p26.xgyro = (short)(short) -589;
            SMP_TEST_CH.send(p26);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ygyro == (short)(short) -22808);
                Debug.Assert(pack.xgyro == (short)(short) -31184);
                Debug.Assert(pack.xacc == (short)(short) -21821);
                Debug.Assert(pack.zacc == (short)(short)13134);
                Debug.Assert(pack.zmag == (short)(short) -26195);
                Debug.Assert(pack.yacc == (short)(short) -14594);
                Debug.Assert(pack.zgyro == (short)(short)14025);
                Debug.Assert(pack.xmag == (short)(short) -10770);
                Debug.Assert(pack.time_usec == (ulong)3551345297408072848L);
                Debug.Assert(pack.ymag == (short)(short) -3497);
            };
            RAW_IMU p27 = new RAW_IMU();
            PH.setPack(p27);
            p27.zgyro = (short)(short)14025;
            p27.zmag = (short)(short) -26195;
            p27.ygyro = (short)(short) -22808;
            p27.ymag = (short)(short) -3497;
            p27.xgyro = (short)(short) -31184;
            p27.time_usec = (ulong)3551345297408072848L;
            p27.xacc = (short)(short) -21821;
            p27.xmag = (short)(short) -10770;
            p27.yacc = (short)(short) -14594;
            p27.zacc = (short)(short)13134;
            SMP_TEST_CH.send(p27);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)6070788287089060843L);
                Debug.Assert(pack.press_diff1 == (short)(short)29097);
                Debug.Assert(pack.temperature == (short)(short) -5631);
                Debug.Assert(pack.press_abs == (short)(short) -14896);
                Debug.Assert(pack.press_diff2 == (short)(short) -28272);
            };
            RAW_PRESSURE p28 = new RAW_PRESSURE();
            PH.setPack(p28);
            p28.time_usec = (ulong)6070788287089060843L;
            p28.temperature = (short)(short) -5631;
            p28.press_diff1 = (short)(short)29097;
            p28.press_abs = (short)(short) -14896;
            p28.press_diff2 = (short)(short) -28272;
            SMP_TEST_CH.send(p28);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_diff == (float)2.3074172E38F);
                Debug.Assert(pack.press_abs == (float) -2.683883E38F);
                Debug.Assert(pack.temperature == (short)(short)18204);
                Debug.Assert(pack.time_boot_ms == (uint)1330174210U);
            };
            SCALED_PRESSURE p29 = new SCALED_PRESSURE();
            PH.setPack(p29);
            p29.temperature = (short)(short)18204;
            p29.time_boot_ms = (uint)1330174210U;
            p29.press_abs = (float) -2.683883E38F;
            p29.press_diff = (float)2.3074172E38F;
            SMP_TEST_CH.send(p29);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll == (float) -9.721634E37F);
                Debug.Assert(pack.rollspeed == (float)7.542091E37F);
                Debug.Assert(pack.yaw == (float) -2.5884387E38F);
                Debug.Assert(pack.pitchspeed == (float) -2.552337E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3199355739U);
                Debug.Assert(pack.pitch == (float) -1.6148866E38F);
                Debug.Assert(pack.yawspeed == (float)3.0891327E38F);
            };
            ATTITUDE p30 = new ATTITUDE();
            PH.setPack(p30);
            p30.pitch = (float) -1.6148866E38F;
            p30.time_boot_ms = (uint)3199355739U;
            p30.pitchspeed = (float) -2.552337E38F;
            p30.yaw = (float) -2.5884387E38F;
            p30.rollspeed = (float)7.542091E37F;
            p30.yawspeed = (float)3.0891327E38F;
            p30.roll = (float) -9.721634E37F;
            SMP_TEST_CH.send(p30);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q3 == (float)1.231882E38F);
                Debug.Assert(pack.rollspeed == (float)3.338857E38F);
                Debug.Assert(pack.yawspeed == (float) -2.6536132E38F);
                Debug.Assert(pack.q1 == (float)1.6847622E38F);
                Debug.Assert(pack.q2 == (float)9.965675E37F);
                Debug.Assert(pack.time_boot_ms == (uint)2816002999U);
                Debug.Assert(pack.pitchspeed == (float) -2.4707975E38F);
                Debug.Assert(pack.q4 == (float) -2.0070243E38F);
            };
            ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
            PH.setPack(p31);
            p31.q3 = (float)1.231882E38F;
            p31.pitchspeed = (float) -2.4707975E38F;
            p31.q4 = (float) -2.0070243E38F;
            p31.time_boot_ms = (uint)2816002999U;
            p31.yawspeed = (float) -2.6536132E38F;
            p31.rollspeed = (float)3.338857E38F;
            p31.q2 = (float)9.965675E37F;
            p31.q1 = (float)1.6847622E38F;
            SMP_TEST_CH.send(p31);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float)1.0723728E38F);
                Debug.Assert(pack.y == (float)1.7252816E38F);
                Debug.Assert(pack.x == (float) -9.678494E37F);
                Debug.Assert(pack.vz == (float) -9.033308E36F);
                Debug.Assert(pack.vy == (float)2.1717792E38F);
                Debug.Assert(pack.vx == (float)3.2001977E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3218709067U);
            };
            LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
            PH.setPack(p32);
            p32.vx = (float)3.2001977E38F;
            p32.z = (float)1.0723728E38F;
            p32.x = (float) -9.678494E37F;
            p32.vy = (float)2.1717792E38F;
            p32.vz = (float) -9.033308E36F;
            p32.y = (float)1.7252816E38F;
            p32.time_boot_ms = (uint)3218709067U;
            SMP_TEST_CH.send(p32);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vy == (short)(short)30333);
                Debug.Assert(pack.time_boot_ms == (uint)4055989254U);
                Debug.Assert(pack.lon == (int) -1732729449);
                Debug.Assert(pack.lat == (int) -1252728029);
                Debug.Assert(pack.vx == (short)(short) -2796);
                Debug.Assert(pack.relative_alt == (int) -1003262166);
                Debug.Assert(pack.vz == (short)(short)16987);
                Debug.Assert(pack.hdg == (ushort)(ushort)61163);
                Debug.Assert(pack.alt == (int) -770043355);
            };
            GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
            PH.setPack(p33);
            p33.vy = (short)(short)30333;
            p33.hdg = (ushort)(ushort)61163;
            p33.relative_alt = (int) -1003262166;
            p33.time_boot_ms = (uint)4055989254U;
            p33.lon = (int) -1732729449;
            p33.vx = (short)(short) -2796;
            p33.lat = (int) -1252728029;
            p33.alt = (int) -770043355;
            p33.vz = (short)(short)16987;
            SMP_TEST_CH.send(p33);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_SCALEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan5_scaled == (short)(short)3704);
                Debug.Assert(pack.chan6_scaled == (short)(short)12592);
                Debug.Assert(pack.port == (byte)(byte)94);
                Debug.Assert(pack.chan1_scaled == (short)(short)22426);
                Debug.Assert(pack.time_boot_ms == (uint)2619536916U);
                Debug.Assert(pack.chan4_scaled == (short)(short)8137);
                Debug.Assert(pack.chan8_scaled == (short)(short)31991);
                Debug.Assert(pack.chan3_scaled == (short)(short) -18832);
                Debug.Assert(pack.chan7_scaled == (short)(short) -10882);
                Debug.Assert(pack.chan2_scaled == (short)(short)23608);
                Debug.Assert(pack.rssi == (byte)(byte)45);
            };
            RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
            PH.setPack(p34);
            p34.port = (byte)(byte)94;
            p34.chan5_scaled = (short)(short)3704;
            p34.chan1_scaled = (short)(short)22426;
            p34.time_boot_ms = (uint)2619536916U;
            p34.rssi = (byte)(byte)45;
            p34.chan8_scaled = (short)(short)31991;
            p34.chan3_scaled = (short)(short) -18832;
            p34.chan7_scaled = (short)(short) -10882;
            p34.chan4_scaled = (short)(short)8137;
            p34.chan2_scaled = (short)(short)23608;
            p34.chan6_scaled = (short)(short)12592;
            SMP_TEST_CH.send(p34);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)55913);
                Debug.Assert(pack.port == (byte)(byte)185);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)45742);
                Debug.Assert(pack.rssi == (byte)(byte)140);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)42101);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)60273);
                Debug.Assert(pack.time_boot_ms == (uint)1399836331U);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)21671);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)37806);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)17531);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)19426);
            };
            RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
            PH.setPack(p35);
            p35.chan3_raw = (ushort)(ushort)17531;
            p35.port = (byte)(byte)185;
            p35.chan7_raw = (ushort)(ushort)21671;
            p35.chan4_raw = (ushort)(ushort)55913;
            p35.chan6_raw = (ushort)(ushort)37806;
            p35.chan1_raw = (ushort)(ushort)45742;
            p35.time_boot_ms = (uint)1399836331U;
            p35.chan8_raw = (ushort)(ushort)42101;
            p35.chan2_raw = (ushort)(ushort)60273;
            p35.chan5_raw = (ushort)(ushort)19426;
            p35.rssi = (byte)(byte)140;
            SMP_TEST_CH.send(p35);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERVO_OUTPUT_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.servo12_raw_TRY(ph) == (ushort)(ushort)7799);
                Debug.Assert(pack.servo13_raw_TRY(ph) == (ushort)(ushort)50173);
                Debug.Assert(pack.servo9_raw_TRY(ph) == (ushort)(ushort)38578);
                Debug.Assert(pack.servo15_raw_TRY(ph) == (ushort)(ushort)34021);
                Debug.Assert(pack.servo16_raw_TRY(ph) == (ushort)(ushort)65191);
                Debug.Assert(pack.servo10_raw_TRY(ph) == (ushort)(ushort)6815);
                Debug.Assert(pack.servo14_raw_TRY(ph) == (ushort)(ushort)11789);
                Debug.Assert(pack.servo11_raw_TRY(ph) == (ushort)(ushort)2633);
                Debug.Assert(pack.servo6_raw == (ushort)(ushort)27970);
                Debug.Assert(pack.time_usec == (uint)4151657035U);
                Debug.Assert(pack.servo7_raw == (ushort)(ushort)33531);
                Debug.Assert(pack.servo4_raw == (ushort)(ushort)44167);
                Debug.Assert(pack.servo8_raw == (ushort)(ushort)28578);
                Debug.Assert(pack.servo3_raw == (ushort)(ushort)26090);
                Debug.Assert(pack.servo5_raw == (ushort)(ushort)20290);
                Debug.Assert(pack.servo2_raw == (ushort)(ushort)37134);
                Debug.Assert(pack.port == (byte)(byte)108);
                Debug.Assert(pack.servo1_raw == (ushort)(ushort)8784);
            };
            SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
            PH.setPack(p36);
            p36.servo2_raw = (ushort)(ushort)37134;
            p36.servo1_raw = (ushort)(ushort)8784;
            p36.servo4_raw = (ushort)(ushort)44167;
            p36.servo5_raw = (ushort)(ushort)20290;
            p36.port = (byte)(byte)108;
            p36.servo14_raw_SET((ushort)(ushort)11789, PH) ;
            p36.servo8_raw = (ushort)(ushort)28578;
            p36.servo16_raw_SET((ushort)(ushort)65191, PH) ;
            p36.time_usec = (uint)4151657035U;
            p36.servo12_raw_SET((ushort)(ushort)7799, PH) ;
            p36.servo6_raw = (ushort)(ushort)27970;
            p36.servo15_raw_SET((ushort)(ushort)34021, PH) ;
            p36.servo9_raw_SET((ushort)(ushort)38578, PH) ;
            p36.servo10_raw_SET((ushort)(ushort)6815, PH) ;
            p36.servo11_raw_SET((ushort)(ushort)2633, PH) ;
            p36.servo7_raw = (ushort)(ushort)33531;
            p36.servo13_raw_SET((ushort)(ushort)50173, PH) ;
            p36.servo3_raw = (ushort)(ushort)26090;
            SMP_TEST_CH.send(p36);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.end_index == (short)(short)31861);
                Debug.Assert(pack.start_index == (short)(short) -28258);
                Debug.Assert(pack.target_component == (byte)(byte)127);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_system == (byte)(byte)121);
            };
            MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
            PH.setPack(p37);
            p37.end_index = (short)(short)31861;
            p37.start_index = (short)(short) -28258;
            p37.target_system = (byte)(byte)121;
            p37.target_component = (byte)(byte)127;
            p37.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            SMP_TEST_CH.send(p37);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_WRITE_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)212);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.start_index == (short)(short) -23329);
                Debug.Assert(pack.target_component == (byte)(byte)216);
                Debug.Assert(pack.end_index == (short)(short)12892);
            };
            MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
            PH.setPack(p38);
            p38.target_component = (byte)(byte)216;
            p38.end_index = (short)(short)12892;
            p38.target_system = (byte)(byte)212;
            p38.start_index = (short)(short) -23329;
            p38.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            SMP_TEST_CH.send(p38);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
                Debug.Assert(pack.z == (float)5.9628885E37F);
                Debug.Assert(pack.autocontinue == (byte)(byte)215);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_DO_LAND_START);
                Debug.Assert(pack.target_system == (byte)(byte)2);
                Debug.Assert(pack.current == (byte)(byte)88);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.seq == (ushort)(ushort)55916);
                Debug.Assert(pack.x == (float)1.1962221E38F);
                Debug.Assert(pack.param3 == (float)4.1105196E36F);
                Debug.Assert(pack.y == (float) -1.7869038E38F);
                Debug.Assert(pack.param1 == (float) -2.2788273E37F);
                Debug.Assert(pack.target_component == (byte)(byte)231);
                Debug.Assert(pack.param4 == (float)6.830498E37F);
                Debug.Assert(pack.param2 == (float)2.7476284E38F);
            };
            MISSION_ITEM p39 = new MISSION_ITEM();
            PH.setPack(p39);
            p39.y = (float) -1.7869038E38F;
            p39.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p39.command = (MAV_CMD)MAV_CMD.MAV_CMD_DO_LAND_START;
            p39.param4 = (float)6.830498E37F;
            p39.autocontinue = (byte)(byte)215;
            p39.param2 = (float)2.7476284E38F;
            p39.param3 = (float)4.1105196E36F;
            p39.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p39.target_system = (byte)(byte)2;
            p39.target_component = (byte)(byte)231;
            p39.param1 = (float) -2.2788273E37F;
            p39.x = (float)1.1962221E38F;
            p39.current = (byte)(byte)88;
            p39.seq = (ushort)(ushort)55916;
            p39.z = (float)5.9628885E37F;
            SMP_TEST_CH.send(p39);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)62872);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_component == (byte)(byte)62);
                Debug.Assert(pack.target_system == (byte)(byte)87);
            };
            MISSION_REQUEST p40 = new MISSION_REQUEST();
            PH.setPack(p40);
            p40.seq = (ushort)(ushort)62872;
            p40.target_system = (byte)(byte)87;
            p40.target_component = (byte)(byte)62;
            p40.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            SMP_TEST_CH.send(p40);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_SET_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)74);
                Debug.Assert(pack.seq == (ushort)(ushort)60378);
                Debug.Assert(pack.target_component == (byte)(byte)245);
            };
            MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
            PH.setPack(p41);
            p41.target_component = (byte)(byte)245;
            p41.target_system = (byte)(byte)74;
            p41.seq = (ushort)(ushort)60378;
            SMP_TEST_CH.send(p41);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)50596);
            };
            MISSION_CURRENT p42 = new MISSION_CURRENT();
            PH.setPack(p42);
            p42.seq = (ushort)(ushort)50596;
            SMP_TEST_CH.send(p42);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)173);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_system == (byte)(byte)116);
            };
            MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
            PH.setPack(p43);
            p43.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p43.target_component = (byte)(byte)173;
            p43.target_system = (byte)(byte)116;
            SMP_TEST_CH.send(p43);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_COUNTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)109);
                Debug.Assert(pack.count == (ushort)(ushort)8981);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.target_system == (byte)(byte)120);
            };
            MISSION_COUNT p44 = new MISSION_COUNT();
            PH.setPack(p44);
            p44.target_system = (byte)(byte)120;
            p44.target_component = (byte)(byte)109;
            p44.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p44.count = (ushort)(ushort)8981;
            SMP_TEST_CH.send(p44);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CLEAR_ALLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_component == (byte)(byte)251);
                Debug.Assert(pack.target_system == (byte)(byte)3);
            };
            MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
            PH.setPack(p45);
            p45.target_component = (byte)(byte)251;
            p45.target_system = (byte)(byte)3;
            p45.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            SMP_TEST_CH.send(p45);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_REACHEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)35458);
            };
            MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
            PH.setPack(p46);
            p46.seq = (ushort)(ushort)35458;
            SMP_TEST_CH.send(p46);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type == (MAV_MISSION_RESULT)MAV_MISSION_RESULT.MAV_MISSION_DENIED);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_component == (byte)(byte)60);
                Debug.Assert(pack.target_system == (byte)(byte)194);
            };
            MISSION_ACK p47 = new MISSION_ACK();
            PH.setPack(p47);
            p47.target_system = (byte)(byte)194;
            p47.target_component = (byte)(byte)60;
            p47.type = (MAV_MISSION_RESULT)MAV_MISSION_RESULT.MAV_MISSION_DENIED;
            p47.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            SMP_TEST_CH.send(p47);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_GPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude == (int)843782301);
                Debug.Assert(pack.longitude == (int) -1808125781);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)42075003472291172L);
                Debug.Assert(pack.latitude == (int)1759467038);
                Debug.Assert(pack.target_system == (byte)(byte)56);
            };
            SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
            PH.setPack(p48);
            p48.time_usec_SET((ulong)42075003472291172L, PH) ;
            p48.longitude = (int) -1808125781;
            p48.target_system = (byte)(byte)56;
            p48.altitude = (int)843782301;
            p48.latitude = (int)1759467038;
            SMP_TEST_CH.send(p48);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.longitude == (int)1302290973);
                Debug.Assert(pack.altitude == (int)729449588);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)4462011208708816168L);
                Debug.Assert(pack.latitude == (int)1553349651);
            };
            GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
            PH.setPack(p49);
            p49.altitude = (int)729449588;
            p49.time_usec_SET((ulong)4462011208708816168L, PH) ;
            p49.latitude = (int)1553349651;
            p49.longitude = (int)1302290973;
            SMP_TEST_CH.send(p49);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_MAP_RCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)96);
                Debug.Assert(pack.param_id_LEN(ph) == 2);
                Debug.Assert(pack.param_id_TRY(ph).Equals("Zg"));
                Debug.Assert(pack.param_index == (short)(short)15057);
                Debug.Assert(pack.param_value_max == (float)1.0989611E38F);
                Debug.Assert(pack.param_value0 == (float)2.1921455E38F);
                Debug.Assert(pack.target_system == (byte)(byte)133);
                Debug.Assert(pack.param_value_min == (float) -4.8617174E37F);
                Debug.Assert(pack.parameter_rc_channel_index == (byte)(byte)7);
                Debug.Assert(pack.scale == (float)2.5260796E38F);
            };
            PARAM_MAP_RC p50 = new PARAM_MAP_RC();
            PH.setPack(p50);
            p50.target_component = (byte)(byte)96;
            p50.target_system = (byte)(byte)133;
            p50.param_value_min = (float) -4.8617174E37F;
            p50.param_value_max = (float)1.0989611E38F;
            p50.param_index = (short)(short)15057;
            p50.parameter_rc_channel_index = (byte)(byte)7;
            p50.param_value0 = (float)2.1921455E38F;
            p50.param_id_SET("Zg", PH) ;
            p50.scale = (float)2.5260796E38F;
            SMP_TEST_CH.send(p50);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)98);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.seq == (ushort)(ushort)209);
                Debug.Assert(pack.target_system == (byte)(byte)193);
            };
            MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
            PH.setPack(p51);
            p51.seq = (ushort)(ushort)209;
            p51.target_system = (byte)(byte)193;
            p51.target_component = (byte)(byte)98;
            p51.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            SMP_TEST_CH.send(p51);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_SET_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p2x == (float) -2.0833505E38F);
                Debug.Assert(pack.p2z == (float)1.762528E38F);
                Debug.Assert(pack.target_system == (byte)(byte)72);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
                Debug.Assert(pack.p2y == (float) -1.4763574E38F);
                Debug.Assert(pack.p1y == (float)1.9156659E38F);
                Debug.Assert(pack.p1x == (float)2.602667E38F);
                Debug.Assert(pack.p1z == (float)2.1766372E38F);
                Debug.Assert(pack.target_component == (byte)(byte)183);
            };
            SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
            PH.setPack(p54);
            p54.p2z = (float)1.762528E38F;
            p54.p1y = (float)1.9156659E38F;
            p54.p2y = (float) -1.4763574E38F;
            p54.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
            p54.target_component = (byte)(byte)183;
            p54.p2x = (float) -2.0833505E38F;
            p54.target_system = (byte)(byte)72;
            p54.p1x = (float)2.602667E38F;
            p54.p1z = (float)2.1766372E38F;
            SMP_TEST_CH.send(p54);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p1y == (float) -5.4635604E37F);
                Debug.Assert(pack.p2y == (float) -2.7899205E38F);
                Debug.Assert(pack.p1x == (float) -6.645724E37F);
                Debug.Assert(pack.p2z == (float)2.4024634E38F);
                Debug.Assert(pack.p2x == (float)3.034126E38F);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
                Debug.Assert(pack.p1z == (float) -2.2720107E37F);
            };
            SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
            PH.setPack(p55);
            p55.p2z = (float)2.4024634E38F;
            p55.p2x = (float)3.034126E38F;
            p55.p1z = (float) -2.2720107E37F;
            p55.p1x = (float) -6.645724E37F;
            p55.p1y = (float) -5.4635604E37F;
            p55.p2y = (float) -2.7899205E38F;
            p55.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT;
            SMP_TEST_CH.send(p55);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNION_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)8843789499261140964L);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {2.402355E38F, 3.905716E37F, -3.334479E38F, 2.851424E38F, 2.3576088E37F, -3.1984516E38F, 4.0091626E37F, 1.7244496E38F, -1.7272348E38F}));
                Debug.Assert(pack.pitchspeed == (float)2.3934401E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {9.317758E37F, -1.5464248E38F, -3.2753771E38F, -1.4861612E38F}));
                Debug.Assert(pack.yawspeed == (float)2.90245E38F);
                Debug.Assert(pack.rollspeed == (float) -3.2663711E38F);
            };
            ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
            PH.setPack(p61);
            p61.time_usec = (ulong)8843789499261140964L;
            p61.q_SET(new float[] {9.317758E37F, -1.5464248E38F, -3.2753771E38F, -1.4861612E38F}, 0) ;
            p61.covariance_SET(new float[] {2.402355E38F, 3.905716E37F, -3.334479E38F, 2.851424E38F, 2.3576088E37F, -3.1984516E38F, 4.0091626E37F, 1.7244496E38F, -1.7272348E38F}, 0) ;
            p61.yawspeed = (float)2.90245E38F;
            p61.pitchspeed = (float)2.3934401E38F;
            p61.rollspeed = (float) -3.2663711E38F;
            SMP_TEST_CH.send(p61);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnNAV_CONTROLLER_OUTPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.alt_error == (float) -2.2378735E38F);
                Debug.Assert(pack.wp_dist == (ushort)(ushort)40999);
                Debug.Assert(pack.nav_roll == (float) -2.1815727E38F);
                Debug.Assert(pack.nav_pitch == (float)9.698602E37F);
                Debug.Assert(pack.target_bearing == (short)(short)2925);
                Debug.Assert(pack.aspd_error == (float)3.2603489E38F);
                Debug.Assert(pack.xtrack_error == (float) -2.1276313E38F);
                Debug.Assert(pack.nav_bearing == (short)(short) -17572);
            };
            NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
            PH.setPack(p62);
            p62.nav_bearing = (short)(short) -17572;
            p62.xtrack_error = (float) -2.1276313E38F;
            p62.aspd_error = (float)3.2603489E38F;
            p62.nav_roll = (float) -2.1815727E38F;
            p62.target_bearing = (short)(short)2925;
            p62.nav_pitch = (float)9.698602E37F;
            p62.wp_dist = (ushort)(ushort)40999;
            p62.alt_error = (float) -2.2378735E38F;
            SMP_TEST_CH.send(p62);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INT_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.alt == (int)1695980313);
                Debug.Assert(pack.lon == (int)830217058);
                Debug.Assert(pack.lat == (int) -1346717434);
                Debug.Assert(pack.estimator_type == (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS);
                Debug.Assert(pack.vy == (float)3.2442037E38F);
                Debug.Assert(pack.relative_alt == (int)645846418);
                Debug.Assert(pack.vz == (float)3.2756838E38F);
                Debug.Assert(pack.vx == (float)2.024013E38F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {1.1644397E38F, -1.4726721E38F, 3.238253E38F, 1.9846283E38F, 2.1693771E38F, -3.0140713E38F, -1.785943E38F, 1.3830967E38F, 2.7084028E38F, 3.0752966E38F, -1.9752373E38F, 4.2714415E37F, -2.9644385E38F, 1.2081465E38F, -2.9680215E38F, 1.758098E38F, -2.808432E38F, 2.4943297E37F, -8.263972E36F, -1.2187213E38F, 2.8972617E38F, -2.570223E38F, 2.2922802E38F, 5.5502667E37F, 6.7378155E37F, 1.223826E37F, 3.0324557E38F, -3.1769192E38F, 3.318073E38F, 3.210738E38F, 1.4655406E38F, 1.8826867E38F, -4.523318E37F, -1.7916704E38F, 1.2166872E38F, 7.382863E37F}));
                Debug.Assert(pack.time_usec == (ulong)768039211602621607L);
            };
            GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
            PH.setPack(p63);
            p63.time_usec = (ulong)768039211602621607L;
            p63.covariance_SET(new float[] {1.1644397E38F, -1.4726721E38F, 3.238253E38F, 1.9846283E38F, 2.1693771E38F, -3.0140713E38F, -1.785943E38F, 1.3830967E38F, 2.7084028E38F, 3.0752966E38F, -1.9752373E38F, 4.2714415E37F, -2.9644385E38F, 1.2081465E38F, -2.9680215E38F, 1.758098E38F, -2.808432E38F, 2.4943297E37F, -8.263972E36F, -1.2187213E38F, 2.8972617E38F, -2.570223E38F, 2.2922802E38F, 5.5502667E37F, 6.7378155E37F, 1.223826E37F, 3.0324557E38F, -3.1769192E38F, 3.318073E38F, 3.210738E38F, 1.4655406E38F, 1.8826867E38F, -4.523318E37F, -1.7916704E38F, 1.2166872E38F, 7.382863E37F}, 0) ;
            p63.vy = (float)3.2442037E38F;
            p63.alt = (int)1695980313;
            p63.vx = (float)2.024013E38F;
            p63.relative_alt = (int)645846418;
            p63.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS;
            p63.lat = (int) -1346717434;
            p63.vz = (float)3.2756838E38F;
            p63.lon = (int)830217058;
            SMP_TEST_CH.send(p63);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-2.8256165E38F, -4.8656173E36F, 2.3596724E38F, -1.2393455E38F, -1.7173631E38F, -3.2793912E38F, 7.8386356E37F, -4.2536964E37F, 2.1562785E38F, -2.3469342E38F, -2.4085772E37F, -9.233773E37F, 2.2505352E38F, -1.9879118E38F, -2.0611404E38F, 6.929875E37F, 2.451113E38F, 2.4251452E38F, -2.7499524E38F, 2.6666592E38F, -1.5649157E38F, 1.916492E38F, 3.1583133E38F, -2.7872323E37F, -2.6179362E38F, -3.3129207E38F, 1.5247961E38F, -3.271644E38F, 2.3189958E38F, -8.962999E37F, -3.3139825E38F, 2.3198164E38F, -1.4640312E38F, -3.2281073E38F, -5.0344744E37F, 2.38883E37F, 1.7005826E38F, -2.3015166E38F, -2.8005905E38F, -1.3152113E38F, -5.408253E37F, -2.9967865E38F, -9.746353E37F, 3.0544834E38F, -2.083471E38F}));
                Debug.Assert(pack.vx == (float)2.0196193E38F);
                Debug.Assert(pack.vz == (float)2.0735855E38F);
                Debug.Assert(pack.ax == (float)4.90411E37F);
                Debug.Assert(pack.ay == (float) -3.3531435E38F);
                Debug.Assert(pack.estimator_type == (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION);
                Debug.Assert(pack.y == (float)2.7904684E37F);
                Debug.Assert(pack.az == (float)8.92318E37F);
                Debug.Assert(pack.vy == (float) -2.9413857E38F);
                Debug.Assert(pack.z == (float) -2.9971526E38F);
                Debug.Assert(pack.time_usec == (ulong)896119104339022129L);
                Debug.Assert(pack.x == (float)1.0931399E38F);
            };
            LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
            PH.setPack(p64);
            p64.vy = (float) -2.9413857E38F;
            p64.x = (float)1.0931399E38F;
            p64.ax = (float)4.90411E37F;
            p64.vz = (float)2.0735855E38F;
            p64.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION;
            p64.time_usec = (ulong)896119104339022129L;
            p64.y = (float)2.7904684E37F;
            p64.covariance_SET(new float[] {-2.8256165E38F, -4.8656173E36F, 2.3596724E38F, -1.2393455E38F, -1.7173631E38F, -3.2793912E38F, 7.8386356E37F, -4.2536964E37F, 2.1562785E38F, -2.3469342E38F, -2.4085772E37F, -9.233773E37F, 2.2505352E38F, -1.9879118E38F, -2.0611404E38F, 6.929875E37F, 2.451113E38F, 2.4251452E38F, -2.7499524E38F, 2.6666592E38F, -1.5649157E38F, 1.916492E38F, 3.1583133E38F, -2.7872323E37F, -2.6179362E38F, -3.3129207E38F, 1.5247961E38F, -3.271644E38F, 2.3189958E38F, -8.962999E37F, -3.3139825E38F, 2.3198164E38F, -1.4640312E38F, -3.2281073E38F, -5.0344744E37F, 2.38883E37F, 1.7005826E38F, -2.3015166E38F, -2.8005905E38F, -1.3152113E38F, -5.408253E37F, -2.9967865E38F, -9.746353E37F, 3.0544834E38F, -2.083471E38F}, 0) ;
            p64.z = (float) -2.9971526E38F;
            p64.vx = (float)2.0196193E38F;
            p64.ay = (float) -3.3531435E38F;
            p64.az = (float)8.92318E37F;
            SMP_TEST_CH.send(p64);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)58844);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)26837);
                Debug.Assert(pack.chan15_raw == (ushort)(ushort)51966);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)64892);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)6402);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)42414);
                Debug.Assert(pack.chan16_raw == (ushort)(ushort)41895);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)28376);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)2529);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)58983);
                Debug.Assert(pack.time_boot_ms == (uint)1898585729U);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)6977);
                Debug.Assert(pack.chan18_raw == (ushort)(ushort)3657);
                Debug.Assert(pack.chan14_raw == (ushort)(ushort)5218);
                Debug.Assert(pack.chan13_raw == (ushort)(ushort)20465);
                Debug.Assert(pack.rssi == (byte)(byte)140);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)18324);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)4440);
                Debug.Assert(pack.chancount == (byte)(byte)194);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)52652);
                Debug.Assert(pack.chan17_raw == (ushort)(ushort)51761);
            };
            RC_CHANNELS p65 = new RC_CHANNELS();
            PH.setPack(p65);
            p65.chan10_raw = (ushort)(ushort)4440;
            p65.chan16_raw = (ushort)(ushort)41895;
            p65.chan2_raw = (ushort)(ushort)6977;
            p65.chan11_raw = (ushort)(ushort)58983;
            p65.chan4_raw = (ushort)(ushort)64892;
            p65.chan14_raw = (ushort)(ushort)5218;
            p65.chan3_raw = (ushort)(ushort)58844;
            p65.chan15_raw = (ushort)(ushort)51966;
            p65.chan6_raw = (ushort)(ushort)6402;
            p65.rssi = (byte)(byte)140;
            p65.chan18_raw = (ushort)(ushort)3657;
            p65.chan12_raw = (ushort)(ushort)26837;
            p65.chan9_raw = (ushort)(ushort)2529;
            p65.time_boot_ms = (uint)1898585729U;
            p65.chan8_raw = (ushort)(ushort)28376;
            p65.chancount = (byte)(byte)194;
            p65.chan13_raw = (ushort)(ushort)20465;
            p65.chan7_raw = (ushort)(ushort)42414;
            p65.chan17_raw = (ushort)(ushort)51761;
            p65.chan1_raw = (ushort)(ushort)18324;
            p65.chan5_raw = (ushort)(ushort)52652;
            SMP_TEST_CH.send(p65);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnREQUEST_DATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start_stop == (byte)(byte)161);
                Debug.Assert(pack.target_system == (byte)(byte)151);
                Debug.Assert(pack.req_message_rate == (ushort)(ushort)41367);
                Debug.Assert(pack.req_stream_id == (byte)(byte)73);
                Debug.Assert(pack.target_component == (byte)(byte)23);
            };
            REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
            PH.setPack(p66);
            p66.req_message_rate = (ushort)(ushort)41367;
            p66.start_stop = (byte)(byte)161;
            p66.target_component = (byte)(byte)23;
            p66.req_stream_id = (byte)(byte)73;
            p66.target_system = (byte)(byte)151;
            SMP_TEST_CH.send(p66);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.message_rate == (ushort)(ushort)50556);
                Debug.Assert(pack.stream_id == (byte)(byte)207);
                Debug.Assert(pack.on_off == (byte)(byte)128);
            };
            DATA_STREAM p67 = new DATA_STREAM();
            PH.setPack(p67);
            p67.message_rate = (ushort)(ushort)50556;
            p67.on_off = (byte)(byte)128;
            p67.stream_id = (byte)(byte)207;
            SMP_TEST_CH.send(p67);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.buttons == (ushort)(ushort)22219);
                Debug.Assert(pack.x == (short)(short)10936);
                Debug.Assert(pack.y == (short)(short)21928);
                Debug.Assert(pack.z == (short)(short)1959);
                Debug.Assert(pack.r == (short)(short)1948);
                Debug.Assert(pack.target == (byte)(byte)225);
            };
            MANUAL_CONTROL p69 = new MANUAL_CONTROL();
            PH.setPack(p69);
            p69.y = (short)(short)21928;
            p69.r = (short)(short)1948;
            p69.buttons = (ushort)(ushort)22219;
            p69.target = (byte)(byte)225;
            p69.z = (short)(short)1959;
            p69.x = (short)(short)10936;
            SMP_TEST_CH.send(p69);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_OVERRIDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)225);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)13011);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)5732);
                Debug.Assert(pack.target_component == (byte)(byte)132);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)8826);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)36969);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)9141);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)33520);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)4220);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)62950);
            };
            RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
            PH.setPack(p70);
            p70.chan8_raw = (ushort)(ushort)9141;
            p70.chan7_raw = (ushort)(ushort)8826;
            p70.chan6_raw = (ushort)(ushort)13011;
            p70.chan3_raw = (ushort)(ushort)33520;
            p70.chan5_raw = (ushort)(ushort)36969;
            p70.target_system = (byte)(byte)225;
            p70.target_component = (byte)(byte)132;
            p70.chan2_raw = (ushort)(ushort)62950;
            p70.chan4_raw = (ushort)(ushort)4220;
            p70.chan1_raw = (ushort)(ushort)5732;
            SMP_TEST_CH.send(p70);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (int) -767929857);
                Debug.Assert(pack.target_system == (byte)(byte)122);
                Debug.Assert(pack.autocontinue == (byte)(byte)173);
                Debug.Assert(pack.param1 == (float) -3.2338286E38F);
                Debug.Assert(pack.param3 == (float)3.1839469E38F);
                Debug.Assert(pack.current == (byte)(byte)5);
                Debug.Assert(pack.param2 == (float) -5.8281276E37F);
                Debug.Assert(pack.seq == (ushort)(ushort)34917);
                Debug.Assert(pack.target_component == (byte)(byte)25);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_MISSION);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.z == (float) -2.8103128E38F);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH);
                Debug.Assert(pack.x == (int)2100687653);
                Debug.Assert(pack.param4 == (float) -2.6522692E38F);
            };
            MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
            PH.setPack(p73);
            p73.current = (byte)(byte)5;
            p73.target_component = (byte)(byte)25;
            p73.seq = (ushort)(ushort)34917;
            p73.param4 = (float) -2.6522692E38F;
            p73.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p73.target_system = (byte)(byte)122;
            p73.z = (float) -2.8103128E38F;
            p73.command = (MAV_CMD)MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH;
            p73.param3 = (float)3.1839469E38F;
            p73.param2 = (float) -5.8281276E37F;
            p73.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_MISSION;
            p73.param1 = (float) -3.2338286E38F;
            p73.x = (int)2100687653;
            p73.autocontinue = (byte)(byte)173;
            p73.y = (int) -767929857;
            SMP_TEST_CH.send(p73);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVFR_HUDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.climb == (float)1.1107907E38F);
                Debug.Assert(pack.alt == (float)1.0072357E38F);
                Debug.Assert(pack.groundspeed == (float) -3.5100196E37F);
                Debug.Assert(pack.heading == (short)(short)13313);
                Debug.Assert(pack.throttle == (ushort)(ushort)19079);
                Debug.Assert(pack.airspeed == (float) -3.6755694E37F);
            };
            VFR_HUD p74 = new VFR_HUD();
            PH.setPack(p74);
            p74.heading = (short)(short)13313;
            p74.groundspeed = (float) -3.5100196E37F;
            p74.throttle = (ushort)(ushort)19079;
            p74.airspeed = (float) -3.6755694E37F;
            p74.climb = (float)1.1107907E38F;
            p74.alt = (float)1.0072357E38F;
            SMP_TEST_CH.send(p74);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)0);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
                Debug.Assert(pack.target_system == (byte)(byte)60);
                Debug.Assert(pack.current == (byte)(byte)226);
                Debug.Assert(pack.z == (float) -3.154939E38F);
                Debug.Assert(pack.autocontinue == (byte)(byte)249);
                Debug.Assert(pack.y == (int) -1781476196);
                Debug.Assert(pack.param4 == (float)3.361981E38F);
                Debug.Assert(pack.param1 == (float) -2.5637945E38F);
                Debug.Assert(pack.param2 == (float) -2.6182313E38F);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_LOGGING_START);
                Debug.Assert(pack.x == (int) -150431363);
                Debug.Assert(pack.param3 == (float) -4.1039937E37F);
            };
            COMMAND_INT p75 = new COMMAND_INT();
            PH.setPack(p75);
            p75.target_system = (byte)(byte)60;
            p75.current = (byte)(byte)226;
            p75.y = (int) -1781476196;
            p75.x = (int) -150431363;
            p75.param1 = (float) -2.5637945E38F;
            p75.param4 = (float)3.361981E38F;
            p75.autocontinue = (byte)(byte)249;
            p75.command = (MAV_CMD)MAV_CMD.MAV_CMD_LOGGING_START;
            p75.target_component = (byte)(byte)0;
            p75.param2 = (float) -2.6182313E38F;
            p75.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p75.z = (float) -3.154939E38F;
            p75.param3 = (float) -4.1039937E37F;
            SMP_TEST_CH.send(p75);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_LONGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.confirmation == (byte)(byte)239);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_SET_FACTORY_TEST_MODE);
                Debug.Assert(pack.param3 == (float) -2.9573575E38F);
                Debug.Assert(pack.param5 == (float) -3.2365516E37F);
                Debug.Assert(pack.target_component == (byte)(byte)255);
                Debug.Assert(pack.param6 == (float) -2.4275685E38F);
                Debug.Assert(pack.target_system == (byte)(byte)160);
                Debug.Assert(pack.param4 == (float) -1.2253213E38F);
                Debug.Assert(pack.param1 == (float) -1.3214321E38F);
                Debug.Assert(pack.param7 == (float) -5.009879E37F);
                Debug.Assert(pack.param2 == (float) -2.5925073E38F);
            };
            COMMAND_LONG p76 = new COMMAND_LONG();
            PH.setPack(p76);
            p76.target_component = (byte)(byte)255;
            p76.param2 = (float) -2.5925073E38F;
            p76.confirmation = (byte)(byte)239;
            p76.target_system = (byte)(byte)160;
            p76.param5 = (float) -3.2365516E37F;
            p76.param7 = (float) -5.009879E37F;
            p76.param4 = (float) -1.2253213E38F;
            p76.param3 = (float) -2.9573575E38F;
            p76.param6 = (float) -2.4275685E38F;
            p76.command = (MAV_CMD)MAV_CMD.MAV_CMD_SET_FACTORY_TEST_MODE;
            p76.param1 = (float) -1.3214321E38F;
            SMP_TEST_CH.send(p76);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.result_param2_TRY(ph) == (int) -522851401);
                Debug.Assert(pack.result == (MAV_RESULT)MAV_RESULT.MAV_RESULT_DENIED);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL);
                Debug.Assert(pack.target_component_TRY(ph) == (byte)(byte)37);
                Debug.Assert(pack.progress_TRY(ph) == (byte)(byte)173);
                Debug.Assert(pack.target_system_TRY(ph) == (byte)(byte)95);
            };
            COMMAND_ACK p77 = new COMMAND_ACK();
            PH.setPack(p77);
            p77.command = (MAV_CMD)MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL;
            p77.result_param2_SET((int) -522851401, PH) ;
            p77.result = (MAV_RESULT)MAV_RESULT.MAV_RESULT_DENIED;
            p77.target_system_SET((byte)(byte)95, PH) ;
            p77.target_component_SET((byte)(byte)37, PH) ;
            p77.progress_SET((byte)(byte)173, PH) ;
            SMP_TEST_CH.send(p77);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_SETPOINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mode_switch == (byte)(byte)49);
                Debug.Assert(pack.manual_override_switch == (byte)(byte)48);
                Debug.Assert(pack.yaw == (float) -5.1592614E37F);
                Debug.Assert(pack.thrust == (float) -1.8292987E38F);
                Debug.Assert(pack.roll == (float) -1.2413088E37F);
                Debug.Assert(pack.pitch == (float)1.9176033E38F);
                Debug.Assert(pack.time_boot_ms == (uint)340557176U);
            };
            MANUAL_SETPOINT p81 = new MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.pitch = (float)1.9176033E38F;
            p81.thrust = (float) -1.8292987E38F;
            p81.roll = (float) -1.2413088E37F;
            p81.mode_switch = (byte)(byte)49;
            p81.manual_override_switch = (byte)(byte)48;
            p81.yaw = (float) -5.1592614E37F;
            p81.time_boot_ms = (uint)340557176U;
            SMP_TEST_CH.send(p81);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type_mask == (byte)(byte)4);
                Debug.Assert(pack.thrust == (float) -7.428689E37F);
                Debug.Assert(pack.body_pitch_rate == (float)8.786126E37F);
                Debug.Assert(pack.body_roll_rate == (float) -2.8736152E38F);
                Debug.Assert(pack.body_yaw_rate == (float)3.8168006E37F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.6997667E38F, 1.2441732E38F, 1.5397186E37F, -1.5373801E38F}));
                Debug.Assert(pack.target_component == (byte)(byte)4);
                Debug.Assert(pack.target_system == (byte)(byte)85);
                Debug.Assert(pack.time_boot_ms == (uint)150505488U);
            };
            SET_ATTITUDE_TARGET p82 = new SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.target_system = (byte)(byte)85;
            p82.body_roll_rate = (float) -2.8736152E38F;
            p82.target_component = (byte)(byte)4;
            p82.time_boot_ms = (uint)150505488U;
            p82.body_pitch_rate = (float)8.786126E37F;
            p82.type_mask = (byte)(byte)4;
            p82.q_SET(new float[] {-1.6997667E38F, 1.2441732E38F, 1.5397186E37F, -1.5373801E38F}, 0) ;
            p82.body_yaw_rate = (float)3.8168006E37F;
            p82.thrust = (float) -7.428689E37F;
            SMP_TEST_CH.send(p82);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type_mask == (byte)(byte)172);
                Debug.Assert(pack.q.SequenceEqual(new float[] {3.3555135E38F, 2.7140752E37F, -2.71274E38F, 3.2321764E38F}));
                Debug.Assert(pack.body_roll_rate == (float)3.9925244E37F);
                Debug.Assert(pack.body_yaw_rate == (float) -1.9980354E38F);
                Debug.Assert(pack.thrust == (float) -1.8784034E38F);
                Debug.Assert(pack.body_pitch_rate == (float)9.376402E37F);
                Debug.Assert(pack.time_boot_ms == (uint)1139723763U);
            };
            ATTITUDE_TARGET p83 = new ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.thrust = (float) -1.8784034E38F;
            p83.body_roll_rate = (float)3.9925244E37F;
            p83.body_pitch_rate = (float)9.376402E37F;
            p83.time_boot_ms = (uint)1139723763U;
            p83.type_mask = (byte)(byte)172;
            p83.body_yaw_rate = (float) -1.9980354E38F;
            p83.q_SET(new float[] {3.3555135E38F, 2.7140752E37F, -2.71274E38F, 3.2321764E38F}, 0) ;
            SMP_TEST_CH.send(p83);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)37);
                Debug.Assert(pack.time_boot_ms == (uint)1202656176U);
                Debug.Assert(pack.y == (float)1.3864746E38F);
                Debug.Assert(pack.z == (float)8.2801416E37F);
                Debug.Assert(pack.afx == (float) -2.103586E37F);
                Debug.Assert(pack.vx == (float) -2.3252797E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)14216);
                Debug.Assert(pack.afy == (float)1.8990737E38F);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
                Debug.Assert(pack.yaw == (float)1.6366751E38F);
                Debug.Assert(pack.x == (float)4.550278E37F);
                Debug.Assert(pack.vz == (float) -9.712512E35F);
                Debug.Assert(pack.yaw_rate == (float) -1.3598152E38F);
                Debug.Assert(pack.target_component == (byte)(byte)244);
                Debug.Assert(pack.afz == (float) -3.268137E38F);
                Debug.Assert(pack.vy == (float)2.2181886E38F);
            };
            SET_POSITION_TARGET_LOCAL_NED p84 = new SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p84.z = (float)8.2801416E37F;
            p84.y = (float)1.3864746E38F;
            p84.target_system = (byte)(byte)37;
            p84.vy = (float)2.2181886E38F;
            p84.afz = (float) -3.268137E38F;
            p84.yaw_rate = (float) -1.3598152E38F;
            p84.vz = (float) -9.712512E35F;
            p84.x = (float)4.550278E37F;
            p84.vx = (float) -2.3252797E38F;
            p84.time_boot_ms = (uint)1202656176U;
            p84.type_mask = (ushort)(ushort)14216;
            p84.afx = (float) -2.103586E37F;
            p84.afy = (float)1.8990737E38F;
            p84.target_component = (byte)(byte)244;
            p84.yaw = (float)1.6366751E38F;
            SMP_TEST_CH.send(p84);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float) -6.080819E37F);
                Debug.Assert(pack.afz == (float)4.80343E37F);
                Debug.Assert(pack.time_boot_ms == (uint)4274129667U);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_MISSION);
                Debug.Assert(pack.afx == (float)3.3606604E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)44959);
                Debug.Assert(pack.lat_int == (int)679031422);
                Debug.Assert(pack.vz == (float) -8.703779E37F);
                Debug.Assert(pack.yaw_rate == (float) -1.7362363E38F);
                Debug.Assert(pack.vx == (float) -1.4726112E38F);
                Debug.Assert(pack.target_system == (byte)(byte)101);
                Debug.Assert(pack.target_component == (byte)(byte)64);
                Debug.Assert(pack.vy == (float) -7.2143684E37F);
                Debug.Assert(pack.afy == (float) -5.8444017E37F);
                Debug.Assert(pack.alt == (float)1.7389138E38F);
                Debug.Assert(pack.lon_int == (int)97111964);
            };
            SET_POSITION_TARGET_GLOBAL_INT p86 = new SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.lon_int = (int)97111964;
            p86.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_MISSION;
            p86.yaw = (float) -6.080819E37F;
            p86.afx = (float)3.3606604E38F;
            p86.vx = (float) -1.4726112E38F;
            p86.type_mask = (ushort)(ushort)44959;
            p86.afy = (float) -5.8444017E37F;
            p86.vy = (float) -7.2143684E37F;
            p86.alt = (float)1.7389138E38F;
            p86.vz = (float) -8.703779E37F;
            p86.afz = (float)4.80343E37F;
            p86.lat_int = (int)679031422;
            p86.target_system = (byte)(byte)101;
            p86.target_component = (byte)(byte)64;
            p86.time_boot_ms = (uint)4274129667U;
            p86.yaw_rate = (float) -1.7362363E38F;
            SMP_TEST_CH.send(p86);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.afx == (float)2.9593782E38F);
                Debug.Assert(pack.lon_int == (int) -1911506159);
                Debug.Assert(pack.afy == (float)2.1255807E38F);
                Debug.Assert(pack.vz == (float)2.894765E38F);
                Debug.Assert(pack.alt == (float) -1.5620235E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)3230);
                Debug.Assert(pack.vx == (float) -3.0907114E37F);
                Debug.Assert(pack.vy == (float) -8.4328616E37F);
                Debug.Assert(pack.lat_int == (int)260437593);
                Debug.Assert(pack.time_boot_ms == (uint)1402985618U);
                Debug.Assert(pack.yaw_rate == (float) -3.3237551E38F);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_INT);
                Debug.Assert(pack.yaw == (float)1.3153838E38F);
                Debug.Assert(pack.afz == (float) -1.418454E38F);
            };
            POSITION_TARGET_GLOBAL_INT p87 = new POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.vz = (float)2.894765E38F;
            p87.lat_int = (int)260437593;
            p87.alt = (float) -1.5620235E38F;
            p87.afz = (float) -1.418454E38F;
            p87.yaw_rate = (float) -3.3237551E38F;
            p87.lon_int = (int) -1911506159;
            p87.time_boot_ms = (uint)1402985618U;
            p87.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_INT;
            p87.afy = (float)2.1255807E38F;
            p87.type_mask = (ushort)(ushort)3230;
            p87.vy = (float) -8.4328616E37F;
            p87.yaw = (float)1.3153838E38F;
            p87.vx = (float) -3.0907114E37F;
            p87.afx = (float)2.9593782E38F;
            SMP_TEST_CH.send(p87);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float)1.7218245E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1370565506U);
                Debug.Assert(pack.yaw == (float) -8.409596E37F);
                Debug.Assert(pack.pitch == (float) -3.1090977E38F);
                Debug.Assert(pack.roll == (float)2.210856E38F);
                Debug.Assert(pack.x == (float) -2.654567E38F);
                Debug.Assert(pack.y == (float) -2.607024E38F);
            };
            LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = new LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.z = (float)1.7218245E38F;
            p89.y = (float) -2.607024E38F;
            p89.yaw = (float) -8.409596E37F;
            p89.pitch = (float) -3.1090977E38F;
            p89.roll = (float)2.210856E38F;
            p89.time_boot_ms = (uint)1370565506U;
            p89.x = (float) -2.654567E38F;
            SMP_TEST_CH.send(p89);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)6599399748396878503L);
                Debug.Assert(pack.yawspeed == (float)7.1452855E37F);
                Debug.Assert(pack.lat == (int)2022932586);
                Debug.Assert(pack.pitchspeed == (float) -5.6143915E37F);
                Debug.Assert(pack.roll == (float) -3.2633217E38F);
                Debug.Assert(pack.alt == (int) -111860887);
                Debug.Assert(pack.yacc == (short)(short)24545);
                Debug.Assert(pack.lon == (int)1087968942);
                Debug.Assert(pack.pitch == (float)9.592457E37F);
                Debug.Assert(pack.vy == (short)(short)8810);
                Debug.Assert(pack.rollspeed == (float)1.3727314E38F);
                Debug.Assert(pack.vz == (short)(short)3444);
                Debug.Assert(pack.xacc == (short)(short)3495);
                Debug.Assert(pack.zacc == (short)(short)11587);
                Debug.Assert(pack.yaw == (float)3.2079298E38F);
                Debug.Assert(pack.vx == (short)(short) -21667);
            };
            HIL_STATE p90 = new HIL_STATE();
            PH.setPack(p90);
            p90.yaw = (float)3.2079298E38F;
            p90.pitch = (float)9.592457E37F;
            p90.yacc = (short)(short)24545;
            p90.vx = (short)(short) -21667;
            p90.lat = (int)2022932586;
            p90.lon = (int)1087968942;
            p90.xacc = (short)(short)3495;
            p90.zacc = (short)(short)11587;
            p90.vz = (short)(short)3444;
            p90.time_usec = (ulong)6599399748396878503L;
            p90.vy = (short)(short)8810;
            p90.pitchspeed = (float) -5.6143915E37F;
            p90.yawspeed = (float)7.1452855E37F;
            p90.alt = (int) -111860887;
            p90.roll = (float) -3.2633217E38F;
            p90.rollspeed = (float)1.3727314E38F;
            SMP_TEST_CH.send(p90);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.throttle == (float)2.2509879E38F);
                Debug.Assert(pack.time_usec == (ulong)3301371802368413991L);
                Debug.Assert(pack.yaw_rudder == (float) -2.9103392E38F);
                Debug.Assert(pack.mode == (MAV_MODE)MAV_MODE.MAV_MODE_MANUAL_DISARMED);
                Debug.Assert(pack.pitch_elevator == (float)4.1126234E37F);
                Debug.Assert(pack.roll_ailerons == (float)3.0572309E38F);
                Debug.Assert(pack.aux4 == (float)8.4779154E37F);
                Debug.Assert(pack.aux3 == (float)1.5623478E38F);
                Debug.Assert(pack.nav_mode == (byte)(byte)197);
                Debug.Assert(pack.aux1 == (float)6.5958584E37F);
                Debug.Assert(pack.aux2 == (float)1.7553705E38F);
            };
            HIL_CONTROLS p91 = new HIL_CONTROLS();
            PH.setPack(p91);
            p91.aux3 = (float)1.5623478E38F;
            p91.roll_ailerons = (float)3.0572309E38F;
            p91.aux1 = (float)6.5958584E37F;
            p91.time_usec = (ulong)3301371802368413991L;
            p91.nav_mode = (byte)(byte)197;
            p91.aux4 = (float)8.4779154E37F;
            p91.yaw_rudder = (float) -2.9103392E38F;
            p91.aux2 = (float)1.7553705E38F;
            p91.pitch_elevator = (float)4.1126234E37F;
            p91.throttle = (float)2.2509879E38F;
            p91.mode = (MAV_MODE)MAV_MODE.MAV_MODE_MANUAL_DISARMED;
            SMP_TEST_CH.send(p91);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_RC_INPUTS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)28311);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)10493);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)27161);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)62321);
                Debug.Assert(pack.rssi == (byte)(byte)139);
                Debug.Assert(pack.time_usec == (ulong)8362690075102150310L);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)26607);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)39089);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)36078);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)54970);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)18907);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)53737);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)6957);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)49340);
            };
            HIL_RC_INPUTS_RAW p92 = new HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.chan12_raw = (ushort)(ushort)49340;
            p92.chan9_raw = (ushort)(ushort)54970;
            p92.time_usec = (ulong)8362690075102150310L;
            p92.chan7_raw = (ushort)(ushort)27161;
            p92.chan10_raw = (ushort)(ushort)18907;
            p92.chan3_raw = (ushort)(ushort)53737;
            p92.chan5_raw = (ushort)(ushort)6957;
            p92.chan8_raw = (ushort)(ushort)26607;
            p92.rssi = (byte)(byte)139;
            p92.chan2_raw = (ushort)(ushort)36078;
            p92.chan11_raw = (ushort)(ushort)28311;
            p92.chan1_raw = (ushort)(ushort)39089;
            p92.chan6_raw = (ushort)(ushort)62321;
            p92.chan4_raw = (ushort)(ushort)10493;
            SMP_TEST_CH.send(p92);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_ACTUATOR_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mode == (MAV_MODE)MAV_MODE.MAV_MODE_AUTO_ARMED);
                Debug.Assert(pack.flags == (ulong)2170362934917393450L);
                Debug.Assert(pack.time_usec == (ulong)9218447243307776992L);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-2.1782044E38F, -8.586041E37F, -1.2031935E37F, -1.0524735E38F, 1.2210894E38F, 2.954032E38F, 5.7891264E36F, -5.503445E37F, -9.832249E35F, -2.4531923E38F, 2.3880189E38F, -1.9903966E38F, 3.0910266E38F, -3.116983E38F, -5.510191E37F, 2.9895779E38F}));
            };
            HIL_ACTUATOR_CONTROLS p93 = new HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.flags = (ulong)2170362934917393450L;
            p93.time_usec = (ulong)9218447243307776992L;
            p93.mode = (MAV_MODE)MAV_MODE.MAV_MODE_AUTO_ARMED;
            p93.controls_SET(new float[] {-2.1782044E38F, -8.586041E37F, -1.2031935E37F, -1.0524735E38F, 1.2210894E38F, 2.954032E38F, 5.7891264E36F, -5.503445E37F, -9.832249E35F, -2.4531923E38F, 2.3880189E38F, -1.9903966E38F, 3.0910266E38F, -3.116983E38F, -5.510191E37F, 2.9895779E38F}, 0) ;
            SMP_TEST_CH.send(p93);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flow_comp_m_y == (float)5.214145E37F);
                Debug.Assert(pack.quality == (byte)(byte)253);
                Debug.Assert(pack.flow_x == (short)(short)13727);
                Debug.Assert(pack.ground_distance == (float)1.9455703E38F);
                Debug.Assert(pack.flow_comp_m_x == (float) -1.3237767E38F);
                Debug.Assert(pack.flow_y == (short)(short)5368);
                Debug.Assert(pack.flow_rate_y_TRY(ph) == (float) -1.9458477E38F);
                Debug.Assert(pack.sensor_id == (byte)(byte)232);
                Debug.Assert(pack.time_usec == (ulong)5281031459928248995L);
                Debug.Assert(pack.flow_rate_x_TRY(ph) == (float)3.091204E38F);
            };
            OPTICAL_FLOW p100 = new OPTICAL_FLOW();
            PH.setPack(p100);
            p100.flow_rate_x_SET((float)3.091204E38F, PH) ;
            p100.flow_x = (short)(short)13727;
            p100.sensor_id = (byte)(byte)232;
            p100.time_usec = (ulong)5281031459928248995L;
            p100.ground_distance = (float)1.9455703E38F;
            p100.flow_y = (short)(short)5368;
            p100.quality = (byte)(byte)253;
            p100.flow_comp_m_x = (float) -1.3237767E38F;
            p100.flow_rate_y_SET((float) -1.9458477E38F, PH) ;
            p100.flow_comp_m_y = (float)5.214145E37F;
            SMP_TEST_CH.send(p100);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_VISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float) -1.9836012E38F);
                Debug.Assert(pack.z == (float) -2.510712E38F);
                Debug.Assert(pack.usec == (ulong)3215725571686351591L);
                Debug.Assert(pack.pitch == (float)1.8819172E38F);
                Debug.Assert(pack.yaw == (float)1.0757807E38F);
                Debug.Assert(pack.roll == (float)2.104966E38F);
                Debug.Assert(pack.x == (float) -8.87334E37F);
            };
            GLOBAL_VISION_POSITION_ESTIMATE p101 = new GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.z = (float) -2.510712E38F;
            p101.y = (float) -1.9836012E38F;
            p101.pitch = (float)1.8819172E38F;
            p101.roll = (float)2.104966E38F;
            p101.yaw = (float)1.0757807E38F;
            p101.usec = (ulong)3215725571686351591L;
            p101.x = (float) -8.87334E37F;
            SMP_TEST_CH.send(p101);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float)6.276604E37F);
                Debug.Assert(pack.yaw == (float) -8.102322E37F);
                Debug.Assert(pack.pitch == (float)4.6276602E36F);
                Debug.Assert(pack.z == (float)1.5779479E38F);
                Debug.Assert(pack.usec == (ulong)6990159671688390505L);
                Debug.Assert(pack.roll == (float) -1.4727633E38F);
                Debug.Assert(pack.x == (float) -2.0166427E38F);
            };
            GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.pitch = (float)4.6276602E36F;
            p102.y = (float)6.276604E37F;
            p102.roll = (float) -1.4727633E38F;
            p102.usec = (ulong)6990159671688390505L;
            p102.yaw = (float) -8.102322E37F;
            p102.z = (float)1.5779479E38F;
            p102.x = (float) -2.0166427E38F;
            CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_SPEED_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.usec == (ulong)3469303482482049049L);
                Debug.Assert(pack.z == (float) -2.1715463E38F);
                Debug.Assert(pack.x == (float)2.0005013E38F);
                Debug.Assert(pack.y == (float) -9.739781E37F);
            };
            GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.z = (float) -2.1715463E38F;
            p103.x = (float)2.0005013E38F;
            p103.y = (float) -9.739781E37F;
            p103.usec = (ulong)3469303482482049049L;
            CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVICON_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.usec == (ulong)214210363304120152L);
                Debug.Assert(pack.z == (float) -3.2926496E38F);
                Debug.Assert(pack.yaw == (float) -4.2299553E37F);
                Debug.Assert(pack.y == (float) -6.637196E36F);
                Debug.Assert(pack.roll == (float)2.2863973E38F);
                Debug.Assert(pack.x == (float) -2.5003118E38F);
                Debug.Assert(pack.pitch == (float) -2.7766848E38F);
            };
            GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.yaw = (float) -4.2299553E37F;
            p104.y = (float) -6.637196E36F;
            p104.x = (float) -2.5003118E38F;
            p104.z = (float) -3.2926496E38F;
            p104.usec = (ulong)214210363304120152L;
            p104.pitch = (float) -2.7766848E38F;
            p104.roll = (float)2.2863973E38F;
            CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIGHRES_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yacc == (float) -5.684318E37F);
                Debug.Assert(pack.ymag == (float) -3.0127325E38F);
                Debug.Assert(pack.xgyro == (float)2.4177515E38F);
                Debug.Assert(pack.ygyro == (float) -2.827451E38F);
                Debug.Assert(pack.fields_updated == (ushort)(ushort)34847);
                Debug.Assert(pack.xmag == (float) -2.6854046E38F);
                Debug.Assert(pack.zmag == (float) -1.1079885E38F);
                Debug.Assert(pack.time_usec == (ulong)8880700362683123127L);
                Debug.Assert(pack.temperature == (float) -1.4766727E37F);
                Debug.Assert(pack.abs_pressure == (float)2.1001474E38F);
                Debug.Assert(pack.xacc == (float)2.6621127E38F);
                Debug.Assert(pack.diff_pressure == (float)1.5378849E38F);
                Debug.Assert(pack.zgyro == (float)2.3470764E38F);
                Debug.Assert(pack.pressure_alt == (float) -1.5024733E38F);
                Debug.Assert(pack.zacc == (float) -2.7339867E38F);
            };
            GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.diff_pressure = (float)1.5378849E38F;
            p105.xgyro = (float)2.4177515E38F;
            p105.pressure_alt = (float) -1.5024733E38F;
            p105.xmag = (float) -2.6854046E38F;
            p105.yacc = (float) -5.684318E37F;
            p105.time_usec = (ulong)8880700362683123127L;
            p105.ygyro = (float) -2.827451E38F;
            p105.zacc = (float) -2.7339867E38F;
            p105.fields_updated = (ushort)(ushort)34847;
            p105.xacc = (float)2.6621127E38F;
            p105.zmag = (float) -1.1079885E38F;
            p105.ymag = (float) -3.0127325E38F;
            p105.zgyro = (float)2.3470764E38F;
            p105.abs_pressure = (float)2.1001474E38F;
            p105.temperature = (float) -1.4766727E37F;
            CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOW_RADReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_delta_distance_us == (uint)3136811238U);
                Debug.Assert(pack.temperature == (short)(short) -2351);
                Debug.Assert(pack.sensor_id == (byte)(byte)198);
                Debug.Assert(pack.integrated_zgyro == (float)2.932814E38F);
                Debug.Assert(pack.integrated_y == (float) -1.7027458E38F);
                Debug.Assert(pack.integrated_xgyro == (float) -2.1230365E38F);
                Debug.Assert(pack.integrated_x == (float)1.5459808E38F);
                Debug.Assert(pack.distance == (float)2.1436998E38F);
                Debug.Assert(pack.integration_time_us == (uint)107539618U);
                Debug.Assert(pack.quality == (byte)(byte)20);
                Debug.Assert(pack.integrated_ygyro == (float)3.8495527E37F);
                Debug.Assert(pack.time_usec == (ulong)7815939635109178205L);
            };
            GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.integrated_xgyro = (float) -2.1230365E38F;
            p106.integrated_zgyro = (float)2.932814E38F;
            p106.temperature = (short)(short) -2351;
            p106.integration_time_us = (uint)107539618U;
            p106.sensor_id = (byte)(byte)198;
            p106.quality = (byte)(byte)20;
            p106.integrated_x = (float)1.5459808E38F;
            p106.integrated_ygyro = (float)3.8495527E37F;
            p106.time_delta_distance_us = (uint)3136811238U;
            p106.distance = (float)2.1436998E38F;
            p106.integrated_y = (float) -1.7027458E38F;
            p106.time_usec = (ulong)7815939635109178205L;
            CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ymag == (float) -3.1962293E38F);
                Debug.Assert(pack.zacc == (float)1.7503225E38F);
                Debug.Assert(pack.xacc == (float)3.3971318E38F);
                Debug.Assert(pack.zgyro == (float) -3.3626907E38F);
                Debug.Assert(pack.pressure_alt == (float)7.8754715E37F);
                Debug.Assert(pack.yacc == (float)3.0364814E38F);
                Debug.Assert(pack.xmag == (float) -5.1158555E37F);
                Debug.Assert(pack.abs_pressure == (float) -6.6272657E37F);
                Debug.Assert(pack.ygyro == (float) -1.0674219E38F);
                Debug.Assert(pack.temperature == (float) -8.1551173E37F);
                Debug.Assert(pack.zmag == (float) -6.961311E37F);
                Debug.Assert(pack.diff_pressure == (float) -1.4280566E38F);
                Debug.Assert(pack.fields_updated == (uint)97131041U);
                Debug.Assert(pack.xgyro == (float) -2.8102733E38F);
                Debug.Assert(pack.time_usec == (ulong)3916972362808464565L);
            };
            GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.diff_pressure = (float) -1.4280566E38F;
            p107.yacc = (float)3.0364814E38F;
            p107.xgyro = (float) -2.8102733E38F;
            p107.zacc = (float)1.7503225E38F;
            p107.ygyro = (float) -1.0674219E38F;
            p107.abs_pressure = (float) -6.6272657E37F;
            p107.xmag = (float) -5.1158555E37F;
            p107.temperature = (float) -8.1551173E37F;
            p107.xacc = (float)3.3971318E38F;
            p107.ymag = (float) -3.1962293E38F;
            p107.zgyro = (float) -3.3626907E38F;
            p107.zmag = (float) -6.961311E37F;
            p107.fields_updated = (uint)97131041U;
            p107.time_usec = (ulong)3916972362808464565L;
            p107.pressure_alt = (float)7.8754715E37F;
            CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSIM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xacc == (float)2.8193732E38F);
                Debug.Assert(pack.zacc == (float) -1.232719E38F);
                Debug.Assert(pack.q3 == (float) -4.200233E37F);
                Debug.Assert(pack.roll == (float)2.9978566E38F);
                Debug.Assert(pack.vd == (float)2.0199428E38F);
                Debug.Assert(pack.ygyro == (float) -1.6538182E38F);
                Debug.Assert(pack.std_dev_horz == (float) -2.1339152E38F);
                Debug.Assert(pack.q1 == (float) -1.3624925E38F);
                Debug.Assert(pack.zgyro == (float) -2.1150294E38F);
                Debug.Assert(pack.lat == (float)3.3758093E38F);
                Debug.Assert(pack.pitch == (float) -1.4692575E38F);
                Debug.Assert(pack.q4 == (float) -6.637684E37F);
                Debug.Assert(pack.yacc == (float) -3.2728353E38F);
                Debug.Assert(pack.q2 == (float)1.7957279E38F);
                Debug.Assert(pack.std_dev_vert == (float) -2.3804992E38F);
                Debug.Assert(pack.xgyro == (float) -1.2555205E38F);
                Debug.Assert(pack.lon == (float) -1.3076498E38F);
                Debug.Assert(pack.yaw == (float) -2.2442043E37F);
                Debug.Assert(pack.ve == (float)1.7766007E37F);
                Debug.Assert(pack.vn == (float) -1.3569616E38F);
                Debug.Assert(pack.alt == (float)2.75389E38F);
            };
            GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.zacc = (float) -1.232719E38F;
            p108.vd = (float)2.0199428E38F;
            p108.xacc = (float)2.8193732E38F;
            p108.q1 = (float) -1.3624925E38F;
            p108.ve = (float)1.7766007E37F;
            p108.lat = (float)3.3758093E38F;
            p108.std_dev_vert = (float) -2.3804992E38F;
            p108.ygyro = (float) -1.6538182E38F;
            p108.yacc = (float) -3.2728353E38F;
            p108.alt = (float)2.75389E38F;
            p108.q2 = (float)1.7957279E38F;
            p108.q3 = (float) -4.200233E37F;
            p108.pitch = (float) -1.4692575E38F;
            p108.std_dev_horz = (float) -2.1339152E38F;
            p108.vn = (float) -1.3569616E38F;
            p108.lon = (float) -1.3076498E38F;
            p108.roll = (float)2.9978566E38F;
            p108.q4 = (float) -6.637684E37F;
            p108.yaw = (float) -2.2442043E37F;
            p108.xgyro = (float) -1.2555205E38F;
            p108.zgyro = (float) -2.1150294E38F;
            CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRADIO_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.fixed_ == (ushort)(ushort)2994);
                Debug.Assert(pack.remrssi == (byte)(byte)246);
                Debug.Assert(pack.noise == (byte)(byte)3);
                Debug.Assert(pack.txbuf == (byte)(byte)133);
                Debug.Assert(pack.remnoise == (byte)(byte)115);
                Debug.Assert(pack.rxerrors == (ushort)(ushort)55340);
                Debug.Assert(pack.rssi == (byte)(byte)29);
            };
            GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.noise = (byte)(byte)3;
            p109.remnoise = (byte)(byte)115;
            p109.rxerrors = (ushort)(ushort)55340;
            p109.txbuf = (byte)(byte)133;
            p109.fixed_ = (ushort)(ushort)2994;
            p109.remrssi = (byte)(byte)246;
            p109.rssi = (byte)(byte)29;
            CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnFILE_TRANSFER_PROTOCOLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)72);
                Debug.Assert(pack.target_system == (byte)(byte)151);
                Debug.Assert(pack.target_network == (byte)(byte)52);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)218, (byte)168, (byte)212, (byte)29, (byte)17, (byte)111, (byte)143, (byte)239, (byte)175, (byte)113, (byte)90, (byte)237, (byte)33, (byte)105, (byte)89, (byte)147, (byte)212, (byte)122, (byte)15, (byte)178, (byte)37, (byte)179, (byte)86, (byte)49, (byte)127, (byte)9, (byte)171, (byte)119, (byte)100, (byte)178, (byte)231, (byte)85, (byte)9, (byte)17, (byte)164, (byte)99, (byte)166, (byte)57, (byte)25, (byte)111, (byte)3, (byte)35, (byte)253, (byte)20, (byte)79, (byte)62, (byte)177, (byte)105, (byte)39, (byte)250, (byte)244, (byte)159, (byte)139, (byte)183, (byte)197, (byte)113, (byte)47, (byte)80, (byte)251, (byte)13, (byte)59, (byte)36, (byte)78, (byte)119, (byte)196, (byte)50, (byte)105, (byte)68, (byte)227, (byte)27, (byte)83, (byte)133, (byte)143, (byte)72, (byte)150, (byte)41, (byte)34, (byte)219, (byte)101, (byte)130, (byte)21, (byte)22, (byte)254, (byte)57, (byte)234, (byte)180, (byte)120, (byte)105, (byte)168, (byte)120, (byte)170, (byte)138, (byte)175, (byte)90, (byte)237, (byte)209, (byte)95, (byte)26, (byte)60, (byte)253, (byte)249, (byte)47, (byte)228, (byte)232, (byte)21, (byte)111, (byte)101, (byte)212, (byte)158, (byte)110, (byte)153, (byte)64, (byte)163, (byte)195, (byte)214, (byte)12, (byte)155, (byte)225, (byte)30, (byte)134, (byte)196, (byte)2, (byte)94, (byte)209, (byte)140, (byte)130, (byte)121, (byte)177, (byte)4, (byte)16, (byte)195, (byte)23, (byte)35, (byte)82, (byte)144, (byte)29, (byte)83, (byte)132, (byte)108, (byte)174, (byte)185, (byte)60, (byte)247, (byte)191, (byte)211, (byte)210, (byte)189, (byte)212, (byte)49, (byte)127, (byte)152, (byte)208, (byte)59, (byte)248, (byte)138, (byte)44, (byte)243, (byte)160, (byte)111, (byte)160, (byte)187, (byte)153, (byte)42, (byte)175, (byte)63, (byte)159, (byte)105, (byte)255, (byte)44, (byte)85, (byte)56, (byte)24, (byte)28, (byte)122, (byte)222, (byte)217, (byte)10, (byte)38, (byte)50, (byte)24, (byte)88, (byte)35, (byte)171, (byte)27, (byte)223, (byte)20, (byte)128, (byte)237, (byte)41, (byte)127, (byte)232, (byte)176, (byte)75, (byte)154, (byte)98, (byte)48, (byte)184, (byte)89, (byte)118, (byte)234, (byte)92, (byte)156, (byte)144, (byte)54, (byte)145, (byte)115, (byte)176, (byte)95, (byte)173, (byte)7, (byte)145, (byte)123, (byte)238, (byte)239, (byte)42, (byte)164, (byte)16, (byte)166, (byte)169, (byte)214, (byte)141, (byte)33, (byte)199, (byte)229, (byte)224, (byte)234, (byte)232, (byte)177, (byte)104, (byte)73, (byte)245, (byte)182, (byte)119, (byte)55, (byte)200, (byte)196, (byte)178, (byte)8, (byte)142, (byte)222, (byte)231, (byte)235, (byte)183, (byte)36, (byte)161, (byte)164, (byte)126, (byte)209, (byte)176, (byte)195, (byte)86}));
            };
            GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_system = (byte)(byte)151;
            p110.payload_SET(new byte[] {(byte)218, (byte)168, (byte)212, (byte)29, (byte)17, (byte)111, (byte)143, (byte)239, (byte)175, (byte)113, (byte)90, (byte)237, (byte)33, (byte)105, (byte)89, (byte)147, (byte)212, (byte)122, (byte)15, (byte)178, (byte)37, (byte)179, (byte)86, (byte)49, (byte)127, (byte)9, (byte)171, (byte)119, (byte)100, (byte)178, (byte)231, (byte)85, (byte)9, (byte)17, (byte)164, (byte)99, (byte)166, (byte)57, (byte)25, (byte)111, (byte)3, (byte)35, (byte)253, (byte)20, (byte)79, (byte)62, (byte)177, (byte)105, (byte)39, (byte)250, (byte)244, (byte)159, (byte)139, (byte)183, (byte)197, (byte)113, (byte)47, (byte)80, (byte)251, (byte)13, (byte)59, (byte)36, (byte)78, (byte)119, (byte)196, (byte)50, (byte)105, (byte)68, (byte)227, (byte)27, (byte)83, (byte)133, (byte)143, (byte)72, (byte)150, (byte)41, (byte)34, (byte)219, (byte)101, (byte)130, (byte)21, (byte)22, (byte)254, (byte)57, (byte)234, (byte)180, (byte)120, (byte)105, (byte)168, (byte)120, (byte)170, (byte)138, (byte)175, (byte)90, (byte)237, (byte)209, (byte)95, (byte)26, (byte)60, (byte)253, (byte)249, (byte)47, (byte)228, (byte)232, (byte)21, (byte)111, (byte)101, (byte)212, (byte)158, (byte)110, (byte)153, (byte)64, (byte)163, (byte)195, (byte)214, (byte)12, (byte)155, (byte)225, (byte)30, (byte)134, (byte)196, (byte)2, (byte)94, (byte)209, (byte)140, (byte)130, (byte)121, (byte)177, (byte)4, (byte)16, (byte)195, (byte)23, (byte)35, (byte)82, (byte)144, (byte)29, (byte)83, (byte)132, (byte)108, (byte)174, (byte)185, (byte)60, (byte)247, (byte)191, (byte)211, (byte)210, (byte)189, (byte)212, (byte)49, (byte)127, (byte)152, (byte)208, (byte)59, (byte)248, (byte)138, (byte)44, (byte)243, (byte)160, (byte)111, (byte)160, (byte)187, (byte)153, (byte)42, (byte)175, (byte)63, (byte)159, (byte)105, (byte)255, (byte)44, (byte)85, (byte)56, (byte)24, (byte)28, (byte)122, (byte)222, (byte)217, (byte)10, (byte)38, (byte)50, (byte)24, (byte)88, (byte)35, (byte)171, (byte)27, (byte)223, (byte)20, (byte)128, (byte)237, (byte)41, (byte)127, (byte)232, (byte)176, (byte)75, (byte)154, (byte)98, (byte)48, (byte)184, (byte)89, (byte)118, (byte)234, (byte)92, (byte)156, (byte)144, (byte)54, (byte)145, (byte)115, (byte)176, (byte)95, (byte)173, (byte)7, (byte)145, (byte)123, (byte)238, (byte)239, (byte)42, (byte)164, (byte)16, (byte)166, (byte)169, (byte)214, (byte)141, (byte)33, (byte)199, (byte)229, (byte)224, (byte)234, (byte)232, (byte)177, (byte)104, (byte)73, (byte)245, (byte)182, (byte)119, (byte)55, (byte)200, (byte)196, (byte)178, (byte)8, (byte)142, (byte)222, (byte)231, (byte)235, (byte)183, (byte)36, (byte)161, (byte)164, (byte)126, (byte)209, (byte)176, (byte)195, (byte)86}, 0) ;
            p110.target_component = (byte)(byte)72;
            p110.target_network = (byte)(byte)52;
            CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTIMESYNCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tc1 == (long) -7702181037844594778L);
                Debug.Assert(pack.ts1 == (long) -5798949233119714543L);
            };
            GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.ts1 = (long) -5798949233119714543L;
            p111.tc1 = (long) -7702181037844594778L;
            CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCAMERA_TRIGGERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (uint)3509691545U);
                Debug.Assert(pack.time_usec == (ulong)7405651983199363612L);
            };
            GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.seq = (uint)3509691545U;
            p112.time_usec = (ulong)7405651983199363612L;
            CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_GPSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.fix_type == (byte)(byte)120);
                Debug.Assert(pack.vn == (short)(short) -18956);
                Debug.Assert(pack.lon == (int)735535618);
                Debug.Assert(pack.alt == (int)1968765730);
                Debug.Assert(pack.satellites_visible == (byte)(byte)186);
                Debug.Assert(pack.lat == (int) -1484790310);
                Debug.Assert(pack.cog == (ushort)(ushort)4107);
                Debug.Assert(pack.epv == (ushort)(ushort)47387);
                Debug.Assert(pack.vd == (short)(short)27567);
                Debug.Assert(pack.vel == (ushort)(ushort)40113);
                Debug.Assert(pack.eph == (ushort)(ushort)62342);
                Debug.Assert(pack.ve == (short)(short)9853);
                Debug.Assert(pack.time_usec == (ulong)1102881950491631216L);
            };
            GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.vd = (short)(short)27567;
            p113.lat = (int) -1484790310;
            p113.time_usec = (ulong)1102881950491631216L;
            p113.vn = (short)(short) -18956;
            p113.satellites_visible = (byte)(byte)186;
            p113.eph = (ushort)(ushort)62342;
            p113.vel = (ushort)(ushort)40113;
            p113.epv = (ushort)(ushort)47387;
            p113.lon = (int)735535618;
            p113.fix_type = (byte)(byte)120;
            p113.cog = (ushort)(ushort)4107;
            p113.ve = (short)(short)9853;
            p113.alt = (int)1968765730;
            CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_OPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.integrated_ygyro == (float) -5.727263E36F);
                Debug.Assert(pack.temperature == (short)(short) -21537);
                Debug.Assert(pack.time_usec == (ulong)7032038397314583786L);
                Debug.Assert(pack.integrated_zgyro == (float)1.5572245E37F);
                Debug.Assert(pack.integrated_x == (float) -9.544777E37F);
                Debug.Assert(pack.distance == (float)3.266867E38F);
                Debug.Assert(pack.time_delta_distance_us == (uint)4186729658U);
                Debug.Assert(pack.sensor_id == (byte)(byte)230);
                Debug.Assert(pack.integrated_y == (float)4.059176E37F);
                Debug.Assert(pack.integrated_xgyro == (float)1.8414189E38F);
                Debug.Assert(pack.quality == (byte)(byte)147);
                Debug.Assert(pack.integration_time_us == (uint)2095931130U);
            };
            GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.sensor_id = (byte)(byte)230;
            p114.integration_time_us = (uint)2095931130U;
            p114.temperature = (short)(short) -21537;
            p114.integrated_x = (float) -9.544777E37F;
            p114.distance = (float)3.266867E38F;
            p114.quality = (byte)(byte)147;
            p114.integrated_zgyro = (float)1.5572245E37F;
            p114.integrated_xgyro = (float)1.8414189E38F;
            p114.time_delta_distance_us = (uint)4186729658U;
            p114.time_usec = (ulong)7032038397314583786L;
            p114.integrated_ygyro = (float) -5.727263E36F;
            p114.integrated_y = (float)4.059176E37F;
            CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ind_airspeed == (ushort)(ushort)23934);
                Debug.Assert(pack.xacc == (short)(short) -21862);
                Debug.Assert(pack.yacc == (short)(short)30717);
                Debug.Assert(pack.zacc == (short)(short)1098);
                Debug.Assert(pack.vz == (short)(short) -22225);
                Debug.Assert(pack.alt == (int)112811711);
                Debug.Assert(pack.true_airspeed == (ushort)(ushort)20117);
                Debug.Assert(pack.pitchspeed == (float)2.3544133E38F);
                Debug.Assert(pack.time_usec == (ulong)8720400819114777223L);
                Debug.Assert(pack.lon == (int) -1021695465);
                Debug.Assert(pack.vx == (short)(short) -16215);
                Debug.Assert(pack.attitude_quaternion.SequenceEqual(new float[] {-9.831289E37F, 2.969413E38F, -1.9512211E38F, -5.056158E37F}));
                Debug.Assert(pack.lat == (int) -1433087416);
                Debug.Assert(pack.vy == (short)(short) -26206);
                Debug.Assert(pack.yawspeed == (float)6.661518E37F);
                Debug.Assert(pack.rollspeed == (float)7.773001E37F);
            };
            GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.time_usec = (ulong)8720400819114777223L;
            p115.xacc = (short)(short) -21862;
            p115.zacc = (short)(short)1098;
            p115.pitchspeed = (float)2.3544133E38F;
            p115.alt = (int)112811711;
            p115.true_airspeed = (ushort)(ushort)20117;
            p115.lat = (int) -1433087416;
            p115.rollspeed = (float)7.773001E37F;
            p115.vx = (short)(short) -16215;
            p115.yacc = (short)(short)30717;
            p115.vy = (short)(short) -26206;
            p115.lon = (int) -1021695465;
            p115.vz = (short)(short) -22225;
            p115.yawspeed = (float)6.661518E37F;
            p115.ind_airspeed = (ushort)(ushort)23934;
            p115.attitude_quaternion_SET(new float[] {-9.831289E37F, 2.969413E38F, -1.9512211E38F, -5.056158E37F}, 0) ;
            CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ymag == (short)(short)5848);
                Debug.Assert(pack.time_boot_ms == (uint)2833118937U);
                Debug.Assert(pack.zgyro == (short)(short)4273);
                Debug.Assert(pack.xgyro == (short)(short)13835);
                Debug.Assert(pack.xmag == (short)(short) -26077);
                Debug.Assert(pack.yacc == (short)(short)23171);
                Debug.Assert(pack.zmag == (short)(short)10250);
                Debug.Assert(pack.xacc == (short)(short) -13519);
                Debug.Assert(pack.zacc == (short)(short)24959);
                Debug.Assert(pack.ygyro == (short)(short)17376);
            };
            GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.xmag = (short)(short) -26077;
            p116.xacc = (short)(short) -13519;
            p116.time_boot_ms = (uint)2833118937U;
            p116.zmag = (short)(short)10250;
            p116.ymag = (short)(short)5848;
            p116.ygyro = (short)(short)17376;
            p116.zgyro = (short)(short)4273;
            p116.zacc = (short)(short)24959;
            p116.xgyro = (short)(short)13835;
            p116.yacc = (short)(short)23171;
            CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.end == (ushort)(ushort)20820);
                Debug.Assert(pack.target_component == (byte)(byte)180);
                Debug.Assert(pack.target_system == (byte)(byte)167);
                Debug.Assert(pack.start == (ushort)(ushort)65304);
            };
            GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.start = (ushort)(ushort)65304;
            p117.target_component = (byte)(byte)180;
            p117.end = (ushort)(ushort)20820;
            p117.target_system = (byte)(byte)167;
            CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ENTRYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.id == (ushort)(ushort)60894);
                Debug.Assert(pack.last_log_num == (ushort)(ushort)49604);
                Debug.Assert(pack.time_utc == (uint)176708842U);
                Debug.Assert(pack.num_logs == (ushort)(ushort)37065);
                Debug.Assert(pack.size == (uint)2940281691U);
            };
            GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.id = (ushort)(ushort)60894;
            p118.time_utc = (uint)176708842U;
            p118.last_log_num = (ushort)(ushort)49604;
            p118.num_logs = (ushort)(ushort)37065;
            p118.size = (uint)2940281691U;
            CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ofs == (uint)2122815938U);
                Debug.Assert(pack.id == (ushort)(ushort)34435);
                Debug.Assert(pack.target_system == (byte)(byte)4);
                Debug.Assert(pack.target_component == (byte)(byte)192);
                Debug.Assert(pack.count == (uint)3999248065U);
            };
            GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.id = (ushort)(ushort)34435;
            p119.count = (uint)3999248065U;
            p119.ofs = (uint)2122815938U;
            p119.target_component = (byte)(byte)192;
            p119.target_system = (byte)(byte)4;
            CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)132, (byte)126, (byte)239, (byte)212, (byte)199, (byte)155, (byte)134, (byte)13, (byte)65, (byte)79, (byte)253, (byte)35, (byte)82, (byte)166, (byte)220, (byte)161, (byte)72, (byte)91, (byte)185, (byte)217, (byte)22, (byte)46, (byte)58, (byte)222, (byte)102, (byte)192, (byte)88, (byte)38, (byte)165, (byte)166, (byte)30, (byte)44, (byte)49, (byte)119, (byte)72, (byte)185, (byte)173, (byte)10, (byte)103, (byte)180, (byte)173, (byte)216, (byte)241, (byte)173, (byte)58, (byte)115, (byte)130, (byte)45, (byte)19, (byte)160, (byte)54, (byte)239, (byte)217, (byte)92, (byte)197, (byte)30, (byte)74, (byte)237, (byte)171, (byte)91, (byte)138, (byte)94, (byte)96, (byte)105, (byte)248, (byte)204, (byte)200, (byte)67, (byte)99, (byte)220, (byte)25, (byte)252, (byte)241, (byte)180, (byte)235, (byte)224, (byte)226, (byte)57, (byte)73, (byte)147, (byte)82, (byte)182, (byte)95, (byte)131, (byte)128, (byte)54, (byte)221, (byte)167, (byte)114, (byte)29}));
                Debug.Assert(pack.ofs == (uint)3214776095U);
                Debug.Assert(pack.id == (ushort)(ushort)57273);
                Debug.Assert(pack.count == (byte)(byte)13);
            };
            GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.ofs = (uint)3214776095U;
            p120.count = (byte)(byte)13;
            p120.id = (ushort)(ushort)57273;
            p120.data__SET(new byte[] {(byte)132, (byte)126, (byte)239, (byte)212, (byte)199, (byte)155, (byte)134, (byte)13, (byte)65, (byte)79, (byte)253, (byte)35, (byte)82, (byte)166, (byte)220, (byte)161, (byte)72, (byte)91, (byte)185, (byte)217, (byte)22, (byte)46, (byte)58, (byte)222, (byte)102, (byte)192, (byte)88, (byte)38, (byte)165, (byte)166, (byte)30, (byte)44, (byte)49, (byte)119, (byte)72, (byte)185, (byte)173, (byte)10, (byte)103, (byte)180, (byte)173, (byte)216, (byte)241, (byte)173, (byte)58, (byte)115, (byte)130, (byte)45, (byte)19, (byte)160, (byte)54, (byte)239, (byte)217, (byte)92, (byte)197, (byte)30, (byte)74, (byte)237, (byte)171, (byte)91, (byte)138, (byte)94, (byte)96, (byte)105, (byte)248, (byte)204, (byte)200, (byte)67, (byte)99, (byte)220, (byte)25, (byte)252, (byte)241, (byte)180, (byte)235, (byte)224, (byte)226, (byte)57, (byte)73, (byte)147, (byte)82, (byte)182, (byte)95, (byte)131, (byte)128, (byte)54, (byte)221, (byte)167, (byte)114, (byte)29}, 0) ;
            CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ERASEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)226);
                Debug.Assert(pack.target_system == (byte)(byte)228);
            };
            GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_component = (byte)(byte)226;
            p121.target_system = (byte)(byte)228;
            CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_ENDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)134);
                Debug.Assert(pack.target_component == (byte)(byte)93);
            };
            GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_component = (byte)(byte)93;
            p122.target_system = (byte)(byte)134;
            CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_INJECT_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)110, (byte)112, (byte)44, (byte)64, (byte)122, (byte)115, (byte)195, (byte)226, (byte)109, (byte)254, (byte)223, (byte)27, (byte)118, (byte)151, (byte)7, (byte)130, (byte)1, (byte)132, (byte)215, (byte)5, (byte)226, (byte)108, (byte)5, (byte)101, (byte)78, (byte)67, (byte)36, (byte)164, (byte)67, (byte)225, (byte)242, (byte)36, (byte)88, (byte)213, (byte)208, (byte)116, (byte)91, (byte)188, (byte)0, (byte)2, (byte)58, (byte)162, (byte)122, (byte)75, (byte)240, (byte)178, (byte)136, (byte)68, (byte)146, (byte)212, (byte)208, (byte)181, (byte)138, (byte)71, (byte)223, (byte)68, (byte)142, (byte)12, (byte)137, (byte)57, (byte)27, (byte)93, (byte)176, (byte)56, (byte)241, (byte)229, (byte)97, (byte)154, (byte)20, (byte)198, (byte)93, (byte)45, (byte)141, (byte)72, (byte)169, (byte)104, (byte)107, (byte)207, (byte)185, (byte)131, (byte)82, (byte)51, (byte)172, (byte)248, (byte)27, (byte)68, (byte)196, (byte)116, (byte)80, (byte)151, (byte)165, (byte)141, (byte)222, (byte)0, (byte)77, (byte)193, (byte)238, (byte)254, (byte)182, (byte)67, (byte)122, (byte)246, (byte)153, (byte)249, (byte)100, (byte)108, (byte)139, (byte)125, (byte)62, (byte)240}));
                Debug.Assert(pack.target_system == (byte)(byte)25);
                Debug.Assert(pack.target_component == (byte)(byte)20);
                Debug.Assert(pack.len == (byte)(byte)135);
            };
            GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.data__SET(new byte[] {(byte)110, (byte)112, (byte)44, (byte)64, (byte)122, (byte)115, (byte)195, (byte)226, (byte)109, (byte)254, (byte)223, (byte)27, (byte)118, (byte)151, (byte)7, (byte)130, (byte)1, (byte)132, (byte)215, (byte)5, (byte)226, (byte)108, (byte)5, (byte)101, (byte)78, (byte)67, (byte)36, (byte)164, (byte)67, (byte)225, (byte)242, (byte)36, (byte)88, (byte)213, (byte)208, (byte)116, (byte)91, (byte)188, (byte)0, (byte)2, (byte)58, (byte)162, (byte)122, (byte)75, (byte)240, (byte)178, (byte)136, (byte)68, (byte)146, (byte)212, (byte)208, (byte)181, (byte)138, (byte)71, (byte)223, (byte)68, (byte)142, (byte)12, (byte)137, (byte)57, (byte)27, (byte)93, (byte)176, (byte)56, (byte)241, (byte)229, (byte)97, (byte)154, (byte)20, (byte)198, (byte)93, (byte)45, (byte)141, (byte)72, (byte)169, (byte)104, (byte)107, (byte)207, (byte)185, (byte)131, (byte)82, (byte)51, (byte)172, (byte)248, (byte)27, (byte)68, (byte)196, (byte)116, (byte)80, (byte)151, (byte)165, (byte)141, (byte)222, (byte)0, (byte)77, (byte)193, (byte)238, (byte)254, (byte)182, (byte)67, (byte)122, (byte)246, (byte)153, (byte)249, (byte)100, (byte)108, (byte)139, (byte)125, (byte)62, (byte)240}, 0) ;
            p123.len = (byte)(byte)135;
            p123.target_system = (byte)(byte)25;
            p123.target_component = (byte)(byte)20;
            CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vel == (ushort)(ushort)344);
                Debug.Assert(pack.dgps_age == (uint)696065844U);
                Debug.Assert(pack.dgps_numch == (byte)(byte)211);
                Debug.Assert(pack.epv == (ushort)(ushort)11813);
                Debug.Assert(pack.fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX);
                Debug.Assert(pack.satellites_visible == (byte)(byte)226);
                Debug.Assert(pack.lon == (int)1313623952);
                Debug.Assert(pack.alt == (int)456734914);
                Debug.Assert(pack.time_usec == (ulong)3516399755292854625L);
                Debug.Assert(pack.cog == (ushort)(ushort)41874);
                Debug.Assert(pack.lat == (int)634266545);
                Debug.Assert(pack.eph == (ushort)(ushort)18674);
            };
            GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.epv = (ushort)(ushort)11813;
            p124.time_usec = (ulong)3516399755292854625L;
            p124.dgps_age = (uint)696065844U;
            p124.eph = (ushort)(ushort)18674;
            p124.lat = (int)634266545;
            p124.lon = (int)1313623952;
            p124.alt = (int)456734914;
            p124.dgps_numch = (byte)(byte)211;
            p124.cog = (ushort)(ushort)41874;
            p124.satellites_visible = (byte)(byte)226;
            p124.vel = (ushort)(ushort)344;
            p124.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX;
            CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOWER_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (MAV_POWER_STATUS)MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED);
                Debug.Assert(pack.Vservo == (ushort)(ushort)60478);
                Debug.Assert(pack.Vcc == (ushort)(ushort)9975);
            };
            GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.flags = (MAV_POWER_STATUS)MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED;
            p125.Vcc = (ushort)(ushort)9975;
            p125.Vservo = (ushort)(ushort)60478;
            CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERIAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (SERIAL_CONTROL_FLAG)SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)6, (byte)54, (byte)170, (byte)170, (byte)127, (byte)189, (byte)25, (byte)100, (byte)63, (byte)14, (byte)33, (byte)239, (byte)47, (byte)40, (byte)79, (byte)188, (byte)121, (byte)113, (byte)161, (byte)169, (byte)239, (byte)235, (byte)210, (byte)250, (byte)139, (byte)240, (byte)203, (byte)54, (byte)124, (byte)122, (byte)67, (byte)98, (byte)93, (byte)135, (byte)78, (byte)19, (byte)11, (byte)100, (byte)39, (byte)250, (byte)245, (byte)133, (byte)35, (byte)28, (byte)216, (byte)96, (byte)48, (byte)30, (byte)207, (byte)209, (byte)204, (byte)34, (byte)197, (byte)169, (byte)88, (byte)184, (byte)121, (byte)29, (byte)198, (byte)26, (byte)171, (byte)4, (byte)60, (byte)110, (byte)129, (byte)97, (byte)176, (byte)233, (byte)99, (byte)168}));
                Debug.Assert(pack.device == (SERIAL_CONTROL_DEV)SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_SHELL);
                Debug.Assert(pack.timeout == (ushort)(ushort)60531);
                Debug.Assert(pack.count == (byte)(byte)111);
                Debug.Assert(pack.baudrate == (uint)3478640143U);
            };
            GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.count = (byte)(byte)111;
            p126.data__SET(new byte[] {(byte)6, (byte)54, (byte)170, (byte)170, (byte)127, (byte)189, (byte)25, (byte)100, (byte)63, (byte)14, (byte)33, (byte)239, (byte)47, (byte)40, (byte)79, (byte)188, (byte)121, (byte)113, (byte)161, (byte)169, (byte)239, (byte)235, (byte)210, (byte)250, (byte)139, (byte)240, (byte)203, (byte)54, (byte)124, (byte)122, (byte)67, (byte)98, (byte)93, (byte)135, (byte)78, (byte)19, (byte)11, (byte)100, (byte)39, (byte)250, (byte)245, (byte)133, (byte)35, (byte)28, (byte)216, (byte)96, (byte)48, (byte)30, (byte)207, (byte)209, (byte)204, (byte)34, (byte)197, (byte)169, (byte)88, (byte)184, (byte)121, (byte)29, (byte)198, (byte)26, (byte)171, (byte)4, (byte)60, (byte)110, (byte)129, (byte)97, (byte)176, (byte)233, (byte)99, (byte)168}, 0) ;
            p126.baudrate = (uint)3478640143U;
            p126.timeout = (ushort)(ushort)60531;
            p126.device = (SERIAL_CONTROL_DEV)SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_SHELL;
            p126.flags = (SERIAL_CONTROL_FLAG)SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND;
            CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.wn == (ushort)(ushort)57234);
                Debug.Assert(pack.baseline_b_mm == (int) -1118595215);
                Debug.Assert(pack.rtk_rate == (byte)(byte)163);
                Debug.Assert(pack.rtk_health == (byte)(byte)58);
                Debug.Assert(pack.iar_num_hypotheses == (int)213194587);
                Debug.Assert(pack.baseline_a_mm == (int)752521366);
                Debug.Assert(pack.time_last_baseline_ms == (uint)3523484661U);
                Debug.Assert(pack.accuracy == (uint)2750973055U);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)224);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)177);
                Debug.Assert(pack.tow == (uint)2488496037U);
                Debug.Assert(pack.nsats == (byte)(byte)149);
                Debug.Assert(pack.baseline_c_mm == (int) -1867873265);
            };
            GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.accuracy = (uint)2750973055U;
            p127.time_last_baseline_ms = (uint)3523484661U;
            p127.baseline_b_mm = (int) -1118595215;
            p127.rtk_receiver_id = (byte)(byte)177;
            p127.rtk_rate = (byte)(byte)163;
            p127.iar_num_hypotheses = (int)213194587;
            p127.baseline_c_mm = (int) -1867873265;
            p127.rtk_health = (byte)(byte)58;
            p127.tow = (uint)2488496037U;
            p127.baseline_a_mm = (int)752521366;
            p127.baseline_coords_type = (byte)(byte)224;
            p127.wn = (ushort)(ushort)57234;
            p127.nsats = (byte)(byte)149;
            CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.accuracy == (uint)384124602U);
                Debug.Assert(pack.iar_num_hypotheses == (int)1418273413);
                Debug.Assert(pack.rtk_health == (byte)(byte)67);
                Debug.Assert(pack.baseline_c_mm == (int)1271932972);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)104);
                Debug.Assert(pack.rtk_rate == (byte)(byte)169);
                Debug.Assert(pack.time_last_baseline_ms == (uint)1280160349U);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)220);
                Debug.Assert(pack.wn == (ushort)(ushort)57285);
                Debug.Assert(pack.nsats == (byte)(byte)54);
                Debug.Assert(pack.baseline_b_mm == (int) -1104382030);
                Debug.Assert(pack.baseline_a_mm == (int) -84805990);
                Debug.Assert(pack.tow == (uint)3517939236U);
            };
            GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.wn = (ushort)(ushort)57285;
            p128.baseline_coords_type = (byte)(byte)104;
            p128.baseline_c_mm = (int)1271932972;
            p128.tow = (uint)3517939236U;
            p128.accuracy = (uint)384124602U;
            p128.rtk_health = (byte)(byte)67;
            p128.rtk_receiver_id = (byte)(byte)220;
            p128.baseline_b_mm = (int) -1104382030;
            p128.baseline_a_mm = (int) -84805990;
            p128.rtk_rate = (byte)(byte)169;
            p128.time_last_baseline_ms = (uint)1280160349U;
            p128.iar_num_hypotheses = (int)1418273413;
            p128.nsats = (byte)(byte)54;
            CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yacc == (short)(short)29683);
                Debug.Assert(pack.zmag == (short)(short)12272);
                Debug.Assert(pack.time_boot_ms == (uint)1677185905U);
                Debug.Assert(pack.xacc == (short)(short) -24909);
                Debug.Assert(pack.ygyro == (short)(short) -9232);
                Debug.Assert(pack.zgyro == (short)(short) -8193);
                Debug.Assert(pack.xmag == (short)(short)23967);
                Debug.Assert(pack.xgyro == (short)(short)15223);
                Debug.Assert(pack.ymag == (short)(short) -12799);
                Debug.Assert(pack.zacc == (short)(short) -32326);
            };
            GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.zacc = (short)(short) -32326;
            p129.ymag = (short)(short) -12799;
            p129.time_boot_ms = (uint)1677185905U;
            p129.zmag = (short)(short)12272;
            p129.xmag = (short)(short)23967;
            p129.xgyro = (short)(short)15223;
            p129.zgyro = (short)(short) -8193;
            p129.yacc = (short)(short)29683;
            p129.xacc = (short)(short) -24909;
            p129.ygyro = (short)(short) -9232;
            CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_TRANSMISSION_HANDSHAKEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.jpg_quality == (byte)(byte)45);
                Debug.Assert(pack.payload == (byte)(byte)122);
                Debug.Assert(pack.height == (ushort)(ushort)13561);
                Debug.Assert(pack.size == (uint)1655019861U);
                Debug.Assert(pack.type == (byte)(byte)137);
                Debug.Assert(pack.packets == (ushort)(ushort)9077);
                Debug.Assert(pack.width == (ushort)(ushort)55867);
            };
            GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.type = (byte)(byte)137;
            p130.height = (ushort)(ushort)13561;
            p130.payload = (byte)(byte)122;
            p130.jpg_quality = (byte)(byte)45;
            p130.width = (ushort)(ushort)55867;
            p130.packets = (ushort)(ushort)9077;
            p130.size = (uint)1655019861U;
            CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnENCAPSULATED_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)47, (byte)92, (byte)47, (byte)119, (byte)122, (byte)49, (byte)152, (byte)179, (byte)50, (byte)210, (byte)44, (byte)223, (byte)145, (byte)158, (byte)100, (byte)20, (byte)140, (byte)184, (byte)13, (byte)25, (byte)96, (byte)245, (byte)126, (byte)64, (byte)73, (byte)24, (byte)92, (byte)153, (byte)116, (byte)196, (byte)238, (byte)138, (byte)12, (byte)18, (byte)177, (byte)226, (byte)80, (byte)216, (byte)137, (byte)131, (byte)224, (byte)186, (byte)117, (byte)159, (byte)122, (byte)18, (byte)207, (byte)142, (byte)236, (byte)221, (byte)9, (byte)206, (byte)62, (byte)60, (byte)30, (byte)137, (byte)97, (byte)153, (byte)123, (byte)224, (byte)60, (byte)194, (byte)173, (byte)92, (byte)78, (byte)224, (byte)200, (byte)236, (byte)139, (byte)46, (byte)206, (byte)92, (byte)5, (byte)49, (byte)240, (byte)137, (byte)24, (byte)168, (byte)208, (byte)80, (byte)34, (byte)194, (byte)69, (byte)127, (byte)219, (byte)18, (byte)245, (byte)249, (byte)236, (byte)94, (byte)216, (byte)129, (byte)16, (byte)125, (byte)206, (byte)227, (byte)1, (byte)202, (byte)85, (byte)50, (byte)29, (byte)54, (byte)115, (byte)193, (byte)67, (byte)27, (byte)174, (byte)248, (byte)215, (byte)83, (byte)173, (byte)114, (byte)217, (byte)90, (byte)62, (byte)205, (byte)87, (byte)146, (byte)121, (byte)203, (byte)68, (byte)97, (byte)57, (byte)227, (byte)108, (byte)220, (byte)66, (byte)21, (byte)63, (byte)108, (byte)151, (byte)250, (byte)221, (byte)50, (byte)189, (byte)229, (byte)101, (byte)40, (byte)254, (byte)127, (byte)71, (byte)252, (byte)64, (byte)236, (byte)213, (byte)31, (byte)58, (byte)96, (byte)99, (byte)172, (byte)110, (byte)138, (byte)107, (byte)186, (byte)85, (byte)97, (byte)37, (byte)97, (byte)75, (byte)254, (byte)86, (byte)189, (byte)237, (byte)66, (byte)151, (byte)178, (byte)221, (byte)107, (byte)6, (byte)147, (byte)175, (byte)21, (byte)88, (byte)155, (byte)13, (byte)219, (byte)39, (byte)88, (byte)17, (byte)79, (byte)249, (byte)240, (byte)140, (byte)36, (byte)201, (byte)36, (byte)38, (byte)248, (byte)113, (byte)163, (byte)213, (byte)254, (byte)161, (byte)112, (byte)254, (byte)172, (byte)149, (byte)205, (byte)64, (byte)240, (byte)3, (byte)126, (byte)72, (byte)236, (byte)179, (byte)19, (byte)184, (byte)152, (byte)217, (byte)188, (byte)59, (byte)118, (byte)33, (byte)9, (byte)61, (byte)110, (byte)100, (byte)142, (byte)244, (byte)78, (byte)152, (byte)89, (byte)15, (byte)63, (byte)69, (byte)160, (byte)25, (byte)73, (byte)55, (byte)70, (byte)71, (byte)219, (byte)61, (byte)179, (byte)133, (byte)109, (byte)133, (byte)0, (byte)21, (byte)10, (byte)137, (byte)29, (byte)136, (byte)163, (byte)96, (byte)194, (byte)117, (byte)17, (byte)16, (byte)89, (byte)251, (byte)247, (byte)212}));
                Debug.Assert(pack.seqnr == (ushort)(ushort)7094);
            };
            GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.seqnr = (ushort)(ushort)7094;
            p131.data__SET(new byte[] {(byte)47, (byte)92, (byte)47, (byte)119, (byte)122, (byte)49, (byte)152, (byte)179, (byte)50, (byte)210, (byte)44, (byte)223, (byte)145, (byte)158, (byte)100, (byte)20, (byte)140, (byte)184, (byte)13, (byte)25, (byte)96, (byte)245, (byte)126, (byte)64, (byte)73, (byte)24, (byte)92, (byte)153, (byte)116, (byte)196, (byte)238, (byte)138, (byte)12, (byte)18, (byte)177, (byte)226, (byte)80, (byte)216, (byte)137, (byte)131, (byte)224, (byte)186, (byte)117, (byte)159, (byte)122, (byte)18, (byte)207, (byte)142, (byte)236, (byte)221, (byte)9, (byte)206, (byte)62, (byte)60, (byte)30, (byte)137, (byte)97, (byte)153, (byte)123, (byte)224, (byte)60, (byte)194, (byte)173, (byte)92, (byte)78, (byte)224, (byte)200, (byte)236, (byte)139, (byte)46, (byte)206, (byte)92, (byte)5, (byte)49, (byte)240, (byte)137, (byte)24, (byte)168, (byte)208, (byte)80, (byte)34, (byte)194, (byte)69, (byte)127, (byte)219, (byte)18, (byte)245, (byte)249, (byte)236, (byte)94, (byte)216, (byte)129, (byte)16, (byte)125, (byte)206, (byte)227, (byte)1, (byte)202, (byte)85, (byte)50, (byte)29, (byte)54, (byte)115, (byte)193, (byte)67, (byte)27, (byte)174, (byte)248, (byte)215, (byte)83, (byte)173, (byte)114, (byte)217, (byte)90, (byte)62, (byte)205, (byte)87, (byte)146, (byte)121, (byte)203, (byte)68, (byte)97, (byte)57, (byte)227, (byte)108, (byte)220, (byte)66, (byte)21, (byte)63, (byte)108, (byte)151, (byte)250, (byte)221, (byte)50, (byte)189, (byte)229, (byte)101, (byte)40, (byte)254, (byte)127, (byte)71, (byte)252, (byte)64, (byte)236, (byte)213, (byte)31, (byte)58, (byte)96, (byte)99, (byte)172, (byte)110, (byte)138, (byte)107, (byte)186, (byte)85, (byte)97, (byte)37, (byte)97, (byte)75, (byte)254, (byte)86, (byte)189, (byte)237, (byte)66, (byte)151, (byte)178, (byte)221, (byte)107, (byte)6, (byte)147, (byte)175, (byte)21, (byte)88, (byte)155, (byte)13, (byte)219, (byte)39, (byte)88, (byte)17, (byte)79, (byte)249, (byte)240, (byte)140, (byte)36, (byte)201, (byte)36, (byte)38, (byte)248, (byte)113, (byte)163, (byte)213, (byte)254, (byte)161, (byte)112, (byte)254, (byte)172, (byte)149, (byte)205, (byte)64, (byte)240, (byte)3, (byte)126, (byte)72, (byte)236, (byte)179, (byte)19, (byte)184, (byte)152, (byte)217, (byte)188, (byte)59, (byte)118, (byte)33, (byte)9, (byte)61, (byte)110, (byte)100, (byte)142, (byte)244, (byte)78, (byte)152, (byte)89, (byte)15, (byte)63, (byte)69, (byte)160, (byte)25, (byte)73, (byte)55, (byte)70, (byte)71, (byte)219, (byte)61, (byte)179, (byte)133, (byte)109, (byte)133, (byte)0, (byte)21, (byte)10, (byte)137, (byte)29, (byte)136, (byte)163, (byte)96, (byte)194, (byte)117, (byte)17, (byte)16, (byte)89, (byte)251, (byte)247, (byte)212}, 0) ;
            CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDISTANCE_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.current_distance == (ushort)(ushort)51318);
                Debug.Assert(pack.max_distance == (ushort)(ushort)53701);
                Debug.Assert(pack.orientation == (MAV_SENSOR_ORIENTATION)MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_135);
                Debug.Assert(pack.min_distance == (ushort)(ushort)23678);
                Debug.Assert(pack.id == (byte)(byte)41);
                Debug.Assert(pack.covariance == (byte)(byte)223);
                Debug.Assert(pack.time_boot_ms == (uint)1394933335U);
                Debug.Assert(pack.type == (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN);
            };
            GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.current_distance = (ushort)(ushort)51318;
            p132.type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN;
            p132.id = (byte)(byte)41;
            p132.orientation = (MAV_SENSOR_ORIENTATION)MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_135;
            p132.max_distance = (ushort)(ushort)53701;
            p132.min_distance = (ushort)(ushort)23678;
            p132.covariance = (byte)(byte)223;
            p132.time_boot_ms = (uint)1394933335U;
            CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int)359286802);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)38774);
                Debug.Assert(pack.mask == (ulong)168772311734051219L);
                Debug.Assert(pack.lon == (int) -158425945);
            };
            GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.mask = (ulong)168772311734051219L;
            p133.lat = (int)359286802;
            p133.lon = (int) -158425945;
            p133.grid_spacing = (ushort)(ushort)38774;
            CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int)1070924746);
                Debug.Assert(pack.data_.SequenceEqual(new short[] {(short)13475, (short) -4790, (short) -4839, (short) -12441, (short)20857, (short) -9484, (short) -27761, (short) -8890, (short) -11985, (short) -14879, (short)21075, (short)21566, (short)9475, (short) -17450, (short)31609, (short) -17740}));
                Debug.Assert(pack.gridbit == (byte)(byte)50);
                Debug.Assert(pack.lat == (int)1610907320);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)37547);
            };
            GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.lat = (int)1610907320;
            p134.lon = (int)1070924746;
            p134.data__SET(new short[] {(short)13475, (short) -4790, (short) -4839, (short) -12441, (short)20857, (short) -9484, (short) -27761, (short) -8890, (short) -11985, (short) -14879, (short)21075, (short)21566, (short)9475, (short) -17450, (short)31609, (short) -17740}, 0) ;
            p134.grid_spacing = (ushort)(ushort)37547;
            p134.gridbit = (byte)(byte)50;
            CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_CHECKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int)208522464);
                Debug.Assert(pack.lat == (int) -301701137);
            };
            GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lat = (int) -301701137;
            p135.lon = (int)208522464;
            CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pending == (ushort)(ushort)57119);
                Debug.Assert(pack.spacing == (ushort)(ushort)12349);
                Debug.Assert(pack.current_height == (float)3.2757584E38F);
                Debug.Assert(pack.loaded == (ushort)(ushort)14530);
                Debug.Assert(pack.lon == (int)1670533651);
                Debug.Assert(pack.terrain_height == (float) -2.5858389E38F);
                Debug.Assert(pack.lat == (int) -1723909862);
            };
            GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.loaded = (ushort)(ushort)14530;
            p136.spacing = (ushort)(ushort)12349;
            p136.terrain_height = (float) -2.5858389E38F;
            p136.pending = (ushort)(ushort)57119;
            p136.lon = (int)1670533651;
            p136.current_height = (float)3.2757584E38F;
            p136.lat = (int) -1723909862;
            CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSURE2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_diff == (float) -2.168004E38F);
                Debug.Assert(pack.press_abs == (float)1.8676136E38F);
                Debug.Assert(pack.temperature == (short)(short) -10646);
                Debug.Assert(pack.time_boot_ms == (uint)1286250514U);
            };
            GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.time_boot_ms = (uint)1286250514U;
            p137.press_diff = (float) -2.168004E38F;
            p137.press_abs = (float)1.8676136E38F;
            p137.temperature = (short)(short) -10646;
            CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATT_POS_MOCAPReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q.SequenceEqual(new float[] {-3.251086E38F, -3.2218914E38F, 9.031704E37F, -2.7477749E38F}));
                Debug.Assert(pack.y == (float)6.5614893E37F);
                Debug.Assert(pack.z == (float)1.8271123E38F);
                Debug.Assert(pack.time_usec == (ulong)3174506362302597778L);
                Debug.Assert(pack.x == (float) -5.299866E37F);
            };
            GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.q_SET(new float[] {-3.251086E38F, -3.2218914E38F, 9.031704E37F, -2.7477749E38F}, 0) ;
            p138.time_usec = (ulong)3174506362302597778L;
            p138.y = (float)6.5614893E37F;
            p138.x = (float) -5.299866E37F;
            p138.z = (float)1.8271123E38F;
            CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)103);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-1.7379603E38F, 3.292653E37F, 8.650006E37F, -5.233613E37F, 2.9747796E38F, -1.4621542E38F, 3.8905358E37F, 3.2289865E36F}));
                Debug.Assert(pack.time_usec == (ulong)6498957872731525554L);
                Debug.Assert(pack.target_system == (byte)(byte)204);
                Debug.Assert(pack.group_mlx == (byte)(byte)15);
            };
            GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.target_system = (byte)(byte)204;
            p139.group_mlx = (byte)(byte)15;
            p139.target_component = (byte)(byte)103;
            p139.controls_SET(new float[] {-1.7379603E38F, 3.292653E37F, 8.650006E37F, -5.233613E37F, 2.9747796E38F, -1.4621542E38F, 3.8905358E37F, 3.2289865E36F}, 0) ;
            p139.time_usec = (ulong)6498957872731525554L;
            CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-2.179223E38F, -1.3982493E37F, -6.7200927E37F, 6.815743E37F, 6.536526E37F, 1.0664969E38F, 1.8519607E38F, 3.6004269E37F}));
                Debug.Assert(pack.time_usec == (ulong)6305219273559608778L);
                Debug.Assert(pack.group_mlx == (byte)(byte)94);
            };
            GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.controls_SET(new float[] {-2.179223E38F, -1.3982493E37F, -6.7200927E37F, 6.815743E37F, 6.536526E37F, 1.0664969E38F, 1.8519607E38F, 3.6004269E37F}, 0) ;
            p140.time_usec = (ulong)6305219273559608778L;
            p140.group_mlx = (byte)(byte)94;
            CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnALTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)1663952672983458803L);
                Debug.Assert(pack.altitude_terrain == (float)2.3363515E38F);
                Debug.Assert(pack.altitude_relative == (float)2.066993E38F);
                Debug.Assert(pack.altitude_monotonic == (float)3.2715478E38F);
                Debug.Assert(pack.bottom_clearance == (float) -2.4869498E38F);
                Debug.Assert(pack.altitude_amsl == (float) -1.01381517E37F);
                Debug.Assert(pack.altitude_local == (float)2.9623198E38F);
            };
            GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.time_usec = (ulong)1663952672983458803L;
            p141.bottom_clearance = (float) -2.4869498E38F;
            p141.altitude_local = (float)2.9623198E38F;
            p141.altitude_terrain = (float)2.3363515E38F;
            p141.altitude_monotonic = (float)3.2715478E38F;
            p141.altitude_relative = (float)2.066993E38F;
            p141.altitude_amsl = (float) -1.01381517E37F;
            CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRESOURCE_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.transfer_type == (byte)(byte)84);
                Debug.Assert(pack.uri_type == (byte)(byte)176);
                Debug.Assert(pack.request_id == (byte)(byte)106);
                Debug.Assert(pack.uri.SequenceEqual(new byte[] {(byte)65, (byte)149, (byte)187, (byte)27, (byte)68, (byte)219, (byte)211, (byte)205, (byte)102, (byte)148, (byte)209, (byte)57, (byte)230, (byte)211, (byte)158, (byte)107, (byte)228, (byte)165, (byte)189, (byte)98, (byte)64, (byte)157, (byte)167, (byte)152, (byte)22, (byte)103, (byte)8, (byte)43, (byte)168, (byte)8, (byte)159, (byte)44, (byte)154, (byte)224, (byte)143, (byte)207, (byte)28, (byte)43, (byte)122, (byte)113, (byte)196, (byte)115, (byte)196, (byte)216, (byte)217, (byte)64, (byte)162, (byte)144, (byte)195, (byte)48, (byte)158, (byte)53, (byte)29, (byte)234, (byte)90, (byte)73, (byte)191, (byte)85, (byte)205, (byte)143, (byte)26, (byte)234, (byte)6, (byte)4, (byte)224, (byte)137, (byte)213, (byte)4, (byte)41, (byte)161, (byte)167, (byte)239, (byte)225, (byte)252, (byte)173, (byte)119, (byte)147, (byte)134, (byte)27, (byte)144, (byte)188, (byte)98, (byte)11, (byte)230, (byte)220, (byte)22, (byte)245, (byte)63, (byte)175, (byte)2, (byte)247, (byte)33, (byte)87, (byte)148, (byte)125, (byte)28, (byte)18, (byte)190, (byte)96, (byte)75, (byte)72, (byte)138, (byte)96, (byte)46, (byte)126, (byte)163, (byte)235, (byte)202, (byte)231, (byte)131, (byte)133, (byte)106, (byte)64, (byte)152, (byte)96, (byte)55, (byte)126, (byte)2, (byte)141, (byte)253}));
                Debug.Assert(pack.storage.SequenceEqual(new byte[] {(byte)25, (byte)29, (byte)252, (byte)217, (byte)43, (byte)250, (byte)84, (byte)215, (byte)46, (byte)205, (byte)185, (byte)63, (byte)90, (byte)165, (byte)82, (byte)207, (byte)144, (byte)124, (byte)122, (byte)153, (byte)226, (byte)78, (byte)8, (byte)227, (byte)196, (byte)235, (byte)53, (byte)207, (byte)57, (byte)48, (byte)130, (byte)215, (byte)123, (byte)231, (byte)67, (byte)33, (byte)14, (byte)93, (byte)38, (byte)105, (byte)207, (byte)101, (byte)32, (byte)223, (byte)146, (byte)106, (byte)17, (byte)204, (byte)236, (byte)249, (byte)202, (byte)196, (byte)116, (byte)13, (byte)39, (byte)69, (byte)177, (byte)134, (byte)231, (byte)155, (byte)128, (byte)161, (byte)160, (byte)180, (byte)214, (byte)24, (byte)116, (byte)125, (byte)215, (byte)221, (byte)174, (byte)126, (byte)174, (byte)170, (byte)165, (byte)206, (byte)164, (byte)129, (byte)249, (byte)180, (byte)152, (byte)194, (byte)219, (byte)201, (byte)178, (byte)89, (byte)26, (byte)218, (byte)9, (byte)224, (byte)235, (byte)115, (byte)228, (byte)161, (byte)186, (byte)14, (byte)26, (byte)220, (byte)96, (byte)242, (byte)58, (byte)195, (byte)247, (byte)137, (byte)10, (byte)65, (byte)10, (byte)116, (byte)221, (byte)2, (byte)76, (byte)38, (byte)221, (byte)120, (byte)47, (byte)228, (byte)57, (byte)178, (byte)212, (byte)73}));
            };
            GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.request_id = (byte)(byte)106;
            p142.uri_SET(new byte[] {(byte)65, (byte)149, (byte)187, (byte)27, (byte)68, (byte)219, (byte)211, (byte)205, (byte)102, (byte)148, (byte)209, (byte)57, (byte)230, (byte)211, (byte)158, (byte)107, (byte)228, (byte)165, (byte)189, (byte)98, (byte)64, (byte)157, (byte)167, (byte)152, (byte)22, (byte)103, (byte)8, (byte)43, (byte)168, (byte)8, (byte)159, (byte)44, (byte)154, (byte)224, (byte)143, (byte)207, (byte)28, (byte)43, (byte)122, (byte)113, (byte)196, (byte)115, (byte)196, (byte)216, (byte)217, (byte)64, (byte)162, (byte)144, (byte)195, (byte)48, (byte)158, (byte)53, (byte)29, (byte)234, (byte)90, (byte)73, (byte)191, (byte)85, (byte)205, (byte)143, (byte)26, (byte)234, (byte)6, (byte)4, (byte)224, (byte)137, (byte)213, (byte)4, (byte)41, (byte)161, (byte)167, (byte)239, (byte)225, (byte)252, (byte)173, (byte)119, (byte)147, (byte)134, (byte)27, (byte)144, (byte)188, (byte)98, (byte)11, (byte)230, (byte)220, (byte)22, (byte)245, (byte)63, (byte)175, (byte)2, (byte)247, (byte)33, (byte)87, (byte)148, (byte)125, (byte)28, (byte)18, (byte)190, (byte)96, (byte)75, (byte)72, (byte)138, (byte)96, (byte)46, (byte)126, (byte)163, (byte)235, (byte)202, (byte)231, (byte)131, (byte)133, (byte)106, (byte)64, (byte)152, (byte)96, (byte)55, (byte)126, (byte)2, (byte)141, (byte)253}, 0) ;
            p142.transfer_type = (byte)(byte)84;
            p142.uri_type = (byte)(byte)176;
            p142.storage_SET(new byte[] {(byte)25, (byte)29, (byte)252, (byte)217, (byte)43, (byte)250, (byte)84, (byte)215, (byte)46, (byte)205, (byte)185, (byte)63, (byte)90, (byte)165, (byte)82, (byte)207, (byte)144, (byte)124, (byte)122, (byte)153, (byte)226, (byte)78, (byte)8, (byte)227, (byte)196, (byte)235, (byte)53, (byte)207, (byte)57, (byte)48, (byte)130, (byte)215, (byte)123, (byte)231, (byte)67, (byte)33, (byte)14, (byte)93, (byte)38, (byte)105, (byte)207, (byte)101, (byte)32, (byte)223, (byte)146, (byte)106, (byte)17, (byte)204, (byte)236, (byte)249, (byte)202, (byte)196, (byte)116, (byte)13, (byte)39, (byte)69, (byte)177, (byte)134, (byte)231, (byte)155, (byte)128, (byte)161, (byte)160, (byte)180, (byte)214, (byte)24, (byte)116, (byte)125, (byte)215, (byte)221, (byte)174, (byte)126, (byte)174, (byte)170, (byte)165, (byte)206, (byte)164, (byte)129, (byte)249, (byte)180, (byte)152, (byte)194, (byte)219, (byte)201, (byte)178, (byte)89, (byte)26, (byte)218, (byte)9, (byte)224, (byte)235, (byte)115, (byte)228, (byte)161, (byte)186, (byte)14, (byte)26, (byte)220, (byte)96, (byte)242, (byte)58, (byte)195, (byte)247, (byte)137, (byte)10, (byte)65, (byte)10, (byte)116, (byte)221, (byte)2, (byte)76, (byte)38, (byte)221, (byte)120, (byte)47, (byte)228, (byte)57, (byte)178, (byte)212, (byte)73}, 0) ;
            CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSURE3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short)17511);
                Debug.Assert(pack.time_boot_ms == (uint)2869165109U);
                Debug.Assert(pack.press_diff == (float) -3.3935222E38F);
                Debug.Assert(pack.press_abs == (float) -2.5298373E38F);
            };
            GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.press_diff = (float) -3.3935222E38F;
            p143.press_abs = (float) -2.5298373E38F;
            p143.temperature = (short)(short)17511;
            p143.time_boot_ms = (uint)2869165109U;
            CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnFOLLOW_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.position_cov.SequenceEqual(new float[] {-2.357114E38F, -1.4022978E38F, -1.9175343E38F}));
                Debug.Assert(pack.vel.SequenceEqual(new float[] {-4.360747E37F, 2.1569947E38F, -2.043385E38F}));
                Debug.Assert(pack.custom_state == (ulong)1937170206150675284L);
                Debug.Assert(pack.rates.SequenceEqual(new float[] {-3.3386025E38F, -5.908525E37F, -1.6832705E37F}));
                Debug.Assert(pack.est_capabilities == (byte)(byte)60);
                Debug.Assert(pack.timestamp == (ulong)8218081520316375819L);
                Debug.Assert(pack.lon == (int) -336504426);
                Debug.Assert(pack.acc.SequenceEqual(new float[] {5.5708746E37F, -1.9968474E38F, -2.1733747E35F}));
                Debug.Assert(pack.lat == (int) -856359437);
                Debug.Assert(pack.attitude_q.SequenceEqual(new float[] {1.4552155E38F, 3.9606244E36F, -1.9589475E38F, -7.6621944E37F}));
                Debug.Assert(pack.alt == (float) -3.2755607E37F);
            };
            GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.est_capabilities = (byte)(byte)60;
            p144.rates_SET(new float[] {-3.3386025E38F, -5.908525E37F, -1.6832705E37F}, 0) ;
            p144.timestamp = (ulong)8218081520316375819L;
            p144.custom_state = (ulong)1937170206150675284L;
            p144.attitude_q_SET(new float[] {1.4552155E38F, 3.9606244E36F, -1.9589475E38F, -7.6621944E37F}, 0) ;
            p144.lon = (int) -336504426;
            p144.vel_SET(new float[] {-4.360747E37F, 2.1569947E38F, -2.043385E38F}, 0) ;
            p144.acc_SET(new float[] {5.5708746E37F, -1.9968474E38F, -2.1733747E35F}, 0) ;
            p144.lat = (int) -856359437;
            p144.alt = (float) -3.2755607E37F;
            p144.position_cov_SET(new float[] {-2.357114E38F, -1.4022978E38F, -1.9175343E38F}, 0) ;
            CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCONTROL_SYSTEM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)9020441581439920401L);
                Debug.Assert(pack.y_vel == (float)2.5794887E38F);
                Debug.Assert(pack.yaw_rate == (float)2.4317903E38F);
                Debug.Assert(pack.roll_rate == (float) -1.6714909E38F);
                Debug.Assert(pack.y_pos == (float) -1.3896824E38F);
                Debug.Assert(pack.vel_variance.SequenceEqual(new float[] {2.5413912E38F, -1.3187413E38F, -1.6475311E38F}));
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.2234257E38F, 1.2023334E38F, -2.0040838E36F, -2.5061465E38F}));
                Debug.Assert(pack.z_acc == (float)2.7419982E38F);
                Debug.Assert(pack.x_acc == (float)1.7090123E38F);
                Debug.Assert(pack.pos_variance.SequenceEqual(new float[] {-1.0719349E38F, 6.717173E37F, 3.3543154E38F}));
                Debug.Assert(pack.pitch_rate == (float) -2.8069173E38F);
                Debug.Assert(pack.airspeed == (float) -1.1138233E38F);
                Debug.Assert(pack.z_vel == (float)1.5155012E38F);
                Debug.Assert(pack.z_pos == (float) -1.5010168E38F);
                Debug.Assert(pack.x_pos == (float) -3.253676E38F);
                Debug.Assert(pack.x_vel == (float) -1.8953683E38F);
                Debug.Assert(pack.y_acc == (float) -4.9922895E37F);
            };
            GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.y_acc = (float) -4.9922895E37F;
            p146.y_pos = (float) -1.3896824E38F;
            p146.pos_variance_SET(new float[] {-1.0719349E38F, 6.717173E37F, 3.3543154E38F}, 0) ;
            p146.z_pos = (float) -1.5010168E38F;
            p146.z_vel = (float)1.5155012E38F;
            p146.roll_rate = (float) -1.6714909E38F;
            p146.x_vel = (float) -1.8953683E38F;
            p146.time_usec = (ulong)9020441581439920401L;
            p146.x_pos = (float) -3.253676E38F;
            p146.q_SET(new float[] {-2.2234257E38F, 1.2023334E38F, -2.0040838E36F, -2.5061465E38F}, 0) ;
            p146.vel_variance_SET(new float[] {2.5413912E38F, -1.3187413E38F, -1.6475311E38F}, 0) ;
            p146.airspeed = (float) -1.1138233E38F;
            p146.z_acc = (float)2.7419982E38F;
            p146.yaw_rate = (float)2.4317903E38F;
            p146.pitch_rate = (float) -2.8069173E38F;
            p146.y_vel = (float)2.5794887E38F;
            p146.x_acc = (float)1.7090123E38F;
            CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnBATTERY_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.voltages.SequenceEqual(new ushort[] {(ushort)57054, (ushort)27787, (ushort)10491, (ushort)12566, (ushort)55906, (ushort)4596, (ushort)7402, (ushort)52508, (ushort)62572, (ushort)30997}));
                Debug.Assert(pack.id == (byte)(byte)100);
                Debug.Assert(pack.battery_function == (MAV_BATTERY_FUNCTION)MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_AVIONICS);
                Debug.Assert(pack.energy_consumed == (int)511180615);
                Debug.Assert(pack.current_battery == (short)(short)5221);
                Debug.Assert(pack.type == (MAV_BATTERY_TYPE)MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LION);
                Debug.Assert(pack.current_consumed == (int) -1955978741);
                Debug.Assert(pack.temperature == (short)(short) -28827);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte) - 48);
            };
            GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.current_consumed = (int) -1955978741;
            p147.voltages_SET(new ushort[] {(ushort)57054, (ushort)27787, (ushort)10491, (ushort)12566, (ushort)55906, (ushort)4596, (ushort)7402, (ushort)52508, (ushort)62572, (ushort)30997}, 0) ;
            p147.energy_consumed = (int)511180615;
            p147.type = (MAV_BATTERY_TYPE)MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LION;
            p147.current_battery = (short)(short)5221;
            p147.temperature = (short)(short) -28827;
            p147.id = (byte)(byte)100;
            p147.battery_function = (MAV_BATTERY_FUNCTION)MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_AVIONICS;
            p147.battery_remaining = (sbyte)(sbyte) - 48;
            CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnAUTOPILOT_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flight_sw_version == (uint)3884840535U);
                Debug.Assert(pack.middleware_custom_version.SequenceEqual(new byte[] {(byte)124, (byte)67, (byte)223, (byte)73, (byte)43, (byte)230, (byte)182, (byte)160}));
                Debug.Assert(pack.vendor_id == (ushort)(ushort)48704);
                Debug.Assert(pack.flight_custom_version.SequenceEqual(new byte[] {(byte)139, (byte)253, (byte)183, (byte)93, (byte)247, (byte)71, (byte)13, (byte)187}));
                Debug.Assert(pack.capabilities == (MAV_PROTOCOL_CAPABILITY)MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION);
                Debug.Assert(pack.board_version == (uint)1472731780U);
                Debug.Assert(pack.middleware_sw_version == (uint)1934656301U);
                Debug.Assert(pack.os_sw_version == (uint)2561623434U);
                Debug.Assert(pack.product_id == (ushort)(ushort)1904);
                Debug.Assert(pack.uid == (ulong)2899777272822001734L);
                Debug.Assert(pack.os_custom_version.SequenceEqual(new byte[] {(byte)53, (byte)242, (byte)104, (byte)218, (byte)1, (byte)124, (byte)117, (byte)123}));
                Debug.Assert(pack.uid2_TRY(ph).SequenceEqual(new byte[] {(byte)4, (byte)136, (byte)161, (byte)153, (byte)55, (byte)187, (byte)77, (byte)242, (byte)210, (byte)194, (byte)56, (byte)155, (byte)84, (byte)185, (byte)224, (byte)74, (byte)125, (byte)210}));
            };
            GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.vendor_id = (ushort)(ushort)48704;
            p148.flight_sw_version = (uint)3884840535U;
            p148.middleware_sw_version = (uint)1934656301U;
            p148.product_id = (ushort)(ushort)1904;
            p148.board_version = (uint)1472731780U;
            p148.uid2_SET(new byte[] {(byte)4, (byte)136, (byte)161, (byte)153, (byte)55, (byte)187, (byte)77, (byte)242, (byte)210, (byte)194, (byte)56, (byte)155, (byte)84, (byte)185, (byte)224, (byte)74, (byte)125, (byte)210}, 0, PH) ;
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY)MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION;
            p148.os_custom_version_SET(new byte[] {(byte)53, (byte)242, (byte)104, (byte)218, (byte)1, (byte)124, (byte)117, (byte)123}, 0) ;
            p148.flight_custom_version_SET(new byte[] {(byte)139, (byte)253, (byte)183, (byte)93, (byte)247, (byte)71, (byte)13, (byte)187}, 0) ;
            p148.os_sw_version = (uint)2561623434U;
            p148.middleware_custom_version_SET(new byte[] {(byte)124, (byte)67, (byte)223, (byte)73, (byte)43, (byte)230, (byte)182, (byte)160}, 0) ;
            p148.uid = (ulong)2899777272822001734L;
            CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLANDING_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x_TRY(ph) == (float)3.0739292E37F);
                Debug.Assert(pack.q_TRY(ph).SequenceEqual(new float[] {2.9709227E38F, -9.263198E37F, -5.1089575E37F, -8.1729E37F}));
                Debug.Assert(pack.y_TRY(ph) == (float)9.9387014E36F);
                Debug.Assert(pack.target_num == (byte)(byte)136);
                Debug.Assert(pack.angle_y == (float) -1.6902192E38F);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
                Debug.Assert(pack.position_valid_TRY(ph) == (byte)(byte)148);
                Debug.Assert(pack.type == (LANDING_TARGET_TYPE)LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON);
                Debug.Assert(pack.angle_x == (float)8.583485E37F);
                Debug.Assert(pack.size_y == (float)2.94556E38F);
                Debug.Assert(pack.time_usec == (ulong)3959196041839990098L);
                Debug.Assert(pack.z_TRY(ph) == (float)1.0346572E38F);
                Debug.Assert(pack.size_x == (float) -2.89045E38F);
                Debug.Assert(pack.distance == (float) -2.4432667E38F);
            };
            GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.y_SET((float)9.9387014E36F, PH) ;
            p149.q_SET(new float[] {2.9709227E38F, -9.263198E37F, -5.1089575E37F, -8.1729E37F}, 0, PH) ;
            p149.time_usec = (ulong)3959196041839990098L;
            p149.x_SET((float)3.0739292E37F, PH) ;
            p149.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
            p149.z_SET((float)1.0346572E38F, PH) ;
            p149.size_x = (float) -2.89045E38F;
            p149.position_valid_SET((byte)(byte)148, PH) ;
            p149.angle_x = (float)8.583485E37F;
            p149.target_num = (byte)(byte)136;
            p149.angle_y = (float) -1.6902192E38F;
            p149.size_y = (float)2.94556E38F;
            p149.distance = (float) -2.4432667E38F;
            p149.type = (LANDING_TARGET_TYPE)LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON;
            CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            ADV_TEST_CH.OnSENSOR_OFFSETSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.accel_cal_x == (float) -9.304168E37F);
                Debug.Assert(pack.gyro_cal_x == (float)1.6874189E38F);
                Debug.Assert(pack.mag_ofs_z == (short)(short)1893);
                Debug.Assert(pack.accel_cal_z == (float) -3.2850777E37F);
                Debug.Assert(pack.gyro_cal_y == (float) -2.7305908E38F);
                Debug.Assert(pack.raw_press == (int)1660062328);
                Debug.Assert(pack.gyro_cal_z == (float) -1.8008944E38F);
                Debug.Assert(pack.mag_ofs_y == (short)(short) -24067);
                Debug.Assert(pack.mag_declination == (float)1.2707965E35F);
                Debug.Assert(pack.mag_ofs_x == (short)(short) -3637);
                Debug.Assert(pack.accel_cal_y == (float) -2.4257198E38F);
                Debug.Assert(pack.raw_temp == (int)1934845359);
            };
            GroundControl.SENSOR_OFFSETS p150 = CommunicationChannel.new_SENSOR_OFFSETS();
            PH.setPack(p150);
            p150.raw_press = (int)1660062328;
            p150.accel_cal_x = (float) -9.304168E37F;
            p150.mag_ofs_z = (short)(short)1893;
            p150.mag_declination = (float)1.2707965E35F;
            p150.gyro_cal_z = (float) -1.8008944E38F;
            p150.gyro_cal_x = (float)1.6874189E38F;
            p150.mag_ofs_x = (short)(short) -3637;
            p150.accel_cal_z = (float) -3.2850777E37F;
            p150.raw_temp = (int)1934845359;
            p150.gyro_cal_y = (float) -2.7305908E38F;
            p150.accel_cal_y = (float) -2.4257198E38F;
            p150.mag_ofs_y = (short)(short) -24067;
            CommunicationChannel.instance.send(p150);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSET_MAG_OFFSETSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mag_ofs_y == (short)(short) -24430);
                Debug.Assert(pack.target_system == (byte)(byte)163);
                Debug.Assert(pack.mag_ofs_z == (short)(short) -21685);
                Debug.Assert(pack.mag_ofs_x == (short)(short)27269);
                Debug.Assert(pack.target_component == (byte)(byte)120);
            };
            GroundControl.SET_MAG_OFFSETS p151 = CommunicationChannel.new_SET_MAG_OFFSETS();
            PH.setPack(p151);
            p151.target_system = (byte)(byte)163;
            p151.target_component = (byte)(byte)120;
            p151.mag_ofs_y = (short)(short) -24430;
            p151.mag_ofs_z = (short)(short) -21685;
            p151.mag_ofs_x = (short)(short)27269;
            CommunicationChannel.instance.send(p151);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMEMINFOReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.freemem32_TRY(ph) == (uint)4085794130U);
                Debug.Assert(pack.brkval == (ushort)(ushort)42248);
                Debug.Assert(pack.freemem == (ushort)(ushort)10717);
            };
            GroundControl.MEMINFO p152 = CommunicationChannel.new_MEMINFO();
            PH.setPack(p152);
            p152.freemem = (ushort)(ushort)10717;
            p152.brkval = (ushort)(ushort)42248;
            p152.freemem32_SET((uint)4085794130U, PH) ;
            CommunicationChannel.instance.send(p152);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnAP_ADCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.adc3 == (ushort)(ushort)58002);
                Debug.Assert(pack.adc4 == (ushort)(ushort)19208);
                Debug.Assert(pack.adc2 == (ushort)(ushort)58914);
                Debug.Assert(pack.adc5 == (ushort)(ushort)9029);
                Debug.Assert(pack.adc1 == (ushort)(ushort)45622);
                Debug.Assert(pack.adc6 == (ushort)(ushort)53423);
            };
            GroundControl.AP_ADC p153 = CommunicationChannel.new_AP_ADC();
            PH.setPack(p153);
            p153.adc4 = (ushort)(ushort)19208;
            p153.adc6 = (ushort)(ushort)53423;
            p153.adc3 = (ushort)(ushort)58002;
            p153.adc2 = (ushort)(ushort)58914;
            p153.adc1 = (ushort)(ushort)45622;
            p153.adc5 = (ushort)(ushort)9029;
            CommunicationChannel.instance.send(p153);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDIGICAM_CONFIGUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)21);
                Debug.Assert(pack.engine_cut_off == (byte)(byte)202);
                Debug.Assert(pack.extra_value == (float)3.0404997E38F);
                Debug.Assert(pack.target_system == (byte)(byte)93);
                Debug.Assert(pack.exposure_type == (byte)(byte)46);
                Debug.Assert(pack.aperture == (byte)(byte)159);
                Debug.Assert(pack.mode == (byte)(byte)16);
                Debug.Assert(pack.command_id == (byte)(byte)84);
                Debug.Assert(pack.shutter_speed == (ushort)(ushort)45484);
                Debug.Assert(pack.iso == (byte)(byte)246);
                Debug.Assert(pack.extra_param == (byte)(byte)238);
            };
            GroundControl.DIGICAM_CONFIGURE p154 = CommunicationChannel.new_DIGICAM_CONFIGURE();
            PH.setPack(p154);
            p154.aperture = (byte)(byte)159;
            p154.iso = (byte)(byte)246;
            p154.exposure_type = (byte)(byte)46;
            p154.extra_value = (float)3.0404997E38F;
            p154.engine_cut_off = (byte)(byte)202;
            p154.target_system = (byte)(byte)93;
            p154.extra_param = (byte)(byte)238;
            p154.mode = (byte)(byte)16;
            p154.target_component = (byte)(byte)21;
            p154.command_id = (byte)(byte)84;
            p154.shutter_speed = (ushort)(ushort)45484;
            CommunicationChannel.instance.send(p154);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDIGICAM_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.focus_lock == (byte)(byte)108);
                Debug.Assert(pack.target_system == (byte)(byte)86);
                Debug.Assert(pack.command_id == (byte)(byte)133);
                Debug.Assert(pack.shot == (byte)(byte)71);
                Debug.Assert(pack.extra_value == (float)2.5338626E38F);
                Debug.Assert(pack.zoom_pos == (byte)(byte)18);
                Debug.Assert(pack.session == (byte)(byte)195);
                Debug.Assert(pack.zoom_step == (sbyte)(sbyte) - 34);
                Debug.Assert(pack.extra_param == (byte)(byte)15);
                Debug.Assert(pack.target_component == (byte)(byte)131);
            };
            GroundControl.DIGICAM_CONTROL p155 = CommunicationChannel.new_DIGICAM_CONTROL();
            PH.setPack(p155);
            p155.target_component = (byte)(byte)131;
            p155.extra_param = (byte)(byte)15;
            p155.target_system = (byte)(byte)86;
            p155.extra_value = (float)2.5338626E38F;
            p155.zoom_pos = (byte)(byte)18;
            p155.command_id = (byte)(byte)133;
            p155.zoom_step = (sbyte)(sbyte) - 34;
            p155.focus_lock = (byte)(byte)108;
            p155.session = (byte)(byte)195;
            p155.shot = (byte)(byte)71;
            CommunicationChannel.instance.send(p155);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMOUNT_CONFIGUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.stab_yaw == (byte)(byte)172);
                Debug.Assert(pack.stab_pitch == (byte)(byte)86);
                Debug.Assert(pack.target_system == (byte)(byte)136);
                Debug.Assert(pack.target_component == (byte)(byte)4);
                Debug.Assert(pack.mount_mode == (MAV_MOUNT_MODE)MAV_MOUNT_MODE.MAV_MOUNT_MODE_RC_TARGETING);
                Debug.Assert(pack.stab_roll == (byte)(byte)186);
            };
            GroundControl.MOUNT_CONFIGURE p156 = CommunicationChannel.new_MOUNT_CONFIGURE();
            PH.setPack(p156);
            p156.target_system = (byte)(byte)136;
            p156.stab_roll = (byte)(byte)186;
            p156.stab_pitch = (byte)(byte)86;
            p156.target_component = (byte)(byte)4;
            p156.mount_mode = (MAV_MOUNT_MODE)MAV_MOUNT_MODE.MAV_MOUNT_MODE_RC_TARGETING;
            p156.stab_yaw = (byte)(byte)172;
            CommunicationChannel.instance.send(p156);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMOUNT_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)172);
                Debug.Assert(pack.target_system == (byte)(byte)195);
                Debug.Assert(pack.input_c == (int) -2028858076);
                Debug.Assert(pack.input_a == (int) -107452380);
                Debug.Assert(pack.save_position == (byte)(byte)125);
                Debug.Assert(pack.input_b == (int)488135666);
            };
            GroundControl.MOUNT_CONTROL p157 = CommunicationChannel.new_MOUNT_CONTROL();
            PH.setPack(p157);
            p157.input_a = (int) -107452380;
            p157.save_position = (byte)(byte)125;
            p157.input_c = (int) -2028858076;
            p157.input_b = (int)488135666;
            p157.target_system = (byte)(byte)195;
            p157.target_component = (byte)(byte)172;
            CommunicationChannel.instance.send(p157);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMOUNT_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)188);
                Debug.Assert(pack.pointing_c == (int) -1853340568);
                Debug.Assert(pack.pointing_a == (int) -999559935);
                Debug.Assert(pack.target_system == (byte)(byte)165);
                Debug.Assert(pack.pointing_b == (int)446983474);
            };
            GroundControl.MOUNT_STATUS p158 = CommunicationChannel.new_MOUNT_STATUS();
            PH.setPack(p158);
            p158.target_component = (byte)(byte)188;
            p158.pointing_a = (int) -999559935;
            p158.pointing_b = (int)446983474;
            p158.pointing_c = (int) -1853340568;
            p158.target_system = (byte)(byte)165;
            CommunicationChannel.instance.send(p158);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnFENCE_POINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lng == (float) -1.243241E38F);
                Debug.Assert(pack.target_system == (byte)(byte)120);
                Debug.Assert(pack.count == (byte)(byte)158);
                Debug.Assert(pack.target_component == (byte)(byte)60);
                Debug.Assert(pack.idx == (byte)(byte)133);
                Debug.Assert(pack.lat == (float) -6.101323E37F);
            };
            GroundControl.FENCE_POINT p160 = CommunicationChannel.new_FENCE_POINT();
            PH.setPack(p160);
            p160.target_system = (byte)(byte)120;
            p160.lat = (float) -6.101323E37F;
            p160.idx = (byte)(byte)133;
            p160.target_component = (byte)(byte)60;
            p160.lng = (float) -1.243241E38F;
            p160.count = (byte)(byte)158;
            CommunicationChannel.instance.send(p160);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnFENCE_FETCH_POINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)209);
                Debug.Assert(pack.target_system == (byte)(byte)93);
                Debug.Assert(pack.idx == (byte)(byte)165);
            };
            GroundControl.FENCE_FETCH_POINT p161 = CommunicationChannel.new_FENCE_FETCH_POINT();
            PH.setPack(p161);
            p161.idx = (byte)(byte)165;
            p161.target_component = (byte)(byte)209;
            p161.target_system = (byte)(byte)93;
            CommunicationChannel.instance.send(p161);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnFENCE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.breach_status == (byte)(byte)30);
                Debug.Assert(pack.breach_type == (FENCE_BREACH)FENCE_BREACH.FENCE_BREACH_MINALT);
                Debug.Assert(pack.breach_count == (ushort)(ushort)634);
                Debug.Assert(pack.breach_time == (uint)800396509U);
            };
            GroundControl.FENCE_STATUS p162 = CommunicationChannel.new_FENCE_STATUS();
            PH.setPack(p162);
            p162.breach_count = (ushort)(ushort)634;
            p162.breach_type = (FENCE_BREACH)FENCE_BREACH.FENCE_BREACH_MINALT;
            p162.breach_status = (byte)(byte)30;
            p162.breach_time = (uint)800396509U;
            CommunicationChannel.instance.send(p162);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnAHRSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.omegaIz == (float)2.4620032E38F);
                Debug.Assert(pack.accel_weight == (float) -2.5871574E38F);
                Debug.Assert(pack.omegaIy == (float)3.358118E38F);
                Debug.Assert(pack.omegaIx == (float)1.5017603E38F);
                Debug.Assert(pack.error_yaw == (float) -1.3510561E38F);
                Debug.Assert(pack.renorm_val == (float) -2.2184587E38F);
                Debug.Assert(pack.error_rp == (float)2.2088994E38F);
            };
            GroundControl.AHRS p163 = CommunicationChannel.new_AHRS();
            PH.setPack(p163);
            p163.omegaIy = (float)3.358118E38F;
            p163.omegaIz = (float)2.4620032E38F;
            p163.omegaIx = (float)1.5017603E38F;
            p163.error_rp = (float)2.2088994E38F;
            p163.error_yaw = (float) -1.3510561E38F;
            p163.accel_weight = (float) -2.5871574E38F;
            p163.renorm_val = (float) -2.2184587E38F;
            CommunicationChannel.instance.send(p163);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSIMSTATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ygyro == (float)2.5338023E37F);
                Debug.Assert(pack.xgyro == (float) -4.8675213E36F);
                Debug.Assert(pack.lat == (int) -966954725);
                Debug.Assert(pack.lng == (int)1018426762);
                Debug.Assert(pack.yacc == (float) -3.1937587E38F);
                Debug.Assert(pack.zgyro == (float)2.086915E38F);
                Debug.Assert(pack.yaw == (float) -2.7278953E38F);
                Debug.Assert(pack.xacc == (float) -2.5429365E38F);
                Debug.Assert(pack.roll == (float)2.0162687E38F);
                Debug.Assert(pack.zacc == (float)9.1468565E36F);
                Debug.Assert(pack.pitch == (float) -3.088168E38F);
            };
            GroundControl.SIMSTATE p164 = CommunicationChannel.new_SIMSTATE();
            PH.setPack(p164);
            p164.lat = (int) -966954725;
            p164.yaw = (float) -2.7278953E38F;
            p164.roll = (float)2.0162687E38F;
            p164.lng = (int)1018426762;
            p164.xgyro = (float) -4.8675213E36F;
            p164.yacc = (float) -3.1937587E38F;
            p164.ygyro = (float)2.5338023E37F;
            p164.xacc = (float) -2.5429365E38F;
            p164.zgyro = (float)2.086915E38F;
            p164.pitch = (float) -3.088168E38F;
            p164.zacc = (float)9.1468565E36F;
            CommunicationChannel.instance.send(p164);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnHWSTATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.I2Cerr == (byte)(byte)190);
                Debug.Assert(pack.Vcc == (ushort)(ushort)35839);
            };
            GroundControl.HWSTATUS p165 = CommunicationChannel.new_HWSTATUS();
            PH.setPack(p165);
            p165.Vcc = (ushort)(ushort)35839;
            p165.I2Cerr = (byte)(byte)190;
            CommunicationChannel.instance.send(p165);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnRADIOReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.txbuf == (byte)(byte)62);
                Debug.Assert(pack.noise == (byte)(byte)206);
                Debug.Assert(pack.fixed_ == (ushort)(ushort)52146);
                Debug.Assert(pack.remrssi == (byte)(byte)22);
                Debug.Assert(pack.remnoise == (byte)(byte)160);
                Debug.Assert(pack.rssi == (byte)(byte)203);
                Debug.Assert(pack.rxerrors == (ushort)(ushort)19004);
            };
            GroundControl.RADIO p166 = CommunicationChannel.new_RADIO();
            PH.setPack(p166);
            p166.rxerrors = (ushort)(ushort)19004;
            p166.fixed_ = (ushort)(ushort)52146;
            p166.txbuf = (byte)(byte)62;
            p166.remnoise = (byte)(byte)160;
            p166.remrssi = (byte)(byte)22;
            p166.rssi = (byte)(byte)203;
            p166.noise = (byte)(byte)206;
            CommunicationChannel.instance.send(p166);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLIMITS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.limits_state == (LIMITS_STATE)LIMITS_STATE.LIMITS_TRIGGERED);
                Debug.Assert(pack.breach_count == (ushort)(ushort)46842);
                Debug.Assert(pack.last_recovery == (uint)2088263286U);
                Debug.Assert(pack.mods_enabled == (LIMIT_MODULE)LIMIT_MODULE.LIMIT_GPSLOCK);
                Debug.Assert(pack.last_action == (uint)3622538293U);
                Debug.Assert(pack.mods_triggered == (LIMIT_MODULE)LIMIT_MODULE.LIMIT_GEOFENCE);
                Debug.Assert(pack.mods_required == (LIMIT_MODULE)LIMIT_MODULE.LIMIT_GPSLOCK);
                Debug.Assert(pack.last_trigger == (uint)3190665137U);
                Debug.Assert(pack.last_clear == (uint)3190473020U);
            };
            GroundControl.LIMITS_STATUS p167 = CommunicationChannel.new_LIMITS_STATUS();
            PH.setPack(p167);
            p167.mods_required = (LIMIT_MODULE)LIMIT_MODULE.LIMIT_GPSLOCK;
            p167.last_recovery = (uint)2088263286U;
            p167.last_clear = (uint)3190473020U;
            p167.last_trigger = (uint)3190665137U;
            p167.breach_count = (ushort)(ushort)46842;
            p167.limits_state = (LIMITS_STATE)LIMITS_STATE.LIMITS_TRIGGERED;
            p167.mods_enabled = (LIMIT_MODULE)LIMIT_MODULE.LIMIT_GPSLOCK;
            p167.last_action = (uint)3622538293U;
            p167.mods_triggered = (LIMIT_MODULE)LIMIT_MODULE.LIMIT_GEOFENCE;
            CommunicationChannel.instance.send(p167);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnWINDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.speed_z == (float)9.797655E37F);
                Debug.Assert(pack.speed == (float) -1.0774356E38F);
                Debug.Assert(pack.direction == (float) -1.6982036E38F);
            };
            GroundControl.WIND p168 = CommunicationChannel.new_WIND();
            PH.setPack(p168);
            p168.speed_z = (float)9.797655E37F;
            p168.speed = (float) -1.0774356E38F;
            p168.direction = (float) -1.6982036E38F;
            CommunicationChannel.instance.send(p168);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDATA16Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)205, (byte)17, (byte)248, (byte)166, (byte)168, (byte)72, (byte)218, (byte)11, (byte)38, (byte)20, (byte)148, (byte)113, (byte)102, (byte)180, (byte)117, (byte)253}));
                Debug.Assert(pack.len == (byte)(byte)180);
                Debug.Assert(pack.type == (byte)(byte)190);
            };
            GroundControl.DATA16 p169 = CommunicationChannel.new_DATA16();
            PH.setPack(p169);
            p169.type = (byte)(byte)190;
            p169.len = (byte)(byte)180;
            p169.data__SET(new byte[] {(byte)205, (byte)17, (byte)248, (byte)166, (byte)168, (byte)72, (byte)218, (byte)11, (byte)38, (byte)20, (byte)148, (byte)113, (byte)102, (byte)180, (byte)117, (byte)253}, 0) ;
            CommunicationChannel.instance.send(p169);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDATA32Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type == (byte)(byte)23);
                Debug.Assert(pack.len == (byte)(byte)177);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)22, (byte)165, (byte)32, (byte)223, (byte)207, (byte)3, (byte)58, (byte)177, (byte)22, (byte)17, (byte)230, (byte)2, (byte)104, (byte)161, (byte)16, (byte)36, (byte)224, (byte)3, (byte)116, (byte)121, (byte)53, (byte)237, (byte)100, (byte)96, (byte)81, (byte)175, (byte)75, (byte)214, (byte)146, (byte)198, (byte)31, (byte)167}));
            };
            GroundControl.DATA32 p170 = CommunicationChannel.new_DATA32();
            PH.setPack(p170);
            p170.data__SET(new byte[] {(byte)22, (byte)165, (byte)32, (byte)223, (byte)207, (byte)3, (byte)58, (byte)177, (byte)22, (byte)17, (byte)230, (byte)2, (byte)104, (byte)161, (byte)16, (byte)36, (byte)224, (byte)3, (byte)116, (byte)121, (byte)53, (byte)237, (byte)100, (byte)96, (byte)81, (byte)175, (byte)75, (byte)214, (byte)146, (byte)198, (byte)31, (byte)167}, 0) ;
            p170.len = (byte)(byte)177;
            p170.type = (byte)(byte)23;
            CommunicationChannel.instance.send(p170);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDATA64Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.len == (byte)(byte)148);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)118, (byte)126, (byte)214, (byte)28, (byte)96, (byte)251, (byte)85, (byte)240, (byte)227, (byte)245, (byte)254, (byte)31, (byte)145, (byte)82, (byte)77, (byte)214, (byte)139, (byte)243, (byte)41, (byte)67, (byte)37, (byte)47, (byte)11, (byte)46, (byte)134, (byte)174, (byte)118, (byte)141, (byte)238, (byte)164, (byte)30, (byte)61, (byte)6, (byte)178, (byte)238, (byte)118, (byte)252, (byte)236, (byte)101, (byte)72, (byte)253, (byte)243, (byte)245, (byte)151, (byte)86, (byte)203, (byte)176, (byte)134, (byte)173, (byte)32, (byte)202, (byte)164, (byte)136, (byte)144, (byte)43, (byte)164, (byte)191, (byte)182, (byte)170, (byte)59, (byte)108, (byte)191, (byte)24, (byte)244}));
                Debug.Assert(pack.type == (byte)(byte)76);
            };
            GroundControl.DATA64 p171 = CommunicationChannel.new_DATA64();
            PH.setPack(p171);
            p171.data__SET(new byte[] {(byte)118, (byte)126, (byte)214, (byte)28, (byte)96, (byte)251, (byte)85, (byte)240, (byte)227, (byte)245, (byte)254, (byte)31, (byte)145, (byte)82, (byte)77, (byte)214, (byte)139, (byte)243, (byte)41, (byte)67, (byte)37, (byte)47, (byte)11, (byte)46, (byte)134, (byte)174, (byte)118, (byte)141, (byte)238, (byte)164, (byte)30, (byte)61, (byte)6, (byte)178, (byte)238, (byte)118, (byte)252, (byte)236, (byte)101, (byte)72, (byte)253, (byte)243, (byte)245, (byte)151, (byte)86, (byte)203, (byte)176, (byte)134, (byte)173, (byte)32, (byte)202, (byte)164, (byte)136, (byte)144, (byte)43, (byte)164, (byte)191, (byte)182, (byte)170, (byte)59, (byte)108, (byte)191, (byte)24, (byte)244}, 0) ;
            p171.len = (byte)(byte)148;
            p171.type = (byte)(byte)76;
            CommunicationChannel.instance.send(p171);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDATA96Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type == (byte)(byte)74);
                Debug.Assert(pack.len == (byte)(byte)191);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)224, (byte)162, (byte)175, (byte)247, (byte)22, (byte)173, (byte)22, (byte)126, (byte)71, (byte)16, (byte)191, (byte)223, (byte)59, (byte)226, (byte)13, (byte)131, (byte)85, (byte)176, (byte)189, (byte)229, (byte)103, (byte)226, (byte)199, (byte)52, (byte)88, (byte)95, (byte)25, (byte)150, (byte)49, (byte)145, (byte)105, (byte)68, (byte)224, (byte)34, (byte)89, (byte)114, (byte)2, (byte)16, (byte)155, (byte)119, (byte)139, (byte)92, (byte)241, (byte)140, (byte)65, (byte)193, (byte)119, (byte)71, (byte)193, (byte)56, (byte)13, (byte)93, (byte)18, (byte)4, (byte)183, (byte)43, (byte)147, (byte)163, (byte)51, (byte)24, (byte)111, (byte)191, (byte)215, (byte)33, (byte)135, (byte)66, (byte)30, (byte)201, (byte)162, (byte)237, (byte)93, (byte)169, (byte)170, (byte)59, (byte)65, (byte)52, (byte)63, (byte)38, (byte)202, (byte)45, (byte)221, (byte)173, (byte)215, (byte)241, (byte)202, (byte)56, (byte)232, (byte)212, (byte)254, (byte)167, (byte)187, (byte)147, (byte)221, (byte)184, (byte)98, (byte)22}));
            };
            GroundControl.DATA96 p172 = CommunicationChannel.new_DATA96();
            PH.setPack(p172);
            p172.data__SET(new byte[] {(byte)224, (byte)162, (byte)175, (byte)247, (byte)22, (byte)173, (byte)22, (byte)126, (byte)71, (byte)16, (byte)191, (byte)223, (byte)59, (byte)226, (byte)13, (byte)131, (byte)85, (byte)176, (byte)189, (byte)229, (byte)103, (byte)226, (byte)199, (byte)52, (byte)88, (byte)95, (byte)25, (byte)150, (byte)49, (byte)145, (byte)105, (byte)68, (byte)224, (byte)34, (byte)89, (byte)114, (byte)2, (byte)16, (byte)155, (byte)119, (byte)139, (byte)92, (byte)241, (byte)140, (byte)65, (byte)193, (byte)119, (byte)71, (byte)193, (byte)56, (byte)13, (byte)93, (byte)18, (byte)4, (byte)183, (byte)43, (byte)147, (byte)163, (byte)51, (byte)24, (byte)111, (byte)191, (byte)215, (byte)33, (byte)135, (byte)66, (byte)30, (byte)201, (byte)162, (byte)237, (byte)93, (byte)169, (byte)170, (byte)59, (byte)65, (byte)52, (byte)63, (byte)38, (byte)202, (byte)45, (byte)221, (byte)173, (byte)215, (byte)241, (byte)202, (byte)56, (byte)232, (byte)212, (byte)254, (byte)167, (byte)187, (byte)147, (byte)221, (byte)184, (byte)98, (byte)22}, 0) ;
            p172.type = (byte)(byte)74;
            p172.len = (byte)(byte)191;
            CommunicationChannel.instance.send(p172);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnRANGEFINDERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.distance == (float)2.4980783E38F);
                Debug.Assert(pack.voltage == (float) -2.6541675E38F);
            };
            GroundControl.RANGEFINDER p173 = CommunicationChannel.new_RANGEFINDER();
            PH.setPack(p173);
            p173.distance = (float)2.4980783E38F;
            p173.voltage = (float) -2.6541675E38F;
            CommunicationChannel.instance.send(p173);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnAIRSPEED_AUTOCALReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.EAS2TAS == (float)2.248595E38F);
                Debug.Assert(pack.Pby == (float)3.2895835E38F);
                Debug.Assert(pack.Pax == (float) -2.396954E38F);
                Debug.Assert(pack.state_z == (float)1.545123E37F);
                Debug.Assert(pack.diff_pressure == (float)8.963423E37F);
                Debug.Assert(pack.vz == (float) -2.3089726E38F);
                Debug.Assert(pack.Pcz == (float) -1.9858958E38F);
                Debug.Assert(pack.state_y == (float)2.634116E38F);
                Debug.Assert(pack.state_x == (float) -5.2900753E37F);
                Debug.Assert(pack.ratio == (float)1.6418445E38F);
                Debug.Assert(pack.vx == (float) -3.3003046E38F);
                Debug.Assert(pack.vy == (float) -3.8437045E37F);
            };
            GroundControl.AIRSPEED_AUTOCAL p174 = CommunicationChannel.new_AIRSPEED_AUTOCAL();
            PH.setPack(p174);
            p174.state_y = (float)2.634116E38F;
            p174.vz = (float) -2.3089726E38F;
            p174.vy = (float) -3.8437045E37F;
            p174.vx = (float) -3.3003046E38F;
            p174.Pby = (float)3.2895835E38F;
            p174.diff_pressure = (float)8.963423E37F;
            p174.Pax = (float) -2.396954E38F;
            p174.ratio = (float)1.6418445E38F;
            p174.state_x = (float) -5.2900753E37F;
            p174.EAS2TAS = (float)2.248595E38F;
            p174.state_z = (float)1.545123E37F;
            p174.Pcz = (float) -1.9858958E38F;
            CommunicationChannel.instance.send(p174);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnRALLY_POINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)66);
                Debug.Assert(pack.alt == (short)(short)21383);
                Debug.Assert(pack.lng == (int) -793511109);
                Debug.Assert(pack.lat == (int) -1559952535);
                Debug.Assert(pack.idx == (byte)(byte)9);
                Debug.Assert(pack.target_system == (byte)(byte)131);
                Debug.Assert(pack.flags == (RALLY_FLAGS)RALLY_FLAGS.LAND_IMMEDIATELY);
                Debug.Assert(pack.count == (byte)(byte)127);
                Debug.Assert(pack.land_dir == (ushort)(ushort)62779);
                Debug.Assert(pack.break_alt == (short)(short) -5494);
            };
            GroundControl.RALLY_POINT p175 = CommunicationChannel.new_RALLY_POINT();
            PH.setPack(p175);
            p175.lng = (int) -793511109;
            p175.target_component = (byte)(byte)66;
            p175.land_dir = (ushort)(ushort)62779;
            p175.count = (byte)(byte)127;
            p175.flags = (RALLY_FLAGS)RALLY_FLAGS.LAND_IMMEDIATELY;
            p175.lat = (int) -1559952535;
            p175.break_alt = (short)(short) -5494;
            p175.target_system = (byte)(byte)131;
            p175.alt = (short)(short)21383;
            p175.idx = (byte)(byte)9;
            CommunicationChannel.instance.send(p175);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnRALLY_FETCH_POINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)172);
                Debug.Assert(pack.idx == (byte)(byte)199);
                Debug.Assert(pack.target_system == (byte)(byte)113);
            };
            GroundControl.RALLY_FETCH_POINT p176 = CommunicationChannel.new_RALLY_FETCH_POINT();
            PH.setPack(p176);
            p176.target_system = (byte)(byte)113;
            p176.idx = (byte)(byte)199;
            p176.target_component = (byte)(byte)172;
            CommunicationChannel.instance.send(p176);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCOMPASSMOT_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.interference == (ushort)(ushort)60931);
                Debug.Assert(pack.current == (float) -2.8027968E37F);
                Debug.Assert(pack.CompensationZ == (float)3.2002663E38F);
                Debug.Assert(pack.throttle == (ushort)(ushort)6898);
                Debug.Assert(pack.CompensationY == (float)2.8311246E38F);
                Debug.Assert(pack.CompensationX == (float) -1.1327868E38F);
            };
            GroundControl.COMPASSMOT_STATUS p177 = CommunicationChannel.new_COMPASSMOT_STATUS();
            PH.setPack(p177);
            p177.CompensationY = (float)2.8311246E38F;
            p177.interference = (ushort)(ushort)60931;
            p177.CompensationZ = (float)3.2002663E38F;
            p177.throttle = (ushort)(ushort)6898;
            p177.current = (float) -2.8027968E37F;
            p177.CompensationX = (float) -1.1327868E38F;
            CommunicationChannel.instance.send(p177);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnAHRS2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch == (float) -3.2096376E38F);
                Debug.Assert(pack.lng == (int)1826803635);
                Debug.Assert(pack.lat == (int)289586637);
                Debug.Assert(pack.yaw == (float)4.614619E37F);
                Debug.Assert(pack.altitude == (float)8.277566E37F);
                Debug.Assert(pack.roll == (float) -2.1676405E38F);
            };
            GroundControl.AHRS2 p178 = CommunicationChannel.new_AHRS2();
            PH.setPack(p178);
            p178.lng = (int)1826803635;
            p178.roll = (float) -2.1676405E38F;
            p178.lat = (int)289586637;
            p178.yaw = (float)4.614619E37F;
            p178.altitude = (float)8.277566E37F;
            p178.pitch = (float) -3.2096376E38F;
            CommunicationChannel.instance.send(p178);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p3 == (float) -3.065511E38F);
                Debug.Assert(pack.p2 == (float)3.2032247E38F);
                Debug.Assert(pack.event_id == (CAMERA_STATUS_TYPES)CAMERA_STATUS_TYPES.CAMERA_STATUS_TYPE_LOWSTOREV);
                Debug.Assert(pack.cam_idx == (byte)(byte)210);
                Debug.Assert(pack.target_system == (byte)(byte)38);
                Debug.Assert(pack.p1 == (float) -6.0612E37F);
                Debug.Assert(pack.p4 == (float) -1.0313155E38F);
                Debug.Assert(pack.img_idx == (ushort)(ushort)37980);
                Debug.Assert(pack.time_usec == (ulong)8383932769819573037L);
            };
            GroundControl.CAMERA_STATUS p179 = CommunicationChannel.new_CAMERA_STATUS();
            PH.setPack(p179);
            p179.p3 = (float) -3.065511E38F;
            p179.p2 = (float)3.2032247E38F;
            p179.cam_idx = (byte)(byte)210;
            p179.event_id = (CAMERA_STATUS_TYPES)CAMERA_STATUS_TYPES.CAMERA_STATUS_TYPE_LOWSTOREV;
            p179.p1 = (float) -6.0612E37F;
            p179.time_usec = (ulong)8383932769819573037L;
            p179.target_system = (byte)(byte)38;
            p179.img_idx = (ushort)(ushort)37980;
            p179.p4 = (float) -1.0313155E38F;
            CommunicationChannel.instance.send(p179);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_FEEDBACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int)1230334809);
                Debug.Assert(pack.time_usec == (ulong)510505311091801833L);
                Debug.Assert(pack.lng == (int)481812878);
                Debug.Assert(pack.flags == (CAMERA_FEEDBACK_FLAGS)CAMERA_FEEDBACK_FLAGS.CAMERA_FEEDBACK_PHOTO);
                Debug.Assert(pack.roll == (float) -1.1056235E38F);
                Debug.Assert(pack.pitch == (float)4.413167E37F);
                Debug.Assert(pack.yaw == (float) -4.223083E37F);
                Debug.Assert(pack.cam_idx == (byte)(byte)86);
                Debug.Assert(pack.img_idx == (ushort)(ushort)51676);
                Debug.Assert(pack.target_system == (byte)(byte)93);
                Debug.Assert(pack.alt_rel == (float) -3.4216126E37F);
                Debug.Assert(pack.foc_len == (float) -1.3135811E38F);
                Debug.Assert(pack.alt_msl == (float)1.7372478E38F);
            };
            GroundControl.CAMERA_FEEDBACK p180 = CommunicationChannel.new_CAMERA_FEEDBACK();
            PH.setPack(p180);
            p180.foc_len = (float) -1.3135811E38F;
            p180.img_idx = (ushort)(ushort)51676;
            p180.cam_idx = (byte)(byte)86;
            p180.alt_msl = (float)1.7372478E38F;
            p180.time_usec = (ulong)510505311091801833L;
            p180.lat = (int)1230334809;
            p180.yaw = (float) -4.223083E37F;
            p180.lng = (int)481812878;
            p180.flags = (CAMERA_FEEDBACK_FLAGS)CAMERA_FEEDBACK_FLAGS.CAMERA_FEEDBACK_PHOTO;
            p180.roll = (float) -1.1056235E38F;
            p180.alt_rel = (float) -3.4216126E37F;
            p180.pitch = (float)4.413167E37F;
            p180.target_system = (byte)(byte)93;
            CommunicationChannel.instance.send(p180);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnBATTERY2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.voltage == (ushort)(ushort)11038);
                Debug.Assert(pack.current_battery == (short)(short) -26725);
            };
            GroundControl.BATTERY2 p181 = CommunicationChannel.new_BATTERY2();
            PH.setPack(p181);
            p181.voltage = (ushort)(ushort)11038;
            p181.current_battery = (short)(short) -26725;
            CommunicationChannel.instance.send(p181);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnAHRS3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int) -1854589786);
                Debug.Assert(pack.v4 == (float)2.9799377E38F);
                Debug.Assert(pack.roll == (float) -7.271324E37F);
                Debug.Assert(pack.lng == (int) -1191735111);
                Debug.Assert(pack.yaw == (float)8.672479E37F);
                Debug.Assert(pack.v2 == (float) -2.2924242E38F);
                Debug.Assert(pack.v3 == (float)2.418589E37F);
                Debug.Assert(pack.v1 == (float)2.6318982E38F);
                Debug.Assert(pack.altitude == (float) -2.8883E38F);
                Debug.Assert(pack.pitch == (float) -2.1466795E38F);
            };
            GroundControl.AHRS3 p182 = CommunicationChannel.new_AHRS3();
            PH.setPack(p182);
            p182.v1 = (float)2.6318982E38F;
            p182.roll = (float) -7.271324E37F;
            p182.v3 = (float)2.418589E37F;
            p182.yaw = (float)8.672479E37F;
            p182.v2 = (float) -2.2924242E38F;
            p182.pitch = (float) -2.1466795E38F;
            p182.lat = (int) -1854589786;
            p182.lng = (int) -1191735111;
            p182.altitude = (float) -2.8883E38F;
            p182.v4 = (float)2.9799377E38F;
            CommunicationChannel.instance.send(p182);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnAUTOPILOT_VERSION_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)38);
                Debug.Assert(pack.target_component == (byte)(byte)251);
            };
            GroundControl.AUTOPILOT_VERSION_REQUEST p183 = CommunicationChannel.new_AUTOPILOT_VERSION_REQUEST();
            PH.setPack(p183);
            p183.target_system = (byte)(byte)38;
            p183.target_component = (byte)(byte)251;
            CommunicationChannel.instance.send(p183);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnREMOTE_LOG_DATA_BLOCKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)212);
                Debug.Assert(pack.target_system == (byte)(byte)205);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)49, (byte)127, (byte)214, (byte)120, (byte)66, (byte)17, (byte)57, (byte)191, (byte)138, (byte)156, (byte)78, (byte)102, (byte)54, (byte)36, (byte)245, (byte)29, (byte)96, (byte)6, (byte)90, (byte)140, (byte)150, (byte)219, (byte)66, (byte)108, (byte)230, (byte)110, (byte)214, (byte)178, (byte)60, (byte)146, (byte)137, (byte)255, (byte)79, (byte)128, (byte)1, (byte)51, (byte)16, (byte)209, (byte)24, (byte)142, (byte)173, (byte)248, (byte)25, (byte)25, (byte)27, (byte)162, (byte)124, (byte)14, (byte)200, (byte)70, (byte)98, (byte)118, (byte)83, (byte)240, (byte)34, (byte)7, (byte)121, (byte)32, (byte)190, (byte)233, (byte)167, (byte)33, (byte)13, (byte)80, (byte)164, (byte)99, (byte)184, (byte)55, (byte)208, (byte)100, (byte)62, (byte)109, (byte)5, (byte)220, (byte)147, (byte)232, (byte)28, (byte)7, (byte)243, (byte)199, (byte)228, (byte)211, (byte)97, (byte)206, (byte)241, (byte)143, (byte)112, (byte)43, (byte)56, (byte)95, (byte)179, (byte)145, (byte)0, (byte)162, (byte)103, (byte)247, (byte)204, (byte)192, (byte)104, (byte)61, (byte)250, (byte)24, (byte)33, (byte)180, (byte)139, (byte)76, (byte)73, (byte)65, (byte)194, (byte)186, (byte)64, (byte)57, (byte)157, (byte)143, (byte)26, (byte)27, (byte)235, (byte)27, (byte)43, (byte)196, (byte)247, (byte)39, (byte)199, (byte)42, (byte)31, (byte)178, (byte)17, (byte)93, (byte)2, (byte)92, (byte)18, (byte)95, (byte)182, (byte)182, (byte)238, (byte)35, (byte)236, (byte)173, (byte)196, (byte)17, (byte)175, (byte)176, (byte)105, (byte)195, (byte)13, (byte)84, (byte)53, (byte)203, (byte)229, (byte)173, (byte)101, (byte)252, (byte)164, (byte)23, (byte)0, (byte)146, (byte)143, (byte)140, (byte)49, (byte)229, (byte)9, (byte)237, (byte)39, (byte)145, (byte)181, (byte)67, (byte)44, (byte)187, (byte)57, (byte)62, (byte)30, (byte)223, (byte)48, (byte)42, (byte)187, (byte)87, (byte)35, (byte)16, (byte)114, (byte)25, (byte)194, (byte)241, (byte)30, (byte)93, (byte)65, (byte)21, (byte)195, (byte)238, (byte)152, (byte)252, (byte)231, (byte)45, (byte)217, (byte)234, (byte)165, (byte)252, (byte)23, (byte)221, (byte)140, (byte)158}));
                Debug.Assert(pack.seqno == (MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS)MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS.MAV_REMOTE_LOG_DATA_BLOCK_STOP);
            };
            GroundControl.REMOTE_LOG_DATA_BLOCK p184 = CommunicationChannel.new_REMOTE_LOG_DATA_BLOCK();
            PH.setPack(p184);
            p184.target_system = (byte)(byte)205;
            p184.seqno = (MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS)MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS.MAV_REMOTE_LOG_DATA_BLOCK_STOP;
            p184.target_component = (byte)(byte)212;
            p184.data__SET(new byte[] {(byte)49, (byte)127, (byte)214, (byte)120, (byte)66, (byte)17, (byte)57, (byte)191, (byte)138, (byte)156, (byte)78, (byte)102, (byte)54, (byte)36, (byte)245, (byte)29, (byte)96, (byte)6, (byte)90, (byte)140, (byte)150, (byte)219, (byte)66, (byte)108, (byte)230, (byte)110, (byte)214, (byte)178, (byte)60, (byte)146, (byte)137, (byte)255, (byte)79, (byte)128, (byte)1, (byte)51, (byte)16, (byte)209, (byte)24, (byte)142, (byte)173, (byte)248, (byte)25, (byte)25, (byte)27, (byte)162, (byte)124, (byte)14, (byte)200, (byte)70, (byte)98, (byte)118, (byte)83, (byte)240, (byte)34, (byte)7, (byte)121, (byte)32, (byte)190, (byte)233, (byte)167, (byte)33, (byte)13, (byte)80, (byte)164, (byte)99, (byte)184, (byte)55, (byte)208, (byte)100, (byte)62, (byte)109, (byte)5, (byte)220, (byte)147, (byte)232, (byte)28, (byte)7, (byte)243, (byte)199, (byte)228, (byte)211, (byte)97, (byte)206, (byte)241, (byte)143, (byte)112, (byte)43, (byte)56, (byte)95, (byte)179, (byte)145, (byte)0, (byte)162, (byte)103, (byte)247, (byte)204, (byte)192, (byte)104, (byte)61, (byte)250, (byte)24, (byte)33, (byte)180, (byte)139, (byte)76, (byte)73, (byte)65, (byte)194, (byte)186, (byte)64, (byte)57, (byte)157, (byte)143, (byte)26, (byte)27, (byte)235, (byte)27, (byte)43, (byte)196, (byte)247, (byte)39, (byte)199, (byte)42, (byte)31, (byte)178, (byte)17, (byte)93, (byte)2, (byte)92, (byte)18, (byte)95, (byte)182, (byte)182, (byte)238, (byte)35, (byte)236, (byte)173, (byte)196, (byte)17, (byte)175, (byte)176, (byte)105, (byte)195, (byte)13, (byte)84, (byte)53, (byte)203, (byte)229, (byte)173, (byte)101, (byte)252, (byte)164, (byte)23, (byte)0, (byte)146, (byte)143, (byte)140, (byte)49, (byte)229, (byte)9, (byte)237, (byte)39, (byte)145, (byte)181, (byte)67, (byte)44, (byte)187, (byte)57, (byte)62, (byte)30, (byte)223, (byte)48, (byte)42, (byte)187, (byte)87, (byte)35, (byte)16, (byte)114, (byte)25, (byte)194, (byte)241, (byte)30, (byte)93, (byte)65, (byte)21, (byte)195, (byte)238, (byte)152, (byte)252, (byte)231, (byte)45, (byte)217, (byte)234, (byte)165, (byte)252, (byte)23, (byte)221, (byte)140, (byte)158}, 0) ;
            CommunicationChannel.instance.send(p184);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnREMOTE_LOG_BLOCK_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)238);
                Debug.Assert(pack.target_component == (byte)(byte)48);
                Debug.Assert(pack.seqno == (uint)2082697516U);
                Debug.Assert(pack.status == (MAV_REMOTE_LOG_DATA_BLOCK_STATUSES)MAV_REMOTE_LOG_DATA_BLOCK_STATUSES.MAV_REMOTE_LOG_DATA_BLOCK_NACK);
            };
            GroundControl.REMOTE_LOG_BLOCK_STATUS p185 = CommunicationChannel.new_REMOTE_LOG_BLOCK_STATUS();
            PH.setPack(p185);
            p185.seqno = (uint)2082697516U;
            p185.target_component = (byte)(byte)48;
            p185.target_system = (byte)(byte)238;
            p185.status = (MAV_REMOTE_LOG_DATA_BLOCK_STATUSES)MAV_REMOTE_LOG_DATA_BLOCK_STATUSES.MAV_REMOTE_LOG_DATA_BLOCK_NACK;
            CommunicationChannel.instance.send(p185);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLED_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pattern == (byte)(byte)122);
                Debug.Assert(pack.custom_bytes.SequenceEqual(new byte[] {(byte)183, (byte)129, (byte)245, (byte)11, (byte)104, (byte)226, (byte)181, (byte)59, (byte)125, (byte)134, (byte)89, (byte)8, (byte)108, (byte)77, (byte)138, (byte)155, (byte)97, (byte)134, (byte)69, (byte)241, (byte)105, (byte)76, (byte)183, (byte)135}));
                Debug.Assert(pack.custom_len == (byte)(byte)212);
                Debug.Assert(pack.instance == (byte)(byte)76);
                Debug.Assert(pack.target_system == (byte)(byte)150);
                Debug.Assert(pack.target_component == (byte)(byte)205);
            };
            GroundControl.LED_CONTROL p186 = CommunicationChannel.new_LED_CONTROL();
            PH.setPack(p186);
            p186.instance = (byte)(byte)76;
            p186.target_component = (byte)(byte)205;
            p186.pattern = (byte)(byte)122;
            p186.custom_bytes_SET(new byte[] {(byte)183, (byte)129, (byte)245, (byte)11, (byte)104, (byte)226, (byte)181, (byte)59, (byte)125, (byte)134, (byte)89, (byte)8, (byte)108, (byte)77, (byte)138, (byte)155, (byte)97, (byte)134, (byte)69, (byte)241, (byte)105, (byte)76, (byte)183, (byte)135}, 0) ;
            p186.custom_len = (byte)(byte)212;
            p186.target_system = (byte)(byte)150;
            CommunicationChannel.instance.send(p186);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMAG_CAL_PROGRESSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.cal_status == (MAG_CAL_STATUS)MAG_CAL_STATUS.MAG_CAL_SUCCESS);
                Debug.Assert(pack.attempt == (byte)(byte)120);
                Debug.Assert(pack.direction_z == (float) -2.8849474E38F);
                Debug.Assert(pack.completion_mask.SequenceEqual(new byte[] {(byte)118, (byte)55, (byte)9, (byte)19, (byte)84, (byte)206, (byte)130, (byte)128, (byte)191, (byte)39}));
                Debug.Assert(pack.cal_mask == (byte)(byte)73);
                Debug.Assert(pack.compass_id == (byte)(byte)226);
                Debug.Assert(pack.completion_pct == (byte)(byte)169);
                Debug.Assert(pack.direction_x == (float) -2.5704778E38F);
                Debug.Assert(pack.direction_y == (float) -2.3677196E37F);
            };
            GroundControl.MAG_CAL_PROGRESS p191 = CommunicationChannel.new_MAG_CAL_PROGRESS();
            PH.setPack(p191);
            p191.attempt = (byte)(byte)120;
            p191.cal_status = (MAG_CAL_STATUS)MAG_CAL_STATUS.MAG_CAL_SUCCESS;
            p191.direction_z = (float) -2.8849474E38F;
            p191.cal_mask = (byte)(byte)73;
            p191.completion_mask_SET(new byte[] {(byte)118, (byte)55, (byte)9, (byte)19, (byte)84, (byte)206, (byte)130, (byte)128, (byte)191, (byte)39}, 0) ;
            p191.compass_id = (byte)(byte)226;
            p191.direction_x = (float) -2.5704778E38F;
            p191.completion_pct = (byte)(byte)169;
            p191.direction_y = (float) -2.3677196E37F;
            CommunicationChannel.instance.send(p191);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMAG_CAL_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.diag_z == (float) -2.7936408E37F);
                Debug.Assert(pack.diag_y == (float) -2.8483847E38F);
                Debug.Assert(pack.offdiag_z == (float)1.3269656E38F);
                Debug.Assert(pack.fitness == (float) -2.6671975E38F);
                Debug.Assert(pack.autosaved == (byte)(byte)225);
                Debug.Assert(pack.diag_x == (float)4.58102E37F);
                Debug.Assert(pack.offdiag_y == (float) -2.2239553E38F);
                Debug.Assert(pack.ofs_y == (float) -1.8190474E38F);
                Debug.Assert(pack.compass_id == (byte)(byte)28);
                Debug.Assert(pack.ofs_x == (float)2.8175565E38F);
                Debug.Assert(pack.ofs_z == (float) -1.3855076E38F);
                Debug.Assert(pack.offdiag_x == (float) -5.1265925E37F);
                Debug.Assert(pack.cal_mask == (byte)(byte)182);
                Debug.Assert(pack.cal_status == (MAG_CAL_STATUS)MAG_CAL_STATUS.MAG_CAL_SUCCESS);
            };
            GroundControl.MAG_CAL_REPORT p192 = CommunicationChannel.new_MAG_CAL_REPORT();
            PH.setPack(p192);
            p192.ofs_z = (float) -1.3855076E38F;
            p192.compass_id = (byte)(byte)28;
            p192.cal_mask = (byte)(byte)182;
            p192.ofs_y = (float) -1.8190474E38F;
            p192.offdiag_y = (float) -2.2239553E38F;
            p192.ofs_x = (float)2.8175565E38F;
            p192.autosaved = (byte)(byte)225;
            p192.diag_y = (float) -2.8483847E38F;
            p192.diag_x = (float)4.58102E37F;
            p192.cal_status = (MAG_CAL_STATUS)MAG_CAL_STATUS.MAG_CAL_SUCCESS;
            p192.offdiag_z = (float)1.3269656E38F;
            p192.fitness = (float) -2.6671975E38F;
            p192.offdiag_x = (float) -5.1265925E37F;
            p192.diag_z = (float) -2.7936408E37F;
            CommunicationChannel.instance.send(p192);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnEKF_STATUS_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pos_horiz_variance == (float) -1.6131072E37F);
                Debug.Assert(pack.terrain_alt_variance == (float) -1.6526538E37F);
                Debug.Assert(pack.compass_variance == (float) -1.8867204E38F);
                Debug.Assert(pack.flags == (EKF_STATUS_FLAGS)EKF_STATUS_FLAGS.EKF_POS_VERT_ABS);
                Debug.Assert(pack.pos_vert_variance == (float) -7.8892844E37F);
                Debug.Assert(pack.velocity_variance == (float) -1.5325725E38F);
            };
            GroundControl.EKF_STATUS_REPORT p193 = CommunicationChannel.new_EKF_STATUS_REPORT();
            PH.setPack(p193);
            p193.pos_vert_variance = (float) -7.8892844E37F;
            p193.velocity_variance = (float) -1.5325725E38F;
            p193.flags = (EKF_STATUS_FLAGS)EKF_STATUS_FLAGS.EKF_POS_VERT_ABS;
            p193.terrain_alt_variance = (float) -1.6526538E37F;
            p193.compass_variance = (float) -1.8867204E38F;
            p193.pos_horiz_variance = (float) -1.6131072E37F;
            CommunicationChannel.instance.send(p193);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPID_TUNINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.I == (float) -9.397529E37F);
                Debug.Assert(pack.achieved == (float)1.8633407E38F);
                Debug.Assert(pack.P == (float) -6.182405E37F);
                Debug.Assert(pack.FF == (float) -5.151148E37F);
                Debug.Assert(pack.axis == (PID_TUNING_AXIS)PID_TUNING_AXIS.PID_TUNING_PITCH);
                Debug.Assert(pack.desired == (float)6.578386E37F);
                Debug.Assert(pack.D == (float) -1.3986037E38F);
            };
            GroundControl.PID_TUNING p194 = CommunicationChannel.new_PID_TUNING();
            PH.setPack(p194);
            p194.desired = (float)6.578386E37F;
            p194.achieved = (float)1.8633407E38F;
            p194.P = (float) -6.182405E37F;
            p194.I = (float) -9.397529E37F;
            p194.FF = (float) -5.151148E37F;
            p194.axis = (PID_TUNING_AXIS)PID_TUNING_AXIS.PID_TUNING_PITCH;
            p194.D = (float) -1.3986037E38F;
            CommunicationChannel.instance.send(p194);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnGIMBAL_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.delta_time == (float) -2.2899139E38F);
                Debug.Assert(pack.delta_angle_z == (float) -2.879884E38F);
                Debug.Assert(pack.delta_velocity_y == (float) -1.6727762E38F);
                Debug.Assert(pack.joint_el == (float)2.613833E38F);
                Debug.Assert(pack.delta_angle_x == (float) -6.6820383E37F);
                Debug.Assert(pack.joint_az == (float) -5.830514E37F);
                Debug.Assert(pack.delta_velocity_x == (float)7.224838E36F);
                Debug.Assert(pack.joint_roll == (float)2.1537485E38F);
                Debug.Assert(pack.target_system == (byte)(byte)165);
                Debug.Assert(pack.delta_angle_y == (float) -2.539336E38F);
                Debug.Assert(pack.delta_velocity_z == (float) -2.2597785E38F);
                Debug.Assert(pack.target_component == (byte)(byte)229);
            };
            GroundControl.GIMBAL_REPORT p200 = CommunicationChannel.new_GIMBAL_REPORT();
            PH.setPack(p200);
            p200.target_component = (byte)(byte)229;
            p200.joint_az = (float) -5.830514E37F;
            p200.joint_roll = (float)2.1537485E38F;
            p200.delta_velocity_z = (float) -2.2597785E38F;
            p200.delta_velocity_x = (float)7.224838E36F;
            p200.delta_time = (float) -2.2899139E38F;
            p200.delta_velocity_y = (float) -1.6727762E38F;
            p200.delta_angle_x = (float) -6.6820383E37F;
            p200.joint_el = (float)2.613833E38F;
            p200.delta_angle_y = (float) -2.539336E38F;
            p200.delta_angle_z = (float) -2.879884E38F;
            p200.target_system = (byte)(byte)165;
            CommunicationChannel.instance.send(p200);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnGIMBAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)85);
                Debug.Assert(pack.demanded_rate_x == (float)1.4448945E38F);
                Debug.Assert(pack.demanded_rate_z == (float)2.990093E38F);
                Debug.Assert(pack.target_system == (byte)(byte)20);
                Debug.Assert(pack.demanded_rate_y == (float)2.2677922E38F);
            };
            GroundControl.GIMBAL_CONTROL p201 = CommunicationChannel.new_GIMBAL_CONTROL();
            PH.setPack(p201);
            p201.demanded_rate_x = (float)1.4448945E38F;
            p201.target_system = (byte)(byte)20;
            p201.target_component = (byte)(byte)85;
            p201.demanded_rate_y = (float)2.2677922E38F;
            p201.demanded_rate_z = (float)2.990093E38F;
            CommunicationChannel.instance.send(p201);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnGIMBAL_TORQUE_CMD_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.el_torque_cmd == (short)(short) -228);
                Debug.Assert(pack.target_component == (byte)(byte)102);
                Debug.Assert(pack.rl_torque_cmd == (short)(short)22586);
                Debug.Assert(pack.target_system == (byte)(byte)81);
                Debug.Assert(pack.az_torque_cmd == (short)(short) -22799);
            };
            GroundControl.GIMBAL_TORQUE_CMD_REPORT p214 = CommunicationChannel.new_GIMBAL_TORQUE_CMD_REPORT();
            PH.setPack(p214);
            p214.el_torque_cmd = (short)(short) -228;
            p214.target_system = (byte)(byte)81;
            p214.rl_torque_cmd = (short)(short)22586;
            p214.target_component = (byte)(byte)102;
            p214.az_torque_cmd = (short)(short) -22799;
            CommunicationChannel.instance.send(p214);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnGOPRO_HEARTBEATReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.capture_mode == (GOPRO_CAPTURE_MODE)GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_PHOTO);
                Debug.Assert(pack.flags == (GOPRO_HEARTBEAT_FLAGS)GOPRO_HEARTBEAT_FLAGS.GOPRO_FLAG_RECORDING);
                Debug.Assert(pack.status == (GOPRO_HEARTBEAT_STATUS)GOPRO_HEARTBEAT_STATUS.GOPRO_HEARTBEAT_STATUS_INCOMPATIBLE);
            };
            GroundControl.GOPRO_HEARTBEAT p215 = CommunicationChannel.new_GOPRO_HEARTBEAT();
            PH.setPack(p215);
            p215.flags = (GOPRO_HEARTBEAT_FLAGS)GOPRO_HEARTBEAT_FLAGS.GOPRO_FLAG_RECORDING;
            p215.capture_mode = (GOPRO_CAPTURE_MODE)GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_PHOTO;
            p215.status = (GOPRO_HEARTBEAT_STATUS)GOPRO_HEARTBEAT_STATUS.GOPRO_HEARTBEAT_STATUS_INCOMPATIBLE;
            CommunicationChannel.instance.send(p215);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnGOPRO_GET_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)3);
                Debug.Assert(pack.target_system == (byte)(byte)49);
                Debug.Assert(pack.cmd_id == (GOPRO_COMMAND)GOPRO_COMMAND.GOPRO_COMMAND_PHOTO_RESOLUTION);
            };
            GroundControl.GOPRO_GET_REQUEST p216 = CommunicationChannel.new_GOPRO_GET_REQUEST();
            PH.setPack(p216);
            p216.cmd_id = (GOPRO_COMMAND)GOPRO_COMMAND.GOPRO_COMMAND_PHOTO_RESOLUTION;
            p216.target_component = (byte)(byte)3;
            p216.target_system = (byte)(byte)49;
            CommunicationChannel.instance.send(p216);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnGOPRO_GET_RESPONSEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value.SequenceEqual(new byte[] {(byte)24, (byte)66, (byte)151, (byte)173}));
                Debug.Assert(pack.cmd_id == (GOPRO_COMMAND)GOPRO_COMMAND.GOPRO_COMMAND_SHUTTER);
                Debug.Assert(pack.status == (GOPRO_REQUEST_STATUS)GOPRO_REQUEST_STATUS.GOPRO_REQUEST_SUCCESS);
            };
            GroundControl.GOPRO_GET_RESPONSE p217 = CommunicationChannel.new_GOPRO_GET_RESPONSE();
            PH.setPack(p217);
            p217.cmd_id = (GOPRO_COMMAND)GOPRO_COMMAND.GOPRO_COMMAND_SHUTTER;
            p217.status = (GOPRO_REQUEST_STATUS)GOPRO_REQUEST_STATUS.GOPRO_REQUEST_SUCCESS;
            p217.value_SET(new byte[] {(byte)24, (byte)66, (byte)151, (byte)173}, 0) ;
            CommunicationChannel.instance.send(p217);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnGOPRO_SET_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value.SequenceEqual(new byte[] {(byte)142, (byte)143, (byte)230, (byte)192}));
                Debug.Assert(pack.target_system == (byte)(byte)143);
                Debug.Assert(pack.cmd_id == (GOPRO_COMMAND)GOPRO_COMMAND.GOPRO_COMMAND_CAPTURE_MODE);
                Debug.Assert(pack.target_component == (byte)(byte)19);
            };
            GroundControl.GOPRO_SET_REQUEST p218 = CommunicationChannel.new_GOPRO_SET_REQUEST();
            PH.setPack(p218);
            p218.value_SET(new byte[] {(byte)142, (byte)143, (byte)230, (byte)192}, 0) ;
            p218.target_system = (byte)(byte)143;
            p218.target_component = (byte)(byte)19;
            p218.cmd_id = (GOPRO_COMMAND)GOPRO_COMMAND.GOPRO_COMMAND_CAPTURE_MODE;
            CommunicationChannel.instance.send(p218);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnGOPRO_SET_RESPONSEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.status == (GOPRO_REQUEST_STATUS)GOPRO_REQUEST_STATUS.GOPRO_REQUEST_FAILED);
                Debug.Assert(pack.cmd_id == (GOPRO_COMMAND)GOPRO_COMMAND.GOPRO_COMMAND_PROTUNE);
            };
            GroundControl.GOPRO_SET_RESPONSE p219 = CommunicationChannel.new_GOPRO_SET_RESPONSE();
            PH.setPack(p219);
            p219.status = (GOPRO_REQUEST_STATUS)GOPRO_REQUEST_STATUS.GOPRO_REQUEST_FAILED;
            p219.cmd_id = (GOPRO_COMMAND)GOPRO_COMMAND.GOPRO_COMMAND_PROTUNE;
            CommunicationChannel.instance.send(p219);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnRPMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rpm2 == (float) -3.2540863E38F);
                Debug.Assert(pack.rpm1 == (float) -1.143492E38F);
            };
            GroundControl.RPM p226 = CommunicationChannel.new_RPM();
            PH.setPack(p226);
            p226.rpm1 = (float) -1.143492E38F;
            p226.rpm2 = (float) -3.2540863E38F;
            CommunicationChannel.instance.send(p226);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            CommunicationChannel.instance.OnESTIMATOR_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pos_vert_ratio == (float) -2.5769895E38F);
                Debug.Assert(pack.hagl_ratio == (float) -2.6237003E38F);
                Debug.Assert(pack.time_usec == (ulong)3970901299172367892L);
                Debug.Assert(pack.mag_ratio == (float)1.502055E38F);
                Debug.Assert(pack.pos_horiz_accuracy == (float)9.306252E37F);
                Debug.Assert(pack.flags == (ESTIMATOR_STATUS_FLAGS)ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ);
                Debug.Assert(pack.pos_vert_accuracy == (float)4.193603E37F);
                Debug.Assert(pack.pos_horiz_ratio == (float) -2.3211145E38F);
                Debug.Assert(pack.tas_ratio == (float)1.9793928E38F);
                Debug.Assert(pack.vel_ratio == (float) -2.908848E37F);
            };
            GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.mag_ratio = (float)1.502055E38F;
            p230.tas_ratio = (float)1.9793928E38F;
            p230.hagl_ratio = (float) -2.6237003E38F;
            p230.pos_horiz_ratio = (float) -2.3211145E38F;
            p230.pos_horiz_accuracy = (float)9.306252E37F;
            p230.flags = (ESTIMATOR_STATUS_FLAGS)ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ;
            p230.pos_vert_accuracy = (float)4.193603E37F;
            p230.time_usec = (ulong)3970901299172367892L;
            p230.pos_vert_ratio = (float) -2.5769895E38F;
            p230.vel_ratio = (float) -2.908848E37F;
            CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnWIND_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.wind_z == (float) -2.4368682E38F);
                Debug.Assert(pack.var_vert == (float) -3.3348403E38F);
                Debug.Assert(pack.vert_accuracy == (float)4.141237E37F);
                Debug.Assert(pack.horiz_accuracy == (float) -5.1577914E37F);
                Debug.Assert(pack.var_horiz == (float)2.1217986E38F);
                Debug.Assert(pack.wind_alt == (float) -1.3331679E38F);
                Debug.Assert(pack.time_usec == (ulong)4853526293325203351L);
                Debug.Assert(pack.wind_y == (float) -2.370915E38F);
                Debug.Assert(pack.wind_x == (float) -1.7429498E38F);
            };
            GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.wind_z = (float) -2.4368682E38F;
            p231.wind_y = (float) -2.370915E38F;
            p231.wind_x = (float) -1.7429498E38F;
            p231.horiz_accuracy = (float) -5.1577914E37F;
            p231.var_horiz = (float)2.1217986E38F;
            p231.var_vert = (float) -3.3348403E38F;
            p231.wind_alt = (float) -1.3331679E38F;
            p231.time_usec = (ulong)4853526293325203351L;
            p231.vert_accuracy = (float)4.141237E37F;
            CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_INPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_week_ms == (uint)4232970366U);
                Debug.Assert(pack.hdop == (float) -2.5470317E38F);
                Debug.Assert(pack.ignore_flags == (GPS_INPUT_IGNORE_FLAGS)GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT);
                Debug.Assert(pack.horiz_accuracy == (float) -1.3061948E38F);
                Debug.Assert(pack.lat == (int)103637289);
                Debug.Assert(pack.lon == (int) -556328924);
                Debug.Assert(pack.vn == (float)6.1026215E36F);
                Debug.Assert(pack.vdop == (float)3.366497E36F);
                Debug.Assert(pack.vd == (float)1.3052519E38F);
                Debug.Assert(pack.speed_accuracy == (float) -2.0086054E38F);
                Debug.Assert(pack.time_usec == (ulong)3117535733446084829L);
                Debug.Assert(pack.fix_type == (byte)(byte)209);
                Debug.Assert(pack.vert_accuracy == (float)1.2067544E38F);
                Debug.Assert(pack.ve == (float)5.3186456E37F);
                Debug.Assert(pack.alt == (float) -3.3097515E38F);
                Debug.Assert(pack.time_week == (ushort)(ushort)32697);
                Debug.Assert(pack.gps_id == (byte)(byte)64);
                Debug.Assert(pack.satellites_visible == (byte)(byte)246);
            };
            GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.speed_accuracy = (float) -2.0086054E38F;
            p232.time_week_ms = (uint)4232970366U;
            p232.lat = (int)103637289;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS)GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT;
            p232.satellites_visible = (byte)(byte)246;
            p232.vd = (float)1.3052519E38F;
            p232.time_usec = (ulong)3117535733446084829L;
            p232.ve = (float)5.3186456E37F;
            p232.gps_id = (byte)(byte)64;
            p232.time_week = (ushort)(ushort)32697;
            p232.alt = (float) -3.3097515E38F;
            p232.vert_accuracy = (float)1.2067544E38F;
            p232.hdop = (float) -2.5470317E38F;
            p232.lon = (int) -556328924;
            p232.vn = (float)6.1026215E36F;
            p232.horiz_accuracy = (float) -1.3061948E38F;
            p232.vdop = (float)3.366497E36F;
            p232.fix_type = (byte)(byte)209;
            CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RTCM_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)77, (byte)161, (byte)166, (byte)22, (byte)123, (byte)83, (byte)21, (byte)27, (byte)180, (byte)227, (byte)47, (byte)190, (byte)160, (byte)48, (byte)221, (byte)201, (byte)40, (byte)80, (byte)254, (byte)1, (byte)49, (byte)131, (byte)168, (byte)249, (byte)12, (byte)238, (byte)46, (byte)152, (byte)115, (byte)226, (byte)78, (byte)78, (byte)36, (byte)158, (byte)169, (byte)140, (byte)126, (byte)61, (byte)179, (byte)203, (byte)59, (byte)202, (byte)163, (byte)38, (byte)225, (byte)111, (byte)225, (byte)219, (byte)155, (byte)191, (byte)224, (byte)131, (byte)173, (byte)196, (byte)198, (byte)161, (byte)82, (byte)83, (byte)13, (byte)35, (byte)210, (byte)57, (byte)214, (byte)131, (byte)104, (byte)0, (byte)151, (byte)137, (byte)59, (byte)197, (byte)12, (byte)176, (byte)226, (byte)207, (byte)62, (byte)231, (byte)173, (byte)136, (byte)143, (byte)54, (byte)212, (byte)48, (byte)134, (byte)47, (byte)131, (byte)207, (byte)64, (byte)92, (byte)11, (byte)73, (byte)164, (byte)93, (byte)8, (byte)208, (byte)9, (byte)37, (byte)207, (byte)125, (byte)170, (byte)18, (byte)82, (byte)65, (byte)251, (byte)9, (byte)199, (byte)92, (byte)131, (byte)10, (byte)142, (byte)127, (byte)21, (byte)239, (byte)180, (byte)220, (byte)37, (byte)236, (byte)59, (byte)74, (byte)234, (byte)251, (byte)214, (byte)82, (byte)76, (byte)161, (byte)22, (byte)98, (byte)149, (byte)249, (byte)14, (byte)190, (byte)253, (byte)11, (byte)75, (byte)35, (byte)56, (byte)171, (byte)170, (byte)231, (byte)181, (byte)166, (byte)53, (byte)255, (byte)68, (byte)224, (byte)38, (byte)35, (byte)235, (byte)56, (byte)1, (byte)215, (byte)251, (byte)244, (byte)96, (byte)236, (byte)140, (byte)181, (byte)50, (byte)249, (byte)71, (byte)28, (byte)204, (byte)240, (byte)230, (byte)159, (byte)100, (byte)23, (byte)43, (byte)175, (byte)171, (byte)240, (byte)2, (byte)190, (byte)38, (byte)17, (byte)161, (byte)245, (byte)236, (byte)253, (byte)129, (byte)154}));
                Debug.Assert(pack.flags == (byte)(byte)193);
                Debug.Assert(pack.len == (byte)(byte)99);
            };
            GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.flags = (byte)(byte)193;
            p233.data__SET(new byte[] {(byte)77, (byte)161, (byte)166, (byte)22, (byte)123, (byte)83, (byte)21, (byte)27, (byte)180, (byte)227, (byte)47, (byte)190, (byte)160, (byte)48, (byte)221, (byte)201, (byte)40, (byte)80, (byte)254, (byte)1, (byte)49, (byte)131, (byte)168, (byte)249, (byte)12, (byte)238, (byte)46, (byte)152, (byte)115, (byte)226, (byte)78, (byte)78, (byte)36, (byte)158, (byte)169, (byte)140, (byte)126, (byte)61, (byte)179, (byte)203, (byte)59, (byte)202, (byte)163, (byte)38, (byte)225, (byte)111, (byte)225, (byte)219, (byte)155, (byte)191, (byte)224, (byte)131, (byte)173, (byte)196, (byte)198, (byte)161, (byte)82, (byte)83, (byte)13, (byte)35, (byte)210, (byte)57, (byte)214, (byte)131, (byte)104, (byte)0, (byte)151, (byte)137, (byte)59, (byte)197, (byte)12, (byte)176, (byte)226, (byte)207, (byte)62, (byte)231, (byte)173, (byte)136, (byte)143, (byte)54, (byte)212, (byte)48, (byte)134, (byte)47, (byte)131, (byte)207, (byte)64, (byte)92, (byte)11, (byte)73, (byte)164, (byte)93, (byte)8, (byte)208, (byte)9, (byte)37, (byte)207, (byte)125, (byte)170, (byte)18, (byte)82, (byte)65, (byte)251, (byte)9, (byte)199, (byte)92, (byte)131, (byte)10, (byte)142, (byte)127, (byte)21, (byte)239, (byte)180, (byte)220, (byte)37, (byte)236, (byte)59, (byte)74, (byte)234, (byte)251, (byte)214, (byte)82, (byte)76, (byte)161, (byte)22, (byte)98, (byte)149, (byte)249, (byte)14, (byte)190, (byte)253, (byte)11, (byte)75, (byte)35, (byte)56, (byte)171, (byte)170, (byte)231, (byte)181, (byte)166, (byte)53, (byte)255, (byte)68, (byte)224, (byte)38, (byte)35, (byte)235, (byte)56, (byte)1, (byte)215, (byte)251, (byte)244, (byte)96, (byte)236, (byte)140, (byte)181, (byte)50, (byte)249, (byte)71, (byte)28, (byte)204, (byte)240, (byte)230, (byte)159, (byte)100, (byte)23, (byte)43, (byte)175, (byte)171, (byte)240, (byte)2, (byte)190, (byte)38, (byte)17, (byte)161, (byte)245, (byte)236, (byte)253, (byte)129, (byte)154}, 0) ;
            p233.len = (byte)(byte)99;
            CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIGH_LATENCYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.longitude == (int) -875233517);
                Debug.Assert(pack.gps_nsat == (byte)(byte)78);
                Debug.Assert(pack.temperature_air == (sbyte)(sbyte)83);
                Debug.Assert(pack.battery_remaining == (byte)(byte)86);
                Debug.Assert(pack.groundspeed == (byte)(byte)164);
                Debug.Assert(pack.latitude == (int)1266410076);
                Debug.Assert(pack.wp_distance == (ushort)(ushort)45371);
                Debug.Assert(pack.heading == (ushort)(ushort)22299);
                Debug.Assert(pack.heading_sp == (short)(short)3614);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED);
                Debug.Assert(pack.airspeed_sp == (byte)(byte)12);
                Debug.Assert(pack.altitude_sp == (short)(short) -20097);
                Debug.Assert(pack.temperature == (sbyte)(sbyte) - 35);
                Debug.Assert(pack.failsafe == (byte)(byte)69);
                Debug.Assert(pack.gps_fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX);
                Debug.Assert(pack.landed_state == (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING);
                Debug.Assert(pack.altitude_amsl == (short)(short)12530);
                Debug.Assert(pack.pitch == (short)(short) -25965);
                Debug.Assert(pack.roll == (short)(short)23616);
                Debug.Assert(pack.climb_rate == (sbyte)(sbyte) - 59);
                Debug.Assert(pack.wp_num == (byte)(byte)81);
                Debug.Assert(pack.custom_mode == (uint)1349732942U);
                Debug.Assert(pack.airspeed == (byte)(byte)88);
                Debug.Assert(pack.throttle == (sbyte)(sbyte)124);
            };
            GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.temperature = (sbyte)(sbyte) - 35;
            p234.altitude_amsl = (short)(short)12530;
            p234.groundspeed = (byte)(byte)164;
            p234.pitch = (short)(short) -25965;
            p234.custom_mode = (uint)1349732942U;
            p234.airspeed_sp = (byte)(byte)12;
            p234.battery_remaining = (byte)(byte)86;
            p234.longitude = (int) -875233517;
            p234.temperature_air = (sbyte)(sbyte)83;
            p234.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED;
            p234.airspeed = (byte)(byte)88;
            p234.gps_nsat = (byte)(byte)78;
            p234.gps_fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX;
            p234.roll = (short)(short)23616;
            p234.heading_sp = (short)(short)3614;
            p234.throttle = (sbyte)(sbyte)124;
            p234.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING;
            p234.wp_distance = (ushort)(ushort)45371;
            p234.climb_rate = (sbyte)(sbyte) - 59;
            p234.latitude = (int)1266410076;
            p234.altitude_sp = (short)(short) -20097;
            p234.wp_num = (byte)(byte)81;
            p234.failsafe = (byte)(byte)69;
            p234.heading = (ushort)(ushort)22299;
            CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVIBRATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.clipping_2 == (uint)3911939246U);
                Debug.Assert(pack.vibration_x == (float) -2.626128E38F);
                Debug.Assert(pack.vibration_z == (float)3.6055385E37F);
                Debug.Assert(pack.clipping_1 == (uint)2149937497U);
                Debug.Assert(pack.clipping_0 == (uint)3098666748U);
                Debug.Assert(pack.time_usec == (ulong)8908892442642917069L);
                Debug.Assert(pack.vibration_y == (float)2.8534336E38F);
            };
            GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.time_usec = (ulong)8908892442642917069L;
            p241.clipping_0 = (uint)3098666748U;
            p241.vibration_x = (float) -2.626128E38F;
            p241.clipping_1 = (uint)2149937497U;
            p241.vibration_z = (float)3.6055385E37F;
            p241.vibration_y = (float)2.8534336E38F;
            p241.clipping_2 = (uint)3911939246U;
            CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float) -4.007221E37F);
                Debug.Assert(pack.approach_x == (float) -2.0475125E38F);
                Debug.Assert(pack.x == (float) -2.000607E37F);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)862319585894005851L);
                Debug.Assert(pack.approach_y == (float)1.0652868E38F);
                Debug.Assert(pack.altitude == (int) -1230642560);
                Debug.Assert(pack.longitude == (int)1943466172);
                Debug.Assert(pack.q.SequenceEqual(new float[] {8.251018E37F, 1.6607871E38F, -1.8497622E38F, 1.2688221E38F}));
                Debug.Assert(pack.approach_z == (float)8.852344E37F);
                Debug.Assert(pack.z == (float) -1.3428417E37F);
                Debug.Assert(pack.latitude == (int) -1302124697);
            };
            GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.approach_z = (float)8.852344E37F;
            p242.approach_x = (float) -2.0475125E38F;
            p242.x = (float) -2.000607E37F;
            p242.q_SET(new float[] {8.251018E37F, 1.6607871E38F, -1.8497622E38F, 1.2688221E38F}, 0) ;
            p242.altitude = (int) -1230642560;
            p242.approach_y = (float)1.0652868E38F;
            p242.longitude = (int)1943466172;
            p242.time_usec_SET((ulong)862319585894005851L, PH) ;
            p242.latitude = (int) -1302124697;
            p242.y = (float) -4.007221E37F;
            p242.z = (float) -1.3428417E37F;
            CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_HOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude == (int)496868899);
                Debug.Assert(pack.z == (float)1.5650612E38F);
                Debug.Assert(pack.y == (float) -3.274382E38F);
                Debug.Assert(pack.latitude == (int) -1168324901);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)6689782209356215198L);
                Debug.Assert(pack.target_system == (byte)(byte)118);
                Debug.Assert(pack.approach_x == (float)9.889524E37F);
                Debug.Assert(pack.approach_y == (float)2.3274702E38F);
                Debug.Assert(pack.approach_z == (float)8.93701E37F);
                Debug.Assert(pack.x == (float) -2.44653E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.6197644E38F, -1.080926E38F, 1.3547115E38F, -9.874112E37F}));
                Debug.Assert(pack.longitude == (int) -1685238286);
            };
            GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.z = (float)1.5650612E38F;
            p243.longitude = (int) -1685238286;
            p243.altitude = (int)496868899;
            p243.latitude = (int) -1168324901;
            p243.time_usec_SET((ulong)6689782209356215198L, PH) ;
            p243.approach_x = (float)9.889524E37F;
            p243.y = (float) -3.274382E38F;
            p243.approach_y = (float)2.3274702E38F;
            p243.approach_z = (float)8.93701E37F;
            p243.x = (float) -2.44653E38F;
            p243.q_SET(new float[] {-1.6197644E38F, -1.080926E38F, 1.3547115E38F, -9.874112E37F}, 0) ;
            p243.target_system = (byte)(byte)118;
            CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMESSAGE_INTERVALReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.interval_us == (int) -1956123265);
                Debug.Assert(pack.message_id == (ushort)(ushort)28845);
            };
            GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.message_id = (ushort)(ushort)28845;
            p244.interval_us = (int) -1956123265;
            CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnEXTENDED_SYS_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vtol_state == (MAV_VTOL_STATE)MAV_VTOL_STATE.MAV_VTOL_STATE_MC);
                Debug.Assert(pack.landed_state == (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND);
            };
            GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND;
            p245.vtol_state = (MAV_VTOL_STATE)MAV_VTOL_STATE.MAV_VTOL_STATE_MC;
            CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnADSB_VEHICLEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tslc == (byte)(byte)247);
                Debug.Assert(pack.altitude_type == (ADSB_ALTITUDE_TYPE)ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
                Debug.Assert(pack.ICAO_address == (uint)3330707463U);
                Debug.Assert(pack.lat == (int)613441170);
                Debug.Assert(pack.ver_velocity == (short)(short) -4855);
                Debug.Assert(pack.callsign_LEN(ph) == 8);
                Debug.Assert(pack.callsign_TRY(ph).Equals("ivmsenfZ"));
                Debug.Assert(pack.heading == (ushort)(ushort)26985);
                Debug.Assert(pack.flags == (ADSB_FLAGS)ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING);
                Debug.Assert(pack.emitter_type == (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_LARGE);
                Debug.Assert(pack.altitude == (int)242450233);
                Debug.Assert(pack.lon == (int) -777426022);
                Debug.Assert(pack.hor_velocity == (ushort)(ushort)22040);
                Debug.Assert(pack.squawk == (ushort)(ushort)43200);
            };
            GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.heading = (ushort)(ushort)26985;
            p246.flags = (ADSB_FLAGS)ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING;
            p246.lat = (int)613441170;
            p246.lon = (int) -777426022;
            p246.tslc = (byte)(byte)247;
            p246.emitter_type = (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_LARGE;
            p246.squawk = (ushort)(ushort)43200;
            p246.altitude_type = (ADSB_ALTITUDE_TYPE)ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
            p246.altitude = (int)242450233;
            p246.ICAO_address = (uint)3330707463U;
            p246.ver_velocity = (short)(short) -4855;
            p246.hor_velocity = (ushort)(ushort)22040;
            p246.callsign_SET("ivmsenfZ", PH) ;
            CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOLLISIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.threat_level == (MAV_COLLISION_THREAT_LEVEL)MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW);
                Debug.Assert(pack.src_ == (MAV_COLLISION_SRC)MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
                Debug.Assert(pack.altitude_minimum_delta == (float) -2.7650031E38F);
                Debug.Assert(pack.action == (MAV_COLLISION_ACTION)MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_NONE);
                Debug.Assert(pack.horizontal_minimum_delta == (float) -2.3254726E38F);
                Debug.Assert(pack.time_to_minimum_delta == (float) -3.1951949E38F);
                Debug.Assert(pack.id == (uint)1580923470U);
            };
            GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
            PH.setPack(p247);
            p247.src_ = (MAV_COLLISION_SRC)MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT;
            p247.threat_level = (MAV_COLLISION_THREAT_LEVEL)MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW;
            p247.time_to_minimum_delta = (float) -3.1951949E38F;
            p247.horizontal_minimum_delta = (float) -2.3254726E38F;
            p247.action = (MAV_COLLISION_ACTION)MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_NONE;
            p247.altitude_minimum_delta = (float) -2.7650031E38F;
            p247.id = (uint)1580923470U;
            CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnV2_EXTENSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.message_type == (ushort)(ushort)35887);
                Debug.Assert(pack.target_component == (byte)(byte)157);
                Debug.Assert(pack.target_system == (byte)(byte)253);
                Debug.Assert(pack.target_network == (byte)(byte)180);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)216, (byte)98, (byte)117, (byte)158, (byte)18, (byte)29, (byte)95, (byte)181, (byte)245, (byte)129, (byte)65, (byte)204, (byte)235, (byte)178, (byte)170, (byte)111, (byte)99, (byte)221, (byte)58, (byte)163, (byte)219, (byte)179, (byte)118, (byte)79, (byte)217, (byte)65, (byte)202, (byte)84, (byte)112, (byte)125, (byte)60, (byte)169, (byte)89, (byte)89, (byte)22, (byte)246, (byte)251, (byte)116, (byte)198, (byte)169, (byte)232, (byte)235, (byte)250, (byte)142, (byte)190, (byte)62, (byte)119, (byte)217, (byte)70, (byte)10, (byte)119, (byte)78, (byte)209, (byte)11, (byte)174, (byte)76, (byte)231, (byte)236, (byte)203, (byte)98, (byte)143, (byte)185, (byte)68, (byte)226, (byte)60, (byte)105, (byte)134, (byte)162, (byte)134, (byte)67, (byte)4, (byte)96, (byte)108, (byte)149, (byte)41, (byte)31, (byte)101, (byte)165, (byte)251, (byte)176, (byte)134, (byte)187, (byte)59, (byte)186, (byte)100, (byte)209, (byte)41, (byte)87, (byte)228, (byte)192, (byte)245, (byte)154, (byte)173, (byte)76, (byte)36, (byte)236, (byte)154, (byte)149, (byte)251, (byte)214, (byte)136, (byte)240, (byte)53, (byte)58, (byte)59, (byte)26, (byte)239, (byte)61, (byte)184, (byte)246, (byte)153, (byte)158, (byte)88, (byte)160, (byte)199, (byte)208, (byte)246, (byte)99, (byte)106, (byte)114, (byte)211, (byte)60, (byte)16, (byte)38, (byte)98, (byte)163, (byte)247, (byte)221, (byte)28, (byte)107, (byte)244, (byte)199, (byte)153, (byte)46, (byte)112, (byte)171, (byte)142, (byte)94, (byte)210, (byte)159, (byte)4, (byte)29, (byte)3, (byte)115, (byte)86, (byte)38, (byte)228, (byte)246, (byte)154, (byte)207, (byte)93, (byte)220, (byte)54, (byte)109, (byte)23, (byte)120, (byte)90, (byte)173, (byte)88, (byte)4, (byte)56, (byte)31, (byte)207, (byte)158, (byte)193, (byte)69, (byte)1, (byte)78, (byte)1, (byte)173, (byte)81, (byte)12, (byte)94, (byte)124, (byte)71, (byte)194, (byte)145, (byte)87, (byte)242, (byte)100, (byte)221, (byte)66, (byte)37, (byte)216, (byte)162, (byte)68, (byte)85, (byte)3, (byte)159, (byte)81, (byte)113, (byte)36, (byte)80, (byte)121, (byte)218, (byte)248, (byte)0, (byte)149, (byte)121, (byte)179, (byte)201, (byte)212, (byte)167, (byte)96, (byte)233, (byte)84, (byte)223, (byte)15, (byte)94, (byte)10, (byte)9, (byte)58, (byte)238, (byte)143, (byte)61, (byte)211, (byte)227, (byte)183, (byte)209, (byte)243, (byte)169, (byte)147, (byte)225, (byte)91, (byte)45, (byte)118, (byte)124, (byte)151, (byte)17, (byte)74, (byte)255, (byte)51, (byte)141, (byte)68, (byte)91, (byte)145, (byte)15, (byte)252, (byte)132, (byte)172, (byte)1, (byte)83, (byte)30, (byte)60, (byte)23, (byte)31, (byte)145, (byte)25, (byte)54}));
            };
            GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.message_type = (ushort)(ushort)35887;
            p248.payload_SET(new byte[] {(byte)216, (byte)98, (byte)117, (byte)158, (byte)18, (byte)29, (byte)95, (byte)181, (byte)245, (byte)129, (byte)65, (byte)204, (byte)235, (byte)178, (byte)170, (byte)111, (byte)99, (byte)221, (byte)58, (byte)163, (byte)219, (byte)179, (byte)118, (byte)79, (byte)217, (byte)65, (byte)202, (byte)84, (byte)112, (byte)125, (byte)60, (byte)169, (byte)89, (byte)89, (byte)22, (byte)246, (byte)251, (byte)116, (byte)198, (byte)169, (byte)232, (byte)235, (byte)250, (byte)142, (byte)190, (byte)62, (byte)119, (byte)217, (byte)70, (byte)10, (byte)119, (byte)78, (byte)209, (byte)11, (byte)174, (byte)76, (byte)231, (byte)236, (byte)203, (byte)98, (byte)143, (byte)185, (byte)68, (byte)226, (byte)60, (byte)105, (byte)134, (byte)162, (byte)134, (byte)67, (byte)4, (byte)96, (byte)108, (byte)149, (byte)41, (byte)31, (byte)101, (byte)165, (byte)251, (byte)176, (byte)134, (byte)187, (byte)59, (byte)186, (byte)100, (byte)209, (byte)41, (byte)87, (byte)228, (byte)192, (byte)245, (byte)154, (byte)173, (byte)76, (byte)36, (byte)236, (byte)154, (byte)149, (byte)251, (byte)214, (byte)136, (byte)240, (byte)53, (byte)58, (byte)59, (byte)26, (byte)239, (byte)61, (byte)184, (byte)246, (byte)153, (byte)158, (byte)88, (byte)160, (byte)199, (byte)208, (byte)246, (byte)99, (byte)106, (byte)114, (byte)211, (byte)60, (byte)16, (byte)38, (byte)98, (byte)163, (byte)247, (byte)221, (byte)28, (byte)107, (byte)244, (byte)199, (byte)153, (byte)46, (byte)112, (byte)171, (byte)142, (byte)94, (byte)210, (byte)159, (byte)4, (byte)29, (byte)3, (byte)115, (byte)86, (byte)38, (byte)228, (byte)246, (byte)154, (byte)207, (byte)93, (byte)220, (byte)54, (byte)109, (byte)23, (byte)120, (byte)90, (byte)173, (byte)88, (byte)4, (byte)56, (byte)31, (byte)207, (byte)158, (byte)193, (byte)69, (byte)1, (byte)78, (byte)1, (byte)173, (byte)81, (byte)12, (byte)94, (byte)124, (byte)71, (byte)194, (byte)145, (byte)87, (byte)242, (byte)100, (byte)221, (byte)66, (byte)37, (byte)216, (byte)162, (byte)68, (byte)85, (byte)3, (byte)159, (byte)81, (byte)113, (byte)36, (byte)80, (byte)121, (byte)218, (byte)248, (byte)0, (byte)149, (byte)121, (byte)179, (byte)201, (byte)212, (byte)167, (byte)96, (byte)233, (byte)84, (byte)223, (byte)15, (byte)94, (byte)10, (byte)9, (byte)58, (byte)238, (byte)143, (byte)61, (byte)211, (byte)227, (byte)183, (byte)209, (byte)243, (byte)169, (byte)147, (byte)225, (byte)91, (byte)45, (byte)118, (byte)124, (byte)151, (byte)17, (byte)74, (byte)255, (byte)51, (byte)141, (byte)68, (byte)91, (byte)145, (byte)15, (byte)252, (byte)132, (byte)172, (byte)1, (byte)83, (byte)30, (byte)60, (byte)23, (byte)31, (byte)145, (byte)25, (byte)54}, 0) ;
            p248.target_network = (byte)(byte)180;
            p248.target_system = (byte)(byte)253;
            p248.target_component = (byte)(byte)157;
            CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMEMORY_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.address == (ushort)(ushort)50388);
                Debug.Assert(pack.ver == (byte)(byte)144);
                Debug.Assert(pack.type == (byte)(byte)36);
                Debug.Assert(pack.value.SequenceEqual(new sbyte[] {(sbyte) - 110, (sbyte)75, (sbyte)113, (sbyte)17, (sbyte) - 1, (sbyte) - 26, (sbyte)22, (sbyte) - 94, (sbyte)112, (sbyte)6, (sbyte)66, (sbyte) - 61, (sbyte)108, (sbyte)18, (sbyte)31, (sbyte) - 75, (sbyte)36, (sbyte)7, (sbyte)28, (sbyte) - 1, (sbyte)111, (sbyte) - 55, (sbyte) - 50, (sbyte) - 1, (sbyte) - 40, (sbyte)9, (sbyte) - 71, (sbyte)1, (sbyte) - 95, (sbyte)4, (sbyte) - 116, (sbyte)93}));
            };
            GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.value_SET(new sbyte[] {(sbyte) - 110, (sbyte)75, (sbyte)113, (sbyte)17, (sbyte) - 1, (sbyte) - 26, (sbyte)22, (sbyte) - 94, (sbyte)112, (sbyte)6, (sbyte)66, (sbyte) - 61, (sbyte)108, (sbyte)18, (sbyte)31, (sbyte) - 75, (sbyte)36, (sbyte)7, (sbyte)28, (sbyte) - 1, (sbyte)111, (sbyte) - 55, (sbyte) - 50, (sbyte) - 1, (sbyte) - 40, (sbyte)9, (sbyte) - 71, (sbyte)1, (sbyte) - 95, (sbyte)4, (sbyte) - 116, (sbyte)93}, 0) ;
            p249.type = (byte)(byte)36;
            p249.address = (ushort)(ushort)50388;
            p249.ver = (byte)(byte)144;
            CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDEBUG_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)3904496965553689768L);
                Debug.Assert(pack.z == (float)1.6402892E38F);
                Debug.Assert(pack.x == (float)3.0663348E38F);
                Debug.Assert(pack.name_LEN(ph) == 10);
                Debug.Assert(pack.name_TRY(ph).Equals("hmXTcqqtit"));
                Debug.Assert(pack.y == (float)2.1739214E38F);
            };
            GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.time_usec = (ulong)3904496965553689768L;
            p250.z = (float)1.6402892E38F;
            p250.name_SET("hmXTcqqtit", PH) ;
            p250.y = (float)2.1739214E38F;
            p250.x = (float)3.0663348E38F;
            CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnNAMED_VALUE_FLOATReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.name_LEN(ph) == 10);
                Debug.Assert(pack.name_TRY(ph).Equals("ykKlnifrmp"));
                Debug.Assert(pack.time_boot_ms == (uint)1732198917U);
                Debug.Assert(pack.value == (float)2.7050605E38F);
            };
            GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.time_boot_ms = (uint)1732198917U;
            p251.value = (float)2.7050605E38F;
            p251.name_SET("ykKlnifrmp", PH) ;
            CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnNAMED_VALUE_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1377133566U);
                Debug.Assert(pack.value == (int) -845209630);
                Debug.Assert(pack.name_LEN(ph) == 10);
                Debug.Assert(pack.name_TRY(ph).Equals("rjjzenjjsc"));
            };
            GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.name_SET("rjjzenjjsc", PH) ;
            p252.value = (int) -845209630;
            p252.time_boot_ms = (uint)1377133566U;
            CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSTATUSTEXTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.severity == (MAV_SEVERITY)MAV_SEVERITY.MAV_SEVERITY_CRITICAL);
                Debug.Assert(pack.text_LEN(ph) == 28);
                Debug.Assert(pack.text_TRY(ph).Equals("rsirwuxyujsgqDejnduOrbssthhs"));
            };
            GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.severity = (MAV_SEVERITY)MAV_SEVERITY.MAV_SEVERITY_CRITICAL;
            p253.text_SET("rsirwuxyujsgqDejnduOrbssthhs", PH) ;
            CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDEBUGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value == (float)1.8213551E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1282371800U);
                Debug.Assert(pack.ind == (byte)(byte)171);
            };
            GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
            PH.setPack(p254);
            p254.time_boot_ms = (uint)1282371800U;
            p254.ind = (byte)(byte)171;
            p254.value = (float)1.8213551E38F;
            CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSETUP_SIGNINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.initial_timestamp == (ulong)3304335350877194587L);
                Debug.Assert(pack.target_component == (byte)(byte)26);
                Debug.Assert(pack.target_system == (byte)(byte)87);
                Debug.Assert(pack.secret_key.SequenceEqual(new byte[] {(byte)4, (byte)168, (byte)214, (byte)192, (byte)55, (byte)236, (byte)187, (byte)63, (byte)170, (byte)48, (byte)84, (byte)41, (byte)75, (byte)167, (byte)27, (byte)190, (byte)138, (byte)73, (byte)254, (byte)30, (byte)106, (byte)247, (byte)241, (byte)167, (byte)183, (byte)35, (byte)44, (byte)178, (byte)216, (byte)242, (byte)142, (byte)15}));
            };
            GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.initial_timestamp = (ulong)3304335350877194587L;
            p256.target_system = (byte)(byte)87;
            p256.secret_key_SET(new byte[] {(byte)4, (byte)168, (byte)214, (byte)192, (byte)55, (byte)236, (byte)187, (byte)63, (byte)170, (byte)48, (byte)84, (byte)41, (byte)75, (byte)167, (byte)27, (byte)190, (byte)138, (byte)73, (byte)254, (byte)30, (byte)106, (byte)247, (byte)241, (byte)167, (byte)183, (byte)35, (byte)44, (byte)178, (byte)216, (byte)242, (byte)142, (byte)15}, 0) ;
            p256.target_component = (byte)(byte)26;
            CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnBUTTON_CHANGEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)672725177U);
                Debug.Assert(pack.state == (byte)(byte)27);
                Debug.Assert(pack.last_change_ms == (uint)1214819002U);
            };
            GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.time_boot_ms = (uint)672725177U;
            p257.state = (byte)(byte)27;
            p257.last_change_ms = (uint)1214819002U;
            CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPLAY_TUNEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)157);
                Debug.Assert(pack.target_component == (byte)(byte)126);
                Debug.Assert(pack.tune_LEN(ph) == 9);
                Debug.Assert(pack.tune_TRY(ph).Equals("yymlcfdqu"));
            };
            GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_component = (byte)(byte)126;
            p258.target_system = (byte)(byte)157;
            p258.tune_SET("yymlcfdqu", PH) ;
            CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCAMERA_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.cam_definition_uri_LEN(ph) == 21);
                Debug.Assert(pack.cam_definition_uri_TRY(ph).Equals("kpjbrmzWuuslqvezlfCkr"));
                Debug.Assert(pack.resolution_h == (ushort)(ushort)15932);
                Debug.Assert(pack.cam_definition_version == (ushort)(ushort)4114);
                Debug.Assert(pack.time_boot_ms == (uint)2780971588U);
                Debug.Assert(pack.firmware_version == (uint)3516456548U);
                Debug.Assert(pack.model_name.SequenceEqual(new byte[] {(byte)95, (byte)205, (byte)96, (byte)23, (byte)235, (byte)14, (byte)62, (byte)212, (byte)16, (byte)58, (byte)64, (byte)91, (byte)215, (byte)220, (byte)243, (byte)173, (byte)97, (byte)74, (byte)211, (byte)150, (byte)94, (byte)53, (byte)179, (byte)3, (byte)253, (byte)136, (byte)133, (byte)122, (byte)180, (byte)46, (byte)152, (byte)77}));
                Debug.Assert(pack.sensor_size_h == (float) -8.4475257E37F);
                Debug.Assert(pack.focal_length == (float)2.4959478E38F);
                Debug.Assert(pack.vendor_name.SequenceEqual(new byte[] {(byte)210, (byte)124, (byte)217, (byte)75, (byte)229, (byte)151, (byte)76, (byte)145, (byte)112, (byte)152, (byte)56, (byte)150, (byte)32, (byte)177, (byte)8, (byte)159, (byte)133, (byte)65, (byte)34, (byte)122, (byte)29, (byte)7, (byte)61, (byte)4, (byte)220, (byte)128, (byte)144, (byte)196, (byte)96, (byte)90, (byte)196, (byte)231}));
                Debug.Assert(pack.lens_id == (byte)(byte)135);
                Debug.Assert(pack.sensor_size_v == (float) -5.761339E37F);
                Debug.Assert(pack.flags == (CAMERA_CAP_FLAGS)CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)54124);
            };
            GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.focal_length = (float)2.4959478E38F;
            p259.resolution_v = (ushort)(ushort)54124;
            p259.resolution_h = (ushort)(ushort)15932;
            p259.cam_definition_uri_SET("kpjbrmzWuuslqvezlfCkr", PH) ;
            p259.sensor_size_v = (float) -5.761339E37F;
            p259.model_name_SET(new byte[] {(byte)95, (byte)205, (byte)96, (byte)23, (byte)235, (byte)14, (byte)62, (byte)212, (byte)16, (byte)58, (byte)64, (byte)91, (byte)215, (byte)220, (byte)243, (byte)173, (byte)97, (byte)74, (byte)211, (byte)150, (byte)94, (byte)53, (byte)179, (byte)3, (byte)253, (byte)136, (byte)133, (byte)122, (byte)180, (byte)46, (byte)152, (byte)77}, 0) ;
            p259.time_boot_ms = (uint)2780971588U;
            p259.vendor_name_SET(new byte[] {(byte)210, (byte)124, (byte)217, (byte)75, (byte)229, (byte)151, (byte)76, (byte)145, (byte)112, (byte)152, (byte)56, (byte)150, (byte)32, (byte)177, (byte)8, (byte)159, (byte)133, (byte)65, (byte)34, (byte)122, (byte)29, (byte)7, (byte)61, (byte)4, (byte)220, (byte)128, (byte)144, (byte)196, (byte)96, (byte)90, (byte)196, (byte)231}, 0) ;
            p259.firmware_version = (uint)3516456548U;
            p259.lens_id = (byte)(byte)135;
            p259.cam_definition_version = (ushort)(ushort)4114;
            p259.sensor_size_h = (float) -8.4475257E37F;
            p259.flags = (CAMERA_CAP_FLAGS)CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE;
            CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCAMERA_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mode_id == (CAMERA_MODE)CAMERA_MODE.CAMERA_MODE_IMAGE);
                Debug.Assert(pack.time_boot_ms == (uint)3770970503U);
            };
            GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.time_boot_ms = (uint)3770970503U;
            p260.mode_id = (CAMERA_MODE)CAMERA_MODE.CAMERA_MODE_IMAGE;
            CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSTORAGE_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.used_capacity == (float)2.99216E38F);
                Debug.Assert(pack.available_capacity == (float)1.339423E38F);
                Debug.Assert(pack.storage_id == (byte)(byte)120);
                Debug.Assert(pack.write_speed == (float) -1.3681185E38F);
                Debug.Assert(pack.status == (byte)(byte)107);
                Debug.Assert(pack.storage_count == (byte)(byte)158);
                Debug.Assert(pack.total_capacity == (float)2.0464805E38F);
                Debug.Assert(pack.read_speed == (float)2.459943E38F);
                Debug.Assert(pack.time_boot_ms == (uint)4085281173U);
            };
            GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.total_capacity = (float)2.0464805E38F;
            p261.status = (byte)(byte)107;
            p261.used_capacity = (float)2.99216E38F;
            p261.read_speed = (float)2.459943E38F;
            p261.available_capacity = (float)1.339423E38F;
            p261.write_speed = (float) -1.3681185E38F;
            p261.time_boot_ms = (uint)4085281173U;
            p261.storage_id = (byte)(byte)120;
            p261.storage_count = (byte)(byte)158;
            CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCAMERA_CAPTURE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.image_interval == (float)2.1258567E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1214430663U);
                Debug.Assert(pack.recording_time_ms == (uint)2974549364U);
                Debug.Assert(pack.image_status == (byte)(byte)86);
                Debug.Assert(pack.video_status == (byte)(byte)5);
                Debug.Assert(pack.available_capacity == (float)3.2557151E38F);
            };
            GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.image_interval = (float)2.1258567E38F;
            p262.time_boot_ms = (uint)1214430663U;
            p262.video_status = (byte)(byte)5;
            p262.available_capacity = (float)3.2557151E38F;
            p262.image_status = (byte)(byte)86;
            p262.recording_time_ms = (uint)2974549364U;
            CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCAMERA_IMAGE_CAPTUREDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q.SequenceEqual(new float[] {-3.3972448E38F, -8.935567E37F, 5.1013515E37F, -8.570625E37F}));
                Debug.Assert(pack.time_utc == (ulong)6244168410015717731L);
                Debug.Assert(pack.relative_alt == (int) -2066529710);
                Debug.Assert(pack.camera_id == (byte)(byte)147);
                Debug.Assert(pack.alt == (int) -89428456);
                Debug.Assert(pack.lat == (int)920069240);
                Debug.Assert(pack.lon == (int) -1885147291);
                Debug.Assert(pack.capture_result == (sbyte)(sbyte) - 65);
                Debug.Assert(pack.image_index == (int) -1158662330);
                Debug.Assert(pack.file_url_LEN(ph) == 66);
                Debug.Assert(pack.file_url_TRY(ph).Equals("eyfzdiXnsqicCsloutUdmqeaoybcvhxslowgcqdLjsxybdwgexnomyhfafzQmljdjx"));
                Debug.Assert(pack.time_boot_ms == (uint)3338787159U);
            };
            GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.lon = (int) -1885147291;
            p263.time_boot_ms = (uint)3338787159U;
            p263.lat = (int)920069240;
            p263.image_index = (int) -1158662330;
            p263.file_url_SET("eyfzdiXnsqicCsloutUdmqeaoybcvhxslowgcqdLjsxybdwgexnomyhfafzQmljdjx", PH) ;
            p263.capture_result = (sbyte)(sbyte) - 65;
            p263.time_utc = (ulong)6244168410015717731L;
            p263.alt = (int) -89428456;
            p263.q_SET(new float[] {-3.3972448E38F, -8.935567E37F, 5.1013515E37F, -8.570625E37F}, 0) ;
            p263.camera_id = (byte)(byte)147;
            p263.relative_alt = (int) -2066529710;
            CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            ADV_TEST_CH.OnFLIGHT_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.takeoff_time_utc == (ulong)1188759926618920402L);
                Debug.Assert(pack.flight_uuid == (ulong)435288202981638559L);
                Debug.Assert(pack.arming_time_utc == (ulong)7000657796588793206L);
                Debug.Assert(pack.time_boot_ms == (uint)694356645U);
            };
            GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.time_boot_ms = (uint)694356645U;
            p264.arming_time_utc = (ulong)7000657796588793206L;
            p264.takeoff_time_utc = (ulong)1188759926618920402L;
            p264.flight_uuid = (ulong)435288202981638559L;
            CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMOUNT_ORIENTATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)603657549U);
                Debug.Assert(pack.roll == (float) -2.8956527E38F);
                Debug.Assert(pack.yaw == (float) -3.3868959E38F);
                Debug.Assert(pack.pitch == (float) -1.75431E37F);
            };
            GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.pitch = (float) -1.75431E37F;
            p265.time_boot_ms = (uint)603657549U;
            p265.yaw = (float) -3.3868959E38F;
            p265.roll = (float) -2.8956527E38F;
            CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.first_message_offset == (byte)(byte)229);
                Debug.Assert(pack.sequence == (ushort)(ushort)18345);
                Debug.Assert(pack.target_component == (byte)(byte)219);
                Debug.Assert(pack.length == (byte)(byte)94);
                Debug.Assert(pack.target_system == (byte)(byte)153);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)192, (byte)112, (byte)178, (byte)152, (byte)63, (byte)78, (byte)70, (byte)19, (byte)250, (byte)226, (byte)45, (byte)178, (byte)57, (byte)108, (byte)16, (byte)11, (byte)72, (byte)242, (byte)128, (byte)9, (byte)92, (byte)188, (byte)100, (byte)99, (byte)150, (byte)133, (byte)193, (byte)133, (byte)177, (byte)244, (byte)25, (byte)225, (byte)166, (byte)29, (byte)139, (byte)139, (byte)252, (byte)200, (byte)42, (byte)182, (byte)168, (byte)25, (byte)95, (byte)19, (byte)140, (byte)210, (byte)106, (byte)71, (byte)76, (byte)189, (byte)207, (byte)14, (byte)211, (byte)9, (byte)72, (byte)83, (byte)91, (byte)136, (byte)178, (byte)152, (byte)183, (byte)218, (byte)102, (byte)202, (byte)229, (byte)204, (byte)119, (byte)117, (byte)164, (byte)67, (byte)65, (byte)247, (byte)110, (byte)193, (byte)162, (byte)224, (byte)178, (byte)196, (byte)40, (byte)89, (byte)60, (byte)97, (byte)169, (byte)31, (byte)155, (byte)168, (byte)95, (byte)140, (byte)196, (byte)61, (byte)218, (byte)58, (byte)68, (byte)2, (byte)189, (byte)39, (byte)116, (byte)13, (byte)15, (byte)181, (byte)4, (byte)173, (byte)22, (byte)15, (byte)196, (byte)214, (byte)205, (byte)115, (byte)3, (byte)218, (byte)157, (byte)132, (byte)7, (byte)203, (byte)110, (byte)102, (byte)186, (byte)239, (byte)223, (byte)235, (byte)148, (byte)185, (byte)89, (byte)39, (byte)84, (byte)152, (byte)72, (byte)177, (byte)200, (byte)23, (byte)10, (byte)22, (byte)157, (byte)161, (byte)166, (byte)199, (byte)138, (byte)192, (byte)192, (byte)37, (byte)193, (byte)138, (byte)81, (byte)109, (byte)167, (byte)79, (byte)225, (byte)156, (byte)110, (byte)164, (byte)56, (byte)23, (byte)106, (byte)34, (byte)135, (byte)43, (byte)215, (byte)213, (byte)180, (byte)170, (byte)22, (byte)132, (byte)122, (byte)139, (byte)171, (byte)24, (byte)81, (byte)122, (byte)15, (byte)244, (byte)254, (byte)175, (byte)170, (byte)9, (byte)41, (byte)206, (byte)237, (byte)79, (byte)132, (byte)155, (byte)123, (byte)76, (byte)99, (byte)238, (byte)9, (byte)175, (byte)175, (byte)120, (byte)116, (byte)253, (byte)68, (byte)109, (byte)232, (byte)107, (byte)125, (byte)28, (byte)3, (byte)202, (byte)254, (byte)188, (byte)187, (byte)233, (byte)114, (byte)135, (byte)166, (byte)201, (byte)61, (byte)40, (byte)112, (byte)156, (byte)184, (byte)131, (byte)95, (byte)27, (byte)11, (byte)44, (byte)213, (byte)173, (byte)129, (byte)170, (byte)148, (byte)46, (byte)92, (byte)68, (byte)150, (byte)224, (byte)88, (byte)81, (byte)10, (byte)108, (byte)175, (byte)68, (byte)80, (byte)83, (byte)158, (byte)197, (byte)132, (byte)172, (byte)106, (byte)30, (byte)206, (byte)181, (byte)138, (byte)155, (byte)45, (byte)32, (byte)71, (byte)48, (byte)250}));
            };
            GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.first_message_offset = (byte)(byte)229;
            p266.target_system = (byte)(byte)153;
            p266.data__SET(new byte[] {(byte)192, (byte)112, (byte)178, (byte)152, (byte)63, (byte)78, (byte)70, (byte)19, (byte)250, (byte)226, (byte)45, (byte)178, (byte)57, (byte)108, (byte)16, (byte)11, (byte)72, (byte)242, (byte)128, (byte)9, (byte)92, (byte)188, (byte)100, (byte)99, (byte)150, (byte)133, (byte)193, (byte)133, (byte)177, (byte)244, (byte)25, (byte)225, (byte)166, (byte)29, (byte)139, (byte)139, (byte)252, (byte)200, (byte)42, (byte)182, (byte)168, (byte)25, (byte)95, (byte)19, (byte)140, (byte)210, (byte)106, (byte)71, (byte)76, (byte)189, (byte)207, (byte)14, (byte)211, (byte)9, (byte)72, (byte)83, (byte)91, (byte)136, (byte)178, (byte)152, (byte)183, (byte)218, (byte)102, (byte)202, (byte)229, (byte)204, (byte)119, (byte)117, (byte)164, (byte)67, (byte)65, (byte)247, (byte)110, (byte)193, (byte)162, (byte)224, (byte)178, (byte)196, (byte)40, (byte)89, (byte)60, (byte)97, (byte)169, (byte)31, (byte)155, (byte)168, (byte)95, (byte)140, (byte)196, (byte)61, (byte)218, (byte)58, (byte)68, (byte)2, (byte)189, (byte)39, (byte)116, (byte)13, (byte)15, (byte)181, (byte)4, (byte)173, (byte)22, (byte)15, (byte)196, (byte)214, (byte)205, (byte)115, (byte)3, (byte)218, (byte)157, (byte)132, (byte)7, (byte)203, (byte)110, (byte)102, (byte)186, (byte)239, (byte)223, (byte)235, (byte)148, (byte)185, (byte)89, (byte)39, (byte)84, (byte)152, (byte)72, (byte)177, (byte)200, (byte)23, (byte)10, (byte)22, (byte)157, (byte)161, (byte)166, (byte)199, (byte)138, (byte)192, (byte)192, (byte)37, (byte)193, (byte)138, (byte)81, (byte)109, (byte)167, (byte)79, (byte)225, (byte)156, (byte)110, (byte)164, (byte)56, (byte)23, (byte)106, (byte)34, (byte)135, (byte)43, (byte)215, (byte)213, (byte)180, (byte)170, (byte)22, (byte)132, (byte)122, (byte)139, (byte)171, (byte)24, (byte)81, (byte)122, (byte)15, (byte)244, (byte)254, (byte)175, (byte)170, (byte)9, (byte)41, (byte)206, (byte)237, (byte)79, (byte)132, (byte)155, (byte)123, (byte)76, (byte)99, (byte)238, (byte)9, (byte)175, (byte)175, (byte)120, (byte)116, (byte)253, (byte)68, (byte)109, (byte)232, (byte)107, (byte)125, (byte)28, (byte)3, (byte)202, (byte)254, (byte)188, (byte)187, (byte)233, (byte)114, (byte)135, (byte)166, (byte)201, (byte)61, (byte)40, (byte)112, (byte)156, (byte)184, (byte)131, (byte)95, (byte)27, (byte)11, (byte)44, (byte)213, (byte)173, (byte)129, (byte)170, (byte)148, (byte)46, (byte)92, (byte)68, (byte)150, (byte)224, (byte)88, (byte)81, (byte)10, (byte)108, (byte)175, (byte)68, (byte)80, (byte)83, (byte)158, (byte)197, (byte)132, (byte)172, (byte)106, (byte)30, (byte)206, (byte)181, (byte)138, (byte)155, (byte)45, (byte)32, (byte)71, (byte)48, (byte)250}, 0) ;
            p266.length = (byte)(byte)94;
            p266.sequence = (ushort)(ushort)18345;
            p266.target_component = (byte)(byte)219;
            CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATA_ACKEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)46, (byte)204, (byte)217, (byte)224, (byte)202, (byte)152, (byte)53, (byte)148, (byte)246, (byte)110, (byte)210, (byte)89, (byte)180, (byte)173, (byte)248, (byte)32, (byte)228, (byte)116, (byte)127, (byte)123, (byte)183, (byte)193, (byte)195, (byte)55, (byte)240, (byte)168, (byte)8, (byte)244, (byte)147, (byte)97, (byte)4, (byte)116, (byte)31, (byte)91, (byte)33, (byte)201, (byte)204, (byte)35, (byte)105, (byte)78, (byte)59, (byte)25, (byte)197, (byte)35, (byte)209, (byte)111, (byte)170, (byte)85, (byte)28, (byte)126, (byte)246, (byte)23, (byte)184, (byte)164, (byte)41, (byte)18, (byte)117, (byte)100, (byte)199, (byte)90, (byte)164, (byte)54, (byte)237, (byte)64, (byte)85, (byte)118, (byte)207, (byte)144, (byte)187, (byte)49, (byte)30, (byte)140, (byte)130, (byte)68, (byte)228, (byte)143, (byte)28, (byte)130, (byte)54, (byte)150, (byte)65, (byte)139, (byte)113, (byte)34, (byte)55, (byte)105, (byte)111, (byte)134, (byte)190, (byte)250, (byte)194, (byte)217, (byte)72, (byte)51, (byte)252, (byte)46, (byte)57, (byte)115, (byte)55, (byte)81, (byte)105, (byte)83, (byte)191, (byte)152, (byte)50, (byte)10, (byte)14, (byte)55, (byte)87, (byte)19, (byte)197, (byte)43, (byte)21, (byte)22, (byte)104, (byte)1, (byte)66, (byte)9, (byte)78, (byte)72, (byte)227, (byte)108, (byte)189, (byte)217, (byte)213, (byte)22, (byte)172, (byte)250, (byte)18, (byte)110, (byte)208, (byte)172, (byte)1, (byte)229, (byte)240, (byte)42, (byte)98, (byte)37, (byte)45, (byte)190, (byte)232, (byte)33, (byte)14, (byte)115, (byte)223, (byte)167, (byte)117, (byte)174, (byte)34, (byte)236, (byte)164, (byte)11, (byte)100, (byte)213, (byte)143, (byte)99, (byte)144, (byte)107, (byte)80, (byte)88, (byte)37, (byte)14, (byte)111, (byte)249, (byte)212, (byte)119, (byte)86, (byte)59, (byte)5, (byte)221, (byte)69, (byte)44, (byte)232, (byte)85, (byte)184, (byte)55, (byte)65, (byte)255, (byte)182, (byte)59, (byte)239, (byte)47, (byte)174, (byte)221, (byte)56, (byte)12, (byte)60, (byte)110, (byte)166, (byte)20, (byte)28, (byte)151, (byte)132, (byte)136, (byte)203, (byte)114, (byte)15, (byte)76, (byte)100, (byte)172, (byte)178, (byte)24, (byte)67, (byte)41, (byte)207, (byte)211, (byte)133, (byte)67, (byte)134, (byte)128, (byte)226, (byte)215, (byte)98, (byte)75, (byte)31, (byte)28, (byte)236, (byte)203, (byte)170, (byte)154, (byte)222, (byte)179, (byte)178, (byte)32, (byte)3, (byte)214, (byte)230, (byte)97, (byte)88, (byte)3, (byte)14, (byte)232, (byte)56, (byte)198, (byte)83, (byte)41, (byte)225, (byte)216, (byte)79, (byte)136, (byte)55, (byte)37, (byte)36, (byte)135, (byte)121, (byte)171, (byte)192, (byte)246, (byte)164}));
                Debug.Assert(pack.target_component == (byte)(byte)51);
                Debug.Assert(pack.length == (byte)(byte)111);
                Debug.Assert(pack.sequence == (ushort)(ushort)22130);
                Debug.Assert(pack.first_message_offset == (byte)(byte)91);
                Debug.Assert(pack.target_system == (byte)(byte)163);
            };
            GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.first_message_offset = (byte)(byte)91;
            p267.target_component = (byte)(byte)51;
            p267.sequence = (ushort)(ushort)22130;
            p267.data__SET(new byte[] {(byte)46, (byte)204, (byte)217, (byte)224, (byte)202, (byte)152, (byte)53, (byte)148, (byte)246, (byte)110, (byte)210, (byte)89, (byte)180, (byte)173, (byte)248, (byte)32, (byte)228, (byte)116, (byte)127, (byte)123, (byte)183, (byte)193, (byte)195, (byte)55, (byte)240, (byte)168, (byte)8, (byte)244, (byte)147, (byte)97, (byte)4, (byte)116, (byte)31, (byte)91, (byte)33, (byte)201, (byte)204, (byte)35, (byte)105, (byte)78, (byte)59, (byte)25, (byte)197, (byte)35, (byte)209, (byte)111, (byte)170, (byte)85, (byte)28, (byte)126, (byte)246, (byte)23, (byte)184, (byte)164, (byte)41, (byte)18, (byte)117, (byte)100, (byte)199, (byte)90, (byte)164, (byte)54, (byte)237, (byte)64, (byte)85, (byte)118, (byte)207, (byte)144, (byte)187, (byte)49, (byte)30, (byte)140, (byte)130, (byte)68, (byte)228, (byte)143, (byte)28, (byte)130, (byte)54, (byte)150, (byte)65, (byte)139, (byte)113, (byte)34, (byte)55, (byte)105, (byte)111, (byte)134, (byte)190, (byte)250, (byte)194, (byte)217, (byte)72, (byte)51, (byte)252, (byte)46, (byte)57, (byte)115, (byte)55, (byte)81, (byte)105, (byte)83, (byte)191, (byte)152, (byte)50, (byte)10, (byte)14, (byte)55, (byte)87, (byte)19, (byte)197, (byte)43, (byte)21, (byte)22, (byte)104, (byte)1, (byte)66, (byte)9, (byte)78, (byte)72, (byte)227, (byte)108, (byte)189, (byte)217, (byte)213, (byte)22, (byte)172, (byte)250, (byte)18, (byte)110, (byte)208, (byte)172, (byte)1, (byte)229, (byte)240, (byte)42, (byte)98, (byte)37, (byte)45, (byte)190, (byte)232, (byte)33, (byte)14, (byte)115, (byte)223, (byte)167, (byte)117, (byte)174, (byte)34, (byte)236, (byte)164, (byte)11, (byte)100, (byte)213, (byte)143, (byte)99, (byte)144, (byte)107, (byte)80, (byte)88, (byte)37, (byte)14, (byte)111, (byte)249, (byte)212, (byte)119, (byte)86, (byte)59, (byte)5, (byte)221, (byte)69, (byte)44, (byte)232, (byte)85, (byte)184, (byte)55, (byte)65, (byte)255, (byte)182, (byte)59, (byte)239, (byte)47, (byte)174, (byte)221, (byte)56, (byte)12, (byte)60, (byte)110, (byte)166, (byte)20, (byte)28, (byte)151, (byte)132, (byte)136, (byte)203, (byte)114, (byte)15, (byte)76, (byte)100, (byte)172, (byte)178, (byte)24, (byte)67, (byte)41, (byte)207, (byte)211, (byte)133, (byte)67, (byte)134, (byte)128, (byte)226, (byte)215, (byte)98, (byte)75, (byte)31, (byte)28, (byte)236, (byte)203, (byte)170, (byte)154, (byte)222, (byte)179, (byte)178, (byte)32, (byte)3, (byte)214, (byte)230, (byte)97, (byte)88, (byte)3, (byte)14, (byte)232, (byte)56, (byte)198, (byte)83, (byte)41, (byte)225, (byte)216, (byte)79, (byte)136, (byte)55, (byte)37, (byte)36, (byte)135, (byte)121, (byte)171, (byte)192, (byte)246, (byte)164}, 0) ;
            p267.target_system = (byte)(byte)163;
            p267.length = (byte)(byte)111;
            CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLOGGING_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)232);
                Debug.Assert(pack.sequence == (ushort)(ushort)54434);
                Debug.Assert(pack.target_system == (byte)(byte)91);
            };
            GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_system = (byte)(byte)91;
            p268.target_component = (byte)(byte)232;
            p268.sequence = (ushort)(ushort)54434;
            CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnVIDEO_STREAM_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.framerate == (float)1.7679704E38F);
                Debug.Assert(pack.camera_id == (byte)(byte)139);
                Debug.Assert(pack.uri_LEN(ph) == 175);
                Debug.Assert(pack.uri_TRY(ph).Equals("rzuomaoItujIsxuYylspzmsFrclmdfqevmywqgoeitDseeckrgbjcApwlckbikbaDqueopiypyeihzzigspzxnRgehxxumnbhdcluqvoktFvfkijAiyolhluwMhlxvGfeqdvhSrkJlszrzlviubmdxqfqcxpanarazlpzsftvnqjVcb"));
                Debug.Assert(pack.resolution_h == (ushort)(ushort)42570);
                Debug.Assert(pack.rotation == (ushort)(ushort)51122);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)51125);
                Debug.Assert(pack.status == (byte)(byte)128);
                Debug.Assert(pack.bitrate == (uint)1280651212U);
            };
            GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.status = (byte)(byte)128;
            p269.resolution_v = (ushort)(ushort)51125;
            p269.uri_SET("rzuomaoItujIsxuYylspzmsFrclmdfqevmywqgoeitDseeckrgbjcApwlckbikbaDqueopiypyeihzzigspzxnRgehxxumnbhdcluqvoktFvfkijAiyolhluwMhlxvGfeqdvhSrkJlszrzlviubmdxqfqcxpanarazlpzsftvnqjVcb", PH) ;
            p269.resolution_h = (ushort)(ushort)42570;
            p269.framerate = (float)1.7679704E38F;
            p269.camera_id = (byte)(byte)139;
            p269.bitrate = (uint)1280651212U;
            p269.rotation = (ushort)(ushort)51122;
            CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSET_VIDEO_STREAM_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.resolution_v == (ushort)(ushort)62658);
                Debug.Assert(pack.framerate == (float)6.464128E37F);
                Debug.Assert(pack.camera_id == (byte)(byte)99);
                Debug.Assert(pack.target_system == (byte)(byte)190);
                Debug.Assert(pack.uri_LEN(ph) == 33);
                Debug.Assert(pack.uri_TRY(ph).Equals("zjwqcgZhlkzggRukpvakpztorvlzmxjsy"));
                Debug.Assert(pack.rotation == (ushort)(ushort)40177);
                Debug.Assert(pack.target_component == (byte)(byte)173);
                Debug.Assert(pack.bitrate == (uint)3490811182U);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)7268);
            };
            GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.framerate = (float)6.464128E37F;
            p270.bitrate = (uint)3490811182U;
            p270.uri_SET("zjwqcgZhlkzggRukpvakpztorvlzmxjsy", PH) ;
            p270.rotation = (ushort)(ushort)40177;
            p270.target_system = (byte)(byte)190;
            p270.target_component = (byte)(byte)173;
            p270.camera_id = (byte)(byte)99;
            p270.resolution_v = (ushort)(ushort)62658;
            p270.resolution_h = (ushort)(ushort)7268;
            CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnWIFI_CONFIG_APReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ssid_LEN(ph) == 32);
                Debug.Assert(pack.ssid_TRY(ph).Equals("ClowxdGpuzqaqfcjwmxlddxrlarwsjOa"));
                Debug.Assert(pack.password_LEN(ph) == 31);
                Debug.Assert(pack.password_TRY(ph).Equals("peprfQgzbxxhkcEncgTzilvkwhdvjAh"));
            };
            GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.ssid_SET("ClowxdGpuzqaqfcjwmxlddxrlarwsjOa", PH) ;
            p299.password_SET("peprfQgzbxxhkcEncgTzilvkwhdvjAh", PH) ;
            CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPROTOCOL_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.version == (ushort)(ushort)11868);
                Debug.Assert(pack.min_version == (ushort)(ushort)18191);
                Debug.Assert(pack.spec_version_hash.SequenceEqual(new byte[] {(byte)6, (byte)17, (byte)89, (byte)55, (byte)19, (byte)197, (byte)254, (byte)9}));
                Debug.Assert(pack.max_version == (ushort)(ushort)40233);
                Debug.Assert(pack.library_version_hash.SequenceEqual(new byte[] {(byte)66, (byte)206, (byte)69, (byte)27, (byte)163, (byte)152, (byte)67, (byte)168}));
            };
            GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.min_version = (ushort)(ushort)18191;
            p300.version = (ushort)(ushort)11868;
            p300.spec_version_hash_SET(new byte[] {(byte)6, (byte)17, (byte)89, (byte)55, (byte)19, (byte)197, (byte)254, (byte)9}, 0) ;
            p300.library_version_hash_SET(new byte[] {(byte)66, (byte)206, (byte)69, (byte)27, (byte)163, (byte)152, (byte)67, (byte)168}, 0) ;
            p300.max_version = (ushort)(ushort)40233;
            CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)692741914187871962L);
                Debug.Assert(pack.vendor_specific_status_code == (ushort)(ushort)28680);
                Debug.Assert(pack.sub_mode == (byte)(byte)43);
                Debug.Assert(pack.uptime_sec == (uint)1920833510U);
                Debug.Assert(pack.mode == (UAVCAN_NODE_MODE)UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION);
                Debug.Assert(pack.health == (UAVCAN_NODE_HEALTH)UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING);
            };
            GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.vendor_specific_status_code = (ushort)(ushort)28680;
            p310.uptime_sec = (uint)1920833510U;
            p310.health = (UAVCAN_NODE_HEALTH)UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING;
            p310.mode = (UAVCAN_NODE_MODE)UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION;
            p310.sub_mode = (byte)(byte)43;
            p310.time_usec = (ulong)692741914187871962L;
            CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_INFOReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uptime_sec == (uint)4055463578U);
                Debug.Assert(pack.hw_unique_id.SequenceEqual(new byte[] {(byte)45, (byte)219, (byte)136, (byte)170, (byte)124, (byte)132, (byte)81, (byte)151, (byte)40, (byte)245, (byte)213, (byte)81, (byte)27, (byte)205, (byte)17, (byte)216}));
                Debug.Assert(pack.sw_version_minor == (byte)(byte)216);
                Debug.Assert(pack.name_LEN(ph) == 4);
                Debug.Assert(pack.name_TRY(ph).Equals("tqah"));
                Debug.Assert(pack.time_usec == (ulong)5267912534395537213L);
                Debug.Assert(pack.sw_version_major == (byte)(byte)254);
                Debug.Assert(pack.sw_vcs_commit == (uint)4262885876U);
                Debug.Assert(pack.hw_version_minor == (byte)(byte)129);
                Debug.Assert(pack.hw_version_major == (byte)(byte)186);
            };
            GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.sw_version_minor = (byte)(byte)216;
            p311.name_SET("tqah", PH) ;
            p311.hw_unique_id_SET(new byte[] {(byte)45, (byte)219, (byte)136, (byte)170, (byte)124, (byte)132, (byte)81, (byte)151, (byte)40, (byte)245, (byte)213, (byte)81, (byte)27, (byte)205, (byte)17, (byte)216}, 0) ;
            p311.sw_vcs_commit = (uint)4262885876U;
            p311.hw_version_minor = (byte)(byte)129;
            p311.sw_version_major = (byte)(byte)254;
            p311.hw_version_major = (byte)(byte)186;
            p311.uptime_sec = (uint)4055463578U;
            p311.time_usec = (ulong)5267912534395537213L;
            CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_index == (short)(short) -3590);
                Debug.Assert(pack.param_id_LEN(ph) == 12);
                Debug.Assert(pack.param_id_TRY(ph).Equals("yymtlgiffncn"));
                Debug.Assert(pack.target_component == (byte)(byte)33);
                Debug.Assert(pack.target_system == (byte)(byte)136);
            };
            GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.param_id_SET("yymtlgiffncn", PH) ;
            p320.param_index = (short)(short) -3590;
            p320.target_component = (byte)(byte)33;
            p320.target_system = (byte)(byte)136;
            CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)62);
                Debug.Assert(pack.target_component == (byte)(byte)27);
            };
            GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_component = (byte)(byte)27;
            p321.target_system = (byte)(byte)62;
            CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 8);
                Debug.Assert(pack.param_id_TRY(ph).Equals("LpAkzofl"));
                Debug.Assert(pack.param_value_LEN(ph) == 29);
                Debug.Assert(pack.param_value_TRY(ph).Equals("urzaxpTwouryfftddaqfbyhyunrlv"));
                Debug.Assert(pack.param_index == (ushort)(ushort)699);
                Debug.Assert(pack.param_count == (ushort)(ushort)53878);
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32);
            };
            GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_count = (ushort)(ushort)53878;
            p322.param_index = (ushort)(ushort)699;
            p322.param_value_SET("urzaxpTwouryfftddaqfbyhyunrlv", PH) ;
            p322.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32;
            p322.param_id_SET("LpAkzofl", PH) ;
            CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 1);
                Debug.Assert(pack.param_id_TRY(ph).Equals("u"));
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64);
                Debug.Assert(pack.target_system == (byte)(byte)82);
                Debug.Assert(pack.target_component == (byte)(byte)154);
                Debug.Assert(pack.param_value_LEN(ph) == 115);
                Debug.Assert(pack.param_value_TRY(ph).Equals("ldokqBeroboakgjfcdgulmUcdCrbZhmhwxNhxrgjohThmyyaRncgJtuxfixhurhbaiprZclrydrismvuzpflNdcBqxhvfNzxfFrzzixzfazkvnxnbga"));
            };
            GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.param_id_SET("u", PH) ;
            p323.param_value_SET("ldokqBeroboakgjfcdgulmUcdCrbZhmhwxNhxrgjohThmyyaRncgJtuxfixhurhbaiprZclrydrismvuzpflNdcBqxhvfNzxfFrzzixzfazkvnxnbga", PH) ;
            p323.target_system = (byte)(byte)82;
            p323.target_component = (byte)(byte)154;
            p323.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64;
            CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_result == (PARAM_ACK)PARAM_ACK.PARAM_ACK_FAILED);
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64);
                Debug.Assert(pack.param_value_LEN(ph) == 115);
                Debug.Assert(pack.param_value_TRY(ph).Equals("sbqwuFyakiloqxXzmwwpyyuexgzqbjsxSxmtpxsixxvalafipwNblpulwRfxApdmbxeSjnnfksQfpnNttYiiBhYtxmcwmlmcymswwkbqrhfJqvfutyf"));
                Debug.Assert(pack.param_id_LEN(ph) == 5);
                Debug.Assert(pack.param_id_TRY(ph).Equals("fbfip"));
            };
            GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_value_SET("sbqwuFyakiloqxXzmwwpyyuexgzqbjsxSxmtpxsixxvalafipwNblpulwRfxApdmbxeSjnnfksQfpnNttYiiBhYtxmcwmlmcymswwkbqrhfJqvfutyf", PH) ;
            p324.param_result = (PARAM_ACK)PARAM_ACK.PARAM_ACK_FAILED;
            p324.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64;
            p324.param_id_SET("fbfip", PH) ;
            CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnOBSTACLE_DISTANCEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.min_distance == (ushort)(ushort)32879);
                Debug.Assert(pack.max_distance == (ushort)(ushort)4113);
                Debug.Assert(pack.distances.SequenceEqual(new ushort[] {(ushort)1501, (ushort)38811, (ushort)37843, (ushort)51172, (ushort)937, (ushort)22884, (ushort)13244, (ushort)57629, (ushort)57923, (ushort)14517, (ushort)28179, (ushort)57752, (ushort)44460, (ushort)59533, (ushort)30363, (ushort)15037, (ushort)62842, (ushort)28744, (ushort)25027, (ushort)18766, (ushort)59679, (ushort)56841, (ushort)57165, (ushort)5395, (ushort)57449, (ushort)40347, (ushort)29787, (ushort)22122, (ushort)55479, (ushort)57935, (ushort)54856, (ushort)48891, (ushort)47225, (ushort)29343, (ushort)34466, (ushort)19910, (ushort)15049, (ushort)65530, (ushort)17775, (ushort)22393, (ushort)22514, (ushort)19348, (ushort)5183, (ushort)34455, (ushort)56784, (ushort)9137, (ushort)17967, (ushort)52310, (ushort)36756, (ushort)62104, (ushort)48749, (ushort)55572, (ushort)2178, (ushort)17944, (ushort)43645, (ushort)30333, (ushort)35515, (ushort)16902, (ushort)9416, (ushort)102, (ushort)64781, (ushort)50694, (ushort)50024, (ushort)34071, (ushort)27046, (ushort)29192, (ushort)16533, (ushort)44483, (ushort)64842, (ushort)32039, (ushort)27855, (ushort)27291}));
                Debug.Assert(pack.time_usec == (ulong)2627908589618031527L);
                Debug.Assert(pack.increment == (byte)(byte)200);
                Debug.Assert(pack.sensor_type == (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND);
            };
            GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.min_distance = (ushort)(ushort)32879;
            p330.sensor_type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND;
            p330.max_distance = (ushort)(ushort)4113;
            p330.increment = (byte)(byte)200;
            p330.time_usec = (ulong)2627908589618031527L;
            p330.distances_SET(new ushort[] {(ushort)1501, (ushort)38811, (ushort)37843, (ushort)51172, (ushort)937, (ushort)22884, (ushort)13244, (ushort)57629, (ushort)57923, (ushort)14517, (ushort)28179, (ushort)57752, (ushort)44460, (ushort)59533, (ushort)30363, (ushort)15037, (ushort)62842, (ushort)28744, (ushort)25027, (ushort)18766, (ushort)59679, (ushort)56841, (ushort)57165, (ushort)5395, (ushort)57449, (ushort)40347, (ushort)29787, (ushort)22122, (ushort)55479, (ushort)57935, (ushort)54856, (ushort)48891, (ushort)47225, (ushort)29343, (ushort)34466, (ushort)19910, (ushort)15049, (ushort)65530, (ushort)17775, (ushort)22393, (ushort)22514, (ushort)19348, (ushort)5183, (ushort)34455, (ushort)56784, (ushort)9137, (ushort)17967, (ushort)52310, (ushort)36756, (ushort)62104, (ushort)48749, (ushort)55572, (ushort)2178, (ushort)17944, (ushort)43645, (ushort)30333, (ushort)35515, (ushort)16902, (ushort)9416, (ushort)102, (ushort)64781, (ushort)50694, (ushort)50024, (ushort)34071, (ushort)27046, (ushort)29192, (ushort)16533, (ushort)44483, (ushort)64842, (ushort)32039, (ushort)27855, (ushort)27291}, 0) ;
            CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnUAVIONIX_ADSB_OUT_CFGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.gpsOffsetLat == (UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT)UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_6M);
                Debug.Assert(pack.aircraftSize == (UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE)UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE.UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L55_52M);
                Debug.Assert(pack.emitterType == (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UAV);
                Debug.Assert(pack.rfSelect == (UAVIONIX_ADSB_OUT_RF_SELECT)UAVIONIX_ADSB_OUT_RF_SELECT.UAVIONIX_ADSB_OUT_RF_SELECT_RX_ENABLED);
                Debug.Assert(pack.callsign_LEN(ph) == 2);
                Debug.Assert(pack.callsign_TRY(ph).Equals("bh"));
                Debug.Assert(pack.stallSpeed == (ushort)(ushort)7976);
                Debug.Assert(pack.gpsOffsetLon == (UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON)UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_APPLIED_BY_SENSOR);
                Debug.Assert(pack.ICAO == (uint)2607399053U);
            };
            GroundControl.UAVIONIX_ADSB_OUT_CFG p10001 = CommunicationChannel.new_UAVIONIX_ADSB_OUT_CFG();
            PH.setPack(p10001);
            p10001.gpsOffsetLon = (UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON)UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_APPLIED_BY_SENSOR;
            p10001.ICAO = (uint)2607399053U;
            p10001.gpsOffsetLat = (UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT)UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_6M;
            p10001.stallSpeed = (ushort)(ushort)7976;
            p10001.emitterType = (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UAV;
            p10001.aircraftSize = (UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE)UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE.UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L55_52M;
            p10001.rfSelect = (UAVIONIX_ADSB_OUT_RF_SELECT)UAVIONIX_ADSB_OUT_RF_SELECT.UAVIONIX_ADSB_OUT_RF_SELECT_RX_ENABLED;
            p10001.callsign_SET("bh", PH) ;
            CommunicationChannel.instance.send(p10001);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnUAVIONIX_ADSB_OUT_DYNAMICReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.accuracyVel == (ushort)(ushort)25691);
                Debug.Assert(pack.state == (UAVIONIX_ADSB_OUT_DYNAMIC_STATE)UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_ON_GROUND);
                Debug.Assert(pack.squawk == (ushort)(ushort)62478);
                Debug.Assert(pack.velNS == (short)(short)30536);
                Debug.Assert(pack.utcTime == (uint)1977106323U);
                Debug.Assert(pack.baroAltMSL == (int) -312517567);
                Debug.Assert(pack.numSats == (byte)(byte)46);
                Debug.Assert(pack.emergencyStatus == (UAVIONIX_ADSB_EMERGENCY_STATUS)UAVIONIX_ADSB_EMERGENCY_STATUS.UAVIONIX_ADSB_OUT_GENERAL_EMERGENCY);
                Debug.Assert(pack.accuracyVert == (ushort)(ushort)45436);
                Debug.Assert(pack.gpsLat == (int)243995405);
                Debug.Assert(pack.gpsFix == (UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX)UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX.UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_DGPS);
                Debug.Assert(pack.VelEW == (short)(short) -29049);
                Debug.Assert(pack.gpsLon == (int)1405106645);
                Debug.Assert(pack.accuracyHor == (uint)239219389U);
                Debug.Assert(pack.gpsAlt == (int) -1768957704);
                Debug.Assert(pack.velVert == (short)(short) -7045);
            };
            GroundControl.UAVIONIX_ADSB_OUT_DYNAMIC p10002 = CommunicationChannel.new_UAVIONIX_ADSB_OUT_DYNAMIC();
            PH.setPack(p10002);
            p10002.velVert = (short)(short) -7045;
            p10002.accuracyHor = (uint)239219389U;
            p10002.squawk = (ushort)(ushort)62478;
            p10002.numSats = (byte)(byte)46;
            p10002.gpsLat = (int)243995405;
            p10002.utcTime = (uint)1977106323U;
            p10002.accuracyVert = (ushort)(ushort)45436;
            p10002.gpsAlt = (int) -1768957704;
            p10002.gpsFix = (UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX)UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX.UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_DGPS;
            p10002.baroAltMSL = (int) -312517567;
            p10002.velNS = (short)(short)30536;
            p10002.VelEW = (short)(short) -29049;
            p10002.accuracyVel = (ushort)(ushort)25691;
            p10002.state = (UAVIONIX_ADSB_OUT_DYNAMIC_STATE)UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_ON_GROUND;
            p10002.emergencyStatus = (UAVIONIX_ADSB_EMERGENCY_STATUS)UAVIONIX_ADSB_EMERGENCY_STATUS.UAVIONIX_ADSB_OUT_GENERAL_EMERGENCY;
            p10002.gpsLon = (int)1405106645;
            CommunicationChannel.instance.send(p10002);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnUAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rfHealth == (UAVIONIX_ADSB_RF_HEALTH)UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_OK);
            };
            GroundControl.UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT p10003 = CommunicationChannel.new_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT();
            PH.setPack(p10003);
            p10003.rfHealth = (UAVIONIX_ADSB_RF_HEALTH)UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_OK;
            CommunicationChannel.instance.send(p10003);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDEVICE_OP_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.regstart == (byte)(byte)86);
                Debug.Assert(pack.count == (byte)(byte)59);
                Debug.Assert(pack.target_system == (byte)(byte)122);
                Debug.Assert(pack.address == (byte)(byte)49);
                Debug.Assert(pack.busname_LEN(ph) == 36);
                Debug.Assert(pack.busname_TRY(ph).Equals("TodljexfzlgjylapdCdomxznuverixiEtxqd"));
                Debug.Assert(pack.bus == (byte)(byte)109);
                Debug.Assert(pack.bustype == (DEVICE_OP_BUSTYPE)DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_SPI);
                Debug.Assert(pack.request_id == (uint)2406425819U);
                Debug.Assert(pack.target_component == (byte)(byte)113);
            };
            GroundControl.DEVICE_OP_READ p11000 = CommunicationChannel.new_DEVICE_OP_READ();
            PH.setPack(p11000);
            p11000.target_component = (byte)(byte)113;
            p11000.address = (byte)(byte)49;
            p11000.target_system = (byte)(byte)122;
            p11000.regstart = (byte)(byte)86;
            p11000.request_id = (uint)2406425819U;
            p11000.busname_SET("TodljexfzlgjylapdCdomxznuverixiEtxqd", PH) ;
            p11000.count = (byte)(byte)59;
            p11000.bus = (byte)(byte)109;
            p11000.bustype = (DEVICE_OP_BUSTYPE)DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_SPI;
            CommunicationChannel.instance.send(p11000);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDEVICE_OP_READ_REPLYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)155, (byte)217, (byte)251, (byte)83, (byte)252, (byte)122, (byte)148, (byte)238, (byte)154, (byte)118, (byte)47, (byte)116, (byte)19, (byte)43, (byte)253, (byte)158, (byte)238, (byte)49, (byte)135, (byte)63, (byte)68, (byte)157, (byte)204, (byte)112, (byte)166, (byte)139, (byte)96, (byte)92, (byte)227, (byte)58, (byte)70, (byte)124, (byte)89, (byte)245, (byte)53, (byte)121, (byte)106, (byte)82, (byte)137, (byte)67, (byte)124, (byte)214, (byte)92, (byte)231, (byte)44, (byte)148, (byte)72, (byte)13, (byte)112, (byte)173, (byte)17, (byte)45, (byte)71, (byte)5, (byte)2, (byte)95, (byte)41, (byte)118, (byte)27, (byte)103, (byte)15, (byte)190, (byte)217, (byte)4, (byte)217, (byte)238, (byte)37, (byte)176, (byte)84, (byte)3, (byte)192, (byte)37, (byte)107, (byte)144, (byte)87, (byte)128, (byte)160, (byte)194, (byte)234, (byte)100, (byte)184, (byte)236, (byte)154, (byte)108, (byte)117, (byte)36, (byte)209, (byte)93, (byte)17, (byte)156, (byte)59, (byte)85, (byte)46, (byte)77, (byte)219, (byte)36, (byte)129, (byte)40, (byte)72, (byte)55, (byte)190, (byte)241, (byte)121, (byte)80, (byte)107, (byte)90, (byte)122, (byte)139, (byte)74, (byte)92, (byte)203, (byte)244, (byte)173, (byte)129, (byte)208, (byte)111, (byte)167, (byte)116, (byte)61, (byte)35, (byte)79, (byte)17, (byte)187, (byte)72, (byte)234, (byte)90, (byte)203, (byte)244}));
                Debug.Assert(pack.regstart == (byte)(byte)63);
                Debug.Assert(pack.request_id == (uint)899937051U);
                Debug.Assert(pack.result == (byte)(byte)238);
                Debug.Assert(pack.count == (byte)(byte)178);
            };
            GroundControl.DEVICE_OP_READ_REPLY p11001 = CommunicationChannel.new_DEVICE_OP_READ_REPLY();
            PH.setPack(p11001);
            p11001.request_id = (uint)899937051U;
            p11001.result = (byte)(byte)238;
            p11001.count = (byte)(byte)178;
            p11001.regstart = (byte)(byte)63;
            p11001.data__SET(new byte[] {(byte)155, (byte)217, (byte)251, (byte)83, (byte)252, (byte)122, (byte)148, (byte)238, (byte)154, (byte)118, (byte)47, (byte)116, (byte)19, (byte)43, (byte)253, (byte)158, (byte)238, (byte)49, (byte)135, (byte)63, (byte)68, (byte)157, (byte)204, (byte)112, (byte)166, (byte)139, (byte)96, (byte)92, (byte)227, (byte)58, (byte)70, (byte)124, (byte)89, (byte)245, (byte)53, (byte)121, (byte)106, (byte)82, (byte)137, (byte)67, (byte)124, (byte)214, (byte)92, (byte)231, (byte)44, (byte)148, (byte)72, (byte)13, (byte)112, (byte)173, (byte)17, (byte)45, (byte)71, (byte)5, (byte)2, (byte)95, (byte)41, (byte)118, (byte)27, (byte)103, (byte)15, (byte)190, (byte)217, (byte)4, (byte)217, (byte)238, (byte)37, (byte)176, (byte)84, (byte)3, (byte)192, (byte)37, (byte)107, (byte)144, (byte)87, (byte)128, (byte)160, (byte)194, (byte)234, (byte)100, (byte)184, (byte)236, (byte)154, (byte)108, (byte)117, (byte)36, (byte)209, (byte)93, (byte)17, (byte)156, (byte)59, (byte)85, (byte)46, (byte)77, (byte)219, (byte)36, (byte)129, (byte)40, (byte)72, (byte)55, (byte)190, (byte)241, (byte)121, (byte)80, (byte)107, (byte)90, (byte)122, (byte)139, (byte)74, (byte)92, (byte)203, (byte)244, (byte)173, (byte)129, (byte)208, (byte)111, (byte)167, (byte)116, (byte)61, (byte)35, (byte)79, (byte)17, (byte)187, (byte)72, (byte)234, (byte)90, (byte)203, (byte)244}, 0) ;
            CommunicationChannel.instance.send(p11001);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDEVICE_OP_WRITEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)48, (byte)97, (byte)150, (byte)108, (byte)121, (byte)127, (byte)215, (byte)54, (byte)227, (byte)254, (byte)209, (byte)31, (byte)100, (byte)230, (byte)129, (byte)188, (byte)55, (byte)163, (byte)204, (byte)249, (byte)223, (byte)63, (byte)194, (byte)63, (byte)87, (byte)98, (byte)223, (byte)118, (byte)113, (byte)106, (byte)81, (byte)186, (byte)36, (byte)100, (byte)141, (byte)60, (byte)162, (byte)43, (byte)163, (byte)111, (byte)99, (byte)56, (byte)107, (byte)158, (byte)73, (byte)214, (byte)140, (byte)235, (byte)148, (byte)232, (byte)113, (byte)90, (byte)137, (byte)106, (byte)106, (byte)251, (byte)20, (byte)112, (byte)5, (byte)54, (byte)0, (byte)203, (byte)7, (byte)178, (byte)35, (byte)68, (byte)101, (byte)210, (byte)85, (byte)210, (byte)34, (byte)156, (byte)114, (byte)242, (byte)27, (byte)150, (byte)17, (byte)2, (byte)203, (byte)122, (byte)216, (byte)14, (byte)200, (byte)229, (byte)168, (byte)138, (byte)255, (byte)3, (byte)51, (byte)149, (byte)232, (byte)160, (byte)241, (byte)209, (byte)100, (byte)3, (byte)155, (byte)93, (byte)176, (byte)176, (byte)11, (byte)190, (byte)161, (byte)113, (byte)228, (byte)205, (byte)93, (byte)102, (byte)189, (byte)40, (byte)3, (byte)146, (byte)30, (byte)104, (byte)159, (byte)44, (byte)228, (byte)145, (byte)112, (byte)225, (byte)201, (byte)91, (byte)188, (byte)164, (byte)97, (byte)10, (byte)5, (byte)251}));
                Debug.Assert(pack.target_system == (byte)(byte)102);
                Debug.Assert(pack.request_id == (uint)2868890621U);
                Debug.Assert(pack.bustype == (DEVICE_OP_BUSTYPE)DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_SPI);
                Debug.Assert(pack.busname_LEN(ph) == 8);
                Debug.Assert(pack.busname_TRY(ph).Equals("sndwcnvs"));
                Debug.Assert(pack.bus == (byte)(byte)45);
                Debug.Assert(pack.address == (byte)(byte)159);
                Debug.Assert(pack.regstart == (byte)(byte)0);
                Debug.Assert(pack.count == (byte)(byte)123);
                Debug.Assert(pack.target_component == (byte)(byte)86);
            };
            GroundControl.DEVICE_OP_WRITE p11002 = CommunicationChannel.new_DEVICE_OP_WRITE();
            PH.setPack(p11002);
            p11002.regstart = (byte)(byte)0;
            p11002.address = (byte)(byte)159;
            p11002.busname_SET("sndwcnvs", PH) ;
            p11002.bustype = (DEVICE_OP_BUSTYPE)DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_SPI;
            p11002.count = (byte)(byte)123;
            p11002.bus = (byte)(byte)45;
            p11002.target_component = (byte)(byte)86;
            p11002.request_id = (uint)2868890621U;
            p11002.target_system = (byte)(byte)102;
            p11002.data__SET(new byte[] {(byte)48, (byte)97, (byte)150, (byte)108, (byte)121, (byte)127, (byte)215, (byte)54, (byte)227, (byte)254, (byte)209, (byte)31, (byte)100, (byte)230, (byte)129, (byte)188, (byte)55, (byte)163, (byte)204, (byte)249, (byte)223, (byte)63, (byte)194, (byte)63, (byte)87, (byte)98, (byte)223, (byte)118, (byte)113, (byte)106, (byte)81, (byte)186, (byte)36, (byte)100, (byte)141, (byte)60, (byte)162, (byte)43, (byte)163, (byte)111, (byte)99, (byte)56, (byte)107, (byte)158, (byte)73, (byte)214, (byte)140, (byte)235, (byte)148, (byte)232, (byte)113, (byte)90, (byte)137, (byte)106, (byte)106, (byte)251, (byte)20, (byte)112, (byte)5, (byte)54, (byte)0, (byte)203, (byte)7, (byte)178, (byte)35, (byte)68, (byte)101, (byte)210, (byte)85, (byte)210, (byte)34, (byte)156, (byte)114, (byte)242, (byte)27, (byte)150, (byte)17, (byte)2, (byte)203, (byte)122, (byte)216, (byte)14, (byte)200, (byte)229, (byte)168, (byte)138, (byte)255, (byte)3, (byte)51, (byte)149, (byte)232, (byte)160, (byte)241, (byte)209, (byte)100, (byte)3, (byte)155, (byte)93, (byte)176, (byte)176, (byte)11, (byte)190, (byte)161, (byte)113, (byte)228, (byte)205, (byte)93, (byte)102, (byte)189, (byte)40, (byte)3, (byte)146, (byte)30, (byte)104, (byte)159, (byte)44, (byte)228, (byte)145, (byte)112, (byte)225, (byte)201, (byte)91, (byte)188, (byte)164, (byte)97, (byte)10, (byte)5, (byte)251}, 0) ;
            CommunicationChannel.instance.send(p11002);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDEVICE_OP_WRITE_REPLYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.result == (byte)(byte)189);
                Debug.Assert(pack.request_id == (uint)3458003808U);
            };
            GroundControl.DEVICE_OP_WRITE_REPLY p11003 = CommunicationChannel.new_DEVICE_OP_WRITE_REPLY();
            PH.setPack(p11003);
            p11003.result = (byte)(byte)189;
            p11003.request_id = (uint)3458003808U;
            CommunicationChannel.instance.send(p11003);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnADAP_TUNINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.f_dot == (float) -2.8642799E38F);
                Debug.Assert(pack.omega == (float) -9.723476E37F);
                Debug.Assert(pack.omega_dot == (float) -9.45659E37F);
                Debug.Assert(pack.u == (float)2.2374816E38F);
                Debug.Assert(pack.theta == (float)8.687174E36F);
                Debug.Assert(pack.sigma_dot == (float)1.4542099E38F);
                Debug.Assert(pack.axis == (PID_TUNING_AXIS)PID_TUNING_AXIS.PID_TUNING_PITCH);
                Debug.Assert(pack.theta_dot == (float) -2.190732E38F);
                Debug.Assert(pack.error == (float)1.4649537E38F);
                Debug.Assert(pack.desired == (float)1.8285986E38F);
                Debug.Assert(pack.f == (float) -1.708888E38F);
                Debug.Assert(pack.achieved == (float)2.6860741E38F);
                Debug.Assert(pack.sigma == (float)3.3124293E37F);
            };
            GroundControl.ADAP_TUNING p11010 = CommunicationChannel.new_ADAP_TUNING();
            PH.setPack(p11010);
            p11010.achieved = (float)2.6860741E38F;
            p11010.theta = (float)8.687174E36F;
            p11010.f_dot = (float) -2.8642799E38F;
            p11010.theta_dot = (float) -2.190732E38F;
            p11010.u = (float)2.2374816E38F;
            p11010.error = (float)1.4649537E38F;
            p11010.axis = (PID_TUNING_AXIS)PID_TUNING_AXIS.PID_TUNING_PITCH;
            p11010.desired = (float)1.8285986E38F;
            p11010.omega = (float) -9.723476E37F;
            p11010.f = (float) -1.708888E38F;
            p11010.omega_dot = (float) -9.45659E37F;
            p11010.sigma = (float)3.3124293E37F;
            p11010.sigma_dot = (float)1.4542099E38F;
            CommunicationChannel.instance.send(p11010);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnVISION_POSITION_DELTAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.confidence == (float)3.0204027E38F);
                Debug.Assert(pack.time_usec == (ulong)2859670459171698570L);
                Debug.Assert(pack.time_delta_usec == (ulong)7141987721865247757L);
                Debug.Assert(pack.position_delta.SequenceEqual(new float[] {1.9297372E38F, -2.2737256E37F, 2.2292177E38F}));
                Debug.Assert(pack.angle_delta.SequenceEqual(new float[] {-2.3497324E38F, 3.2139174E38F, 7.0278955E37F}));
            };
            GroundControl.VISION_POSITION_DELTA p11011 = CommunicationChannel.new_VISION_POSITION_DELTA();
            PH.setPack(p11011);
            p11011.time_delta_usec = (ulong)7141987721865247757L;
            p11011.time_usec = (ulong)2859670459171698570L;
            p11011.position_delta_SET(new float[] {1.9297372E38F, -2.2737256E37F, 2.2292177E38F}, 0) ;
            p11011.angle_delta_SET(new float[] {-2.3497324E38F, 3.2139174E38F, 7.0278955E37F}, 0) ;
            p11011.confidence = (float)3.0204027E38F;
            CommunicationChannel.instance.send(p11011);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
        }
    }
}
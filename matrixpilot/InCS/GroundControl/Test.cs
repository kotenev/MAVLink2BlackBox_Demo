
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
                        case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE_ADVANCED:
                            id = 0;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_WAYPOINT:
                            id = 1;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_LOITER_UNLIM:
                            id = 2;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_LOITER_TURNS:
                            id = 3;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_LOITER_TIME:
                            id = 4;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH:
                            id = 5;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_LAND:
                            id = 6;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_TAKEOFF:
                            id = 7;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_LAND_LOCAL:
                            id = 8;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_TAKEOFF_LOCAL:
                            id = 9;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FOLLOW:
                            id = 10;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
                            id = 11;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_LOITER_TO_ALT:
                            id = 12;
                            break;
                        case MAV_CMD.MAV_CMD_DO_FOLLOW:
                            id = 13;
                            break;
                        case MAV_CMD.MAV_CMD_DO_FOLLOW_REPOSITION:
                            id = 14;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_ROI:
                            id = 15;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_PATHPLANNING:
                            id = 16;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_SPLINE_WAYPOINT:
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
                        case MAV_CMD.MAV_CMD_NAV_SET_YAW_SPEED:
                            id = 57;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
                            id = 58;
                            break;
                        case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL_QUAT:
                            id = 59;
                            break;
                        case MAV_CMD.MAV_CMD_DO_GUIDED_MASTER:
                            id = 60;
                            break;
                        case MAV_CMD.MAV_CMD_DO_GUIDED_LIMITS:
                            id = 61;
                            break;
                        case MAV_CMD.MAV_CMD_DO_ENGINE_CONTROL:
                            id = 62;
                            break;
                        case MAV_CMD.MAV_CMD_DO_LAST:
                            id = 63;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION:
                            id = 64;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
                            id = 65;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_UAVCAN:
                            id = 66;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE:
                            id = 67;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
                            id = 68;
                            break;
                        case MAV_CMD.MAV_CMD_OVERRIDE_GOTO:
                            id = 69;
                            break;
                        case MAV_CMD.MAV_CMD_MISSION_START:
                            id = 70;
                            break;
                        case MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM:
                            id = 71;
                            break;
                        case MAV_CMD.MAV_CMD_GET_HOME_POSITION:
                            id = 72;
                            break;
                        case MAV_CMD.MAV_CMD_START_RX_PAIR:
                            id = 73;
                            break;
                        case MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL:
                            id = 74;
                            break;
                        case MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL:
                            id = 75;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION:
                            id = 76;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
                            id = 77;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_CAMERA_INFORMATION:
                            id = 78;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS:
                            id = 79;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION:
                            id = 80;
                            break;
                        case MAV_CMD.MAV_CMD_STORAGE_FORMAT:
                            id = 81;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
                            id = 82;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_FLIGHT_INFORMATION:
                            id = 83;
                            break;
                        case MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS:
                            id = 84;
                            break;
                        case MAV_CMD.MAV_CMD_SET_CAMERA_MODE:
                            id = 85;
                            break;
                        case MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE:
                            id = 86;
                            break;
                        case MAV_CMD.MAV_CMD_IMAGE_STOP_CAPTURE:
                            id = 87;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE:
                            id = 88;
                            break;
                        case MAV_CMD.MAV_CMD_DO_TRIGGER_CONTROL:
                            id = 89;
                            break;
                        case MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE:
                            id = 90;
                            break;
                        case MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE:
                            id = 91;
                            break;
                        case MAV_CMD.MAV_CMD_VIDEO_START_STREAMING:
                            id = 92;
                            break;
                        case MAV_CMD.MAV_CMD_VIDEO_STOP_STREAMING:
                            id = 93;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
                            id = 94;
                            break;
                        case MAV_CMD.MAV_CMD_LOGGING_START:
                            id = 95;
                            break;
                        case MAV_CMD.MAV_CMD_LOGGING_STOP:
                            id = 96;
                            break;
                        case MAV_CMD.MAV_CMD_AIRFRAME_CONFIGURATION:
                            id = 97;
                            break;
                        case MAV_CMD.MAV_CMD_PANORAMA_CREATE:
                            id = 98;
                            break;
                        case MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION:
                            id = 99;
                            break;
                        case MAV_CMD.MAV_CMD_ARM_AUTHORIZATION_REQUEST:
                            id = 100;
                            break;
                        case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_STANDARD:
                            id = 101;
                            break;
                        case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE:
                            id = 102;
                            break;
                        case MAV_CMD.MAV_CMD_CONDITION_GATE:
                            id = 103;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_RETURN_POINT:
                            id = 104;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
                            id = 105;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
                            id = 106;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
                            id = 107;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
                            id = 108;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_RALLY_POINT:
                            id = 109;
                            break;
                        case MAV_CMD.MAV_CMD_UAVCAN_GET_NODE_INFO:
                            id = 110;
                            break;
                        case MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
                            id = 111;
                            break;
                        case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
                            id = 112;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_1:
                            id = 113;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_2:
                            id = 114;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_3:
                            id = 115;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_4:
                            id = 116;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_5:
                            id = 117;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_1:
                            id = 118;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_2:
                            id = 119;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_3:
                            id = 120;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_4:
                            id = 121;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_5:
                            id = 122;
                            break;
                        case MAV_CMD.MAV_CMD_USER_1:
                            id = 123;
                            break;
                        case MAV_CMD.MAV_CMD_USER_2:
                            id = 124;
                            break;
                        case MAV_CMD.MAV_CMD_USER_3:
                            id = 125;
                            break;
                        case MAV_CMD.MAV_CMD_USER_4:
                            id = 126;
                            break;
                        case MAV_CMD.MAV_CMD_USER_5:
                            id = 127;
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
                        case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE_ADVANCED:
                            id = 0;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_WAYPOINT:
                            id = 1;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_LOITER_UNLIM:
                            id = 2;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_LOITER_TURNS:
                            id = 3;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_LOITER_TIME:
                            id = 4;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH:
                            id = 5;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_LAND:
                            id = 6;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_TAKEOFF:
                            id = 7;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_LAND_LOCAL:
                            id = 8;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_TAKEOFF_LOCAL:
                            id = 9;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FOLLOW:
                            id = 10;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
                            id = 11;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_LOITER_TO_ALT:
                            id = 12;
                            break;
                        case MAV_CMD.MAV_CMD_DO_FOLLOW:
                            id = 13;
                            break;
                        case MAV_CMD.MAV_CMD_DO_FOLLOW_REPOSITION:
                            id = 14;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_ROI:
                            id = 15;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_PATHPLANNING:
                            id = 16;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_SPLINE_WAYPOINT:
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
                        case MAV_CMD.MAV_CMD_NAV_SET_YAW_SPEED:
                            id = 57;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
                            id = 58;
                            break;
                        case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL_QUAT:
                            id = 59;
                            break;
                        case MAV_CMD.MAV_CMD_DO_GUIDED_MASTER:
                            id = 60;
                            break;
                        case MAV_CMD.MAV_CMD_DO_GUIDED_LIMITS:
                            id = 61;
                            break;
                        case MAV_CMD.MAV_CMD_DO_ENGINE_CONTROL:
                            id = 62;
                            break;
                        case MAV_CMD.MAV_CMD_DO_LAST:
                            id = 63;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION:
                            id = 64;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
                            id = 65;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_UAVCAN:
                            id = 66;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE:
                            id = 67;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
                            id = 68;
                            break;
                        case MAV_CMD.MAV_CMD_OVERRIDE_GOTO:
                            id = 69;
                            break;
                        case MAV_CMD.MAV_CMD_MISSION_START:
                            id = 70;
                            break;
                        case MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM:
                            id = 71;
                            break;
                        case MAV_CMD.MAV_CMD_GET_HOME_POSITION:
                            id = 72;
                            break;
                        case MAV_CMD.MAV_CMD_START_RX_PAIR:
                            id = 73;
                            break;
                        case MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL:
                            id = 74;
                            break;
                        case MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL:
                            id = 75;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION:
                            id = 76;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
                            id = 77;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_CAMERA_INFORMATION:
                            id = 78;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS:
                            id = 79;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION:
                            id = 80;
                            break;
                        case MAV_CMD.MAV_CMD_STORAGE_FORMAT:
                            id = 81;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
                            id = 82;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_FLIGHT_INFORMATION:
                            id = 83;
                            break;
                        case MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS:
                            id = 84;
                            break;
                        case MAV_CMD.MAV_CMD_SET_CAMERA_MODE:
                            id = 85;
                            break;
                        case MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE:
                            id = 86;
                            break;
                        case MAV_CMD.MAV_CMD_IMAGE_STOP_CAPTURE:
                            id = 87;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE:
                            id = 88;
                            break;
                        case MAV_CMD.MAV_CMD_DO_TRIGGER_CONTROL:
                            id = 89;
                            break;
                        case MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE:
                            id = 90;
                            break;
                        case MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE:
                            id = 91;
                            break;
                        case MAV_CMD.MAV_CMD_VIDEO_START_STREAMING:
                            id = 92;
                            break;
                        case MAV_CMD.MAV_CMD_VIDEO_STOP_STREAMING:
                            id = 93;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
                            id = 94;
                            break;
                        case MAV_CMD.MAV_CMD_LOGGING_START:
                            id = 95;
                            break;
                        case MAV_CMD.MAV_CMD_LOGGING_STOP:
                            id = 96;
                            break;
                        case MAV_CMD.MAV_CMD_AIRFRAME_CONFIGURATION:
                            id = 97;
                            break;
                        case MAV_CMD.MAV_CMD_PANORAMA_CREATE:
                            id = 98;
                            break;
                        case MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION:
                            id = 99;
                            break;
                        case MAV_CMD.MAV_CMD_ARM_AUTHORIZATION_REQUEST:
                            id = 100;
                            break;
                        case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_STANDARD:
                            id = 101;
                            break;
                        case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE:
                            id = 102;
                            break;
                        case MAV_CMD.MAV_CMD_CONDITION_GATE:
                            id = 103;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_RETURN_POINT:
                            id = 104;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
                            id = 105;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
                            id = 106;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
                            id = 107;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
                            id = 108;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_RALLY_POINT:
                            id = 109;
                            break;
                        case MAV_CMD.MAV_CMD_UAVCAN_GET_NODE_INFO:
                            id = 110;
                            break;
                        case MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
                            id = 111;
                            break;
                        case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
                            id = 112;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_1:
                            id = 113;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_2:
                            id = 114;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_3:
                            id = 115;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_4:
                            id = 116;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_5:
                            id = 117;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_1:
                            id = 118;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_2:
                            id = 119;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_3:
                            id = 120;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_4:
                            id = 121;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_5:
                            id = 122;
                            break;
                        case MAV_CMD.MAV_CMD_USER_1:
                            id = 123;
                            break;
                        case MAV_CMD.MAV_CMD_USER_2:
                            id = 124;
                            break;
                        case MAV_CMD.MAV_CMD_USER_3:
                            id = 125;
                            break;
                        case MAV_CMD.MAV_CMD_USER_4:
                            id = 126;
                            break;
                        case MAV_CMD.MAV_CMD_USER_5:
                            id = 127;
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
                        case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE_ADVANCED:
                            id = 0;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_WAYPOINT:
                            id = 1;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_LOITER_UNLIM:
                            id = 2;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_LOITER_TURNS:
                            id = 3;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_LOITER_TIME:
                            id = 4;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH:
                            id = 5;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_LAND:
                            id = 6;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_TAKEOFF:
                            id = 7;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_LAND_LOCAL:
                            id = 8;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_TAKEOFF_LOCAL:
                            id = 9;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FOLLOW:
                            id = 10;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
                            id = 11;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_LOITER_TO_ALT:
                            id = 12;
                            break;
                        case MAV_CMD.MAV_CMD_DO_FOLLOW:
                            id = 13;
                            break;
                        case MAV_CMD.MAV_CMD_DO_FOLLOW_REPOSITION:
                            id = 14;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_ROI:
                            id = 15;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_PATHPLANNING:
                            id = 16;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_SPLINE_WAYPOINT:
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
                        case MAV_CMD.MAV_CMD_NAV_SET_YAW_SPEED:
                            id = 57;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
                            id = 58;
                            break;
                        case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL_QUAT:
                            id = 59;
                            break;
                        case MAV_CMD.MAV_CMD_DO_GUIDED_MASTER:
                            id = 60;
                            break;
                        case MAV_CMD.MAV_CMD_DO_GUIDED_LIMITS:
                            id = 61;
                            break;
                        case MAV_CMD.MAV_CMD_DO_ENGINE_CONTROL:
                            id = 62;
                            break;
                        case MAV_CMD.MAV_CMD_DO_LAST:
                            id = 63;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION:
                            id = 64;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
                            id = 65;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_UAVCAN:
                            id = 66;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE:
                            id = 67;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
                            id = 68;
                            break;
                        case MAV_CMD.MAV_CMD_OVERRIDE_GOTO:
                            id = 69;
                            break;
                        case MAV_CMD.MAV_CMD_MISSION_START:
                            id = 70;
                            break;
                        case MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM:
                            id = 71;
                            break;
                        case MAV_CMD.MAV_CMD_GET_HOME_POSITION:
                            id = 72;
                            break;
                        case MAV_CMD.MAV_CMD_START_RX_PAIR:
                            id = 73;
                            break;
                        case MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL:
                            id = 74;
                            break;
                        case MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL:
                            id = 75;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION:
                            id = 76;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
                            id = 77;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_CAMERA_INFORMATION:
                            id = 78;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS:
                            id = 79;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION:
                            id = 80;
                            break;
                        case MAV_CMD.MAV_CMD_STORAGE_FORMAT:
                            id = 81;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
                            id = 82;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_FLIGHT_INFORMATION:
                            id = 83;
                            break;
                        case MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS:
                            id = 84;
                            break;
                        case MAV_CMD.MAV_CMD_SET_CAMERA_MODE:
                            id = 85;
                            break;
                        case MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE:
                            id = 86;
                            break;
                        case MAV_CMD.MAV_CMD_IMAGE_STOP_CAPTURE:
                            id = 87;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE:
                            id = 88;
                            break;
                        case MAV_CMD.MAV_CMD_DO_TRIGGER_CONTROL:
                            id = 89;
                            break;
                        case MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE:
                            id = 90;
                            break;
                        case MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE:
                            id = 91;
                            break;
                        case MAV_CMD.MAV_CMD_VIDEO_START_STREAMING:
                            id = 92;
                            break;
                        case MAV_CMD.MAV_CMD_VIDEO_STOP_STREAMING:
                            id = 93;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
                            id = 94;
                            break;
                        case MAV_CMD.MAV_CMD_LOGGING_START:
                            id = 95;
                            break;
                        case MAV_CMD.MAV_CMD_LOGGING_STOP:
                            id = 96;
                            break;
                        case MAV_CMD.MAV_CMD_AIRFRAME_CONFIGURATION:
                            id = 97;
                            break;
                        case MAV_CMD.MAV_CMD_PANORAMA_CREATE:
                            id = 98;
                            break;
                        case MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION:
                            id = 99;
                            break;
                        case MAV_CMD.MAV_CMD_ARM_AUTHORIZATION_REQUEST:
                            id = 100;
                            break;
                        case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_STANDARD:
                            id = 101;
                            break;
                        case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE:
                            id = 102;
                            break;
                        case MAV_CMD.MAV_CMD_CONDITION_GATE:
                            id = 103;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_RETURN_POINT:
                            id = 104;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
                            id = 105;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
                            id = 106;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
                            id = 107;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
                            id = 108;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_RALLY_POINT:
                            id = 109;
                            break;
                        case MAV_CMD.MAV_CMD_UAVCAN_GET_NODE_INFO:
                            id = 110;
                            break;
                        case MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
                            id = 111;
                            break;
                        case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
                            id = 112;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_1:
                            id = 113;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_2:
                            id = 114;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_3:
                            id = 115;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_4:
                            id = 116;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_5:
                            id = 117;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_1:
                            id = 118;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_2:
                            id = 119;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_3:
                            id = 120;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_4:
                            id = 121;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_5:
                            id = 122;
                            break;
                        case MAV_CMD.MAV_CMD_USER_1:
                            id = 123;
                            break;
                        case MAV_CMD.MAV_CMD_USER_2:
                            id = 124;
                            break;
                        case MAV_CMD.MAV_CMD_USER_3:
                            id = 125;
                            break;
                        case MAV_CMD.MAV_CMD_USER_4:
                            id = 126;
                            break;
                        case MAV_CMD.MAV_CMD_USER_5:
                            id = 127;
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
                        case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE_ADVANCED:
                            id = 0;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_WAYPOINT:
                            id = 1;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_LOITER_UNLIM:
                            id = 2;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_LOITER_TURNS:
                            id = 3;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_LOITER_TIME:
                            id = 4;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH:
                            id = 5;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_LAND:
                            id = 6;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_TAKEOFF:
                            id = 7;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_LAND_LOCAL:
                            id = 8;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_TAKEOFF_LOCAL:
                            id = 9;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FOLLOW:
                            id = 10;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
                            id = 11;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_LOITER_TO_ALT:
                            id = 12;
                            break;
                        case MAV_CMD.MAV_CMD_DO_FOLLOW:
                            id = 13;
                            break;
                        case MAV_CMD.MAV_CMD_DO_FOLLOW_REPOSITION:
                            id = 14;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_ROI:
                            id = 15;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_PATHPLANNING:
                            id = 16;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_SPLINE_WAYPOINT:
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
                        case MAV_CMD.MAV_CMD_NAV_SET_YAW_SPEED:
                            id = 57;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
                            id = 58;
                            break;
                        case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL_QUAT:
                            id = 59;
                            break;
                        case MAV_CMD.MAV_CMD_DO_GUIDED_MASTER:
                            id = 60;
                            break;
                        case MAV_CMD.MAV_CMD_DO_GUIDED_LIMITS:
                            id = 61;
                            break;
                        case MAV_CMD.MAV_CMD_DO_ENGINE_CONTROL:
                            id = 62;
                            break;
                        case MAV_CMD.MAV_CMD_DO_LAST:
                            id = 63;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION:
                            id = 64;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
                            id = 65;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_UAVCAN:
                            id = 66;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE:
                            id = 67;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
                            id = 68;
                            break;
                        case MAV_CMD.MAV_CMD_OVERRIDE_GOTO:
                            id = 69;
                            break;
                        case MAV_CMD.MAV_CMD_MISSION_START:
                            id = 70;
                            break;
                        case MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM:
                            id = 71;
                            break;
                        case MAV_CMD.MAV_CMD_GET_HOME_POSITION:
                            id = 72;
                            break;
                        case MAV_CMD.MAV_CMD_START_RX_PAIR:
                            id = 73;
                            break;
                        case MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL:
                            id = 74;
                            break;
                        case MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL:
                            id = 75;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION:
                            id = 76;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
                            id = 77;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_CAMERA_INFORMATION:
                            id = 78;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS:
                            id = 79;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION:
                            id = 80;
                            break;
                        case MAV_CMD.MAV_CMD_STORAGE_FORMAT:
                            id = 81;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
                            id = 82;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_FLIGHT_INFORMATION:
                            id = 83;
                            break;
                        case MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS:
                            id = 84;
                            break;
                        case MAV_CMD.MAV_CMD_SET_CAMERA_MODE:
                            id = 85;
                            break;
                        case MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE:
                            id = 86;
                            break;
                        case MAV_CMD.MAV_CMD_IMAGE_STOP_CAPTURE:
                            id = 87;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE:
                            id = 88;
                            break;
                        case MAV_CMD.MAV_CMD_DO_TRIGGER_CONTROL:
                            id = 89;
                            break;
                        case MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE:
                            id = 90;
                            break;
                        case MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE:
                            id = 91;
                            break;
                        case MAV_CMD.MAV_CMD_VIDEO_START_STREAMING:
                            id = 92;
                            break;
                        case MAV_CMD.MAV_CMD_VIDEO_STOP_STREAMING:
                            id = 93;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
                            id = 94;
                            break;
                        case MAV_CMD.MAV_CMD_LOGGING_START:
                            id = 95;
                            break;
                        case MAV_CMD.MAV_CMD_LOGGING_STOP:
                            id = 96;
                            break;
                        case MAV_CMD.MAV_CMD_AIRFRAME_CONFIGURATION:
                            id = 97;
                            break;
                        case MAV_CMD.MAV_CMD_PANORAMA_CREATE:
                            id = 98;
                            break;
                        case MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION:
                            id = 99;
                            break;
                        case MAV_CMD.MAV_CMD_ARM_AUTHORIZATION_REQUEST:
                            id = 100;
                            break;
                        case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_STANDARD:
                            id = 101;
                            break;
                        case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE:
                            id = 102;
                            break;
                        case MAV_CMD.MAV_CMD_CONDITION_GATE:
                            id = 103;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_RETURN_POINT:
                            id = 104;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
                            id = 105;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
                            id = 106;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
                            id = 107;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
                            id = 108;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_RALLY_POINT:
                            id = 109;
                            break;
                        case MAV_CMD.MAV_CMD_UAVCAN_GET_NODE_INFO:
                            id = 110;
                            break;
                        case MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
                            id = 111;
                            break;
                        case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
                            id = 112;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_1:
                            id = 113;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_2:
                            id = 114;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_3:
                            id = 115;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_4:
                            id = 116;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_5:
                            id = 117;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_1:
                            id = 118;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_2:
                            id = 119;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_3:
                            id = 120;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_4:
                            id = 121;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_5:
                            id = 122;
                            break;
                        case MAV_CMD.MAV_CMD_USER_1:
                            id = 123;
                            break;
                        case MAV_CMD.MAV_CMD_USER_2:
                            id = 124;
                            break;
                        case MAV_CMD.MAV_CMD_USER_3:
                            id = 125;
                            break;
                        case MAV_CMD.MAV_CMD_USER_4:
                            id = 126;
                            break;
                        case MAV_CMD.MAV_CMD_USER_5:
                            id = 127;
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
                        case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE_ADVANCED:
                            id = 0;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_WAYPOINT:
                            id = 1;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_LOITER_UNLIM:
                            id = 2;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_LOITER_TURNS:
                            id = 3;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_LOITER_TIME:
                            id = 4;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH:
                            id = 5;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_LAND:
                            id = 6;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_TAKEOFF:
                            id = 7;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_LAND_LOCAL:
                            id = 8;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_TAKEOFF_LOCAL:
                            id = 9;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FOLLOW:
                            id = 10;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
                            id = 11;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_LOITER_TO_ALT:
                            id = 12;
                            break;
                        case MAV_CMD.MAV_CMD_DO_FOLLOW:
                            id = 13;
                            break;
                        case MAV_CMD.MAV_CMD_DO_FOLLOW_REPOSITION:
                            id = 14;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_ROI:
                            id = 15;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_PATHPLANNING:
                            id = 16;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_SPLINE_WAYPOINT:
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
                        case MAV_CMD.MAV_CMD_NAV_SET_YAW_SPEED:
                            id = 57;
                            break;
                        case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
                            id = 58;
                            break;
                        case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL_QUAT:
                            id = 59;
                            break;
                        case MAV_CMD.MAV_CMD_DO_GUIDED_MASTER:
                            id = 60;
                            break;
                        case MAV_CMD.MAV_CMD_DO_GUIDED_LIMITS:
                            id = 61;
                            break;
                        case MAV_CMD.MAV_CMD_DO_ENGINE_CONTROL:
                            id = 62;
                            break;
                        case MAV_CMD.MAV_CMD_DO_LAST:
                            id = 63;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION:
                            id = 64;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
                            id = 65;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_UAVCAN:
                            id = 66;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE:
                            id = 67;
                            break;
                        case MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
                            id = 68;
                            break;
                        case MAV_CMD.MAV_CMD_OVERRIDE_GOTO:
                            id = 69;
                            break;
                        case MAV_CMD.MAV_CMD_MISSION_START:
                            id = 70;
                            break;
                        case MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM:
                            id = 71;
                            break;
                        case MAV_CMD.MAV_CMD_GET_HOME_POSITION:
                            id = 72;
                            break;
                        case MAV_CMD.MAV_CMD_START_RX_PAIR:
                            id = 73;
                            break;
                        case MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL:
                            id = 74;
                            break;
                        case MAV_CMD.MAV_CMD_SET_MESSAGE_INTERVAL:
                            id = 75;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_PROTOCOL_VERSION:
                            id = 76;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
                            id = 77;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_CAMERA_INFORMATION:
                            id = 78;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS:
                            id = 79;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION:
                            id = 80;
                            break;
                        case MAV_CMD.MAV_CMD_STORAGE_FORMAT:
                            id = 81;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
                            id = 82;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_FLIGHT_INFORMATION:
                            id = 83;
                            break;
                        case MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS:
                            id = 84;
                            break;
                        case MAV_CMD.MAV_CMD_SET_CAMERA_MODE:
                            id = 85;
                            break;
                        case MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE:
                            id = 86;
                            break;
                        case MAV_CMD.MAV_CMD_IMAGE_STOP_CAPTURE:
                            id = 87;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE:
                            id = 88;
                            break;
                        case MAV_CMD.MAV_CMD_DO_TRIGGER_CONTROL:
                            id = 89;
                            break;
                        case MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE:
                            id = 90;
                            break;
                        case MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE:
                            id = 91;
                            break;
                        case MAV_CMD.MAV_CMD_VIDEO_START_STREAMING:
                            id = 92;
                            break;
                        case MAV_CMD.MAV_CMD_VIDEO_STOP_STREAMING:
                            id = 93;
                            break;
                        case MAV_CMD.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
                            id = 94;
                            break;
                        case MAV_CMD.MAV_CMD_LOGGING_START:
                            id = 95;
                            break;
                        case MAV_CMD.MAV_CMD_LOGGING_STOP:
                            id = 96;
                            break;
                        case MAV_CMD.MAV_CMD_AIRFRAME_CONFIGURATION:
                            id = 97;
                            break;
                        case MAV_CMD.MAV_CMD_PANORAMA_CREATE:
                            id = 98;
                            break;
                        case MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION:
                            id = 99;
                            break;
                        case MAV_CMD.MAV_CMD_ARM_AUTHORIZATION_REQUEST:
                            id = 100;
                            break;
                        case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_STANDARD:
                            id = 101;
                            break;
                        case MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE:
                            id = 102;
                            break;
                        case MAV_CMD.MAV_CMD_CONDITION_GATE:
                            id = 103;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_RETURN_POINT:
                            id = 104;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
                            id = 105;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
                            id = 106;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
                            id = 107;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
                            id = 108;
                            break;
                        case MAV_CMD.MAV_CMD_NAV_RALLY_POINT:
                            id = 109;
                            break;
                        case MAV_CMD.MAV_CMD_UAVCAN_GET_NODE_INFO:
                            id = 110;
                            break;
                        case MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
                            id = 111;
                            break;
                        case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
                            id = 112;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_1:
                            id = 113;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_2:
                            id = 114;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_3:
                            id = 115;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_4:
                            id = 116;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_5:
                            id = 117;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_1:
                            id = 118;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_2:
                            id = 119;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_3:
                            id = 120;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_4:
                            id = 121;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_5:
                            id = 122;
                            break;
                        case MAV_CMD.MAV_CMD_USER_1:
                            id = 123;
                            break;
                        case MAV_CMD.MAV_CMD_USER_2:
                            id = 124;
                            break;
                        case MAV_CMD.MAV_CMD_USER_3:
                            id = 125;
                            break;
                        case MAV_CMD.MAV_CMD_USER_4:
                            id = 126;
                            break;
                        case MAV_CMD.MAV_CMD_USER_5:
                            id = 127;
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
        new class FLEXIFUNCTION_SET : GroundControl.FLEXIFUNCTION_SET
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
        new class FLEXIFUNCTION_READ_REQ : GroundControl.FLEXIFUNCTION_READ_REQ
        {
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public short read_req_type //Type of flexifunction data requested
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  2, 2));}
            }

            public short data_index //index into data where needed
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  4, 2));}
            }
        }
        new class FLEXIFUNCTION_BUFFER_FUNCTION : GroundControl.FLEXIFUNCTION_BUFFER_FUNCTION
        {
            public ushort func_index //Function index
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ushort func_count //Total count of functions
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
            }

            public ushort data_address //Address in the flexifunction data, Set to 0xFFFF to use address in target memory
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
            }

            public ushort data_size //Size of the
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  6, 2));}
            }

            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  8, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  9, 1));}
            }

            public sbyte[] data_ //Settings data
            {
                get {return data__GET(new sbyte[48], 0);}
            }
            public sbyte[]data__GET(sbyte[] dst_ch, int pos)  //Settings data
            {
                for(int BYTE = 10, dst_max = pos + 48; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (sbyte)((sbyte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
        }
        new class FLEXIFUNCTION_BUFFER_FUNCTION_ACK : GroundControl.FLEXIFUNCTION_BUFFER_FUNCTION_ACK
        {
            public ushort func_index //Function index
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ushort result //result of acknowledge, 0=fail, 1=good
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
            }

            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
            }
        }
        new class FLEXIFUNCTION_DIRECTORY : GroundControl.FLEXIFUNCTION_DIRECTORY
        {
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public byte directory_type //0=inputs, 1=outputs
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }

            public byte start_index //index of first directory entry to write
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
            }

            public byte count //count of directory entries to write
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            public sbyte[] directory_data //Settings data
            {
                get {return directory_data_GET(new sbyte[48], 0);}
            }
            public sbyte[]directory_data_GET(sbyte[] dst_ch, int pos)  //Settings data
            {
                for(int BYTE = 5, dst_max = pos + 48; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (sbyte)((sbyte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
        }
        new class FLEXIFUNCTION_DIRECTORY_ACK : GroundControl.FLEXIFUNCTION_DIRECTORY_ACK
        {
            public ushort result //result of acknowledge, 0=fail, 1=good
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

            public byte directory_type //0=inputs, 1=outputs
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            public byte start_index //index of first directory entry to write
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
            }

            public byte count //count of directory entries to write
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  6, 1));}
            }
        }
        new class FLEXIFUNCTION_COMMAND : GroundControl.FLEXIFUNCTION_COMMAND
        {
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public byte command_type //Flexifunction command type
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }
        }
        new class FLEXIFUNCTION_COMMAND_ACK : GroundControl.FLEXIFUNCTION_COMMAND_ACK
        {
            public ushort command_type //Command acknowledged
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ushort result //result of acknowledge
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
            }
        }
        new class SERIAL_UDB_EXTRA_F2_A : GroundControl.SERIAL_UDB_EXTRA_F2_A
        {
            public ushort sue_waypoint_index //Serial UDB Extra Waypoint Index
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ushort sue_cog //Serial UDB Extra GPS Course Over Ground
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
            }

            public ushort sue_cpu_load //Serial UDB Extra CPU Load
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
            }

            public ushort sue_air_speed_3DIMU //Serial UDB Extra 3D IMU Air Speed
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  6, 2));}
            }

            public uint sue_time //Serial UDB Extra Time
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  8, 4));}
            }

            public byte sue_status //Serial UDB Extra Status
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  12, 1));}
            }

            public int sue_latitude //Serial UDB Extra Latitude
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  13, 4));}
            }

            public int sue_longitude //Serial UDB Extra Longitude
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  17, 4));}
            }

            public int sue_altitude //Serial UDB Extra Altitude
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  21, 4));}
            }

            public short sue_rmat0 //Serial UDB Extra Rmat 0
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  25, 2));}
            }

            public short sue_rmat1 //Serial UDB Extra Rmat 1
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  27, 2));}
            }

            public short sue_rmat2 //Serial UDB Extra Rmat 2
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  29, 2));}
            }

            public short sue_rmat3 //Serial UDB Extra Rmat 3
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  31, 2));}
            }

            public short sue_rmat4 //Serial UDB Extra Rmat 4
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  33, 2));}
            }

            public short sue_rmat5 //Serial UDB Extra Rmat 5
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  35, 2));}
            }

            public short sue_rmat6 //Serial UDB Extra Rmat 6
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  37, 2));}
            }

            public short sue_rmat7 //Serial UDB Extra Rmat 7
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  39, 2));}
            }

            public short sue_rmat8 //Serial UDB Extra Rmat 8
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  41, 2));}
            }

            public short sue_sog //Serial UDB Extra Speed Over Ground
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  43, 2));}
            }

            public short sue_estimated_wind_0 //Serial UDB Extra Estimated Wind 0
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  45, 2));}
            }

            public short sue_estimated_wind_1 //Serial UDB Extra Estimated Wind 1
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  47, 2));}
            }

            public short sue_estimated_wind_2 //Serial UDB Extra Estimated Wind 2
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  49, 2));}
            }

            public short sue_magFieldEarth0 //Serial UDB Extra Magnetic Field Earth 0
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  51, 2));}
            }

            public short sue_magFieldEarth1 //Serial UDB Extra Magnetic Field Earth 1
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  53, 2));}
            }

            public short sue_magFieldEarth2 //Serial UDB Extra Magnetic Field Earth 2
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  55, 2));}
            }

            public short sue_svs //Serial UDB Extra Number of Sattelites in View
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  57, 2));}
            }

            public short sue_hdop //Serial UDB Extra GPS Horizontal Dilution of Precision
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  59, 2));}
            }
        }
        new class SERIAL_UDB_EXTRA_F2_B : GroundControl.SERIAL_UDB_EXTRA_F2_B
        {
            public uint sue_time //Serial UDB Extra Time
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public uint sue_flags //Serial UDB Extra Status Flags
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  4, 4));}
            }

            public short sue_pwm_input_1 //Serial UDB Extra PWM Input Channel 1
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  8, 2));}
            }

            public short sue_pwm_input_2 //Serial UDB Extra PWM Input Channel 2
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  10, 2));}
            }

            public short sue_pwm_input_3 //Serial UDB Extra PWM Input Channel 3
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  12, 2));}
            }

            public short sue_pwm_input_4 //Serial UDB Extra PWM Input Channel 4
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  14, 2));}
            }

            public short sue_pwm_input_5 //Serial UDB Extra PWM Input Channel 5
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  16, 2));}
            }

            public short sue_pwm_input_6 //Serial UDB Extra PWM Input Channel 6
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  18, 2));}
            }

            public short sue_pwm_input_7 //Serial UDB Extra PWM Input Channel 7
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  20, 2));}
            }

            public short sue_pwm_input_8 //Serial UDB Extra PWM Input Channel 8
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  22, 2));}
            }

            public short sue_pwm_input_9 //Serial UDB Extra PWM Input Channel 9
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  24, 2));}
            }

            public short sue_pwm_input_10 //Serial UDB Extra PWM Input Channel 10
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  26, 2));}
            }

            public short sue_pwm_input_11 //Serial UDB Extra PWM Input Channel 11
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  28, 2));}
            }

            public short sue_pwm_input_12 //Serial UDB Extra PWM Input Channel 12
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  30, 2));}
            }

            public short sue_pwm_output_1 //Serial UDB Extra PWM Output Channel 1
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  32, 2));}
            }

            public short sue_pwm_output_2 //Serial UDB Extra PWM Output Channel 2
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  34, 2));}
            }

            public short sue_pwm_output_3 //Serial UDB Extra PWM Output Channel 3
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  36, 2));}
            }

            public short sue_pwm_output_4 //Serial UDB Extra PWM Output Channel 4
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  38, 2));}
            }

            public short sue_pwm_output_5 //Serial UDB Extra PWM Output Channel 5
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  40, 2));}
            }

            public short sue_pwm_output_6 //Serial UDB Extra PWM Output Channel 6
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  42, 2));}
            }

            public short sue_pwm_output_7 //Serial UDB Extra PWM Output Channel 7
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  44, 2));}
            }

            public short sue_pwm_output_8 //Serial UDB Extra PWM Output Channel 8
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  46, 2));}
            }

            public short sue_pwm_output_9 //Serial UDB Extra PWM Output Channel 9
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  48, 2));}
            }

            public short sue_pwm_output_10 //Serial UDB Extra PWM Output Channel 10
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  50, 2));}
            }

            public short sue_pwm_output_11 //Serial UDB Extra PWM Output Channel 11
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  52, 2));}
            }

            public short sue_pwm_output_12 //Serial UDB Extra PWM Output Channel 12
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  54, 2));}
            }

            public short sue_imu_location_x //Serial UDB Extra IMU Location X
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  56, 2));}
            }

            public short sue_imu_location_y //Serial UDB Extra IMU Location Y
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  58, 2));}
            }

            public short sue_imu_location_z //Serial UDB Extra IMU Location Z
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  60, 2));}
            }

            public short sue_location_error_earth_x //Serial UDB Location Error Earth X
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  62, 2));}
            }

            public short sue_location_error_earth_y //Serial UDB Location Error Earth Y
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  64, 2));}
            }

            public short sue_location_error_earth_z //Serial UDB Location Error Earth Z
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  66, 2));}
            }

            public short sue_osc_fails //Serial UDB Extra Oscillator Failure Count
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  68, 2));}
            }

            public short sue_imu_velocity_x //Serial UDB Extra IMU Velocity X
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  70, 2));}
            }

            public short sue_imu_velocity_y //Serial UDB Extra IMU Velocity Y
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  72, 2));}
            }

            public short sue_imu_velocity_z //Serial UDB Extra IMU Velocity Z
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  74, 2));}
            }

            public short sue_waypoint_goal_x //Serial UDB Extra Current Waypoint Goal X
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  76, 2));}
            }

            public short sue_waypoint_goal_y //Serial UDB Extra Current Waypoint Goal Y
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  78, 2));}
            }

            public short sue_waypoint_goal_z //Serial UDB Extra Current Waypoint Goal Z
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  80, 2));}
            }

            public short sue_aero_x //Aeroforce in UDB X Axis
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  82, 2));}
            }

            public short sue_aero_y //Aeroforce in UDB Y Axis
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  84, 2));}
            }

            public short sue_aero_z //Aeroforce in UDB Z axis
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  86, 2));}
            }

            public short sue_barom_temp //SUE barometer temperature
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  88, 2));}
            }

            public int sue_barom_press //SUE barometer pressure
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  90, 4));}
            }

            public int sue_barom_alt //SUE barometer altitude
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  94, 4));}
            }

            public short sue_bat_volt //SUE battery voltage
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  98, 2));}
            }

            public short sue_bat_amp //SUE battery current
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  100, 2));}
            }

            public short sue_bat_amp_hours //SUE battery milli amp hours used
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  102, 2));}
            }

            public short sue_desired_height //Sue autopilot desired height
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  104, 2));}
            }

            public short sue_memory_stack_free //Serial UDB Extra Stack Memory Free
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  106, 2));}
            }
        }
        new class SERIAL_UDB_EXTRA_F4 : GroundControl.SERIAL_UDB_EXTRA_F4
        {
            public byte sue_ROLL_STABILIZATION_AILERONS //Serial UDB Extra Roll Stabilization with Ailerons Enabled
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte sue_ROLL_STABILIZATION_RUDDER //Serial UDB Extra Roll Stabilization with Rudder Enabled
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public byte sue_PITCH_STABILIZATION //Serial UDB Extra Pitch Stabilization Enabled
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }

            public byte sue_YAW_STABILIZATION_RUDDER //Serial UDB Extra Yaw Stabilization using Rudder Enabled
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
            }

            public byte sue_YAW_STABILIZATION_AILERON //Serial UDB Extra Yaw Stabilization using Ailerons Enabled
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            public byte sue_AILERON_NAVIGATION //Serial UDB Extra Navigation with Ailerons Enabled
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
            }

            public byte sue_RUDDER_NAVIGATION //Serial UDB Extra Navigation with Rudder Enabled
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  6, 1));}
            }

            public byte sue_ALTITUDEHOLD_STABILIZED //Serial UDB Extra Type of Alitude Hold when in Stabilized Mode
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  7, 1));}
            }

            public byte sue_ALTITUDEHOLD_WAYPOINT //Serial UDB Extra Type of Alitude Hold when in Waypoint Mode
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  8, 1));}
            }

            public byte sue_RACING_MODE //Serial UDB Extra Firmware racing mode enabled
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  9, 1));}
            }
        }
        new class SERIAL_UDB_EXTRA_F5 : GroundControl.SERIAL_UDB_EXTRA_F5
        {
            public float sue_YAWKP_AILERON //Serial UDB YAWKP_AILERON Gain for Proporional control of navigation
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  0, 4)));}
            }

            public float sue_YAWKD_AILERON //Serial UDB YAWKD_AILERON Gain for Rate control of navigation
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float sue_ROLLKP //Serial UDB Extra ROLLKP Gain for Proportional control of roll stabilization
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float sue_ROLLKD //Serial UDB Extra ROLLKD Gain for Rate control of roll stabilization
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }
        }
        new class SERIAL_UDB_EXTRA_F6 : GroundControl.SERIAL_UDB_EXTRA_F6
        {
            public float sue_PITCHGAIN //Serial UDB Extra PITCHGAIN Proportional Control
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  0, 4)));}
            }

            public float sue_PITCHKD //Serial UDB Extra Pitch Rate Control
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float sue_RUDDER_ELEV_MIX //Serial UDB Extra Rudder to Elevator Mix
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float sue_ROLL_ELEV_MIX //Serial UDB Extra Roll to Elevator Mix
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float sue_ELEVATOR_BOOST //Gain For Boosting Manual Elevator control When Plane Stabilized
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }
        }
        new class SERIAL_UDB_EXTRA_F7 : GroundControl.SERIAL_UDB_EXTRA_F7
        {
            public float sue_YAWKP_RUDDER //Serial UDB YAWKP_RUDDER Gain for Proporional control of navigation
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  0, 4)));}
            }

            public float sue_YAWKD_RUDDER //Serial UDB YAWKD_RUDDER Gain for Rate control of navigation
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float sue_ROLLKP_RUDDER //Serial UDB Extra ROLLKP_RUDDER Gain for Proportional control of roll stabilization
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float sue_ROLLKD_RUDDER //Serial UDB Extra ROLLKD_RUDDER Gain for Rate control of roll stabilization
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float sue_RUDDER_BOOST //SERIAL UDB EXTRA Rudder Boost Gain to Manual Control when stabilized
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }

            public float sue_RTL_PITCH_DOWN //Serial UDB Extra Return To Landing - Angle to Pitch Plane Down
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
            }
        }
        new class SERIAL_UDB_EXTRA_F8 : GroundControl.SERIAL_UDB_EXTRA_F8
        {
            public float sue_HEIGHT_TARGET_MAX //Serial UDB Extra HEIGHT_TARGET_MAX
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  0, 4)));}
            }

            public float sue_HEIGHT_TARGET_MIN //Serial UDB Extra HEIGHT_TARGET_MIN
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float sue_ALT_HOLD_THROTTLE_MIN //Serial UDB Extra ALT_HOLD_THROTTLE_MIN
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float sue_ALT_HOLD_THROTTLE_MAX //Serial UDB Extra ALT_HOLD_THROTTLE_MAX
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float sue_ALT_HOLD_PITCH_MIN //Serial UDB Extra ALT_HOLD_PITCH_MIN
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }

            public float sue_ALT_HOLD_PITCH_MAX //Serial UDB Extra ALT_HOLD_PITCH_MAX
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
            }

            public float sue_ALT_HOLD_PITCH_HIGH //Serial UDB Extra ALT_HOLD_PITCH_HIGH
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
            }
        }
        new class SERIAL_UDB_EXTRA_F13 : GroundControl.SERIAL_UDB_EXTRA_F13
        {
            public short sue_week_no //Serial UDB Extra GPS Week Number
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  0, 2));}
            }

            public int sue_lat_origin //Serial UDB Extra MP Origin Latitude
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  2, 4));}
            }

            public int sue_lon_origin //Serial UDB Extra MP Origin Longitude
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  6, 4));}
            }

            public int sue_alt_origin //Serial UDB Extra MP Origin Altitude Above Sea Level
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  10, 4));}
            }
        }
        new class SERIAL_UDB_EXTRA_F14 : GroundControl.SERIAL_UDB_EXTRA_F14
        {
            public uint sue_TRAP_SOURCE //Serial UDB Extra Type Program Address of Last Trap
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public byte sue_WIND_ESTIMATION //Serial UDB Extra Wind Estimation Enabled
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            public byte sue_GPS_TYPE //Serial UDB Extra Type of GPS Unit
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
            }

            public byte sue_DR //Serial UDB Extra Dead Reckoning Enabled
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  6, 1));}
            }

            public byte sue_BOARD_TYPE //Serial UDB Extra Type of UDB Hardware
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  7, 1));}
            }

            public byte sue_AIRFRAME //Serial UDB Extra Type of Airframe
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  8, 1));}
            }

            public short sue_RCON //Serial UDB Extra Reboot Register of DSPIC
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  9, 2));}
            }

            public short sue_TRAP_FLAGS //Serial UDB Extra  Last dspic Trap Flags
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  11, 2));}
            }

            public short sue_osc_fail_count //Serial UDB Extra Number of Ocillator Failures
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  13, 2));}
            }

            public byte sue_CLOCK_CONFIG //Serial UDB Extra UDB Internal Clock Configuration
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  15, 1));}
            }

            public byte sue_FLIGHT_PLAN_TYPE //Serial UDB Extra Type of Flight Plan
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  16, 1));}
            }
        }
        new class SERIAL_UDB_EXTRA_F15 : GroundControl.SERIAL_UDB_EXTRA_F15
        {
            public byte[] sue_ID_VEHICLE_MODEL_NAME //Serial UDB Extra Model Name Of Vehicle
            {
                get {return sue_ID_VEHICLE_MODEL_NAME_GET(new byte[40], 0);}
            }
            public byte[]sue_ID_VEHICLE_MODEL_NAME_GET(byte[] dst_ch, int pos)  //Serial UDB Extra Model Name Of Vehicle
            {
                for(int BYTE = 0, dst_max = pos + 40; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            public byte[] sue_ID_VEHICLE_REGISTRATION //Serial UDB Extra Registraton Number of Vehicle
            {
                get {return sue_ID_VEHICLE_REGISTRATION_GET(new byte[20], 0);}
            }
            public byte[]sue_ID_VEHICLE_REGISTRATION_GET(byte[] dst_ch, int pos)  //Serial UDB Extra Registraton Number of Vehicle
            {
                for(int BYTE = 40, dst_max = pos + 20; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
        }
        new class SERIAL_UDB_EXTRA_F16 : GroundControl.SERIAL_UDB_EXTRA_F16
        {
            public byte[] sue_ID_LEAD_PILOT //Serial UDB Extra Name of Expected Lead Pilot
            {
                get {return sue_ID_LEAD_PILOT_GET(new byte[40], 0);}
            }
            public byte[]sue_ID_LEAD_PILOT_GET(byte[] dst_ch, int pos)  //Serial UDB Extra Name of Expected Lead Pilot
            {
                for(int BYTE = 0, dst_max = pos + 40; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            public byte[] sue_ID_DIY_DRONES_URL //Serial UDB Extra URL of Lead Pilot or Team
            {
                get {return sue_ID_DIY_DRONES_URL_GET(new byte[70], 0);}
            }
            public byte[]sue_ID_DIY_DRONES_URL_GET(byte[] dst_ch, int pos)  //Serial UDB Extra URL of Lead Pilot or Team
            {
                for(int BYTE = 40, dst_max = pos + 70; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
        }
        new class ALTITUDES : GroundControl.ALTITUDES
        {
            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public int alt_gps //GPS altitude in meters, expressed as * 1000 (millimeters), above MSL
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  4, 4));}
            }

            public int alt_imu //IMU altitude above ground in meters, expressed as * 1000 (millimeters)
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  8, 4));}
            }

            public int alt_barometric //barometeric altitude above ground in meters, expressed as * 1000 (millimeters)
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  12, 4));}
            }

            public int alt_optical_flow //Optical flow altitude above ground in meters, expressed as * 1000 (millimeters)
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  16, 4));}
            }

            public int alt_range_finder //Rangefinder Altitude above ground in meters, expressed as * 1000 (millimeters)
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  20, 4));}
            }

            public int alt_extra //Extra altitude above ground in meters, expressed as * 1000 (millimeters)
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  24, 4));}
            }
        }
        new class AIRSPEEDS : GroundControl.AIRSPEEDS
        {
            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public short airspeed_imu //Airspeed estimate from IMU, cm/s
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  4, 2));}
            }

            public short airspeed_pitot //Pitot measured forward airpseed, cm/s
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  6, 2));}
            }

            public short airspeed_hot_wire //Hot wire anenometer measured airspeed, cm/s
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  8, 2));}
            }

            public short airspeed_ultrasonic //Ultrasonic measured airspeed, cm/s
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  10, 2));}
            }

            public short aoa //Angle of attack sensor, degrees * 10
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  12, 2));}
            }

            public short aoy //Yaw angle sensor, degrees * 10
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  14, 2));}
            }
        }
        new class SERIAL_UDB_EXTRA_F17 : GroundControl.SERIAL_UDB_EXTRA_F17
        {
            public float sue_feed_forward //SUE Feed Forward Gain
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  0, 4)));}
            }

            public float sue_turn_rate_nav //SUE Max Turn Rate when Navigating
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float sue_turn_rate_fbw //SUE Max Turn Rate in Fly By Wire Mode
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }
        }
        new class SERIAL_UDB_EXTRA_F18 : GroundControl.SERIAL_UDB_EXTRA_F18
        {
            public float angle_of_attack_normal //SUE Angle of Attack Normal
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  0, 4)));}
            }

            public float angle_of_attack_inverted //SUE Angle of Attack Inverted
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float elevator_trim_normal //SUE Elevator Trim Normal
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float elevator_trim_inverted //SUE Elevator Trim Inverted
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float reference_speed //SUE reference_speed
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }
        }
        new class SERIAL_UDB_EXTRA_F19 : GroundControl.SERIAL_UDB_EXTRA_F19
        {
            public byte sue_aileron_output_channel //SUE aileron output channel
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte sue_aileron_reversed //SUE aileron reversed
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public byte sue_elevator_output_channel //SUE elevator output channel
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }

            public byte sue_elevator_reversed //SUE elevator reversed
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
            }

            public byte sue_throttle_output_channel //SUE throttle output channel
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            public byte sue_throttle_reversed //SUE throttle reversed
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
            }

            public byte sue_rudder_output_channel //SUE rudder output channel
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  6, 1));}
            }

            public byte sue_rudder_reversed //SUE rudder reversed
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  7, 1));}
            }
        }
        new class SERIAL_UDB_EXTRA_F20 : GroundControl.SERIAL_UDB_EXTRA_F20
        {
            public byte sue_number_of_inputs //SUE Number of Input Channels
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public short sue_trim_value_input_1 //SUE UDB PWM Trim Value on Input 1
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  1, 2));}
            }

            public short sue_trim_value_input_2 //SUE UDB PWM Trim Value on Input 2
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  3, 2));}
            }

            public short sue_trim_value_input_3 //SUE UDB PWM Trim Value on Input 3
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  5, 2));}
            }

            public short sue_trim_value_input_4 //SUE UDB PWM Trim Value on Input 4
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  7, 2));}
            }

            public short sue_trim_value_input_5 //SUE UDB PWM Trim Value on Input 5
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  9, 2));}
            }

            public short sue_trim_value_input_6 //SUE UDB PWM Trim Value on Input 6
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  11, 2));}
            }

            public short sue_trim_value_input_7 //SUE UDB PWM Trim Value on Input 7
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  13, 2));}
            }

            public short sue_trim_value_input_8 //SUE UDB PWM Trim Value on Input 8
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  15, 2));}
            }

            public short sue_trim_value_input_9 //SUE UDB PWM Trim Value on Input 9
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  17, 2));}
            }

            public short sue_trim_value_input_10 //SUE UDB PWM Trim Value on Input 10
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  19, 2));}
            }

            public short sue_trim_value_input_11 //SUE UDB PWM Trim Value on Input 11
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  21, 2));}
            }

            public short sue_trim_value_input_12 //SUE UDB PWM Trim Value on Input 12
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  23, 2));}
            }
        }
        new class SERIAL_UDB_EXTRA_F21 : GroundControl.SERIAL_UDB_EXTRA_F21
        {
            public short sue_accel_x_offset //SUE X accelerometer offset
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  0, 2));}
            }

            public short sue_accel_y_offset //SUE Y accelerometer offset
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  2, 2));}
            }

            public short sue_accel_z_offset //SUE Z accelerometer offset
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  4, 2));}
            }

            public short sue_gyro_x_offset //SUE X gyro offset
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  6, 2));}
            }

            public short sue_gyro_y_offset //SUE Y gyro offset
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  8, 2));}
            }

            public short sue_gyro_z_offset //SUE Z gyro offset
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  10, 2));}
            }
        }
        new class SERIAL_UDB_EXTRA_F22 : GroundControl.SERIAL_UDB_EXTRA_F22
        {
            public short sue_accel_x_at_calibration //SUE X accelerometer at calibration time
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  0, 2));}
            }

            public short sue_accel_y_at_calibration //SUE Y accelerometer at calibration time
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  2, 2));}
            }

            public short sue_accel_z_at_calibration //SUE Z accelerometer at calibration time
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  4, 2));}
            }

            public short sue_gyro_x_at_calibration //SUE X gyro at calibration time
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  6, 2));}
            }

            public short sue_gyro_y_at_calibration //SUE Y gyro at calibration time
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  8, 2));}
            }

            public short sue_gyro_z_at_calibration //SUE Z gyro at calibration time
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  10, 2));}
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

            public void OnFLEXIFUNCTION_SETReceive_direct(Channel src, Inside ph, FLEXIFUNCTION_SET pack) {OnFLEXIFUNCTION_SETReceive(this, ph,  pack);}
            public event FLEXIFUNCTION_SETReceiveHandler OnFLEXIFUNCTION_SETReceive;
            public delegate void FLEXIFUNCTION_SETReceiveHandler(Channel src, Inside ph, FLEXIFUNCTION_SET pack);
            public void OnFLEXIFUNCTION_READ_REQReceive_direct(Channel src, Inside ph, FLEXIFUNCTION_READ_REQ pack) {OnFLEXIFUNCTION_READ_REQReceive(this, ph,  pack);}
            public event FLEXIFUNCTION_READ_REQReceiveHandler OnFLEXIFUNCTION_READ_REQReceive;
            public delegate void FLEXIFUNCTION_READ_REQReceiveHandler(Channel src, Inside ph, FLEXIFUNCTION_READ_REQ pack);
            public void OnFLEXIFUNCTION_BUFFER_FUNCTIONReceive_direct(Channel src, Inside ph, FLEXIFUNCTION_BUFFER_FUNCTION pack) {OnFLEXIFUNCTION_BUFFER_FUNCTIONReceive(this, ph,  pack);}
            public event FLEXIFUNCTION_BUFFER_FUNCTIONReceiveHandler OnFLEXIFUNCTION_BUFFER_FUNCTIONReceive;
            public delegate void FLEXIFUNCTION_BUFFER_FUNCTIONReceiveHandler(Channel src, Inside ph, FLEXIFUNCTION_BUFFER_FUNCTION pack);
            public void OnFLEXIFUNCTION_BUFFER_FUNCTION_ACKReceive_direct(Channel src, Inside ph, FLEXIFUNCTION_BUFFER_FUNCTION_ACK pack) {OnFLEXIFUNCTION_BUFFER_FUNCTION_ACKReceive(this, ph,  pack);}
            public event FLEXIFUNCTION_BUFFER_FUNCTION_ACKReceiveHandler OnFLEXIFUNCTION_BUFFER_FUNCTION_ACKReceive;
            public delegate void FLEXIFUNCTION_BUFFER_FUNCTION_ACKReceiveHandler(Channel src, Inside ph, FLEXIFUNCTION_BUFFER_FUNCTION_ACK pack);
            public void OnFLEXIFUNCTION_DIRECTORYReceive_direct(Channel src, Inside ph, FLEXIFUNCTION_DIRECTORY pack) {OnFLEXIFUNCTION_DIRECTORYReceive(this, ph,  pack);}
            public event FLEXIFUNCTION_DIRECTORYReceiveHandler OnFLEXIFUNCTION_DIRECTORYReceive;
            public delegate void FLEXIFUNCTION_DIRECTORYReceiveHandler(Channel src, Inside ph, FLEXIFUNCTION_DIRECTORY pack);
            public void OnFLEXIFUNCTION_DIRECTORY_ACKReceive_direct(Channel src, Inside ph, FLEXIFUNCTION_DIRECTORY_ACK pack) {OnFLEXIFUNCTION_DIRECTORY_ACKReceive(this, ph,  pack);}
            public event FLEXIFUNCTION_DIRECTORY_ACKReceiveHandler OnFLEXIFUNCTION_DIRECTORY_ACKReceive;
            public delegate void FLEXIFUNCTION_DIRECTORY_ACKReceiveHandler(Channel src, Inside ph, FLEXIFUNCTION_DIRECTORY_ACK pack);
            public void OnFLEXIFUNCTION_COMMANDReceive_direct(Channel src, Inside ph, FLEXIFUNCTION_COMMAND pack) {OnFLEXIFUNCTION_COMMANDReceive(this, ph,  pack);}
            public event FLEXIFUNCTION_COMMANDReceiveHandler OnFLEXIFUNCTION_COMMANDReceive;
            public delegate void FLEXIFUNCTION_COMMANDReceiveHandler(Channel src, Inside ph, FLEXIFUNCTION_COMMAND pack);
            public void OnFLEXIFUNCTION_COMMAND_ACKReceive_direct(Channel src, Inside ph, FLEXIFUNCTION_COMMAND_ACK pack) {OnFLEXIFUNCTION_COMMAND_ACKReceive(this, ph,  pack);}
            public event FLEXIFUNCTION_COMMAND_ACKReceiveHandler OnFLEXIFUNCTION_COMMAND_ACKReceive;
            public delegate void FLEXIFUNCTION_COMMAND_ACKReceiveHandler(Channel src, Inside ph, FLEXIFUNCTION_COMMAND_ACK pack);
            public void OnSERIAL_UDB_EXTRA_F2_AReceive_direct(Channel src, Inside ph, SERIAL_UDB_EXTRA_F2_A pack) {OnSERIAL_UDB_EXTRA_F2_AReceive(this, ph,  pack);}
            public event SERIAL_UDB_EXTRA_F2_AReceiveHandler OnSERIAL_UDB_EXTRA_F2_AReceive;
            public delegate void SERIAL_UDB_EXTRA_F2_AReceiveHandler(Channel src, Inside ph, SERIAL_UDB_EXTRA_F2_A pack);
            public void OnSERIAL_UDB_EXTRA_F2_BReceive_direct(Channel src, Inside ph, SERIAL_UDB_EXTRA_F2_B pack) {OnSERIAL_UDB_EXTRA_F2_BReceive(this, ph,  pack);}
            public event SERIAL_UDB_EXTRA_F2_BReceiveHandler OnSERIAL_UDB_EXTRA_F2_BReceive;
            public delegate void SERIAL_UDB_EXTRA_F2_BReceiveHandler(Channel src, Inside ph, SERIAL_UDB_EXTRA_F2_B pack);
            public void OnSERIAL_UDB_EXTRA_F4Receive_direct(Channel src, Inside ph, SERIAL_UDB_EXTRA_F4 pack) {OnSERIAL_UDB_EXTRA_F4Receive(this, ph,  pack);}
            public event SERIAL_UDB_EXTRA_F4ReceiveHandler OnSERIAL_UDB_EXTRA_F4Receive;
            public delegate void SERIAL_UDB_EXTRA_F4ReceiveHandler(Channel src, Inside ph, SERIAL_UDB_EXTRA_F4 pack);
            public void OnSERIAL_UDB_EXTRA_F5Receive_direct(Channel src, Inside ph, SERIAL_UDB_EXTRA_F5 pack) {OnSERIAL_UDB_EXTRA_F5Receive(this, ph,  pack);}
            public event SERIAL_UDB_EXTRA_F5ReceiveHandler OnSERIAL_UDB_EXTRA_F5Receive;
            public delegate void SERIAL_UDB_EXTRA_F5ReceiveHandler(Channel src, Inside ph, SERIAL_UDB_EXTRA_F5 pack);
            public void OnSERIAL_UDB_EXTRA_F6Receive_direct(Channel src, Inside ph, SERIAL_UDB_EXTRA_F6 pack) {OnSERIAL_UDB_EXTRA_F6Receive(this, ph,  pack);}
            public event SERIAL_UDB_EXTRA_F6ReceiveHandler OnSERIAL_UDB_EXTRA_F6Receive;
            public delegate void SERIAL_UDB_EXTRA_F6ReceiveHandler(Channel src, Inside ph, SERIAL_UDB_EXTRA_F6 pack);
            public void OnSERIAL_UDB_EXTRA_F7Receive_direct(Channel src, Inside ph, SERIAL_UDB_EXTRA_F7 pack) {OnSERIAL_UDB_EXTRA_F7Receive(this, ph,  pack);}
            public event SERIAL_UDB_EXTRA_F7ReceiveHandler OnSERIAL_UDB_EXTRA_F7Receive;
            public delegate void SERIAL_UDB_EXTRA_F7ReceiveHandler(Channel src, Inside ph, SERIAL_UDB_EXTRA_F7 pack);
            public void OnSERIAL_UDB_EXTRA_F8Receive_direct(Channel src, Inside ph, SERIAL_UDB_EXTRA_F8 pack) {OnSERIAL_UDB_EXTRA_F8Receive(this, ph,  pack);}
            public event SERIAL_UDB_EXTRA_F8ReceiveHandler OnSERIAL_UDB_EXTRA_F8Receive;
            public delegate void SERIAL_UDB_EXTRA_F8ReceiveHandler(Channel src, Inside ph, SERIAL_UDB_EXTRA_F8 pack);
            public void OnSERIAL_UDB_EXTRA_F13Receive_direct(Channel src, Inside ph, SERIAL_UDB_EXTRA_F13 pack) {OnSERIAL_UDB_EXTRA_F13Receive(this, ph,  pack);}
            public event SERIAL_UDB_EXTRA_F13ReceiveHandler OnSERIAL_UDB_EXTRA_F13Receive;
            public delegate void SERIAL_UDB_EXTRA_F13ReceiveHandler(Channel src, Inside ph, SERIAL_UDB_EXTRA_F13 pack);
            public void OnSERIAL_UDB_EXTRA_F14Receive_direct(Channel src, Inside ph, SERIAL_UDB_EXTRA_F14 pack) {OnSERIAL_UDB_EXTRA_F14Receive(this, ph,  pack);}
            public event SERIAL_UDB_EXTRA_F14ReceiveHandler OnSERIAL_UDB_EXTRA_F14Receive;
            public delegate void SERIAL_UDB_EXTRA_F14ReceiveHandler(Channel src, Inside ph, SERIAL_UDB_EXTRA_F14 pack);
            public void OnSERIAL_UDB_EXTRA_F15Receive_direct(Channel src, Inside ph, SERIAL_UDB_EXTRA_F15 pack) {OnSERIAL_UDB_EXTRA_F15Receive(this, ph,  pack);}
            public event SERIAL_UDB_EXTRA_F15ReceiveHandler OnSERIAL_UDB_EXTRA_F15Receive;
            public delegate void SERIAL_UDB_EXTRA_F15ReceiveHandler(Channel src, Inside ph, SERIAL_UDB_EXTRA_F15 pack);
            public void OnSERIAL_UDB_EXTRA_F16Receive_direct(Channel src, Inside ph, SERIAL_UDB_EXTRA_F16 pack) {OnSERIAL_UDB_EXTRA_F16Receive(this, ph,  pack);}
            public event SERIAL_UDB_EXTRA_F16ReceiveHandler OnSERIAL_UDB_EXTRA_F16Receive;
            public delegate void SERIAL_UDB_EXTRA_F16ReceiveHandler(Channel src, Inside ph, SERIAL_UDB_EXTRA_F16 pack);
            public void OnALTITUDESReceive_direct(Channel src, Inside ph, ALTITUDES pack) {OnALTITUDESReceive(this, ph,  pack);}
            public event ALTITUDESReceiveHandler OnALTITUDESReceive;
            public delegate void ALTITUDESReceiveHandler(Channel src, Inside ph, ALTITUDES pack);
            public void OnAIRSPEEDSReceive_direct(Channel src, Inside ph, AIRSPEEDS pack) {OnAIRSPEEDSReceive(this, ph,  pack);}
            public event AIRSPEEDSReceiveHandler OnAIRSPEEDSReceive;
            public delegate void AIRSPEEDSReceiveHandler(Channel src, Inside ph, AIRSPEEDS pack);
            public void OnSERIAL_UDB_EXTRA_F17Receive_direct(Channel src, Inside ph, SERIAL_UDB_EXTRA_F17 pack) {OnSERIAL_UDB_EXTRA_F17Receive(this, ph,  pack);}
            public event SERIAL_UDB_EXTRA_F17ReceiveHandler OnSERIAL_UDB_EXTRA_F17Receive;
            public delegate void SERIAL_UDB_EXTRA_F17ReceiveHandler(Channel src, Inside ph, SERIAL_UDB_EXTRA_F17 pack);
            public void OnSERIAL_UDB_EXTRA_F18Receive_direct(Channel src, Inside ph, SERIAL_UDB_EXTRA_F18 pack) {OnSERIAL_UDB_EXTRA_F18Receive(this, ph,  pack);}
            public event SERIAL_UDB_EXTRA_F18ReceiveHandler OnSERIAL_UDB_EXTRA_F18Receive;
            public delegate void SERIAL_UDB_EXTRA_F18ReceiveHandler(Channel src, Inside ph, SERIAL_UDB_EXTRA_F18 pack);
            public void OnSERIAL_UDB_EXTRA_F19Receive_direct(Channel src, Inside ph, SERIAL_UDB_EXTRA_F19 pack) {OnSERIAL_UDB_EXTRA_F19Receive(this, ph,  pack);}
            public event SERIAL_UDB_EXTRA_F19ReceiveHandler OnSERIAL_UDB_EXTRA_F19Receive;
            public delegate void SERIAL_UDB_EXTRA_F19ReceiveHandler(Channel src, Inside ph, SERIAL_UDB_EXTRA_F19 pack);
            public void OnSERIAL_UDB_EXTRA_F20Receive_direct(Channel src, Inside ph, SERIAL_UDB_EXTRA_F20 pack) {OnSERIAL_UDB_EXTRA_F20Receive(this, ph,  pack);}
            public event SERIAL_UDB_EXTRA_F20ReceiveHandler OnSERIAL_UDB_EXTRA_F20Receive;
            public delegate void SERIAL_UDB_EXTRA_F20ReceiveHandler(Channel src, Inside ph, SERIAL_UDB_EXTRA_F20 pack);
            public void OnSERIAL_UDB_EXTRA_F21Receive_direct(Channel src, Inside ph, SERIAL_UDB_EXTRA_F21 pack) {OnSERIAL_UDB_EXTRA_F21Receive(this, ph,  pack);}
            public event SERIAL_UDB_EXTRA_F21ReceiveHandler OnSERIAL_UDB_EXTRA_F21Receive;
            public delegate void SERIAL_UDB_EXTRA_F21ReceiveHandler(Channel src, Inside ph, SERIAL_UDB_EXTRA_F21 pack);
            public void OnSERIAL_UDB_EXTRA_F22Receive_direct(Channel src, Inside ph, SERIAL_UDB_EXTRA_F22 pack) {OnSERIAL_UDB_EXTRA_F22Receive(this, ph,  pack);}
            public event SERIAL_UDB_EXTRA_F22ReceiveHandler OnSERIAL_UDB_EXTRA_F22Receive;
            public delegate void SERIAL_UDB_EXTRA_F22ReceiveHandler(Channel src, Inside ph, SERIAL_UDB_EXTRA_F22 pack);
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
                    case 150:
                        if(pack == null) return new FLEXIFUNCTION_SET();
                        OnFLEXIFUNCTION_SETReceive(this, ph, (FLEXIFUNCTION_SET) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 151:
                        if(pack == null) return new FLEXIFUNCTION_READ_REQ();
                        OnFLEXIFUNCTION_READ_REQReceive(this, ph, (FLEXIFUNCTION_READ_REQ) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 152:
                        if(pack == null) return new FLEXIFUNCTION_BUFFER_FUNCTION();
                        OnFLEXIFUNCTION_BUFFER_FUNCTIONReceive(this, ph, (FLEXIFUNCTION_BUFFER_FUNCTION) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 153:
                        if(pack == null) return new FLEXIFUNCTION_BUFFER_FUNCTION_ACK();
                        OnFLEXIFUNCTION_BUFFER_FUNCTION_ACKReceive(this, ph, (FLEXIFUNCTION_BUFFER_FUNCTION_ACK) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 155:
                        if(pack == null) return new FLEXIFUNCTION_DIRECTORY();
                        OnFLEXIFUNCTION_DIRECTORYReceive(this, ph, (FLEXIFUNCTION_DIRECTORY) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 156:
                        if(pack == null) return new FLEXIFUNCTION_DIRECTORY_ACK();
                        OnFLEXIFUNCTION_DIRECTORY_ACKReceive(this, ph, (FLEXIFUNCTION_DIRECTORY_ACK) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 157:
                        if(pack == null) return new FLEXIFUNCTION_COMMAND();
                        OnFLEXIFUNCTION_COMMANDReceive(this, ph, (FLEXIFUNCTION_COMMAND) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 158:
                        if(pack == null) return new FLEXIFUNCTION_COMMAND_ACK();
                        OnFLEXIFUNCTION_COMMAND_ACKReceive(this, ph, (FLEXIFUNCTION_COMMAND_ACK) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 170:
                        if(pack == null) return new SERIAL_UDB_EXTRA_F2_A();
                        OnSERIAL_UDB_EXTRA_F2_AReceive(this, ph, (SERIAL_UDB_EXTRA_F2_A) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 171:
                        if(pack == null) return new SERIAL_UDB_EXTRA_F2_B();
                        OnSERIAL_UDB_EXTRA_F2_BReceive(this, ph, (SERIAL_UDB_EXTRA_F2_B) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 172:
                        if(pack == null) return new SERIAL_UDB_EXTRA_F4();
                        OnSERIAL_UDB_EXTRA_F4Receive(this, ph, (SERIAL_UDB_EXTRA_F4) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 173:
                        if(pack == null) return new SERIAL_UDB_EXTRA_F5();
                        OnSERIAL_UDB_EXTRA_F5Receive(this, ph, (SERIAL_UDB_EXTRA_F5) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 174:
                        if(pack == null) return new SERIAL_UDB_EXTRA_F6();
                        OnSERIAL_UDB_EXTRA_F6Receive(this, ph, (SERIAL_UDB_EXTRA_F6) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 175:
                        if(pack == null) return new SERIAL_UDB_EXTRA_F7();
                        OnSERIAL_UDB_EXTRA_F7Receive(this, ph, (SERIAL_UDB_EXTRA_F7) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 176:
                        if(pack == null) return new SERIAL_UDB_EXTRA_F8();
                        OnSERIAL_UDB_EXTRA_F8Receive(this, ph, (SERIAL_UDB_EXTRA_F8) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 177:
                        if(pack == null) return new SERIAL_UDB_EXTRA_F13();
                        OnSERIAL_UDB_EXTRA_F13Receive(this, ph, (SERIAL_UDB_EXTRA_F13) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 178:
                        if(pack == null) return new SERIAL_UDB_EXTRA_F14();
                        OnSERIAL_UDB_EXTRA_F14Receive(this, ph, (SERIAL_UDB_EXTRA_F14) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 179:
                        if(pack == null) return new SERIAL_UDB_EXTRA_F15();
                        OnSERIAL_UDB_EXTRA_F15Receive(this, ph, (SERIAL_UDB_EXTRA_F15) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 180:
                        if(pack == null) return new SERIAL_UDB_EXTRA_F16();
                        OnSERIAL_UDB_EXTRA_F16Receive(this, ph, (SERIAL_UDB_EXTRA_F16) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 181:
                        if(pack == null) return new ALTITUDES();
                        OnALTITUDESReceive(this, ph, (ALTITUDES) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 182:
                        if(pack == null) return new AIRSPEEDS();
                        OnAIRSPEEDSReceive(this, ph, (AIRSPEEDS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 183:
                        if(pack == null) return new SERIAL_UDB_EXTRA_F17();
                        OnSERIAL_UDB_EXTRA_F17Receive(this, ph, (SERIAL_UDB_EXTRA_F17) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 184:
                        if(pack == null) return new SERIAL_UDB_EXTRA_F18();
                        OnSERIAL_UDB_EXTRA_F18Receive(this, ph, (SERIAL_UDB_EXTRA_F18) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 185:
                        if(pack == null) return new SERIAL_UDB_EXTRA_F19();
                        OnSERIAL_UDB_EXTRA_F19Receive(this, ph, (SERIAL_UDB_EXTRA_F19) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 186:
                        if(pack == null) return new SERIAL_UDB_EXTRA_F20();
                        OnSERIAL_UDB_EXTRA_F20Receive(this, ph, (SERIAL_UDB_EXTRA_F20) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 187:
                        if(pack == null) return new SERIAL_UDB_EXTRA_F21();
                        OnSERIAL_UDB_EXTRA_F21Receive(this, ph, (SERIAL_UDB_EXTRA_F21) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 188:
                        if(pack == null) return new SERIAL_UDB_EXTRA_F22();
                        OnSERIAL_UDB_EXTRA_F22Receive(this, ph, (SERIAL_UDB_EXTRA_F22) pack);//no any host channels can receive this pack. Handle it with test channel handler
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
                Debug.Assert(pack.custom_mode == (uint)4039043113U);
                Debug.Assert(pack.mavlink_version == (byte)(byte)173);
                Debug.Assert(pack.autopilot == (MAV_AUTOPILOT)MAV_AUTOPILOT.MAV_AUTOPILOT_SMACCMPILOT);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED);
                Debug.Assert(pack.system_status == (MAV_STATE)MAV_STATE.MAV_STATE_UNINIT);
                Debug.Assert(pack.type == (MAV_TYPE)MAV_TYPE.MAV_TYPE_ANTENNA_TRACKER);
            };
            HEARTBEAT p0 = new HEARTBEAT();
            PH.setPack(p0);
            p0.type = (MAV_TYPE)MAV_TYPE.MAV_TYPE_ANTENNA_TRACKER;
            p0.system_status = (MAV_STATE)MAV_STATE.MAV_STATE_UNINIT;
            p0.custom_mode = (uint)4039043113U;
            p0.autopilot = (MAV_AUTOPILOT)MAV_AUTOPILOT.MAV_AUTOPILOT_SMACCMPILOT;
            p0.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
            p0.mavlink_version = (byte)(byte)173;
            ADV_TEST_CH.send(p0);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.onboard_control_sensors_health == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL);
                Debug.Assert(pack.errors_count2 == (ushort)(ushort)53160);
                Debug.Assert(pack.onboard_control_sensors_present == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL);
                Debug.Assert(pack.voltage_battery == (ushort)(ushort)4221);
                Debug.Assert(pack.drop_rate_comm == (ushort)(ushort)26760);
                Debug.Assert(pack.onboard_control_sensors_enabled == (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE);
                Debug.Assert(pack.current_battery == (short)(short) -31397);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte)6);
                Debug.Assert(pack.errors_count3 == (ushort)(ushort)11556);
                Debug.Assert(pack.errors_count4 == (ushort)(ushort)43840);
                Debug.Assert(pack.errors_count1 == (ushort)(ushort)51559);
                Debug.Assert(pack.load == (ushort)(ushort)38480);
                Debug.Assert(pack.errors_comm == (ushort)(ushort)43176);
            };
            SYS_STATUS p1 = new SYS_STATUS();
            PH.setPack(p1);
            p1.voltage_battery = (ushort)(ushort)4221;
            p1.errors_comm = (ushort)(ushort)43176;
            p1.drop_rate_comm = (ushort)(ushort)26760;
            p1.onboard_control_sensors_present = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL;
            p1.battery_remaining = (sbyte)(sbyte)6;
            p1.errors_count4 = (ushort)(ushort)43840;
            p1.onboard_control_sensors_enabled = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
            p1.errors_count3 = (ushort)(ushort)11556;
            p1.errors_count1 = (ushort)(ushort)51559;
            p1.load = (ushort)(ushort)38480;
            p1.onboard_control_sensors_health = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL;
            p1.current_battery = (short)(short) -31397;
            p1.errors_count2 = (ushort)(ushort)53160;
            ADV_TEST_CH.send(p1);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYSTEM_TIMEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_unix_usec == (ulong)2437886292825054043L);
                Debug.Assert(pack.time_boot_ms == (uint)4171532166U);
            };
            SYSTEM_TIME p2 = new SYSTEM_TIME();
            PH.setPack(p2);
            p2.time_unix_usec = (ulong)2437886292825054043L;
            p2.time_boot_ms = (uint)4171532166U;
            ADV_TEST_CH.send(p2);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vy == (float)5.4444706E37F);
                Debug.Assert(pack.vx == (float)2.4814636E38F);
                Debug.Assert(pack.vz == (float) -2.9956214E38F);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL);
                Debug.Assert(pack.yaw == (float)4.1041354E37F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)45336);
                Debug.Assert(pack.afy == (float) -2.889555E38F);
                Debug.Assert(pack.afx == (float) -2.2349404E38F);
                Debug.Assert(pack.x == (float) -1.6602594E38F);
                Debug.Assert(pack.afz == (float)3.0295667E38F);
                Debug.Assert(pack.y == (float) -2.3039292E38F);
                Debug.Assert(pack.z == (float)3.040617E38F);
                Debug.Assert(pack.yaw_rate == (float)2.5720365E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1014052545U);
            };
            POSITION_TARGET_LOCAL_NED p3 = new POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.y = (float) -2.3039292E38F;
            p3.afx = (float) -2.2349404E38F;
            p3.x = (float) -1.6602594E38F;
            p3.type_mask = (ushort)(ushort)45336;
            p3.yaw_rate = (float)2.5720365E38F;
            p3.vy = (float)5.4444706E37F;
            p3.time_boot_ms = (uint)1014052545U;
            p3.vz = (float) -2.9956214E38F;
            p3.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL;
            p3.yaw = (float)4.1041354E37F;
            p3.afy = (float) -2.889555E38F;
            p3.afz = (float)3.0295667E38F;
            p3.vx = (float)2.4814636E38F;
            p3.z = (float)3.040617E38F;
            ADV_TEST_CH.send(p3);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)222);
                Debug.Assert(pack.target_system == (byte)(byte)105);
                Debug.Assert(pack.seq == (uint)1743880589U);
                Debug.Assert(pack.time_usec == (ulong)5169753746011176498L);
            };
            PING p4 = new PING();
            PH.setPack(p4);
            p4.target_system = (byte)(byte)105;
            p4.time_usec = (ulong)5169753746011176498L;
            p4.target_component = (byte)(byte)222;
            p4.seq = (uint)1743880589U;
            ADV_TEST_CH.send(p4);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.passkey_LEN(ph) == 22);
                Debug.Assert(pack.passkey_TRY(ph).Equals("tbqtuRwnkzhdptjpqfkxub"));
                Debug.Assert(pack.control_request == (byte)(byte)34);
                Debug.Assert(pack.target_system == (byte)(byte)197);
                Debug.Assert(pack.version == (byte)(byte)204);
            };
            CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
            PH.setPack(p5);
            p5.target_system = (byte)(byte)197;
            p5.control_request = (byte)(byte)34;
            p5.passkey_SET("tbqtuRwnkzhdptjpqfkxub", PH) ;
            p5.version = (byte)(byte)204;
            ADV_TEST_CH.send(p5);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROL_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ack == (byte)(byte)24);
                Debug.Assert(pack.control_request == (byte)(byte)180);
                Debug.Assert(pack.gcs_system_id == (byte)(byte)79);
            };
            CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
            PH.setPack(p6);
            p6.ack = (byte)(byte)24;
            p6.control_request = (byte)(byte)180;
            p6.gcs_system_id = (byte)(byte)79;
            ADV_TEST_CH.send(p6);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnAUTH_KEYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.key_LEN(ph) == 17);
                Debug.Assert(pack.key_TRY(ph).Equals("wayquefksxtuicfwy"));
            };
            AUTH_KEY p7 = new AUTH_KEY();
            PH.setPack(p7);
            p7.key_SET("wayquefksxtuicfwy", PH) ;
            ADV_TEST_CH.send(p7);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_MODEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.base_mode == (MAV_MODE)MAV_MODE.MAV_MODE_PREFLIGHT);
                Debug.Assert(pack.target_system == (byte)(byte)96);
                Debug.Assert(pack.custom_mode == (uint)1073518978U);
            };
            SET_MODE p11 = new SET_MODE();
            PH.setPack(p11);
            p11.custom_mode = (uint)1073518978U;
            p11.base_mode = (MAV_MODE)MAV_MODE.MAV_MODE_PREFLIGHT;
            p11.target_system = (byte)(byte)96;
            ADV_TEST_CH.send(p11);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)183);
                Debug.Assert(pack.param_id_LEN(ph) == 16);
                Debug.Assert(pack.param_id_TRY(ph).Equals("bwigiqGkyyxyrgxe"));
                Debug.Assert(pack.target_component == (byte)(byte)210);
                Debug.Assert(pack.param_index == (short)(short) -10974);
            };
            PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
            PH.setPack(p20);
            p20.target_system = (byte)(byte)183;
            p20.target_component = (byte)(byte)210;
            p20.param_index = (short)(short) -10974;
            p20.param_id_SET("bwigiqGkyyxyrgxe", PH) ;
            ADV_TEST_CH.send(p20);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)73);
                Debug.Assert(pack.target_system == (byte)(byte)104);
            };
            PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
            PH.setPack(p21);
            p21.target_component = (byte)(byte)73;
            p21.target_system = (byte)(byte)104;
            ADV_TEST_CH.send(p21);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_count == (ushort)(ushort)17229);
                Debug.Assert(pack.param_value == (float) -7.359983E37F);
                Debug.Assert(pack.param_index == (ushort)(ushort)2061);
                Debug.Assert(pack.param_type == (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32);
                Debug.Assert(pack.param_id_LEN(ph) == 16);
                Debug.Assert(pack.param_id_TRY(ph).Equals("XqydzewphfYmcift"));
            };
            PARAM_VALUE p22 = new PARAM_VALUE();
            PH.setPack(p22);
            p22.param_count = (ushort)(ushort)17229;
            p22.param_index = (ushort)(ushort)2061;
            p22.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32;
            p22.param_value = (float) -7.359983E37F;
            p22.param_id_SET("XqydzewphfYmcift", PH) ;
            ADV_TEST_CH.send(p22);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)143);
                Debug.Assert(pack.param_type == (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16);
                Debug.Assert(pack.target_component == (byte)(byte)213);
                Debug.Assert(pack.param_id_LEN(ph) == 10);
                Debug.Assert(pack.param_id_TRY(ph).Equals("jaamdrhvgo"));
                Debug.Assert(pack.param_value == (float)8.768457E37F);
            };
            PARAM_SET p23 = new PARAM_SET();
            PH.setPack(p23);
            p23.target_component = (byte)(byte)213;
            p23.param_id_SET("jaamdrhvgo", PH) ;
            p23.target_system = (byte)(byte)143;
            p23.param_value = (float)8.768457E37F;
            p23.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16;
            ADV_TEST_CH.send(p23);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RAW_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT);
                Debug.Assert(pack.epv == (ushort)(ushort)51754);
                Debug.Assert(pack.h_acc_TRY(ph) == (uint)2002332962U);
                Debug.Assert(pack.hdg_acc_TRY(ph) == (uint)2338172066U);
                Debug.Assert(pack.alt == (int)1850332011);
                Debug.Assert(pack.vel == (ushort)(ushort)5430);
                Debug.Assert(pack.vel_acc_TRY(ph) == (uint)3317721573U);
                Debug.Assert(pack.time_usec == (ulong)3364729058085492349L);
                Debug.Assert(pack.lat == (int) -1854150461);
                Debug.Assert(pack.cog == (ushort)(ushort)52084);
                Debug.Assert(pack.eph == (ushort)(ushort)54783);
                Debug.Assert(pack.alt_ellipsoid_TRY(ph) == (int) -1561774375);
                Debug.Assert(pack.satellites_visible == (byte)(byte)132);
                Debug.Assert(pack.lon == (int) -1612442535);
                Debug.Assert(pack.v_acc_TRY(ph) == (uint)1724881117U);
            };
            GPS_RAW_INT p24 = new GPS_RAW_INT();
            PH.setPack(p24);
            p24.alt_ellipsoid_SET((int) -1561774375, PH) ;
            p24.vel = (ushort)(ushort)5430;
            p24.cog = (ushort)(ushort)52084;
            p24.v_acc_SET((uint)1724881117U, PH) ;
            p24.eph = (ushort)(ushort)54783;
            p24.vel_acc_SET((uint)3317721573U, PH) ;
            p24.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT;
            p24.hdg_acc_SET((uint)2338172066U, PH) ;
            p24.lon = (int) -1612442535;
            p24.epv = (ushort)(ushort)51754;
            p24.satellites_visible = (byte)(byte)132;
            p24.alt = (int)1850332011;
            p24.lat = (int) -1854150461;
            p24.h_acc_SET((uint)2002332962U, PH) ;
            p24.time_usec = (ulong)3364729058085492349L;
            ADV_TEST_CH.send(p24);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellite_elevation.SequenceEqual(new byte[] {(byte)93, (byte)61, (byte)89, (byte)181, (byte)193, (byte)200, (byte)247, (byte)219, (byte)169, (byte)79, (byte)124, (byte)214, (byte)10, (byte)28, (byte)245, (byte)23, (byte)61, (byte)44, (byte)165, (byte)103}));
                Debug.Assert(pack.satellite_prn.SequenceEqual(new byte[] {(byte)87, (byte)250, (byte)224, (byte)105, (byte)208, (byte)185, (byte)240, (byte)2, (byte)83, (byte)61, (byte)200, (byte)183, (byte)135, (byte)135, (byte)98, (byte)171, (byte)111, (byte)109, (byte)222, (byte)117}));
                Debug.Assert(pack.satellite_snr.SequenceEqual(new byte[] {(byte)68, (byte)191, (byte)147, (byte)17, (byte)88, (byte)119, (byte)136, (byte)202, (byte)215, (byte)66, (byte)88, (byte)33, (byte)205, (byte)154, (byte)198, (byte)111, (byte)130, (byte)116, (byte)8, (byte)164}));
                Debug.Assert(pack.satellite_used.SequenceEqual(new byte[] {(byte)230, (byte)250, (byte)143, (byte)2, (byte)150, (byte)70, (byte)64, (byte)207, (byte)240, (byte)197, (byte)155, (byte)237, (byte)145, (byte)220, (byte)144, (byte)202, (byte)248, (byte)56, (byte)24, (byte)139}));
                Debug.Assert(pack.satellite_azimuth.SequenceEqual(new byte[] {(byte)64, (byte)49, (byte)143, (byte)164, (byte)195, (byte)255, (byte)184, (byte)151, (byte)130, (byte)35, (byte)237, (byte)82, (byte)145, (byte)185, (byte)167, (byte)9, (byte)60, (byte)242, (byte)162, (byte)88}));
                Debug.Assert(pack.satellites_visible == (byte)(byte)251);
            };
            GPS_STATUS p25 = new GPS_STATUS();
            PH.setPack(p25);
            p25.satellites_visible = (byte)(byte)251;
            p25.satellite_used_SET(new byte[] {(byte)230, (byte)250, (byte)143, (byte)2, (byte)150, (byte)70, (byte)64, (byte)207, (byte)240, (byte)197, (byte)155, (byte)237, (byte)145, (byte)220, (byte)144, (byte)202, (byte)248, (byte)56, (byte)24, (byte)139}, 0) ;
            p25.satellite_elevation_SET(new byte[] {(byte)93, (byte)61, (byte)89, (byte)181, (byte)193, (byte)200, (byte)247, (byte)219, (byte)169, (byte)79, (byte)124, (byte)214, (byte)10, (byte)28, (byte)245, (byte)23, (byte)61, (byte)44, (byte)165, (byte)103}, 0) ;
            p25.satellite_azimuth_SET(new byte[] {(byte)64, (byte)49, (byte)143, (byte)164, (byte)195, (byte)255, (byte)184, (byte)151, (byte)130, (byte)35, (byte)237, (byte)82, (byte)145, (byte)185, (byte)167, (byte)9, (byte)60, (byte)242, (byte)162, (byte)88}, 0) ;
            p25.satellite_snr_SET(new byte[] {(byte)68, (byte)191, (byte)147, (byte)17, (byte)88, (byte)119, (byte)136, (byte)202, (byte)215, (byte)66, (byte)88, (byte)33, (byte)205, (byte)154, (byte)198, (byte)111, (byte)130, (byte)116, (byte)8, (byte)164}, 0) ;
            p25.satellite_prn_SET(new byte[] {(byte)87, (byte)250, (byte)224, (byte)105, (byte)208, (byte)185, (byte)240, (byte)2, (byte)83, (byte)61, (byte)200, (byte)183, (byte)135, (byte)135, (byte)98, (byte)171, (byte)111, (byte)109, (byte)222, (byte)117}, 0) ;
            ADV_TEST_CH.send(p25);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zmag == (short)(short) -23672);
                Debug.Assert(pack.xmag == (short)(short)18766);
                Debug.Assert(pack.time_boot_ms == (uint)2102853576U);
                Debug.Assert(pack.xacc == (short)(short)8771);
                Debug.Assert(pack.zgyro == (short)(short)5680);
                Debug.Assert(pack.yacc == (short)(short)22552);
                Debug.Assert(pack.ymag == (short)(short) -326);
                Debug.Assert(pack.zacc == (short)(short)11355);
                Debug.Assert(pack.ygyro == (short)(short) -2962);
                Debug.Assert(pack.xgyro == (short)(short) -13835);
            };
            SCALED_IMU p26 = new SCALED_IMU();
            PH.setPack(p26);
            p26.zgyro = (short)(short)5680;
            p26.yacc = (short)(short)22552;
            p26.time_boot_ms = (uint)2102853576U;
            p26.xacc = (short)(short)8771;
            p26.zacc = (short)(short)11355;
            p26.ymag = (short)(short) -326;
            p26.xgyro = (short)(short) -13835;
            p26.ygyro = (short)(short) -2962;
            p26.xmag = (short)(short)18766;
            p26.zmag = (short)(short) -23672;
            ADV_TEST_CH.send(p26);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xmag == (short)(short)29220);
                Debug.Assert(pack.zgyro == (short)(short) -6310);
                Debug.Assert(pack.zmag == (short)(short)18783);
                Debug.Assert(pack.yacc == (short)(short) -29211);
                Debug.Assert(pack.ymag == (short)(short) -21030);
                Debug.Assert(pack.ygyro == (short)(short)9716);
                Debug.Assert(pack.zacc == (short)(short) -31170);
                Debug.Assert(pack.time_usec == (ulong)3364431722058675052L);
                Debug.Assert(pack.xgyro == (short)(short) -28870);
                Debug.Assert(pack.xacc == (short)(short)5458);
            };
            RAW_IMU p27 = new RAW_IMU();
            PH.setPack(p27);
            p27.xgyro = (short)(short) -28870;
            p27.xacc = (short)(short)5458;
            p27.time_usec = (ulong)3364431722058675052L;
            p27.zgyro = (short)(short) -6310;
            p27.zmag = (short)(short)18783;
            p27.xmag = (short)(short)29220;
            p27.ygyro = (short)(short)9716;
            p27.yacc = (short)(short) -29211;
            p27.ymag = (short)(short) -21030;
            p27.zacc = (short)(short) -31170;
            ADV_TEST_CH.send(p27);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_abs == (short)(short)7862);
                Debug.Assert(pack.temperature == (short)(short)17511);
                Debug.Assert(pack.time_usec == (ulong)1841136903945637097L);
                Debug.Assert(pack.press_diff1 == (short)(short)28258);
                Debug.Assert(pack.press_diff2 == (short)(short)25336);
            };
            RAW_PRESSURE p28 = new RAW_PRESSURE();
            PH.setPack(p28);
            p28.temperature = (short)(short)17511;
            p28.press_diff1 = (short)(short)28258;
            p28.press_abs = (short)(short)7862;
            p28.press_diff2 = (short)(short)25336;
            p28.time_usec = (ulong)1841136903945637097L;
            ADV_TEST_CH.send(p28);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)337758138U);
                Debug.Assert(pack.press_diff == (float) -1.5844651E38F);
                Debug.Assert(pack.press_abs == (float)3.1851476E38F);
                Debug.Assert(pack.temperature == (short)(short)32337);
            };
            SCALED_PRESSURE p29 = new SCALED_PRESSURE();
            PH.setPack(p29);
            p29.press_diff = (float) -1.5844651E38F;
            p29.temperature = (short)(short)32337;
            p29.press_abs = (float)3.1851476E38F;
            p29.time_boot_ms = (uint)337758138U;
            ADV_TEST_CH.send(p29);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitchspeed == (float)4.8229897E37F);
                Debug.Assert(pack.time_boot_ms == (uint)1250098723U);
                Debug.Assert(pack.rollspeed == (float) -2.6483896E38F);
                Debug.Assert(pack.yaw == (float)7.403569E37F);
                Debug.Assert(pack.yawspeed == (float)2.3149488E38F);
                Debug.Assert(pack.pitch == (float)2.813976E37F);
                Debug.Assert(pack.roll == (float)2.564502E36F);
            };
            ATTITUDE p30 = new ATTITUDE();
            PH.setPack(p30);
            p30.roll = (float)2.564502E36F;
            p30.yawspeed = (float)2.3149488E38F;
            p30.yaw = (float)7.403569E37F;
            p30.pitch = (float)2.813976E37F;
            p30.time_boot_ms = (uint)1250098723U;
            p30.rollspeed = (float) -2.6483896E38F;
            p30.pitchspeed = (float)4.8229897E37F;
            ADV_TEST_CH.send(p30);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q1 == (float)2.1710109E38F);
                Debug.Assert(pack.q2 == (float) -1.0067822E37F);
                Debug.Assert(pack.rollspeed == (float) -2.7390457E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1889208768U);
                Debug.Assert(pack.q3 == (float)6.5612175E37F);
                Debug.Assert(pack.pitchspeed == (float)3.7812836E37F);
                Debug.Assert(pack.q4 == (float) -4.619742E37F);
                Debug.Assert(pack.yawspeed == (float)1.272027E37F);
            };
            ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
            PH.setPack(p31);
            p31.q4 = (float) -4.619742E37F;
            p31.pitchspeed = (float)3.7812836E37F;
            p31.rollspeed = (float) -2.7390457E38F;
            p31.yawspeed = (float)1.272027E37F;
            p31.q2 = (float) -1.0067822E37F;
            p31.q1 = (float)2.1710109E38F;
            p31.time_boot_ms = (uint)1889208768U;
            p31.q3 = (float)6.5612175E37F;
            ADV_TEST_CH.send(p31);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vx == (float)5.5175623E37F);
                Debug.Assert(pack.time_boot_ms == (uint)3090552598U);
                Debug.Assert(pack.x == (float)2.7150228E38F);
                Debug.Assert(pack.z == (float)3.0492491E38F);
                Debug.Assert(pack.vz == (float) -2.2291088E36F);
                Debug.Assert(pack.y == (float) -6.531897E37F);
                Debug.Assert(pack.vy == (float) -1.6763746E38F);
            };
            LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
            PH.setPack(p32);
            p32.x = (float)2.7150228E38F;
            p32.z = (float)3.0492491E38F;
            p32.vx = (float)5.5175623E37F;
            p32.vy = (float) -1.6763746E38F;
            p32.y = (float) -6.531897E37F;
            p32.time_boot_ms = (uint)3090552598U;
            p32.vz = (float) -2.2291088E36F;
            ADV_TEST_CH.send(p32);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.hdg == (ushort)(ushort)23524);
                Debug.Assert(pack.alt == (int) -386069120);
                Debug.Assert(pack.vy == (short)(short)30437);
                Debug.Assert(pack.time_boot_ms == (uint)811344897U);
                Debug.Assert(pack.vz == (short)(short) -18265);
                Debug.Assert(pack.vx == (short)(short)28303);
                Debug.Assert(pack.relative_alt == (int) -1124305254);
                Debug.Assert(pack.lon == (int)61316902);
                Debug.Assert(pack.lat == (int)1831420903);
            };
            GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
            PH.setPack(p33);
            p33.relative_alt = (int) -1124305254;
            p33.alt = (int) -386069120;
            p33.vy = (short)(short)30437;
            p33.time_boot_ms = (uint)811344897U;
            p33.vx = (short)(short)28303;
            p33.lat = (int)1831420903;
            p33.vz = (short)(short) -18265;
            p33.hdg = (ushort)(ushort)23524;
            p33.lon = (int)61316902;
            ADV_TEST_CH.send(p33);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_SCALEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)4111537355U);
                Debug.Assert(pack.chan6_scaled == (short)(short)20282);
                Debug.Assert(pack.chan1_scaled == (short)(short)15120);
                Debug.Assert(pack.chan2_scaled == (short)(short) -11958);
                Debug.Assert(pack.chan3_scaled == (short)(short) -15025);
                Debug.Assert(pack.port == (byte)(byte)173);
                Debug.Assert(pack.chan7_scaled == (short)(short) -15810);
                Debug.Assert(pack.chan8_scaled == (short)(short)7460);
                Debug.Assert(pack.chan4_scaled == (short)(short) -29433);
                Debug.Assert(pack.rssi == (byte)(byte)218);
                Debug.Assert(pack.chan5_scaled == (short)(short)23893);
            };
            RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
            PH.setPack(p34);
            p34.rssi = (byte)(byte)218;
            p34.chan5_scaled = (short)(short)23893;
            p34.chan2_scaled = (short)(short) -11958;
            p34.chan1_scaled = (short)(short)15120;
            p34.time_boot_ms = (uint)4111537355U;
            p34.chan7_scaled = (short)(short) -15810;
            p34.chan6_scaled = (short)(short)20282;
            p34.chan3_scaled = (short)(short) -15025;
            p34.chan8_scaled = (short)(short)7460;
            p34.port = (byte)(byte)173;
            p34.chan4_scaled = (short)(short) -29433;
            ADV_TEST_CH.send(p34);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.port == (byte)(byte)137);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)35456);
                Debug.Assert(pack.time_boot_ms == (uint)1075239001U);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)56419);
                Debug.Assert(pack.rssi == (byte)(byte)226);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)61656);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)34561);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)52325);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)32470);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)17754);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)1540);
            };
            RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
            PH.setPack(p35);
            p35.chan7_raw = (ushort)(ushort)61656;
            p35.chan8_raw = (ushort)(ushort)35456;
            p35.time_boot_ms = (uint)1075239001U;
            p35.rssi = (byte)(byte)226;
            p35.port = (byte)(byte)137;
            p35.chan4_raw = (ushort)(ushort)34561;
            p35.chan5_raw = (ushort)(ushort)1540;
            p35.chan1_raw = (ushort)(ushort)52325;
            p35.chan3_raw = (ushort)(ushort)17754;
            p35.chan2_raw = (ushort)(ushort)32470;
            p35.chan6_raw = (ushort)(ushort)56419;
            ADV_TEST_CH.send(p35);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERVO_OUTPUT_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.servo9_raw_TRY(ph) == (ushort)(ushort)56129);
                Debug.Assert(pack.time_usec == (uint)3970520444U);
                Debug.Assert(pack.servo6_raw == (ushort)(ushort)24313);
                Debug.Assert(pack.port == (byte)(byte)157);
                Debug.Assert(pack.servo3_raw == (ushort)(ushort)13623);
                Debug.Assert(pack.servo13_raw_TRY(ph) == (ushort)(ushort)46261);
                Debug.Assert(pack.servo7_raw == (ushort)(ushort)62183);
                Debug.Assert(pack.servo12_raw_TRY(ph) == (ushort)(ushort)32057);
                Debug.Assert(pack.servo4_raw == (ushort)(ushort)62175);
                Debug.Assert(pack.servo16_raw_TRY(ph) == (ushort)(ushort)26967);
                Debug.Assert(pack.servo10_raw_TRY(ph) == (ushort)(ushort)65421);
                Debug.Assert(pack.servo11_raw_TRY(ph) == (ushort)(ushort)18324);
                Debug.Assert(pack.servo5_raw == (ushort)(ushort)56691);
                Debug.Assert(pack.servo8_raw == (ushort)(ushort)2057);
                Debug.Assert(pack.servo2_raw == (ushort)(ushort)30298);
                Debug.Assert(pack.servo14_raw_TRY(ph) == (ushort)(ushort)58248);
                Debug.Assert(pack.servo15_raw_TRY(ph) == (ushort)(ushort)43887);
                Debug.Assert(pack.servo1_raw == (ushort)(ushort)35598);
            };
            SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
            PH.setPack(p36);
            p36.servo8_raw = (ushort)(ushort)2057;
            p36.servo3_raw = (ushort)(ushort)13623;
            p36.servo10_raw_SET((ushort)(ushort)65421, PH) ;
            p36.servo1_raw = (ushort)(ushort)35598;
            p36.servo11_raw_SET((ushort)(ushort)18324, PH) ;
            p36.time_usec = (uint)3970520444U;
            p36.servo2_raw = (ushort)(ushort)30298;
            p36.servo7_raw = (ushort)(ushort)62183;
            p36.servo6_raw = (ushort)(ushort)24313;
            p36.servo16_raw_SET((ushort)(ushort)26967, PH) ;
            p36.servo14_raw_SET((ushort)(ushort)58248, PH) ;
            p36.servo12_raw_SET((ushort)(ushort)32057, PH) ;
            p36.servo13_raw_SET((ushort)(ushort)46261, PH) ;
            p36.servo9_raw_SET((ushort)(ushort)56129, PH) ;
            p36.servo4_raw = (ushort)(ushort)62175;
            p36.servo15_raw_SET((ushort)(ushort)43887, PH) ;
            p36.port = (byte)(byte)157;
            p36.servo5_raw = (ushort)(ushort)56691;
            ADV_TEST_CH.send(p36);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)69);
                Debug.Assert(pack.target_system == (byte)(byte)152);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.start_index == (short)(short) -2124);
                Debug.Assert(pack.end_index == (short)(short)14709);
            };
            MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
            PH.setPack(p37);
            p37.end_index = (short)(short)14709;
            p37.target_component = (byte)(byte)69;
            p37.target_system = (byte)(byte)152;
            p37.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p37.start_index = (short)(short) -2124;
            ADV_TEST_CH.send(p37);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_WRITE_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)2);
                Debug.Assert(pack.end_index == (short)(short)1875);
                Debug.Assert(pack.start_index == (short)(short) -26);
                Debug.Assert(pack.target_system == (byte)(byte)125);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            };
            MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
            PH.setPack(p38);
            p38.target_system = (byte)(byte)125;
            p38.start_index = (short)(short) -26;
            p38.end_index = (short)(short)1875;
            p38.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p38.target_component = (byte)(byte)2;
            ADV_TEST_CH.send(p38);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float) -2.2366056E38F);
                Debug.Assert(pack.param3 == (float)1.8805714E38F);
                Debug.Assert(pack.y == (float) -7.5555895E37F);
                Debug.Assert(pack.param1 == (float)2.155427E38F);
                Debug.Assert(pack.target_system == (byte)(byte)67);
                Debug.Assert(pack.autocontinue == (byte)(byte)153);
                Debug.Assert(pack.target_component == (byte)(byte)234);
                Debug.Assert(pack.current == (byte)(byte)151);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
                Debug.Assert(pack.param2 == (float)2.5355284E38F);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_DO_JUMP);
                Debug.Assert(pack.z == (float) -2.552111E37F);
                Debug.Assert(pack.seq == (ushort)(ushort)48023);
                Debug.Assert(pack.param4 == (float)1.4012915E38F);
            };
            MISSION_ITEM p39 = new MISSION_ITEM();
            PH.setPack(p39);
            p39.target_system = (byte)(byte)67;
            p39.y = (float) -7.5555895E37F;
            p39.param2 = (float)2.5355284E38F;
            p39.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p39.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
            p39.param4 = (float)1.4012915E38F;
            p39.param1 = (float)2.155427E38F;
            p39.z = (float) -2.552111E37F;
            p39.target_component = (byte)(byte)234;
            p39.param3 = (float)1.8805714E38F;
            p39.autocontinue = (byte)(byte)153;
            p39.command = (MAV_CMD)MAV_CMD.MAV_CMD_DO_JUMP;
            p39.current = (byte)(byte)151;
            p39.seq = (ushort)(ushort)48023;
            p39.x = (float) -2.2366056E38F;
            ADV_TEST_CH.send(p39);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)242);
                Debug.Assert(pack.seq == (ushort)(ushort)42593);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_component == (byte)(byte)94);
            };
            MISSION_REQUEST p40 = new MISSION_REQUEST();
            PH.setPack(p40);
            p40.seq = (ushort)(ushort)42593;
            p40.target_system = (byte)(byte)242;
            p40.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p40.target_component = (byte)(byte)94;
            ADV_TEST_CH.send(p40);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_SET_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)47154);
                Debug.Assert(pack.target_system == (byte)(byte)121);
                Debug.Assert(pack.target_component == (byte)(byte)88);
            };
            MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
            PH.setPack(p41);
            p41.seq = (ushort)(ushort)47154;
            p41.target_component = (byte)(byte)88;
            p41.target_system = (byte)(byte)121;
            ADV_TEST_CH.send(p41);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)47533);
            };
            MISSION_CURRENT p42 = new MISSION_CURRENT();
            PH.setPack(p42);
            p42.seq = (ushort)(ushort)47533;
            ADV_TEST_CH.send(p42);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)214);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.target_component == (byte)(byte)161);
            };
            MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
            PH.setPack(p43);
            p43.target_component = (byte)(byte)161;
            p43.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p43.target_system = (byte)(byte)214;
            ADV_TEST_CH.send(p43);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_COUNTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_system == (byte)(byte)101);
                Debug.Assert(pack.count == (ushort)(ushort)3656);
                Debug.Assert(pack.target_component == (byte)(byte)249);
            };
            MISSION_COUNT p44 = new MISSION_COUNT();
            PH.setPack(p44);
            p44.target_system = (byte)(byte)101;
            p44.count = (ushort)(ushort)3656;
            p44.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p44.target_component = (byte)(byte)249;
            ADV_TEST_CH.send(p44);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CLEAR_ALLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)57);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_component == (byte)(byte)204);
            };
            MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
            PH.setPack(p45);
            p45.target_component = (byte)(byte)204;
            p45.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p45.target_system = (byte)(byte)57;
            ADV_TEST_CH.send(p45);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_REACHEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)6629);
            };
            MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
            PH.setPack(p46);
            p46.seq = (ushort)(ushort)6629;
            ADV_TEST_CH.send(p46);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_component == (byte)(byte)46);
                Debug.Assert(pack.target_system == (byte)(byte)2);
                Debug.Assert(pack.type == (MAV_MISSION_RESULT)MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM4);
            };
            MISSION_ACK p47 = new MISSION_ACK();
            PH.setPack(p47);
            p47.target_system = (byte)(byte)2;
            p47.type = (MAV_MISSION_RESULT)MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM4;
            p47.target_component = (byte)(byte)46;
            p47.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            ADV_TEST_CH.send(p47);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_GPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude == (int)1295381690);
                Debug.Assert(pack.longitude == (int)907166709);
                Debug.Assert(pack.latitude == (int) -107105828);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)3729770391649237223L);
                Debug.Assert(pack.target_system == (byte)(byte)28);
            };
            SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
            PH.setPack(p48);
            p48.target_system = (byte)(byte)28;
            p48.time_usec_SET((ulong)3729770391649237223L, PH) ;
            p48.altitude = (int)1295381690;
            p48.latitude = (int) -107105828;
            p48.longitude = (int)907166709;
            ADV_TEST_CH.send(p48);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.longitude == (int)1513958293);
                Debug.Assert(pack.latitude == (int) -10033529);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)3892270472524701907L);
                Debug.Assert(pack.altitude == (int)1657907848);
            };
            GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
            PH.setPack(p49);
            p49.altitude = (int)1657907848;
            p49.longitude = (int)1513958293;
            p49.time_usec_SET((ulong)3892270472524701907L, PH) ;
            p49.latitude = (int) -10033529;
            ADV_TEST_CH.send(p49);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_MAP_RCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 7);
                Debug.Assert(pack.param_id_TRY(ph).Equals("dtpewnv"));
                Debug.Assert(pack.param_value_min == (float) -5.51669E37F);
                Debug.Assert(pack.param_value_max == (float) -1.0449146E38F);
                Debug.Assert(pack.target_component == (byte)(byte)207);
                Debug.Assert(pack.parameter_rc_channel_index == (byte)(byte)222);
                Debug.Assert(pack.param_index == (short)(short) -32057);
                Debug.Assert(pack.target_system == (byte)(byte)147);
                Debug.Assert(pack.param_value0 == (float)2.017455E38F);
                Debug.Assert(pack.scale == (float)2.3890005E38F);
            };
            PARAM_MAP_RC p50 = new PARAM_MAP_RC();
            PH.setPack(p50);
            p50.param_id_SET("dtpewnv", PH) ;
            p50.param_value0 = (float)2.017455E38F;
            p50.scale = (float)2.3890005E38F;
            p50.param_index = (short)(short) -32057;
            p50.target_component = (byte)(byte)207;
            p50.param_value_max = (float) -1.0449146E38F;
            p50.target_system = (byte)(byte)147;
            p50.param_value_min = (float) -5.51669E37F;
            p50.parameter_rc_channel_index = (byte)(byte)222;
            ADV_TEST_CH.send(p50);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)213);
                Debug.Assert(pack.target_component == (byte)(byte)124);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.seq == (ushort)(ushort)53645);
            };
            MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
            PH.setPack(p51);
            p51.target_component = (byte)(byte)124;
            p51.target_system = (byte)(byte)213;
            p51.seq = (ushort)(ushort)53645;
            p51.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            ADV_TEST_CH.send(p51);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_SET_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)15);
                Debug.Assert(pack.target_system == (byte)(byte)169);
                Debug.Assert(pack.p2x == (float)3.3966367E38F);
                Debug.Assert(pack.p2y == (float)2.5713193E38F);
                Debug.Assert(pack.p1z == (float)2.128285E38F);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL);
                Debug.Assert(pack.p1x == (float)3.013474E38F);
                Debug.Assert(pack.p2z == (float)2.7783342E38F);
                Debug.Assert(pack.p1y == (float)3.2647869E38F);
            };
            SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
            PH.setPack(p54);
            p54.p2y = (float)2.5713193E38F;
            p54.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL;
            p54.target_system = (byte)(byte)169;
            p54.target_component = (byte)(byte)15;
            p54.p1z = (float)2.128285E38F;
            p54.p2x = (float)3.3966367E38F;
            p54.p2z = (float)2.7783342E38F;
            p54.p1y = (float)3.2647869E38F;
            p54.p1x = (float)3.013474E38F;
            ADV_TEST_CH.send(p54);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p2x == (float)2.0500731E38F);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
                Debug.Assert(pack.p2z == (float) -3.154445E38F);
                Debug.Assert(pack.p1y == (float) -6.668585E37F);
                Debug.Assert(pack.p1z == (float)9.834397E37F);
                Debug.Assert(pack.p1x == (float) -3.966257E37F);
                Debug.Assert(pack.p2y == (float) -1.4859255E38F);
            };
            SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
            PH.setPack(p55);
            p55.p2z = (float) -3.154445E38F;
            p55.p1y = (float) -6.668585E37F;
            p55.p2x = (float)2.0500731E38F;
            p55.p1z = (float)9.834397E37F;
            p55.p1x = (float) -3.966257E37F;
            p55.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p55.p2y = (float) -1.4859255E38F;
            ADV_TEST_CH.send(p55);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNION_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitchspeed == (float) -8.4214147E37F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.716591E38F, -1.3047803E38F, -1.3546267E38F, -8.868165E37F}));
                Debug.Assert(pack.time_usec == (ulong)4444764286026219094L);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {2.5245546E38F, 2.2577553E38F, -1.5007124E38F, 2.8130424E38F, 3.2738288E38F, 1.2743395E38F, -2.6356545E38F, -8.725743E37F, 1.3851304E38F}));
                Debug.Assert(pack.rollspeed == (float) -1.929702E38F);
                Debug.Assert(pack.yawspeed == (float) -2.2513984E38F);
            };
            ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
            PH.setPack(p61);
            p61.rollspeed = (float) -1.929702E38F;
            p61.q_SET(new float[] {-2.716591E38F, -1.3047803E38F, -1.3546267E38F, -8.868165E37F}, 0) ;
            p61.pitchspeed = (float) -8.4214147E37F;
            p61.yawspeed = (float) -2.2513984E38F;
            p61.time_usec = (ulong)4444764286026219094L;
            p61.covariance_SET(new float[] {2.5245546E38F, 2.2577553E38F, -1.5007124E38F, 2.8130424E38F, 3.2738288E38F, 1.2743395E38F, -2.6356545E38F, -8.725743E37F, 1.3851304E38F}, 0) ;
            ADV_TEST_CH.send(p61);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnNAV_CONTROLLER_OUTPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.aspd_error == (float)8.4363577E37F);
                Debug.Assert(pack.xtrack_error == (float)3.1014003E38F);
                Debug.Assert(pack.nav_pitch == (float) -2.633017E38F);
                Debug.Assert(pack.target_bearing == (short)(short) -5434);
                Debug.Assert(pack.alt_error == (float)2.7675206E38F);
                Debug.Assert(pack.nav_roll == (float) -1.6462659E38F);
                Debug.Assert(pack.wp_dist == (ushort)(ushort)55592);
                Debug.Assert(pack.nav_bearing == (short)(short)32473);
            };
            NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
            PH.setPack(p62);
            p62.nav_roll = (float) -1.6462659E38F;
            p62.target_bearing = (short)(short) -5434;
            p62.alt_error = (float)2.7675206E38F;
            p62.xtrack_error = (float)3.1014003E38F;
            p62.wp_dist = (ushort)(ushort)55592;
            p62.aspd_error = (float)8.4363577E37F;
            p62.nav_pitch = (float) -2.633017E38F;
            p62.nav_bearing = (short)(short)32473;
            ADV_TEST_CH.send(p62);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INT_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.alt == (int) -583231081);
                Debug.Assert(pack.estimator_type == (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION);
                Debug.Assert(pack.vy == (float) -9.185596E37F);
                Debug.Assert(pack.lon == (int)1266716007);
                Debug.Assert(pack.relative_alt == (int) -1268069279);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {5.3034176E37F, 3.2624138E38F, -1.3434427E38F, 9.175376E37F, 1.9311645E38F, -5.25863E37F, -2.7364907E38F, 2.2307474E38F, -6.2609334E37F, -1.5819401E37F, 8.902185E37F, -1.7612017E38F, -2.9565602E38F, -7.941677E37F, 6.886824E37F, -1.5640793E38F, 3.2800327E38F, 1.5876246E38F, -3.5767682E37F, -2.5137386E38F, 1.5061034E38F, -3.1750684E38F, 2.4035357E38F, -1.6927338E38F, 9.4069366E36F, -3.3609218E38F, 2.5652115E38F, 1.7710468E38F, -2.1592811E38F, -3.1523513E38F, 2.4970419E38F, 1.2228161E38F, 7.1586237E37F, 3.0012163E37F, 3.380268E38F, -1.9382191E38F}));
                Debug.Assert(pack.time_usec == (ulong)2925875372673515983L);
                Debug.Assert(pack.vx == (float) -2.1550634E38F);
                Debug.Assert(pack.vz == (float) -2.3578488E38F);
                Debug.Assert(pack.lat == (int)1422314241);
            };
            GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
            PH.setPack(p63);
            p63.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION;
            p63.alt = (int) -583231081;
            p63.vx = (float) -2.1550634E38F;
            p63.covariance_SET(new float[] {5.3034176E37F, 3.2624138E38F, -1.3434427E38F, 9.175376E37F, 1.9311645E38F, -5.25863E37F, -2.7364907E38F, 2.2307474E38F, -6.2609334E37F, -1.5819401E37F, 8.902185E37F, -1.7612017E38F, -2.9565602E38F, -7.941677E37F, 6.886824E37F, -1.5640793E38F, 3.2800327E38F, 1.5876246E38F, -3.5767682E37F, -2.5137386E38F, 1.5061034E38F, -3.1750684E38F, 2.4035357E38F, -1.6927338E38F, 9.4069366E36F, -3.3609218E38F, 2.5652115E38F, 1.7710468E38F, -2.1592811E38F, -3.1523513E38F, 2.4970419E38F, 1.2228161E38F, 7.1586237E37F, 3.0012163E37F, 3.380268E38F, -1.9382191E38F}, 0) ;
            p63.vz = (float) -2.3578488E38F;
            p63.time_usec = (ulong)2925875372673515983L;
            p63.relative_alt = (int) -1268069279;
            p63.lon = (int)1266716007;
            p63.vy = (float) -9.185596E37F;
            p63.lat = (int)1422314241;
            ADV_TEST_CH.send(p63);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vy == (float)1.4525605E38F);
                Debug.Assert(pack.time_usec == (ulong)5387429021651686901L);
                Debug.Assert(pack.vz == (float) -3.2808542E38F);
                Debug.Assert(pack.z == (float)2.5406103E38F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {1.3677906E38F, -2.791651E38F, -1.8618794E37F, -1.1279175E38F, 1.2858688E38F, 1.44916E38F, -1.7738499E38F, -1.960787E38F, -2.8368893E38F, 1.7949428E37F, 3.3726704E38F, -5.999837E37F, -6.1464706E37F, 1.1328811E38F, 2.6684735E38F, -3.0981616E38F, 2.6600252E38F, -6.96413E37F, -3.1336978E38F, 1.625166E38F, -1.3210578E38F, 1.4194409E38F, -2.1416434E38F, 1.227963E38F, -9.252466E37F, 2.9839483E38F, 1.6228374E38F, -2.4307616E38F, -1.1537145E38F, 1.2409868E38F, -1.3192009E38F, -4.2126532E37F, -2.3783967E37F, -3.3721465E38F, 5.516291E37F, 9.595987E37F, 2.0248061E38F, -1.370157E38F, 2.1135863E38F, -1.101455E38F, -1.4524947E38F, 2.6190534E37F, 1.7852644E37F, -3.2367616E38F, 3.327069E38F}));
                Debug.Assert(pack.estimator_type == (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION);
                Debug.Assert(pack.x == (float)2.6873132E38F);
                Debug.Assert(pack.y == (float) -5.1744854E37F);
                Debug.Assert(pack.vx == (float) -2.3882738E38F);
                Debug.Assert(pack.az == (float) -2.6272724E38F);
                Debug.Assert(pack.ax == (float) -1.9875313E38F);
                Debug.Assert(pack.ay == (float)3.9827062E37F);
            };
            LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
            PH.setPack(p64);
            p64.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION;
            p64.vy = (float)1.4525605E38F;
            p64.vx = (float) -2.3882738E38F;
            p64.time_usec = (ulong)5387429021651686901L;
            p64.y = (float) -5.1744854E37F;
            p64.ay = (float)3.9827062E37F;
            p64.x = (float)2.6873132E38F;
            p64.ax = (float) -1.9875313E38F;
            p64.z = (float)2.5406103E38F;
            p64.az = (float) -2.6272724E38F;
            p64.vz = (float) -3.2808542E38F;
            p64.covariance_SET(new float[] {1.3677906E38F, -2.791651E38F, -1.8618794E37F, -1.1279175E38F, 1.2858688E38F, 1.44916E38F, -1.7738499E38F, -1.960787E38F, -2.8368893E38F, 1.7949428E37F, 3.3726704E38F, -5.999837E37F, -6.1464706E37F, 1.1328811E38F, 2.6684735E38F, -3.0981616E38F, 2.6600252E38F, -6.96413E37F, -3.1336978E38F, 1.625166E38F, -1.3210578E38F, 1.4194409E38F, -2.1416434E38F, 1.227963E38F, -9.252466E37F, 2.9839483E38F, 1.6228374E38F, -2.4307616E38F, -1.1537145E38F, 1.2409868E38F, -1.3192009E38F, -4.2126532E37F, -2.3783967E37F, -3.3721465E38F, 5.516291E37F, 9.595987E37F, 2.0248061E38F, -1.370157E38F, 2.1135863E38F, -1.101455E38F, -1.4524947E38F, 2.6190534E37F, 1.7852644E37F, -3.2367616E38F, 3.327069E38F}, 0) ;
            ADV_TEST_CH.send(p64);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)45381);
                Debug.Assert(pack.chan13_raw == (ushort)(ushort)41751);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)7087);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)19974);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)33906);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)23103);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)8267);
                Debug.Assert(pack.time_boot_ms == (uint)1652436311U);
                Debug.Assert(pack.chan17_raw == (ushort)(ushort)51328);
                Debug.Assert(pack.chan16_raw == (ushort)(ushort)18940);
                Debug.Assert(pack.chancount == (byte)(byte)4);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)24039);
                Debug.Assert(pack.rssi == (byte)(byte)87);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)7684);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)29377);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)4522);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)50499);
                Debug.Assert(pack.chan14_raw == (ushort)(ushort)21870);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)55246);
                Debug.Assert(pack.chan15_raw == (ushort)(ushort)22430);
                Debug.Assert(pack.chan18_raw == (ushort)(ushort)63333);
            };
            RC_CHANNELS p65 = new RC_CHANNELS();
            PH.setPack(p65);
            p65.chan9_raw = (ushort)(ushort)19974;
            p65.time_boot_ms = (uint)1652436311U;
            p65.chan12_raw = (ushort)(ushort)7684;
            p65.chan7_raw = (ushort)(ushort)24039;
            p65.chan2_raw = (ushort)(ushort)8267;
            p65.chan1_raw = (ushort)(ushort)33906;
            p65.chancount = (byte)(byte)4;
            p65.rssi = (byte)(byte)87;
            p65.chan6_raw = (ushort)(ushort)4522;
            p65.chan5_raw = (ushort)(ushort)50499;
            p65.chan3_raw = (ushort)(ushort)7087;
            p65.chan11_raw = (ushort)(ushort)29377;
            p65.chan17_raw = (ushort)(ushort)51328;
            p65.chan10_raw = (ushort)(ushort)55246;
            p65.chan15_raw = (ushort)(ushort)22430;
            p65.chan13_raw = (ushort)(ushort)41751;
            p65.chan18_raw = (ushort)(ushort)63333;
            p65.chan14_raw = (ushort)(ushort)21870;
            p65.chan8_raw = (ushort)(ushort)23103;
            p65.chan16_raw = (ushort)(ushort)18940;
            p65.chan4_raw = (ushort)(ushort)45381;
            ADV_TEST_CH.send(p65);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnREQUEST_DATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start_stop == (byte)(byte)195);
                Debug.Assert(pack.target_system == (byte)(byte)240);
                Debug.Assert(pack.req_message_rate == (ushort)(ushort)24472);
                Debug.Assert(pack.target_component == (byte)(byte)226);
                Debug.Assert(pack.req_stream_id == (byte)(byte)48);
            };
            REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
            PH.setPack(p66);
            p66.target_system = (byte)(byte)240;
            p66.target_component = (byte)(byte)226;
            p66.start_stop = (byte)(byte)195;
            p66.req_message_rate = (ushort)(ushort)24472;
            p66.req_stream_id = (byte)(byte)48;
            ADV_TEST_CH.send(p66);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.message_rate == (ushort)(ushort)59065);
                Debug.Assert(pack.on_off == (byte)(byte)39);
                Debug.Assert(pack.stream_id == (byte)(byte)239);
            };
            DATA_STREAM p67 = new DATA_STREAM();
            PH.setPack(p67);
            p67.stream_id = (byte)(byte)239;
            p67.on_off = (byte)(byte)39;
            p67.message_rate = (ushort)(ushort)59065;
            ADV_TEST_CH.send(p67);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (short)(short)7054);
                Debug.Assert(pack.y == (short)(short) -6000);
                Debug.Assert(pack.r == (short)(short)1857);
                Debug.Assert(pack.x == (short)(short)21983);
                Debug.Assert(pack.buttons == (ushort)(ushort)42685);
                Debug.Assert(pack.target == (byte)(byte)203);
            };
            MANUAL_CONTROL p69 = new MANUAL_CONTROL();
            PH.setPack(p69);
            p69.x = (short)(short)21983;
            p69.y = (short)(short) -6000;
            p69.target = (byte)(byte)203;
            p69.r = (short)(short)1857;
            p69.buttons = (ushort)(ushort)42685;
            p69.z = (short)(short)7054;
            ADV_TEST_CH.send(p69);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_OVERRIDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)30773);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)35406);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)33955);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)146);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)56951);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)1416);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)28319);
                Debug.Assert(pack.target_component == (byte)(byte)234);
                Debug.Assert(pack.target_system == (byte)(byte)220);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)48911);
            };
            RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
            PH.setPack(p70);
            p70.chan8_raw = (ushort)(ushort)1416;
            p70.chan7_raw = (ushort)(ushort)146;
            p70.chan1_raw = (ushort)(ushort)30773;
            p70.chan4_raw = (ushort)(ushort)35406;
            p70.target_system = (byte)(byte)220;
            p70.chan5_raw = (ushort)(ushort)48911;
            p70.target_component = (byte)(byte)234;
            p70.chan3_raw = (ushort)(ushort)33955;
            p70.chan6_raw = (ushort)(ushort)56951;
            p70.chan2_raw = (ushort)(ushort)28319;
            ADV_TEST_CH.send(p70);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
                Debug.Assert(pack.mission_type == (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.current == (byte)(byte)192);
                Debug.Assert(pack.param2 == (float)1.1510889E38F);
                Debug.Assert(pack.target_component == (byte)(byte)174);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_DO_SET_HOME);
                Debug.Assert(pack.seq == (ushort)(ushort)22885);
                Debug.Assert(pack.target_system == (byte)(byte)146);
                Debug.Assert(pack.autocontinue == (byte)(byte)135);
                Debug.Assert(pack.param1 == (float)3.7969888E37F);
                Debug.Assert(pack.param4 == (float)2.7724575E38F);
                Debug.Assert(pack.y == (int) -1540568089);
                Debug.Assert(pack.x == (int)2108054689);
                Debug.Assert(pack.z == (float)7.5565134E37F);
                Debug.Assert(pack.param3 == (float)2.2402984E38F);
            };
            MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
            PH.setPack(p73);
            p73.seq = (ushort)(ushort)22885;
            p73.z = (float)7.5565134E37F;
            p73.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT;
            p73.x = (int)2108054689;
            p73.param1 = (float)3.7969888E37F;
            p73.y = (int) -1540568089;
            p73.current = (byte)(byte)192;
            p73.autocontinue = (byte)(byte)135;
            p73.command = (MAV_CMD)MAV_CMD.MAV_CMD_DO_SET_HOME;
            p73.param3 = (float)2.2402984E38F;
            p73.target_component = (byte)(byte)174;
            p73.param4 = (float)2.7724575E38F;
            p73.param2 = (float)1.1510889E38F;
            p73.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p73.target_system = (byte)(byte)146;
            ADV_TEST_CH.send(p73);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVFR_HUDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.throttle == (ushort)(ushort)12414);
                Debug.Assert(pack.climb == (float)3.3699534E38F);
                Debug.Assert(pack.groundspeed == (float) -3.6652852E37F);
                Debug.Assert(pack.heading == (short)(short) -27375);
                Debug.Assert(pack.airspeed == (float) -1.8125467E38F);
                Debug.Assert(pack.alt == (float) -2.0363953E38F);
            };
            VFR_HUD p74 = new VFR_HUD();
            PH.setPack(p74);
            p74.heading = (short)(short) -27375;
            p74.alt = (float) -2.0363953E38F;
            p74.airspeed = (float) -1.8125467E38F;
            p74.groundspeed = (float) -3.6652852E37F;
            p74.climb = (float)3.3699534E38F;
            p74.throttle = (ushort)(ushort)12414;
            ADV_TEST_CH.send(p74);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param2 == (float)4.375187E37F);
                Debug.Assert(pack.param4 == (float)2.862939E38F);
                Debug.Assert(pack.y == (int) -1981246378);
                Debug.Assert(pack.x == (int) -1154132140);
                Debug.Assert(pack.z == (float)2.970204E38F);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_NED);
                Debug.Assert(pack.target_component == (byte)(byte)221);
                Debug.Assert(pack.param1 == (float)2.0360503E38F);
                Debug.Assert(pack.target_system == (byte)(byte)126);
                Debug.Assert(pack.current == (byte)(byte)59);
                Debug.Assert(pack.param3 == (float)2.8500177E38F);
                Debug.Assert(pack.autocontinue == (byte)(byte)230);
            };
            COMMAND_INT p75 = new COMMAND_INT();
            PH.setPack(p75);
            p75.command = (MAV_CMD)MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION;
            p75.z = (float)2.970204E38F;
            p75.param4 = (float)2.862939E38F;
            p75.param2 = (float)4.375187E37F;
            p75.param1 = (float)2.0360503E38F;
            p75.x = (int) -1154132140;
            p75.param3 = (float)2.8500177E38F;
            p75.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_NED;
            p75.current = (byte)(byte)59;
            p75.y = (int) -1981246378;
            p75.autocontinue = (byte)(byte)230;
            p75.target_system = (byte)(byte)126;
            p75.target_component = (byte)(byte)221;
            ADV_TEST_CH.send(p75);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_LONGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param5 == (float)6.6923124E37F);
                Debug.Assert(pack.confirmation == (byte)(byte)57);
                Debug.Assert(pack.param1 == (float) -5.6125674E36F);
                Debug.Assert(pack.param6 == (float) -3.2491374E38F);
                Debug.Assert(pack.param3 == (float) -3.3877376E38F);
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE);
                Debug.Assert(pack.param4 == (float) -2.7406933E37F);
                Debug.Assert(pack.target_component == (byte)(byte)30);
                Debug.Assert(pack.target_system == (byte)(byte)103);
                Debug.Assert(pack.param2 == (float)1.0967351E38F);
                Debug.Assert(pack.param7 == (float)2.186745E38F);
            };
            COMMAND_LONG p76 = new COMMAND_LONG();
            PH.setPack(p76);
            p76.param7 = (float)2.186745E38F;
            p76.param5 = (float)6.6923124E37F;
            p76.param4 = (float) -2.7406933E37F;
            p76.confirmation = (byte)(byte)57;
            p76.param6 = (float) -3.2491374E38F;
            p76.target_system = (byte)(byte)103;
            p76.target_component = (byte)(byte)30;
            p76.param2 = (float)1.0967351E38F;
            p76.param3 = (float) -3.3877376E38F;
            p76.command = (MAV_CMD)MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE;
            p76.param1 = (float) -5.6125674E36F;
            ADV_TEST_CH.send(p76);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.command == (MAV_CMD)MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE);
                Debug.Assert(pack.result == (MAV_RESULT)MAV_RESULT.MAV_RESULT_DENIED);
                Debug.Assert(pack.target_component_TRY(ph) == (byte)(byte)74);
                Debug.Assert(pack.result_param2_TRY(ph) == (int) -865465212);
                Debug.Assert(pack.progress_TRY(ph) == (byte)(byte)224);
                Debug.Assert(pack.target_system_TRY(ph) == (byte)(byte)182);
            };
            COMMAND_ACK p77 = new COMMAND_ACK();
            PH.setPack(p77);
            p77.target_system_SET((byte)(byte)182, PH) ;
            p77.command = (MAV_CMD)MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE;
            p77.result_param2_SET((int) -865465212, PH) ;
            p77.target_component_SET((byte)(byte)74, PH) ;
            p77.result = (MAV_RESULT)MAV_RESULT.MAV_RESULT_DENIED;
            p77.progress_SET((byte)(byte)224, PH) ;
            ADV_TEST_CH.send(p77);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_SETPOINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.manual_override_switch == (byte)(byte)7);
                Debug.Assert(pack.yaw == (float)3.3540177E38F);
                Debug.Assert(pack.roll == (float)2.8613245E38F);
                Debug.Assert(pack.mode_switch == (byte)(byte)121);
                Debug.Assert(pack.thrust == (float)6.9999965E37F);
                Debug.Assert(pack.pitch == (float) -2.3134076E38F);
                Debug.Assert(pack.time_boot_ms == (uint)670344488U);
            };
            MANUAL_SETPOINT p81 = new MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.yaw = (float)3.3540177E38F;
            p81.manual_override_switch = (byte)(byte)7;
            p81.thrust = (float)6.9999965E37F;
            p81.time_boot_ms = (uint)670344488U;
            p81.mode_switch = (byte)(byte)121;
            p81.pitch = (float) -2.3134076E38F;
            p81.roll = (float)2.8613245E38F;
            ADV_TEST_CH.send(p81);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2031013079U);
                Debug.Assert(pack.type_mask == (byte)(byte)195);
                Debug.Assert(pack.target_component == (byte)(byte)0);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.6114253E38F, 5.5472304E37F, -9.873527E37F, -1.0786587E38F}));
                Debug.Assert(pack.body_yaw_rate == (float)3.2866006E38F);
                Debug.Assert(pack.body_pitch_rate == (float)1.0196364E38F);
                Debug.Assert(pack.target_system == (byte)(byte)146);
                Debug.Assert(pack.body_roll_rate == (float) -3.902886E37F);
                Debug.Assert(pack.thrust == (float) -2.391732E38F);
            };
            SET_ATTITUDE_TARGET p82 = new SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.body_roll_rate = (float) -3.902886E37F;
            p82.q_SET(new float[] {-2.6114253E38F, 5.5472304E37F, -9.873527E37F, -1.0786587E38F}, 0) ;
            p82.time_boot_ms = (uint)2031013079U;
            p82.body_yaw_rate = (float)3.2866006E38F;
            p82.thrust = (float) -2.391732E38F;
            p82.type_mask = (byte)(byte)195;
            p82.target_system = (byte)(byte)146;
            p82.body_pitch_rate = (float)1.0196364E38F;
            p82.target_component = (byte)(byte)0;
            ADV_TEST_CH.send(p82);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.thrust == (float)3.106932E38F);
                Debug.Assert(pack.body_pitch_rate == (float)2.1906532E38F);
                Debug.Assert(pack.body_yaw_rate == (float) -1.5453096E38F);
                Debug.Assert(pack.body_roll_rate == (float) -3.0943637E38F);
                Debug.Assert(pack.type_mask == (byte)(byte)113);
                Debug.Assert(pack.time_boot_ms == (uint)2053884832U);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-3.9727087E37F, -9.022472E37F, 2.2362567E38F, 3.1003397E38F}));
            };
            ATTITUDE_TARGET p83 = new ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.q_SET(new float[] {-3.9727087E37F, -9.022472E37F, 2.2362567E38F, 3.1003397E38F}, 0) ;
            p83.body_pitch_rate = (float)2.1906532E38F;
            p83.body_roll_rate = (float) -3.0943637E38F;
            p83.thrust = (float)3.106932E38F;
            p83.body_yaw_rate = (float) -1.5453096E38F;
            p83.type_mask = (byte)(byte)113;
            p83.time_boot_ms = (uint)2053884832U;
            ADV_TEST_CH.send(p83);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vx == (float)1.5032933E38F);
                Debug.Assert(pack.target_system == (byte)(byte)239);
                Debug.Assert(pack.yaw == (float)1.4197374E38F);
                Debug.Assert(pack.z == (float) -7.747684E37F);
                Debug.Assert(pack.afx == (float)2.1346338E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2863657093U);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_NED);
                Debug.Assert(pack.type_mask == (ushort)(ushort)60461);
                Debug.Assert(pack.target_component == (byte)(byte)197);
                Debug.Assert(pack.x == (float)4.0371703E37F);
                Debug.Assert(pack.yaw_rate == (float) -2.1934187E38F);
                Debug.Assert(pack.y == (float) -4.9858387E37F);
                Debug.Assert(pack.vz == (float)1.9841393E38F);
                Debug.Assert(pack.afy == (float) -3.1002123E38F);
                Debug.Assert(pack.vy == (float) -1.1451821E38F);
                Debug.Assert(pack.afz == (float)1.2811978E38F);
            };
            SET_POSITION_TARGET_LOCAL_NED p84 = new SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.y = (float) -4.9858387E37F;
            p84.vy = (float) -1.1451821E38F;
            p84.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_NED;
            p84.target_system = (byte)(byte)239;
            p84.vx = (float)1.5032933E38F;
            p84.afy = (float) -3.1002123E38F;
            p84.afz = (float)1.2811978E38F;
            p84.time_boot_ms = (uint)2863657093U;
            p84.target_component = (byte)(byte)197;
            p84.type_mask = (ushort)(ushort)60461;
            p84.afx = (float)2.1346338E38F;
            p84.z = (float) -7.747684E37F;
            p84.yaw = (float)1.4197374E38F;
            p84.yaw_rate = (float) -2.1934187E38F;
            p84.x = (float)4.0371703E37F;
            p84.vz = (float)1.9841393E38F;
            ADV_TEST_CH.send(p84);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type_mask == (ushort)(ushort)32069);
                Debug.Assert(pack.vz == (float) -2.0099083E38F);
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
                Debug.Assert(pack.yaw_rate == (float) -2.2905465E38F);
                Debug.Assert(pack.lat_int == (int)214291068);
                Debug.Assert(pack.afz == (float) -1.1172187E38F);
                Debug.Assert(pack.vx == (float) -3.4209478E37F);
                Debug.Assert(pack.afx == (float) -2.7177394E38F);
                Debug.Assert(pack.yaw == (float) -2.3020598E38F);
                Debug.Assert(pack.vy == (float) -3.0783132E38F);
                Debug.Assert(pack.time_boot_ms == (uint)4212508735U);
                Debug.Assert(pack.lon_int == (int)1048503885);
                Debug.Assert(pack.afy == (float)9.643219E37F);
                Debug.Assert(pack.target_component == (byte)(byte)179);
                Debug.Assert(pack.alt == (float) -6.302955E37F);
                Debug.Assert(pack.target_system == (byte)(byte)55);
            };
            GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.afy = (float)9.643219E37F;
            p86.alt = (float) -6.302955E37F;
            p86.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p86.type_mask = (ushort)(ushort)32069;
            p86.lat_int = (int)214291068;
            p86.afx = (float) -2.7177394E38F;
            p86.vx = (float) -3.4209478E37F;
            p86.afz = (float) -1.1172187E38F;
            p86.time_boot_ms = (uint)4212508735U;
            p86.target_component = (byte)(byte)179;
            p86.yaw_rate = (float) -2.2905465E38F;
            p86.vy = (float) -3.0783132E38F;
            p86.lon_int = (int)1048503885;
            p86.yaw = (float) -2.3020598E38F;
            p86.vz = (float) -2.0099083E38F;
            p86.target_system = (byte)(byte)55;
            CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.coordinate_frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_MISSION);
                Debug.Assert(pack.yaw == (float)1.4316601E37F);
                Debug.Assert(pack.yaw_rate == (float) -7.217205E36F);
                Debug.Assert(pack.lat_int == (int) -1895336224);
                Debug.Assert(pack.lon_int == (int)1936447194);
                Debug.Assert(pack.afy == (float)1.3782518E38F);
                Debug.Assert(pack.vz == (float) -2.8767703E37F);
                Debug.Assert(pack.time_boot_ms == (uint)1747330792U);
                Debug.Assert(pack.vy == (float)2.607754E38F);
                Debug.Assert(pack.vx == (float) -9.898158E36F);
                Debug.Assert(pack.afz == (float)1.1901112E38F);
                Debug.Assert(pack.afx == (float)4.570529E37F);
                Debug.Assert(pack.alt == (float) -2.3203352E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)10572);
            };
            GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.time_boot_ms = (uint)1747330792U;
            p87.afz = (float)1.1901112E38F;
            p87.yaw_rate = (float) -7.217205E36F;
            p87.vx = (float) -9.898158E36F;
            p87.lat_int = (int) -1895336224;
            p87.vy = (float)2.607754E38F;
            p87.afx = (float)4.570529E37F;
            p87.yaw = (float)1.4316601E37F;
            p87.vz = (float) -2.8767703E37F;
            p87.lon_int = (int)1936447194;
            p87.alt = (float) -2.3203352E38F;
            p87.type_mask = (ushort)(ushort)10572;
            p87.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_MISSION;
            p87.afy = (float)1.3782518E38F;
            CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float)4.28662E37F);
                Debug.Assert(pack.x == (float) -2.115332E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2480322382U);
                Debug.Assert(pack.roll == (float) -3.5678193E37F);
                Debug.Assert(pack.y == (float) -1.3002815E38F);
                Debug.Assert(pack.pitch == (float) -1.7808753E38F);
                Debug.Assert(pack.yaw == (float) -1.3549079E37F);
            };
            GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.yaw = (float) -1.3549079E37F;
            p89.time_boot_ms = (uint)2480322382U;
            p89.x = (float) -2.115332E38F;
            p89.roll = (float) -3.5678193E37F;
            p89.z = (float)4.28662E37F;
            p89.pitch = (float) -1.7808753E38F;
            p89.y = (float) -1.3002815E38F;
            CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int)1957528927);
                Debug.Assert(pack.pitchspeed == (float)2.9837702E38F);
                Debug.Assert(pack.lat == (int)2053009766);
                Debug.Assert(pack.roll == (float)2.968983E37F);
                Debug.Assert(pack.alt == (int)1742723100);
                Debug.Assert(pack.zacc == (short)(short) -29706);
                Debug.Assert(pack.yacc == (short)(short) -7725);
                Debug.Assert(pack.vy == (short)(short) -24326);
                Debug.Assert(pack.yawspeed == (float) -1.7416402E38F);
                Debug.Assert(pack.time_usec == (ulong)1479623141815326594L);
                Debug.Assert(pack.rollspeed == (float) -8.799454E37F);
                Debug.Assert(pack.vz == (short)(short)5286);
                Debug.Assert(pack.xacc == (short)(short)1747);
                Debug.Assert(pack.vx == (short)(short)7851);
                Debug.Assert(pack.pitch == (float)9.698063E36F);
                Debug.Assert(pack.yaw == (float)5.9320364E37F);
            };
            GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.vz = (short)(short)5286;
            p90.yawspeed = (float) -1.7416402E38F;
            p90.time_usec = (ulong)1479623141815326594L;
            p90.roll = (float)2.968983E37F;
            p90.zacc = (short)(short) -29706;
            p90.yacc = (short)(short) -7725;
            p90.vy = (short)(short) -24326;
            p90.pitch = (float)9.698063E36F;
            p90.yaw = (float)5.9320364E37F;
            p90.pitchspeed = (float)2.9837702E38F;
            p90.alt = (int)1742723100;
            p90.lat = (int)2053009766;
            p90.vx = (short)(short)7851;
            p90.lon = (int)1957528927;
            p90.xacc = (short)(short)1747;
            p90.rollspeed = (float) -8.799454E37F;
            CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.aux2 == (float)3.132224E38F);
                Debug.Assert(pack.aux1 == (float) -2.5221288E38F);
                Debug.Assert(pack.pitch_elevator == (float)8.774273E37F);
                Debug.Assert(pack.aux3 == (float)1.5781673E38F);
                Debug.Assert(pack.nav_mode == (byte)(byte)82);
                Debug.Assert(pack.time_usec == (ulong)7591749239304562330L);
                Debug.Assert(pack.aux4 == (float) -3.240565E38F);
                Debug.Assert(pack.yaw_rudder == (float)1.3649201E38F);
                Debug.Assert(pack.mode == (MAV_MODE)MAV_MODE.MAV_MODE_TEST_DISARMED);
                Debug.Assert(pack.roll_ailerons == (float)2.9929841E38F);
                Debug.Assert(pack.throttle == (float) -8.92108E37F);
            };
            GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.time_usec = (ulong)7591749239304562330L;
            p91.throttle = (float) -8.92108E37F;
            p91.mode = (MAV_MODE)MAV_MODE.MAV_MODE_TEST_DISARMED;
            p91.aux2 = (float)3.132224E38F;
            p91.roll_ailerons = (float)2.9929841E38F;
            p91.pitch_elevator = (float)8.774273E37F;
            p91.yaw_rudder = (float)1.3649201E38F;
            p91.aux4 = (float) -3.240565E38F;
            p91.nav_mode = (byte)(byte)82;
            p91.aux3 = (float)1.5781673E38F;
            p91.aux1 = (float) -2.5221288E38F;
            CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_RC_INPUTS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)1547);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)64006);
                Debug.Assert(pack.rssi == (byte)(byte)56);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)43519);
                Debug.Assert(pack.time_usec == (ulong)3470309116054189456L);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)30405);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)11270);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)3383);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)15591);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)54884);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)6550);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)37425);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)34382);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)30825);
            };
            GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.chan10_raw = (ushort)(ushort)64006;
            p92.chan8_raw = (ushort)(ushort)43519;
            p92.chan12_raw = (ushort)(ushort)37425;
            p92.chan3_raw = (ushort)(ushort)54884;
            p92.chan2_raw = (ushort)(ushort)30825;
            p92.chan6_raw = (ushort)(ushort)15591;
            p92.rssi = (byte)(byte)56;
            p92.chan1_raw = (ushort)(ushort)1547;
            p92.chan7_raw = (ushort)(ushort)30405;
            p92.time_usec = (ulong)3470309116054189456L;
            p92.chan4_raw = (ushort)(ushort)3383;
            p92.chan11_raw = (ushort)(ushort)6550;
            p92.chan9_raw = (ushort)(ushort)34382;
            p92.chan5_raw = (ushort)(ushort)11270;
            CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_ACTUATOR_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.controls.SequenceEqual(new float[] {1.5987079E38F, 4.932091E37F, -9.6935625E35F, 1.2561334E37F, 4.9245386E37F, 7.9247035E37F, -1.1831175E38F, 6.7429403E37F, 2.290247E38F, -3.3776005E38F, -3.1050246E38F, 1.3425459E38F, -1.5355993E38F, 2.6031674E37F, 2.7232567E38F, -8.4253986E37F}));
                Debug.Assert(pack.time_usec == (ulong)8491562110833879896L);
                Debug.Assert(pack.mode == (MAV_MODE)MAV_MODE.MAV_MODE_GUIDED_DISARMED);
                Debug.Assert(pack.flags == (ulong)2749056590405063473L);
            };
            GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.time_usec = (ulong)8491562110833879896L;
            p93.mode = (MAV_MODE)MAV_MODE.MAV_MODE_GUIDED_DISARMED;
            p93.flags = (ulong)2749056590405063473L;
            p93.controls_SET(new float[] {1.5987079E38F, 4.932091E37F, -9.6935625E35F, 1.2561334E37F, 4.9245386E37F, 7.9247035E37F, -1.1831175E38F, 6.7429403E37F, 2.290247E38F, -3.3776005E38F, -3.1050246E38F, 1.3425459E38F, -1.5355993E38F, 2.6031674E37F, 2.7232567E38F, -8.4253986E37F}, 0) ;
            CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flow_rate_y_TRY(ph) == (float)2.782017E38F);
                Debug.Assert(pack.quality == (byte)(byte)137);
                Debug.Assert(pack.time_usec == (ulong)8108495124448233351L);
                Debug.Assert(pack.flow_rate_x_TRY(ph) == (float)2.4889798E38F);
                Debug.Assert(pack.flow_y == (short)(short) -25774);
                Debug.Assert(pack.ground_distance == (float)1.0948113E38F);
                Debug.Assert(pack.flow_comp_m_x == (float)3.2277346E38F);
                Debug.Assert(pack.flow_x == (short)(short)10087);
                Debug.Assert(pack.flow_comp_m_y == (float) -4.1515443E37F);
                Debug.Assert(pack.sensor_id == (byte)(byte)160);
            };
            GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.flow_comp_m_y = (float) -4.1515443E37F;
            p100.flow_rate_x_SET((float)2.4889798E38F, PH) ;
            p100.ground_distance = (float)1.0948113E38F;
            p100.quality = (byte)(byte)137;
            p100.time_usec = (ulong)8108495124448233351L;
            p100.flow_comp_m_x = (float)3.2277346E38F;
            p100.flow_rate_y_SET((float)2.782017E38F, PH) ;
            p100.sensor_id = (byte)(byte)160;
            p100.flow_x = (short)(short)10087;
            p100.flow_y = (short)(short) -25774;
            CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_VISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll == (float)1.662295E38F);
                Debug.Assert(pack.y == (float)2.8196454E38F);
                Debug.Assert(pack.z == (float) -3.3321892E38F);
                Debug.Assert(pack.usec == (ulong)2657862213573910252L);
                Debug.Assert(pack.pitch == (float)5.2589317E37F);
                Debug.Assert(pack.x == (float)1.9387747E37F);
                Debug.Assert(pack.yaw == (float)2.0223163E38F);
            };
            GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.z = (float) -3.3321892E38F;
            p101.yaw = (float)2.0223163E38F;
            p101.x = (float)1.9387747E37F;
            p101.roll = (float)1.662295E38F;
            p101.pitch = (float)5.2589317E37F;
            p101.y = (float)2.8196454E38F;
            p101.usec = (ulong)2657862213573910252L;
            CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float) -4.2406399E37F);
                Debug.Assert(pack.z == (float) -3.1526537E38F);
                Debug.Assert(pack.roll == (float)1.2331716E38F);
                Debug.Assert(pack.yaw == (float) -8.564104E37F);
                Debug.Assert(pack.pitch == (float)9.410366E37F);
                Debug.Assert(pack.x == (float) -1.0671959E37F);
                Debug.Assert(pack.usec == (ulong)5515370280760766882L);
            };
            GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.roll = (float)1.2331716E38F;
            p102.pitch = (float)9.410366E37F;
            p102.usec = (ulong)5515370280760766882L;
            p102.x = (float) -1.0671959E37F;
            p102.y = (float) -4.2406399E37F;
            p102.yaw = (float) -8.564104E37F;
            p102.z = (float) -3.1526537E38F;
            CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_SPEED_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float) -3.3602111E38F);
                Debug.Assert(pack.usec == (ulong)7693941382435357565L);
                Debug.Assert(pack.z == (float)2.8789704E38F);
                Debug.Assert(pack.y == (float)1.2286442E38F);
            };
            GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.usec = (ulong)7693941382435357565L;
            p103.y = (float)1.2286442E38F;
            p103.x = (float) -3.3602111E38F;
            p103.z = (float)2.8789704E38F;
            CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVICON_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll == (float)3.556847E37F);
                Debug.Assert(pack.usec == (ulong)4357414138659997030L);
                Debug.Assert(pack.yaw == (float)2.4333365E38F);
                Debug.Assert(pack.y == (float)2.507664E38F);
                Debug.Assert(pack.pitch == (float)2.577779E38F);
                Debug.Assert(pack.x == (float) -5.7672646E37F);
                Debug.Assert(pack.z == (float)3.1341406E38F);
            };
            GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.usec = (ulong)4357414138659997030L;
            p104.pitch = (float)2.577779E38F;
            p104.yaw = (float)2.4333365E38F;
            p104.z = (float)3.1341406E38F;
            p104.x = (float) -5.7672646E37F;
            p104.y = (float)2.507664E38F;
            p104.roll = (float)3.556847E37F;
            CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIGHRES_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ygyro == (float)3.0073796E38F);
                Debug.Assert(pack.xgyro == (float)2.7738357E38F);
                Debug.Assert(pack.zacc == (float)1.2659515E38F);
                Debug.Assert(pack.time_usec == (ulong)7392040257943638106L);
                Debug.Assert(pack.abs_pressure == (float) -3.1210982E38F);
                Debug.Assert(pack.temperature == (float)1.8140374E38F);
                Debug.Assert(pack.zgyro == (float)1.2706405E38F);
                Debug.Assert(pack.diff_pressure == (float)1.2531193E38F);
                Debug.Assert(pack.zmag == (float) -2.0392281E38F);
                Debug.Assert(pack.xmag == (float) -2.3901868E38F);
                Debug.Assert(pack.fields_updated == (ushort)(ushort)63769);
                Debug.Assert(pack.yacc == (float) -1.1693059E38F);
                Debug.Assert(pack.xacc == (float)1.1039599E38F);
                Debug.Assert(pack.pressure_alt == (float)1.368643E37F);
                Debug.Assert(pack.ymag == (float)2.0160918E38F);
            };
            GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.zacc = (float)1.2659515E38F;
            p105.zmag = (float) -2.0392281E38F;
            p105.xgyro = (float)2.7738357E38F;
            p105.fields_updated = (ushort)(ushort)63769;
            p105.xacc = (float)1.1039599E38F;
            p105.ymag = (float)2.0160918E38F;
            p105.yacc = (float) -1.1693059E38F;
            p105.time_usec = (ulong)7392040257943638106L;
            p105.zgyro = (float)1.2706405E38F;
            p105.temperature = (float)1.8140374E38F;
            p105.ygyro = (float)3.0073796E38F;
            p105.xmag = (float) -2.3901868E38F;
            p105.abs_pressure = (float) -3.1210982E38F;
            p105.diff_pressure = (float)1.2531193E38F;
            p105.pressure_alt = (float)1.368643E37F;
            CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOW_RADReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.integrated_ygyro == (float)3.3158533E38F);
                Debug.Assert(pack.distance == (float)2.8330214E38F);
                Debug.Assert(pack.time_usec == (ulong)2191415475646617848L);
                Debug.Assert(pack.integrated_x == (float) -1.6663362E38F);
                Debug.Assert(pack.time_delta_distance_us == (uint)855266267U);
                Debug.Assert(pack.quality == (byte)(byte)42);
                Debug.Assert(pack.integrated_zgyro == (float) -2.3619306E38F);
                Debug.Assert(pack.integration_time_us == (uint)728142309U);
                Debug.Assert(pack.integrated_y == (float) -1.3979508E38F);
                Debug.Assert(pack.sensor_id == (byte)(byte)248);
                Debug.Assert(pack.integrated_xgyro == (float)1.9145208E37F);
                Debug.Assert(pack.temperature == (short)(short) -26155);
            };
            GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.quality = (byte)(byte)42;
            p106.distance = (float)2.8330214E38F;
            p106.sensor_id = (byte)(byte)248;
            p106.integrated_zgyro = (float) -2.3619306E38F;
            p106.temperature = (short)(short) -26155;
            p106.time_usec = (ulong)2191415475646617848L;
            p106.integrated_x = (float) -1.6663362E38F;
            p106.integration_time_us = (uint)728142309U;
            p106.integrated_y = (float) -1.3979508E38F;
            p106.time_delta_distance_us = (uint)855266267U;
            p106.integrated_ygyro = (float)3.3158533E38F;
            p106.integrated_xgyro = (float)1.9145208E37F;
            CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.fields_updated == (uint)2416217106U);
                Debug.Assert(pack.diff_pressure == (float)2.8347779E38F);
                Debug.Assert(pack.zmag == (float) -7.9586806E37F);
                Debug.Assert(pack.ymag == (float)5.696661E36F);
                Debug.Assert(pack.xmag == (float) -2.0780588E38F);
                Debug.Assert(pack.temperature == (float) -8.889748E37F);
                Debug.Assert(pack.xacc == (float) -8.736096E37F);
                Debug.Assert(pack.zgyro == (float) -1.3471124E38F);
                Debug.Assert(pack.time_usec == (ulong)4633060564861726052L);
                Debug.Assert(pack.pressure_alt == (float) -1.3868555E37F);
                Debug.Assert(pack.abs_pressure == (float)1.823903E38F);
                Debug.Assert(pack.xgyro == (float) -3.3443716E38F);
                Debug.Assert(pack.zacc == (float) -2.3836588E38F);
                Debug.Assert(pack.yacc == (float) -5.257167E37F);
                Debug.Assert(pack.ygyro == (float)1.3364325E38F);
            };
            GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.xacc = (float) -8.736096E37F;
            p107.fields_updated = (uint)2416217106U;
            p107.zacc = (float) -2.3836588E38F;
            p107.diff_pressure = (float)2.8347779E38F;
            p107.xmag = (float) -2.0780588E38F;
            p107.zmag = (float) -7.9586806E37F;
            p107.time_usec = (ulong)4633060564861726052L;
            p107.temperature = (float) -8.889748E37F;
            p107.abs_pressure = (float)1.823903E38F;
            p107.xgyro = (float) -3.3443716E38F;
            p107.ymag = (float)5.696661E36F;
            p107.yacc = (float) -5.257167E37F;
            p107.ygyro = (float)1.3364325E38F;
            p107.pressure_alt = (float) -1.3868555E37F;
            p107.zgyro = (float) -1.3471124E38F;
            CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSIM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (float) -8.902563E37F);
                Debug.Assert(pack.ygyro == (float)2.666572E38F);
                Debug.Assert(pack.lon == (float)1.0010504E38F);
                Debug.Assert(pack.zgyro == (float) -1.4534856E38F);
                Debug.Assert(pack.yaw == (float) -6.357457E37F);
                Debug.Assert(pack.std_dev_vert == (float)2.159703E38F);
                Debug.Assert(pack.vd == (float) -2.606195E38F);
                Debug.Assert(pack.q2 == (float) -2.6107984E38F);
                Debug.Assert(pack.pitch == (float)9.54357E37F);
                Debug.Assert(pack.q3 == (float) -1.3048154E38F);
                Debug.Assert(pack.yacc == (float)1.947439E38F);
                Debug.Assert(pack.zacc == (float)3.2152858E38F);
                Debug.Assert(pack.alt == (float)1.1689271E38F);
                Debug.Assert(pack.roll == (float) -4.792977E37F);
                Debug.Assert(pack.q4 == (float) -3.6097844E37F);
                Debug.Assert(pack.std_dev_horz == (float)2.179079E38F);
                Debug.Assert(pack.ve == (float) -1.6972131E38F);
                Debug.Assert(pack.xgyro == (float)2.102376E38F);
                Debug.Assert(pack.q1 == (float)3.3669502E37F);
                Debug.Assert(pack.xacc == (float) -2.0264512E38F);
                Debug.Assert(pack.vn == (float)2.8511855E38F);
            };
            GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.q4 = (float) -3.6097844E37F;
            p108.alt = (float)1.1689271E38F;
            p108.xgyro = (float)2.102376E38F;
            p108.roll = (float) -4.792977E37F;
            p108.lon = (float)1.0010504E38F;
            p108.yaw = (float) -6.357457E37F;
            p108.std_dev_horz = (float)2.179079E38F;
            p108.yacc = (float)1.947439E38F;
            p108.zgyro = (float) -1.4534856E38F;
            p108.vn = (float)2.8511855E38F;
            p108.std_dev_vert = (float)2.159703E38F;
            p108.xacc = (float) -2.0264512E38F;
            p108.zacc = (float)3.2152858E38F;
            p108.q3 = (float) -1.3048154E38F;
            p108.pitch = (float)9.54357E37F;
            p108.vd = (float) -2.606195E38F;
            p108.lat = (float) -8.902563E37F;
            p108.q1 = (float)3.3669502E37F;
            p108.ve = (float) -1.6972131E38F;
            p108.ygyro = (float)2.666572E38F;
            p108.q2 = (float) -2.6107984E38F;
            CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRADIO_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.remrssi == (byte)(byte)7);
                Debug.Assert(pack.rssi == (byte)(byte)150);
                Debug.Assert(pack.noise == (byte)(byte)148);
                Debug.Assert(pack.rxerrors == (ushort)(ushort)16484);
                Debug.Assert(pack.txbuf == (byte)(byte)154);
                Debug.Assert(pack.fixed_ == (ushort)(ushort)33345);
                Debug.Assert(pack.remnoise == (byte)(byte)39);
            };
            GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.txbuf = (byte)(byte)154;
            p109.noise = (byte)(byte)148;
            p109.remnoise = (byte)(byte)39;
            p109.fixed_ = (ushort)(ushort)33345;
            p109.remrssi = (byte)(byte)7;
            p109.rxerrors = (ushort)(ushort)16484;
            p109.rssi = (byte)(byte)150;
            CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnFILE_TRANSFER_PROTOCOLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)183);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)130, (byte)53, (byte)105, (byte)6, (byte)231, (byte)251, (byte)101, (byte)156, (byte)200, (byte)90, (byte)57, (byte)92, (byte)45, (byte)208, (byte)212, (byte)19, (byte)163, (byte)87, (byte)130, (byte)24, (byte)46, (byte)139, (byte)8, (byte)26, (byte)231, (byte)20, (byte)45, (byte)77, (byte)96, (byte)31, (byte)52, (byte)24, (byte)204, (byte)44, (byte)222, (byte)115, (byte)134, (byte)224, (byte)47, (byte)221, (byte)67, (byte)64, (byte)117, (byte)37, (byte)109, (byte)50, (byte)191, (byte)178, (byte)150, (byte)216, (byte)99, (byte)255, (byte)102, (byte)26, (byte)165, (byte)165, (byte)201, (byte)225, (byte)208, (byte)137, (byte)138, (byte)191, (byte)189, (byte)155, (byte)14, (byte)168, (byte)123, (byte)36, (byte)186, (byte)84, (byte)20, (byte)62, (byte)70, (byte)113, (byte)11, (byte)242, (byte)218, (byte)64, (byte)120, (byte)50, (byte)37, (byte)223, (byte)33, (byte)92, (byte)83, (byte)152, (byte)115, (byte)232, (byte)149, (byte)152, (byte)208, (byte)210, (byte)202, (byte)105, (byte)213, (byte)145, (byte)34, (byte)93, (byte)125, (byte)92, (byte)105, (byte)205, (byte)44, (byte)220, (byte)23, (byte)19, (byte)133, (byte)94, (byte)148, (byte)177, (byte)21, (byte)222, (byte)217, (byte)46, (byte)201, (byte)170, (byte)23, (byte)25, (byte)211, (byte)27, (byte)189, (byte)84, (byte)57, (byte)132, (byte)39, (byte)190, (byte)35, (byte)127, (byte)215, (byte)50, (byte)65, (byte)197, (byte)171, (byte)207, (byte)225, (byte)82, (byte)92, (byte)233, (byte)41, (byte)129, (byte)161, (byte)209, (byte)80, (byte)152, (byte)165, (byte)90, (byte)121, (byte)52, (byte)28, (byte)45, (byte)78, (byte)143, (byte)195, (byte)78, (byte)25, (byte)234, (byte)179, (byte)149, (byte)124, (byte)158, (byte)104, (byte)1, (byte)25, (byte)251, (byte)242, (byte)246, (byte)193, (byte)65, (byte)95, (byte)101, (byte)67, (byte)146, (byte)131, (byte)214, (byte)228, (byte)236, (byte)142, (byte)65, (byte)102, (byte)109, (byte)73, (byte)214, (byte)45, (byte)157, (byte)187, (byte)39, (byte)189, (byte)34, (byte)233, (byte)6, (byte)244, (byte)59, (byte)129, (byte)49, (byte)54, (byte)220, (byte)74, (byte)30, (byte)89, (byte)134, (byte)212, (byte)216, (byte)3, (byte)22, (byte)133, (byte)120, (byte)43, (byte)111, (byte)168, (byte)244, (byte)166, (byte)226, (byte)137, (byte)171, (byte)220, (byte)183, (byte)195, (byte)4, (byte)50, (byte)60, (byte)79, (byte)217, (byte)85, (byte)255, (byte)88, (byte)116, (byte)20, (byte)189, (byte)185, (byte)77, (byte)232, (byte)220, (byte)130, (byte)74, (byte)206, (byte)99, (byte)17, (byte)138, (byte)96, (byte)210, (byte)55, (byte)76, (byte)64, (byte)150, (byte)49, (byte)119, (byte)62, (byte)100, (byte)120, (byte)26, (byte)89}));
                Debug.Assert(pack.target_network == (byte)(byte)98);
                Debug.Assert(pack.target_system == (byte)(byte)153);
            };
            GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.payload_SET(new byte[] {(byte)130, (byte)53, (byte)105, (byte)6, (byte)231, (byte)251, (byte)101, (byte)156, (byte)200, (byte)90, (byte)57, (byte)92, (byte)45, (byte)208, (byte)212, (byte)19, (byte)163, (byte)87, (byte)130, (byte)24, (byte)46, (byte)139, (byte)8, (byte)26, (byte)231, (byte)20, (byte)45, (byte)77, (byte)96, (byte)31, (byte)52, (byte)24, (byte)204, (byte)44, (byte)222, (byte)115, (byte)134, (byte)224, (byte)47, (byte)221, (byte)67, (byte)64, (byte)117, (byte)37, (byte)109, (byte)50, (byte)191, (byte)178, (byte)150, (byte)216, (byte)99, (byte)255, (byte)102, (byte)26, (byte)165, (byte)165, (byte)201, (byte)225, (byte)208, (byte)137, (byte)138, (byte)191, (byte)189, (byte)155, (byte)14, (byte)168, (byte)123, (byte)36, (byte)186, (byte)84, (byte)20, (byte)62, (byte)70, (byte)113, (byte)11, (byte)242, (byte)218, (byte)64, (byte)120, (byte)50, (byte)37, (byte)223, (byte)33, (byte)92, (byte)83, (byte)152, (byte)115, (byte)232, (byte)149, (byte)152, (byte)208, (byte)210, (byte)202, (byte)105, (byte)213, (byte)145, (byte)34, (byte)93, (byte)125, (byte)92, (byte)105, (byte)205, (byte)44, (byte)220, (byte)23, (byte)19, (byte)133, (byte)94, (byte)148, (byte)177, (byte)21, (byte)222, (byte)217, (byte)46, (byte)201, (byte)170, (byte)23, (byte)25, (byte)211, (byte)27, (byte)189, (byte)84, (byte)57, (byte)132, (byte)39, (byte)190, (byte)35, (byte)127, (byte)215, (byte)50, (byte)65, (byte)197, (byte)171, (byte)207, (byte)225, (byte)82, (byte)92, (byte)233, (byte)41, (byte)129, (byte)161, (byte)209, (byte)80, (byte)152, (byte)165, (byte)90, (byte)121, (byte)52, (byte)28, (byte)45, (byte)78, (byte)143, (byte)195, (byte)78, (byte)25, (byte)234, (byte)179, (byte)149, (byte)124, (byte)158, (byte)104, (byte)1, (byte)25, (byte)251, (byte)242, (byte)246, (byte)193, (byte)65, (byte)95, (byte)101, (byte)67, (byte)146, (byte)131, (byte)214, (byte)228, (byte)236, (byte)142, (byte)65, (byte)102, (byte)109, (byte)73, (byte)214, (byte)45, (byte)157, (byte)187, (byte)39, (byte)189, (byte)34, (byte)233, (byte)6, (byte)244, (byte)59, (byte)129, (byte)49, (byte)54, (byte)220, (byte)74, (byte)30, (byte)89, (byte)134, (byte)212, (byte)216, (byte)3, (byte)22, (byte)133, (byte)120, (byte)43, (byte)111, (byte)168, (byte)244, (byte)166, (byte)226, (byte)137, (byte)171, (byte)220, (byte)183, (byte)195, (byte)4, (byte)50, (byte)60, (byte)79, (byte)217, (byte)85, (byte)255, (byte)88, (byte)116, (byte)20, (byte)189, (byte)185, (byte)77, (byte)232, (byte)220, (byte)130, (byte)74, (byte)206, (byte)99, (byte)17, (byte)138, (byte)96, (byte)210, (byte)55, (byte)76, (byte)64, (byte)150, (byte)49, (byte)119, (byte)62, (byte)100, (byte)120, (byte)26, (byte)89}, 0) ;
            p110.target_system = (byte)(byte)153;
            p110.target_component = (byte)(byte)183;
            p110.target_network = (byte)(byte)98;
            CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTIMESYNCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tc1 == (long)8988292747967462339L);
                Debug.Assert(pack.ts1 == (long) -2180437540774685732L);
            };
            GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.tc1 = (long)8988292747967462339L;
            p111.ts1 = (long) -2180437540774685732L;
            CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCAMERA_TRIGGERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (uint)1139973204U);
                Debug.Assert(pack.time_usec == (ulong)3228403988024169387L);
            };
            GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.time_usec = (ulong)3228403988024169387L;
            p112.seq = (uint)1139973204U;
            CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_GPSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.eph == (ushort)(ushort)54711);
                Debug.Assert(pack.satellites_visible == (byte)(byte)61);
                Debug.Assert(pack.lon == (int)465880365);
                Debug.Assert(pack.fix_type == (byte)(byte)85);
                Debug.Assert(pack.time_usec == (ulong)1226293309225978123L);
                Debug.Assert(pack.ve == (short)(short) -24885);
                Debug.Assert(pack.lat == (int)1223763073);
                Debug.Assert(pack.alt == (int) -1378129185);
                Debug.Assert(pack.vd == (short)(short)20357);
                Debug.Assert(pack.epv == (ushort)(ushort)38365);
                Debug.Assert(pack.vel == (ushort)(ushort)54832);
                Debug.Assert(pack.vn == (short)(short)6047);
                Debug.Assert(pack.cog == (ushort)(ushort)6891);
            };
            GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.time_usec = (ulong)1226293309225978123L;
            p113.lon = (int)465880365;
            p113.fix_type = (byte)(byte)85;
            p113.alt = (int) -1378129185;
            p113.lat = (int)1223763073;
            p113.epv = (ushort)(ushort)38365;
            p113.cog = (ushort)(ushort)6891;
            p113.vn = (short)(short)6047;
            p113.vel = (ushort)(ushort)54832;
            p113.eph = (ushort)(ushort)54711;
            p113.satellites_visible = (byte)(byte)61;
            p113.ve = (short)(short) -24885;
            p113.vd = (short)(short)20357;
            CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_OPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sensor_id == (byte)(byte)223);
                Debug.Assert(pack.quality == (byte)(byte)220);
                Debug.Assert(pack.distance == (float) -4.1458224E37F);
                Debug.Assert(pack.integrated_y == (float)2.6710331E38F);
                Debug.Assert(pack.integrated_xgyro == (float)2.46878E37F);
                Debug.Assert(pack.integrated_x == (float)1.5549004E38F);
                Debug.Assert(pack.integrated_ygyro == (float) -6.4043863E37F);
                Debug.Assert(pack.time_usec == (ulong)9041252644606173204L);
                Debug.Assert(pack.integrated_zgyro == (float) -7.3211893E37F);
                Debug.Assert(pack.time_delta_distance_us == (uint)902767130U);
                Debug.Assert(pack.integration_time_us == (uint)2180004483U);
                Debug.Assert(pack.temperature == (short)(short)4281);
            };
            GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.integrated_ygyro = (float) -6.4043863E37F;
            p114.time_delta_distance_us = (uint)902767130U;
            p114.integrated_y = (float)2.6710331E38F;
            p114.distance = (float) -4.1458224E37F;
            p114.sensor_id = (byte)(byte)223;
            p114.quality = (byte)(byte)220;
            p114.integrated_xgyro = (float)2.46878E37F;
            p114.time_usec = (ulong)9041252644606173204L;
            p114.temperature = (short)(short)4281;
            p114.integrated_x = (float)1.5549004E38F;
            p114.integration_time_us = (uint)2180004483U;
            p114.integrated_zgyro = (float) -7.3211893E37F;
            CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yawspeed == (float) -5.4781287E37F);
                Debug.Assert(pack.lon == (int) -1621841585);
                Debug.Assert(pack.zacc == (short)(short)6360);
                Debug.Assert(pack.yacc == (short)(short)4176);
                Debug.Assert(pack.rollspeed == (float) -2.2504915E38F);
                Debug.Assert(pack.vz == (short)(short) -28909);
                Debug.Assert(pack.attitude_quaternion.SequenceEqual(new float[] {2.4631052E38F, -2.5604033E38F, -7.383555E37F, -2.5972662E38F}));
                Debug.Assert(pack.alt == (int)1474238989);
                Debug.Assert(pack.ind_airspeed == (ushort)(ushort)23861);
                Debug.Assert(pack.time_usec == (ulong)1363615112472258768L);
                Debug.Assert(pack.pitchspeed == (float)2.0920143E38F);
                Debug.Assert(pack.vy == (short)(short)2930);
                Debug.Assert(pack.true_airspeed == (ushort)(ushort)29208);
                Debug.Assert(pack.xacc == (short)(short)12230);
                Debug.Assert(pack.lat == (int) -533781710);
                Debug.Assert(pack.vx == (short)(short) -2729);
            };
            GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.lon = (int) -1621841585;
            p115.pitchspeed = (float)2.0920143E38F;
            p115.vy = (short)(short)2930;
            p115.vz = (short)(short) -28909;
            p115.vx = (short)(short) -2729;
            p115.true_airspeed = (ushort)(ushort)29208;
            p115.time_usec = (ulong)1363615112472258768L;
            p115.zacc = (short)(short)6360;
            p115.alt = (int)1474238989;
            p115.ind_airspeed = (ushort)(ushort)23861;
            p115.attitude_quaternion_SET(new float[] {2.4631052E38F, -2.5604033E38F, -7.383555E37F, -2.5972662E38F}, 0) ;
            p115.lat = (int) -533781710;
            p115.xacc = (short)(short)12230;
            p115.rollspeed = (float) -2.2504915E38F;
            p115.yawspeed = (float) -5.4781287E37F;
            p115.yacc = (short)(short)4176;
            CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2629794847U);
                Debug.Assert(pack.yacc == (short)(short)1900);
                Debug.Assert(pack.zmag == (short)(short)9038);
                Debug.Assert(pack.xacc == (short)(short) -8429);
                Debug.Assert(pack.xmag == (short)(short) -20904);
                Debug.Assert(pack.ygyro == (short)(short)28059);
                Debug.Assert(pack.ymag == (short)(short) -10588);
                Debug.Assert(pack.zacc == (short)(short) -21455);
                Debug.Assert(pack.zgyro == (short)(short)4915);
                Debug.Assert(pack.xgyro == (short)(short) -14241);
            };
            GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.xgyro = (short)(short) -14241;
            p116.ygyro = (short)(short)28059;
            p116.zmag = (short)(short)9038;
            p116.xmag = (short)(short) -20904;
            p116.zgyro = (short)(short)4915;
            p116.yacc = (short)(short)1900;
            p116.xacc = (short)(short) -8429;
            p116.zacc = (short)(short) -21455;
            p116.time_boot_ms = (uint)2629794847U;
            p116.ymag = (short)(short) -10588;
            CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start == (ushort)(ushort)12021);
                Debug.Assert(pack.target_component == (byte)(byte)71);
                Debug.Assert(pack.target_system == (byte)(byte)143);
                Debug.Assert(pack.end == (ushort)(ushort)46554);
            };
            GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.start = (ushort)(ushort)12021;
            p117.target_system = (byte)(byte)143;
            p117.end = (ushort)(ushort)46554;
            p117.target_component = (byte)(byte)71;
            CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ENTRYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.last_log_num == (ushort)(ushort)2155);
                Debug.Assert(pack.num_logs == (ushort)(ushort)41026);
                Debug.Assert(pack.time_utc == (uint)1105584335U);
                Debug.Assert(pack.id == (ushort)(ushort)38679);
                Debug.Assert(pack.size == (uint)1912330653U);
            };
            GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.id = (ushort)(ushort)38679;
            p118.size = (uint)1912330653U;
            p118.time_utc = (uint)1105584335U;
            p118.last_log_num = (ushort)(ushort)2155;
            p118.num_logs = (ushort)(ushort)41026;
            CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)65);
                Debug.Assert(pack.count == (uint)4270822993U);
                Debug.Assert(pack.ofs == (uint)3056922348U);
                Debug.Assert(pack.id == (ushort)(ushort)13288);
                Debug.Assert(pack.target_component == (byte)(byte)4);
            };
            GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.id = (ushort)(ushort)13288;
            p119.ofs = (uint)3056922348U;
            p119.count = (uint)4270822993U;
            p119.target_system = (byte)(byte)65;
            p119.target_component = (byte)(byte)4;
            CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)175, (byte)39, (byte)43, (byte)225, (byte)120, (byte)41, (byte)129, (byte)224, (byte)93, (byte)19, (byte)214, (byte)152, (byte)254, (byte)61, (byte)67, (byte)164, (byte)22, (byte)141, (byte)206, (byte)163, (byte)52, (byte)210, (byte)166, (byte)118, (byte)225, (byte)250, (byte)58, (byte)30, (byte)140, (byte)30, (byte)144, (byte)115, (byte)86, (byte)138, (byte)36, (byte)67, (byte)237, (byte)133, (byte)217, (byte)161, (byte)192, (byte)107, (byte)171, (byte)115, (byte)142, (byte)81, (byte)43, (byte)237, (byte)166, (byte)109, (byte)93, (byte)85, (byte)99, (byte)132, (byte)160, (byte)10, (byte)242, (byte)173, (byte)175, (byte)190, (byte)156, (byte)176, (byte)110, (byte)75, (byte)222, (byte)25, (byte)19, (byte)136, (byte)135, (byte)43, (byte)2, (byte)46, (byte)187, (byte)172, (byte)181, (byte)136, (byte)34, (byte)130, (byte)219, (byte)66, (byte)66, (byte)39, (byte)157, (byte)18, (byte)212, (byte)124, (byte)16, (byte)207, (byte)68, (byte)18}));
                Debug.Assert(pack.ofs == (uint)1095335605U);
                Debug.Assert(pack.count == (byte)(byte)84);
                Debug.Assert(pack.id == (ushort)(ushort)50394);
            };
            GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.ofs = (uint)1095335605U;
            p120.data__SET(new byte[] {(byte)175, (byte)39, (byte)43, (byte)225, (byte)120, (byte)41, (byte)129, (byte)224, (byte)93, (byte)19, (byte)214, (byte)152, (byte)254, (byte)61, (byte)67, (byte)164, (byte)22, (byte)141, (byte)206, (byte)163, (byte)52, (byte)210, (byte)166, (byte)118, (byte)225, (byte)250, (byte)58, (byte)30, (byte)140, (byte)30, (byte)144, (byte)115, (byte)86, (byte)138, (byte)36, (byte)67, (byte)237, (byte)133, (byte)217, (byte)161, (byte)192, (byte)107, (byte)171, (byte)115, (byte)142, (byte)81, (byte)43, (byte)237, (byte)166, (byte)109, (byte)93, (byte)85, (byte)99, (byte)132, (byte)160, (byte)10, (byte)242, (byte)173, (byte)175, (byte)190, (byte)156, (byte)176, (byte)110, (byte)75, (byte)222, (byte)25, (byte)19, (byte)136, (byte)135, (byte)43, (byte)2, (byte)46, (byte)187, (byte)172, (byte)181, (byte)136, (byte)34, (byte)130, (byte)219, (byte)66, (byte)66, (byte)39, (byte)157, (byte)18, (byte)212, (byte)124, (byte)16, (byte)207, (byte)68, (byte)18}, 0) ;
            p120.count = (byte)(byte)84;
            p120.id = (ushort)(ushort)50394;
            CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ERASEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)196);
                Debug.Assert(pack.target_component == (byte)(byte)118);
            };
            GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_system = (byte)(byte)196;
            p121.target_component = (byte)(byte)118;
            CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_ENDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)43);
                Debug.Assert(pack.target_system == (byte)(byte)241);
            };
            GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_system = (byte)(byte)241;
            p122.target_component = (byte)(byte)43;
            CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_INJECT_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)49, (byte)202, (byte)133, (byte)97, (byte)186, (byte)117, (byte)153, (byte)9, (byte)188, (byte)151, (byte)250, (byte)173, (byte)231, (byte)28, (byte)37, (byte)152, (byte)145, (byte)238, (byte)50, (byte)96, (byte)31, (byte)25, (byte)245, (byte)113, (byte)167, (byte)140, (byte)75, (byte)6, (byte)21, (byte)182, (byte)8, (byte)228, (byte)48, (byte)34, (byte)219, (byte)55, (byte)53, (byte)186, (byte)203, (byte)234, (byte)98, (byte)144, (byte)118, (byte)8, (byte)121, (byte)151, (byte)185, (byte)221, (byte)225, (byte)103, (byte)70, (byte)250, (byte)148, (byte)179, (byte)109, (byte)152, (byte)246, (byte)121, (byte)160, (byte)231, (byte)53, (byte)71, (byte)28, (byte)190, (byte)114, (byte)130, (byte)192, (byte)4, (byte)26, (byte)228, (byte)54, (byte)200, (byte)141, (byte)225, (byte)89, (byte)167, (byte)60, (byte)176, (byte)157, (byte)46, (byte)238, (byte)198, (byte)154, (byte)113, (byte)209, (byte)185, (byte)191, (byte)230, (byte)154, (byte)2, (byte)48, (byte)255, (byte)10, (byte)90, (byte)88, (byte)80, (byte)226, (byte)81, (byte)212, (byte)104, (byte)5, (byte)18, (byte)139, (byte)121, (byte)114, (byte)224, (byte)115, (byte)43, (byte)68, (byte)238}));
                Debug.Assert(pack.target_system == (byte)(byte)240);
                Debug.Assert(pack.len == (byte)(byte)164);
                Debug.Assert(pack.target_component == (byte)(byte)101);
            };
            GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.len = (byte)(byte)164;
            p123.data__SET(new byte[] {(byte)49, (byte)202, (byte)133, (byte)97, (byte)186, (byte)117, (byte)153, (byte)9, (byte)188, (byte)151, (byte)250, (byte)173, (byte)231, (byte)28, (byte)37, (byte)152, (byte)145, (byte)238, (byte)50, (byte)96, (byte)31, (byte)25, (byte)245, (byte)113, (byte)167, (byte)140, (byte)75, (byte)6, (byte)21, (byte)182, (byte)8, (byte)228, (byte)48, (byte)34, (byte)219, (byte)55, (byte)53, (byte)186, (byte)203, (byte)234, (byte)98, (byte)144, (byte)118, (byte)8, (byte)121, (byte)151, (byte)185, (byte)221, (byte)225, (byte)103, (byte)70, (byte)250, (byte)148, (byte)179, (byte)109, (byte)152, (byte)246, (byte)121, (byte)160, (byte)231, (byte)53, (byte)71, (byte)28, (byte)190, (byte)114, (byte)130, (byte)192, (byte)4, (byte)26, (byte)228, (byte)54, (byte)200, (byte)141, (byte)225, (byte)89, (byte)167, (byte)60, (byte)176, (byte)157, (byte)46, (byte)238, (byte)198, (byte)154, (byte)113, (byte)209, (byte)185, (byte)191, (byte)230, (byte)154, (byte)2, (byte)48, (byte)255, (byte)10, (byte)90, (byte)88, (byte)80, (byte)226, (byte)81, (byte)212, (byte)104, (byte)5, (byte)18, (byte)139, (byte)121, (byte)114, (byte)224, (byte)115, (byte)43, (byte)68, (byte)238}, 0) ;
            p123.target_system = (byte)(byte)240;
            p123.target_component = (byte)(byte)101;
            CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vel == (ushort)(ushort)49599);
                Debug.Assert(pack.epv == (ushort)(ushort)21732);
                Debug.Assert(pack.satellites_visible == (byte)(byte)36);
                Debug.Assert(pack.time_usec == (ulong)3200725829032277220L);
                Debug.Assert(pack.fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX);
                Debug.Assert(pack.lon == (int) -745855085);
                Debug.Assert(pack.cog == (ushort)(ushort)61545);
                Debug.Assert(pack.lat == (int)288499956);
                Debug.Assert(pack.eph == (ushort)(ushort)49844);
                Debug.Assert(pack.dgps_age == (uint)2191335389U);
                Debug.Assert(pack.dgps_numch == (byte)(byte)156);
                Debug.Assert(pack.alt == (int)258308683);
            };
            GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.dgps_numch = (byte)(byte)156;
            p124.satellites_visible = (byte)(byte)36;
            p124.alt = (int)258308683;
            p124.epv = (ushort)(ushort)21732;
            p124.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX;
            p124.eph = (ushort)(ushort)49844;
            p124.lat = (int)288499956;
            p124.vel = (ushort)(ushort)49599;
            p124.dgps_age = (uint)2191335389U;
            p124.lon = (int) -745855085;
            p124.cog = (ushort)(ushort)61545;
            p124.time_usec = (ulong)3200725829032277220L;
            CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOWER_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.Vservo == (ushort)(ushort)4189);
                Debug.Assert(pack.Vcc == (ushort)(ushort)46065);
                Debug.Assert(pack.flags == (MAV_POWER_STATUS)MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED);
            };
            GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vcc = (ushort)(ushort)46065;
            p125.flags = (MAV_POWER_STATUS)MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED;
            p125.Vservo = (ushort)(ushort)4189;
            CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERIAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (SERIAL_CONTROL_FLAG)SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)124, (byte)47, (byte)135, (byte)35, (byte)103, (byte)103, (byte)7, (byte)207, (byte)246, (byte)144, (byte)105, (byte)237, (byte)114, (byte)198, (byte)208, (byte)7, (byte)141, (byte)213, (byte)187, (byte)32, (byte)101, (byte)140, (byte)157, (byte)4, (byte)116, (byte)83, (byte)75, (byte)100, (byte)146, (byte)70, (byte)60, (byte)112, (byte)149, (byte)166, (byte)113, (byte)229, (byte)203, (byte)105, (byte)79, (byte)38, (byte)49, (byte)236, (byte)245, (byte)109, (byte)59, (byte)172, (byte)51, (byte)33, (byte)69, (byte)85, (byte)0, (byte)244, (byte)7, (byte)134, (byte)205, (byte)140, (byte)16, (byte)211, (byte)39, (byte)91, (byte)97, (byte)237, (byte)17, (byte)233, (byte)176, (byte)50, (byte)152, (byte)174, (byte)248, (byte)197}));
                Debug.Assert(pack.baudrate == (uint)4033111521U);
                Debug.Assert(pack.count == (byte)(byte)249);
                Debug.Assert(pack.device == (SERIAL_CONTROL_DEV)SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2);
                Debug.Assert(pack.timeout == (ushort)(ushort)51946);
            };
            GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.baudrate = (uint)4033111521U;
            p126.data__SET(new byte[] {(byte)124, (byte)47, (byte)135, (byte)35, (byte)103, (byte)103, (byte)7, (byte)207, (byte)246, (byte)144, (byte)105, (byte)237, (byte)114, (byte)198, (byte)208, (byte)7, (byte)141, (byte)213, (byte)187, (byte)32, (byte)101, (byte)140, (byte)157, (byte)4, (byte)116, (byte)83, (byte)75, (byte)100, (byte)146, (byte)70, (byte)60, (byte)112, (byte)149, (byte)166, (byte)113, (byte)229, (byte)203, (byte)105, (byte)79, (byte)38, (byte)49, (byte)236, (byte)245, (byte)109, (byte)59, (byte)172, (byte)51, (byte)33, (byte)69, (byte)85, (byte)0, (byte)244, (byte)7, (byte)134, (byte)205, (byte)140, (byte)16, (byte)211, (byte)39, (byte)91, (byte)97, (byte)237, (byte)17, (byte)233, (byte)176, (byte)50, (byte)152, (byte)174, (byte)248, (byte)197}, 0) ;
            p126.count = (byte)(byte)249;
            p126.flags = (SERIAL_CONTROL_FLAG)SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY;
            p126.device = (SERIAL_CONTROL_DEV)SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2;
            p126.timeout = (ushort)(ushort)51946;
            CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)246);
                Debug.Assert(pack.rtk_rate == (byte)(byte)138);
                Debug.Assert(pack.baseline_c_mm == (int) -1493131966);
                Debug.Assert(pack.baseline_a_mm == (int)1641060748);
                Debug.Assert(pack.accuracy == (uint)2251854470U);
                Debug.Assert(pack.time_last_baseline_ms == (uint)3605557007U);
                Debug.Assert(pack.rtk_health == (byte)(byte)76);
                Debug.Assert(pack.wn == (ushort)(ushort)64670);
                Debug.Assert(pack.nsats == (byte)(byte)7);
                Debug.Assert(pack.baseline_b_mm == (int) -383872312);
                Debug.Assert(pack.iar_num_hypotheses == (int)1784863101);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)248);
                Debug.Assert(pack.tow == (uint)3462109369U);
            };
            GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.baseline_b_mm = (int) -383872312;
            p127.nsats = (byte)(byte)7;
            p127.wn = (ushort)(ushort)64670;
            p127.accuracy = (uint)2251854470U;
            p127.baseline_c_mm = (int) -1493131966;
            p127.baseline_coords_type = (byte)(byte)248;
            p127.tow = (uint)3462109369U;
            p127.time_last_baseline_ms = (uint)3605557007U;
            p127.rtk_receiver_id = (byte)(byte)246;
            p127.rtk_rate = (byte)(byte)138;
            p127.iar_num_hypotheses = (int)1784863101;
            p127.baseline_a_mm = (int)1641060748;
            p127.rtk_health = (byte)(byte)76;
            CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rtk_rate == (byte)(byte)171);
                Debug.Assert(pack.nsats == (byte)(byte)220);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)177);
                Debug.Assert(pack.time_last_baseline_ms == (uint)3839383150U);
                Debug.Assert(pack.accuracy == (uint)2846709553U);
                Debug.Assert(pack.baseline_c_mm == (int)970758514);
                Debug.Assert(pack.tow == (uint)3682898705U);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)11);
                Debug.Assert(pack.baseline_b_mm == (int) -1009110312);
                Debug.Assert(pack.wn == (ushort)(ushort)63608);
                Debug.Assert(pack.baseline_a_mm == (int) -1877473610);
                Debug.Assert(pack.iar_num_hypotheses == (int) -618875771);
                Debug.Assert(pack.rtk_health == (byte)(byte)149);
            };
            GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.rtk_health = (byte)(byte)149;
            p128.rtk_rate = (byte)(byte)171;
            p128.tow = (uint)3682898705U;
            p128.baseline_coords_type = (byte)(byte)177;
            p128.time_last_baseline_ms = (uint)3839383150U;
            p128.baseline_c_mm = (int)970758514;
            p128.rtk_receiver_id = (byte)(byte)11;
            p128.baseline_b_mm = (int) -1009110312;
            p128.iar_num_hypotheses = (int) -618875771;
            p128.accuracy = (uint)2846709553U;
            p128.wn = (ushort)(ushort)63608;
            p128.baseline_a_mm = (int) -1877473610;
            p128.nsats = (byte)(byte)220;
            CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zgyro == (short)(short) -7076);
                Debug.Assert(pack.zmag == (short)(short) -20470);
                Debug.Assert(pack.xgyro == (short)(short) -24144);
                Debug.Assert(pack.xmag == (short)(short) -24406);
                Debug.Assert(pack.zacc == (short)(short)7367);
                Debug.Assert(pack.ygyro == (short)(short)21383);
                Debug.Assert(pack.ymag == (short)(short) -22675);
                Debug.Assert(pack.yacc == (short)(short)9201);
                Debug.Assert(pack.time_boot_ms == (uint)4089707402U);
                Debug.Assert(pack.xacc == (short)(short) -9620);
            };
            GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.xmag = (short)(short) -24406;
            p129.zmag = (short)(short) -20470;
            p129.ygyro = (short)(short)21383;
            p129.xacc = (short)(short) -9620;
            p129.time_boot_ms = (uint)4089707402U;
            p129.ymag = (short)(short) -22675;
            p129.yacc = (short)(short)9201;
            p129.zgyro = (short)(short) -7076;
            p129.zacc = (short)(short)7367;
            p129.xgyro = (short)(short) -24144;
            CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_TRANSMISSION_HANDSHAKEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.packets == (ushort)(ushort)8555);
                Debug.Assert(pack.type == (byte)(byte)206);
                Debug.Assert(pack.size == (uint)1730770742U);
                Debug.Assert(pack.height == (ushort)(ushort)40470);
                Debug.Assert(pack.jpg_quality == (byte)(byte)19);
                Debug.Assert(pack.payload == (byte)(byte)78);
                Debug.Assert(pack.width == (ushort)(ushort)13352);
            };
            GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.packets = (ushort)(ushort)8555;
            p130.jpg_quality = (byte)(byte)19;
            p130.width = (ushort)(ushort)13352;
            p130.type = (byte)(byte)206;
            p130.height = (ushort)(ushort)40470;
            p130.payload = (byte)(byte)78;
            p130.size = (uint)1730770742U;
            CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnENCAPSULATED_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)71, (byte)255, (byte)137, (byte)210, (byte)99, (byte)146, (byte)143, (byte)154, (byte)36, (byte)251, (byte)94, (byte)21, (byte)243, (byte)252, (byte)60, (byte)4, (byte)140, (byte)69, (byte)32, (byte)105, (byte)223, (byte)57, (byte)211, (byte)171, (byte)5, (byte)22, (byte)17, (byte)171, (byte)183, (byte)72, (byte)214, (byte)163, (byte)236, (byte)88, (byte)90, (byte)8, (byte)120, (byte)5, (byte)233, (byte)168, (byte)65, (byte)208, (byte)12, (byte)234, (byte)134, (byte)167, (byte)213, (byte)110, (byte)28, (byte)36, (byte)32, (byte)131, (byte)121, (byte)87, (byte)119, (byte)130, (byte)180, (byte)26, (byte)41, (byte)209, (byte)36, (byte)80, (byte)185, (byte)187, (byte)83, (byte)111, (byte)11, (byte)120, (byte)11, (byte)216, (byte)76, (byte)224, (byte)134, (byte)196, (byte)235, (byte)151, (byte)126, (byte)51, (byte)60, (byte)19, (byte)34, (byte)245, (byte)7, (byte)217, (byte)40, (byte)82, (byte)3, (byte)83, (byte)56, (byte)138, (byte)249, (byte)211, (byte)192, (byte)0, (byte)159, (byte)105, (byte)226, (byte)16, (byte)95, (byte)77, (byte)40, (byte)149, (byte)63, (byte)122, (byte)245, (byte)230, (byte)101, (byte)232, (byte)170, (byte)106, (byte)47, (byte)255, (byte)119, (byte)19, (byte)52, (byte)210, (byte)148, (byte)222, (byte)203, (byte)148, (byte)220, (byte)17, (byte)55, (byte)46, (byte)175, (byte)70, (byte)104, (byte)73, (byte)200, (byte)220, (byte)21, (byte)158, (byte)158, (byte)111, (byte)148, (byte)250, (byte)61, (byte)232, (byte)188, (byte)135, (byte)231, (byte)194, (byte)39, (byte)19, (byte)80, (byte)146, (byte)203, (byte)181, (byte)171, (byte)176, (byte)121, (byte)46, (byte)233, (byte)233, (byte)56, (byte)250, (byte)23, (byte)178, (byte)201, (byte)203, (byte)179, (byte)119, (byte)175, (byte)72, (byte)79, (byte)22, (byte)225, (byte)60, (byte)63, (byte)230, (byte)2, (byte)113, (byte)166, (byte)90, (byte)149, (byte)12, (byte)30, (byte)39, (byte)107, (byte)56, (byte)152, (byte)179, (byte)64, (byte)21, (byte)82, (byte)230, (byte)201, (byte)232, (byte)251, (byte)244, (byte)42, (byte)126, (byte)141, (byte)217, (byte)191, (byte)125, (byte)60, (byte)77, (byte)227, (byte)119, (byte)2, (byte)73, (byte)113, (byte)240, (byte)81, (byte)111, (byte)60, (byte)193, (byte)175, (byte)201, (byte)133, (byte)50, (byte)24, (byte)227, (byte)89, (byte)3, (byte)12, (byte)6, (byte)24, (byte)239, (byte)192, (byte)240, (byte)111, (byte)148, (byte)73, (byte)37, (byte)84, (byte)85, (byte)30, (byte)201, (byte)104, (byte)168, (byte)81, (byte)66, (byte)120, (byte)105, (byte)178, (byte)8, (byte)188, (byte)237, (byte)111, (byte)151, (byte)252, (byte)244, (byte)178, (byte)202, (byte)6, (byte)252, (byte)255, (byte)54, (byte)115, (byte)206, (byte)214}));
                Debug.Assert(pack.seqnr == (ushort)(ushort)32598);
            };
            GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.data__SET(new byte[] {(byte)71, (byte)255, (byte)137, (byte)210, (byte)99, (byte)146, (byte)143, (byte)154, (byte)36, (byte)251, (byte)94, (byte)21, (byte)243, (byte)252, (byte)60, (byte)4, (byte)140, (byte)69, (byte)32, (byte)105, (byte)223, (byte)57, (byte)211, (byte)171, (byte)5, (byte)22, (byte)17, (byte)171, (byte)183, (byte)72, (byte)214, (byte)163, (byte)236, (byte)88, (byte)90, (byte)8, (byte)120, (byte)5, (byte)233, (byte)168, (byte)65, (byte)208, (byte)12, (byte)234, (byte)134, (byte)167, (byte)213, (byte)110, (byte)28, (byte)36, (byte)32, (byte)131, (byte)121, (byte)87, (byte)119, (byte)130, (byte)180, (byte)26, (byte)41, (byte)209, (byte)36, (byte)80, (byte)185, (byte)187, (byte)83, (byte)111, (byte)11, (byte)120, (byte)11, (byte)216, (byte)76, (byte)224, (byte)134, (byte)196, (byte)235, (byte)151, (byte)126, (byte)51, (byte)60, (byte)19, (byte)34, (byte)245, (byte)7, (byte)217, (byte)40, (byte)82, (byte)3, (byte)83, (byte)56, (byte)138, (byte)249, (byte)211, (byte)192, (byte)0, (byte)159, (byte)105, (byte)226, (byte)16, (byte)95, (byte)77, (byte)40, (byte)149, (byte)63, (byte)122, (byte)245, (byte)230, (byte)101, (byte)232, (byte)170, (byte)106, (byte)47, (byte)255, (byte)119, (byte)19, (byte)52, (byte)210, (byte)148, (byte)222, (byte)203, (byte)148, (byte)220, (byte)17, (byte)55, (byte)46, (byte)175, (byte)70, (byte)104, (byte)73, (byte)200, (byte)220, (byte)21, (byte)158, (byte)158, (byte)111, (byte)148, (byte)250, (byte)61, (byte)232, (byte)188, (byte)135, (byte)231, (byte)194, (byte)39, (byte)19, (byte)80, (byte)146, (byte)203, (byte)181, (byte)171, (byte)176, (byte)121, (byte)46, (byte)233, (byte)233, (byte)56, (byte)250, (byte)23, (byte)178, (byte)201, (byte)203, (byte)179, (byte)119, (byte)175, (byte)72, (byte)79, (byte)22, (byte)225, (byte)60, (byte)63, (byte)230, (byte)2, (byte)113, (byte)166, (byte)90, (byte)149, (byte)12, (byte)30, (byte)39, (byte)107, (byte)56, (byte)152, (byte)179, (byte)64, (byte)21, (byte)82, (byte)230, (byte)201, (byte)232, (byte)251, (byte)244, (byte)42, (byte)126, (byte)141, (byte)217, (byte)191, (byte)125, (byte)60, (byte)77, (byte)227, (byte)119, (byte)2, (byte)73, (byte)113, (byte)240, (byte)81, (byte)111, (byte)60, (byte)193, (byte)175, (byte)201, (byte)133, (byte)50, (byte)24, (byte)227, (byte)89, (byte)3, (byte)12, (byte)6, (byte)24, (byte)239, (byte)192, (byte)240, (byte)111, (byte)148, (byte)73, (byte)37, (byte)84, (byte)85, (byte)30, (byte)201, (byte)104, (byte)168, (byte)81, (byte)66, (byte)120, (byte)105, (byte)178, (byte)8, (byte)188, (byte)237, (byte)111, (byte)151, (byte)252, (byte)244, (byte)178, (byte)202, (byte)6, (byte)252, (byte)255, (byte)54, (byte)115, (byte)206, (byte)214}, 0) ;
            p131.seqnr = (ushort)(ushort)32598;
            CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDISTANCE_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.min_distance == (ushort)(ushort)40130);
                Debug.Assert(pack.time_boot_ms == (uint)495283656U);
                Debug.Assert(pack.orientation == (MAV_SENSOR_ORIENTATION)MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_270);
                Debug.Assert(pack.current_distance == (ushort)(ushort)34121);
                Debug.Assert(pack.type == (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND);
                Debug.Assert(pack.covariance == (byte)(byte)3);
                Debug.Assert(pack.id == (byte)(byte)27);
                Debug.Assert(pack.max_distance == (ushort)(ushort)44704);
            };
            GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.id = (byte)(byte)27;
            p132.type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND;
            p132.orientation = (MAV_SENSOR_ORIENTATION)MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_270;
            p132.max_distance = (ushort)(ushort)44704;
            p132.time_boot_ms = (uint)495283656U;
            p132.covariance = (byte)(byte)3;
            p132.min_distance = (ushort)(ushort)40130;
            p132.current_distance = (ushort)(ushort)34121;
            CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int) -1974578813);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)21590);
                Debug.Assert(pack.lat == (int) -658377820);
                Debug.Assert(pack.mask == (ulong)7414955819183801498L);
            };
            GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.lat = (int) -658377820;
            p133.grid_spacing = (ushort)(ushort)21590;
            p133.mask = (ulong)7414955819183801498L;
            p133.lon = (int) -1974578813;
            CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int)665140671);
                Debug.Assert(pack.gridbit == (byte)(byte)67);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)3838);
                Debug.Assert(pack.lat == (int)894124276);
                Debug.Assert(pack.data_.SequenceEqual(new short[] {(short) -13003, (short) -31156, (short)12778, (short)31056, (short) -30109, (short)29470, (short) -23678, (short)22216, (short) -7378, (short)7333, (short)1780, (short) -214, (short)16070, (short) -24316, (short) -15206, (short)27392}));
            };
            GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.grid_spacing = (ushort)(ushort)3838;
            p134.gridbit = (byte)(byte)67;
            p134.lat = (int)894124276;
            p134.data__SET(new short[] {(short) -13003, (short) -31156, (short)12778, (short)31056, (short) -30109, (short)29470, (short) -23678, (short)22216, (short) -7378, (short)7333, (short)1780, (short) -214, (short)16070, (short) -24316, (short) -15206, (short)27392}, 0) ;
            p134.lon = (int)665140671;
            CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_CHECKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int) -262466702);
                Debug.Assert(pack.lon == (int)1798427828);
            };
            GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lon = (int)1798427828;
            p135.lat = (int) -262466702;
            CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pending == (ushort)(ushort)15762);
                Debug.Assert(pack.terrain_height == (float) -3.2578671E38F);
                Debug.Assert(pack.lon == (int)1256689384);
                Debug.Assert(pack.spacing == (ushort)(ushort)17735);
                Debug.Assert(pack.loaded == (ushort)(ushort)31715);
                Debug.Assert(pack.lat == (int) -712674836);
                Debug.Assert(pack.current_height == (float) -2.838128E38F);
            };
            GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.pending = (ushort)(ushort)15762;
            p136.spacing = (ushort)(ushort)17735;
            p136.loaded = (ushort)(ushort)31715;
            p136.terrain_height = (float) -3.2578671E38F;
            p136.lat = (int) -712674836;
            p136.lon = (int)1256689384;
            p136.current_height = (float) -2.838128E38F;
            CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSURE2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1031789461U);
                Debug.Assert(pack.press_diff == (float) -2.5390407E38F);
                Debug.Assert(pack.temperature == (short)(short) -3879);
                Debug.Assert(pack.press_abs == (float)5.1720236E36F);
            };
            GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.temperature = (short)(short) -3879;
            p137.time_boot_ms = (uint)1031789461U;
            p137.press_abs = (float)5.1720236E36F;
            p137.press_diff = (float) -2.5390407E38F;
            CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATT_POS_MOCAPReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)6786879439167759346L);
                Debug.Assert(pack.x == (float)1.9817356E38F);
                Debug.Assert(pack.y == (float) -2.7035837E38F);
                Debug.Assert(pack.z == (float) -1.808534E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.5263855E38F, 1.7399803E38F, 2.6501297E38F, 1.8392926E37F}));
            };
            GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.y = (float) -2.7035837E38F;
            p138.q_SET(new float[] {2.5263855E38F, 1.7399803E38F, 2.6501297E38F, 1.8392926E37F}, 0) ;
            p138.z = (float) -1.808534E38F;
            p138.time_usec = (ulong)6786879439167759346L;
            p138.x = (float)1.9817356E38F;
            CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)7419376248399703254L);
                Debug.Assert(pack.group_mlx == (byte)(byte)135);
                Debug.Assert(pack.target_component == (byte)(byte)219);
                Debug.Assert(pack.target_system == (byte)(byte)66);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {5.7694166E37F, -1.4005381E38F, 1.5767806E38F, 1.7993732E38F, 2.2043028E38F, 1.8057652E38F, 2.7788276E38F, 3.0127868E38F}));
            };
            GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.controls_SET(new float[] {5.7694166E37F, -1.4005381E38F, 1.5767806E38F, 1.7993732E38F, 2.2043028E38F, 1.8057652E38F, 2.7788276E38F, 3.0127868E38F}, 0) ;
            p139.time_usec = (ulong)7419376248399703254L;
            p139.target_system = (byte)(byte)66;
            p139.group_mlx = (byte)(byte)135;
            p139.target_component = (byte)(byte)219;
            CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.group_mlx == (byte)(byte)142);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-3.1461092E38F, 1.1293323E37F, -1.1676234E38F, -6.9757023E37F, -3.2139227E37F, 2.8647642E38F, -1.9053994E37F, 2.6883851E38F}));
                Debug.Assert(pack.time_usec == (ulong)1856020590720233635L);
            };
            GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.controls_SET(new float[] {-3.1461092E38F, 1.1293323E37F, -1.1676234E38F, -6.9757023E37F, -3.2139227E37F, 2.8647642E38F, -1.9053994E37F, 2.6883851E38F}, 0) ;
            p140.time_usec = (ulong)1856020590720233635L;
            p140.group_mlx = (byte)(byte)142;
            CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnALTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.bottom_clearance == (float)1.0317103E38F);
                Debug.Assert(pack.altitude_monotonic == (float)1.2563369E38F);
                Debug.Assert(pack.altitude_local == (float)1.110193E38F);
                Debug.Assert(pack.altitude_relative == (float) -5.639167E37F);
                Debug.Assert(pack.altitude_amsl == (float)1.3817367E38F);
                Debug.Assert(pack.time_usec == (ulong)3876102118516196410L);
                Debug.Assert(pack.altitude_terrain == (float) -1.8933068E38F);
            };
            GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.bottom_clearance = (float)1.0317103E38F;
            p141.altitude_monotonic = (float)1.2563369E38F;
            p141.altitude_local = (float)1.110193E38F;
            p141.altitude_relative = (float) -5.639167E37F;
            p141.altitude_amsl = (float)1.3817367E38F;
            p141.altitude_terrain = (float) -1.8933068E38F;
            p141.time_usec = (ulong)3876102118516196410L;
            CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRESOURCE_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.storage.SequenceEqual(new byte[] {(byte)140, (byte)24, (byte)159, (byte)183, (byte)94, (byte)25, (byte)150, (byte)79, (byte)134, (byte)240, (byte)146, (byte)234, (byte)4, (byte)80, (byte)50, (byte)29, (byte)134, (byte)122, (byte)28, (byte)208, (byte)194, (byte)248, (byte)207, (byte)175, (byte)117, (byte)18, (byte)197, (byte)126, (byte)241, (byte)118, (byte)225, (byte)197, (byte)79, (byte)11, (byte)246, (byte)232, (byte)47, (byte)89, (byte)232, (byte)8, (byte)69, (byte)70, (byte)151, (byte)118, (byte)96, (byte)222, (byte)180, (byte)114, (byte)244, (byte)16, (byte)133, (byte)234, (byte)231, (byte)242, (byte)153, (byte)66, (byte)149, (byte)124, (byte)246, (byte)13, (byte)75, (byte)189, (byte)83, (byte)191, (byte)187, (byte)221, (byte)68, (byte)6, (byte)53, (byte)222, (byte)9, (byte)58, (byte)138, (byte)102, (byte)102, (byte)145, (byte)75, (byte)222, (byte)136, (byte)192, (byte)247, (byte)152, (byte)168, (byte)200, (byte)151, (byte)47, (byte)98, (byte)53, (byte)1, (byte)241, (byte)114, (byte)205, (byte)14, (byte)219, (byte)146, (byte)14, (byte)48, (byte)241, (byte)107, (byte)241, (byte)70, (byte)127, (byte)36, (byte)153, (byte)72, (byte)116, (byte)147, (byte)103, (byte)101, (byte)35, (byte)236, (byte)93, (byte)107, (byte)171, (byte)149, (byte)142, (byte)199, (byte)199, (byte)124, (byte)231}));
                Debug.Assert(pack.transfer_type == (byte)(byte)118);
                Debug.Assert(pack.uri_type == (byte)(byte)16);
                Debug.Assert(pack.uri.SequenceEqual(new byte[] {(byte)230, (byte)98, (byte)160, (byte)156, (byte)216, (byte)153, (byte)227, (byte)8, (byte)218, (byte)187, (byte)7, (byte)13, (byte)241, (byte)106, (byte)242, (byte)183, (byte)40, (byte)16, (byte)159, (byte)79, (byte)126, (byte)201, (byte)147, (byte)237, (byte)51, (byte)228, (byte)99, (byte)179, (byte)19, (byte)3, (byte)47, (byte)145, (byte)39, (byte)158, (byte)247, (byte)155, (byte)225, (byte)197, (byte)99, (byte)206, (byte)170, (byte)141, (byte)88, (byte)242, (byte)13, (byte)55, (byte)138, (byte)137, (byte)241, (byte)125, (byte)128, (byte)63, (byte)22, (byte)250, (byte)136, (byte)72, (byte)147, (byte)49, (byte)208, (byte)242, (byte)123, (byte)167, (byte)122, (byte)51, (byte)228, (byte)124, (byte)145, (byte)16, (byte)243, (byte)148, (byte)44, (byte)128, (byte)54, (byte)210, (byte)68, (byte)183, (byte)54, (byte)76, (byte)211, (byte)64, (byte)127, (byte)188, (byte)51, (byte)211, (byte)22, (byte)151, (byte)192, (byte)213, (byte)20, (byte)208, (byte)75, (byte)112, (byte)80, (byte)43, (byte)227, (byte)16, (byte)151, (byte)128, (byte)211, (byte)63, (byte)86, (byte)22, (byte)142, (byte)95, (byte)181, (byte)187, (byte)19, (byte)172, (byte)52, (byte)167, (byte)178, (byte)159, (byte)167, (byte)36, (byte)88, (byte)77, (byte)113, (byte)146, (byte)42, (byte)82}));
                Debug.Assert(pack.request_id == (byte)(byte)47);
            };
            GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.uri_type = (byte)(byte)16;
            p142.uri_SET(new byte[] {(byte)230, (byte)98, (byte)160, (byte)156, (byte)216, (byte)153, (byte)227, (byte)8, (byte)218, (byte)187, (byte)7, (byte)13, (byte)241, (byte)106, (byte)242, (byte)183, (byte)40, (byte)16, (byte)159, (byte)79, (byte)126, (byte)201, (byte)147, (byte)237, (byte)51, (byte)228, (byte)99, (byte)179, (byte)19, (byte)3, (byte)47, (byte)145, (byte)39, (byte)158, (byte)247, (byte)155, (byte)225, (byte)197, (byte)99, (byte)206, (byte)170, (byte)141, (byte)88, (byte)242, (byte)13, (byte)55, (byte)138, (byte)137, (byte)241, (byte)125, (byte)128, (byte)63, (byte)22, (byte)250, (byte)136, (byte)72, (byte)147, (byte)49, (byte)208, (byte)242, (byte)123, (byte)167, (byte)122, (byte)51, (byte)228, (byte)124, (byte)145, (byte)16, (byte)243, (byte)148, (byte)44, (byte)128, (byte)54, (byte)210, (byte)68, (byte)183, (byte)54, (byte)76, (byte)211, (byte)64, (byte)127, (byte)188, (byte)51, (byte)211, (byte)22, (byte)151, (byte)192, (byte)213, (byte)20, (byte)208, (byte)75, (byte)112, (byte)80, (byte)43, (byte)227, (byte)16, (byte)151, (byte)128, (byte)211, (byte)63, (byte)86, (byte)22, (byte)142, (byte)95, (byte)181, (byte)187, (byte)19, (byte)172, (byte)52, (byte)167, (byte)178, (byte)159, (byte)167, (byte)36, (byte)88, (byte)77, (byte)113, (byte)146, (byte)42, (byte)82}, 0) ;
            p142.request_id = (byte)(byte)47;
            p142.transfer_type = (byte)(byte)118;
            p142.storage_SET(new byte[] {(byte)140, (byte)24, (byte)159, (byte)183, (byte)94, (byte)25, (byte)150, (byte)79, (byte)134, (byte)240, (byte)146, (byte)234, (byte)4, (byte)80, (byte)50, (byte)29, (byte)134, (byte)122, (byte)28, (byte)208, (byte)194, (byte)248, (byte)207, (byte)175, (byte)117, (byte)18, (byte)197, (byte)126, (byte)241, (byte)118, (byte)225, (byte)197, (byte)79, (byte)11, (byte)246, (byte)232, (byte)47, (byte)89, (byte)232, (byte)8, (byte)69, (byte)70, (byte)151, (byte)118, (byte)96, (byte)222, (byte)180, (byte)114, (byte)244, (byte)16, (byte)133, (byte)234, (byte)231, (byte)242, (byte)153, (byte)66, (byte)149, (byte)124, (byte)246, (byte)13, (byte)75, (byte)189, (byte)83, (byte)191, (byte)187, (byte)221, (byte)68, (byte)6, (byte)53, (byte)222, (byte)9, (byte)58, (byte)138, (byte)102, (byte)102, (byte)145, (byte)75, (byte)222, (byte)136, (byte)192, (byte)247, (byte)152, (byte)168, (byte)200, (byte)151, (byte)47, (byte)98, (byte)53, (byte)1, (byte)241, (byte)114, (byte)205, (byte)14, (byte)219, (byte)146, (byte)14, (byte)48, (byte)241, (byte)107, (byte)241, (byte)70, (byte)127, (byte)36, (byte)153, (byte)72, (byte)116, (byte)147, (byte)103, (byte)101, (byte)35, (byte)236, (byte)93, (byte)107, (byte)171, (byte)149, (byte)142, (byte)199, (byte)199, (byte)124, (byte)231}, 0) ;
            CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSURE3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_diff == (float)1.0084657E38F);
                Debug.Assert(pack.temperature == (short)(short)29251);
                Debug.Assert(pack.time_boot_ms == (uint)1093559449U);
                Debug.Assert(pack.press_abs == (float) -2.9441757E38F);
            };
            GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.press_abs = (float) -2.9441757E38F;
            p143.time_boot_ms = (uint)1093559449U;
            p143.temperature = (short)(short)29251;
            p143.press_diff = (float)1.0084657E38F;
            CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnFOLLOW_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.custom_state == (ulong)6060120843334794003L);
                Debug.Assert(pack.vel.SequenceEqual(new float[] {1.1946253E38F, 1.7178743E38F, 3.3919612E38F}));
                Debug.Assert(pack.acc.SequenceEqual(new float[] {1.0389027E38F, 8.4723915E37F, -3.0184714E38F}));
                Debug.Assert(pack.est_capabilities == (byte)(byte)178);
                Debug.Assert(pack.lat == (int) -1583990439);
                Debug.Assert(pack.attitude_q.SequenceEqual(new float[] {-1.1769396E38F, 1.401164E37F, 2.9230258E37F, -2.6686686E38F}));
                Debug.Assert(pack.alt == (float) -2.5588047E38F);
                Debug.Assert(pack.rates.SequenceEqual(new float[] {2.1120077E38F, -3.2325494E38F, -1.5172558E38F}));
                Debug.Assert(pack.lon == (int) -606502706);
                Debug.Assert(pack.timestamp == (ulong)7940491533620950227L);
                Debug.Assert(pack.position_cov.SequenceEqual(new float[] {1.4431082E38F, -1.6702469E38F, 1.209334E38F}));
            };
            GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.position_cov_SET(new float[] {1.4431082E38F, -1.6702469E38F, 1.209334E38F}, 0) ;
            p144.alt = (float) -2.5588047E38F;
            p144.rates_SET(new float[] {2.1120077E38F, -3.2325494E38F, -1.5172558E38F}, 0) ;
            p144.lon = (int) -606502706;
            p144.custom_state = (ulong)6060120843334794003L;
            p144.timestamp = (ulong)7940491533620950227L;
            p144.est_capabilities = (byte)(byte)178;
            p144.acc_SET(new float[] {1.0389027E38F, 8.4723915E37F, -3.0184714E38F}, 0) ;
            p144.vel_SET(new float[] {1.1946253E38F, 1.7178743E38F, 3.3919612E38F}, 0) ;
            p144.lat = (int) -1583990439;
            p144.attitude_q_SET(new float[] {-1.1769396E38F, 1.401164E37F, 2.9230258E37F, -2.6686686E38F}, 0) ;
            CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCONTROL_SYSTEM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll_rate == (float) -1.6083798E38F);
                Debug.Assert(pack.x_acc == (float)2.5971427E38F);
                Debug.Assert(pack.x_pos == (float)3.1442467E38F);
                Debug.Assert(pack.vel_variance.SequenceEqual(new float[] {-1.2825997E36F, -2.719565E38F, 2.6784922E38F}));
                Debug.Assert(pack.yaw_rate == (float) -1.3398761E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-5.27403E37F, 1.8748863E37F, 1.6443666E38F, 2.0973792E38F}));
                Debug.Assert(pack.y_acc == (float)2.895032E36F);
                Debug.Assert(pack.pitch_rate == (float) -3.1560833E38F);
                Debug.Assert(pack.airspeed == (float) -4.5944323E37F);
                Debug.Assert(pack.z_vel == (float) -1.949573E38F);
                Debug.Assert(pack.x_vel == (float)1.9985303E38F);
                Debug.Assert(pack.y_pos == (float) -2.7263869E38F);
                Debug.Assert(pack.y_vel == (float)2.35527E38F);
                Debug.Assert(pack.time_usec == (ulong)914587738447321092L);
                Debug.Assert(pack.pos_variance.SequenceEqual(new float[] {-6.8055846E37F, -8.5762166E36F, 1.4308487E38F}));
                Debug.Assert(pack.z_acc == (float) -2.6847832E38F);
                Debug.Assert(pack.z_pos == (float) -2.3559785E38F);
            };
            GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.x_acc = (float)2.5971427E38F;
            p146.vel_variance_SET(new float[] {-1.2825997E36F, -2.719565E38F, 2.6784922E38F}, 0) ;
            p146.airspeed = (float) -4.5944323E37F;
            p146.y_acc = (float)2.895032E36F;
            p146.z_acc = (float) -2.6847832E38F;
            p146.y_pos = (float) -2.7263869E38F;
            p146.pos_variance_SET(new float[] {-6.8055846E37F, -8.5762166E36F, 1.4308487E38F}, 0) ;
            p146.pitch_rate = (float) -3.1560833E38F;
            p146.q_SET(new float[] {-5.27403E37F, 1.8748863E37F, 1.6443666E38F, 2.0973792E38F}, 0) ;
            p146.x_vel = (float)1.9985303E38F;
            p146.yaw_rate = (float) -1.3398761E38F;
            p146.time_usec = (ulong)914587738447321092L;
            p146.x_pos = (float)3.1442467E38F;
            p146.y_vel = (float)2.35527E38F;
            p146.roll_rate = (float) -1.6083798E38F;
            p146.z_pos = (float) -2.3559785E38F;
            p146.z_vel = (float) -1.949573E38F;
            CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnBATTERY_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.energy_consumed == (int) -1250333199);
                Debug.Assert(pack.id == (byte)(byte)176);
                Debug.Assert(pack.current_consumed == (int)681291484);
                Debug.Assert(pack.type == (MAV_BATTERY_TYPE)MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIPO);
                Debug.Assert(pack.temperature == (short)(short)1216);
                Debug.Assert(pack.voltages.SequenceEqual(new ushort[] {(ushort)63183, (ushort)43798, (ushort)18077, (ushort)52767, (ushort)3891, (ushort)7133, (ushort)51001, (ushort)20698, (ushort)7511, (ushort)25742}));
                Debug.Assert(pack.current_battery == (short)(short)27746);
                Debug.Assert(pack.battery_function == (MAV_BATTERY_FUNCTION)MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte)69);
            };
            GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.type = (MAV_BATTERY_TYPE)MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIPO;
            p147.current_consumed = (int)681291484;
            p147.voltages_SET(new ushort[] {(ushort)63183, (ushort)43798, (ushort)18077, (ushort)52767, (ushort)3891, (ushort)7133, (ushort)51001, (ushort)20698, (ushort)7511, (ushort)25742}, 0) ;
            p147.id = (byte)(byte)176;
            p147.battery_remaining = (sbyte)(sbyte)69;
            p147.energy_consumed = (int) -1250333199;
            p147.battery_function = (MAV_BATTERY_FUNCTION)MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD;
            p147.current_battery = (short)(short)27746;
            p147.temperature = (short)(short)1216;
            CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnAUTOPILOT_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uid == (ulong)4166363669680328701L);
                Debug.Assert(pack.flight_sw_version == (uint)3460375827U);
                Debug.Assert(pack.middleware_sw_version == (uint)658035372U);
                Debug.Assert(pack.board_version == (uint)3325526863U);
                Debug.Assert(pack.middleware_custom_version.SequenceEqual(new byte[] {(byte)82, (byte)216, (byte)192, (byte)64, (byte)33, (byte)102, (byte)178, (byte)166}));
                Debug.Assert(pack.capabilities == (MAV_PROTOCOL_CAPABILITY)MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP);
                Debug.Assert(pack.vendor_id == (ushort)(ushort)45385);
                Debug.Assert(pack.uid2_TRY(ph).SequenceEqual(new byte[] {(byte)52, (byte)182, (byte)83, (byte)73, (byte)53, (byte)92, (byte)251, (byte)66, (byte)101, (byte)100, (byte)146, (byte)248, (byte)47, (byte)105, (byte)222, (byte)93, (byte)105, (byte)30}));
                Debug.Assert(pack.flight_custom_version.SequenceEqual(new byte[] {(byte)76, (byte)153, (byte)28, (byte)0, (byte)85, (byte)52, (byte)178, (byte)110}));
                Debug.Assert(pack.os_sw_version == (uint)3520225083U);
                Debug.Assert(pack.product_id == (ushort)(ushort)41415);
                Debug.Assert(pack.os_custom_version.SequenceEqual(new byte[] {(byte)45, (byte)60, (byte)163, (byte)192, (byte)61, (byte)160, (byte)129, (byte)196}));
            };
            GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.uid = (ulong)4166363669680328701L;
            p148.uid2_SET(new byte[] {(byte)52, (byte)182, (byte)83, (byte)73, (byte)53, (byte)92, (byte)251, (byte)66, (byte)101, (byte)100, (byte)146, (byte)248, (byte)47, (byte)105, (byte)222, (byte)93, (byte)105, (byte)30}, 0, PH) ;
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY)MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP;
            p148.middleware_sw_version = (uint)658035372U;
            p148.middleware_custom_version_SET(new byte[] {(byte)82, (byte)216, (byte)192, (byte)64, (byte)33, (byte)102, (byte)178, (byte)166}, 0) ;
            p148.flight_custom_version_SET(new byte[] {(byte)76, (byte)153, (byte)28, (byte)0, (byte)85, (byte)52, (byte)178, (byte)110}, 0) ;
            p148.flight_sw_version = (uint)3460375827U;
            p148.vendor_id = (ushort)(ushort)45385;
            p148.os_sw_version = (uint)3520225083U;
            p148.product_id = (ushort)(ushort)41415;
            p148.board_version = (uint)3325526863U;
            p148.os_custom_version_SET(new byte[] {(byte)45, (byte)60, (byte)163, (byte)192, (byte)61, (byte)160, (byte)129, (byte)196}, 0) ;
            CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLANDING_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y_TRY(ph) == (float)1.112344E38F);
                Debug.Assert(pack.angle_x == (float)2.8352915E37F);
                Debug.Assert(pack.type == (LANDING_TARGET_TYPE)LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON);
                Debug.Assert(pack.frame == (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
                Debug.Assert(pack.size_y == (float)2.8898087E38F);
                Debug.Assert(pack.x_TRY(ph) == (float)1.322846E38F);
                Debug.Assert(pack.z_TRY(ph) == (float)2.5626524E38F);
                Debug.Assert(pack.time_usec == (ulong)8109372531118780291L);
                Debug.Assert(pack.distance == (float)2.011733E38F);
                Debug.Assert(pack.angle_y == (float) -1.0133423E38F);
                Debug.Assert(pack.target_num == (byte)(byte)191);
                Debug.Assert(pack.q_TRY(ph).SequenceEqual(new float[] {1.8070268E38F, 1.0498921E38F, -4.2125153E37F, -9.934871E37F}));
                Debug.Assert(pack.position_valid_TRY(ph) == (byte)(byte)179);
                Debug.Assert(pack.size_x == (float)1.6675696E38F);
            };
            GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.q_SET(new float[] {1.8070268E38F, 1.0498921E38F, -4.2125153E37F, -9.934871E37F}, 0, PH) ;
            p149.z_SET((float)2.5626524E38F, PH) ;
            p149.angle_y = (float) -1.0133423E38F;
            p149.y_SET((float)1.112344E38F, PH) ;
            p149.size_y = (float)2.8898087E38F;
            p149.type = (LANDING_TARGET_TYPE)LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON;
            p149.position_valid_SET((byte)(byte)179, PH) ;
            p149.distance = (float)2.011733E38F;
            p149.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT;
            p149.time_usec = (ulong)8109372531118780291L;
            p149.target_num = (byte)(byte)191;
            p149.size_x = (float)1.6675696E38F;
            p149.angle_x = (float)2.8352915E37F;
            p149.x_SET((float)1.322846E38F, PH) ;
            CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            ADV_TEST_CH.OnFLEXIFUNCTION_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)225);
                Debug.Assert(pack.target_component == (byte)(byte)141);
            };
            GroundControl.FLEXIFUNCTION_SET p150 = CommunicationChannel.new_FLEXIFUNCTION_SET();
            PH.setPack(p150);
            p150.target_component = (byte)(byte)141;
            p150.target_system = (byte)(byte)225;
            CommunicationChannel.instance.send(p150);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFLEXIFUNCTION_READ_REQReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)203);
                Debug.Assert(pack.data_index == (short)(short) -15276);
                Debug.Assert(pack.target_component == (byte)(byte)138);
                Debug.Assert(pack.read_req_type == (short)(short)4069);
            };
            GroundControl.FLEXIFUNCTION_READ_REQ p151 = CommunicationChannel.new_FLEXIFUNCTION_READ_REQ();
            PH.setPack(p151);
            p151.target_component = (byte)(byte)138;
            p151.data_index = (short)(short) -15276;
            p151.target_system = (byte)(byte)203;
            p151.read_req_type = (short)(short)4069;
            CommunicationChannel.instance.send(p151);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFLEXIFUNCTION_BUFFER_FUNCTIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_address == (ushort)(ushort)4791);
                Debug.Assert(pack.data_size == (ushort)(ushort)45039);
                Debug.Assert(pack.data_.SequenceEqual(new sbyte[] {(sbyte)80, (sbyte) - 28, (sbyte) - 9, (sbyte) - 21, (sbyte) - 90, (sbyte) - 81, (sbyte) - 9, (sbyte)104, (sbyte) - 40, (sbyte) - 70, (sbyte) - 36, (sbyte)118, (sbyte) - 67, (sbyte) - 38, (sbyte) - 64, (sbyte)107, (sbyte) - 73, (sbyte)64, (sbyte)2, (sbyte) - 63, (sbyte)126, (sbyte)61, (sbyte)71, (sbyte) - 9, (sbyte)103, (sbyte)85, (sbyte) - 56, (sbyte)15, (sbyte) - 26, (sbyte) - 60, (sbyte) - 68, (sbyte) - 3, (sbyte) - 118, (sbyte) - 25, (sbyte)50, (sbyte)38, (sbyte)65, (sbyte) - 112, (sbyte)113, (sbyte)70, (sbyte)64, (sbyte) - 82, (sbyte) - 113, (sbyte) - 72, (sbyte)103, (sbyte) - 29, (sbyte) - 28, (sbyte) - 108}));
                Debug.Assert(pack.func_index == (ushort)(ushort)24159);
                Debug.Assert(pack.target_component == (byte)(byte)126);
                Debug.Assert(pack.target_system == (byte)(byte)104);
                Debug.Assert(pack.func_count == (ushort)(ushort)55657);
            };
            GroundControl.FLEXIFUNCTION_BUFFER_FUNCTION p152 = CommunicationChannel.new_FLEXIFUNCTION_BUFFER_FUNCTION();
            PH.setPack(p152);
            p152.func_index = (ushort)(ushort)24159;
            p152.func_count = (ushort)(ushort)55657;
            p152.data__SET(new sbyte[] {(sbyte)80, (sbyte) - 28, (sbyte) - 9, (sbyte) - 21, (sbyte) - 90, (sbyte) - 81, (sbyte) - 9, (sbyte)104, (sbyte) - 40, (sbyte) - 70, (sbyte) - 36, (sbyte)118, (sbyte) - 67, (sbyte) - 38, (sbyte) - 64, (sbyte)107, (sbyte) - 73, (sbyte)64, (sbyte)2, (sbyte) - 63, (sbyte)126, (sbyte)61, (sbyte)71, (sbyte) - 9, (sbyte)103, (sbyte)85, (sbyte) - 56, (sbyte)15, (sbyte) - 26, (sbyte) - 60, (sbyte) - 68, (sbyte) - 3, (sbyte) - 118, (sbyte) - 25, (sbyte)50, (sbyte)38, (sbyte)65, (sbyte) - 112, (sbyte)113, (sbyte)70, (sbyte)64, (sbyte) - 82, (sbyte) - 113, (sbyte) - 72, (sbyte)103, (sbyte) - 29, (sbyte) - 28, (sbyte) - 108}, 0) ;
            p152.target_component = (byte)(byte)126;
            p152.data_address = (ushort)(ushort)4791;
            p152.target_system = (byte)(byte)104;
            p152.data_size = (ushort)(ushort)45039;
            CommunicationChannel.instance.send(p152);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFLEXIFUNCTION_BUFFER_FUNCTION_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)168);
                Debug.Assert(pack.result == (ushort)(ushort)19754);
                Debug.Assert(pack.func_index == (ushort)(ushort)63740);
                Debug.Assert(pack.target_component == (byte)(byte)209);
            };
            GroundControl.FLEXIFUNCTION_BUFFER_FUNCTION_ACK p153 = CommunicationChannel.new_FLEXIFUNCTION_BUFFER_FUNCTION_ACK();
            PH.setPack(p153);
            p153.func_index = (ushort)(ushort)63740;
            p153.target_component = (byte)(byte)209;
            p153.target_system = (byte)(byte)168;
            p153.result = (ushort)(ushort)19754;
            CommunicationChannel.instance.send(p153);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFLEXIFUNCTION_DIRECTORYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.directory_type == (byte)(byte)107);
                Debug.Assert(pack.target_component == (byte)(byte)91);
                Debug.Assert(pack.count == (byte)(byte)221);
                Debug.Assert(pack.directory_data.SequenceEqual(new sbyte[] {(sbyte)75, (sbyte)58, (sbyte)11, (sbyte) - 87, (sbyte) - 3, (sbyte)34, (sbyte) - 114, (sbyte)29, (sbyte) - 27, (sbyte) - 96, (sbyte)108, (sbyte)110, (sbyte) - 21, (sbyte) - 54, (sbyte) - 60, (sbyte)76, (sbyte)16, (sbyte)114, (sbyte) - 98, (sbyte)61, (sbyte)4, (sbyte)93, (sbyte)56, (sbyte)53, (sbyte)64, (sbyte)5, (sbyte)103, (sbyte) - 67, (sbyte) - 58, (sbyte) - 46, (sbyte) - 123, (sbyte) - 52, (sbyte) - 17, (sbyte) - 90, (sbyte) - 92, (sbyte)80, (sbyte)13, (sbyte) - 74, (sbyte)43, (sbyte)117, (sbyte) - 13, (sbyte)103, (sbyte) - 96, (sbyte) - 64, (sbyte)53, (sbyte) - 126, (sbyte)0, (sbyte)59}));
                Debug.Assert(pack.target_system == (byte)(byte)143);
                Debug.Assert(pack.start_index == (byte)(byte)129);
            };
            GroundControl.FLEXIFUNCTION_DIRECTORY p155 = CommunicationChannel.new_FLEXIFUNCTION_DIRECTORY();
            PH.setPack(p155);
            p155.start_index = (byte)(byte)129;
            p155.target_component = (byte)(byte)91;
            p155.count = (byte)(byte)221;
            p155.directory_type = (byte)(byte)107;
            p155.target_system = (byte)(byte)143;
            p155.directory_data_SET(new sbyte[] {(sbyte)75, (sbyte)58, (sbyte)11, (sbyte) - 87, (sbyte) - 3, (sbyte)34, (sbyte) - 114, (sbyte)29, (sbyte) - 27, (sbyte) - 96, (sbyte)108, (sbyte)110, (sbyte) - 21, (sbyte) - 54, (sbyte) - 60, (sbyte)76, (sbyte)16, (sbyte)114, (sbyte) - 98, (sbyte)61, (sbyte)4, (sbyte)93, (sbyte)56, (sbyte)53, (sbyte)64, (sbyte)5, (sbyte)103, (sbyte) - 67, (sbyte) - 58, (sbyte) - 46, (sbyte) - 123, (sbyte) - 52, (sbyte) - 17, (sbyte) - 90, (sbyte) - 92, (sbyte)80, (sbyte)13, (sbyte) - 74, (sbyte)43, (sbyte)117, (sbyte) - 13, (sbyte)103, (sbyte) - 96, (sbyte) - 64, (sbyte)53, (sbyte) - 126, (sbyte)0, (sbyte)59}, 0) ;
            CommunicationChannel.instance.send(p155);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFLEXIFUNCTION_DIRECTORY_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.directory_type == (byte)(byte)233);
                Debug.Assert(pack.result == (ushort)(ushort)12568);
                Debug.Assert(pack.count == (byte)(byte)227);
                Debug.Assert(pack.target_system == (byte)(byte)150);
                Debug.Assert(pack.start_index == (byte)(byte)52);
                Debug.Assert(pack.target_component == (byte)(byte)58);
            };
            GroundControl.FLEXIFUNCTION_DIRECTORY_ACK p156 = CommunicationChannel.new_FLEXIFUNCTION_DIRECTORY_ACK();
            PH.setPack(p156);
            p156.result = (ushort)(ushort)12568;
            p156.start_index = (byte)(byte)52;
            p156.directory_type = (byte)(byte)233;
            p156.target_system = (byte)(byte)150;
            p156.target_component = (byte)(byte)58;
            p156.count = (byte)(byte)227;
            CommunicationChannel.instance.send(p156);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFLEXIFUNCTION_COMMANDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)212);
                Debug.Assert(pack.target_component == (byte)(byte)167);
                Debug.Assert(pack.command_type == (byte)(byte)35);
            };
            GroundControl.FLEXIFUNCTION_COMMAND p157 = CommunicationChannel.new_FLEXIFUNCTION_COMMAND();
            PH.setPack(p157);
            p157.command_type = (byte)(byte)35;
            p157.target_component = (byte)(byte)167;
            p157.target_system = (byte)(byte)212;
            CommunicationChannel.instance.send(p157);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFLEXIFUNCTION_COMMAND_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.command_type == (ushort)(ushort)22945);
                Debug.Assert(pack.result == (ushort)(ushort)17562);
            };
            GroundControl.FLEXIFUNCTION_COMMAND_ACK p158 = CommunicationChannel.new_FLEXIFUNCTION_COMMAND_ACK();
            PH.setPack(p158);
            p158.result = (ushort)(ushort)17562;
            p158.command_type = (ushort)(ushort)22945;
            CommunicationChannel.instance.send(p158);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F2_AReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_estimated_wind_2 == (short)(short)27775);
                Debug.Assert(pack.sue_rmat0 == (short)(short) -25740);
                Debug.Assert(pack.sue_rmat4 == (short)(short) -30683);
                Debug.Assert(pack.sue_cog == (ushort)(ushort)56843);
                Debug.Assert(pack.sue_rmat2 == (short)(short) -3736);
                Debug.Assert(pack.sue_estimated_wind_0 == (short)(short)1604);
                Debug.Assert(pack.sue_longitude == (int)963780440);
                Debug.Assert(pack.sue_rmat8 == (short)(short)4478);
                Debug.Assert(pack.sue_rmat6 == (short)(short)5838);
                Debug.Assert(pack.sue_magFieldEarth2 == (short)(short)2741);
                Debug.Assert(pack.sue_hdop == (short)(short)30036);
                Debug.Assert(pack.sue_sog == (short)(short) -8066);
                Debug.Assert(pack.sue_rmat3 == (short)(short) -7327);
                Debug.Assert(pack.sue_waypoint_index == (ushort)(ushort)61195);
                Debug.Assert(pack.sue_altitude == (int)1509449390);
                Debug.Assert(pack.sue_cpu_load == (ushort)(ushort)28352);
                Debug.Assert(pack.sue_estimated_wind_1 == (short)(short) -8342);
                Debug.Assert(pack.sue_rmat5 == (short)(short)18431);
                Debug.Assert(pack.sue_rmat7 == (short)(short)7333);
                Debug.Assert(pack.sue_latitude == (int) -1084486435);
                Debug.Assert(pack.sue_air_speed_3DIMU == (ushort)(ushort)10717);
                Debug.Assert(pack.sue_status == (byte)(byte)224);
                Debug.Assert(pack.sue_time == (uint)2305555436U);
                Debug.Assert(pack.sue_rmat1 == (short)(short)15105);
                Debug.Assert(pack.sue_magFieldEarth1 == (short)(short)16086);
                Debug.Assert(pack.sue_svs == (short)(short)15875);
                Debug.Assert(pack.sue_magFieldEarth0 == (short)(short)8827);
            };
            GroundControl.SERIAL_UDB_EXTRA_F2_A p170 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F2_A();
            PH.setPack(p170);
            p170.sue_magFieldEarth1 = (short)(short)16086;
            p170.sue_rmat2 = (short)(short) -3736;
            p170.sue_magFieldEarth2 = (short)(short)2741;
            p170.sue_rmat6 = (short)(short)5838;
            p170.sue_cpu_load = (ushort)(ushort)28352;
            p170.sue_estimated_wind_0 = (short)(short)1604;
            p170.sue_rmat5 = (short)(short)18431;
            p170.sue_status = (byte)(byte)224;
            p170.sue_rmat8 = (short)(short)4478;
            p170.sue_time = (uint)2305555436U;
            p170.sue_altitude = (int)1509449390;
            p170.sue_hdop = (short)(short)30036;
            p170.sue_latitude = (int) -1084486435;
            p170.sue_rmat0 = (short)(short) -25740;
            p170.sue_estimated_wind_1 = (short)(short) -8342;
            p170.sue_sog = (short)(short) -8066;
            p170.sue_rmat3 = (short)(short) -7327;
            p170.sue_magFieldEarth0 = (short)(short)8827;
            p170.sue_rmat4 = (short)(short) -30683;
            p170.sue_rmat7 = (short)(short)7333;
            p170.sue_rmat1 = (short)(short)15105;
            p170.sue_estimated_wind_2 = (short)(short)27775;
            p170.sue_svs = (short)(short)15875;
            p170.sue_cog = (ushort)(ushort)56843;
            p170.sue_longitude = (int)963780440;
            p170.sue_waypoint_index = (ushort)(ushort)61195;
            p170.sue_air_speed_3DIMU = (ushort)(ushort)10717;
            CommunicationChannel.instance.send(p170);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F2_BReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_memory_stack_free == (short)(short) -22290);
                Debug.Assert(pack.sue_pwm_output_7 == (short)(short)30490);
                Debug.Assert(pack.sue_pwm_output_11 == (short)(short)27789);
                Debug.Assert(pack.sue_barom_alt == (int) -983775171);
                Debug.Assert(pack.sue_time == (uint)3644052959U);
                Debug.Assert(pack.sue_pwm_output_6 == (short)(short) -15076);
                Debug.Assert(pack.sue_pwm_input_1 == (short)(short) -24717);
                Debug.Assert(pack.sue_barom_temp == (short)(short) -29303);
                Debug.Assert(pack.sue_pwm_output_5 == (short)(short)10456);
                Debug.Assert(pack.sue_aero_y == (short)(short) -28355);
                Debug.Assert(pack.sue_pwm_input_2 == (short)(short) -8972);
                Debug.Assert(pack.sue_imu_location_y == (short)(short)24494);
                Debug.Assert(pack.sue_bat_amp == (short)(short)25786);
                Debug.Assert(pack.sue_pwm_output_3 == (short)(short) -27575);
                Debug.Assert(pack.sue_pwm_input_11 == (short)(short) -19973);
                Debug.Assert(pack.sue_pwm_input_12 == (short)(short) -21483);
                Debug.Assert(pack.sue_osc_fails == (short)(short) -19137);
                Debug.Assert(pack.sue_location_error_earth_y == (short)(short) -19611);
                Debug.Assert(pack.sue_pwm_output_1 == (short)(short)15441);
                Debug.Assert(pack.sue_location_error_earth_z == (short)(short) -1348);
                Debug.Assert(pack.sue_waypoint_goal_x == (short)(short)17808);
                Debug.Assert(pack.sue_pwm_output_9 == (short)(short)29271);
                Debug.Assert(pack.sue_pwm_input_6 == (short)(short)17292);
                Debug.Assert(pack.sue_waypoint_goal_z == (short)(short) -6631);
                Debug.Assert(pack.sue_pwm_input_9 == (short)(short) -7608);
                Debug.Assert(pack.sue_pwm_output_10 == (short)(short)7893);
                Debug.Assert(pack.sue_location_error_earth_x == (short)(short) -268);
                Debug.Assert(pack.sue_barom_press == (int) -1011214244);
                Debug.Assert(pack.sue_pwm_input_3 == (short)(short) -11741);
                Debug.Assert(pack.sue_pwm_output_8 == (short)(short) -25809);
                Debug.Assert(pack.sue_pwm_output_12 == (short)(short) -24013);
                Debug.Assert(pack.sue_bat_volt == (short)(short) -27942);
                Debug.Assert(pack.sue_flags == (uint)1541384575U);
                Debug.Assert(pack.sue_imu_location_z == (short)(short) -20520);
                Debug.Assert(pack.sue_waypoint_goal_y == (short)(short)7004);
                Debug.Assert(pack.sue_aero_z == (short)(short)3066);
                Debug.Assert(pack.sue_desired_height == (short)(short)20917);
                Debug.Assert(pack.sue_pwm_input_5 == (short)(short) -26467);
                Debug.Assert(pack.sue_imu_location_x == (short)(short)26766);
                Debug.Assert(pack.sue_pwm_output_4 == (short)(short) -2744);
                Debug.Assert(pack.sue_pwm_output_2 == (short)(short) -1939);
                Debug.Assert(pack.sue_imu_velocity_x == (short)(short) -3871);
                Debug.Assert(pack.sue_pwm_input_8 == (short)(short)2754);
                Debug.Assert(pack.sue_pwm_input_7 == (short)(short)4834);
                Debug.Assert(pack.sue_pwm_input_10 == (short)(short)6965);
                Debug.Assert(pack.sue_pwm_input_4 == (short)(short)24404);
                Debug.Assert(pack.sue_bat_amp_hours == (short)(short)16644);
                Debug.Assert(pack.sue_aero_x == (short)(short) -19886);
                Debug.Assert(pack.sue_imu_velocity_z == (short)(short)28886);
                Debug.Assert(pack.sue_imu_velocity_y == (short)(short)419);
            };
            GroundControl.SERIAL_UDB_EXTRA_F2_B p171 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F2_B();
            PH.setPack(p171);
            p171.sue_waypoint_goal_z = (short)(short) -6631;
            p171.sue_pwm_input_11 = (short)(short) -19973;
            p171.sue_waypoint_goal_y = (short)(short)7004;
            p171.sue_location_error_earth_x = (short)(short) -268;
            p171.sue_location_error_earth_y = (short)(short) -19611;
            p171.sue_aero_x = (short)(short) -19886;
            p171.sue_bat_amp_hours = (short)(short)16644;
            p171.sue_osc_fails = (short)(short) -19137;
            p171.sue_pwm_input_3 = (short)(short) -11741;
            p171.sue_pwm_input_9 = (short)(short) -7608;
            p171.sue_pwm_output_5 = (short)(short)10456;
            p171.sue_imu_location_z = (short)(short) -20520;
            p171.sue_imu_velocity_y = (short)(short)419;
            p171.sue_pwm_output_6 = (short)(short) -15076;
            p171.sue_pwm_output_7 = (short)(short)30490;
            p171.sue_barom_temp = (short)(short) -29303;
            p171.sue_pwm_output_9 = (short)(short)29271;
            p171.sue_barom_press = (int) -1011214244;
            p171.sue_pwm_output_4 = (short)(short) -2744;
            p171.sue_bat_amp = (short)(short)25786;
            p171.sue_pwm_output_11 = (short)(short)27789;
            p171.sue_imu_velocity_z = (short)(short)28886;
            p171.sue_imu_location_y = (short)(short)24494;
            p171.sue_desired_height = (short)(short)20917;
            p171.sue_aero_y = (short)(short) -28355;
            p171.sue_location_error_earth_z = (short)(short) -1348;
            p171.sue_pwm_input_2 = (short)(short) -8972;
            p171.sue_pwm_output_10 = (short)(short)7893;
            p171.sue_pwm_input_4 = (short)(short)24404;
            p171.sue_pwm_input_12 = (short)(short) -21483;
            p171.sue_imu_location_x = (short)(short)26766;
            p171.sue_time = (uint)3644052959U;
            p171.sue_pwm_input_6 = (short)(short)17292;
            p171.sue_pwm_output_2 = (short)(short) -1939;
            p171.sue_barom_alt = (int) -983775171;
            p171.sue_pwm_input_5 = (short)(short) -26467;
            p171.sue_pwm_output_1 = (short)(short)15441;
            p171.sue_pwm_input_8 = (short)(short)2754;
            p171.sue_memory_stack_free = (short)(short) -22290;
            p171.sue_pwm_input_1 = (short)(short) -24717;
            p171.sue_bat_volt = (short)(short) -27942;
            p171.sue_imu_velocity_x = (short)(short) -3871;
            p171.sue_aero_z = (short)(short)3066;
            p171.sue_pwm_output_3 = (short)(short) -27575;
            p171.sue_flags = (uint)1541384575U;
            p171.sue_pwm_input_7 = (short)(short)4834;
            p171.sue_pwm_input_10 = (short)(short)6965;
            p171.sue_pwm_output_8 = (short)(short) -25809;
            p171.sue_pwm_output_12 = (short)(short) -24013;
            p171.sue_waypoint_goal_x = (short)(short)17808;
            CommunicationChannel.instance.send(p171);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F4Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_ROLL_STABILIZATION_RUDDER == (byte)(byte)54);
                Debug.Assert(pack.sue_AILERON_NAVIGATION == (byte)(byte)115);
                Debug.Assert(pack.sue_ALTITUDEHOLD_WAYPOINT == (byte)(byte)223);
                Debug.Assert(pack.sue_ALTITUDEHOLD_STABILIZED == (byte)(byte)199);
                Debug.Assert(pack.sue_YAW_STABILIZATION_RUDDER == (byte)(byte)193);
                Debug.Assert(pack.sue_PITCH_STABILIZATION == (byte)(byte)174);
                Debug.Assert(pack.sue_YAW_STABILIZATION_AILERON == (byte)(byte)234);
                Debug.Assert(pack.sue_RUDDER_NAVIGATION == (byte)(byte)255);
                Debug.Assert(pack.sue_RACING_MODE == (byte)(byte)227);
                Debug.Assert(pack.sue_ROLL_STABILIZATION_AILERONS == (byte)(byte)205);
            };
            GroundControl.SERIAL_UDB_EXTRA_F4 p172 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F4();
            PH.setPack(p172);
            p172.sue_ALTITUDEHOLD_STABILIZED = (byte)(byte)199;
            p172.sue_YAW_STABILIZATION_AILERON = (byte)(byte)234;
            p172.sue_AILERON_NAVIGATION = (byte)(byte)115;
            p172.sue_ALTITUDEHOLD_WAYPOINT = (byte)(byte)223;
            p172.sue_ROLL_STABILIZATION_RUDDER = (byte)(byte)54;
            p172.sue_YAW_STABILIZATION_RUDDER = (byte)(byte)193;
            p172.sue_PITCH_STABILIZATION = (byte)(byte)174;
            p172.sue_ROLL_STABILIZATION_AILERONS = (byte)(byte)205;
            p172.sue_RUDDER_NAVIGATION = (byte)(byte)255;
            p172.sue_RACING_MODE = (byte)(byte)227;
            CommunicationChannel.instance.send(p172);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F5Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_ROLLKD == (float) -2.471453E37F);
                Debug.Assert(pack.sue_YAWKP_AILERON == (float)6.599312E37F);
                Debug.Assert(pack.sue_YAWKD_AILERON == (float) -2.0278625E38F);
                Debug.Assert(pack.sue_ROLLKP == (float)1.187663E38F);
            };
            GroundControl.SERIAL_UDB_EXTRA_F5 p173 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F5();
            PH.setPack(p173);
            p173.sue_YAWKP_AILERON = (float)6.599312E37F;
            p173.sue_YAWKD_AILERON = (float) -2.0278625E38F;
            p173.sue_ROLLKD = (float) -2.471453E37F;
            p173.sue_ROLLKP = (float)1.187663E38F;
            CommunicationChannel.instance.send(p173);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F6Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_ELEVATOR_BOOST == (float)2.8253366E38F);
                Debug.Assert(pack.sue_PITCHKD == (float)2.9934683E38F);
                Debug.Assert(pack.sue_ROLL_ELEV_MIX == (float) -1.3288395E38F);
                Debug.Assert(pack.sue_RUDDER_ELEV_MIX == (float)1.1395765E38F);
                Debug.Assert(pack.sue_PITCHGAIN == (float)2.6636412E37F);
            };
            GroundControl.SERIAL_UDB_EXTRA_F6 p174 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F6();
            PH.setPack(p174);
            p174.sue_PITCHKD = (float)2.9934683E38F;
            p174.sue_ELEVATOR_BOOST = (float)2.8253366E38F;
            p174.sue_RUDDER_ELEV_MIX = (float)1.1395765E38F;
            p174.sue_ROLL_ELEV_MIX = (float) -1.3288395E38F;
            p174.sue_PITCHGAIN = (float)2.6636412E37F;
            CommunicationChannel.instance.send(p174);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F7Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_RUDDER_BOOST == (float) -9.742181E37F);
                Debug.Assert(pack.sue_YAWKD_RUDDER == (float)1.0639569E38F);
                Debug.Assert(pack.sue_YAWKP_RUDDER == (float) -3.0477752E38F);
                Debug.Assert(pack.sue_RTL_PITCH_DOWN == (float)2.9731443E38F);
                Debug.Assert(pack.sue_ROLLKD_RUDDER == (float) -1.0229158E37F);
                Debug.Assert(pack.sue_ROLLKP_RUDDER == (float)1.2625204E38F);
            };
            GroundControl.SERIAL_UDB_EXTRA_F7 p175 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F7();
            PH.setPack(p175);
            p175.sue_RTL_PITCH_DOWN = (float)2.9731443E38F;
            p175.sue_YAWKP_RUDDER = (float) -3.0477752E38F;
            p175.sue_RUDDER_BOOST = (float) -9.742181E37F;
            p175.sue_YAWKD_RUDDER = (float)1.0639569E38F;
            p175.sue_ROLLKP_RUDDER = (float)1.2625204E38F;
            p175.sue_ROLLKD_RUDDER = (float) -1.0229158E37F;
            CommunicationChannel.instance.send(p175);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F8Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_ALT_HOLD_PITCH_MAX == (float) -1.9569994E38F);
                Debug.Assert(pack.sue_ALT_HOLD_PITCH_MIN == (float)1.2159438E38F);
                Debug.Assert(pack.sue_ALT_HOLD_THROTTLE_MAX == (float)1.3675464E38F);
                Debug.Assert(pack.sue_ALT_HOLD_THROTTLE_MIN == (float)3.1717837E38F);
                Debug.Assert(pack.sue_ALT_HOLD_PITCH_HIGH == (float) -3.2881047E38F);
                Debug.Assert(pack.sue_HEIGHT_TARGET_MIN == (float) -2.8523698E38F);
                Debug.Assert(pack.sue_HEIGHT_TARGET_MAX == (float)1.0913296E38F);
            };
            GroundControl.SERIAL_UDB_EXTRA_F8 p176 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F8();
            PH.setPack(p176);
            p176.sue_HEIGHT_TARGET_MAX = (float)1.0913296E38F;
            p176.sue_ALT_HOLD_PITCH_MIN = (float)1.2159438E38F;
            p176.sue_ALT_HOLD_PITCH_MAX = (float) -1.9569994E38F;
            p176.sue_HEIGHT_TARGET_MIN = (float) -2.8523698E38F;
            p176.sue_ALT_HOLD_PITCH_HIGH = (float) -3.2881047E38F;
            p176.sue_ALT_HOLD_THROTTLE_MIN = (float)3.1717837E38F;
            p176.sue_ALT_HOLD_THROTTLE_MAX = (float)1.3675464E38F;
            CommunicationChannel.instance.send(p176);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F13Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_lon_origin == (int)1870186485);
                Debug.Assert(pack.sue_lat_origin == (int)2052710643);
                Debug.Assert(pack.sue_alt_origin == (int)1394777124);
                Debug.Assert(pack.sue_week_no == (short)(short)21206);
            };
            GroundControl.SERIAL_UDB_EXTRA_F13 p177 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F13();
            PH.setPack(p177);
            p177.sue_week_no = (short)(short)21206;
            p177.sue_alt_origin = (int)1394777124;
            p177.sue_lat_origin = (int)2052710643;
            p177.sue_lon_origin = (int)1870186485;
            CommunicationChannel.instance.send(p177);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F14Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_BOARD_TYPE == (byte)(byte)64);
                Debug.Assert(pack.sue_GPS_TYPE == (byte)(byte)133);
                Debug.Assert(pack.sue_TRAP_SOURCE == (uint)723331444U);
                Debug.Assert(pack.sue_DR == (byte)(byte)244);
                Debug.Assert(pack.sue_AIRFRAME == (byte)(byte)167);
                Debug.Assert(pack.sue_osc_fail_count == (short)(short) -6498);
                Debug.Assert(pack.sue_FLIGHT_PLAN_TYPE == (byte)(byte)213);
                Debug.Assert(pack.sue_WIND_ESTIMATION == (byte)(byte)172);
                Debug.Assert(pack.sue_RCON == (short)(short) -3835);
                Debug.Assert(pack.sue_TRAP_FLAGS == (short)(short)28294);
                Debug.Assert(pack.sue_CLOCK_CONFIG == (byte)(byte)38);
            };
            GroundControl.SERIAL_UDB_EXTRA_F14 p178 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F14();
            PH.setPack(p178);
            p178.sue_WIND_ESTIMATION = (byte)(byte)172;
            p178.sue_CLOCK_CONFIG = (byte)(byte)38;
            p178.sue_TRAP_SOURCE = (uint)723331444U;
            p178.sue_DR = (byte)(byte)244;
            p178.sue_osc_fail_count = (short)(short) -6498;
            p178.sue_BOARD_TYPE = (byte)(byte)64;
            p178.sue_GPS_TYPE = (byte)(byte)133;
            p178.sue_FLIGHT_PLAN_TYPE = (byte)(byte)213;
            p178.sue_RCON = (short)(short) -3835;
            p178.sue_AIRFRAME = (byte)(byte)167;
            p178.sue_TRAP_FLAGS = (short)(short)28294;
            CommunicationChannel.instance.send(p178);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F15Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_ID_VEHICLE_MODEL_NAME.SequenceEqual(new byte[] {(byte)49, (byte)209, (byte)196, (byte)105, (byte)146, (byte)247, (byte)97, (byte)12, (byte)105, (byte)136, (byte)108, (byte)36, (byte)185, (byte)155, (byte)167, (byte)44, (byte)171, (byte)99, (byte)168, (byte)20, (byte)47, (byte)132, (byte)48, (byte)160, (byte)194, (byte)248, (byte)213, (byte)4, (byte)60, (byte)88, (byte)213, (byte)63, (byte)45, (byte)62, (byte)219, (byte)19, (byte)143, (byte)219, (byte)55, (byte)204}));
                Debug.Assert(pack.sue_ID_VEHICLE_REGISTRATION.SequenceEqual(new byte[] {(byte)193, (byte)15, (byte)155, (byte)171, (byte)248, (byte)142, (byte)163, (byte)98, (byte)144, (byte)86, (byte)171, (byte)214, (byte)44, (byte)34, (byte)253, (byte)223, (byte)147, (byte)247, (byte)70, (byte)239}));
            };
            GroundControl.SERIAL_UDB_EXTRA_F15 p179 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F15();
            PH.setPack(p179);
            p179.sue_ID_VEHICLE_REGISTRATION_SET(new byte[] {(byte)193, (byte)15, (byte)155, (byte)171, (byte)248, (byte)142, (byte)163, (byte)98, (byte)144, (byte)86, (byte)171, (byte)214, (byte)44, (byte)34, (byte)253, (byte)223, (byte)147, (byte)247, (byte)70, (byte)239}, 0) ;
            p179.sue_ID_VEHICLE_MODEL_NAME_SET(new byte[] {(byte)49, (byte)209, (byte)196, (byte)105, (byte)146, (byte)247, (byte)97, (byte)12, (byte)105, (byte)136, (byte)108, (byte)36, (byte)185, (byte)155, (byte)167, (byte)44, (byte)171, (byte)99, (byte)168, (byte)20, (byte)47, (byte)132, (byte)48, (byte)160, (byte)194, (byte)248, (byte)213, (byte)4, (byte)60, (byte)88, (byte)213, (byte)63, (byte)45, (byte)62, (byte)219, (byte)19, (byte)143, (byte)219, (byte)55, (byte)204}, 0) ;
            CommunicationChannel.instance.send(p179);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F16Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_ID_DIY_DRONES_URL.SequenceEqual(new byte[] {(byte)59, (byte)122, (byte)0, (byte)41, (byte)102, (byte)194, (byte)130, (byte)188, (byte)200, (byte)15, (byte)235, (byte)132, (byte)204, (byte)254, (byte)145, (byte)147, (byte)41, (byte)249, (byte)165, (byte)103, (byte)96, (byte)12, (byte)190, (byte)164, (byte)24, (byte)99, (byte)28, (byte)230, (byte)6, (byte)148, (byte)90, (byte)74, (byte)8, (byte)140, (byte)172, (byte)3, (byte)225, (byte)101, (byte)253, (byte)116, (byte)246, (byte)32, (byte)234, (byte)205, (byte)5, (byte)185, (byte)103, (byte)110, (byte)119, (byte)252, (byte)125, (byte)125, (byte)81, (byte)45, (byte)137, (byte)157, (byte)136, (byte)112, (byte)83, (byte)4, (byte)55, (byte)102, (byte)124, (byte)109, (byte)167, (byte)98, (byte)159, (byte)128, (byte)218, (byte)198}));
                Debug.Assert(pack.sue_ID_LEAD_PILOT.SequenceEqual(new byte[] {(byte)217, (byte)168, (byte)190, (byte)123, (byte)212, (byte)226, (byte)19, (byte)0, (byte)33, (byte)57, (byte)27, (byte)114, (byte)57, (byte)86, (byte)237, (byte)218, (byte)32, (byte)220, (byte)120, (byte)26, (byte)136, (byte)60, (byte)248, (byte)42, (byte)177, (byte)155, (byte)238, (byte)57, (byte)251, (byte)27, (byte)150, (byte)37, (byte)62, (byte)114, (byte)86, (byte)207, (byte)34, (byte)73, (byte)183, (byte)68}));
            };
            GroundControl.SERIAL_UDB_EXTRA_F16 p180 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F16();
            PH.setPack(p180);
            p180.sue_ID_LEAD_PILOT_SET(new byte[] {(byte)217, (byte)168, (byte)190, (byte)123, (byte)212, (byte)226, (byte)19, (byte)0, (byte)33, (byte)57, (byte)27, (byte)114, (byte)57, (byte)86, (byte)237, (byte)218, (byte)32, (byte)220, (byte)120, (byte)26, (byte)136, (byte)60, (byte)248, (byte)42, (byte)177, (byte)155, (byte)238, (byte)57, (byte)251, (byte)27, (byte)150, (byte)37, (byte)62, (byte)114, (byte)86, (byte)207, (byte)34, (byte)73, (byte)183, (byte)68}, 0) ;
            p180.sue_ID_DIY_DRONES_URL_SET(new byte[] {(byte)59, (byte)122, (byte)0, (byte)41, (byte)102, (byte)194, (byte)130, (byte)188, (byte)200, (byte)15, (byte)235, (byte)132, (byte)204, (byte)254, (byte)145, (byte)147, (byte)41, (byte)249, (byte)165, (byte)103, (byte)96, (byte)12, (byte)190, (byte)164, (byte)24, (byte)99, (byte)28, (byte)230, (byte)6, (byte)148, (byte)90, (byte)74, (byte)8, (byte)140, (byte)172, (byte)3, (byte)225, (byte)101, (byte)253, (byte)116, (byte)246, (byte)32, (byte)234, (byte)205, (byte)5, (byte)185, (byte)103, (byte)110, (byte)119, (byte)252, (byte)125, (byte)125, (byte)81, (byte)45, (byte)137, (byte)157, (byte)136, (byte)112, (byte)83, (byte)4, (byte)55, (byte)102, (byte)124, (byte)109, (byte)167, (byte)98, (byte)159, (byte)128, (byte)218, (byte)198}, 0) ;
            CommunicationChannel.instance.send(p180);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnALTITUDESReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.alt_extra == (int) -1831626846);
                Debug.Assert(pack.time_boot_ms == (uint)3665873464U);
                Debug.Assert(pack.alt_imu == (int) -903361516);
                Debug.Assert(pack.alt_range_finder == (int) -246133385);
                Debug.Assert(pack.alt_optical_flow == (int) -2136334314);
                Debug.Assert(pack.alt_gps == (int)1911654076);
                Debug.Assert(pack.alt_barometric == (int)370245116);
            };
            GroundControl.ALTITUDES p181 = CommunicationChannel.new_ALTITUDES();
            PH.setPack(p181);
            p181.alt_extra = (int) -1831626846;
            p181.alt_optical_flow = (int) -2136334314;
            p181.time_boot_ms = (uint)3665873464U;
            p181.alt_imu = (int) -903361516;
            p181.alt_barometric = (int)370245116;
            p181.alt_gps = (int)1911654076;
            p181.alt_range_finder = (int) -246133385;
            CommunicationChannel.instance.send(p181);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnAIRSPEEDSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.airspeed_ultrasonic == (short)(short)20329);
                Debug.Assert(pack.airspeed_hot_wire == (short)(short) -9273);
                Debug.Assert(pack.airspeed_imu == (short)(short) -18532);
                Debug.Assert(pack.aoy == (short)(short)12286);
                Debug.Assert(pack.aoa == (short)(short)24769);
                Debug.Assert(pack.airspeed_pitot == (short)(short)8334);
                Debug.Assert(pack.time_boot_ms == (uint)53552268U);
            };
            GroundControl.AIRSPEEDS p182 = CommunicationChannel.new_AIRSPEEDS();
            PH.setPack(p182);
            p182.aoy = (short)(short)12286;
            p182.airspeed_imu = (short)(short) -18532;
            p182.aoa = (short)(short)24769;
            p182.airspeed_hot_wire = (short)(short) -9273;
            p182.airspeed_pitot = (short)(short)8334;
            p182.airspeed_ultrasonic = (short)(short)20329;
            p182.time_boot_ms = (uint)53552268U;
            CommunicationChannel.instance.send(p182);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F17Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_turn_rate_fbw == (float)6.498484E36F);
                Debug.Assert(pack.sue_turn_rate_nav == (float) -2.5840791E37F);
                Debug.Assert(pack.sue_feed_forward == (float)2.5317104E38F);
            };
            GroundControl.SERIAL_UDB_EXTRA_F17 p183 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F17();
            PH.setPack(p183);
            p183.sue_turn_rate_nav = (float) -2.5840791E37F;
            p183.sue_turn_rate_fbw = (float)6.498484E36F;
            p183.sue_feed_forward = (float)2.5317104E38F;
            CommunicationChannel.instance.send(p183);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F18Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.angle_of_attack_inverted == (float) -2.9602682E38F);
                Debug.Assert(pack.reference_speed == (float)1.9334771E37F);
                Debug.Assert(pack.elevator_trim_normal == (float) -2.1817573E38F);
                Debug.Assert(pack.angle_of_attack_normal == (float)5.5707484E37F);
                Debug.Assert(pack.elevator_trim_inverted == (float) -1.156891E38F);
            };
            GroundControl.SERIAL_UDB_EXTRA_F18 p184 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F18();
            PH.setPack(p184);
            p184.elevator_trim_normal = (float) -2.1817573E38F;
            p184.reference_speed = (float)1.9334771E37F;
            p184.angle_of_attack_normal = (float)5.5707484E37F;
            p184.elevator_trim_inverted = (float) -1.156891E38F;
            p184.angle_of_attack_inverted = (float) -2.9602682E38F;
            CommunicationChannel.instance.send(p184);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F19Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_elevator_output_channel == (byte)(byte)73);
                Debug.Assert(pack.sue_throttle_reversed == (byte)(byte)46);
                Debug.Assert(pack.sue_throttle_output_channel == (byte)(byte)113);
                Debug.Assert(pack.sue_aileron_reversed == (byte)(byte)95);
                Debug.Assert(pack.sue_rudder_output_channel == (byte)(byte)127);
                Debug.Assert(pack.sue_aileron_output_channel == (byte)(byte)22);
                Debug.Assert(pack.sue_rudder_reversed == (byte)(byte)204);
                Debug.Assert(pack.sue_elevator_reversed == (byte)(byte)205);
            };
            GroundControl.SERIAL_UDB_EXTRA_F19 p185 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F19();
            PH.setPack(p185);
            p185.sue_rudder_reversed = (byte)(byte)204;
            p185.sue_elevator_reversed = (byte)(byte)205;
            p185.sue_aileron_reversed = (byte)(byte)95;
            p185.sue_throttle_reversed = (byte)(byte)46;
            p185.sue_aileron_output_channel = (byte)(byte)22;
            p185.sue_throttle_output_channel = (byte)(byte)113;
            p185.sue_elevator_output_channel = (byte)(byte)73;
            p185.sue_rudder_output_channel = (byte)(byte)127;
            CommunicationChannel.instance.send(p185);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F20Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_trim_value_input_9 == (short)(short)20245);
                Debug.Assert(pack.sue_trim_value_input_1 == (short)(short)7796);
                Debug.Assert(pack.sue_trim_value_input_8 == (short)(short) -10194);
                Debug.Assert(pack.sue_trim_value_input_7 == (short)(short)18078);
                Debug.Assert(pack.sue_trim_value_input_11 == (short)(short) -2015);
                Debug.Assert(pack.sue_trim_value_input_4 == (short)(short) -5122);
                Debug.Assert(pack.sue_trim_value_input_3 == (short)(short) -24489);
                Debug.Assert(pack.sue_trim_value_input_10 == (short)(short)25346);
                Debug.Assert(pack.sue_trim_value_input_6 == (short)(short)1650);
                Debug.Assert(pack.sue_trim_value_input_5 == (short)(short) -19031);
                Debug.Assert(pack.sue_trim_value_input_2 == (short)(short) -3097);
                Debug.Assert(pack.sue_trim_value_input_12 == (short)(short)10244);
                Debug.Assert(pack.sue_number_of_inputs == (byte)(byte)206);
            };
            GroundControl.SERIAL_UDB_EXTRA_F20 p186 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F20();
            PH.setPack(p186);
            p186.sue_trim_value_input_11 = (short)(short) -2015;
            p186.sue_trim_value_input_3 = (short)(short) -24489;
            p186.sue_trim_value_input_5 = (short)(short) -19031;
            p186.sue_number_of_inputs = (byte)(byte)206;
            p186.sue_trim_value_input_7 = (short)(short)18078;
            p186.sue_trim_value_input_4 = (short)(short) -5122;
            p186.sue_trim_value_input_10 = (short)(short)25346;
            p186.sue_trim_value_input_9 = (short)(short)20245;
            p186.sue_trim_value_input_6 = (short)(short)1650;
            p186.sue_trim_value_input_1 = (short)(short)7796;
            p186.sue_trim_value_input_12 = (short)(short)10244;
            p186.sue_trim_value_input_8 = (short)(short) -10194;
            p186.sue_trim_value_input_2 = (short)(short) -3097;
            CommunicationChannel.instance.send(p186);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F21Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_gyro_y_offset == (short)(short)8408);
                Debug.Assert(pack.sue_gyro_x_offset == (short)(short)487);
                Debug.Assert(pack.sue_accel_z_offset == (short)(short)29925);
                Debug.Assert(pack.sue_accel_y_offset == (short)(short)11527);
                Debug.Assert(pack.sue_accel_x_offset == (short)(short)32337);
                Debug.Assert(pack.sue_gyro_z_offset == (short)(short)19685);
            };
            GroundControl.SERIAL_UDB_EXTRA_F21 p187 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F21();
            PH.setPack(p187);
            p187.sue_accel_x_offset = (short)(short)32337;
            p187.sue_accel_y_offset = (short)(short)11527;
            p187.sue_gyro_x_offset = (short)(short)487;
            p187.sue_gyro_z_offset = (short)(short)19685;
            p187.sue_accel_z_offset = (short)(short)29925;
            p187.sue_gyro_y_offset = (short)(short)8408;
            CommunicationChannel.instance.send(p187);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSERIAL_UDB_EXTRA_F22Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sue_accel_z_at_calibration == (short)(short)20855);
                Debug.Assert(pack.sue_gyro_y_at_calibration == (short)(short)4520);
                Debug.Assert(pack.sue_accel_x_at_calibration == (short)(short)9785);
                Debug.Assert(pack.sue_gyro_z_at_calibration == (short)(short) -17402);
                Debug.Assert(pack.sue_gyro_x_at_calibration == (short)(short)12858);
                Debug.Assert(pack.sue_accel_y_at_calibration == (short)(short)3870);
            };
            GroundControl.SERIAL_UDB_EXTRA_F22 p188 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F22();
            PH.setPack(p188);
            p188.sue_gyro_z_at_calibration = (short)(short) -17402;
            p188.sue_gyro_x_at_calibration = (short)(short)12858;
            p188.sue_accel_y_at_calibration = (short)(short)3870;
            p188.sue_accel_z_at_calibration = (short)(short)20855;
            p188.sue_gyro_y_at_calibration = (short)(short)4520;
            p188.sue_accel_x_at_calibration = (short)(short)9785;
            CommunicationChannel.instance.send(p188);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            CommunicationChannel.instance.OnESTIMATOR_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (ESTIMATOR_STATUS_FLAGS)ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE);
                Debug.Assert(pack.vel_ratio == (float)1.8164001E38F);
                Debug.Assert(pack.hagl_ratio == (float) -5.651725E37F);
                Debug.Assert(pack.pos_horiz_ratio == (float) -1.836643E38F);
                Debug.Assert(pack.pos_horiz_accuracy == (float) -4.5012574E37F);
                Debug.Assert(pack.tas_ratio == (float) -2.237E38F);
                Debug.Assert(pack.time_usec == (ulong)6488948601940953346L);
                Debug.Assert(pack.mag_ratio == (float)1.733652E37F);
                Debug.Assert(pack.pos_vert_accuracy == (float)2.585532E38F);
                Debug.Assert(pack.pos_vert_ratio == (float) -2.444093E38F);
            };
            GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.pos_horiz_accuracy = (float) -4.5012574E37F;
            p230.pos_vert_ratio = (float) -2.444093E38F;
            p230.mag_ratio = (float)1.733652E37F;
            p230.vel_ratio = (float)1.8164001E38F;
            p230.hagl_ratio = (float) -5.651725E37F;
            p230.pos_horiz_ratio = (float) -1.836643E38F;
            p230.pos_vert_accuracy = (float)2.585532E38F;
            p230.time_usec = (ulong)6488948601940953346L;
            p230.flags = (ESTIMATOR_STATUS_FLAGS)ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE;
            p230.tas_ratio = (float) -2.237E38F;
            CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnWIND_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.var_horiz == (float) -1.8573158E38F);
                Debug.Assert(pack.time_usec == (ulong)6447732558420201240L);
                Debug.Assert(pack.vert_accuracy == (float) -1.2351675E38F);
                Debug.Assert(pack.wind_z == (float)1.9851068E38F);
                Debug.Assert(pack.wind_x == (float) -1.9341892E38F);
                Debug.Assert(pack.var_vert == (float) -1.4434085E38F);
                Debug.Assert(pack.wind_y == (float) -3.1635564E37F);
                Debug.Assert(pack.wind_alt == (float)9.496481E37F);
                Debug.Assert(pack.horiz_accuracy == (float) -3.8502846E37F);
            };
            GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.var_vert = (float) -1.4434085E38F;
            p231.time_usec = (ulong)6447732558420201240L;
            p231.wind_z = (float)1.9851068E38F;
            p231.var_horiz = (float) -1.8573158E38F;
            p231.wind_y = (float) -3.1635564E37F;
            p231.wind_x = (float) -1.9341892E38F;
            p231.horiz_accuracy = (float) -3.8502846E37F;
            p231.wind_alt = (float)9.496481E37F;
            p231.vert_accuracy = (float) -1.2351675E38F;
            CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_INPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int)1115393724);
                Debug.Assert(pack.time_usec == (ulong)821385782276361600L);
                Debug.Assert(pack.vdop == (float) -2.4501575E38F);
                Debug.Assert(pack.ve == (float)1.3484284E38F);
                Debug.Assert(pack.vert_accuracy == (float) -2.1563759E38F);
                Debug.Assert(pack.horiz_accuracy == (float)1.1033447E38F);
                Debug.Assert(pack.gps_id == (byte)(byte)40);
                Debug.Assert(pack.alt == (float) -2.4478406E38F);
                Debug.Assert(pack.vn == (float)1.2686845E38F);
                Debug.Assert(pack.fix_type == (byte)(byte)213);
                Debug.Assert(pack.time_week == (ushort)(ushort)49050);
                Debug.Assert(pack.time_week_ms == (uint)1575683866U);
                Debug.Assert(pack.ignore_flags == (GPS_INPUT_IGNORE_FLAGS)GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY);
                Debug.Assert(pack.speed_accuracy == (float) -2.05652E37F);
                Debug.Assert(pack.vd == (float) -2.4494902E38F);
                Debug.Assert(pack.lon == (int)985200365);
                Debug.Assert(pack.hdop == (float) -8.131653E37F);
                Debug.Assert(pack.satellites_visible == (byte)(byte)241);
            };
            GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.vd = (float) -2.4494902E38F;
            p232.vert_accuracy = (float) -2.1563759E38F;
            p232.vdop = (float) -2.4501575E38F;
            p232.ve = (float)1.3484284E38F;
            p232.alt = (float) -2.4478406E38F;
            p232.gps_id = (byte)(byte)40;
            p232.time_week = (ushort)(ushort)49050;
            p232.lat = (int)1115393724;
            p232.time_week_ms = (uint)1575683866U;
            p232.hdop = (float) -8.131653E37F;
            p232.horiz_accuracy = (float)1.1033447E38F;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS)GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY;
            p232.lon = (int)985200365;
            p232.satellites_visible = (byte)(byte)241;
            p232.speed_accuracy = (float) -2.05652E37F;
            p232.fix_type = (byte)(byte)213;
            p232.vn = (float)1.2686845E38F;
            p232.time_usec = (ulong)821385782276361600L;
            CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RTCM_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)200, (byte)180, (byte)173, (byte)112, (byte)31, (byte)104, (byte)107, (byte)142, (byte)128, (byte)243, (byte)105, (byte)81, (byte)231, (byte)192, (byte)64, (byte)112, (byte)100, (byte)157, (byte)25, (byte)9, (byte)14, (byte)253, (byte)87, (byte)203, (byte)1, (byte)164, (byte)118, (byte)180, (byte)188, (byte)198, (byte)253, (byte)13, (byte)70, (byte)197, (byte)149, (byte)56, (byte)3, (byte)216, (byte)193, (byte)36, (byte)160, (byte)58, (byte)18, (byte)162, (byte)230, (byte)205, (byte)70, (byte)4, (byte)61, (byte)79, (byte)170, (byte)74, (byte)4, (byte)35, (byte)33, (byte)174, (byte)69, (byte)210, (byte)245, (byte)40, (byte)236, (byte)127, (byte)72, (byte)213, (byte)153, (byte)78, (byte)202, (byte)96, (byte)114, (byte)149, (byte)150, (byte)247, (byte)86, (byte)43, (byte)85, (byte)62, (byte)134, (byte)197, (byte)242, (byte)7, (byte)173, (byte)238, (byte)212, (byte)71, (byte)83, (byte)47, (byte)195, (byte)208, (byte)175, (byte)202, (byte)118, (byte)172, (byte)190, (byte)198, (byte)185, (byte)67, (byte)170, (byte)143, (byte)144, (byte)73, (byte)146, (byte)219, (byte)76, (byte)115, (byte)234, (byte)41, (byte)10, (byte)112, (byte)190, (byte)227, (byte)204, (byte)47, (byte)146, (byte)97, (byte)33, (byte)171, (byte)220, (byte)232, (byte)140, (byte)51, (byte)3, (byte)143, (byte)103, (byte)181, (byte)207, (byte)250, (byte)164, (byte)71, (byte)152, (byte)175, (byte)81, (byte)119, (byte)204, (byte)164, (byte)177, (byte)178, (byte)233, (byte)251, (byte)48, (byte)206, (byte)69, (byte)145, (byte)52, (byte)254, (byte)191, (byte)64, (byte)91, (byte)103, (byte)235, (byte)254, (byte)131, (byte)254, (byte)71, (byte)178, (byte)84, (byte)223, (byte)167, (byte)251, (byte)24, (byte)146, (byte)201, (byte)194, (byte)11, (byte)157, (byte)20, (byte)184, (byte)236, (byte)246, (byte)41, (byte)89, (byte)155, (byte)130, (byte)161, (byte)16, (byte)99, (byte)109, (byte)2, (byte)215, (byte)72, (byte)23}));
                Debug.Assert(pack.len == (byte)(byte)33);
                Debug.Assert(pack.flags == (byte)(byte)169);
            };
            GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.len = (byte)(byte)33;
            p233.flags = (byte)(byte)169;
            p233.data__SET(new byte[] {(byte)200, (byte)180, (byte)173, (byte)112, (byte)31, (byte)104, (byte)107, (byte)142, (byte)128, (byte)243, (byte)105, (byte)81, (byte)231, (byte)192, (byte)64, (byte)112, (byte)100, (byte)157, (byte)25, (byte)9, (byte)14, (byte)253, (byte)87, (byte)203, (byte)1, (byte)164, (byte)118, (byte)180, (byte)188, (byte)198, (byte)253, (byte)13, (byte)70, (byte)197, (byte)149, (byte)56, (byte)3, (byte)216, (byte)193, (byte)36, (byte)160, (byte)58, (byte)18, (byte)162, (byte)230, (byte)205, (byte)70, (byte)4, (byte)61, (byte)79, (byte)170, (byte)74, (byte)4, (byte)35, (byte)33, (byte)174, (byte)69, (byte)210, (byte)245, (byte)40, (byte)236, (byte)127, (byte)72, (byte)213, (byte)153, (byte)78, (byte)202, (byte)96, (byte)114, (byte)149, (byte)150, (byte)247, (byte)86, (byte)43, (byte)85, (byte)62, (byte)134, (byte)197, (byte)242, (byte)7, (byte)173, (byte)238, (byte)212, (byte)71, (byte)83, (byte)47, (byte)195, (byte)208, (byte)175, (byte)202, (byte)118, (byte)172, (byte)190, (byte)198, (byte)185, (byte)67, (byte)170, (byte)143, (byte)144, (byte)73, (byte)146, (byte)219, (byte)76, (byte)115, (byte)234, (byte)41, (byte)10, (byte)112, (byte)190, (byte)227, (byte)204, (byte)47, (byte)146, (byte)97, (byte)33, (byte)171, (byte)220, (byte)232, (byte)140, (byte)51, (byte)3, (byte)143, (byte)103, (byte)181, (byte)207, (byte)250, (byte)164, (byte)71, (byte)152, (byte)175, (byte)81, (byte)119, (byte)204, (byte)164, (byte)177, (byte)178, (byte)233, (byte)251, (byte)48, (byte)206, (byte)69, (byte)145, (byte)52, (byte)254, (byte)191, (byte)64, (byte)91, (byte)103, (byte)235, (byte)254, (byte)131, (byte)254, (byte)71, (byte)178, (byte)84, (byte)223, (byte)167, (byte)251, (byte)24, (byte)146, (byte)201, (byte)194, (byte)11, (byte)157, (byte)20, (byte)184, (byte)236, (byte)246, (byte)41, (byte)89, (byte)155, (byte)130, (byte)161, (byte)16, (byte)99, (byte)109, (byte)2, (byte)215, (byte)72, (byte)23}, 0) ;
            CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIGH_LATENCYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.wp_distance == (ushort)(ushort)59209);
                Debug.Assert(pack.longitude == (int)347798945);
                Debug.Assert(pack.pitch == (short)(short) -805);
                Debug.Assert(pack.temperature_air == (sbyte)(sbyte)49);
                Debug.Assert(pack.gps_nsat == (byte)(byte)181);
                Debug.Assert(pack.latitude == (int) -902262338);
                Debug.Assert(pack.groundspeed == (byte)(byte)31);
                Debug.Assert(pack.climb_rate == (sbyte)(sbyte) - 93);
                Debug.Assert(pack.heading_sp == (short)(short) -23583);
                Debug.Assert(pack.airspeed == (byte)(byte)238);
                Debug.Assert(pack.roll == (short)(short)10493);
                Debug.Assert(pack.battery_remaining == (byte)(byte)122);
                Debug.Assert(pack.altitude_amsl == (short)(short)3419);
                Debug.Assert(pack.heading == (ushort)(ushort)10958);
                Debug.Assert(pack.throttle == (sbyte)(sbyte)26);
                Debug.Assert(pack.landed_state == (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND);
                Debug.Assert(pack.gps_fix_type == (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX);
                Debug.Assert(pack.temperature == (sbyte)(sbyte)118);
                Debug.Assert(pack.altitude_sp == (short)(short) -6399);
                Debug.Assert(pack.failsafe == (byte)(byte)74);
                Debug.Assert(pack.wp_num == (byte)(byte)133);
                Debug.Assert(pack.airspeed_sp == (byte)(byte)254);
                Debug.Assert(pack.custom_mode == (uint)1239930719U);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED);
            };
            GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.pitch = (short)(short) -805;
            p234.heading_sp = (short)(short) -23583;
            p234.failsafe = (byte)(byte)74;
            p234.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
            p234.groundspeed = (byte)(byte)31;
            p234.climb_rate = (sbyte)(sbyte) - 93;
            p234.altitude_sp = (short)(short) -6399;
            p234.wp_distance = (ushort)(ushort)59209;
            p234.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND;
            p234.airspeed = (byte)(byte)238;
            p234.longitude = (int)347798945;
            p234.roll = (short)(short)10493;
            p234.battery_remaining = (byte)(byte)122;
            p234.gps_nsat = (byte)(byte)181;
            p234.throttle = (sbyte)(sbyte)26;
            p234.temperature = (sbyte)(sbyte)118;
            p234.custom_mode = (uint)1239930719U;
            p234.latitude = (int) -902262338;
            p234.altitude_amsl = (short)(short)3419;
            p234.wp_num = (byte)(byte)133;
            p234.heading = (ushort)(ushort)10958;
            p234.gps_fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX;
            p234.temperature_air = (sbyte)(sbyte)49;
            p234.airspeed_sp = (byte)(byte)254;
            CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVIBRATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vibration_z == (float)2.3368881E38F);
                Debug.Assert(pack.vibration_x == (float) -1.0691211E38F);
                Debug.Assert(pack.vibration_y == (float)5.766746E37F);
                Debug.Assert(pack.time_usec == (ulong)6109059512953625207L);
                Debug.Assert(pack.clipping_0 == (uint)935887476U);
                Debug.Assert(pack.clipping_2 == (uint)1611162624U);
                Debug.Assert(pack.clipping_1 == (uint)767571079U);
            };
            GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.time_usec = (ulong)6109059512953625207L;
            p241.vibration_z = (float)2.3368881E38F;
            p241.vibration_y = (float)5.766746E37F;
            p241.clipping_0 = (uint)935887476U;
            p241.clipping_2 = (uint)1611162624U;
            p241.clipping_1 = (uint)767571079U;
            p241.vibration_x = (float) -1.0691211E38F;
            CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.longitude == (int)1089380695);
                Debug.Assert(pack.altitude == (int)2003417212);
                Debug.Assert(pack.y == (float) -9.854403E37F);
                Debug.Assert(pack.approach_z == (float) -2.8544948E38F);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)6336304004693764702L);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.1042979E38F, 1.5222661E38F, 4.5253387E37F, -3.3426086E38F}));
                Debug.Assert(pack.approach_y == (float) -1.6122623E38F);
                Debug.Assert(pack.latitude == (int)663763909);
                Debug.Assert(pack.z == (float)8.983522E37F);
                Debug.Assert(pack.approach_x == (float) -7.5363197E37F);
                Debug.Assert(pack.x == (float)2.8953811E38F);
            };
            GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.y = (float) -9.854403E37F;
            p242.q_SET(new float[] {-1.1042979E38F, 1.5222661E38F, 4.5253387E37F, -3.3426086E38F}, 0) ;
            p242.approach_x = (float) -7.5363197E37F;
            p242.approach_y = (float) -1.6122623E38F;
            p242.altitude = (int)2003417212;
            p242.x = (float)2.8953811E38F;
            p242.latitude = (int)663763909;
            p242.z = (float)8.983522E37F;
            p242.time_usec_SET((ulong)6336304004693764702L, PH) ;
            p242.approach_z = (float) -2.8544948E38F;
            p242.longitude = (int)1089380695;
            CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            ADV_TEST_CH.OnSET_HOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.approach_y == (float)2.3281626E38F);
                Debug.Assert(pack.altitude == (int) -1678998875);
                Debug.Assert(pack.target_system == (byte)(byte)182);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.3788173E38F, 1.0356489E38F, 2.9007125E38F, -1.7819223E38F}));
                Debug.Assert(pack.longitude == (int) -1688150493);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)6390492147915988544L);
                Debug.Assert(pack.z == (float) -2.2486039E38F);
                Debug.Assert(pack.y == (float)1.5591818E38F);
                Debug.Assert(pack.latitude == (int)637060048);
                Debug.Assert(pack.approach_x == (float) -1.3602677E38F);
                Debug.Assert(pack.x == (float) -1.5428708E38F);
                Debug.Assert(pack.approach_z == (float)3.3658405E37F);
            };
            GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.target_system = (byte)(byte)182;
            p243.time_usec_SET((ulong)6390492147915988544L, PH) ;
            p243.q_SET(new float[] {-2.3788173E38F, 1.0356489E38F, 2.9007125E38F, -1.7819223E38F}, 0) ;
            p243.latitude = (int)637060048;
            p243.x = (float) -1.5428708E38F;
            p243.y = (float)1.5591818E38F;
            p243.approach_x = (float) -1.3602677E38F;
            p243.altitude = (int) -1678998875;
            p243.approach_y = (float)2.3281626E38F;
            p243.approach_z = (float)3.3658405E37F;
            p243.longitude = (int) -1688150493;
            p243.z = (float) -2.2486039E38F;
            CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnMESSAGE_INTERVALReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.message_id == (ushort)(ushort)31129);
                Debug.Assert(pack.interval_us == (int) -1261147387);
            };
            GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.interval_us = (int) -1261147387;
            p244.message_id = (ushort)(ushort)31129;
            CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnEXTENDED_SYS_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vtol_state == (MAV_VTOL_STATE)MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_FW);
                Debug.Assert(pack.landed_state == (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF);
            };
            GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.vtol_state = (MAV_VTOL_STATE)MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_FW;
            p245.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF;
            CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnADSB_VEHICLEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.callsign_LEN(ph) == 3);
                Debug.Assert(pack.callsign_TRY(ph).Equals("lmz"));
                Debug.Assert(pack.ICAO_address == (uint)627571590U);
                Debug.Assert(pack.hor_velocity == (ushort)(ushort)6176);
                Debug.Assert(pack.altitude_type == (ADSB_ALTITUDE_TYPE)ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
                Debug.Assert(pack.lat == (int)806713636);
                Debug.Assert(pack.ver_velocity == (short)(short) -3593);
                Debug.Assert(pack.squawk == (ushort)(ushort)7191);
                Debug.Assert(pack.flags == (ADSB_FLAGS)ADSB_FLAGS.ADSB_FLAGS_SIMULATED);
                Debug.Assert(pack.tslc == (byte)(byte)6);
                Debug.Assert(pack.emitter_type == (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_SPACE);
                Debug.Assert(pack.heading == (ushort)(ushort)17145);
                Debug.Assert(pack.lon == (int)1583091649);
                Debug.Assert(pack.altitude == (int)1252631308);
            };
            GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.emitter_type = (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_SPACE;
            p246.flags = (ADSB_FLAGS)ADSB_FLAGS.ADSB_FLAGS_SIMULATED;
            p246.altitude_type = (ADSB_ALTITUDE_TYPE)ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
            p246.callsign_SET("lmz", PH) ;
            p246.tslc = (byte)(byte)6;
            p246.hor_velocity = (ushort)(ushort)6176;
            p246.ver_velocity = (short)(short) -3593;
            p246.lat = (int)806713636;
            p246.lon = (int)1583091649;
            p246.heading = (ushort)(ushort)17145;
            p246.squawk = (ushort)(ushort)7191;
            p246.altitude = (int)1252631308;
            p246.ICAO_address = (uint)627571590U;
            CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCOLLISIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.threat_level == (MAV_COLLISION_THREAT_LEVEL)MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH);
                Debug.Assert(pack.src_ == (MAV_COLLISION_SRC)MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
                Debug.Assert(pack.time_to_minimum_delta == (float)3.1381259E38F);
                Debug.Assert(pack.action == (MAV_COLLISION_ACTION)MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_REPORT);
                Debug.Assert(pack.id == (uint)3264209447U);
                Debug.Assert(pack.altitude_minimum_delta == (float) -2.4503925E38F);
                Debug.Assert(pack.horizontal_minimum_delta == (float)1.7604754E38F);
            };
            GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
            PH.setPack(p247);
            p247.horizontal_minimum_delta = (float)1.7604754E38F;
            p247.id = (uint)3264209447U;
            p247.action = (MAV_COLLISION_ACTION)MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_REPORT;
            p247.src_ = (MAV_COLLISION_SRC)MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB;
            p247.threat_level = (MAV_COLLISION_THREAT_LEVEL)MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH;
            p247.time_to_minimum_delta = (float)3.1381259E38F;
            p247.altitude_minimum_delta = (float) -2.4503925E38F;
            CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnV2_EXTENSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_network == (byte)(byte)56);
                Debug.Assert(pack.target_system == (byte)(byte)227);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)50, (byte)99, (byte)18, (byte)164, (byte)207, (byte)69, (byte)109, (byte)153, (byte)165, (byte)244, (byte)122, (byte)225, (byte)140, (byte)214, (byte)67, (byte)215, (byte)53, (byte)100, (byte)244, (byte)42, (byte)73, (byte)180, (byte)119, (byte)8, (byte)142, (byte)4, (byte)76, (byte)123, (byte)172, (byte)94, (byte)246, (byte)101, (byte)115, (byte)166, (byte)210, (byte)113, (byte)123, (byte)19, (byte)83, (byte)191, (byte)135, (byte)55, (byte)137, (byte)8, (byte)173, (byte)27, (byte)72, (byte)0, (byte)209, (byte)16, (byte)118, (byte)30, (byte)117, (byte)58, (byte)22, (byte)46, (byte)86, (byte)210, (byte)168, (byte)207, (byte)3, (byte)108, (byte)136, (byte)80, (byte)85, (byte)77, (byte)28, (byte)167, (byte)114, (byte)206, (byte)15, (byte)101, (byte)161, (byte)113, (byte)88, (byte)46, (byte)228, (byte)221, (byte)214, (byte)152, (byte)93, (byte)227, (byte)235, (byte)118, (byte)140, (byte)21, (byte)45, (byte)232, (byte)222, (byte)20, (byte)232, (byte)153, (byte)233, (byte)202, (byte)140, (byte)212, (byte)237, (byte)142, (byte)200, (byte)159, (byte)154, (byte)19, (byte)35, (byte)226, (byte)234, (byte)206, (byte)217, (byte)246, (byte)119, (byte)226, (byte)126, (byte)248, (byte)1, (byte)185, (byte)147, (byte)179, (byte)210, (byte)31, (byte)206, (byte)127, (byte)43, (byte)81, (byte)206, (byte)92, (byte)240, (byte)146, (byte)74, (byte)107, (byte)160, (byte)122, (byte)80, (byte)158, (byte)14, (byte)81, (byte)171, (byte)211, (byte)193, (byte)106, (byte)106, (byte)167, (byte)169, (byte)153, (byte)107, (byte)169, (byte)143, (byte)161, (byte)230, (byte)140, (byte)233, (byte)39, (byte)34, (byte)81, (byte)198, (byte)109, (byte)232, (byte)66, (byte)127, (byte)81, (byte)243, (byte)147, (byte)114, (byte)138, (byte)227, (byte)248, (byte)246, (byte)153, (byte)150, (byte)198, (byte)222, (byte)108, (byte)70, (byte)203, (byte)141, (byte)110, (byte)227, (byte)234, (byte)36, (byte)56, (byte)34, (byte)48, (byte)129, (byte)168, (byte)212, (byte)5, (byte)221, (byte)161, (byte)106, (byte)246, (byte)193, (byte)223, (byte)34, (byte)104, (byte)10, (byte)56, (byte)214, (byte)117, (byte)125, (byte)74, (byte)77, (byte)20, (byte)132, (byte)99, (byte)55, (byte)177, (byte)254, (byte)105, (byte)113, (byte)159, (byte)5, (byte)170, (byte)230, (byte)150, (byte)240, (byte)225, (byte)16, (byte)149, (byte)67, (byte)6, (byte)109, (byte)161, (byte)198, (byte)107, (byte)37, (byte)169, (byte)92, (byte)110, (byte)57, (byte)215, (byte)64, (byte)40, (byte)213, (byte)192, (byte)44, (byte)165, (byte)59, (byte)91, (byte)246, (byte)100, (byte)98, (byte)57, (byte)161, (byte)63, (byte)177, (byte)213, (byte)133, (byte)76, (byte)8, (byte)118, (byte)220}));
                Debug.Assert(pack.message_type == (ushort)(ushort)44815);
                Debug.Assert(pack.target_component == (byte)(byte)13);
            };
            GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.target_network = (byte)(byte)56;
            p248.target_system = (byte)(byte)227;
            p248.target_component = (byte)(byte)13;
            p248.message_type = (ushort)(ushort)44815;
            p248.payload_SET(new byte[] {(byte)50, (byte)99, (byte)18, (byte)164, (byte)207, (byte)69, (byte)109, (byte)153, (byte)165, (byte)244, (byte)122, (byte)225, (byte)140, (byte)214, (byte)67, (byte)215, (byte)53, (byte)100, (byte)244, (byte)42, (byte)73, (byte)180, (byte)119, (byte)8, (byte)142, (byte)4, (byte)76, (byte)123, (byte)172, (byte)94, (byte)246, (byte)101, (byte)115, (byte)166, (byte)210, (byte)113, (byte)123, (byte)19, (byte)83, (byte)191, (byte)135, (byte)55, (byte)137, (byte)8, (byte)173, (byte)27, (byte)72, (byte)0, (byte)209, (byte)16, (byte)118, (byte)30, (byte)117, (byte)58, (byte)22, (byte)46, (byte)86, (byte)210, (byte)168, (byte)207, (byte)3, (byte)108, (byte)136, (byte)80, (byte)85, (byte)77, (byte)28, (byte)167, (byte)114, (byte)206, (byte)15, (byte)101, (byte)161, (byte)113, (byte)88, (byte)46, (byte)228, (byte)221, (byte)214, (byte)152, (byte)93, (byte)227, (byte)235, (byte)118, (byte)140, (byte)21, (byte)45, (byte)232, (byte)222, (byte)20, (byte)232, (byte)153, (byte)233, (byte)202, (byte)140, (byte)212, (byte)237, (byte)142, (byte)200, (byte)159, (byte)154, (byte)19, (byte)35, (byte)226, (byte)234, (byte)206, (byte)217, (byte)246, (byte)119, (byte)226, (byte)126, (byte)248, (byte)1, (byte)185, (byte)147, (byte)179, (byte)210, (byte)31, (byte)206, (byte)127, (byte)43, (byte)81, (byte)206, (byte)92, (byte)240, (byte)146, (byte)74, (byte)107, (byte)160, (byte)122, (byte)80, (byte)158, (byte)14, (byte)81, (byte)171, (byte)211, (byte)193, (byte)106, (byte)106, (byte)167, (byte)169, (byte)153, (byte)107, (byte)169, (byte)143, (byte)161, (byte)230, (byte)140, (byte)233, (byte)39, (byte)34, (byte)81, (byte)198, (byte)109, (byte)232, (byte)66, (byte)127, (byte)81, (byte)243, (byte)147, (byte)114, (byte)138, (byte)227, (byte)248, (byte)246, (byte)153, (byte)150, (byte)198, (byte)222, (byte)108, (byte)70, (byte)203, (byte)141, (byte)110, (byte)227, (byte)234, (byte)36, (byte)56, (byte)34, (byte)48, (byte)129, (byte)168, (byte)212, (byte)5, (byte)221, (byte)161, (byte)106, (byte)246, (byte)193, (byte)223, (byte)34, (byte)104, (byte)10, (byte)56, (byte)214, (byte)117, (byte)125, (byte)74, (byte)77, (byte)20, (byte)132, (byte)99, (byte)55, (byte)177, (byte)254, (byte)105, (byte)113, (byte)159, (byte)5, (byte)170, (byte)230, (byte)150, (byte)240, (byte)225, (byte)16, (byte)149, (byte)67, (byte)6, (byte)109, (byte)161, (byte)198, (byte)107, (byte)37, (byte)169, (byte)92, (byte)110, (byte)57, (byte)215, (byte)64, (byte)40, (byte)213, (byte)192, (byte)44, (byte)165, (byte)59, (byte)91, (byte)246, (byte)100, (byte)98, (byte)57, (byte)161, (byte)63, (byte)177, (byte)213, (byte)133, (byte)76, (byte)8, (byte)118, (byte)220}, 0) ;
            CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnMEMORY_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type == (byte)(byte)113);
                Debug.Assert(pack.value.SequenceEqual(new sbyte[] {(sbyte)12, (sbyte) - 57, (sbyte) - 32, (sbyte) - 108, (sbyte)39, (sbyte) - 79, (sbyte)79, (sbyte) - 13, (sbyte) - 74, (sbyte) - 40, (sbyte)113, (sbyte) - 31, (sbyte)112, (sbyte)33, (sbyte)41, (sbyte) - 34, (sbyte)43, (sbyte)48, (sbyte)23, (sbyte)83, (sbyte) - 87, (sbyte) - 82, (sbyte) - 51, (sbyte)121, (sbyte) - 50, (sbyte) - 5, (sbyte) - 70, (sbyte)69, (sbyte) - 27, (sbyte) - 50, (sbyte) - 103, (sbyte)98}));
                Debug.Assert(pack.ver == (byte)(byte)196);
                Debug.Assert(pack.address == (ushort)(ushort)13649);
            };
            GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.ver = (byte)(byte)196;
            p249.address = (ushort)(ushort)13649;
            p249.type = (byte)(byte)113;
            p249.value_SET(new sbyte[] {(sbyte)12, (sbyte) - 57, (sbyte) - 32, (sbyte) - 108, (sbyte)39, (sbyte) - 79, (sbyte)79, (sbyte) - 13, (sbyte) - 74, (sbyte) - 40, (sbyte)113, (sbyte) - 31, (sbyte)112, (sbyte)33, (sbyte)41, (sbyte) - 34, (sbyte)43, (sbyte)48, (sbyte)23, (sbyte)83, (sbyte) - 87, (sbyte) - 82, (sbyte) - 51, (sbyte)121, (sbyte) - 50, (sbyte) - 5, (sbyte) - 70, (sbyte)69, (sbyte) - 27, (sbyte) - 50, (sbyte) - 103, (sbyte)98}, 0) ;
            CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnDEBUG_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float) -3.0466526E38F);
                Debug.Assert(pack.y == (float)4.3357864E37F);
                Debug.Assert(pack.x == (float)3.3481098E38F);
                Debug.Assert(pack.time_usec == (ulong)7801406492427327715L);
                Debug.Assert(pack.name_LEN(ph) == 10);
                Debug.Assert(pack.name_TRY(ph).Equals("qqmifdhgfp"));
            };
            GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.x = (float)3.3481098E38F;
            p250.y = (float)4.3357864E37F;
            p250.time_usec = (ulong)7801406492427327715L;
            p250.name_SET("qqmifdhgfp", PH) ;
            p250.z = (float) -3.0466526E38F;
            CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_FLOATReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.name_LEN(ph) == 8);
                Debug.Assert(pack.name_TRY(ph).Equals("jkrysyfg"));
                Debug.Assert(pack.time_boot_ms == (uint)76372951U);
                Debug.Assert(pack.value == (float) -6.237105E37F);
            };
            GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.time_boot_ms = (uint)76372951U;
            p251.name_SET("jkrysyfg", PH) ;
            p251.value = (float) -6.237105E37F;
            CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value == (int)1730063345);
                Debug.Assert(pack.name_LEN(ph) == 3);
                Debug.Assert(pack.name_TRY(ph).Equals("cWg"));
                Debug.Assert(pack.time_boot_ms == (uint)597893602U);
            };
            GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.value = (int)1730063345;
            p252.time_boot_ms = (uint)597893602U;
            p252.name_SET("cWg", PH) ;
            CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSTATUSTEXTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.text_LEN(ph) == 46);
                Debug.Assert(pack.text_TRY(ph).Equals("ojzojEviaGqljvulvdiozSjktswgpgzeggzhmEuyllxygX"));
                Debug.Assert(pack.severity == (MAV_SEVERITY)MAV_SEVERITY.MAV_SEVERITY_NOTICE);
            };
            GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.severity = (MAV_SEVERITY)MAV_SEVERITY.MAV_SEVERITY_NOTICE;
            p253.text_SET("ojzojEviaGqljvulvdiozSjktswgpgzeggzhmEuyllxygX", PH) ;
            CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnDEBUGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value == (float)2.9282285E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1564289540U);
                Debug.Assert(pack.ind == (byte)(byte)168);
            };
            GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
            PH.setPack(p254);
            p254.time_boot_ms = (uint)1564289540U;
            p254.ind = (byte)(byte)168;
            p254.value = (float)2.9282285E38F;
            CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSETUP_SIGNINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.initial_timestamp == (ulong)1501984956638718483L);
                Debug.Assert(pack.secret_key.SequenceEqual(new byte[] {(byte)164, (byte)211, (byte)184, (byte)158, (byte)23, (byte)39, (byte)145, (byte)129, (byte)110, (byte)66, (byte)115, (byte)9, (byte)151, (byte)177, (byte)180, (byte)81, (byte)185, (byte)215, (byte)241, (byte)154, (byte)110, (byte)23, (byte)66, (byte)91, (byte)112, (byte)157, (byte)130, (byte)247, (byte)245, (byte)18, (byte)80, (byte)217}));
                Debug.Assert(pack.target_component == (byte)(byte)124);
                Debug.Assert(pack.target_system == (byte)(byte)131);
            };
            GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.initial_timestamp = (ulong)1501984956638718483L;
            p256.target_system = (byte)(byte)131;
            p256.target_component = (byte)(byte)124;
            p256.secret_key_SET(new byte[] {(byte)164, (byte)211, (byte)184, (byte)158, (byte)23, (byte)39, (byte)145, (byte)129, (byte)110, (byte)66, (byte)115, (byte)9, (byte)151, (byte)177, (byte)180, (byte)81, (byte)185, (byte)215, (byte)241, (byte)154, (byte)110, (byte)23, (byte)66, (byte)91, (byte)112, (byte)157, (byte)130, (byte)247, (byte)245, (byte)18, (byte)80, (byte)217}, 0) ;
            CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnBUTTON_CHANGEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.last_change_ms == (uint)3739825097U);
                Debug.Assert(pack.state == (byte)(byte)86);
                Debug.Assert(pack.time_boot_ms == (uint)918817014U);
            };
            GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.time_boot_ms = (uint)918817014U;
            p257.last_change_ms = (uint)3739825097U;
            p257.state = (byte)(byte)86;
            CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPLAY_TUNEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tune_LEN(ph) == 2);
                Debug.Assert(pack.tune_TRY(ph).Equals("yc"));
                Debug.Assert(pack.target_system == (byte)(byte)54);
                Debug.Assert(pack.target_component == (byte)(byte)49);
            };
            GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_system = (byte)(byte)54;
            p258.tune_SET("yc", PH) ;
            p258.target_component = (byte)(byte)49;
            CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vendor_name.SequenceEqual(new byte[] {(byte)195, (byte)224, (byte)167, (byte)11, (byte)210, (byte)242, (byte)27, (byte)255, (byte)78, (byte)22, (byte)54, (byte)51, (byte)128, (byte)209, (byte)165, (byte)237, (byte)8, (byte)32, (byte)237, (byte)180, (byte)240, (byte)196, (byte)83, (byte)204, (byte)119, (byte)156, (byte)2, (byte)1, (byte)84, (byte)1, (byte)134, (byte)235}));
                Debug.Assert(pack.flags == (CAMERA_CAP_FLAGS)CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_MODES);
                Debug.Assert(pack.firmware_version == (uint)947861994U);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)39039);
                Debug.Assert(pack.focal_length == (float) -5.1088427E36F);
                Debug.Assert(pack.cam_definition_version == (ushort)(ushort)28253);
                Debug.Assert(pack.time_boot_ms == (uint)727742976U);
                Debug.Assert(pack.sensor_size_v == (float)2.526095E38F);
                Debug.Assert(pack.lens_id == (byte)(byte)107);
                Debug.Assert(pack.sensor_size_h == (float) -3.2163874E38F);
                Debug.Assert(pack.model_name.SequenceEqual(new byte[] {(byte)9, (byte)39, (byte)43, (byte)162, (byte)107, (byte)242, (byte)142, (byte)106, (byte)197, (byte)44, (byte)77, (byte)218, (byte)190, (byte)129, (byte)150, (byte)91, (byte)130, (byte)217, (byte)219, (byte)77, (byte)107, (byte)141, (byte)37, (byte)20, (byte)201, (byte)108, (byte)71, (byte)207, (byte)55, (byte)85, (byte)147, (byte)34}));
                Debug.Assert(pack.resolution_h == (ushort)(ushort)20433);
                Debug.Assert(pack.cam_definition_uri_LEN(ph) == 96);
                Debug.Assert(pack.cam_definition_uri_TRY(ph).Equals("cImnbboqailbzkyaomtyVdiepdfIrVzuhjAwxfaHgjJadnVwacxovyqcisxnvetaxpDwcopelritCpumhuakkiEhtztuqnch"));
            };
            GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.vendor_name_SET(new byte[] {(byte)195, (byte)224, (byte)167, (byte)11, (byte)210, (byte)242, (byte)27, (byte)255, (byte)78, (byte)22, (byte)54, (byte)51, (byte)128, (byte)209, (byte)165, (byte)237, (byte)8, (byte)32, (byte)237, (byte)180, (byte)240, (byte)196, (byte)83, (byte)204, (byte)119, (byte)156, (byte)2, (byte)1, (byte)84, (byte)1, (byte)134, (byte)235}, 0) ;
            p259.resolution_h = (ushort)(ushort)20433;
            p259.lens_id = (byte)(byte)107;
            p259.sensor_size_h = (float) -3.2163874E38F;
            p259.model_name_SET(new byte[] {(byte)9, (byte)39, (byte)43, (byte)162, (byte)107, (byte)242, (byte)142, (byte)106, (byte)197, (byte)44, (byte)77, (byte)218, (byte)190, (byte)129, (byte)150, (byte)91, (byte)130, (byte)217, (byte)219, (byte)77, (byte)107, (byte)141, (byte)37, (byte)20, (byte)201, (byte)108, (byte)71, (byte)207, (byte)55, (byte)85, (byte)147, (byte)34}, 0) ;
            p259.flags = (CAMERA_CAP_FLAGS)CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_MODES;
            p259.focal_length = (float) -5.1088427E36F;
            p259.resolution_v = (ushort)(ushort)39039;
            p259.sensor_size_v = (float)2.526095E38F;
            p259.cam_definition_uri_SET("cImnbboqailbzkyaomtyVdiepdfIrVzuhjAwxfaHgjJadnVwacxovyqcisxnvetaxpDwcopelritCpumhuakkiEhtztuqnch", PH) ;
            p259.time_boot_ms = (uint)727742976U;
            p259.cam_definition_version = (ushort)(ushort)28253;
            p259.firmware_version = (uint)947861994U;
            CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mode_id == (CAMERA_MODE)CAMERA_MODE.CAMERA_MODE_VIDEO);
                Debug.Assert(pack.time_boot_ms == (uint)3700502760U);
            };
            GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.mode_id = (CAMERA_MODE)CAMERA_MODE.CAMERA_MODE_VIDEO;
            p260.time_boot_ms = (uint)3700502760U;
            CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSTORAGE_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.storage_count == (byte)(byte)129);
                Debug.Assert(pack.time_boot_ms == (uint)1399700357U);
                Debug.Assert(pack.used_capacity == (float)1.4968591E38F);
                Debug.Assert(pack.available_capacity == (float)1.8366649E38F);
                Debug.Assert(pack.status == (byte)(byte)219);
                Debug.Assert(pack.total_capacity == (float) -2.890705E38F);
                Debug.Assert(pack.storage_id == (byte)(byte)90);
                Debug.Assert(pack.read_speed == (float) -3.3831258E38F);
                Debug.Assert(pack.write_speed == (float) -2.9995508E38F);
            };
            GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.storage_count = (byte)(byte)129;
            p261.storage_id = (byte)(byte)90;
            p261.total_capacity = (float) -2.890705E38F;
            p261.used_capacity = (float)1.4968591E38F;
            p261.available_capacity = (float)1.8366649E38F;
            p261.status = (byte)(byte)219;
            p261.write_speed = (float) -2.9995508E38F;
            p261.time_boot_ms = (uint)1399700357U;
            p261.read_speed = (float) -3.3831258E38F;
            CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_CAPTURE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.recording_time_ms == (uint)2617862266U);
                Debug.Assert(pack.available_capacity == (float)8.454567E37F);
                Debug.Assert(pack.time_boot_ms == (uint)3409465666U);
                Debug.Assert(pack.image_interval == (float) -1.8866038E38F);
                Debug.Assert(pack.image_status == (byte)(byte)85);
                Debug.Assert(pack.video_status == (byte)(byte)60);
            };
            GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.recording_time_ms = (uint)2617862266U;
            p262.available_capacity = (float)8.454567E37F;
            p262.image_interval = (float) -1.8866038E38F;
            p262.time_boot_ms = (uint)3409465666U;
            p262.image_status = (byte)(byte)85;
            p262.video_status = (byte)(byte)60;
            CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_IMAGE_CAPTUREDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_utc == (ulong)5253129310042564402L);
                Debug.Assert(pack.q.SequenceEqual(new float[] {1.2633258E38F, 1.1385683E38F, -1.8808024E38F, 3.1929689E38F}));
                Debug.Assert(pack.camera_id == (byte)(byte)203);
                Debug.Assert(pack.capture_result == (sbyte)(sbyte)37);
                Debug.Assert(pack.image_index == (int) -1301047753);
                Debug.Assert(pack.lon == (int)917524001);
                Debug.Assert(pack.relative_alt == (int) -674892531);
                Debug.Assert(pack.file_url_LEN(ph) == 42);
                Debug.Assert(pack.file_url_TRY(ph).Equals("ldqoctkzglrrcdnjszpwtqlxjadamMzxegUjtpgizn"));
                Debug.Assert(pack.time_boot_ms == (uint)2269878639U);
                Debug.Assert(pack.lat == (int) -1469462252);
                Debug.Assert(pack.alt == (int) -1669169933);
            };
            GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.lat = (int) -1469462252;
            p263.file_url_SET("ldqoctkzglrrcdnjszpwtqlxjadamMzxegUjtpgizn", PH) ;
            p263.alt = (int) -1669169933;
            p263.lon = (int)917524001;
            p263.relative_alt = (int) -674892531;
            p263.time_utc = (ulong)5253129310042564402L;
            p263.camera_id = (byte)(byte)203;
            p263.capture_result = (sbyte)(sbyte)37;
            p263.time_boot_ms = (uint)2269878639U;
            p263.q_SET(new float[] {1.2633258E38F, 1.1385683E38F, -1.8808024E38F, 3.1929689E38F}, 0) ;
            p263.image_index = (int) -1301047753;
            CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFLIGHT_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.takeoff_time_utc == (ulong)1948638558666314000L);
                Debug.Assert(pack.time_boot_ms == (uint)174598527U);
                Debug.Assert(pack.flight_uuid == (ulong)1146510719699456161L);
                Debug.Assert(pack.arming_time_utc == (ulong)6280289785239157980L);
            };
            GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.time_boot_ms = (uint)174598527U;
            p264.arming_time_utc = (ulong)6280289785239157980L;
            p264.takeoff_time_utc = (ulong)1948638558666314000L;
            p264.flight_uuid = (ulong)1146510719699456161L;
            CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnMOUNT_ORIENTATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float)1.9384035E38F);
                Debug.Assert(pack.pitch == (float)1.3361422E38F);
                Debug.Assert(pack.roll == (float)1.3276333E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3060433954U);
            };
            GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.time_boot_ms = (uint)3060433954U;
            p265.pitch = (float)1.3361422E38F;
            p265.roll = (float)1.3276333E38F;
            p265.yaw = (float)1.9384035E38F;
            CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)169);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)189, (byte)245, (byte)131, (byte)220, (byte)240, (byte)41, (byte)97, (byte)38, (byte)218, (byte)131, (byte)95, (byte)81, (byte)79, (byte)157, (byte)230, (byte)77, (byte)242, (byte)253, (byte)5, (byte)202, (byte)162, (byte)5, (byte)152, (byte)31, (byte)149, (byte)255, (byte)74, (byte)23, (byte)197, (byte)147, (byte)200, (byte)88, (byte)94, (byte)43, (byte)38, (byte)116, (byte)64, (byte)30, (byte)44, (byte)77, (byte)38, (byte)210, (byte)244, (byte)86, (byte)137, (byte)128, (byte)107, (byte)11, (byte)49, (byte)98, (byte)25, (byte)204, (byte)137, (byte)48, (byte)160, (byte)16, (byte)101, (byte)238, (byte)64, (byte)81, (byte)154, (byte)59, (byte)42, (byte)208, (byte)122, (byte)135, (byte)88, (byte)221, (byte)124, (byte)83, (byte)153, (byte)107, (byte)241, (byte)185, (byte)87, (byte)231, (byte)131, (byte)82, (byte)153, (byte)95, (byte)110, (byte)86, (byte)36, (byte)62, (byte)66, (byte)16, (byte)118, (byte)124, (byte)81, (byte)232, (byte)121, (byte)50, (byte)201, (byte)136, (byte)234, (byte)132, (byte)175, (byte)53, (byte)110, (byte)189, (byte)219, (byte)165, (byte)91, (byte)92, (byte)210, (byte)156, (byte)68, (byte)125, (byte)26, (byte)48, (byte)75, (byte)244, (byte)214, (byte)80, (byte)130, (byte)38, (byte)191, (byte)116, (byte)44, (byte)210, (byte)143, (byte)183, (byte)102, (byte)241, (byte)98, (byte)67, (byte)159, (byte)38, (byte)212, (byte)207, (byte)8, (byte)67, (byte)180, (byte)83, (byte)190, (byte)185, (byte)170, (byte)54, (byte)159, (byte)238, (byte)30, (byte)61, (byte)71, (byte)2, (byte)188, (byte)52, (byte)244, (byte)85, (byte)85, (byte)98, (byte)16, (byte)47, (byte)20, (byte)187, (byte)104, (byte)11, (byte)175, (byte)51, (byte)154, (byte)102, (byte)102, (byte)145, (byte)95, (byte)246, (byte)61, (byte)145, (byte)195, (byte)64, (byte)171, (byte)32, (byte)126, (byte)190, (byte)61, (byte)88, (byte)170, (byte)148, (byte)212, (byte)57, (byte)42, (byte)66, (byte)115, (byte)66, (byte)55, (byte)12, (byte)121, (byte)252, (byte)212, (byte)203, (byte)117, (byte)230, (byte)137, (byte)28, (byte)215, (byte)43, (byte)51, (byte)150, (byte)182, (byte)220, (byte)176, (byte)41, (byte)4, (byte)146, (byte)190, (byte)132, (byte)89, (byte)138, (byte)231, (byte)108, (byte)127, (byte)138, (byte)150, (byte)30, (byte)224, (byte)208, (byte)76, (byte)47, (byte)233, (byte)63, (byte)59, (byte)247, (byte)134, (byte)190, (byte)116, (byte)233, (byte)122, (byte)55, (byte)38, (byte)226, (byte)141, (byte)113, (byte)233, (byte)133, (byte)135, (byte)106, (byte)236, (byte)68, (byte)101, (byte)0, (byte)156, (byte)77, (byte)209, (byte)144, (byte)159, (byte)204, (byte)92, (byte)194, (byte)212, (byte)164, (byte)152}));
                Debug.Assert(pack.target_component == (byte)(byte)216);
                Debug.Assert(pack.sequence == (ushort)(ushort)59339);
                Debug.Assert(pack.length == (byte)(byte)0);
                Debug.Assert(pack.first_message_offset == (byte)(byte)0);
            };
            GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.sequence = (ushort)(ushort)59339;
            p266.data__SET(new byte[] {(byte)189, (byte)245, (byte)131, (byte)220, (byte)240, (byte)41, (byte)97, (byte)38, (byte)218, (byte)131, (byte)95, (byte)81, (byte)79, (byte)157, (byte)230, (byte)77, (byte)242, (byte)253, (byte)5, (byte)202, (byte)162, (byte)5, (byte)152, (byte)31, (byte)149, (byte)255, (byte)74, (byte)23, (byte)197, (byte)147, (byte)200, (byte)88, (byte)94, (byte)43, (byte)38, (byte)116, (byte)64, (byte)30, (byte)44, (byte)77, (byte)38, (byte)210, (byte)244, (byte)86, (byte)137, (byte)128, (byte)107, (byte)11, (byte)49, (byte)98, (byte)25, (byte)204, (byte)137, (byte)48, (byte)160, (byte)16, (byte)101, (byte)238, (byte)64, (byte)81, (byte)154, (byte)59, (byte)42, (byte)208, (byte)122, (byte)135, (byte)88, (byte)221, (byte)124, (byte)83, (byte)153, (byte)107, (byte)241, (byte)185, (byte)87, (byte)231, (byte)131, (byte)82, (byte)153, (byte)95, (byte)110, (byte)86, (byte)36, (byte)62, (byte)66, (byte)16, (byte)118, (byte)124, (byte)81, (byte)232, (byte)121, (byte)50, (byte)201, (byte)136, (byte)234, (byte)132, (byte)175, (byte)53, (byte)110, (byte)189, (byte)219, (byte)165, (byte)91, (byte)92, (byte)210, (byte)156, (byte)68, (byte)125, (byte)26, (byte)48, (byte)75, (byte)244, (byte)214, (byte)80, (byte)130, (byte)38, (byte)191, (byte)116, (byte)44, (byte)210, (byte)143, (byte)183, (byte)102, (byte)241, (byte)98, (byte)67, (byte)159, (byte)38, (byte)212, (byte)207, (byte)8, (byte)67, (byte)180, (byte)83, (byte)190, (byte)185, (byte)170, (byte)54, (byte)159, (byte)238, (byte)30, (byte)61, (byte)71, (byte)2, (byte)188, (byte)52, (byte)244, (byte)85, (byte)85, (byte)98, (byte)16, (byte)47, (byte)20, (byte)187, (byte)104, (byte)11, (byte)175, (byte)51, (byte)154, (byte)102, (byte)102, (byte)145, (byte)95, (byte)246, (byte)61, (byte)145, (byte)195, (byte)64, (byte)171, (byte)32, (byte)126, (byte)190, (byte)61, (byte)88, (byte)170, (byte)148, (byte)212, (byte)57, (byte)42, (byte)66, (byte)115, (byte)66, (byte)55, (byte)12, (byte)121, (byte)252, (byte)212, (byte)203, (byte)117, (byte)230, (byte)137, (byte)28, (byte)215, (byte)43, (byte)51, (byte)150, (byte)182, (byte)220, (byte)176, (byte)41, (byte)4, (byte)146, (byte)190, (byte)132, (byte)89, (byte)138, (byte)231, (byte)108, (byte)127, (byte)138, (byte)150, (byte)30, (byte)224, (byte)208, (byte)76, (byte)47, (byte)233, (byte)63, (byte)59, (byte)247, (byte)134, (byte)190, (byte)116, (byte)233, (byte)122, (byte)55, (byte)38, (byte)226, (byte)141, (byte)113, (byte)233, (byte)133, (byte)135, (byte)106, (byte)236, (byte)68, (byte)101, (byte)0, (byte)156, (byte)77, (byte)209, (byte)144, (byte)159, (byte)204, (byte)92, (byte)194, (byte)212, (byte)164, (byte)152}, 0) ;
            p266.target_system = (byte)(byte)169;
            p266.length = (byte)(byte)0;
            p266.target_component = (byte)(byte)216;
            p266.first_message_offset = (byte)(byte)0;
            CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATA_ACKEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.first_message_offset == (byte)(byte)60);
                Debug.Assert(pack.target_system == (byte)(byte)109);
                Debug.Assert(pack.target_component == (byte)(byte)35);
                Debug.Assert(pack.sequence == (ushort)(ushort)1435);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)161, (byte)240, (byte)251, (byte)6, (byte)48, (byte)167, (byte)96, (byte)73, (byte)127, (byte)221, (byte)237, (byte)20, (byte)255, (byte)155, (byte)1, (byte)187, (byte)224, (byte)198, (byte)234, (byte)177, (byte)56, (byte)17, (byte)253, (byte)125, (byte)64, (byte)78, (byte)27, (byte)99, (byte)185, (byte)11, (byte)13, (byte)132, (byte)207, (byte)10, (byte)190, (byte)240, (byte)210, (byte)194, (byte)232, (byte)33, (byte)135, (byte)254, (byte)45, (byte)72, (byte)53, (byte)172, (byte)166, (byte)176, (byte)38, (byte)188, (byte)144, (byte)83, (byte)58, (byte)165, (byte)55, (byte)183, (byte)107, (byte)89, (byte)197, (byte)149, (byte)112, (byte)68, (byte)163, (byte)219, (byte)24, (byte)71, (byte)28, (byte)58, (byte)141, (byte)19, (byte)156, (byte)68, (byte)121, (byte)48, (byte)117, (byte)222, (byte)182, (byte)180, (byte)129, (byte)104, (byte)253, (byte)223, (byte)17, (byte)15, (byte)61, (byte)42, (byte)58, (byte)204, (byte)197, (byte)4, (byte)21, (byte)158, (byte)255, (byte)233, (byte)198, (byte)108, (byte)80, (byte)158, (byte)85, (byte)181, (byte)68, (byte)210, (byte)70, (byte)8, (byte)119, (byte)188, (byte)58, (byte)125, (byte)29, (byte)201, (byte)234, (byte)174, (byte)125, (byte)106, (byte)194, (byte)168, (byte)210, (byte)128, (byte)231, (byte)21, (byte)243, (byte)156, (byte)202, (byte)81, (byte)232, (byte)226, (byte)187, (byte)117, (byte)110, (byte)15, (byte)178, (byte)29, (byte)192, (byte)144, (byte)41, (byte)141, (byte)234, (byte)110, (byte)252, (byte)206, (byte)15, (byte)149, (byte)231, (byte)202, (byte)184, (byte)108, (byte)181, (byte)156, (byte)167, (byte)184, (byte)133, (byte)239, (byte)44, (byte)29, (byte)46, (byte)135, (byte)211, (byte)46, (byte)138, (byte)30, (byte)242, (byte)240, (byte)110, (byte)12, (byte)250, (byte)167, (byte)79, (byte)144, (byte)201, (byte)97, (byte)34, (byte)16, (byte)22, (byte)86, (byte)107, (byte)1, (byte)48, (byte)63, (byte)209, (byte)161, (byte)59, (byte)84, (byte)52, (byte)237, (byte)183, (byte)146, (byte)74, (byte)20, (byte)174, (byte)233, (byte)75, (byte)40, (byte)44, (byte)217, (byte)27, (byte)27, (byte)50, (byte)213, (byte)40, (byte)76, (byte)196, (byte)142, (byte)244, (byte)79, (byte)26, (byte)68, (byte)160, (byte)54, (byte)234, (byte)5, (byte)50, (byte)106, (byte)144, (byte)193, (byte)20, (byte)36, (byte)11, (byte)48, (byte)252, (byte)106, (byte)71, (byte)69, (byte)132, (byte)208, (byte)213, (byte)95, (byte)87, (byte)168, (byte)253, (byte)80, (byte)152, (byte)150, (byte)205, (byte)248, (byte)136, (byte)100, (byte)95, (byte)177, (byte)230, (byte)110, (byte)142, (byte)0, (byte)130, (byte)106, (byte)168, (byte)172, (byte)101, (byte)238, (byte)171}));
                Debug.Assert(pack.length == (byte)(byte)71);
            };
            GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.data__SET(new byte[] {(byte)161, (byte)240, (byte)251, (byte)6, (byte)48, (byte)167, (byte)96, (byte)73, (byte)127, (byte)221, (byte)237, (byte)20, (byte)255, (byte)155, (byte)1, (byte)187, (byte)224, (byte)198, (byte)234, (byte)177, (byte)56, (byte)17, (byte)253, (byte)125, (byte)64, (byte)78, (byte)27, (byte)99, (byte)185, (byte)11, (byte)13, (byte)132, (byte)207, (byte)10, (byte)190, (byte)240, (byte)210, (byte)194, (byte)232, (byte)33, (byte)135, (byte)254, (byte)45, (byte)72, (byte)53, (byte)172, (byte)166, (byte)176, (byte)38, (byte)188, (byte)144, (byte)83, (byte)58, (byte)165, (byte)55, (byte)183, (byte)107, (byte)89, (byte)197, (byte)149, (byte)112, (byte)68, (byte)163, (byte)219, (byte)24, (byte)71, (byte)28, (byte)58, (byte)141, (byte)19, (byte)156, (byte)68, (byte)121, (byte)48, (byte)117, (byte)222, (byte)182, (byte)180, (byte)129, (byte)104, (byte)253, (byte)223, (byte)17, (byte)15, (byte)61, (byte)42, (byte)58, (byte)204, (byte)197, (byte)4, (byte)21, (byte)158, (byte)255, (byte)233, (byte)198, (byte)108, (byte)80, (byte)158, (byte)85, (byte)181, (byte)68, (byte)210, (byte)70, (byte)8, (byte)119, (byte)188, (byte)58, (byte)125, (byte)29, (byte)201, (byte)234, (byte)174, (byte)125, (byte)106, (byte)194, (byte)168, (byte)210, (byte)128, (byte)231, (byte)21, (byte)243, (byte)156, (byte)202, (byte)81, (byte)232, (byte)226, (byte)187, (byte)117, (byte)110, (byte)15, (byte)178, (byte)29, (byte)192, (byte)144, (byte)41, (byte)141, (byte)234, (byte)110, (byte)252, (byte)206, (byte)15, (byte)149, (byte)231, (byte)202, (byte)184, (byte)108, (byte)181, (byte)156, (byte)167, (byte)184, (byte)133, (byte)239, (byte)44, (byte)29, (byte)46, (byte)135, (byte)211, (byte)46, (byte)138, (byte)30, (byte)242, (byte)240, (byte)110, (byte)12, (byte)250, (byte)167, (byte)79, (byte)144, (byte)201, (byte)97, (byte)34, (byte)16, (byte)22, (byte)86, (byte)107, (byte)1, (byte)48, (byte)63, (byte)209, (byte)161, (byte)59, (byte)84, (byte)52, (byte)237, (byte)183, (byte)146, (byte)74, (byte)20, (byte)174, (byte)233, (byte)75, (byte)40, (byte)44, (byte)217, (byte)27, (byte)27, (byte)50, (byte)213, (byte)40, (byte)76, (byte)196, (byte)142, (byte)244, (byte)79, (byte)26, (byte)68, (byte)160, (byte)54, (byte)234, (byte)5, (byte)50, (byte)106, (byte)144, (byte)193, (byte)20, (byte)36, (byte)11, (byte)48, (byte)252, (byte)106, (byte)71, (byte)69, (byte)132, (byte)208, (byte)213, (byte)95, (byte)87, (byte)168, (byte)253, (byte)80, (byte)152, (byte)150, (byte)205, (byte)248, (byte)136, (byte)100, (byte)95, (byte)177, (byte)230, (byte)110, (byte)142, (byte)0, (byte)130, (byte)106, (byte)168, (byte)172, (byte)101, (byte)238, (byte)171}, 0) ;
            p267.sequence = (ushort)(ushort)1435;
            p267.target_system = (byte)(byte)109;
            p267.target_component = (byte)(byte)35;
            p267.first_message_offset = (byte)(byte)60;
            p267.length = (byte)(byte)71;
            CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLOGGING_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)171);
                Debug.Assert(pack.sequence == (ushort)(ushort)2704);
                Debug.Assert(pack.target_component == (byte)(byte)108);
            };
            GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_system = (byte)(byte)171;
            p268.sequence = (ushort)(ushort)2704;
            p268.target_component = (byte)(byte)108;
            CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnVIDEO_STREAM_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.status == (byte)(byte)143);
                Debug.Assert(pack.rotation == (ushort)(ushort)30027);
                Debug.Assert(pack.camera_id == (byte)(byte)226);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)36928);
                Debug.Assert(pack.bitrate == (uint)2817687243U);
                Debug.Assert(pack.uri_LEN(ph) == 161);
                Debug.Assert(pack.uri_TRY(ph).Equals("rokpzzktnfzlnRiloDuqcgocjMweprompqtnEyioauhobjXwmcVldfrunnnjevqopzcTJpznAuwEcnrlozokpvbwtjsomolcrxtyqduCnnfmcjlmxnyzdlJzazlcgxivpvjixzwjeablekcDtmglWwjbxqhpfjsnc"));
                Debug.Assert(pack.framerate == (float)1.9004873E38F);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)63405);
            };
            GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.resolution_v = (ushort)(ushort)36928;
            p269.status = (byte)(byte)143;
            p269.framerate = (float)1.9004873E38F;
            p269.uri_SET("rokpzzktnfzlnRiloDuqcgocjMweprompqtnEyioauhobjXwmcVldfrunnnjevqopzcTJpznAuwEcnrlozokpvbwtjsomolcrxtyqduCnnfmcjlmxnyzdlJzazlcgxivpvjixzwjeablekcDtmglWwjbxqhpfjsnc", PH) ;
            p269.rotation = (ushort)(ushort)30027;
            p269.camera_id = (byte)(byte)226;
            p269.bitrate = (uint)2817687243U;
            p269.resolution_h = (ushort)(ushort)63405;
            CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSET_VIDEO_STREAM_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uri_LEN(ph) == 73);
                Debug.Assert(pack.uri_TRY(ph).Equals("decBpzhPfhoVjlyrcuehcogipnvvwskjdarvhizfadjofowhqakmvqbigoSgimlmrlpagnxhm"));
                Debug.Assert(pack.target_system == (byte)(byte)76);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)32594);
                Debug.Assert(pack.camera_id == (byte)(byte)237);
                Debug.Assert(pack.framerate == (float)9.250557E37F);
                Debug.Assert(pack.rotation == (ushort)(ushort)23298);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)37710);
                Debug.Assert(pack.target_component == (byte)(byte)155);
                Debug.Assert(pack.bitrate == (uint)3078360372U);
            };
            GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.resolution_v = (ushort)(ushort)37710;
            p270.camera_id = (byte)(byte)237;
            p270.target_system = (byte)(byte)76;
            p270.target_component = (byte)(byte)155;
            p270.uri_SET("decBpzhPfhoVjlyrcuehcogipnvvwskjdarvhizfadjofowhqakmvqbigoSgimlmrlpagnxhm", PH) ;
            p270.framerate = (float)9.250557E37F;
            p270.rotation = (ushort)(ushort)23298;
            p270.bitrate = (uint)3078360372U;
            p270.resolution_h = (ushort)(ushort)32594;
            CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnWIFI_CONFIG_APReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ssid_LEN(ph) == 1);
                Debug.Assert(pack.ssid_TRY(ph).Equals("r"));
                Debug.Assert(pack.password_LEN(ph) == 51);
                Debug.Assert(pack.password_TRY(ph).Equals("onsjsqxoCcoruzkgrraqfoEfeicyloBYLaugzcomowuepzrcbsj"));
            };
            GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.password_SET("onsjsqxoCcoruzkgrraqfoEfeicyloBYLaugzcomowuepzrcbsj", PH) ;
            p299.ssid_SET("r", PH) ;
            CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPROTOCOL_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.spec_version_hash.SequenceEqual(new byte[] {(byte)41, (byte)31, (byte)213, (byte)195, (byte)238, (byte)42, (byte)233, (byte)13}));
                Debug.Assert(pack.version == (ushort)(ushort)4112);
                Debug.Assert(pack.library_version_hash.SequenceEqual(new byte[] {(byte)244, (byte)208, (byte)2, (byte)59, (byte)14, (byte)152, (byte)50, (byte)155}));
                Debug.Assert(pack.max_version == (ushort)(ushort)63766);
                Debug.Assert(pack.min_version == (ushort)(ushort)542);
            };
            GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.max_version = (ushort)(ushort)63766;
            p300.version = (ushort)(ushort)4112;
            p300.library_version_hash_SET(new byte[] {(byte)244, (byte)208, (byte)2, (byte)59, (byte)14, (byte)152, (byte)50, (byte)155}, 0) ;
            p300.spec_version_hash_SET(new byte[] {(byte)41, (byte)31, (byte)213, (byte)195, (byte)238, (byte)42, (byte)233, (byte)13}, 0) ;
            p300.min_version = (ushort)(ushort)542;
            CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vendor_specific_status_code == (ushort)(ushort)34963);
                Debug.Assert(pack.sub_mode == (byte)(byte)229);
                Debug.Assert(pack.uptime_sec == (uint)183815858U);
                Debug.Assert(pack.mode == (UAVCAN_NODE_MODE)UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE);
                Debug.Assert(pack.health == (UAVCAN_NODE_HEALTH)UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR);
                Debug.Assert(pack.time_usec == (ulong)2899036445010065785L);
            };
            GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.sub_mode = (byte)(byte)229;
            p310.time_usec = (ulong)2899036445010065785L;
            p310.health = (UAVCAN_NODE_HEALTH)UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR;
            p310.uptime_sec = (uint)183815858U;
            p310.vendor_specific_status_code = (ushort)(ushort)34963;
            p310.mode = (UAVCAN_NODE_MODE)UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE;
            CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_INFOReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.hw_unique_id.SequenceEqual(new byte[] {(byte)28, (byte)40, (byte)49, (byte)176, (byte)158, (byte)46, (byte)238, (byte)2, (byte)137, (byte)181, (byte)98, (byte)156, (byte)120, (byte)135, (byte)45, (byte)208}));
                Debug.Assert(pack.sw_vcs_commit == (uint)3290549208U);
                Debug.Assert(pack.name_LEN(ph) == 16);
                Debug.Assert(pack.name_TRY(ph).Equals("aoSuggowTiaIiBbY"));
                Debug.Assert(pack.hw_version_minor == (byte)(byte)66);
                Debug.Assert(pack.hw_version_major == (byte)(byte)101);
                Debug.Assert(pack.time_usec == (ulong)7536800831100429520L);
                Debug.Assert(pack.sw_version_major == (byte)(byte)28);
                Debug.Assert(pack.uptime_sec == (uint)1203310909U);
                Debug.Assert(pack.sw_version_minor == (byte)(byte)35);
            };
            GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.hw_version_minor = (byte)(byte)66;
            p311.name_SET("aoSuggowTiaIiBbY", PH) ;
            p311.sw_vcs_commit = (uint)3290549208U;
            p311.sw_version_minor = (byte)(byte)35;
            p311.hw_version_major = (byte)(byte)101;
            p311.sw_version_major = (byte)(byte)28;
            p311.hw_unique_id_SET(new byte[] {(byte)28, (byte)40, (byte)49, (byte)176, (byte)158, (byte)46, (byte)238, (byte)2, (byte)137, (byte)181, (byte)98, (byte)156, (byte)120, (byte)135, (byte)45, (byte)208}, 0) ;
            p311.time_usec = (ulong)7536800831100429520L;
            p311.uptime_sec = (uint)1203310909U;
            CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_index == (short)(short)20686);
                Debug.Assert(pack.target_component == (byte)(byte)82);
                Debug.Assert(pack.target_system == (byte)(byte)249);
                Debug.Assert(pack.param_id_LEN(ph) == 11);
                Debug.Assert(pack.param_id_TRY(ph).Equals("nbgtilqmhks"));
            };
            GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.param_id_SET("nbgtilqmhks", PH) ;
            p320.param_index = (short)(short)20686;
            p320.target_system = (byte)(byte)249;
            p320.target_component = (byte)(byte)82;
            CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)225);
                Debug.Assert(pack.target_system == (byte)(byte)125);
            };
            GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_component = (byte)(byte)225;
            p321.target_system = (byte)(byte)125;
            CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32);
                Debug.Assert(pack.param_count == (ushort)(ushort)25497);
                Debug.Assert(pack.param_index == (ushort)(ushort)57280);
                Debug.Assert(pack.param_id_LEN(ph) == 4);
                Debug.Assert(pack.param_id_TRY(ph).Equals("jsfe"));
                Debug.Assert(pack.param_value_LEN(ph) == 112);
                Debug.Assert(pack.param_value_TRY(ph).Equals("FoxnaotqzhlhveimhsvvrupFquejyykryhsjcBdzahqcqAfTlcvifapyglvblliZaoqtocsHhwqwefohzeubizliEcqchvsifxfnhwIfdxpbXxBM"));
            };
            GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_value_SET("FoxnaotqzhlhveimhsvvrupFquejyykryhsjcBdzahqcqAfTlcvifapyglvblliZaoqtocsHhwqwefohzeubizliEcqchvsifxfnhwIfdxpbXxBM", PH) ;
            p322.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32;
            p322.param_id_SET("jsfe", PH) ;
            p322.param_index = (ushort)(ushort)57280;
            p322.param_count = (ushort)(ushort)25497;
            CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)235);
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8);
                Debug.Assert(pack.param_id_LEN(ph) == 9);
                Debug.Assert(pack.param_id_TRY(ph).Equals("oycoqmgmh"));
                Debug.Assert(pack.target_system == (byte)(byte)25);
                Debug.Assert(pack.param_value_LEN(ph) == 1);
                Debug.Assert(pack.param_value_TRY(ph).Equals("h"));
            };
            GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.param_value_SET("h", PH) ;
            p323.target_component = (byte)(byte)235;
            p323.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8;
            p323.param_id_SET("oycoqmgmh", PH) ;
            p323.target_system = (byte)(byte)25;
            CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_result == (PARAM_ACK)PARAM_ACK.PARAM_ACK_IN_PROGRESS);
                Debug.Assert(pack.param_type == (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64);
                Debug.Assert(pack.param_id_LEN(ph) == 8);
                Debug.Assert(pack.param_id_TRY(ph).Equals("sqmcbBnt"));
                Debug.Assert(pack.param_value_LEN(ph) == 119);
                Debug.Assert(pack.param_value_TRY(ph).Equals("jzxleavdlagjmEzqqozwaLdDugsowmtkwlywgckdifuhanmeptnwootxkVvrfdciqpktsvNilyjagpnvryfMzwJsqjacubtpojeoowtxkfliymGcmoairar"));
            };
            GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64;
            p324.param_result = (PARAM_ACK)PARAM_ACK.PARAM_ACK_IN_PROGRESS;
            p324.param_id_SET("sqmcbBnt", PH) ;
            p324.param_value_SET("jzxleavdlagjmEzqqozwaLdDugsowmtkwlywgckdifuhanmeptnwootxkVvrfdciqpktsvNilyjagpnvryfMzwJsqjacubtpojeoowtxkfliymGcmoairar", PH) ;
            CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnOBSTACLE_DISTANCEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)3540506755345337717L);
                Debug.Assert(pack.max_distance == (ushort)(ushort)36462);
                Debug.Assert(pack.min_distance == (ushort)(ushort)58726);
                Debug.Assert(pack.distances.SequenceEqual(new ushort[] {(ushort)7519, (ushort)20586, (ushort)5871, (ushort)2111, (ushort)64083, (ushort)15493, (ushort)14393, (ushort)65135, (ushort)41760, (ushort)30871, (ushort)26200, (ushort)7339, (ushort)18425, (ushort)29363, (ushort)2460, (ushort)44088, (ushort)37966, (ushort)33974, (ushort)43781, (ushort)45379, (ushort)14378, (ushort)61594, (ushort)18583, (ushort)43710, (ushort)8199, (ushort)19918, (ushort)61945, (ushort)32066, (ushort)38961, (ushort)40552, (ushort)47160, (ushort)38651, (ushort)38270, (ushort)35869, (ushort)7266, (ushort)5949, (ushort)34028, (ushort)31304, (ushort)17771, (ushort)28021, (ushort)21329, (ushort)465, (ushort)57735, (ushort)51973, (ushort)60480, (ushort)35880, (ushort)6778, (ushort)42487, (ushort)3097, (ushort)31630, (ushort)2240, (ushort)19729, (ushort)54082, (ushort)49887, (ushort)41732, (ushort)54958, (ushort)20923, (ushort)45168, (ushort)26321, (ushort)56145, (ushort)26571, (ushort)10336, (ushort)48003, (ushort)11258, (ushort)29413, (ushort)43619, (ushort)45201, (ushort)2664, (ushort)6445, (ushort)12835, (ushort)64099, (ushort)14721}));
                Debug.Assert(pack.sensor_type == (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED);
                Debug.Assert(pack.increment == (byte)(byte)76);
            };
            GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.sensor_type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED;
            p330.distances_SET(new ushort[] {(ushort)7519, (ushort)20586, (ushort)5871, (ushort)2111, (ushort)64083, (ushort)15493, (ushort)14393, (ushort)65135, (ushort)41760, (ushort)30871, (ushort)26200, (ushort)7339, (ushort)18425, (ushort)29363, (ushort)2460, (ushort)44088, (ushort)37966, (ushort)33974, (ushort)43781, (ushort)45379, (ushort)14378, (ushort)61594, (ushort)18583, (ushort)43710, (ushort)8199, (ushort)19918, (ushort)61945, (ushort)32066, (ushort)38961, (ushort)40552, (ushort)47160, (ushort)38651, (ushort)38270, (ushort)35869, (ushort)7266, (ushort)5949, (ushort)34028, (ushort)31304, (ushort)17771, (ushort)28021, (ushort)21329, (ushort)465, (ushort)57735, (ushort)51973, (ushort)60480, (ushort)35880, (ushort)6778, (ushort)42487, (ushort)3097, (ushort)31630, (ushort)2240, (ushort)19729, (ushort)54082, (ushort)49887, (ushort)41732, (ushort)54958, (ushort)20923, (ushort)45168, (ushort)26321, (ushort)56145, (ushort)26571, (ushort)10336, (ushort)48003, (ushort)11258, (ushort)29413, (ushort)43619, (ushort)45201, (ushort)2664, (ushort)6445, (ushort)12835, (ushort)64099, (ushort)14721}, 0) ;
            p330.time_usec = (ulong)3540506755345337717L;
            p330.min_distance = (ushort)(ushort)58726;
            p330.max_distance = (ushort)(ushort)36462;
            p330.increment = (byte)(byte)76;
            CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
        }
    }
}
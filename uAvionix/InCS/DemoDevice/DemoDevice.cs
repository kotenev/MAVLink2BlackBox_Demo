
using System;
using System.Collections.Generic;
using System.Threading;
using System.Diagnostics;
using org.unirail.BlackBox;
using Inside = org.unirail.BlackBox.Host.Pack.Meta.Field.Bounds.Inside;
using Field = org.unirail.BlackBox.Host.Pack.Meta.Field;
namespace org.noname
{
    public class DemoDevice : Host
    {
        public class HEARTBEAT : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal HEARTBEAT() : base(meta0, 0) { }
            internal HEARTBEAT(int bytes) : base(meta0, bytes) { }
            public uint custom_mode //A bitfield for use for autopilot-specific flags
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public byte mavlink_version //MAVLink version, not writable by user, gets added by protocol because of magic data type: byte_mavlink_versi
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  4);}
            }

            public MAV_TYPE type //Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM
            {
                get {  return (MAV_TYPE)(0 +  BitUtils.get_bits(data, 40, 5));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 5, data, 40);}
            }

            public MAV_AUTOPILOT autopilot //Autopilot type / class. defined in MAV_AUTOPILOT ENU
            {
                get {  return (MAV_AUTOPILOT)(0 +  BitUtils.get_bits(data, 45, 5));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 5, data, 45);}
            }

            public MAV_MODE_FLAG base_mode //System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.
            {
                get
                {
                    switch((int)BitUtils.get_bits(data, 50, 4))
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

            public MAV_STATE system_status //System status flag, see MAV_STATE ENU
            {
                get {  return (MAV_STATE)(0 +  BitUtils.get_bits(data, 54, 4));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 54);}
            }
            static readonly Meta meta0 = new Meta(0, 0, 1, 0, 8, 58);
        } public class SYS_STATUS : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal SYS_STATUS() : base(meta1, 0) { }
            internal SYS_STATUS(int bytes) : base(meta1, bytes) { }
            public ushort load //Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 100
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort voltage_battery //Battery voltage, in millivolts (1 = 1 millivolt
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public ushort drop_rate_comm
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  4);}
            }

            public ushort errors_comm
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  6, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  6);}
            }

            public ushort errors_count1 //Autopilot-specific error
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  8, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  8);}
            }

            public ushort errors_count2 //Autopilot-specific error
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  10, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  10);}
            }

            public ushort errors_count3 //Autopilot-specific error
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  12, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  12);}
            }

            public ushort errors_count4 //Autopilot-specific error
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  14, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  14);}
            }

            public short current_battery //Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the curre
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  16, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  16);}
            }

            public sbyte battery_remaining //Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining batter
            {
                get {  return (sbyte)((sbyte) BitUtils.get_bytes(data,  18, 1));}
                set {  BitUtils.set_bytes((byte)(value), 1, data,  18);}
            }

            public MAV_SYS_STATUS_SENSOR onboard_control_sensors_present
            {
                get
                {
                    switch((int)BitUtils.get_bits(data, 152, 5))
                    {
                        case 0:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO;
                        case 1:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL;
                        case 2:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG;
                        case 3:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
                        case 4:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
                        case 5:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS;
                        case 6:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
                        case 7:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION;
                        case 8:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION;
                        case 9:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH;
                        case 10:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL;
                        case 11:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION;
                        case 12:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION;
                        case 13:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
                        case 14:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
                        case 15:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
                        case 16:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
                        case 17:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2;
                        case 18:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2;
                        case 19:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2;
                        case 20:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE;
                        case 21:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS;
                        case 22:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN;
                        case 23:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR;
                        case 24:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING;
                        case 25:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY;
                    }
                    throw  new ArgumentException("Unknown enum ID ");
                }
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

            public MAV_SYS_STATUS_SENSOR onboard_control_sensors_enabled
            {
                get
                {
                    switch((int)BitUtils.get_bits(data, 157, 5))
                    {
                        case 0:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO;
                        case 1:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL;
                        case 2:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG;
                        case 3:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
                        case 4:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
                        case 5:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS;
                        case 6:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
                        case 7:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION;
                        case 8:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION;
                        case 9:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH;
                        case 10:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL;
                        case 11:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION;
                        case 12:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION;
                        case 13:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
                        case 14:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
                        case 15:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
                        case 16:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
                        case 17:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2;
                        case 18:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2;
                        case 19:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2;
                        case 20:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE;
                        case 21:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS;
                        case 22:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN;
                        case 23:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR;
                        case 24:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING;
                        case 25:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY;
                    }
                    throw  new ArgumentException("Unknown enum ID ");
                }
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

            public MAV_SYS_STATUS_SENSOR onboard_control_sensors_health
            {
                get
                {
                    switch((int)BitUtils.get_bits(data, 162, 5))
                    {
                        case 0:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO;
                        case 1:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL;
                        case 2:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG;
                        case 3:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
                        case 4:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
                        case 5:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS;
                        case 6:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
                        case 7:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION;
                        case 8:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION;
                        case 9:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH;
                        case 10:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL;
                        case 11:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION;
                        case 12:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION;
                        case 13:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
                        case 14:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
                        case 15:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
                        case 16:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
                        case 17:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2;
                        case 18:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2;
                        case 19:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2;
                        case 20:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE;
                        case 21:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS;
                        case 22:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN;
                        case 23:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR;
                        case 24:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING;
                        case 25:
                            return MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY;
                    }
                    throw  new ArgumentException("Unknown enum ID ");
                }
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
            static readonly Meta meta1 = new Meta(1, 8, 0, 0, 21, 167);
        } public class SYSTEM_TIME : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal SYSTEM_TIME() : base(meta2, 0) { }
            internal SYSTEM_TIME(int bytes) : base(meta2, bytes) { }
            public uint time_boot_ms //Timestamp of the component clock since boot time in milliseconds
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public ulong time_unix_usec //Timestamp of the master clock in microseconds since UNIX epoch
            {
                get {  return (BitUtils.get_bytes(data,  4, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  4);}
            }
            static readonly Meta meta2 = new Meta(2, 0, 1, 1, 12, 96);
        } public class POSITION_TARGET_LOCAL_NED : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal POSITION_TARGET_LOCAL_NED() : base(meta3, 0) { }
            internal POSITION_TARGET_LOCAL_NED(int bytes) : base(meta3, bytes) { }
            public ushort type_mask
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public uint time_boot_ms //Timestamp in milliseconds since system boo
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  2, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  2);}
            }

            public float x //X Position in NED frame in meter
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  6, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 6);}
            }

            public float y //Y Position in NED frame in meter
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  10, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 10);}
            }

            public float z //Z Position in NED frame in meters (note, altitude is negative in NED
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  14, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 14);}
            }

            public float vx //X velocity in NED frame in meter /
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  18, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 18);}
            }

            public float vy //Y velocity in NED frame in meter /
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  22, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 22);}
            }

            public float vz //Z velocity in NED frame in meter /
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  26, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 26);}
            }

            public float afx //X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  30, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 30);}
            }

            public float afy //Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  34, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 34);}
            }

            public float afz //Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  38, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 38);}
            }

            public float yaw //yaw setpoint in ra
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  42, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 42);}
            }

            public float yaw_rate //yaw rate setpoint in rad/
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  46, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 46);}
            }

            public MAV_FRAME coordinate_frame
            {
                get {  return (MAV_FRAME)(0 +  BitUtils.get_bits(data, 400, 4));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 400);}
            }
            static readonly Meta meta3 = new Meta(3, 1, 1, 0, 51, 404);
        } public class PING : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal PING() : base(meta4, 0) { }
            internal PING(int bytes) : base(meta4, bytes) { }
            public uint seq //PING sequenc
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public ulong time_usec //Unix timestamp in microseconds or since system boot if smaller than MAVLink epoch (1.1.2009
            {
                get {  return (BitUtils.get_bytes(data,  4, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  4);}
            }

            public byte target_system
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  12, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  12);}
            }

            public byte target_component
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  13, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  13);}
            }
            static readonly Meta meta4 = new Meta(4, 0, 1, 1, 14, 112);
        } public class CHANGE_OPERATOR_CONTROL : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal CHANGE_OPERATOR_CONTROL() : base(meta5, 0) { }
            internal CHANGE_OPERATOR_CONTROL(int bytes) : base(meta5, bytes) { }
            public byte target_system //System the GCS requests control fo
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte control_request //0: request control of this MAV, 1: Release control of this MA
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            public byte version
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }
            public string passkey_TRY(Inside ph)
            {
                if(ph.field_bit !=  24 && !try_visit_field(ph, 24)  ||  !try_visit_item(ph, 0)) return null;
                return new string(passkey_GET(ph, new char[ph.items], 0));
            }
            public char[]passkey_GET(Inside ph, char[] dst_ch, int pos)
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int passkey_LEN(Inside ph)
            {
                return (ph.field_bit !=  24 && !try_visit_field(ph, 24)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            } public void passkey_SET(string src, Inside ph) {passkey_SET(src.ToCharArray(), 0, src.Length, ph);} public void passkey_SET(char[] src, int pos, int items, Inside ph)
            {
                if(ph.field_bit != 24 && insert_field(ph, 24, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            } static readonly Meta meta5 = new Meta(5, 0, 0, 0, 4, 24, 0, _g);
        } public class CHANGE_OPERATOR_CONTROL_ACK : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal CHANGE_OPERATOR_CONTROL_ACK() : base(meta6, 0) { }
            internal CHANGE_OPERATOR_CONTROL_ACK(int bytes) : base(meta6, bytes) { }
            public byte gcs_system_id //ID of the GCS this message
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte control_request //0: request control of this MAV, 1: Release control of this MA
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            public byte ack
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }
            static readonly Meta meta6 = new Meta(6, 0, 0, 0, 3, 24);
        } public class AUTH_KEY : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal AUTH_KEY() : base(meta7, 0) { }
            internal AUTH_KEY(int bytes) : base(meta7, bytes) { }  public string key_TRY(Inside ph)//ke
            {
                if(ph.field_bit !=  0 && !try_visit_field(ph, 0)  ||  !try_visit_item(ph, 0)) return null;
                return new string(key_GET(ph, new char[ph.items], 0));
            }
            public char[]key_GET(Inside ph, char[] dst_ch, int pos) //ke
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int key_LEN(Inside ph)
            {
                return (ph.field_bit !=  0 && !try_visit_field(ph, 0)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            } public void key_SET(string src, Inside ph) //ke
            {key_SET(src.ToCharArray(), 0, src.Length, ph);} public void key_SET(char[] src, int pos, int items, Inside ph) //ke
            {
                if(ph.field_bit != 0 && insert_field(ph, 0, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            } static readonly Meta meta7 = new Meta(7, 0, 0, 0, 1, 0, 0, _o);
        } public class SET_MODE : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal SET_MODE() : base(meta11, 0) { }
            internal SET_MODE(int bytes) : base(meta11, bytes) { }
            public uint custom_mode //The new autopilot-specific mode. This field can be ignored by an autopilot
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public byte target_system //The system setting the mod
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  4);}
            }

            public MAV_MODE base_mode //The new base mod
            {
                get
                {
                    switch((int)BitUtils.get_bits(data, 40, 4))
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
            static readonly Meta meta11 = new Meta(11, 0, 1, 0, 6, 44);
        } public class PARAM_REQUEST_READ : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal PARAM_REQUEST_READ() : base(meta20, 0) { }
            internal PARAM_REQUEST_READ(int bytes) : base(meta20, bytes) { }
            public byte target_system //System I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            public short param_index //Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignore
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  2, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  2);}
            }
            public string param_id_TRY(Inside ph)
            {
                if(ph.field_bit !=  32 && !try_visit_field(ph, 32)  ||  !try_visit_item(ph, 0)) return null;
                return new string(param_id_GET(ph, new char[ph.items], 0));
            }
            public char[]param_id_GET(Inside ph, char[] dst_ch, int pos)
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int param_id_LEN(Inside ph)
            {
                return (ph.field_bit !=  32 && !try_visit_field(ph, 32)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            } public void param_id_SET(string src, Inside ph) {param_id_SET(src.ToCharArray(), 0, src.Length, ph);} public void param_id_SET(char[] src, int pos, int items, Inside ph)
            {
                if(ph.field_bit != 32 && insert_field(ph, 32, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            } static readonly Meta meta20 = new Meta(20, 0, 0, 0, 5, 32, 0, _n);
        } public class PARAM_REQUEST_LIST : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal PARAM_REQUEST_LIST() : base(meta21, 0) { }
            internal PARAM_REQUEST_LIST(int bytes) : base(meta21, bytes) { }
            public byte target_system //System I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }
            static readonly Meta meta21 = new Meta(21, 0, 0, 0, 2, 16);
        } public class PARAM_VALUE : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal PARAM_VALUE() : base(meta22, 0) { }
            internal PARAM_VALUE(int bytes) : base(meta22, bytes) { }
            public ushort param_count //Total number of onboard parameter
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort param_index //Index of this onboard paramete
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public float param_value //Onboard parameter valu
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 4);}
            }

            public MAV_PARAM_TYPE param_type //Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types
            {
                get {  return (MAV_PARAM_TYPE)(1 +  BitUtils.get_bits(data, 64, 4));}
                set {  BitUtils.set_bits((ulong)(- 1 +   value), 4, data, 64);}
            }
            public string param_id_TRY(Inside ph)
            {
                if(ph.field_bit !=  68 && !try_visit_field(ph, 68)  ||  !try_visit_item(ph, 0)) return null;
                return new string(param_id_GET(ph, new char[ph.items], 0));
            }
            public char[]param_id_GET(Inside ph, char[] dst_ch, int pos)
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int param_id_LEN(Inside ph)
            {
                return (ph.field_bit !=  68 && !try_visit_field(ph, 68)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            } public void param_id_SET(string src, Inside ph) {param_id_SET(src.ToCharArray(), 0, src.Length, ph);} public void param_id_SET(char[] src, int pos, int items, Inside ph)
            {
                if(ph.field_bit != 68 && insert_field(ph, 68, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            } static readonly Meta meta22 = new Meta(22, 2, 0, 0, 10, 68, 0, _T);
        } public class PARAM_SET : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal PARAM_SET() : base(meta23, 0) { }
            internal PARAM_SET(int bytes) : base(meta23, bytes) { }
            public byte target_system //System I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            public float param_value //Onboard parameter valu
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  2, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 2);}
            }

            public MAV_PARAM_TYPE param_type //Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types
            {
                get {  return (MAV_PARAM_TYPE)(1 +  BitUtils.get_bits(data, 48, 4));}
                set {  BitUtils.set_bits((ulong)(- 1 +   value), 4, data, 48);}
            }
            public string param_id_TRY(Inside ph)
            {
                if(ph.field_bit !=  52 && !try_visit_field(ph, 52)  ||  !try_visit_item(ph, 0)) return null;
                return new string(param_id_GET(ph, new char[ph.items], 0));
            }
            public char[]param_id_GET(Inside ph, char[] dst_ch, int pos)
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int param_id_LEN(Inside ph)
            {
                return (ph.field_bit !=  52 && !try_visit_field(ph, 52)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            } public void param_id_SET(string src, Inside ph) {param_id_SET(src.ToCharArray(), 0, src.Length, ph);} public void param_id_SET(char[] src, int pos, int items, Inside ph)
            {
                if(ph.field_bit != 52 && insert_field(ph, 52, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            } static readonly Meta meta23 = new Meta(23, 0, 0, 0, 8, 52, 0, _H);
        } public class GPS_RAW_INT : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal GPS_RAW_INT() : base(meta24, 0) { }
            internal GPS_RAW_INT(int bytes) : base(meta24, bytes) { }
            public ushort eph //GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MA
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort epv //GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MA
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public ushort vel //GPS ground speed (m/s * 100). If unknown, set to: UINT16_MA
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  4);}
            }

            public ushort cog
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  6, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  6);}
            }

            public ulong time_usec //Timestamp (microseconds since UNIX epoch or microseconds since system boot
            {
                get {  return (BitUtils.get_bytes(data,  8, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  8);}
            }

            public int lat //Latitude (WGS84, EGM96 ellipsoid), in degrees * 1E
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  16, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  16);}
            }

            public int lon //Longitude (WGS84, EGM96 ellipsoid), in degrees * 1E
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  20, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  20);}
            }

            public int alt
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  24, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  24);}
            }

            public byte satellites_visible //Number of satellites visible. If unknown, set to 25
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  28, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  28);}
            }

            public GPS_FIX_TYPE fix_type //See the GPS_FIX_TYPE enum
            {
                get {  return (GPS_FIX_TYPE)(0 +  BitUtils.get_bits(data, 232, 4));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 232);}
            }
            public int alt_ellipsoid_TRY(Inside ph)//Altitude (above WGS84, EGM96 ellipsoid), in meters * 1000 (positive for up)
            {
                if(ph.field_bit !=  236 && !try_visit_field(ph, 236)) return 0;
                return (int)((int) BitUtils.get_bytes(data,  ph.BYTE, 4));
            }
            public void alt_ellipsoid_SET(int src, Inside ph)//Altitude (above WGS84, EGM96 ellipsoid), in meters * 1000 (positive for up)
            {
                if(ph.field_bit != 236)insert_field(ph, 236, 0);
                BitUtils.set_bytes((uint)(src), 4, data,  ph.BYTE);
            } public uint h_acc_TRY(Inside ph) //Position uncertainty in meters * 1000 (positive for up)
            {
                if(ph.field_bit !=  237 && !try_visit_field(ph, 237)) return 0;
                return (uint)((uint) BitUtils.get_bytes(data,  ph.BYTE, 4));
            }
            public void h_acc_SET(uint src, Inside ph)//Position uncertainty in meters * 1000 (positive for up)
            {
                if(ph.field_bit != 237)insert_field(ph, 237, 0);
                BitUtils.set_bytes((ulong)(src), 4, data,  ph.BYTE);
            } public uint v_acc_TRY(Inside ph) //Altitude uncertainty in meters * 1000 (positive for up)
            {
                if(ph.field_bit !=  238 && !try_visit_field(ph, 238)) return 0;
                return (uint)((uint) BitUtils.get_bytes(data,  ph.BYTE, 4));
            }
            public void v_acc_SET(uint src, Inside ph)//Altitude uncertainty in meters * 1000 (positive for up)
            {
                if(ph.field_bit != 238)insert_field(ph, 238, 0);
                BitUtils.set_bytes((ulong)(src), 4, data,  ph.BYTE);
            } public uint vel_acc_TRY(Inside ph) //Speed uncertainty in meters * 1000 (positive for up)
            {
                if(ph.field_bit !=  239 && !try_visit_field(ph, 239)) return 0;
                return (uint)((uint) BitUtils.get_bytes(data,  ph.BYTE, 4));
            }
            public void vel_acc_SET(uint src, Inside ph)//Speed uncertainty in meters * 1000 (positive for up)
            {
                if(ph.field_bit != 239)insert_field(ph, 239, 0);
                BitUtils.set_bytes((ulong)(src), 4, data,  ph.BYTE);
            } public uint hdg_acc_TRY(Inside ph) //Heading / track uncertainty in degrees * 1e5
            {
                if(ph.field_bit !=  240 && !try_visit_field(ph, 240)) return 0;
                return (uint)((uint) BitUtils.get_bytes(data,  ph.BYTE, 4));
            }
            public void hdg_acc_SET(uint src, Inside ph)//Heading / track uncertainty in degrees * 1e5
            {
                if(ph.field_bit != 240)insert_field(ph, 240, 0);
                BitUtils.set_bytes((ulong)(src), 4, data,  ph.BYTE);
            } static readonly Meta meta24 = new Meta(24, 4, 0, 1, 31, 236, 0, _tA, _EA, _KA, _kA, _xA);
        } public class GPS_STATUS : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal GPS_STATUS() : base(meta25, 0) { }
            internal GPS_STATUS(int bytes) : base(meta25, bytes) { }
            public byte satellites_visible //Number of satellites visibl
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte[] satellite_prn //Global satellite I
            {
                get {return satellite_prn_GET(new byte[20], 0);}
                set {satellite_prn_SET(value, 0)  ;}
            }
            public byte[]satellite_prn_GET(byte[] dst_ch, int pos)  //Global satellite I
            {
                for(int BYTE = 1, dst_max = pos + 20; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }

            public void satellite_prn_SET(byte[] src, int pos)  //Global satellite I
            {
                for(int BYTE =  1, src_max = pos + 20; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }


            public byte[] satellite_used //0: Satellite not used, 1: used for localizatio
            {
                get {return satellite_used_GET(new byte[20], 0);}
                set {satellite_used_SET(value, 0)  ;}
            }
            public byte[]satellite_used_GET(byte[] dst_ch, int pos)  //0: Satellite not used, 1: used for localizatio
            {
                for(int BYTE = 21, dst_max = pos + 20; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }

            public void satellite_used_SET(byte[] src, int pos)  //0: Satellite not used, 1: used for localizatio
            {
                for(int BYTE =  21, src_max = pos + 20; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }


            public byte[] satellite_elevation //Elevation (0: right on top of receiver, 90: on the horizon) of satellit
            {
                get {return satellite_elevation_GET(new byte[20], 0);}
                set {satellite_elevation_SET(value, 0)  ;}
            }
            public byte[]satellite_elevation_GET(byte[] dst_ch, int pos)  //Elevation (0: right on top of receiver, 90: on the horizon) of satellit
            {
                for(int BYTE = 41, dst_max = pos + 20; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }

            public void satellite_elevation_SET(byte[] src, int pos)  //Elevation (0: right on top of receiver, 90: on the horizon) of satellit
            {
                for(int BYTE =  41, src_max = pos + 20; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }


            public byte[] satellite_azimuth //Direction of satellite, 0: 0 deg, 255: 360 deg
            {
                get {return satellite_azimuth_GET(new byte[20], 0);}
                set {satellite_azimuth_SET(value, 0)  ;}
            }
            public byte[]satellite_azimuth_GET(byte[] dst_ch, int pos)  //Direction of satellite, 0: 0 deg, 255: 360 deg
            {
                for(int BYTE = 61, dst_max = pos + 20; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }

            public void satellite_azimuth_SET(byte[] src, int pos)  //Direction of satellite, 0: 0 deg, 255: 360 deg
            {
                for(int BYTE =  61, src_max = pos + 20; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }


            public byte[] satellite_snr //Signal to noise ratio of satellit
            {
                get {return satellite_snr_GET(new byte[20], 0);}
                set {satellite_snr_SET(value, 0)  ;}
            }
            public byte[]satellite_snr_GET(byte[] dst_ch, int pos)  //Signal to noise ratio of satellit
            {
                for(int BYTE = 81, dst_max = pos + 20; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }

            public void satellite_snr_SET(byte[] src, int pos)  //Signal to noise ratio of satellit
            {
                for(int BYTE =  81, src_max = pos + 20; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }

            static readonly Meta meta25 = new Meta(25, 0, 0, 0, 101, 808);
        } public class SCALED_IMU : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal SCALED_IMU() : base(meta26, 0) { }
            internal SCALED_IMU(int bytes) : base(meta26, bytes) { }
            public uint time_boot_ms //Timestamp (milliseconds since system boot
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public short xacc //X acceleration (mg
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  4, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  4);}
            }

            public short yacc //Y acceleration (mg
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  6, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  6);}
            }

            public short zacc //Z acceleration (mg
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  8, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  8);}
            }

            public short xgyro //Angular speed around X axis (millirad /sec
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  10, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  10);}
            }

            public short ygyro //Angular speed around Y axis (millirad /sec
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  12, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  12);}
            }

            public short zgyro //Angular speed around Z axis (millirad /sec
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  14, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  14);}
            }

            public short xmag //X Magnetic field (milli tesla
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  16, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  16);}
            }

            public short ymag //Y Magnetic field (milli tesla
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  18, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  18);}
            }

            public short zmag //Z Magnetic field (milli tesla
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  20, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  20);}
            }
            static readonly Meta meta26 = new Meta(26, 0, 1, 0, 22, 176);
        } public class RAW_IMU : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal RAW_IMU() : base(meta27, 0) { }
            internal RAW_IMU(int bytes) : base(meta27, bytes) { }
            public ulong time_usec //Timestamp (microseconds since UNIX epoch or microseconds since system boot
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public short xacc //X acceleration (raw
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  8, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  8);}
            }

            public short yacc //Y acceleration (raw
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  10, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  10);}
            }

            public short zacc //Z acceleration (raw
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  12, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  12);}
            }

            public short xgyro //Angular speed around X axis (raw
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  14, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  14);}
            }

            public short ygyro //Angular speed around Y axis (raw
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  16, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  16);}
            }

            public short zgyro //Angular speed around Z axis (raw
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  18, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  18);}
            }

            public short xmag //X Magnetic field (raw
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  20, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  20);}
            }

            public short ymag //Y Magnetic field (raw
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  22, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  22);}
            }

            public short zmag //Z Magnetic field (raw
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  24, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  24);}
            }
            static readonly Meta meta27 = new Meta(27, 0, 0, 1, 26, 208);
        } public class RAW_PRESSURE : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal RAW_PRESSURE() : base(meta28, 0) { }
            internal RAW_PRESSURE(int bytes) : base(meta28, bytes) { }
            public ulong time_usec //Timestamp (microseconds since UNIX epoch or microseconds since system boot
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public short press_abs //Absolute pressure (raw
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  8, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  8);}
            }

            public short press_diff1 //Differential pressure 1 (raw, 0 if nonexistant
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  10, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  10);}
            }

            public short press_diff2 //Differential pressure 2 (raw, 0 if nonexistant
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  12, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  12);}
            }

            public short temperature //Raw Temperature measurement (raw
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  14, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  14);}
            }
            static readonly Meta meta28 = new Meta(28, 0, 0, 1, 16, 128);
        } public class SCALED_PRESSURE : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal SCALED_PRESSURE() : base(meta29, 0) { }
            internal SCALED_PRESSURE(int bytes) : base(meta29, bytes) { }
            public uint time_boot_ms //Timestamp (milliseconds since system boot
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public float press_abs //Absolute pressure (hectopascal
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 4);}
            }

            public float press_diff //Differential pressure 1 (hectopascal
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public short temperature //Temperature measurement (0.01 degrees celsius
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  12, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  12);}
            }
            static readonly Meta meta29 = new Meta(29, 0, 1, 0, 14, 112);
        } public class ATTITUDE : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal ATTITUDE() : base(meta30, 0) { }
            internal ATTITUDE(int bytes) : base(meta30, bytes) { }
            public uint time_boot_ms //Timestamp (milliseconds since system boot
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public float roll //Roll angle (rad, -pi..+pi
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 4);}
            }

            public float pitch //Pitch angle (rad, -pi..+pi
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float yaw //Yaw angle (rad, -pi..+pi
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float rollspeed //Roll angular speed (rad/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float pitchspeed //Pitch angular speed (rad/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float yawspeed //Yaw angular speed (rad/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }
            static readonly Meta meta30 = new Meta(30, 0, 1, 0, 28, 224);
        } public class ATTITUDE_QUATERNION : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal ATTITUDE_QUATERNION() : base(meta31, 0) { }
            internal ATTITUDE_QUATERNION(int bytes) : base(meta31, bytes) { }
            public uint time_boot_ms //Timestamp (milliseconds since system boot
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public float q1 //Quaternion component 1, w (1 in null-rotation
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 4);}
            }

            public float q2 //Quaternion component 2, x (0 in null-rotation
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float q3 //Quaternion component 3, y (0 in null-rotation
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float q4 //Quaternion component 4, z (0 in null-rotation
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float rollspeed //Roll angular speed (rad/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float pitchspeed //Pitch angular speed (rad/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float yawspeed //Yaw angular speed (rad/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }
            static readonly Meta meta31 = new Meta(31, 0, 1, 0, 32, 256);
        } public class LOCAL_POSITION_NED : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal LOCAL_POSITION_NED() : base(meta32, 0) { }
            internal LOCAL_POSITION_NED(int bytes) : base(meta32, bytes) { }
            public uint time_boot_ms //Timestamp (milliseconds since system boot
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public float x //X Positio
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 4);}
            }

            public float y //Y Positio
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float z //Z Positio
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float vx //X Spee
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float vy //Y Spee
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float vz //Z Spee
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }
            static readonly Meta meta32 = new Meta(32, 0, 1, 0, 28, 224);
        } public class GLOBAL_POSITION_INT : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal GLOBAL_POSITION_INT() : base(meta33, 0) { }
            internal GLOBAL_POSITION_INT(int bytes) : base(meta33, bytes) { }
            public ushort hdg //Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MA
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public uint time_boot_ms //Timestamp (milliseconds since system boot
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  2, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  2);}
            }

            public int lat //Latitude, expressed as degrees * 1E
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  6, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  6);}
            }

            public int lon //Longitude, expressed as degrees * 1E
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  10, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  10);}
            }

            public int alt
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  14, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  14);}
            }

            public int relative_alt //Altitude above ground in meters, expressed as * 1000 (millimeters
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  18, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  18);}
            }

            public short vx //Ground X Speed (Latitude, positive north), expressed as m/s * 10
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  22, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  22);}
            }

            public short vy //Ground Y Speed (Longitude, positive east), expressed as m/s * 10
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  24, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  24);}
            }

            public short vz //Ground Z Speed (Altitude, positive down), expressed as m/s * 10
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  26, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  26);}
            }
            static readonly Meta meta33 = new Meta(33, 1, 1, 0, 28, 224);
        } public class RC_CHANNELS_SCALED : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal RC_CHANNELS_SCALED() : base(meta34, 0) { }
            internal RC_CHANNELS_SCALED(int bytes) : base(meta34, bytes) { }
            public uint time_boot_ms //Timestamp (milliseconds since system boot
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public byte port
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  4);}
            }

            public short chan1_scaled //RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  5, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  5);}
            }

            public short chan2_scaled //RC channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  7, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  7);}
            }

            public short chan3_scaled //RC channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  9, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  9);}
            }

            public short chan4_scaled //RC channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  11, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  11);}
            }

            public short chan5_scaled //RC channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  13, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  13);}
            }

            public short chan6_scaled //RC channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  15, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  15);}
            }

            public short chan7_scaled //RC channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  17, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  17);}
            }

            public short chan8_scaled //RC channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  19, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  19);}
            }

            public byte rssi //Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  21, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  21);}
            }
            static readonly Meta meta34 = new Meta(34, 0, 1, 0, 22, 176);
        } public class RC_CHANNELS_RAW : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal RC_CHANNELS_RAW() : base(meta35, 0) { }
            internal RC_CHANNELS_RAW(int bytes) : base(meta35, bytes) { }
            public ushort chan1_raw //RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort chan2_raw //RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public ushort chan3_raw //RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  4);}
            }

            public ushort chan4_raw //RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  6, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  6);}
            }

            public ushort chan5_raw //RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  8, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  8);}
            }

            public ushort chan6_raw //RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  10, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  10);}
            }

            public ushort chan7_raw //RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  12, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  12);}
            }

            public ushort chan8_raw //RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  14, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  14);}
            }

            public uint time_boot_ms //Timestamp (milliseconds since system boot
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  16, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  16);}
            }

            public byte port
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  20, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  20);}
            }

            public byte rssi //Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  21, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  21);}
            }
            static readonly Meta meta35 = new Meta(35, 8, 1, 0, 22, 176);
        } public class SERVO_OUTPUT_RAW : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal SERVO_OUTPUT_RAW() : base(meta36, 0) { }
            internal SERVO_OUTPUT_RAW(int bytes) : base(meta36, bytes) { }
            public ushort servo1_raw //Servo output 1 value, in microsecond
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort servo2_raw //Servo output 2 value, in microsecond
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public ushort servo3_raw //Servo output 3 value, in microsecond
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  4);}
            }

            public ushort servo4_raw //Servo output 4 value, in microsecond
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  6, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  6);}
            }

            public ushort servo5_raw //Servo output 5 value, in microsecond
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  8, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  8);}
            }

            public ushort servo6_raw //Servo output 6 value, in microsecond
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  10, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  10);}
            }

            public ushort servo7_raw //Servo output 7 value, in microsecond
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  12, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  12);}
            }

            public ushort servo8_raw //Servo output 8 value, in microsecond
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  14, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  14);}
            }

            public uint time_usec //Timestamp (microseconds since system boot
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  16, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  16);}
            }

            public byte port
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  20, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  20);}
            }
            public ushort servo9_raw_TRY(Inside ph)//Servo output 9 value, in microsecond
            {
                if(ph.field_bit !=  168 && !try_visit_field(ph, 168)) return 0;
                return (ushort)((ushort) BitUtils.get_bytes(data,  ph.BYTE, 2));
            }
            public void servo9_raw_SET(ushort src, Inside ph)//Servo output 9 value, in microsecond
            {
                if(ph.field_bit != 168)insert_field(ph, 168, 0);
                BitUtils.set_bytes((ulong)(src), 2, data,  ph.BYTE);
            } public ushort servo10_raw_TRY(Inside ph) //Servo output 10 value, in microsecond
            {
                if(ph.field_bit !=  169 && !try_visit_field(ph, 169)) return 0;
                return (ushort)((ushort) BitUtils.get_bytes(data,  ph.BYTE, 2));
            }
            public void servo10_raw_SET(ushort src, Inside ph)//Servo output 10 value, in microsecond
            {
                if(ph.field_bit != 169)insert_field(ph, 169, 0);
                BitUtils.set_bytes((ulong)(src), 2, data,  ph.BYTE);
            } public ushort servo11_raw_TRY(Inside ph) //Servo output 11 value, in microsecond
            {
                if(ph.field_bit !=  170 && !try_visit_field(ph, 170)) return 0;
                return (ushort)((ushort) BitUtils.get_bytes(data,  ph.BYTE, 2));
            }
            public void servo11_raw_SET(ushort src, Inside ph)//Servo output 11 value, in microsecond
            {
                if(ph.field_bit != 170)insert_field(ph, 170, 0);
                BitUtils.set_bytes((ulong)(src), 2, data,  ph.BYTE);
            } public ushort servo12_raw_TRY(Inside ph) //Servo output 12 value, in microsecond
            {
                if(ph.field_bit !=  171 && !try_visit_field(ph, 171)) return 0;
                return (ushort)((ushort) BitUtils.get_bytes(data,  ph.BYTE, 2));
            }
            public void servo12_raw_SET(ushort src, Inside ph)//Servo output 12 value, in microsecond
            {
                if(ph.field_bit != 171)insert_field(ph, 171, 0);
                BitUtils.set_bytes((ulong)(src), 2, data,  ph.BYTE);
            } public ushort servo13_raw_TRY(Inside ph) //Servo output 13 value, in microsecond
            {
                if(ph.field_bit !=  172 && !try_visit_field(ph, 172)) return 0;
                return (ushort)((ushort) BitUtils.get_bytes(data,  ph.BYTE, 2));
            }
            public void servo13_raw_SET(ushort src, Inside ph)//Servo output 13 value, in microsecond
            {
                if(ph.field_bit != 172)insert_field(ph, 172, 0);
                BitUtils.set_bytes((ulong)(src), 2, data,  ph.BYTE);
            } public ushort servo14_raw_TRY(Inside ph) //Servo output 14 value, in microsecond
            {
                if(ph.field_bit !=  173 && !try_visit_field(ph, 173)) return 0;
                return (ushort)((ushort) BitUtils.get_bytes(data,  ph.BYTE, 2));
            }
            public void servo14_raw_SET(ushort src, Inside ph)//Servo output 14 value, in microsecond
            {
                if(ph.field_bit != 173)insert_field(ph, 173, 0);
                BitUtils.set_bytes((ulong)(src), 2, data,  ph.BYTE);
            } public ushort servo15_raw_TRY(Inside ph) //Servo output 15 value, in microsecond
            {
                if(ph.field_bit !=  174 && !try_visit_field(ph, 174)) return 0;
                return (ushort)((ushort) BitUtils.get_bytes(data,  ph.BYTE, 2));
            }
            public void servo15_raw_SET(ushort src, Inside ph)//Servo output 15 value, in microsecond
            {
                if(ph.field_bit != 174)insert_field(ph, 174, 0);
                BitUtils.set_bytes((ulong)(src), 2, data,  ph.BYTE);
            } public ushort servo16_raw_TRY(Inside ph) //Servo output 16 value, in microsecond
            {
                if(ph.field_bit !=  175 && !try_visit_field(ph, 175)) return 0;
                return (ushort)((ushort) BitUtils.get_bytes(data,  ph.BYTE, 2));
            }
            public void servo16_raw_SET(ushort src, Inside ph)//Servo output 16 value, in microsecond
            {
                if(ph.field_bit != 175)insert_field(ph, 175, 0);
                BitUtils.set_bytes((ulong)(src), 2, data,  ph.BYTE);
            } static readonly Meta meta36 = new Meta(36, 8, 1, 0, 22, 168, 0, _IJ, _tJ, _EJ, _KJ, _kJ, _xJ, _mJ, _rJ);
        } public class MISSION_REQUEST_PARTIAL_LIST : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal MISSION_REQUEST_PARTIAL_LIST() : base(meta37, 0) { }
            internal MISSION_REQUEST_PARTIAL_LIST(int bytes) : base(meta37, bytes) { }
            public byte target_system //System I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            public short start_index //Start index, 0 by defaul
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  2, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  2);}
            }

            public short end_index //End index, -1 by default (-1: send list to end). Else a valid index of the lis
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  4, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  4);}
            }

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYP
            {
                get
                {
                    switch((int)BitUtils.get_bits(data, 48, 3))
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
            static readonly Meta meta37 = new Meta(37, 0, 0, 0, 7, 51);
        } public class MISSION_WRITE_PARTIAL_LIST : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal MISSION_WRITE_PARTIAL_LIST() : base(meta38, 0) { }
            internal MISSION_WRITE_PARTIAL_LIST(int bytes) : base(meta38, bytes) { }
            public byte target_system //System I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            public short start_index //Start index, 0 by default and smaller / equal to the largest index of the current onboard list
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  2, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  2);}
            }

            public short end_index //End index, equal or greater than start index
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  4, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  4);}
            }

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYP
            {
                get
                {
                    switch((int)BitUtils.get_bits(data, 48, 3))
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
            static readonly Meta meta38 = new Meta(38, 0, 0, 0, 7, 51);
        } public class MISSION_ITEM : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal MISSION_ITEM() : base(meta39, 0) { }
            internal MISSION_ITEM(int bytes) : base(meta39, bytes) { }
            public ushort seq //Sequenc
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public byte target_system //System I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }

            public byte target_component //Component I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  3);}
            }

            public byte current //false:0, true:
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  4);}
            }

            public byte autocontinue //autocontinue to next w
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  5);}
            }

            public float param1 //PARAM1, see MAV_CMD enu
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  6, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 6);}
            }

            public float param2 //PARAM2, see MAV_CMD enu
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  10, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 10);}
            }

            public float param3 //PARAM3, see MAV_CMD enu
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  14, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 14);}
            }

            public float param4 //PARAM4, see MAV_CMD enu
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  18, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 18);}
            }

            public float x //PARAM5 / local: x position, global: latitud
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  22, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 22);}
            }

            public float y //PARAM6 / y position: global: longitud
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  26, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 26);}
            }

            public float z //PARAM7 / z position: global: altitude (relative or absolute, depending on frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  30, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 30);}
            }

            public MAV_FRAME frame //The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.
            {
                get {  return (MAV_FRAME)(0 +  BitUtils.get_bits(data, 272, 4));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 272);}
            }

            public MAV_CMD command //The scheduled action for the waypoint. see MAV_CMD in common.xml MAVLink spec
            {
                get
                {
                    switch((int)BitUtils.get_bits(data, 276, 7))
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
                    }
                    throw  new ArgumentException("Unknown enum ID ");
                }
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

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYP
            {
                get
                {
                    switch((int)BitUtils.get_bits(data, 283, 3))
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
            static readonly Meta meta39 = new Meta(39, 1, 0, 0, 36, 286);
        } public class MISSION_REQUEST : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal MISSION_REQUEST() : base(meta40, 0) { }
            internal MISSION_REQUEST(int bytes) : base(meta40, bytes) { }
            public ushort seq //Sequenc
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public byte target_system //System I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }

            public byte target_component //Component I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  3);}
            }

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYP
            {
                get
                {
                    switch((int)BitUtils.get_bits(data, 32, 3))
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
            static readonly Meta meta40 = new Meta(40, 1, 0, 0, 5, 35);
        } public class MISSION_SET_CURRENT : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal MISSION_SET_CURRENT() : base(meta41, 0) { }
            internal MISSION_SET_CURRENT(int bytes) : base(meta41, bytes) { }
            public ushort seq //Sequenc
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public byte target_system //System I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }

            public byte target_component //Component I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  3);}
            }
            static readonly Meta meta41 = new Meta(41, 1, 0, 0, 4, 32);
        } public class MISSION_CURRENT : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal MISSION_CURRENT() : base(meta42, 0) { }
            internal MISSION_CURRENT(int bytes) : base(meta42, bytes) { }
            public ushort seq //Sequenc
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }
            static readonly Meta meta42 = new Meta(42, 1, 0, 0, 2, 16);
        } public class MISSION_REQUEST_LIST : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal MISSION_REQUEST_LIST() : base(meta43, 0) { }
            internal MISSION_REQUEST_LIST(int bytes) : base(meta43, bytes) { }
            public byte target_system //System I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYP
            {
                get
                {
                    switch((int)BitUtils.get_bits(data, 16, 3))
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
            static readonly Meta meta43 = new Meta(43, 0, 0, 0, 3, 19);
        } public class MISSION_COUNT : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal MISSION_COUNT() : base(meta44, 0) { }
            internal MISSION_COUNT(int bytes) : base(meta44, bytes) { }
            public ushort count //Number of mission items in the sequenc
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public byte target_system //System I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }

            public byte target_component //Component I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  3);}
            }

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYP
            {
                get
                {
                    switch((int)BitUtils.get_bits(data, 32, 3))
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
            static readonly Meta meta44 = new Meta(44, 1, 0, 0, 5, 35);
        } public class MISSION_CLEAR_ALL : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal MISSION_CLEAR_ALL() : base(meta45, 0) { }
            internal MISSION_CLEAR_ALL(int bytes) : base(meta45, bytes) { }
            public byte target_system //System I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYP
            {
                get
                {
                    switch((int)BitUtils.get_bits(data, 16, 3))
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
            static readonly Meta meta45 = new Meta(45, 0, 0, 0, 3, 19);
        } public class MISSION_ITEM_REACHED : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal MISSION_ITEM_REACHED() : base(meta46, 0) { }
            internal MISSION_ITEM_REACHED(int bytes) : base(meta46, bytes) { }
            public ushort seq //Sequenc
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }
            static readonly Meta meta46 = new Meta(46, 1, 0, 0, 2, 16);
        } public class MISSION_ACK : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal MISSION_ACK() : base(meta47, 0) { }
            internal MISSION_ACK(int bytes) : base(meta47, bytes) { }
            public byte target_system //System I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            public MAV_MISSION_RESULT type //See MAV_MISSION_RESULT enu
            {
                get {  return (MAV_MISSION_RESULT)(0 +  BitUtils.get_bits(data, 16, 4));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 16);}
            }

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYP
            {
                get
                {
                    switch((int)BitUtils.get_bits(data, 20, 3))
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
            static readonly Meta meta47 = new Meta(47, 0, 0, 0, 3, 23);
        } public class SET_GPS_GLOBAL_ORIGIN : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal SET_GPS_GLOBAL_ORIGIN() : base(meta48, 0) { }
            internal SET_GPS_GLOBAL_ORIGIN(int bytes) : base(meta48, bytes) { }
            public byte target_system //System I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public int latitude //Latitude (WGS84), in degrees * 1E
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  1, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  1);}
            }

            public int longitude //Longitude (WGS84, in degrees * 1E
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  5, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  5);}
            }

            public int altitude //Altitude (AMSL), in meters * 1000 (positive for up
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  9, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  9);}
            }
            public ulong time_usec_TRY(Inside ph)//Timestamp (microseconds since UNIX epoch or microseconds since system boot
            {
                if(ph.field_bit !=  104 && !try_visit_field(ph, 104)) return 0;
                return (BitUtils.get_bytes(data,  ph.BYTE, 8));
            }
            public void time_usec_SET(ulong src, Inside ph)//Timestamp (microseconds since UNIX epoch or microseconds since system boot
            {
                if(ph.field_bit != 104)insert_field(ph, 104, 0);
                BitUtils.set_bytes((ulong)(src), 8, data,  ph.BYTE);
            } static readonly Meta meta48 = new Meta(48, 0, 0, 0, 14, 104, 0, _ib);
        } public class GPS_GLOBAL_ORIGIN : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal GPS_GLOBAL_ORIGIN() : base(meta49, 0) { }
            internal GPS_GLOBAL_ORIGIN(int bytes) : base(meta49, bytes) { }
            public int latitude //Latitude (WGS84), in degrees * 1E
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  0);}
            }

            public int longitude //Longitude (WGS84), in degrees * 1E
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  4, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  4);}
            }

            public int altitude //Altitude (AMSL), in meters * 1000 (positive for up
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  8, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  8);}
            }
            public ulong time_usec_TRY(Inside ph)//Timestamp (microseconds since UNIX epoch or microseconds since system boot
            {
                if(ph.field_bit !=  96 && !try_visit_field(ph, 96)) return 0;
                return (BitUtils.get_bytes(data,  ph.BYTE, 8));
            }
            public void time_usec_SET(ulong src, Inside ph)//Timestamp (microseconds since UNIX epoch or microseconds since system boot
            {
                if(ph.field_bit != 96)insert_field(ph, 96, 0);
                BitUtils.set_bytes((ulong)(src), 8, data,  ph.BYTE);
            } static readonly Meta meta49 = new Meta(49, 0, 0, 0, 13, 96, 0, _Lb);
        } public class PARAM_MAP_RC : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal PARAM_MAP_RC() : base(meta50, 0) { }
            internal PARAM_MAP_RC(int bytes) : base(meta50, bytes) { }
            public byte target_system //System I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            public short param_index
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  2, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  2);}
            }

            public byte parameter_rc_channel_index
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  4);}
            }

            public float param_value0 //Initial parameter valu
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  5, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 5);}
            }

            public float scale //Scale, maps the RC range [-1, 1] to a parameter valu
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  9, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 9);}
            }

            public float param_value_min
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  13, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 13);}
            }

            public float param_value_max
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  17, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 17);}
            }
            public string param_id_TRY(Inside ph)
            {
                if(ph.field_bit !=  168 && !try_visit_field(ph, 168)  ||  !try_visit_item(ph, 0)) return null;
                return new string(param_id_GET(ph, new char[ph.items], 0));
            }
            public char[]param_id_GET(Inside ph, char[] dst_ch, int pos)
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int param_id_LEN(Inside ph)
            {
                return (ph.field_bit !=  168 && !try_visit_field(ph, 168)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            } public void param_id_SET(string src, Inside ph) {param_id_SET(src.ToCharArray(), 0, src.Length, ph);} public void param_id_SET(char[] src, int pos, int items, Inside ph)
            {
                if(ph.field_bit != 168 && insert_field(ph, 168, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            } static readonly Meta meta50 = new Meta(50, 0, 0, 0, 22, 168, 0, _hb);
        } public class MISSION_REQUEST_INT : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal MISSION_REQUEST_INT() : base(meta51, 0) { }
            internal MISSION_REQUEST_INT(int bytes) : base(meta51, bytes) { }
            public ushort seq //Sequenc
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public byte target_system //System I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }

            public byte target_component //Component I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  3);}
            }

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYP
            {
                get
                {
                    switch((int)BitUtils.get_bits(data, 32, 3))
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
            static readonly Meta meta51 = new Meta(51, 1, 0, 0, 5, 35);
        } public class SAFETY_SET_ALLOWED_AREA : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal SAFETY_SET_ALLOWED_AREA() : base(meta54, 0) { }
            internal SAFETY_SET_ALLOWED_AREA(int bytes) : base(meta54, bytes) { }
            public byte target_system //System I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            public float p1x //x position 1 / Latitude
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  2, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 2);}
            }

            public float p1y //y position 1 / Longitude
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  6, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 6);}
            }

            public float p1z //z position 1 / Altitude
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  10, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 10);}
            }

            public float p2x //x position 2 / Latitude
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  14, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 14);}
            }

            public float p2y //y position 2 / Longitude
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  18, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 18);}
            }

            public float p2z //z position 2 / Altitude
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  22, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 22);}
            }

            public MAV_FRAME frame
            {
                get {  return (MAV_FRAME)(0 +  BitUtils.get_bits(data, 208, 4));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 208);}
            }
            static readonly Meta meta54 = new Meta(54, 0, 0, 0, 27, 212);
        } public class SAFETY_ALLOWED_AREA : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal SAFETY_ALLOWED_AREA() : base(meta55, 0) { }
            internal SAFETY_ALLOWED_AREA(int bytes) : base(meta55, bytes) { }
            public float p1x //x position 1 / Latitude
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  0, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 0);}
            }

            public float p1y //y position 1 / Longitude
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 4);}
            }

            public float p1z //z position 1 / Altitude
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float p2x //x position 2 / Latitude
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float p2y //y position 2 / Longitude
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float p2z //z position 2 / Altitude
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public MAV_FRAME frame
            {
                get {  return (MAV_FRAME)(0 +  BitUtils.get_bits(data, 192, 4));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 192);}
            }
            static readonly Meta meta55 = new Meta(55, 0, 0, 0, 25, 196);
        } public class ATTITUDE_QUATERNION_COV : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal ATTITUDE_QUATERNION_COV() : base(meta61, 0) { }
            internal ATTITUDE_QUATERNION_COV(int bytes) : base(meta61, bytes) { }
            public ulong time_usec //Timestamp (microseconds since system boot or since UNIX epoch
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public float[] q //Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation
            {
                get {return q_GET(new float[4], 0);}
                set {q_SET(value, 0)  ;}
            }
            public float[]q_GET(float[] dst_ch, int pos)  //Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation
            {
                for(int BYTE = 8, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }

            public void q_SET(float[] src, int pos)  //Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation
            {
                for(int BYTE =  8, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }


            public float rollspeed //Roll angular speed (rad/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float pitchspeed //Pitch angular speed (rad/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }

            public float yawspeed //Yaw angular speed (rad/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  32, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 32);}
            }

            public float[] covariance //Attitude covarianc
            {
                get {return covariance_GET(new float[9], 0);}
                set {covariance_SET(value, 0)  ;}
            }
            public float[]covariance_GET(float[] dst_ch, int pos)  //Attitude covarianc
            {
                for(int BYTE = 36, dst_max = pos + 9; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }

            public void covariance_SET(float[] src, int pos)  //Attitude covarianc
            {
                for(int BYTE =  36, src_max = pos + 9; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }

            static readonly Meta meta61 = new Meta(61, 0, 0, 1, 72, 576);
        } public class NAV_CONTROLLER_OUTPUT : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal NAV_CONTROLLER_OUTPUT() : base(meta62, 0) { }
            internal NAV_CONTROLLER_OUTPUT(int bytes) : base(meta62, bytes) { }
            public ushort wp_dist //Distance to active waypoint in meter
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public float nav_roll //Current desired roll in degree
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  2, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 2);}
            }

            public float nav_pitch //Current desired pitch in degree
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  6, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 6);}
            }

            public short nav_bearing //Current desired heading in degree
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  10, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  10);}
            }

            public short target_bearing //Bearing to current waypoint/target in degree
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  12, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  12);}
            }

            public float alt_error //Current altitude error in meter
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  14, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 14);}
            }

            public float aspd_error //Current airspeed error in meters/secon
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  18, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 18);}
            }

            public float xtrack_error //Current crosstrack error on x-y plane in meter
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  22, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 22);}
            }
            static readonly Meta meta62 = new Meta(62, 1, 0, 0, 26, 208);
        } public class GLOBAL_POSITION_INT_COV : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal GLOBAL_POSITION_INT_COV() : base(meta63, 0) { }
            internal GLOBAL_POSITION_INT_COV(int bytes) : base(meta63, bytes) { }
            public ulong time_usec //Timestamp (microseconds since system boot or since UNIX epoch
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public int lat //Latitude, expressed as degrees * 1E
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  8, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  8);}
            }

            public int lon //Longitude, expressed as degrees * 1E
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  12, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  12);}
            }

            public int alt //Altitude in meters, expressed as * 1000 (millimeters), above MS
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  16, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  16);}
            }

            public int relative_alt //Altitude above ground in meters, expressed as * 1000 (millimeters
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  20, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  20);}
            }

            public float vx //Ground X Speed (Latitude), expressed as m/
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float vy //Ground Y Speed (Longitude), expressed as m/
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }

            public float vz //Ground Z Speed (Altitude), expressed as m/
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  32, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 32);}
            }

            public float[] covariance //Covariance matrix (first six entries are the first ROW, next six entries are the second row, etc.
            {
                get {return covariance_GET(new float[36], 0);}
                set {covariance_SET(value, 0)  ;}
            }
            public float[]covariance_GET(float[] dst_ch, int pos)  //Covariance matrix (first six entries are the first ROW, next six entries are the second row, etc.
            {
                for(int BYTE = 36, dst_max = pos + 36; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }

            public void covariance_SET(float[] src, int pos)  //Covariance matrix (first six entries are the first ROW, next six entries are the second row, etc.
            {
                for(int BYTE =  36, src_max = pos + 36; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }


            public MAV_ESTIMATOR_TYPE estimator_type //Class id of the estimator this estimate originated from
            {
                get {  return (MAV_ESTIMATOR_TYPE)(1 +  BitUtils.get_bits(data, 1440, 3));}
                set {  BitUtils.set_bits((ulong)(- 1 +   value), 3, data, 1440);}
            }
            static readonly Meta meta63 = new Meta(63, 0, 0, 1, 181, 1443);
        } public class LOCAL_POSITION_NED_COV : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal LOCAL_POSITION_NED_COV() : base(meta64, 0) { }
            internal LOCAL_POSITION_NED_COV(int bytes) : base(meta64, bytes) { }
            public ulong time_usec //Timestamp (microseconds since system boot or since UNIX epoch
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public float x //X Positio
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float y //Y Positio
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float z //Z Positio
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float vx //X Speed (m/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float vy //Y Speed (m/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float vz //Z Speed (m/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }

            public float ax //X Acceleration (m/s^2
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  32, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 32);}
            }

            public float ay //Y Acceleration (m/s^2
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  36, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 36);}
            }

            public float az //Z Acceleration (m/s^2
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  40, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 40);}
            }

            public float[] covariance
            {
                get {return covariance_GET(new float[45], 0);}
                set {covariance_SET(value, 0)  ;}
            }
            public float[]covariance_GET(float[] dst_ch, int pos)
            {
                for(int BYTE = 44, dst_max = pos + 45; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }

            public void covariance_SET(float[] src, int pos)
            {
                for(int BYTE =  44, src_max = pos + 45; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }


            public MAV_ESTIMATOR_TYPE estimator_type //Class id of the estimator this estimate originated from
            {
                get {  return (MAV_ESTIMATOR_TYPE)(1 +  BitUtils.get_bits(data, 1792, 3));}
                set {  BitUtils.set_bits((ulong)(- 1 +   value), 3, data, 1792);}
            }
            static readonly Meta meta64 = new Meta(64, 0, 0, 1, 225, 1795);
        } public class RC_CHANNELS : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal RC_CHANNELS() : base(meta65, 0) { }
            internal RC_CHANNELS(int bytes) : base(meta65, bytes) { }
            public ushort chan1_raw //RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort chan2_raw //RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public ushort chan3_raw //RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  4);}
            }

            public ushort chan4_raw //RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  6, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  6);}
            }

            public ushort chan5_raw //RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  8, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  8);}
            }

            public ushort chan6_raw //RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  10, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  10);}
            }

            public ushort chan7_raw //RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  12, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  12);}
            }

            public ushort chan8_raw //RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  14, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  14);}
            }

            public ushort chan9_raw //RC channel 9 value, in microseconds. A value of UINT16_MAX implies the channel is unused
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  16, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  16);}
            }

            public ushort chan10_raw //RC channel 10 value, in microseconds. A value of UINT16_MAX implies the channel is unused
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  18, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  18);}
            }

            public ushort chan11_raw //RC channel 11 value, in microseconds. A value of UINT16_MAX implies the channel is unused
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  20, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  20);}
            }

            public ushort chan12_raw //RC channel 12 value, in microseconds. A value of UINT16_MAX implies the channel is unused
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  22, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  22);}
            }

            public ushort chan13_raw //RC channel 13 value, in microseconds. A value of UINT16_MAX implies the channel is unused
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  24, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  24);}
            }

            public ushort chan14_raw //RC channel 14 value, in microseconds. A value of UINT16_MAX implies the channel is unused
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  26, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  26);}
            }

            public ushort chan15_raw //RC channel 15 value, in microseconds. A value of UINT16_MAX implies the channel is unused
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  28, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  28);}
            }

            public ushort chan16_raw //RC channel 16 value, in microseconds. A value of UINT16_MAX implies the channel is unused
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  30, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  30);}
            }

            public ushort chan17_raw //RC channel 17 value, in microseconds. A value of UINT16_MAX implies the channel is unused
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  32, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  32);}
            }

            public ushort chan18_raw //RC channel 18 value, in microseconds. A value of UINT16_MAX implies the channel is unused
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  34, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  34);}
            }

            public uint time_boot_ms //Timestamp (milliseconds since system boot
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  36, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  36);}
            }

            public byte chancount
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  40, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  40);}
            }

            public byte rssi //Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  41, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  41);}
            }
            static readonly Meta meta65 = new Meta(65, 18, 1, 0, 42, 336);
        } public class REQUEST_DATA_STREAM : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal REQUEST_DATA_STREAM() : base(meta66, 0) { }
            internal REQUEST_DATA_STREAM(int bytes) : base(meta66, bytes) { }
            public ushort req_message_rate //The requested message rat
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public byte target_system //The target requested to send the message stream
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }

            public byte target_component //The target requested to send the message stream
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  3);}
            }

            public byte req_stream_id //The ID of the requested data strea
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  4);}
            }

            public byte start_stop //1 to start sending, 0 to stop sending
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  5);}
            }
            static readonly Meta meta66 = new Meta(66, 1, 0, 0, 6, 48);
        } public class DATA_STREAM : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal DATA_STREAM() : base(meta67, 0) { }
            internal DATA_STREAM(int bytes) : base(meta67, bytes) { }
            public ushort message_rate //The message rat
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public byte stream_id //The ID of the requested data strea
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }

            public byte on_off //1 stream is enabled, 0 stream is stopped
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  3);}
            }
            static readonly Meta meta67 = new Meta(67, 1, 0, 0, 4, 32);
        } public class MANUAL_CONTROL : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal MANUAL_CONTROL() : base(meta69, 0) { }
            internal MANUAL_CONTROL(int bytes) : base(meta69, bytes) { }
            public ushort buttons
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public byte target //The system to be controlled
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }

            public short x
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  3, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  3);}
            }

            public short y
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  5, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  5);}
            }

            public short z
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  7, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  7);}
            }

            public short r
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  9, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  9);}
            }
            static readonly Meta meta69 = new Meta(69, 1, 0, 0, 11, 88);
        } public class RC_CHANNELS_OVERRIDE : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal RC_CHANNELS_OVERRIDE() : base(meta70, 0) { }
            internal RC_CHANNELS_OVERRIDE(int bytes) : base(meta70, bytes) { }
            public ushort chan1_raw //RC channel 1 value, in microseconds. A value of UINT16_MAX means to ignore this field
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort chan2_raw //RC channel 2 value, in microseconds. A value of UINT16_MAX means to ignore this field
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public ushort chan3_raw //RC channel 3 value, in microseconds. A value of UINT16_MAX means to ignore this field
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  4);}
            }

            public ushort chan4_raw //RC channel 4 value, in microseconds. A value of UINT16_MAX means to ignore this field
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  6, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  6);}
            }

            public ushort chan5_raw //RC channel 5 value, in microseconds. A value of UINT16_MAX means to ignore this field
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  8, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  8);}
            }

            public ushort chan6_raw //RC channel 6 value, in microseconds. A value of UINT16_MAX means to ignore this field
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  10, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  10);}
            }

            public ushort chan7_raw //RC channel 7 value, in microseconds. A value of UINT16_MAX means to ignore this field
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  12, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  12);}
            }

            public ushort chan8_raw //RC channel 8 value, in microseconds. A value of UINT16_MAX means to ignore this field
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  14, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  14);}
            }

            public byte target_system //System I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  16, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  16);}
            }

            public byte target_component //Component I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  17, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  17);}
            }
            static readonly Meta meta70 = new Meta(70, 8, 0, 0, 18, 144);
        } public class MISSION_ITEM_INT : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal MISSION_ITEM_INT() : base(meta73, 0) { }
            internal MISSION_ITEM_INT(int bytes) : base(meta73, bytes) { }
            public ushort seq
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public byte target_system //System I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }

            public byte target_component //Component I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  3);}
            }

            public byte current //false:0, true:
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  4);}
            }

            public byte autocontinue //autocontinue to next w
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  5);}
            }

            public float param1 //PARAM1, see MAV_CMD enu
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  6, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 6);}
            }

            public float param2 //PARAM2, see MAV_CMD enu
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  10, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 10);}
            }

            public float param3 //PARAM3, see MAV_CMD enu
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  14, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 14);}
            }

            public float param4 //PARAM4, see MAV_CMD enu
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  18, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 18);}
            }

            public int x //PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  22, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  22);}
            }

            public int y //PARAM6 / y position: local: x position in meters * 1e4, global: longitude in degrees *10^
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  26, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  26);}
            }

            public float z //PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  30, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 30);}
            }

            public MAV_FRAME frame //The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.
            {
                get {  return (MAV_FRAME)(0 +  BitUtils.get_bits(data, 272, 4));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 272);}
            }

            public MAV_CMD command //The scheduled action for the waypoint. see MAV_CMD in common.xml MAVLink spec
            {
                get
                {
                    switch((int)BitUtils.get_bits(data, 276, 7))
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
                    }
                    throw  new ArgumentException("Unknown enum ID ");
                }
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

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYP
            {
                get
                {
                    switch((int)BitUtils.get_bits(data, 283, 3))
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
            static readonly Meta meta73 = new Meta(73, 1, 0, 0, 36, 286);
        } public class VFR_HUD : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal VFR_HUD() : base(meta74, 0) { }
            internal VFR_HUD(int bytes) : base(meta74, bytes) { }
            public ushort throttle //Current throttle setting in integer percent, 0 to 10
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public float airspeed //Current airspeed in m/
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  2, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 2);}
            }

            public float groundspeed //Current ground speed in m/
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  6, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 6);}
            }

            public short heading //Current heading in degrees, in compass units (0..360, 0=north
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  10, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  10);}
            }

            public float alt //Current altitude (MSL), in meter
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float climb //Current climb rate in meters/secon
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }
            static readonly Meta meta74 = new Meta(74, 1, 0, 0, 20, 160);
        } public class COMMAND_INT : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal COMMAND_INT() : base(meta75, 0) { }
            internal COMMAND_INT(int bytes) : base(meta75, bytes) { }
            public byte target_system //System I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            public byte current //false:0, true:
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }

            public byte autocontinue //autocontinue to next w
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  3);}
            }

            public float param1 //PARAM1, see MAV_CMD enu
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 4);}
            }

            public float param2 //PARAM2, see MAV_CMD enu
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float param3 //PARAM3, see MAV_CMD enu
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float param4 //PARAM4, see MAV_CMD enu
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public int x //PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  20, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  20);}
            }

            public int y //PARAM6 / local: y position in meters * 1e4, global: longitude in degrees * 10^
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  24, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  24);}
            }

            public float z //PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }

            public MAV_FRAME frame //The coordinate system of the COMMAND. see MAV_FRAME in mavlink_types.
            {
                get {  return (MAV_FRAME)(0 +  BitUtils.get_bits(data, 256, 4));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 256);}
            }

            public MAV_CMD command //The scheduled action for the mission item. see MAV_CMD in common.xml MAVLink spec
            {
                get
                {
                    switch((int)BitUtils.get_bits(data, 260, 7))
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
                    }
                    throw  new ArgumentException("Unknown enum ID ");
                }
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
            static readonly Meta meta75 = new Meta(75, 0, 0, 0, 34, 267);
        } public class COMMAND_LONG : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal COMMAND_LONG() : base(meta76, 0) { }
            internal COMMAND_LONG(int bytes) : base(meta76, bytes) { }
            public byte target_system //System which should execute the comman
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component which should execute the command, 0 for all component
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            public byte confirmation //0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }

            public float param1 //Parameter 1, as defined by MAV_CMD enum
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  3, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 3);}
            }

            public float param2 //Parameter 2, as defined by MAV_CMD enum
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  7, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 7);}
            }

            public float param3 //Parameter 3, as defined by MAV_CMD enum
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  11, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 11);}
            }

            public float param4 //Parameter 4, as defined by MAV_CMD enum
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  15, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 15);}
            }

            public float param5 //Parameter 5, as defined by MAV_CMD enum
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  19, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 19);}
            }

            public float param6 //Parameter 6, as defined by MAV_CMD enum
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  23, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 23);}
            }

            public float param7 //Parameter 7, as defined by MAV_CMD enum
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  27, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 27);}
            }

            public MAV_CMD command //Command ID, as defined by MAV_CMD enum
            {
                get
                {
                    switch((int)BitUtils.get_bits(data, 248, 7))
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
                    }
                    throw  new ArgumentException("Unknown enum ID ");
                }
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
                    BitUtils.set_bits(id, 7, data, 248);
                }
            }
            static readonly Meta meta76 = new Meta(76, 0, 0, 0, 32, 255);
        } public class COMMAND_ACK : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal COMMAND_ACK() : base(meta77, 0) { }
            internal COMMAND_ACK(int bytes) : base(meta77, bytes) { }
            public MAV_CMD command //Command ID, as defined by MAV_CMD enum
            {
                get
                {
                    switch((int)BitUtils.get_bits(data, 0, 7))
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
                    }
                    throw  new ArgumentException("Unknown enum ID ");
                }
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
                    BitUtils.set_bits(id, 7, data, 0);
                }
            }

            public MAV_RESULT result //See MAV_RESULT enu
            {
                get {  return (MAV_RESULT)(0 +  BitUtils.get_bits(data, 7, 3));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 3, data, 7);}
            }
            public byte progress_TRY(Inside ph)
            {
                if(ph.field_bit !=  10 && !try_visit_field(ph, 10)) return 0;
                return (byte)((byte) BitUtils.get_bytes(data,  ph.BYTE, 1));
            }
            public void progress_SET(byte src, Inside ph)
            {
                if(ph.field_bit != 10)insert_field(ph, 10, 0);
                BitUtils.set_bytes((ulong)(src), 1, data,  ph.BYTE);
            } public int result_param2_TRY(Inside ph)
            {
                if(ph.field_bit !=  11 && !try_visit_field(ph, 11)) return 0;
                return (int)((int) BitUtils.get_bytes(data,  ph.BYTE, 4));
            }
            public void result_param2_SET(int src, Inside ph)
            {
                if(ph.field_bit != 11)insert_field(ph, 11, 0);
                BitUtils.set_bytes((uint)(src), 4, data,  ph.BYTE);
            } public byte target_system_TRY(Inside ph) //WIP: System which requested the command to be execute
            {
                if(ph.field_bit !=  12 && !try_visit_field(ph, 12)) return 0;
                return (byte)((byte) BitUtils.get_bytes(data,  ph.BYTE, 1));
            }
            public void target_system_SET(byte src, Inside ph)//WIP: System which requested the command to be execute
            {
                if(ph.field_bit != 12)insert_field(ph, 12, 0);
                BitUtils.set_bytes((ulong)(src), 1, data,  ph.BYTE);
            } public byte target_component_TRY(Inside ph) //WIP: Component which requested the command to be execute
            {
                if(ph.field_bit !=  13 && !try_visit_field(ph, 13)) return 0;
                return (byte)((byte) BitUtils.get_bytes(data,  ph.BYTE, 1));
            }
            public void target_component_SET(byte src, Inside ph)//WIP: Component which requested the command to be execute
            {
                if(ph.field_bit != 13)insert_field(ph, 13, 0);
                BitUtils.set_bytes((ulong)(src), 1, data,  ph.BYTE);
            } static readonly Meta meta77 = new Meta(77, 0, 0, 0, 3, 10, 0, _VF, _hF, _QF, _vF);
        } public class MANUAL_SETPOINT : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal MANUAL_SETPOINT() : base(meta81, 0) { }
            internal MANUAL_SETPOINT(int bytes) : base(meta81, bytes) { }
            public uint time_boot_ms //Timestamp in milliseconds since system boo
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public float roll //Desired roll rate in radians per secon
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 4);}
            }

            public float pitch //Desired pitch rate in radians per secon
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float yaw //Desired yaw rate in radians per secon
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float thrust //Collective thrust, normalized to 0 ..
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public byte mode_switch //Flight mode switch position, 0.. 25
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  20, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  20);}
            }

            public byte manual_override_switch //Override mode switch position, 0.. 25
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  21, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  21);}
            }
            static readonly Meta meta81 = new Meta(81, 0, 1, 0, 22, 176);
        } public class SET_ATTITUDE_TARGET : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal SET_ATTITUDE_TARGET() : base(meta82, 0) { }
            internal SET_ATTITUDE_TARGET(int bytes) : base(meta82, bytes) { }
            public uint time_boot_ms //Timestamp in milliseconds since system boo
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public byte target_system //System I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  4);}
            }

            public byte target_component //Component I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  5);}
            }

            public byte type_mask
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  6, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  6);}
            }

            public float[] q //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0
            {
                get {return q_GET(new float[4], 0);}
                set {q_SET(value, 0)  ;}
            }
            public float[]q_GET(float[] dst_ch, int pos)  //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0
            {
                for(int BYTE = 7, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }

            public void q_SET(float[] src, int pos)  //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0
            {
                for(int BYTE =  7, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }


            public float body_roll_rate //Body roll rate in radians per secon
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  23, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 23);}
            }

            public float body_pitch_rate //Body roll rate in radians per secon
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  27, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 27);}
            }

            public float body_yaw_rate //Body roll rate in radians per secon
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  31, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 31);}
            }

            public float thrust //Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  35, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 35);}
            }
            static readonly Meta meta82 = new Meta(82, 0, 1, 0, 39, 312);
        } public class ATTITUDE_TARGET : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal ATTITUDE_TARGET() : base(meta83, 0) { }
            internal ATTITUDE_TARGET(int bytes) : base(meta83, bytes) { }
            public uint time_boot_ms //Timestamp in milliseconds since system boo
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public byte type_mask
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  4);}
            }

            public float[] q //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0
            {
                get {return q_GET(new float[4], 0);}
                set {q_SET(value, 0)  ;}
            }
            public float[]q_GET(float[] dst_ch, int pos)  //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0
            {
                for(int BYTE = 5, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }

            public void q_SET(float[] src, int pos)  //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0
            {
                for(int BYTE =  5, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }


            public float body_roll_rate //Body roll rate in radians per secon
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  21, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 21);}
            }

            public float body_pitch_rate //Body pitch rate in radians per secon
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  25, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 25);}
            }

            public float body_yaw_rate //Body yaw rate in radians per secon
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  29, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 29);}
            }

            public float thrust //Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  33, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 33);}
            }
            static readonly Meta meta83 = new Meta(83, 0, 1, 0, 37, 296);
        } public class SET_POSITION_TARGET_LOCAL_NED : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal SET_POSITION_TARGET_LOCAL_NED() : base(meta84, 0) { }
            internal SET_POSITION_TARGET_LOCAL_NED(int bytes) : base(meta84, bytes) { }
            public ushort type_mask
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public uint time_boot_ms //Timestamp in milliseconds since system boo
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  2, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  2);}
            }

            public byte target_system //System I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  6, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  6);}
            }

            public byte target_component //Component I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  7, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  7);}
            }

            public float x //X Position in NED frame in meter
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float y //Y Position in NED frame in meter
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float z //Z Position in NED frame in meters (note, altitude is negative in NED
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float vx //X velocity in NED frame in meter /
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float vy //Y velocity in NED frame in meter /
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float vz //Z velocity in NED frame in meter /
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }

            public float afx //X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  32, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 32);}
            }

            public float afy //Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  36, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 36);}
            }

            public float afz //Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  40, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 40);}
            }

            public float yaw //yaw setpoint in ra
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  44, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 44);}
            }

            public float yaw_rate //yaw rate setpoint in rad/
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  48, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 48);}
            }

            public MAV_FRAME coordinate_frame
            {
                get {  return (MAV_FRAME)(0 +  BitUtils.get_bits(data, 416, 4));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 416);}
            }
            static readonly Meta meta84 = new Meta(84, 1, 1, 0, 53, 420);
        } public class SET_POSITION_TARGET_GLOBAL_INT : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal SET_POSITION_TARGET_GLOBAL_INT() : base(meta86, 0) { }
            internal SET_POSITION_TARGET_GLOBAL_INT(int bytes) : base(meta86, bytes) { }
            public ushort type_mask
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public uint time_boot_ms
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  2, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  2);}
            }

            public byte target_system //System I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  6, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  6);}
            }

            public byte target_component //Component I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  7, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  7);}
            }

            public int lat_int //X Position in WGS84 frame in 1e7 * meter
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  8, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  8);}
            }

            public int lon_int //Y Position in WGS84 frame in 1e7 * meter
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  12, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  12);}
            }

            public float alt //Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_I
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float vx //X velocity in NED frame in meter /
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float vy //Y velocity in NED frame in meter /
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float vz //Z velocity in NED frame in meter /
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }

            public float afx //X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  32, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 32);}
            }

            public float afy //Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  36, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 36);}
            }

            public float afz //Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  40, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 40);}
            }

            public float yaw //yaw setpoint in ra
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  44, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 44);}
            }

            public float yaw_rate //yaw rate setpoint in rad/
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  48, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 48);}
            }

            public MAV_FRAME coordinate_frame
            {
                get {  return (MAV_FRAME)(0 +  BitUtils.get_bits(data, 416, 4));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 416);}
            }
            static readonly Meta meta86 = new Meta(86, 1, 1, 0, 53, 420);
        } public class POSITION_TARGET_GLOBAL_INT : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal POSITION_TARGET_GLOBAL_INT() : base(meta87, 0) { }
            internal POSITION_TARGET_GLOBAL_INT(int bytes) : base(meta87, bytes) { }
            public ushort type_mask
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public uint time_boot_ms
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  2, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  2);}
            }

            public int lat_int //X Position in WGS84 frame in 1e7 * meter
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  6, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  6);}
            }

            public int lon_int //Y Position in WGS84 frame in 1e7 * meter
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  10, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  10);}
            }

            public float alt //Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_I
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  14, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 14);}
            }

            public float vx //X velocity in NED frame in meter /
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  18, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 18);}
            }

            public float vy //Y velocity in NED frame in meter /
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  22, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 22);}
            }

            public float vz //Z velocity in NED frame in meter /
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  26, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 26);}
            }

            public float afx //X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  30, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 30);}
            }

            public float afy //Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  34, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 34);}
            }

            public float afz //Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  38, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 38);}
            }

            public float yaw //yaw setpoint in ra
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  42, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 42);}
            }

            public float yaw_rate //yaw rate setpoint in rad/
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  46, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 46);}
            }

            public MAV_FRAME coordinate_frame
            {
                get {  return (MAV_FRAME)(0 +  BitUtils.get_bits(data, 400, 4));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 400);}
            }
            static readonly Meta meta87 = new Meta(87, 1, 1, 0, 51, 404);
        } public class LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET() : base(meta89, 0) { }
            internal LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET(int bytes) : base(meta89, bytes) { }
            public uint time_boot_ms //Timestamp (milliseconds since system boot
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public float x //X Positio
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 4);}
            }

            public float y //Y Positio
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float z //Z Positio
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float roll //Rol
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float pitch //Pitc
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float yaw //Ya
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }
            static readonly Meta meta89 = new Meta(89, 0, 1, 0, 28, 224);
        } public class HIL_STATE : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal HIL_STATE() : base(meta90, 0) { }
            internal HIL_STATE(int bytes) : base(meta90, bytes) { }
            public ulong time_usec //Timestamp (microseconds since UNIX epoch or microseconds since system boot
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public float roll //Roll angle (rad
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float pitch //Pitch angle (rad
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float yaw //Yaw angle (rad
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float rollspeed //Body frame roll / phi angular speed (rad/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float pitchspeed //Body frame pitch / theta angular speed (rad/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float yawspeed //Body frame yaw / psi angular speed (rad/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }

            public int lat //Latitude, expressed as * 1E
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  32, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  32);}
            }

            public int lon //Longitude, expressed as * 1E
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  36, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  36);}
            }

            public int alt //Altitude in meters, expressed as * 1000 (millimeters
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  40, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  40);}
            }

            public short vx //Ground X Speed (Latitude), expressed as m/s * 10
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  44, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  44);}
            }

            public short vy //Ground Y Speed (Longitude), expressed as m/s * 10
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  46, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  46);}
            }

            public short vz //Ground Z Speed (Altitude), expressed as m/s * 10
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  48, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  48);}
            }

            public short xacc //X acceleration (mg
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  50, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  50);}
            }

            public short yacc //Y acceleration (mg
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  52, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  52);}
            }

            public short zacc //Z acceleration (mg
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  54, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  54);}
            }
            static readonly Meta meta90 = new Meta(90, 0, 0, 1, 56, 448);
        } public class HIL_CONTROLS : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal HIL_CONTROLS() : base(meta91, 0) { }
            internal HIL_CONTROLS(int bytes) : base(meta91, bytes) { }
            public ulong time_usec //Timestamp (microseconds since UNIX epoch or microseconds since system boot
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public float roll_ailerons //Control output -1 ..
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float pitch_elevator //Control output -1 ..
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float yaw_rudder //Control output -1 ..
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float throttle //Throttle 0 ..
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float aux1 //Aux 1, -1 ..
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float aux2 //Aux 2, -1 ..
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }

            public float aux3 //Aux 3, -1 ..
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  32, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 32);}
            }

            public float aux4 //Aux 4, -1 ..
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  36, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 36);}
            }

            public byte nav_mode //Navigation mode (MAV_NAV_MODE
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  40, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  40);}
            }

            public MAV_MODE mode //System mode (MAV_MODE
            {
                get
                {
                    switch((int)BitUtils.get_bits(data, 328, 4))
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
            static readonly Meta meta91 = new Meta(91, 0, 0, 1, 42, 332);
        } public class HIL_RC_INPUTS_RAW : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal HIL_RC_INPUTS_RAW() : base(meta92, 0) { }
            internal HIL_RC_INPUTS_RAW(int bytes) : base(meta92, bytes) { }
            public ushort chan1_raw //RC channel 1 value, in microsecond
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort chan2_raw //RC channel 2 value, in microsecond
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public ushort chan3_raw //RC channel 3 value, in microsecond
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  4);}
            }

            public ushort chan4_raw //RC channel 4 value, in microsecond
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  6, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  6);}
            }

            public ushort chan5_raw //RC channel 5 value, in microsecond
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  8, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  8);}
            }

            public ushort chan6_raw //RC channel 6 value, in microsecond
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  10, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  10);}
            }

            public ushort chan7_raw //RC channel 7 value, in microsecond
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  12, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  12);}
            }

            public ushort chan8_raw //RC channel 8 value, in microsecond
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  14, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  14);}
            }

            public ushort chan9_raw //RC channel 9 value, in microsecond
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  16, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  16);}
            }

            public ushort chan10_raw //RC channel 10 value, in microsecond
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  18, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  18);}
            }

            public ushort chan11_raw //RC channel 11 value, in microsecond
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  20, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  20);}
            }

            public ushort chan12_raw //RC channel 12 value, in microsecond
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  22, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  22);}
            }

            public ulong time_usec //Timestamp (microseconds since UNIX epoch or microseconds since system boot
            {
                get {  return (BitUtils.get_bytes(data,  24, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  24);}
            }

            public byte rssi //Receive signal strength indicator, 0: 0%, 255: 100
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  32, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  32);}
            }
            static readonly Meta meta92 = new Meta(92, 12, 0, 1, 33, 264);
        } public class HIL_ACTUATOR_CONTROLS : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal HIL_ACTUATOR_CONTROLS() : base(meta93, 0) { }
            internal HIL_ACTUATOR_CONTROLS(int bytes) : base(meta93, bytes) { }
            public ulong time_usec //Timestamp (microseconds since UNIX epoch or microseconds since system boot
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public ulong flags //Flags as bitfield, reserved for future use
            {
                get {  return (BitUtils.get_bytes(data,  8, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  8);}
            }

            public float[] controls //Control outputs -1 .. 1. Channel assignment depends on the simulated hardware
            {
                get {return controls_GET(new float[16], 0);}
                set {controls_SET(value, 0)  ;}
            }
            public float[]controls_GET(float[] dst_ch, int pos)  //Control outputs -1 .. 1. Channel assignment depends on the simulated hardware
            {
                for(int BYTE = 16, dst_max = pos + 16; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }

            public void controls_SET(float[] src, int pos)  //Control outputs -1 .. 1. Channel assignment depends on the simulated hardware
            {
                for(int BYTE =  16, src_max = pos + 16; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }


            public MAV_MODE mode //System mode (MAV_MODE), includes arming state
            {
                get
                {
                    switch((int)BitUtils.get_bits(data, 640, 4))
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
            static readonly Meta meta93 = new Meta(93, 0, 0, 2, 81, 644);
        } public class OPTICAL_FLOW : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal OPTICAL_FLOW() : base(meta100, 0) { }
            internal OPTICAL_FLOW(int bytes) : base(meta100, bytes) { }
            public ulong time_usec //Timestamp (UNIX
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public byte sensor_id //Sensor I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  8, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  8);}
            }

            public short flow_x //Flow in pixels * 10 in x-sensor direction (dezi-pixels
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  9, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  9);}
            }

            public short flow_y //Flow in pixels * 10 in y-sensor direction (dezi-pixels
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  11, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  11);}
            }

            public float flow_comp_m_x //Flow in meters in x-sensor direction, angular-speed compensate
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  13, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 13);}
            }

            public float flow_comp_m_y //Flow in meters in y-sensor direction, angular-speed compensate
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  17, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 17);}
            }

            public byte quality //Optical flow quality / confidence. 0: bad, 255: maximum qualit
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  21, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  21);}
            }

            public float ground_distance //Ground distance in meters. Positive value: distance known. Negative value: Unknown distanc
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  22, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 22);}
            }
            public float flow_rate_x_TRY(Inside ph)//Flow rate in radians/second about X axi
            {
                if(ph.field_bit !=  208 && !try_visit_field(ph, 208)) return 0;
                return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  ph.BYTE, 4)));
            }
            public void flow_rate_x_SET(float src, Inside ph)//Flow rate in radians/second about X axi
            {
                if(ph.field_bit != 208)insert_field(ph, 208, 0);
                BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src), 4, data, ph.BYTE);
            } public float flow_rate_y_TRY(Inside ph) //Flow rate in radians/second about Y axi
            {
                if(ph.field_bit !=  209 && !try_visit_field(ph, 209)) return 0;
                return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  ph.BYTE, 4)));
            }
            public void flow_rate_y_SET(float src, Inside ph)//Flow rate in radians/second about Y axi
            {
                if(ph.field_bit != 209)insert_field(ph, 209, 0);
                BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src), 4, data, ph.BYTE);
            } static readonly Meta meta100 = new Meta(100, 0, 0, 1, 27, 208, 0, _xt, _mt);
        } public class GLOBAL_VISION_POSITION_ESTIMATE : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal GLOBAL_VISION_POSITION_ESTIMATE() : base(meta101, 0) { }
            internal GLOBAL_VISION_POSITION_ESTIMATE(int bytes) : base(meta101, bytes) { }
            public ulong usec //Timestamp (microseconds, synced to UNIX time or since system boot
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public float x //Global X positio
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float y //Global Y positio
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float z //Global Z positio
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float roll //Roll angle in ra
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float pitch //Pitch angle in ra
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float yaw //Yaw angle in ra
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }
            static readonly Meta meta101 = new Meta(101, 0, 0, 1, 32, 256);
        } public class VISION_POSITION_ESTIMATE : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal VISION_POSITION_ESTIMATE() : base(meta102, 0) { }
            internal VISION_POSITION_ESTIMATE(int bytes) : base(meta102, bytes) { }
            public ulong usec //Timestamp (microseconds, synced to UNIX time or since system boot
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public float x //Global X positio
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float y //Global Y positio
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float z //Global Z positio
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float roll //Roll angle in ra
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float pitch //Pitch angle in ra
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float yaw //Yaw angle in ra
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }
            static readonly Meta meta102 = new Meta(102, 0, 0, 1, 32, 256);
        } public class VISION_SPEED_ESTIMATE : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal VISION_SPEED_ESTIMATE() : base(meta103, 0) { }
            internal VISION_SPEED_ESTIMATE(int bytes) : base(meta103, bytes) { }
            public ulong usec //Timestamp (microseconds, synced to UNIX time or since system boot
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public float x //Global X spee
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float y //Global Y spee
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float z //Global Z spee
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }
            static readonly Meta meta103 = new Meta(103, 0, 0, 1, 20, 160);
        } public class VICON_POSITION_ESTIMATE : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal VICON_POSITION_ESTIMATE() : base(meta104, 0) { }
            internal VICON_POSITION_ESTIMATE(int bytes) : base(meta104, bytes) { }
            public ulong usec //Timestamp (microseconds, synced to UNIX time or since system boot
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public float x //Global X positio
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float y //Global Y positio
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float z //Global Z positio
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float roll //Roll angle in ra
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float pitch //Pitch angle in ra
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float yaw //Yaw angle in ra
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }
            static readonly Meta meta104 = new Meta(104, 0, 0, 1, 32, 256);
        } public class HIGHRES_IMU : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal HIGHRES_IMU() : base(meta105, 0) { }
            internal HIGHRES_IMU(int bytes) : base(meta105, bytes) { }
            public ushort fields_updated //Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperatur
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ulong time_usec //Timestamp (microseconds, synced to UNIX time or since system boot
            {
                get {  return (BitUtils.get_bytes(data,  2, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  2);}
            }

            public float xacc //X acceleration (m/s^2
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  10, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 10);}
            }

            public float yacc //Y acceleration (m/s^2
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  14, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 14);}
            }

            public float zacc //Z acceleration (m/s^2
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  18, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 18);}
            }

            public float xgyro //Angular speed around X axis (rad / sec
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  22, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 22);}
            }

            public float ygyro //Angular speed around Y axis (rad / sec
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  26, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 26);}
            }

            public float zgyro //Angular speed around Z axis (rad / sec
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  30, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 30);}
            }

            public float xmag //X Magnetic field (Gauss
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  34, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 34);}
            }

            public float ymag //Y Magnetic field (Gauss
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  38, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 38);}
            }

            public float zmag //Z Magnetic field (Gauss
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  42, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 42);}
            }

            public float abs_pressure //Absolute pressure in milliba
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  46, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 46);}
            }

            public float diff_pressure //Differential pressure in milliba
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  50, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 50);}
            }

            public float pressure_alt //Altitude calculated from pressur
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  54, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 54);}
            }

            public float temperature //Temperature in degrees celsiu
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  58, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 58);}
            }
            static readonly Meta meta105 = new Meta(105, 1, 0, 1, 62, 496);
        } public class OPTICAL_FLOW_RAD : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal OPTICAL_FLOW_RAD() : base(meta106, 0) { }
            internal OPTICAL_FLOW_RAD(int bytes) : base(meta106, bytes) { }
            public uint integration_time_us
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public uint time_delta_distance_us //Time in microseconds since the distance was sampled
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  4, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  4);}
            }

            public ulong time_usec //Timestamp (microseconds, synced to UNIX time or since system boot
            {
                get {  return (BitUtils.get_bytes(data,  8, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  8);}
            }

            public byte sensor_id //Sensor I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  16, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  16);}
            }

            public float integrated_x
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  17, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 17);}
            }

            public float integrated_y
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  21, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 21);}
            }

            public float integrated_xgyro //RH rotation around X axis (rad
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  25, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 25);}
            }

            public float integrated_ygyro //RH rotation around Y axis (rad
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  29, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 29);}
            }

            public float integrated_zgyro //RH rotation around Z axis (rad
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  33, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 33);}
            }

            public short temperature //Temperature * 100 in centi-degrees Celsiu
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  37, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  37);}
            }

            public byte quality //Optical flow quality / confidence. 0: no valid flow, 255: maximum qualit
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  39, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  39);}
            }

            public float distance
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  40, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 40);}
            }
            static readonly Meta meta106 = new Meta(106, 0, 2, 1, 44, 352);
        } public class HIL_SENSOR : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal HIL_SENSOR() : base(meta107, 0) { }
            internal HIL_SENSOR(int bytes) : base(meta107, bytes) { }
            public uint fields_updated
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public ulong time_usec //Timestamp (microseconds, synced to UNIX time or since system boot
            {
                get {  return (BitUtils.get_bytes(data,  4, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  4);}
            }

            public float xacc //X acceleration (m/s^2
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float yacc //Y acceleration (m/s^2
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float zacc //Z acceleration (m/s^2
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float xgyro //Angular speed around X axis in body frame (rad / sec
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float ygyro //Angular speed around Y axis in body frame (rad / sec
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }

            public float zgyro //Angular speed around Z axis in body frame (rad / sec
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  32, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 32);}
            }

            public float xmag //X Magnetic field (Gauss
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  36, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 36);}
            }

            public float ymag //Y Magnetic field (Gauss
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  40, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 40);}
            }

            public float zmag //Z Magnetic field (Gauss
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  44, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 44);}
            }

            public float abs_pressure //Absolute pressure in milliba
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  48, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 48);}
            }

            public float diff_pressure //Differential pressure (airspeed) in milliba
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  52, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 52);}
            }

            public float pressure_alt //Altitude calculated from pressur
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  56, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 56);}
            }

            public float temperature //Temperature in degrees celsiu
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  60, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 60);}
            }
            static readonly Meta meta107 = new Meta(107, 0, 1, 1, 64, 512);
        } public class SIM_STATE : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal SIM_STATE() : base(meta108, 0) { }
            internal SIM_STATE(int bytes) : base(meta108, bytes) { }
            public float q1 //True attitude quaternion component 1, w (1 in null-rotation
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  0, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 0);}
            }

            public float q2 //True attitude quaternion component 2, x (0 in null-rotation
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 4);}
            }

            public float q3 //True attitude quaternion component 3, y (0 in null-rotation
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float q4 //True attitude quaternion component 4, z (0 in null-rotation
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float roll //Attitude roll expressed as Euler angles, not recommended except for human-readable output
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float pitch //Attitude pitch expressed as Euler angles, not recommended except for human-readable output
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float yaw //Attitude yaw expressed as Euler angles, not recommended except for human-readable output
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float xacc //X acceleration m/s/
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }

            public float yacc //Y acceleration m/s/
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  32, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 32);}
            }

            public float zacc //Z acceleration m/s/
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  36, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 36);}
            }

            public float xgyro //Angular speed around X axis rad/
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  40, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 40);}
            }

            public float ygyro //Angular speed around Y axis rad/
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  44, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 44);}
            }

            public float zgyro //Angular speed around Z axis rad/
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  48, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 48);}
            }

            public float lat //Latitude in degree
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  52, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 52);}
            }

            public float lon //Longitude in degree
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  56, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 56);}
            }

            public float alt //Altitude in meter
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  60, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 60);}
            }

            public float std_dev_horz //Horizontal position standard deviatio
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  64, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 64);}
            }

            public float std_dev_vert //Vertical position standard deviatio
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  68, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 68);}
            }

            public float vn //True velocity in m/s in NORTH direction in earth-fixed NED fram
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  72, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 72);}
            }

            public float ve //True velocity in m/s in EAST direction in earth-fixed NED fram
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  76, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 76);}
            }

            public float vd //True velocity in m/s in DOWN direction in earth-fixed NED fram
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  80, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 80);}
            }
            static readonly Meta meta108 = new Meta(108, 0, 0, 0, 84, 672);
        } public class RADIO_STATUS : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal RADIO_STATUS() : base(meta109, 0) { }
            internal RADIO_STATUS(int bytes) : base(meta109, bytes) { }
            public ushort rxerrors //Receive error
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort fixed_ //Count of error corrected packet
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public byte rssi //Local signal strengt
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  4);}
            }

            public byte remrssi //Remote signal strengt
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  5);}
            }

            public byte txbuf //Remaining free buffer space in percent
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  6, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  6);}
            }

            public byte noise //Background noise leve
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  7, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  7);}
            }

            public byte remnoise //Remote background noise leve
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  8, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  8);}
            }
            static readonly Meta meta109 = new Meta(109, 2, 0, 0, 9, 72);
        } public class FILE_TRANSFER_PROTOCOL : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal FILE_TRANSFER_PROTOCOL() : base(meta110, 0) { }
            internal FILE_TRANSFER_PROTOCOL(int bytes) : base(meta110, bytes) { }
            public byte target_network //Network ID (0 for broadcast
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_system //System ID (0 for broadcast
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            public byte target_component //Component ID (0 for broadcast
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }

            public byte[] payload
            {
                get {return payload_GET(new byte[251], 0);}
                set {payload_SET(value, 0)  ;}
            }
            public byte[]payload_GET(byte[] dst_ch, int pos)
            {
                for(int BYTE = 3, dst_max = pos + 251; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }

            public void payload_SET(byte[] src, int pos)
            {
                for(int BYTE =  3, src_max = pos + 251; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }

            static readonly Meta meta110 = new Meta(110, 0, 0, 0, 254, 2032);
        } public class TIMESYNC : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal TIMESYNC() : base(meta111, 0) { }
            internal TIMESYNC(int bytes) : base(meta111, bytes) { }
            public long tc1 //Time sync timestamp
            {
                get {  return (long)((long) BitUtils.get_bytes(data,  0, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public long ts1 //Time sync timestamp
            {
                get {  return (long)((long) BitUtils.get_bytes(data,  8, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  8);}
            }
            static readonly Meta meta111 = new Meta(111, 0, 0, 0, 16, 128);
        } public class CAMERA_TRIGGER : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal CAMERA_TRIGGER() : base(meta112, 0) { }
            internal CAMERA_TRIGGER(int bytes) : base(meta112, bytes) { }
            public uint seq //Image frame sequenc
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public ulong time_usec //Timestamp for the image frame in microsecond
            {
                get {  return (BitUtils.get_bytes(data,  4, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  4);}
            }
            static readonly Meta meta112 = new Meta(112, 0, 1, 1, 12, 96);
        } public class HIL_GPS : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal HIL_GPS() : base(meta113, 0) { }
            internal HIL_GPS(int bytes) : base(meta113, bytes) { }
            public ushort eph //GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 6553
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort epv //GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: 6553
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public ushort vel //GPS ground speed in cm/s. If unknown, set to: 6553
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  4);}
            }

            public ushort cog
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  6, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  6);}
            }

            public ulong time_usec //Timestamp (microseconds since UNIX epoch or microseconds since system boot
            {
                get {  return (BitUtils.get_bytes(data,  8, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  8);}
            }

            public byte fix_type
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  16, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  16);}
            }

            public int lat //Latitude (WGS84), in degrees * 1E
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  17, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  17);}
            }

            public int lon //Longitude (WGS84), in degrees * 1E
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  21, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  21);}
            }

            public int alt //Altitude (AMSL, not WGS84), in meters * 1000 (positive for up
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  25, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  25);}
            }

            public short vn //GPS velocity in cm/s in NORTH direction in earth-fixed NED fram
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  29, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  29);}
            }

            public short ve //GPS velocity in cm/s in EAST direction in earth-fixed NED fram
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  31, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  31);}
            }

            public short vd //GPS velocity in cm/s in DOWN direction in earth-fixed NED fram
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  33, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  33);}
            }

            public byte satellites_visible //Number of satellites visible. If unknown, set to 25
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  35, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  35);}
            }
            static readonly Meta meta113 = new Meta(113, 4, 0, 1, 36, 288);
        } public class HIL_OPTICAL_FLOW : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal HIL_OPTICAL_FLOW() : base(meta114, 0) { }
            internal HIL_OPTICAL_FLOW(int bytes) : base(meta114, bytes) { }
            public uint integration_time_us
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public uint time_delta_distance_us //Time in microseconds since the distance was sampled
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  4, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  4);}
            }

            public ulong time_usec //Timestamp (microseconds, synced to UNIX time or since system boot
            {
                get {  return (BitUtils.get_bytes(data,  8, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  8);}
            }

            public byte sensor_id //Sensor I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  16, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  16);}
            }

            public float integrated_x
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  17, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 17);}
            }

            public float integrated_y
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  21, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 21);}
            }

            public float integrated_xgyro //RH rotation around X axis (rad
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  25, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 25);}
            }

            public float integrated_ygyro //RH rotation around Y axis (rad
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  29, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 29);}
            }

            public float integrated_zgyro //RH rotation around Z axis (rad
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  33, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 33);}
            }

            public short temperature //Temperature * 100 in centi-degrees Celsiu
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  37, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  37);}
            }

            public byte quality //Optical flow quality / confidence. 0: no valid flow, 255: maximum qualit
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  39, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  39);}
            }

            public float distance
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  40, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 40);}
            }
            static readonly Meta meta114 = new Meta(114, 0, 2, 1, 44, 352);
        } public class HIL_STATE_QUATERNION : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal HIL_STATE_QUATERNION() : base(meta115, 0) { }
            internal HIL_STATE_QUATERNION(int bytes) : base(meta115, bytes) { }
            public ushort ind_airspeed //Indicated airspeed, expressed as cm/
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort true_airspeed //True airspeed, expressed as cm/
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public ulong time_usec //Timestamp (microseconds since UNIX epoch or microseconds since system boot
            {
                get {  return (BitUtils.get_bytes(data,  4, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  4);}
            }

            public float[] attitude_quaternion //Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotatio
            {
                get {return attitude_quaternion_GET(new float[4], 0);}
                set {attitude_quaternion_SET(value, 0)  ;}
            }
            public float[]attitude_quaternion_GET(float[] dst_ch, int pos)  //Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotatio
            {
                for(int BYTE = 12, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }

            public void attitude_quaternion_SET(float[] src, int pos)  //Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotatio
            {
                for(int BYTE =  12, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }


            public float rollspeed //Body frame roll / phi angular speed (rad/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }

            public float pitchspeed //Body frame pitch / theta angular speed (rad/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  32, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 32);}
            }

            public float yawspeed //Body frame yaw / psi angular speed (rad/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  36, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 36);}
            }

            public int lat //Latitude, expressed as * 1E
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  40, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  40);}
            }

            public int lon //Longitude, expressed as * 1E
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  44, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  44);}
            }

            public int alt //Altitude in meters, expressed as * 1000 (millimeters
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  48, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  48);}
            }

            public short vx //Ground X Speed (Latitude), expressed as cm/
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  52, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  52);}
            }

            public short vy //Ground Y Speed (Longitude), expressed as cm/
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  54, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  54);}
            }

            public short vz //Ground Z Speed (Altitude), expressed as cm/
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  56, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  56);}
            }

            public short xacc //X acceleration (mg
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  58, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  58);}
            }

            public short yacc //Y acceleration (mg
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  60, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  60);}
            }

            public short zacc //Z acceleration (mg
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  62, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  62);}
            }
            static readonly Meta meta115 = new Meta(115, 2, 0, 1, 64, 512);
        } public class SCALED_IMU2 : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal SCALED_IMU2() : base(meta116, 0) { }
            internal SCALED_IMU2(int bytes) : base(meta116, bytes) { }
            public uint time_boot_ms //Timestamp (milliseconds since system boot
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public short xacc //X acceleration (mg
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  4, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  4);}
            }

            public short yacc //Y acceleration (mg
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  6, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  6);}
            }

            public short zacc //Z acceleration (mg
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  8, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  8);}
            }

            public short xgyro //Angular speed around X axis (millirad /sec
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  10, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  10);}
            }

            public short ygyro //Angular speed around Y axis (millirad /sec
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  12, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  12);}
            }

            public short zgyro //Angular speed around Z axis (millirad /sec
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  14, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  14);}
            }

            public short xmag //X Magnetic field (milli tesla
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  16, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  16);}
            }

            public short ymag //Y Magnetic field (milli tesla
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  18, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  18);}
            }

            public short zmag //Z Magnetic field (milli tesla
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  20, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  20);}
            }
            static readonly Meta meta116 = new Meta(116, 0, 1, 0, 22, 176);
        } public class LOG_REQUEST_LIST : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal LOG_REQUEST_LIST() : base(meta117, 0) { }
            internal LOG_REQUEST_LIST(int bytes) : base(meta117, bytes) { }
            public ushort start //First log id (0 for first available
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort end //Last log id (0xffff for last available
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public byte target_system //System I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  4);}
            }

            public byte target_component //Component I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  5);}
            }
            static readonly Meta meta117 = new Meta(117, 2, 0, 0, 6, 48);
        } public class LOG_ENTRY : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal LOG_ENTRY() : base(meta118, 0) { }
            internal LOG_ENTRY(int bytes) : base(meta118, bytes) { }
            public ushort id //Log i
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort num_logs //Total number of log
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public ushort last_log_num //High log numbe
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  4);}
            }

            public uint time_utc //UTC timestamp of log in seconds since 1970, or 0 if not availabl
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  6, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  6);}
            }

            public uint size //Size of the log (may be approximate) in byte
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  10, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  10);}
            }
            static readonly Meta meta118 = new Meta(118, 3, 2, 0, 14, 112);
        } public class LOG_REQUEST_DATA : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal LOG_REQUEST_DATA() : base(meta119, 0) { }
            internal LOG_REQUEST_DATA(int bytes) : base(meta119, bytes) { }
            public ushort id //Log id (from LOG_ENTRY reply
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public uint ofs //Offset into the lo
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  2, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  2);}
            }

            public uint count //Number of byte
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  6, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  6);}
            }

            public byte target_system //System I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  10, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  10);}
            }

            public byte target_component //Component I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  11, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  11);}
            }
            static readonly Meta meta119 = new Meta(119, 1, 2, 0, 12, 96);
        } public class LOG_DATA : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal LOG_DATA() : base(meta120, 0) { }
            internal LOG_DATA(int bytes) : base(meta120, bytes) { }
            public ushort id //Log id (from LOG_ENTRY reply
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public uint ofs //Offset into the lo
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  2, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  2);}
            }

            public byte count //Number of bytes (zero for end of log
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  6, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  6);}
            }

            public byte[] data_ //log dat
            {
                get {return data__GET(new byte[90], 0);}
                set {data__SET(value, 0)  ;}
            }
            public byte[]data__GET(byte[] dst_ch, int pos)  //log dat
            {
                for(int BYTE = 7, dst_max = pos + 90; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }

            public void data__SET(byte[] src, int pos)  //log dat
            {
                for(int BYTE =  7, src_max = pos + 90; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }

            static readonly Meta meta120 = new Meta(120, 1, 1, 0, 97, 776);
        } public class LOG_ERASE : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal LOG_ERASE() : base(meta121, 0) { }
            internal LOG_ERASE(int bytes) : base(meta121, bytes) { }
            public byte target_system //System I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }
            static readonly Meta meta121 = new Meta(121, 0, 0, 0, 2, 16);
        } public class LOG_REQUEST_END : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal LOG_REQUEST_END() : base(meta122, 0) { }
            internal LOG_REQUEST_END(int bytes) : base(meta122, bytes) { }
            public byte target_system //System I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }
            static readonly Meta meta122 = new Meta(122, 0, 0, 0, 2, 16);
        } public class GPS_INJECT_DATA : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal GPS_INJECT_DATA() : base(meta123, 0) { }
            internal GPS_INJECT_DATA(int bytes) : base(meta123, bytes) { }
            public byte target_system //System I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            public byte len //data lengt
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }

            public byte[] data_ //raw data (110 is enough for 12 satellites of RTCMv2
            {
                get {return data__GET(new byte[110], 0);}
                set {data__SET(value, 0)  ;}
            }
            public byte[]data__GET(byte[] dst_ch, int pos)  //raw data (110 is enough for 12 satellites of RTCMv2
            {
                for(int BYTE = 3, dst_max = pos + 110; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }

            public void data__SET(byte[] src, int pos)  //raw data (110 is enough for 12 satellites of RTCMv2
            {
                for(int BYTE =  3, src_max = pos + 110; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }

            static readonly Meta meta123 = new Meta(123, 0, 0, 0, 113, 904);
        } public class GPS2_RAW : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal GPS2_RAW() : base(meta124, 0) { }
            internal GPS2_RAW(int bytes) : base(meta124, bytes) { }
            public ushort eph //GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: UINT16_MA
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort epv //GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: UINT16_MA
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public ushort vel //GPS ground speed (m/s * 100). If unknown, set to: UINT16_MA
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  4);}
            }

            public ushort cog
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  6, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  6);}
            }

            public uint dgps_age //Age of DGPS inf
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  8, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  8);}
            }

            public ulong time_usec //Timestamp (microseconds since UNIX epoch or microseconds since system boot
            {
                get {  return (BitUtils.get_bytes(data,  12, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  12);}
            }

            public int lat //Latitude (WGS84), in degrees * 1E
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  20, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  20);}
            }

            public int lon //Longitude (WGS84), in degrees * 1E
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  24, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  24);}
            }

            public int alt //Altitude (AMSL, not WGS84), in meters * 1000 (positive for up
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  28, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  28);}
            }

            public byte satellites_visible //Number of satellites visible. If unknown, set to 25
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  32, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  32);}
            }

            public byte dgps_numch //Number of DGPS satellite
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  33, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  33);}
            }

            public GPS_FIX_TYPE fix_type //See the GPS_FIX_TYPE enum
            {
                get {  return (GPS_FIX_TYPE)(0 +  BitUtils.get_bits(data, 272, 4));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 272);}
            }
            static readonly Meta meta124 = new Meta(124, 4, 1, 1, 35, 276);
        } public class POWER_STATUS : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal POWER_STATUS() : base(meta125, 0) { }
            internal POWER_STATUS(int bytes) : base(meta125, bytes) { }
            public ushort Vcc //5V rail voltage in millivolt
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort Vservo //servo rail voltage in millivolt
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public MAV_POWER_STATUS flags //power supply status flags (see MAV_POWER_STATUS enum
            {
                get
                {
                    switch((int)BitUtils.get_bits(data, 32, 3))
                    {
                        case 0:
                            return MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID;
                        case 1:
                            return MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID;
                        case 2:
                            return MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED;
                        case 3:
                            return MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT;
                        case 4:
                            return MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT;
                        case 5:
                            return MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED;
                    }
                    throw  new ArgumentException("Unknown enum ID ");
                }
                set
                {
                    ulong id = 0;
                    switch(value)
                    {
                        case MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID:
                            id = 0;
                            break;
                        case MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID:
                            id = 1;
                            break;
                        case MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED:
                            id = 2;
                            break;
                        case MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT:
                            id = 3;
                            break;
                        case MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT:
                            id = 4;
                            break;
                        case MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED:
                            id = 5;
                            break;
                        default:
                            throw  new ArgumentException("Unknown enum " + value);
                    }
                    BitUtils.set_bits(id, 3, data, 32);
                }
            }
            static readonly Meta meta125 = new Meta(125, 2, 0, 0, 5, 35);
        } public class SERIAL_CONTROL : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal SERIAL_CONTROL() : base(meta126, 0) { }
            internal SERIAL_CONTROL(int bytes) : base(meta126, bytes) { }
            public ushort timeout //Timeout for reply data in millisecond
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public uint baudrate //Baudrate of transfer. Zero means no change
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  2, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  2);}
            }

            public byte count //how many bytes in this transfe
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  6, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  6);}
            }

            public byte[] data_ //serial dat
            {
                get {return data__GET(new byte[70], 0);}
                set {data__SET(value, 0)  ;}
            }
            public byte[]data__GET(byte[] dst_ch, int pos)  //serial dat
            {
                for(int BYTE = 7, dst_max = pos + 70; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }

            public void data__SET(byte[] src, int pos)  //serial dat
            {
                for(int BYTE =  7, src_max = pos + 70; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }


            public SERIAL_CONTROL_DEV device //See SERIAL_CONTROL_DEV enu
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

            public SERIAL_CONTROL_FLAG flags //See SERIAL_CONTROL_FLAG enu
            {
                get
                {
                    switch((int)BitUtils.get_bits(data, 619, 3))
                    {
                        case 0:
                            return SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY;
                        case 1:
                            return SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND;
                        case 2:
                            return SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE;
                        case 3:
                            return SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING;
                        case 4:
                            return SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI;
                    }
                    throw  new ArgumentException("Unknown enum ID ");
                }
                set
                {
                    ulong id = 0;
                    switch(value)
                    {
                        case SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY:
                            id = 0;
                            break;
                        case SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND:
                            id = 1;
                            break;
                        case SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE:
                            id = 2;
                            break;
                        case SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING:
                            id = 3;
                            break;
                        case SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI:
                            id = 4;
                            break;
                        default:
                            throw  new ArgumentException("Unknown enum " + value);
                    }
                    BitUtils.set_bits(id, 3, data, 619);
                }
            }
            static readonly Meta meta126 = new Meta(126, 1, 1, 0, 78, 622);
        } public class GPS_RTK : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal GPS_RTK() : base(meta127, 0) { }
            internal GPS_RTK(int bytes) : base(meta127, bytes) { }
            public ushort wn //GPS Week Number of last baselin
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public uint time_last_baseline_ms //Time since boot of last baseline message received in ms
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  2, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  2);}
            }

            public uint tow //GPS Time of Week of last baselin
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  6, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  6);}
            }

            public uint accuracy //Current estimate of baseline accuracy
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  10, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  10);}
            }

            public byte rtk_receiver_id //Identification of connected RTK receiver
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  14, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  14);}
            }

            public byte rtk_health //GPS-specific health report for RTK data
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  15, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  15);}
            }

            public byte rtk_rate //Rate of baseline messages being received by GPS, in H
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  16, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  16);}
            }

            public byte nsats //Current number of sats used for RTK calculation
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  17, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  17);}
            }

            public byte baseline_coords_type //Coordinate system of baseline. 0 == ECEF, 1 == NE
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  18, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  18);}
            }

            public int baseline_a_mm //Current baseline in ECEF x or NED north component in mm
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  19, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  19);}
            }

            public int baseline_b_mm //Current baseline in ECEF y or NED east component in mm
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  23, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  23);}
            }

            public int baseline_c_mm //Current baseline in ECEF z or NED down component in mm
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  27, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  27);}
            }

            public int iar_num_hypotheses //Current number of integer ambiguity hypotheses
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  31, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  31);}
            }
            static readonly Meta meta127 = new Meta(127, 1, 3, 0, 35, 280);
        } public class GPS2_RTK : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal GPS2_RTK() : base(meta128, 0) { }
            internal GPS2_RTK(int bytes) : base(meta128, bytes) { }
            public ushort wn //GPS Week Number of last baselin
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public uint time_last_baseline_ms //Time since boot of last baseline message received in ms
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  2, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  2);}
            }

            public uint tow //GPS Time of Week of last baselin
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  6, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  6);}
            }

            public uint accuracy //Current estimate of baseline accuracy
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  10, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  10);}
            }

            public byte rtk_receiver_id //Identification of connected RTK receiver
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  14, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  14);}
            }

            public byte rtk_health //GPS-specific health report for RTK data
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  15, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  15);}
            }

            public byte rtk_rate //Rate of baseline messages being received by GPS, in H
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  16, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  16);}
            }

            public byte nsats //Current number of sats used for RTK calculation
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  17, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  17);}
            }

            public byte baseline_coords_type //Coordinate system of baseline. 0 == ECEF, 1 == NE
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  18, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  18);}
            }

            public int baseline_a_mm //Current baseline in ECEF x or NED north component in mm
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  19, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  19);}
            }

            public int baseline_b_mm //Current baseline in ECEF y or NED east component in mm
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  23, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  23);}
            }

            public int baseline_c_mm //Current baseline in ECEF z or NED down component in mm
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  27, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  27);}
            }

            public int iar_num_hypotheses //Current number of integer ambiguity hypotheses
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  31, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  31);}
            }
            static readonly Meta meta128 = new Meta(128, 1, 3, 0, 35, 280);
        } public class SCALED_IMU3 : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal SCALED_IMU3() : base(meta129, 0) { }
            internal SCALED_IMU3(int bytes) : base(meta129, bytes) { }
            public uint time_boot_ms //Timestamp (milliseconds since system boot
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public short xacc //X acceleration (mg
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  4, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  4);}
            }

            public short yacc //Y acceleration (mg
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  6, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  6);}
            }

            public short zacc //Z acceleration (mg
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  8, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  8);}
            }

            public short xgyro //Angular speed around X axis (millirad /sec
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  10, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  10);}
            }

            public short ygyro //Angular speed around Y axis (millirad /sec
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  12, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  12);}
            }

            public short zgyro //Angular speed around Z axis (millirad /sec
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  14, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  14);}
            }

            public short xmag //X Magnetic field (milli tesla
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  16, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  16);}
            }

            public short ymag //Y Magnetic field (milli tesla
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  18, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  18);}
            }

            public short zmag //Z Magnetic field (milli tesla
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  20, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  20);}
            }
            static readonly Meta meta129 = new Meta(129, 0, 1, 0, 22, 176);
        } public class DATA_TRANSMISSION_HANDSHAKE : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal DATA_TRANSMISSION_HANDSHAKE() : base(meta130, 0) { }
            internal DATA_TRANSMISSION_HANDSHAKE(int bytes) : base(meta130, bytes) { }
            public ushort width //Width of a matrix or imag
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort height //Height of a matrix or imag
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public ushort packets //number of packets beeing sent (set on ACK only
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  4);}
            }

            public uint size //total data size in bytes (set on ACK only
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  6, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  6);}
            }

            public byte type //type of requested/acknowledged data (as defined in ENUM DATA_TYPES in mavlink/include/mavlink_types.
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  10, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  10);}
            }

            public byte payload
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  11, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  11);}
            }

            public byte jpg_quality //JPEG quality out of [1,100
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  12, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  12);}
            }
            static readonly Meta meta130 = new Meta(130, 3, 1, 0, 13, 104);
        } public class ENCAPSULATED_DATA : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal ENCAPSULATED_DATA() : base(meta131, 0) { }
            internal ENCAPSULATED_DATA(int bytes) : base(meta131, bytes) { }
            public ushort seqnr //sequence number (starting with 0 on every transmission
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public byte[] data_ //image data byte
            {
                get {return data__GET(new byte[253], 0);}
                set {data__SET(value, 0)  ;}
            }
            public byte[]data__GET(byte[] dst_ch, int pos)  //image data byte
            {
                for(int BYTE = 2, dst_max = pos + 253; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }

            public void data__SET(byte[] src, int pos)  //image data byte
            {
                for(int BYTE =  2, src_max = pos + 253; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }

            static readonly Meta meta131 = new Meta(131, 1, 0, 0, 255, 2040);
        } public class DISTANCE_SENSOR : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal DISTANCE_SENSOR() : base(meta132, 0) { }
            internal DISTANCE_SENSOR(int bytes) : base(meta132, bytes) { }
            public ushort min_distance //Minimum distance the sensor can measure in centimeter
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort max_distance //Maximum distance the sensor can measure in centimeter
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public ushort current_distance //Current distance readin
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  4);}
            }

            public uint time_boot_ms //Time since system boo
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  6, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  6);}
            }

            public byte id //Onboard ID of the senso
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  10, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  10);}
            }

            public byte covariance //Measurement covariance in centimeters, 0 for unknown / invalid reading
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  11, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  11);}
            }

            public MAV_DISTANCE_SENSOR type //Type from MAV_DISTANCE_SENSOR enum
            {
                get {  return (MAV_DISTANCE_SENSOR)(0 +  BitUtils.get_bits(data, 96, 3));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 3, data, 96);}
            }

            public MAV_SENSOR_ORIENTATION orientation
            {
                get {  return (MAV_SENSOR_ORIENTATION)(0 +  BitUtils.get_bits(data, 99, 6));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 6, data, 99);}
            }
            static readonly Meta meta132 = new Meta(132, 3, 1, 0, 14, 105);
        } public class TERRAIN_REQUEST : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal TERRAIN_REQUEST() : base(meta133, 0) { }
            internal TERRAIN_REQUEST(int bytes) : base(meta133, bytes) { }
            public ushort grid_spacing //Grid spacing in meter
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ulong mask //Bitmask of requested 4x4 grids (row major 8x7 array of grids, 56 bits
            {
                get {  return (BitUtils.get_bytes(data,  2, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  2);}
            }

            public int lat //Latitude of SW corner of first grid (degrees *10^7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  10, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  10);}
            }

            public int lon //Longitude of SW corner of first grid (in degrees *10^7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  14, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  14);}
            }
            static readonly Meta meta133 = new Meta(133, 1, 0, 1, 18, 144);
        } public class TERRAIN_DATA : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal TERRAIN_DATA() : base(meta134, 0) { }
            internal TERRAIN_DATA(int bytes) : base(meta134, bytes) { }
            public ushort grid_spacing //Grid spacing in meter
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public int lat //Latitude of SW corner of first grid (degrees *10^7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  2, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  2);}
            }

            public int lon //Longitude of SW corner of first grid (in degrees *10^7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  6, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  6);}
            }

            public byte gridbit //bit within the terrain request mas
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  10, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  10);}
            }

            public short[] data_ //Terrain data in meters AMS
            {
                get {return data__GET(new short[16], 0);}
                set {data__SET(value, 0)  ;}
            }
            public short[]data__GET(short[] dst_ch, int pos)  //Terrain data in meters AMS
            {
                for(int BYTE = 11, dst_max = pos + 16; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (short)((short) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }

            public void data__SET(short[] src, int pos)  //Terrain data in meters AMS
            {
                for(int BYTE =  11, src_max = pos + 16; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            }

            static readonly Meta meta134 = new Meta(134, 1, 0, 0, 43, 344);
        } public class TERRAIN_CHECK : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal TERRAIN_CHECK() : base(meta135, 0) { }
            internal TERRAIN_CHECK(int bytes) : base(meta135, bytes) { }
            public int lat //Latitude (degrees *10^7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  0);}
            }

            public int lon //Longitude (degrees *10^7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  4, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  4);}
            }
            static readonly Meta meta135 = new Meta(135, 0, 0, 0, 8, 64);
        } public class TERRAIN_REPORT : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal TERRAIN_REPORT() : base(meta136, 0) { }
            internal TERRAIN_REPORT(int bytes) : base(meta136, bytes) { }
            public ushort spacing //grid spacing (zero if terrain at this location unavailable
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort pending //Number of 4x4 terrain blocks waiting to be received or read from dis
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public ushort loaded //Number of 4x4 terrain blocks in memor
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  4);}
            }

            public int lat //Latitude (degrees *10^7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  6, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  6);}
            }

            public int lon //Longitude (degrees *10^7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  10, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  10);}
            }

            public float terrain_height //Terrain height in meters AMS
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  14, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 14);}
            }

            public float current_height //Current vehicle height above lat/lon terrain height (meters
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  18, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 18);}
            }
            static readonly Meta meta136 = new Meta(136, 3, 0, 0, 22, 176);
        } public class SCALED_PRESSURE2 : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal SCALED_PRESSURE2() : base(meta137, 0) { }
            internal SCALED_PRESSURE2(int bytes) : base(meta137, bytes) { }
            public uint time_boot_ms //Timestamp (milliseconds since system boot
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public float press_abs //Absolute pressure (hectopascal
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 4);}
            }

            public float press_diff //Differential pressure 1 (hectopascal
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public short temperature //Temperature measurement (0.01 degrees celsius
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  12, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  12);}
            }
            static readonly Meta meta137 = new Meta(137, 0, 1, 0, 14, 112);
        } public class ATT_POS_MOCAP : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal ATT_POS_MOCAP() : base(meta138, 0) { }
            internal ATT_POS_MOCAP(int bytes) : base(meta138, bytes) { }
            public ulong time_usec //Timestamp (micros since boot or Unix epoch
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public float[] q //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0
            {
                get {return q_GET(new float[4], 0);}
                set {q_SET(value, 0)  ;}
            }
            public float[]q_GET(float[] dst_ch, int pos)  //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0
            {
                for(int BYTE = 8, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }

            public void q_SET(float[] src, int pos)  //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0
            {
                for(int BYTE =  8, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }


            public float x //X position in meters (NED
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float y //Y position in meters (NED
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }

            public float z //Z position in meters (NED
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  32, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 32);}
            }
            static readonly Meta meta138 = new Meta(138, 0, 0, 1, 36, 288);
        } public class SET_ACTUATOR_CONTROL_TARGET : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal SET_ACTUATOR_CONTROL_TARGET() : base(meta139, 0) { }
            internal SET_ACTUATOR_CONTROL_TARGET(int bytes) : base(meta139, bytes) { }
            public ulong time_usec //Timestamp (micros since boot or Unix epoch
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public byte group_mlx
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  8, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  8);}
            }

            public byte target_system //System I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  9, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  9);}
            }

            public byte target_component //Component I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  10, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  10);}
            }

            public float[] controls
            {
                get {return controls_GET(new float[8], 0);}
                set {controls_SET(value, 0)  ;}
            }
            public float[]controls_GET(float[] dst_ch, int pos)
            {
                for(int BYTE = 11, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }

            public void controls_SET(float[] src, int pos)
            {
                for(int BYTE =  11, src_max = pos + 8; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }

            static readonly Meta meta139 = new Meta(139, 0, 0, 1, 43, 344);
        } public class ACTUATOR_CONTROL_TARGET : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal ACTUATOR_CONTROL_TARGET() : base(meta140, 0) { }
            internal ACTUATOR_CONTROL_TARGET(int bytes) : base(meta140, bytes) { }
            public ulong time_usec //Timestamp (micros since boot or Unix epoch
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public byte group_mlx
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  8, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  8);}
            }

            public float[] controls
            {
                get {return controls_GET(new float[8], 0);}
                set {controls_SET(value, 0)  ;}
            }
            public float[]controls_GET(float[] dst_ch, int pos)
            {
                for(int BYTE = 9, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }

            public void controls_SET(float[] src, int pos)
            {
                for(int BYTE =  9, src_max = pos + 8; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }

            static readonly Meta meta140 = new Meta(140, 0, 0, 1, 41, 328);
        } public class ALTITUDE : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal ALTITUDE() : base(meta141, 0) { }
            internal ALTITUDE(int bytes) : base(meta141, bytes) { }
            public ulong time_usec //Timestamp (micros since boot or Unix epoch
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public float altitude_monotonic
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float altitude_amsl
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float altitude_local
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float altitude_relative //This is the altitude above the home position. It resets on each change of the current home positio
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float altitude_terrain
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float bottom_clearance
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }
            static readonly Meta meta141 = new Meta(141, 0, 0, 1, 32, 256);
        } public class RESOURCE_REQUEST : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal RESOURCE_REQUEST() : base(meta142, 0) { }
            internal RESOURCE_REQUEST(int bytes) : base(meta142, bytes) { }
            public byte request_id //Request ID. This ID should be re-used when sending back URI content
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte uri_type //The type of requested URI. 0 = a file via URL. 1 = a UAVCAN binar
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            public byte[] uri
            {
                get {return uri_GET(new byte[120], 0);}
                set {uri_SET(value, 0)  ;}
            }
            public byte[]uri_GET(byte[] dst_ch, int pos)
            {
                for(int BYTE = 2, dst_max = pos + 120; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }

            public void uri_SET(byte[] src, int pos)
            {
                for(int BYTE =  2, src_max = pos + 120; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }


            public byte transfer_type //The way the autopilot wants to receive the URI. 0 = MAVLink FTP. 1 = binary stream
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  122, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  122);}
            }

            public byte[] storage
            {
                get {return storage_GET(new byte[120], 0);}
                set {storage_SET(value, 0)  ;}
            }
            public byte[]storage_GET(byte[] dst_ch, int pos)
            {
                for(int BYTE = 123, dst_max = pos + 120; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }

            public void storage_SET(byte[] src, int pos)
            {
                for(int BYTE =  123, src_max = pos + 120; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }

            static readonly Meta meta142 = new Meta(142, 0, 0, 0, 243, 1944);
        } public class SCALED_PRESSURE3 : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal SCALED_PRESSURE3() : base(meta143, 0) { }
            internal SCALED_PRESSURE3(int bytes) : base(meta143, bytes) { }
            public uint time_boot_ms //Timestamp (milliseconds since system boot
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public float press_abs //Absolute pressure (hectopascal
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 4);}
            }

            public float press_diff //Differential pressure 1 (hectopascal
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public short temperature //Temperature measurement (0.01 degrees celsius
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  12, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  12);}
            }
            static readonly Meta meta143 = new Meta(143, 0, 1, 0, 14, 112);
        } public class FOLLOW_TARGET : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal FOLLOW_TARGET() : base(meta144, 0) { }
            internal FOLLOW_TARGET(int bytes) : base(meta144, bytes) { }
            public ulong timestamp //Timestamp in milliseconds since system boo
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public ulong custom_state //button states or switches of a tracker devic
            {
                get {  return (BitUtils.get_bytes(data,  8, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  8);}
            }

            public byte est_capabilities //bit positions for tracker reporting capabilities (POS = 0, VEL = 1, ACCEL = 2, ATT + RATES = 3
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  16, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  16);}
            }

            public int lat //Latitude (WGS84), in degrees * 1E
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  17, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  17);}
            }

            public int lon //Longitude (WGS84), in degrees * 1E
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  21, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  21);}
            }

            public float alt //AMSL, in meter
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  25, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 25);}
            }

            public float[] vel //target velocity (0,0,0) for unknow
            {
                get {return vel_GET(new float[3], 0);}
                set {vel_SET(value, 0)  ;}
            }
            public float[]vel_GET(float[] dst_ch, int pos)  //target velocity (0,0,0) for unknow
            {
                for(int BYTE = 29, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }

            public void vel_SET(float[] src, int pos)  //target velocity (0,0,0) for unknow
            {
                for(int BYTE =  29, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }


            public float[] acc //linear target acceleration (0,0,0) for unknow
            {
                get {return acc_GET(new float[3], 0);}
                set {acc_SET(value, 0)  ;}
            }
            public float[]acc_GET(float[] dst_ch, int pos)  //linear target acceleration (0,0,0) for unknow
            {
                for(int BYTE = 41, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }

            public void acc_SET(float[] src, int pos)  //linear target acceleration (0,0,0) for unknow
            {
                for(int BYTE =  41, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }


            public float[] attitude_q //(1 0 0 0 for unknown
            {
                get {return attitude_q_GET(new float[4], 0);}
                set {attitude_q_SET(value, 0)  ;}
            }
            public float[]attitude_q_GET(float[] dst_ch, int pos)  //(1 0 0 0 for unknown
            {
                for(int BYTE = 53, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }

            public void attitude_q_SET(float[] src, int pos)  //(1 0 0 0 for unknown
            {
                for(int BYTE =  53, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }


            public float[] rates //(0 0 0 for unknown
            {
                get {return rates_GET(new float[3], 0);}
                set {rates_SET(value, 0)  ;}
            }
            public float[]rates_GET(float[] dst_ch, int pos)  //(0 0 0 for unknown
            {
                for(int BYTE = 69, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }

            public void rates_SET(float[] src, int pos)  //(0 0 0 for unknown
            {
                for(int BYTE =  69, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }


            public float[] position_cov //eph ep
            {
                get {return position_cov_GET(new float[3], 0);}
                set {position_cov_SET(value, 0)  ;}
            }
            public float[]position_cov_GET(float[] dst_ch, int pos)  //eph ep
            {
                for(int BYTE = 81, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }

            public void position_cov_SET(float[] src, int pos)  //eph ep
            {
                for(int BYTE =  81, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }

            static readonly Meta meta144 = new Meta(144, 0, 0, 2, 93, 744);
        } public class CONTROL_SYSTEM_STATE : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal CONTROL_SYSTEM_STATE() : base(meta146, 0) { }
            internal CONTROL_SYSTEM_STATE(int bytes) : base(meta146, bytes) { }
            public ulong time_usec //Timestamp (micros since boot or Unix epoch
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public float x_acc //X acceleration in body fram
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float y_acc //Y acceleration in body fram
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float z_acc //Z acceleration in body fram
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float x_vel //X velocity in body fram
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float y_vel //Y velocity in body fram
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float z_vel //Z velocity in body fram
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }

            public float x_pos //X position in local fram
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  32, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 32);}
            }

            public float y_pos //Y position in local fram
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  36, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 36);}
            }

            public float z_pos //Z position in local fram
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  40, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 40);}
            }

            public float airspeed //Airspeed, set to -1 if unknow
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  44, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 44);}
            }

            public float[] vel_variance //Variance of body velocity estimat
            {
                get {return vel_variance_GET(new float[3], 0);}
                set {vel_variance_SET(value, 0)  ;}
            }
            public float[]vel_variance_GET(float[] dst_ch, int pos)  //Variance of body velocity estimat
            {
                for(int BYTE = 48, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }

            public void vel_variance_SET(float[] src, int pos)  //Variance of body velocity estimat
            {
                for(int BYTE =  48, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }


            public float[] pos_variance //Variance in local positio
            {
                get {return pos_variance_GET(new float[3], 0);}
                set {pos_variance_SET(value, 0)  ;}
            }
            public float[]pos_variance_GET(float[] dst_ch, int pos)  //Variance in local positio
            {
                for(int BYTE = 60, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }

            public void pos_variance_SET(float[] src, int pos)  //Variance in local positio
            {
                for(int BYTE =  60, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }


            public float[] q //The attitude, represented as Quaternio
            {
                get {return q_GET(new float[4], 0);}
                set {q_SET(value, 0)  ;}
            }
            public float[]q_GET(float[] dst_ch, int pos)  //The attitude, represented as Quaternio
            {
                for(int BYTE = 72, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }

            public void q_SET(float[] src, int pos)  //The attitude, represented as Quaternio
            {
                for(int BYTE =  72, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }


            public float roll_rate //Angular rate in roll axi
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  88, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 88);}
            }

            public float pitch_rate //Angular rate in pitch axi
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  92, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 92);}
            }

            public float yaw_rate //Angular rate in yaw axi
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  96, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 96);}
            }
            static readonly Meta meta146 = new Meta(146, 0, 0, 1, 100, 800);
        } public class BATTERY_STATUS : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal BATTERY_STATUS() : base(meta147, 0) { }
            internal BATTERY_STATUS(int bytes) : base(meta147, bytes) { }
            public ushort[] voltages
            {
                get {return voltages_GET(new ushort[10], 0);}
                set {voltages_SET(value, 0)  ;}
            }
            public ushort[]voltages_GET(ushort[] dst_ch, int pos)
            {
                for(int BYTE = 0, dst_max = pos + 10; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (ushort)((ushort) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }

            public void voltages_SET(ushort[] src, int pos)
            {
                for(int BYTE =  0, src_max = pos + 10; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ulong)(src[pos]), 2, data,  BYTE);
            }


            public byte id //Battery I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  20, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  20);}
            }

            public short temperature //Temperature of the battery in centi-degrees celsius. INT16_MAX for unknown temperature
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  21, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  21);}
            }

            public short current_battery //Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the curre
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  23, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  23);}
            }

            public int current_consumed //Consumed charge, in milliampere hours (1 = 1 mAh), -1: autopilot does not provide mAh consumption estima
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  25, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  25);}
            }

            public int energy_consumed
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  29, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  29);}
            }

            public sbyte battery_remaining //Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot does not estimate the remaining batter
            {
                get {  return (sbyte)((sbyte) BitUtils.get_bytes(data,  33, 1));}
                set {  BitUtils.set_bytes((byte)(value), 1, data,  33);}
            }

            public MAV_BATTERY_FUNCTION battery_function //Function of the batter
            {
                get {  return (MAV_BATTERY_FUNCTION)(0 +  BitUtils.get_bits(data, 272, 3));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 3, data, 272);}
            }

            public MAV_BATTERY_TYPE type //Type (chemistry) of the batter
            {
                get {  return (MAV_BATTERY_TYPE)(0 +  BitUtils.get_bits(data, 275, 3));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 3, data, 275);}
            }
            static readonly Meta meta147 = new Meta(147, 10, 0, 0, 35, 278);
        } public class AUTOPILOT_VERSION : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal AUTOPILOT_VERSION() : base(meta148, 0) { }
            internal AUTOPILOT_VERSION(int bytes) : base(meta148, bytes) { }
            public ushort vendor_id //ID of the board vendo
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort product_id //ID of the produc
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public uint flight_sw_version //Firmware version numbe
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  4, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  4);}
            }

            public uint middleware_sw_version //Middleware version numbe
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  8, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  8);}
            }

            public uint os_sw_version //Operating system version numbe
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  12, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  12);}
            }

            public uint board_version //HW / board version (last 8 bytes should be silicon ID, if any
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  16, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  16);}
            }

            public ulong uid //UID if provided by hardware (see uid2
            {
                get {  return (BitUtils.get_bytes(data,  20, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  20);}
            }

            public byte[] flight_custom_version
            {
                get {return flight_custom_version_GET(new byte[8], 0);}
                set {flight_custom_version_SET(value, 0)  ;}
            }
            public byte[]flight_custom_version_GET(byte[] dst_ch, int pos)
            {
                for(int BYTE = 28, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }

            public void flight_custom_version_SET(byte[] src, int pos)
            {
                for(int BYTE =  28, src_max = pos + 8; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }


            public byte[] middleware_custom_version
            {
                get {return middleware_custom_version_GET(new byte[8], 0);}
                set {middleware_custom_version_SET(value, 0)  ;}
            }
            public byte[]middleware_custom_version_GET(byte[] dst_ch, int pos)
            {
                for(int BYTE = 36, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }

            public void middleware_custom_version_SET(byte[] src, int pos)
            {
                for(int BYTE =  36, src_max = pos + 8; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }


            public byte[] os_custom_version
            {
                get {return os_custom_version_GET(new byte[8], 0);}
                set {os_custom_version_SET(value, 0)  ;}
            }
            public byte[]os_custom_version_GET(byte[] dst_ch, int pos)
            {
                for(int BYTE = 44, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }

            public void os_custom_version_SET(byte[] src, int pos)
            {
                for(int BYTE =  44, src_max = pos + 8; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }


            public MAV_PROTOCOL_CAPABILITY capabilities //bitmask of capabilities (see MAV_PROTOCOL_CAPABILITY enum
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
                set
                {
                    ulong id = 0;
                    switch(value)
                    {
                        case MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT:
                            id = 0;
                            break;
                        case MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT:
                            id = 1;
                            break;
                        case MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_INT:
                            id = 2;
                            break;
                        case MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMMAND_INT:
                            id = 3;
                            break;
                        case MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION:
                            id = 4;
                            break;
                        case MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP:
                            id = 5;
                            break;
                        case MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET:
                            id = 6;
                            break;
                        case MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED:
                            id = 7;
                            break;
                        case MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT:
                            id = 8;
                            break;
                        case MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_TERRAIN:
                            id = 9;
                            break;
                        case MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET:
                            id = 10;
                            break;
                        case MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION:
                            id = 11;
                            break;
                        case MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION:
                            id = 12;
                            break;
                        case MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MAVLINK2:
                            id = 13;
                            break;
                        case MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE:
                            id = 14;
                            break;
                        case MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY:
                            id = 15;
                            break;
                        case MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION:
                            id = 16;
                            break;
                        default:
                            throw  new ArgumentException("Unknown enum " + value);
                    }
                    BitUtils.set_bits(id, 5, data, 416);
                }
            }
            public byte[] uid2_TRY(Inside ph)
            {
                if(ph.field_bit !=  421 && !try_visit_field(ph, 421)) return null;
                return uid2_GET(ph, new byte[ph.items], 0);
            }
            public byte[]uid2_GET(Inside ph, byte[] dst_ch, int pos)
            {
                for(int BYTE = ph.BYTE, dst_max = pos + 18; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            public int uid2_LEN()
            {
                return 18;
            } public void uid2_SET(byte[] src, int pos, Inside ph)
            {
                if(ph.field_bit != 421)insert_field(ph, 421, 0);
                for(int BYTE =  ph.BYTE, src_max = pos + 18; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            } static readonly Meta meta148 = new Meta(148, 2, 4, 1, 54, 421, 0, _Gi);
        } public class LANDING_TARGET : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal LANDING_TARGET() : base(meta149, 0) { }
            internal LANDING_TARGET(int bytes) : base(meta149, bytes) { }
            public ulong time_usec //Timestamp (micros since boot or Unix epoch
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public byte target_num //The ID of the target if multiple targets are presen
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  8, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  8);}
            }

            public float angle_x //X-axis angular offset (in radians) of the target from the center of the imag
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  9, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 9);}
            }

            public float angle_y //Y-axis angular offset (in radians) of the target from the center of the imag
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  13, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 13);}
            }

            public float distance //Distance to the target from the vehicle in meter
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  17, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 17);}
            }

            public float size_x //Size in radians of target along x-axi
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  21, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 21);}
            }

            public float size_y //Size in radians of target along y-axi
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  25, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 25);}
            }

            public MAV_FRAME frame //MAV_FRAME enum specifying the whether the following feilds are earth-frame, body-frame, etc
            {
                get {  return (MAV_FRAME)(0 +  BitUtils.get_bits(data, 232, 4));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 232);}
            }

            public LANDING_TARGET_TYPE type //LANDING_TARGET_TYPE enum specifying the type of landing targe
            {
                get {  return (LANDING_TARGET_TYPE)(0 +  BitUtils.get_bits(data, 236, 3));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 3, data, 236);}
            }
            public float x_TRY(Inside ph)//X Position of the landing target on MAV_FRAM
            {
                if(ph.field_bit !=  239 && !try_visit_field(ph, 239)) return 0;
                return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  ph.BYTE, 4)));
            }
            public void x_SET(float src, Inside ph)//X Position of the landing target on MAV_FRAM
            {
                if(ph.field_bit != 239)insert_field(ph, 239, 0);
                BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src), 4, data, ph.BYTE);
            } public float y_TRY(Inside ph) //Y Position of the landing target on MAV_FRAM
            {
                if(ph.field_bit !=  240 && !try_visit_field(ph, 240)) return 0;
                return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  ph.BYTE, 4)));
            }
            public void y_SET(float src, Inside ph)//Y Position of the landing target on MAV_FRAM
            {
                if(ph.field_bit != 240)insert_field(ph, 240, 0);
                BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src), 4, data, ph.BYTE);
            } public float z_TRY(Inside ph) //Z Position of the landing target on MAV_FRAM
            {
                if(ph.field_bit !=  241 && !try_visit_field(ph, 241)) return 0;
                return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  ph.BYTE, 4)));
            }
            public void z_SET(float src, Inside ph)//Z Position of the landing target on MAV_FRAM
            {
                if(ph.field_bit != 241)insert_field(ph, 241, 0);
                BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src), 4, data, ph.BYTE);
            } public float[] q_TRY(Inside ph) //Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0
            {
                if(ph.field_bit !=  242 && !try_visit_field(ph, 242)) return null;
                return q_GET(ph, new float[ph.items], 0);
            }
            public float[]q_GET(Inside ph, float[] dst_ch, int pos) //Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0
            {
                for(int BYTE = ph.BYTE, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
            public int q_LEN()
            {
                return 4;
            } public void q_SET(float[] src, int pos, Inside ph)//Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0
            {
                if(ph.field_bit != 242)insert_field(ph, 242, 0);
                for(int BYTE =  ph.BYTE, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            } public byte position_valid_TRY(Inside ph)
            {
                if(ph.field_bit !=  243 && !try_visit_field(ph, 243)) return 0;
                return (byte)((byte) BitUtils.get_bytes(data,  ph.BYTE, 1));
            }
            public void position_valid_SET(byte src, Inside ph)
            {
                if(ph.field_bit != 243)insert_field(ph, 243, 0);
                BitUtils.set_bytes((ulong)(src), 1, data,  ph.BYTE);
            } static readonly Meta meta149 = new Meta(149, 0, 0, 1, 31, 239, 0, _xi, _mi, _ri, _ii, _Di);
        } public class ESTIMATOR_STATUS : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal ESTIMATOR_STATUS() : base(meta230, 0) { }
            internal ESTIMATOR_STATUS(int bytes) : base(meta230, bytes) { }
            public ulong time_usec //Timestamp (micros since boot or Unix epoch
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public float vel_ratio //Velocity innovation test rati
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float pos_horiz_ratio //Horizontal position innovation test rati
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float pos_vert_ratio //Vertical position innovation test rati
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float mag_ratio //Magnetometer innovation test rati
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float hagl_ratio //Height above terrain innovation test rati
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float tas_ratio //True airspeed innovation test rati
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }

            public float pos_horiz_accuracy //Horizontal position 1-STD accuracy relative to the EKF local origin (m
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  32, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 32);}
            }

            public float pos_vert_accuracy //Vertical position 1-STD accuracy relative to the EKF local origin (m
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  36, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 36);}
            }

            public ESTIMATOR_STATUS_FLAGS flags //Integer bitmask indicating which EKF outputs are valid. See definition for ESTIMATOR_STATUS_FLAGS
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
                set
                {
                    ulong id = 0;
                    switch(value)
                    {
                        case ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE:
                            id = 0;
                            break;
                        case ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ:
                            id = 1;
                            break;
                        case ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT:
                            id = 2;
                            break;
                        case ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL:
                            id = 3;
                            break;
                        case ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS:
                            id = 4;
                            break;
                        case ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS:
                            id = 5;
                            break;
                        case ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL:
                            id = 6;
                            break;
                        case ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE:
                            id = 7;
                            break;
                        case ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL:
                            id = 8;
                            break;
                        case ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS:
                            id = 9;
                            break;
                        case ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH:
                            id = 10;
                            break;
                        default:
                            throw  new ArgumentException("Unknown enum " + value);
                    }
                    BitUtils.set_bits(id, 4, data, 320);
                }
            }
            static readonly Meta meta230 = new Meta(230, 0, 0, 1, 41, 324);
        } public class WIND_COV : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal WIND_COV() : base(meta231, 0) { }
            internal WIND_COV(int bytes) : base(meta231, bytes) { }
            public ulong time_usec //Timestamp (micros since boot or Unix epoch
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public float wind_x //Wind in X (NED) direction in m/
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float wind_y //Wind in Y (NED) direction in m/
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float wind_z //Wind in Z (NED) direction in m/
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float var_horiz //Variability of the wind in XY. RMS of a 1 Hz lowpassed wind estimate
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float var_vert //Variability of the wind in Z. RMS of a 1 Hz lowpassed wind estimate
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float wind_alt //AMSL altitude (m) this measurement was taken a
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }

            public float horiz_accuracy //Horizontal speed 1-STD accurac
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  32, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 32);}
            }

            public float vert_accuracy //Vertical speed 1-STD accurac
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  36, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 36);}
            }
            static readonly Meta meta231 = new Meta(231, 0, 0, 1, 40, 320);
        } public class GPS_INPUT : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal GPS_INPUT() : base(meta232, 0) { }
            internal GPS_INPUT(int bytes) : base(meta232, bytes) { }
            public ushort time_week //GPS week numbe
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public uint time_week_ms //GPS time (milliseconds from start of GPS week
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  2, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  2);}
            }

            public ulong time_usec //Timestamp (micros since boot or Unix epoch
            {
                get {  return (BitUtils.get_bytes(data,  6, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  6);}
            }

            public byte gps_id //ID of the GPS for multiple GPS input
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  14, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  14);}
            }

            public byte fix_type //0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RT
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  15, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  15);}
            }

            public int lat //Latitude (WGS84), in degrees * 1E
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  16, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  16);}
            }

            public int lon //Longitude (WGS84), in degrees * 1E
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  20, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  20);}
            }

            public float alt //Altitude (AMSL, not WGS84), in m (positive for up
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float hdop //GPS HDOP horizontal dilution of position in
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }

            public float vdop //GPS VDOP vertical dilution of position in
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  32, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 32);}
            }

            public float vn //GPS velocity in m/s in NORTH direction in earth-fixed NED fram
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  36, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 36);}
            }

            public float ve //GPS velocity in m/s in EAST direction in earth-fixed NED fram
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  40, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 40);}
            }

            public float vd //GPS velocity in m/s in DOWN direction in earth-fixed NED fram
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  44, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 44);}
            }

            public float speed_accuracy //GPS speed accuracy in m/
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  48, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 48);}
            }

            public float horiz_accuracy //GPS horizontal accuracy in
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  52, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 52);}
            }

            public float vert_accuracy //GPS vertical accuracy in
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  56, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 56);}
            }

            public byte satellites_visible //Number of satellites visible
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  60, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  60);}
            }

            public GPS_INPUT_IGNORE_FLAGS ignore_flags //Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum).  All other fields must be provide
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
                set
                {
                    ulong id = 0;
                    switch(value)
                    {
                        case GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT:
                            id = 0;
                            break;
                        case GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP:
                            id = 1;
                            break;
                        case GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP:
                            id = 2;
                            break;
                        case GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ:
                            id = 3;
                            break;
                        case GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT:
                            id = 4;
                            break;
                        case GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY:
                            id = 5;
                            break;
                        case GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY:
                            id = 6;
                            break;
                        case GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY:
                            id = 7;
                            break;
                        default:
                            throw  new ArgumentException("Unknown enum " + value);
                    }
                    BitUtils.set_bits(id, 4, data, 488);
                }
            }
            static readonly Meta meta232 = new Meta(232, 1, 1, 1, 62, 492);
        } public class GPS_RTCM_DATA : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal GPS_RTCM_DATA() : base(meta233, 0) { }
            internal GPS_RTCM_DATA(int bytes) : base(meta233, bytes) { }
            public byte flags
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte len //data lengt
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            public byte[] data_ //RTCM message (may be fragmented
            {
                get {return data__GET(new byte[180], 0);}
                set {data__SET(value, 0)  ;}
            }
            public byte[]data__GET(byte[] dst_ch, int pos)  //RTCM message (may be fragmented
            {
                for(int BYTE = 2, dst_max = pos + 180; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }

            public void data__SET(byte[] src, int pos)  //RTCM message (may be fragmented
            {
                for(int BYTE =  2, src_max = pos + 180; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }

            static readonly Meta meta233 = new Meta(233, 0, 0, 0, 182, 1456);
        } public class HIGH_LATENCY : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal HIGH_LATENCY() : base(meta234, 0) { }
            internal HIGH_LATENCY(int bytes) : base(meta234, bytes) { }
            public ushort heading //heading (centidegrees
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort wp_distance //distance to target (meters
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public uint custom_mode //A bitfield for use for autopilot-specific flags
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  4, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  4);}
            }

            public short roll //roll (centidegrees
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  8, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  8);}
            }

            public short pitch //pitch (centidegrees
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  10, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  10);}
            }

            public sbyte throttle //throttle (percentage
            {
                get {  return (sbyte)((sbyte) BitUtils.get_bytes(data,  12, 1));}
                set {  BitUtils.set_bytes((byte)(value), 1, data,  12);}
            }

            public short heading_sp //heading setpoint (centidegrees
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  13, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  13);}
            }

            public int latitude //Latitude, expressed as degrees * 1E
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  15, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  15);}
            }

            public int longitude //Longitude, expressed as degrees * 1E
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  19, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  19);}
            }

            public short altitude_amsl //Altitude above mean sea level (meters
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  23, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  23);}
            }

            public short altitude_sp //Altitude setpoint relative to the home position (meters
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  25, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  25);}
            }

            public byte airspeed //airspeed (m/s
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  27, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  27);}
            }

            public byte airspeed_sp //airspeed setpoint (m/s
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  28, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  28);}
            }

            public byte groundspeed //groundspeed (m/s
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  29, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  29);}
            }

            public sbyte climb_rate //climb rate (m/s
            {
                get {  return (sbyte)((sbyte) BitUtils.get_bytes(data,  30, 1));}
                set {  BitUtils.set_bytes((byte)(value), 1, data,  30);}
            }

            public byte gps_nsat //Number of satellites visible. If unknown, set to 25
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  31, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  31);}
            }

            public byte battery_remaining //Remaining battery (percentage
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  32, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  32);}
            }

            public sbyte temperature //Autopilot temperature (degrees C
            {
                get {  return (sbyte)((sbyte) BitUtils.get_bytes(data,  33, 1));}
                set {  BitUtils.set_bytes((byte)(value), 1, data,  33);}
            }

            public sbyte temperature_air //Air temperature (degrees C) from airspeed senso
            {
                get {  return (sbyte)((sbyte) BitUtils.get_bytes(data,  34, 1));}
                set {  BitUtils.set_bytes((byte)(value), 1, data,  34);}
            }

            public byte failsafe
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  35, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  35);}
            }

            public byte wp_num //current waypoint numbe
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  36, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  36);}
            }

            public MAV_MODE_FLAG base_mode //System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.
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
                    BitUtils.set_bits(id, 4, data, 296);
                }
            }

            public MAV_LANDED_STATE landed_state //The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown
            {
                get {  return (MAV_LANDED_STATE)(0 +  BitUtils.get_bits(data, 300, 3));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 3, data, 300);}
            }

            public GPS_FIX_TYPE gps_fix_type //See the GPS_FIX_TYPE enum
            {
                get {  return (GPS_FIX_TYPE)(0 +  BitUtils.get_bits(data, 303, 4));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 303);}
            }
            static readonly Meta meta234 = new Meta(234, 2, 1, 0, 39, 307);
        } public class VIBRATION : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal VIBRATION() : base(meta241, 0) { }
            internal VIBRATION(int bytes) : base(meta241, bytes) { }
            public uint clipping_0 //first accelerometer clipping coun
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public uint clipping_1 //second accelerometer clipping coun
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  4, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  4);}
            }

            public uint clipping_2 //third accelerometer clipping coun
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  8, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  8);}
            }

            public ulong time_usec //Timestamp (micros since boot or Unix epoch
            {
                get {  return (BitUtils.get_bytes(data,  12, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  12);}
            }

            public float vibration_x //Vibration levels on X-axi
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float vibration_y //Vibration levels on Y-axi
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 24);}
            }

            public float vibration_z //Vibration levels on Z-axi
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 28);}
            }
            static readonly Meta meta241 = new Meta(241, 0, 3, 1, 32, 256);
        } public class HOME_POSITION : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal HOME_POSITION() : base(meta242, 0) { }
            internal HOME_POSITION(int bytes) : base(meta242, bytes) { }
            public int latitude //Latitude (WGS84), in degrees * 1E
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  0);}
            }

            public int longitude //Longitude (WGS84, in degrees * 1E
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  4, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  4);}
            }

            public int altitude //Altitude (AMSL), in meters * 1000 (positive for up
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  8, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  8);}
            }

            public float x //Local X position of this position in the local coordinate fram
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float y //Local Y position of this position in the local coordinate fram
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }

            public float z //Local Z position of this position in the local coordinate fram
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 20);}
            }

            public float[] q
            {
                get {return q_GET(new float[4], 0);}
                set {q_SET(value, 0)  ;}
            }
            public float[]q_GET(float[] dst_ch, int pos)
            {
                for(int BYTE = 24, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }

            public void q_SET(float[] src, int pos)
            {
                for(int BYTE =  24, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }


            public float approach_x
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  40, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 40);}
            }

            public float approach_y
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  44, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 44);}
            }

            public float approach_z
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  48, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 48);}
            }
            public ulong time_usec_TRY(Inside ph)//Timestamp (microseconds since UNIX epoch or microseconds since system boot
            {
                if(ph.field_bit !=  416 && !try_visit_field(ph, 416)) return 0;
                return (BitUtils.get_bytes(data,  ph.BYTE, 8));
            }
            public void time_usec_SET(ulong src, Inside ph)//Timestamp (microseconds since UNIX epoch or microseconds since system boot
            {
                if(ph.field_bit != 416)insert_field(ph, 416, 0);
                BitUtils.set_bytes((ulong)(src), 8, data,  ph.BYTE);
            } static readonly Meta meta242 = new Meta(242, 0, 0, 0, 53, 416, 0, _HC);
        } public class SET_HOME_POSITION : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal SET_HOME_POSITION() : base(meta243, 0) { }
            internal SET_HOME_POSITION(int bytes) : base(meta243, bytes) { }
            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public int latitude //Latitude (WGS84), in degrees * 1E
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  1, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  1);}
            }

            public int longitude //Longitude (WGS84, in degrees * 1E
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  5, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  5);}
            }

            public int altitude //Altitude (AMSL), in meters * 1000 (positive for up
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  9, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  9);}
            }

            public float x //Local X position of this position in the local coordinate fram
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  13, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 13);}
            }

            public float y //Local Y position of this position in the local coordinate fram
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  17, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 17);}
            }

            public float z //Local Z position of this position in the local coordinate fram
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  21, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 21);}
            }

            public float[] q
            {
                get {return q_GET(new float[4], 0);}
                set {q_SET(value, 0)  ;}
            }
            public float[]q_GET(float[] dst_ch, int pos)
            {
                for(int BYTE = 25, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }

            public void q_SET(float[] src, int pos)
            {
                for(int BYTE =  25, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }


            public float approach_x
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  41, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 41);}
            }

            public float approach_y
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  45, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 45);}
            }

            public float approach_z
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  49, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 49);}
            }
            public ulong time_usec_TRY(Inside ph)//Timestamp (microseconds since UNIX epoch or microseconds since system boot
            {
                if(ph.field_bit !=  424 && !try_visit_field(ph, 424)) return 0;
                return (BitUtils.get_bytes(data,  ph.BYTE, 8));
            }
            public void time_usec_SET(ulong src, Inside ph)//Timestamp (microseconds since UNIX epoch or microseconds since system boot
            {
                if(ph.field_bit != 424)insert_field(ph, 424, 0);
                BitUtils.set_bytes((ulong)(src), 8, data,  ph.BYTE);
            } static readonly Meta meta243 = new Meta(243, 0, 0, 0, 54, 424, 0, _ID);
        } public class MESSAGE_INTERVAL : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal MESSAGE_INTERVAL() : base(meta244, 0) { }
            internal MESSAGE_INTERVAL(int bytes) : base(meta244, bytes) { }
            public ushort message_id //The ID of the requested MAVLink message. v1.0 is limited to 254 messages
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public int interval_us //0 indicates the interval at which it is sent
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  2, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  2);}
            }
            static readonly Meta meta244 = new Meta(244, 1, 0, 0, 6, 48);
        } public class EXTENDED_SYS_STATE : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal EXTENDED_SYS_STATE() : base(meta245, 0) { }
            internal EXTENDED_SYS_STATE(int bytes) : base(meta245, bytes) { }
            public MAV_VTOL_STATE vtol_state //The VTOL state if applicable. Is set to MAV_VTOL_STATE_UNDEFINED if UAV is not in VTOL configuratio
            {
                get {  return (MAV_VTOL_STATE)(0 +  BitUtils.get_bits(data, 0, 3));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 3, data, 0);}
            }

            public MAV_LANDED_STATE landed_state //The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown
            {
                get {  return (MAV_LANDED_STATE)(0 +  BitUtils.get_bits(data, 3, 3));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 3, data, 3);}
            }
            static readonly Meta meta245 = new Meta(245, 0, 0, 0, 1, 6);
        } public class ADSB_VEHICLE : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal ADSB_VEHICLE() : base(meta246, 0) { }
            internal ADSB_VEHICLE(int bytes) : base(meta246, bytes) { }
            public ushort heading //Course over ground in centidegree
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort hor_velocity //The horizontal velocity in centimeters/secon
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public ushort squawk //Squawk cod
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  4);}
            }

            public uint ICAO_address //ICAO addres
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  6, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  6);}
            }

            public int lat //Latitude, expressed as degrees * 1E
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  10, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  10);}
            }

            public int lon //Longitude, expressed as degrees * 1E
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  14, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  14);}
            }

            public int altitude //Altitude(ASL) in millimeter
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  18, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  18);}
            }

            public short ver_velocity //The vertical velocity in centimeters/second, positive is u
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  22, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  22);}
            }

            public byte tslc //Time since last communication in second
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  24, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  24);}
            }

            public ADSB_ALTITUDE_TYPE altitude_type //Type from ADSB_ALTITUDE_TYPE enu
            {
                get {  return (ADSB_ALTITUDE_TYPE)(0 +  BitUtils.get_bits(data, 200, 2));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 2, data, 200);}
            }

            public ADSB_EMITTER_TYPE emitter_type //Type from ADSB_EMITTER_TYPE enu
            {
                get {  return (ADSB_EMITTER_TYPE)(0 +  BitUtils.get_bits(data, 202, 5));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 5, data, 202);}
            }

            public ADSB_FLAGS flags //Flags to indicate various statuses including valid data field
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
                set
                {
                    ulong id = 0;
                    switch(value)
                    {
                        case ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS:
                            id = 0;
                            break;
                        case ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE:
                            id = 1;
                            break;
                        case ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING:
                            id = 2;
                            break;
                        case ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY:
                            id = 3;
                            break;
                        case ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN:
                            id = 4;
                            break;
                        case ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK:
                            id = 5;
                            break;
                        case ADSB_FLAGS.ADSB_FLAGS_SIMULATED:
                            id = 6;
                            break;
                        default:
                            throw  new ArgumentException("Unknown enum " + value);
                    }
                    BitUtils.set_bits(id, 3, data, 207);
                }
            }
            public string callsign_TRY(Inside ph)//The callsign, 8+nul
            {
                if(ph.field_bit !=  210 && !try_visit_field(ph, 210)  ||  !try_visit_item(ph, 0)) return null;
                return new string(callsign_GET(ph, new char[ph.items], 0));
            }
            public char[]callsign_GET(Inside ph, char[] dst_ch, int pos) //The callsign, 8+nul
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int callsign_LEN(Inside ph)
            {
                return (ph.field_bit !=  210 && !try_visit_field(ph, 210)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            } public void callsign_SET(string src, Inside ph) //The callsign, 8+nul
            {callsign_SET(src.ToCharArray(), 0, src.Length, ph);} public void callsign_SET(char[] src, int pos, int items, Inside ph) //The callsign, 8+nul
            {
                if(ph.field_bit != 210 && insert_field(ph, 210, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            } static readonly Meta meta246 = new Meta(246, 3, 1, 0, 28, 210, 0, _zD);
        } public class COLLISION : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal COLLISION() : base(meta247, 0) { }
            internal COLLISION(int bytes) : base(meta247, bytes) { }
            public uint id //Unique identifier, domain based on src fiel
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public float time_to_minimum_delta //Estimated time until collision occurs (seconds
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 4);}
            }

            public float altitude_minimum_delta //Closest vertical distance in meters between vehicle and objec
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float horizontal_minimum_delta //Closest horizontal distance in meteres between vehicle and objec
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public MAV_COLLISION_SRC src_ //Collision data sourc
            {
                get {  return (MAV_COLLISION_SRC)(0 +  BitUtils.get_bits(data, 128, 2));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 2, data, 128);}
            }

            public MAV_COLLISION_ACTION action //Action that is being taken to avoid this collisio
            {
                get {  return (MAV_COLLISION_ACTION)(0 +  BitUtils.get_bits(data, 130, 3));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 3, data, 130);}
            }

            public MAV_COLLISION_THREAT_LEVEL threat_level //How concerned the aircraft is about this collisio
            {
                get {  return (MAV_COLLISION_THREAT_LEVEL)(0 +  BitUtils.get_bits(data, 133, 2));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 2, data, 133);}
            }
            static readonly Meta meta247 = new Meta(247, 0, 1, 0, 17, 135);
        } public class V2_EXTENSION : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal V2_EXTENSION() : base(meta248, 0) { }
            internal V2_EXTENSION(int bytes) : base(meta248, bytes) { }
            public ushort message_type
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public byte target_network //Network ID (0 for broadcast
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }

            public byte target_system //System ID (0 for broadcast
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  3);}
            }

            public byte target_component //Component ID (0 for broadcast
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  4);}
            }

            public byte[] payload
            {
                get {return payload_GET(new byte[249], 0);}
                set {payload_SET(value, 0)  ;}
            }
            public byte[]payload_GET(byte[] dst_ch, int pos)
            {
                for(int BYTE = 5, dst_max = pos + 249; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }

            public void payload_SET(byte[] src, int pos)
            {
                for(int BYTE =  5, src_max = pos + 249; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }

            static readonly Meta meta248 = new Meta(248, 1, 0, 0, 254, 2032);
        } public class MEMORY_VECT : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal MEMORY_VECT() : base(meta249, 0) { }
            internal MEMORY_VECT(int bytes) : base(meta249, bytes) { }
            public ushort address //Starting address of the debug variable
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public byte ver //Version code of the type variable. 0=unknown, type ignored and assumed short. 1=as belo
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }

            public byte type //Type code of the memory variables. for ver = 1: 0=16 x short, 1=16 x ushort, 2=16 x Q15, 3=16 x 1Q
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  3);}
            }

            public sbyte[] value //Memory contents at specified addres
            {
                get {return value_GET(new sbyte[32], 0);}
                set {value_SET(value, 0)  ;}
            }
            public sbyte[]value_GET(sbyte[] dst_ch, int pos)  //Memory contents at specified addres
            {
                for(int BYTE = 4, dst_max = pos + 32; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (sbyte)((sbyte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }

            public void value_SET(sbyte[] src, int pos)  //Memory contents at specified addres
            {
                for(int BYTE =  4, src_max = pos + 32; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((byte)(src[pos]), 1, data,  BYTE);
            }

            static readonly Meta meta249 = new Meta(249, 1, 0, 0, 36, 288);
        } public class DEBUG_VECT : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal DEBUG_VECT() : base(meta250, 0) { }
            internal DEBUG_VECT(int bytes) : base(meta250, bytes) { }
            public ulong time_usec //Timestam
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public float x
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float y
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }

            public float z
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 16);}
            }
            public string name_TRY(Inside ph)//Nam
            {
                if(ph.field_bit !=  160 && !try_visit_field(ph, 160)  ||  !try_visit_item(ph, 0)) return null;
                return new string(name_GET(ph, new char[ph.items], 0));
            }
            public char[]name_GET(Inside ph, char[] dst_ch, int pos) //Nam
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int name_LEN(Inside ph)
            {
                return (ph.field_bit !=  160 && !try_visit_field(ph, 160)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            } public void name_SET(string src, Inside ph) //Nam
            {name_SET(src.ToCharArray(), 0, src.Length, ph);} public void name_SET(char[] src, int pos, int items, Inside ph) //Nam
            {
                if(ph.field_bit != 160 && insert_field(ph, 160, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            } static readonly Meta meta250 = new Meta(250, 0, 0, 1, 21, 160, 0, _eD);
        } public class NAMED_VALUE_FLOAT : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal NAMED_VALUE_FLOAT() : base(meta251, 0) { }
            internal NAMED_VALUE_FLOAT(int bytes) : base(meta251, bytes) { }
            public uint time_boot_ms //Timestamp (milliseconds since system boot
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public float value //Floating point valu
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 4);}
            }
            public string name_TRY(Inside ph)//Name of the debug variabl
            {
                if(ph.field_bit !=  64 && !try_visit_field(ph, 64)  ||  !try_visit_item(ph, 0)) return null;
                return new string(name_GET(ph, new char[ph.items], 0));
            }
            public char[]name_GET(Inside ph, char[] dst_ch, int pos) //Name of the debug variabl
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int name_LEN(Inside ph)
            {
                return (ph.field_bit !=  64 && !try_visit_field(ph, 64)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            } public void name_SET(string src, Inside ph) //Name of the debug variabl
            {name_SET(src.ToCharArray(), 0, src.Length, ph);} public void name_SET(char[] src, int pos, int items, Inside ph) //Name of the debug variabl
            {
                if(ph.field_bit != 64 && insert_field(ph, 64, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            } static readonly Meta meta251 = new Meta(251, 0, 1, 0, 9, 64, 0, _HD);
        } public class NAMED_VALUE_INT : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal NAMED_VALUE_INT() : base(meta252, 0) { }
            internal NAMED_VALUE_INT(int bytes) : base(meta252, bytes) { }
            public uint time_boot_ms //Timestamp (milliseconds since system boot
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public int value //Signed integer valu
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  4, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  4);}
            }
            public string name_TRY(Inside ph)//Name of the debug variabl
            {
                if(ph.field_bit !=  64 && !try_visit_field(ph, 64)  ||  !try_visit_item(ph, 0)) return null;
                return new string(name_GET(ph, new char[ph.items], 0));
            }
            public char[]name_GET(Inside ph, char[] dst_ch, int pos) //Name of the debug variabl
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int name_LEN(Inside ph)
            {
                return (ph.field_bit !=  64 && !try_visit_field(ph, 64)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            } public void name_SET(string src, Inside ph) //Name of the debug variabl
            {name_SET(src.ToCharArray(), 0, src.Length, ph);} public void name_SET(char[] src, int pos, int items, Inside ph) //Name of the debug variabl
            {
                if(ph.field_bit != 64 && insert_field(ph, 64, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            } static readonly Meta meta252 = new Meta(252, 0, 1, 0, 9, 64, 0, _fM);
        } public class STATUSTEXT : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal STATUSTEXT() : base(meta253, 0) { }
            internal STATUSTEXT(int bytes) : base(meta253, bytes) { }
            public MAV_SEVERITY severity //Severity of status. Relies on the definitions within RFC-5424. See enum MAV_SEVERITY
            {
                get {  return (MAV_SEVERITY)(0 +  BitUtils.get_bits(data, 0, 4));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 0);}
            }
            public string text_TRY(Inside ph)//Status text message, without null termination characte
            {
                if(ph.field_bit !=  4 && !try_visit_field(ph, 4)  ||  !try_visit_item(ph, 0)) return null;
                return new string(text_GET(ph, new char[ph.items], 0));
            }
            public char[]text_GET(Inside ph, char[] dst_ch, int pos) //Status text message, without null termination characte
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int text_LEN(Inside ph)
            {
                return (ph.field_bit !=  4 && !try_visit_field(ph, 4)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            } public void text_SET(string src, Inside ph) //Status text message, without null termination characte
            {text_SET(src.ToCharArray(), 0, src.Length, ph);} public void text_SET(char[] src, int pos, int items, Inside ph) //Status text message, without null termination characte
            {
                if(ph.field_bit != 4 && insert_field(ph, 4, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            } static readonly Meta meta253 = new Meta(253, 0, 0, 0, 2, 4, 0, _JM);
        } public class DEBUG : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal DEBUG() : base(meta254, 0) { }
            internal DEBUG(int bytes) : base(meta254, bytes) { }
            public uint time_boot_ms //Timestamp (milliseconds since system boot
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public byte ind //index of debug variabl
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  4);}
            }

            public float value //DEBUG valu
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  5, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 5);}
            }
            static readonly Meta meta254 = new Meta(254, 0, 1, 0, 9, 72);
        } public class SETUP_SIGNING : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal SETUP_SIGNING() : base(meta256, 0) { }
            internal SETUP_SIGNING(int bytes) : base(meta256, bytes) { }
            public ulong initial_timestamp //initial timestam
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  0);}
            }

            public byte target_system //system id of the targe
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  8, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  8);}
            }

            public byte target_component //component ID of the targe
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  9, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  9);}
            }

            public byte[] secret_key //signing ke
            {
                get {return secret_key_GET(new byte[32], 0);}
                set {secret_key_SET(value, 0)  ;}
            }
            public byte[]secret_key_GET(byte[] dst_ch, int pos)  //signing ke
            {
                for(int BYTE = 10, dst_max = pos + 32; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }

            public void secret_key_SET(byte[] src, int pos)  //signing ke
            {
                for(int BYTE =  10, src_max = pos + 32; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }

            static readonly Meta meta256 = new Meta(256, 0, 0, 1, 42, 336);
        } public class BUTTON_CHANGE : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal BUTTON_CHANGE() : base(meta257, 0) { }
            internal BUTTON_CHANGE(int bytes) : base(meta257, bytes) { }
            public uint time_boot_ms //Timestamp (milliseconds since system boot
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public uint last_change_ms //Time of last change of button stat
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  4, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  4);}
            }

            public byte state //Bitmap state of button
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  8, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  8);}
            }
            static readonly Meta meta257 = new Meta(257, 0, 2, 0, 9, 72);
        } public class PLAY_TUNE : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal PLAY_TUNE() : base(meta258, 0) { }
            internal PLAY_TUNE(int bytes) : base(meta258, bytes) { }
            public byte target_system //System I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }
            public string tune_TRY(Inside ph)//tune in board specific forma
            {
                if(ph.field_bit !=  16 && !try_visit_field(ph, 16)  ||  !try_visit_item(ph, 0)) return null;
                return new string(tune_GET(ph, new char[ph.items], 0));
            }
            public char[]tune_GET(Inside ph, char[] dst_ch, int pos) //tune in board specific forma
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int tune_LEN(Inside ph)
            {
                return (ph.field_bit !=  16 && !try_visit_field(ph, 16)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            } public void tune_SET(string src, Inside ph) //tune in board specific forma
            {tune_SET(src.ToCharArray(), 0, src.Length, ph);} public void tune_SET(char[] src, int pos, int items, Inside ph) //tune in board specific forma
            {
                if(ph.field_bit != 16 && insert_field(ph, 16, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            } static readonly Meta meta258 = new Meta(258, 0, 0, 0, 3, 16, 0, _rM);
        } public class CAMERA_INFORMATION : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal CAMERA_INFORMATION() : base(meta259, 0) { }
            internal CAMERA_INFORMATION(int bytes) : base(meta259, bytes) { }
            public ushort resolution_h //Image resolution in pixels horizonta
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort resolution_v //Image resolution in pixels vertica
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public ushort cam_definition_version //Camera definition version (iteration
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  4);}
            }

            public uint time_boot_ms //Timestamp (milliseconds since system boot
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  6, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  6);}
            }

            public uint firmware_version //0xff = Major
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  10, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  10);}
            }

            public byte[] vendor_name //Name of the camera vendo
            {
                get {return vendor_name_GET(new byte[32], 0);}
                set {vendor_name_SET(value, 0)  ;}
            }
            public byte[]vendor_name_GET(byte[] dst_ch, int pos)  //Name of the camera vendo
            {
                for(int BYTE = 14, dst_max = pos + 32; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }

            public void vendor_name_SET(byte[] src, int pos)  //Name of the camera vendo
            {
                for(int BYTE =  14, src_max = pos + 32; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }


            public byte[] model_name //Name of the camera mode
            {
                get {return model_name_GET(new byte[32], 0);}
                set {model_name_SET(value, 0)  ;}
            }
            public byte[]model_name_GET(byte[] dst_ch, int pos)  //Name of the camera mode
            {
                for(int BYTE = 46, dst_max = pos + 32; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }

            public void model_name_SET(byte[] src, int pos)  //Name of the camera mode
            {
                for(int BYTE =  46, src_max = pos + 32; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }


            public float focal_length //Focal length in m
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  78, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 78);}
            }

            public float sensor_size_h //Image sensor size horizontal in m
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  82, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 82);}
            }

            public float sensor_size_v //Image sensor size vertical in m
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  86, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 86);}
            }

            public byte lens_id //Reserved for a lens I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  90, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  90);}
            }

            public CAMERA_CAP_FLAGS flags //CAMERA_CAP_FLAGS enum flags (bitmap) describing camera capabilities
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
                set
                {
                    ulong id = 0;
                    switch(value)
                    {
                        case CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO:
                            id = 0;
                            break;
                        case CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE:
                            id = 1;
                            break;
                        case CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_MODES:
                            id = 2;
                            break;
                        case CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE:
                            id = 3;
                            break;
                        case CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE:
                            id = 4;
                            break;
                        case CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE:
                            id = 5;
                            break;
                        default:
                            throw  new ArgumentException("Unknown enum " + value);
                    }
                    BitUtils.set_bits(id, 3, data, 728);
                }
            }
            public string cam_definition_uri_TRY(Inside ph)//Camera definition URI (if any, otherwise only basic functions will be available)
            {
                if(ph.field_bit !=  731 && !try_visit_field(ph, 731)  ||  !try_visit_item(ph, 0)) return null;
                return new string(cam_definition_uri_GET(ph, new char[ph.items], 0));
            }
            public char[]cam_definition_uri_GET(Inside ph, char[] dst_ch, int pos) //Camera definition URI (if any, otherwise only basic functions will be available)
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int cam_definition_uri_LEN(Inside ph)
            {
                return (ph.field_bit !=  731 && !try_visit_field(ph, 731)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            } public void cam_definition_uri_SET(string src, Inside ph) //Camera definition URI (if any, otherwise only basic functions will be available)
            {cam_definition_uri_SET(src.ToCharArray(), 0, src.Length, ph);} public void cam_definition_uri_SET(char[] src, int pos, int items, Inside ph) //Camera definition URI (if any, otherwise only basic functions will be available)
            {
                if(ph.field_bit != 731 && insert_field(ph, 731, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            } static readonly Meta meta259 = new Meta(259, 3, 2, 0, 93, 731, 0, _ZM);
        } public class CAMERA_SETTINGS : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal CAMERA_SETTINGS() : base(meta260, 0) { }
            internal CAMERA_SETTINGS(int bytes) : base(meta260, bytes) { }
            public uint time_boot_ms //Timestamp (milliseconds since system boot
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public CAMERA_MODE mode_id //Camera mode (CAMERA_MODE
            {
                get {  return (CAMERA_MODE)(0 +  BitUtils.get_bits(data, 32, 2));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 2, data, 32);}
            }
            static readonly Meta meta260 = new Meta(260, 0, 1, 0, 5, 34);
        } public class STORAGE_INFORMATION : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal STORAGE_INFORMATION() : base(meta261, 0) { }
            internal STORAGE_INFORMATION(int bytes) : base(meta261, bytes) { }
            public uint time_boot_ms //Timestamp (milliseconds since system boot
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public byte storage_id //Storage ID (1 for first, 2 for second, etc.
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  4);}
            }

            public byte storage_count //Number of storage device
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  5);}
            }

            public byte status //Status of storage (0 not available, 1 unformatted, 2 formatted
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  6, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  6);}
            }

            public float total_capacity //Total capacity in Mi
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  7, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 7);}
            }

            public float used_capacity //Used capacity in Mi
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  11, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 11);}
            }

            public float available_capacity //Available capacity in Mi
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  15, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 15);}
            }

            public float read_speed //Read speed in MiB/
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  19, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 19);}
            }

            public float write_speed //Write speed in MiB/
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  23, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 23);}
            }
            static readonly Meta meta261 = new Meta(261, 0, 1, 0, 27, 216);
        } public class CAMERA_CAPTURE_STATUS : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal CAMERA_CAPTURE_STATUS() : base(meta262, 0) { }
            internal CAMERA_CAPTURE_STATUS(int bytes) : base(meta262, bytes) { }
            public uint time_boot_ms //Timestamp (milliseconds since system boot
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public uint recording_time_ms //Time in milliseconds since recording starte
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  4, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  4);}
            }

            public byte image_status
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  8, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  8);}
            }

            public byte video_status //Current status of video capturing (0: idle, 1: capture in progress
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  9, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  9);}
            }

            public float image_interval //Image capture interval in second
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  10, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 10);}
            }

            public float available_capacity //Available storage capacity in Mi
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  14, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 14);}
            }
            static readonly Meta meta262 = new Meta(262, 0, 2, 0, 18, 144);
        } public class CAMERA_IMAGE_CAPTURED : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal CAMERA_IMAGE_CAPTURED() : base(meta263, 0) { }
            internal CAMERA_IMAGE_CAPTURED(int bytes) : base(meta263, bytes) { }
            public uint time_boot_ms //Timestamp (milliseconds since system boot
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public ulong time_utc //Timestamp (microseconds since UNIX epoch) in UTC. 0 for unknown
            {
                get {  return (BitUtils.get_bytes(data,  4, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  4);}
            }

            public byte camera_id //Camera ID (1 for first, 2 for second, etc.
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  12, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  12);}
            }

            public int lat //Latitude, expressed as degrees * 1E7 where image was take
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  13, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  13);}
            }

            public int lon //Longitude, expressed as degrees * 1E7 where capture was take
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  17, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  17);}
            }

            public int alt //Altitude in meters, expressed as * 1E3 (AMSL, not WGS84) where image was take
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  21, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  21);}
            }

            public int relative_alt //Altitude above ground in meters, expressed as * 1E3 where image was take
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  25, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  25);}
            }

            public float[] q //Quaternion of camera orientation (w, x, y, z order, zero-rotation is 0, 0, 0, 0
            {
                get {return q_GET(new float[4], 0);}
                set {q_SET(value, 0)  ;}
            }
            public float[]q_GET(float[] dst_ch, int pos)  //Quaternion of camera orientation (w, x, y, z order, zero-rotation is 0, 0, 0, 0
            {
                for(int BYTE = 29, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }

            public void q_SET(float[] src, int pos)  //Quaternion of camera orientation (w, x, y, z order, zero-rotation is 0, 0, 0, 0
            {
                for(int BYTE =  29, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
                    BitUtils.set_bytes(BitUtils.FloatToInt32Bits(src[pos]), 4, data, BYTE);
            }


            public int image_index //Zero based index of this image (image count since armed -1
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  45, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  45);}
            }

            public sbyte capture_result //Boolean indicating success (1) or failure (0) while capturing this image
            {
                get {  return (sbyte)((sbyte) BitUtils.get_bytes(data,  49, 1));}
                set {  BitUtils.set_bytes((byte)(value), 1, data,  49);}
            }
            public string file_url_TRY(Inside ph)//foo.jpg if camera provides an HTTP interface
            {
                if(ph.field_bit !=  402 && !try_visit_field(ph, 402)  ||  !try_visit_item(ph, 0)) return null;
                return new string(file_url_GET(ph, new char[ph.items], 0));
            }
            public char[]file_url_GET(Inside ph, char[] dst_ch, int pos) //foo.jpg if camera provides an HTTP interface
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int file_url_LEN(Inside ph)
            {
                return (ph.field_bit !=  402 && !try_visit_field(ph, 402)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            } public void file_url_SET(string src, Inside ph) //foo.jpg if camera provides an HTTP interface
            {file_url_SET(src.ToCharArray(), 0, src.Length, ph);} public void file_url_SET(char[] src, int pos, int items, Inside ph) //foo.jpg if camera provides an HTTP interface
            {
                if(ph.field_bit != 402 && insert_field(ph, 402, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            } static readonly Meta meta263 = new Meta(263, 0, 1, 1, 51, 402, 2, _GL);
        } public class FLIGHT_INFORMATION : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal FLIGHT_INFORMATION() : base(meta264, 0) { }
            internal FLIGHT_INFORMATION(int bytes) : base(meta264, bytes) { }
            public uint time_boot_ms //Timestamp (milliseconds since system boot
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public ulong arming_time_utc //Timestamp at arming (microseconds since UNIX epoch) in UTC, 0 for unknow
            {
                get {  return (BitUtils.get_bytes(data,  4, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  4);}
            }

            public ulong takeoff_time_utc //Timestamp at takeoff (microseconds since UNIX epoch) in UTC, 0 for unknow
            {
                get {  return (BitUtils.get_bytes(data,  12, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  12);}
            }

            public ulong flight_uuid //Universally unique identifier (UUID) of flight, should correspond to name of logfile
            {
                get {  return (BitUtils.get_bytes(data,  20, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  20);}
            }
            static readonly Meta meta264 = new Meta(264, 0, 1, 3, 28, 224);
        } public class MOUNT_ORIENTATION : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal MOUNT_ORIENTATION() : base(meta265, 0) { }
            internal MOUNT_ORIENTATION(int bytes) : base(meta265, bytes) { }
            public uint time_boot_ms //Timestamp (milliseconds since system boot
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public float roll //Roll in degree
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 4);}
            }

            public float pitch //Pitch in degree
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 8);}
            }

            public float yaw //Yaw in degree
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }
            static readonly Meta meta265 = new Meta(265, 0, 1, 0, 16, 128);
        } public class LOGGING_DATA : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal LOGGING_DATA() : base(meta266, 0) { }
            internal LOGGING_DATA(int bytes) : base(meta266, bytes) { }
            public ushort sequence //sequence number (can wrap
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public byte target_system //system ID of the targe
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }

            public byte target_component //component ID of the targe
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  3);}
            }

            public byte length //data lengt
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  4);}
            }

            public byte first_message_offset
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  5);}
            }

            public byte[] data_ //logged dat
            {
                get {return data__GET(new byte[249], 0);}
                set {data__SET(value, 0)  ;}
            }
            public byte[]data__GET(byte[] dst_ch, int pos)  //logged dat
            {
                for(int BYTE = 6, dst_max = pos + 249; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }

            public void data__SET(byte[] src, int pos)  //logged dat
            {
                for(int BYTE =  6, src_max = pos + 249; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }

            static readonly Meta meta266 = new Meta(266, 1, 0, 0, 255, 2040);
        } public class LOGGING_DATA_ACKED : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal LOGGING_DATA_ACKED() : base(meta267, 0) { }
            internal LOGGING_DATA_ACKED(int bytes) : base(meta267, bytes) { }
            public ushort sequence //sequence number (can wrap
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public byte target_system //system ID of the targe
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }

            public byte target_component //component ID of the targe
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  3);}
            }

            public byte length //data lengt
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  4);}
            }

            public byte first_message_offset
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  5);}
            }

            public byte[] data_ //logged dat
            {
                get {return data__GET(new byte[249], 0);}
                set {data__SET(value, 0)  ;}
            }
            public byte[]data__GET(byte[] dst_ch, int pos)  //logged dat
            {
                for(int BYTE = 6, dst_max = pos + 249; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }

            public void data__SET(byte[] src, int pos)  //logged dat
            {
                for(int BYTE =  6, src_max = pos + 249; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }

            static readonly Meta meta267 = new Meta(267, 1, 0, 0, 255, 2040);
        } public class LOGGING_ACK : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal LOGGING_ACK() : base(meta268, 0) { }
            internal LOGGING_ACK(int bytes) : base(meta268, bytes) { }
            public ushort sequence //sequence number (must match the one in LOGGING_DATA_ACKED
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public byte target_system //system ID of the targe
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  2);}
            }

            public byte target_component //component ID of the targe
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  3);}
            }
            static readonly Meta meta268 = new Meta(268, 1, 0, 0, 4, 32);
        } public class VIDEO_STREAM_INFORMATION : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal VIDEO_STREAM_INFORMATION() : base(meta269, 0) { }
            internal VIDEO_STREAM_INFORMATION(int bytes) : base(meta269, bytes) { }
            public ushort resolution_h //Resolution horizontal in pixel
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort resolution_v //Resolution vertical in pixel
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public ushort rotation //Video image rotation clockwis
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  4);}
            }

            public uint bitrate //Bit rate in bits per secon
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  6, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  6);}
            }

            public byte camera_id //Camera ID (1 for first, 2 for second, etc.
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  10, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  10);}
            }

            public byte status //Current status of video streaming (0: not running, 1: in progress
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  11, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  11);}
            }

            public float framerate //Frames per secon
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 12);}
            }
            public string uri_TRY(Inside ph)//Video stream UR
            {
                if(ph.field_bit !=  130 && !try_visit_field(ph, 130)  ||  !try_visit_item(ph, 0)) return null;
                return new string(uri_GET(ph, new char[ph.items], 0));
            }
            public char[]uri_GET(Inside ph, char[] dst_ch, int pos) //Video stream UR
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int uri_LEN(Inside ph)
            {
                return (ph.field_bit !=  130 && !try_visit_field(ph, 130)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            } public void uri_SET(string src, Inside ph) //Video stream UR
            {uri_SET(src.ToCharArray(), 0, src.Length, ph);} public void uri_SET(char[] src, int pos, int items, Inside ph) //Video stream UR
            {
                if(ph.field_bit != 130 && insert_field(ph, 130, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            } static readonly Meta meta269 = new Meta(269, 3, 1, 0, 17, 130, 2, _uL);
        } public class SET_VIDEO_STREAM_SETTINGS : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal SET_VIDEO_STREAM_SETTINGS() : base(meta270, 0) { }
            internal SET_VIDEO_STREAM_SETTINGS(int bytes) : base(meta270, bytes) { }
            public ushort resolution_h //Resolution horizontal in pixels (set to -1 for highest resolution possible
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort resolution_v //Resolution vertical in pixels (set to -1 for highest resolution possible
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public ushort rotation //Video image rotation clockwise (0-359 degrees
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  4);}
            }

            public uint bitrate //Bit rate in bits per second (set to -1 for auto
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  6, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  6);}
            }

            public byte target_system //system ID of the targe
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  10, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  10);}
            }

            public byte target_component //component ID of the targe
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  11, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  11);}
            }

            public byte camera_id //Camera ID (1 for first, 2 for second, etc.
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  12, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  12);}
            }

            public float framerate //Frames per second (set to -1 for highest framerate possible
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  13, 4)));}
                set {  BitUtils.set_bytes(BitUtils.FloatToInt32Bits(value), 4, data, 13);}
            }
            public string uri_TRY(Inside ph)//Video stream UR
            {
                if(ph.field_bit !=  138 && !try_visit_field(ph, 138)  ||  !try_visit_item(ph, 0)) return null;
                return new string(uri_GET(ph, new char[ph.items], 0));
            }
            public char[]uri_GET(Inside ph, char[] dst_ch, int pos) //Video stream UR
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int uri_LEN(Inside ph)
            {
                return (ph.field_bit !=  138 && !try_visit_field(ph, 138)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            } public void uri_SET(string src, Inside ph) //Video stream UR
            {uri_SET(src.ToCharArray(), 0, src.Length, ph);} public void uri_SET(char[] src, int pos, int items, Inside ph) //Video stream UR
            {
                if(ph.field_bit != 138 && insert_field(ph, 138, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            } static readonly Meta meta270 = new Meta(270, 3, 1, 0, 18, 138, 2, _yL);
        } public class WIFI_CONFIG_AP : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal WIFI_CONFIG_AP() : base(meta299, 0) { }
            internal WIFI_CONFIG_AP(int bytes) : base(meta299, bytes) { }  public string ssid_TRY(Inside ph)//Name of Wi-Fi network (SSID). Leave it blank to leave it unchanged
            {
                if(ph.field_bit !=  2 && !try_visit_field(ph, 2)  ||  !try_visit_item(ph, 0)) return null;
                return new string(ssid_GET(ph, new char[ph.items], 0));
            }
            public char[]ssid_GET(Inside ph, char[] dst_ch, int pos) //Name of Wi-Fi network (SSID). Leave it blank to leave it unchanged
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int ssid_LEN(Inside ph)
            {
                return (ph.field_bit !=  2 && !try_visit_field(ph, 2)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            } public void ssid_SET(string src, Inside ph) //Name of Wi-Fi network (SSID). Leave it blank to leave it unchanged
            {ssid_SET(src.ToCharArray(), 0, src.Length, ph);} public void ssid_SET(char[] src, int pos, int items, Inside ph) //Name of Wi-Fi network (SSID). Leave it blank to leave it unchanged
            {
                if(ph.field_bit != 2 && insert_field(ph, 2, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            } public string password_TRY(Inside ph)//Password. Leave it blank for an open AP
            {
                if(ph.field_bit !=  3 && !try_visit_field(ph, 3)  ||  !try_visit_item(ph, 0)) return null;
                return new string(password_GET(ph, new char[ph.items], 0));
            }
            public char[]password_GET(Inside ph, char[] dst_ch, int pos) //Password. Leave it blank for an open AP
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int password_LEN(Inside ph)
            {
                return (ph.field_bit !=  3 && !try_visit_field(ph, 3)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            } public void password_SET(string src, Inside ph) //Password. Leave it blank for an open AP
            {password_SET(src.ToCharArray(), 0, src.Length, ph);} public void password_SET(char[] src, int pos, int items, Inside ph) //Password. Leave it blank for an open AP
            {
                if(ph.field_bit != 3 && insert_field(ph, 3, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            } static readonly Meta meta299 = new Meta(299, 0, 0, 0, 1, 2, 2, _pL, _qL);
        } public class PROTOCOL_VERSION : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal PROTOCOL_VERSION() : base(meta300, 0) { }
            internal PROTOCOL_VERSION(int bytes) : base(meta300, bytes) { }
            public ushort version //Currently active MAVLink version number * 100: v1.0 is 100, v2.0 is 200, etc
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort min_version //Minimum MAVLink version supporte
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public ushort max_version //Maximum MAVLink version supported (set to the same value as version by default
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  4);}
            }

            public byte[] spec_version_hash //The first 8 bytes (not characters printed in hex!) of the git hash
            {
                get {return spec_version_hash_GET(new byte[8], 0);}
                set {spec_version_hash_SET(value, 0)  ;}
            }
            public byte[]spec_version_hash_GET(byte[] dst_ch, int pos)  //The first 8 bytes (not characters printed in hex!) of the git hash
            {
                for(int BYTE = 6, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }

            public void spec_version_hash_SET(byte[] src, int pos)  //The first 8 bytes (not characters printed in hex!) of the git hash
            {
                for(int BYTE =  6, src_max = pos + 8; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }


            public byte[] library_version_hash //The first 8 bytes (not characters printed in hex!) of the git hash
            {
                get {return library_version_hash_GET(new byte[8], 0);}
                set {library_version_hash_SET(value, 0)  ;}
            }
            public byte[]library_version_hash_GET(byte[] dst_ch, int pos)  //The first 8 bytes (not characters printed in hex!) of the git hash
            {
                for(int BYTE = 14, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }

            public void library_version_hash_SET(byte[] src, int pos)  //The first 8 bytes (not characters printed in hex!) of the git hash
            {
                for(int BYTE =  14, src_max = pos + 8; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }

            static readonly Meta meta300 = new Meta(300, 3, 0, 0, 22, 176);
        } public class UAVCAN_NODE_STATUS : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal UAVCAN_NODE_STATUS() : base(meta310, 0) { }
            internal UAVCAN_NODE_STATUS(int bytes) : base(meta310, bytes) { }
            public ushort vendor_specific_status_code //Vendor-specific status information
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public uint uptime_sec //The number of seconds since the start-up of the node
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  2, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  2);}
            }

            public ulong time_usec //Timestamp (microseconds since UNIX epoch or microseconds since system boot
            {
                get {  return (BitUtils.get_bytes(data,  6, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  6);}
            }

            public byte sub_mode //Not used currently
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  14, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  14);}
            }

            public UAVCAN_NODE_HEALTH health //Generalized node health status
            {
                get {  return (UAVCAN_NODE_HEALTH)(0 +  BitUtils.get_bits(data, 120, 3));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 3, data, 120);}
            }

            public UAVCAN_NODE_MODE mode //Generalized operating mode
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
                    BitUtils.set_bits(id, 3, data, 123);
                }
            }
            static readonly Meta meta310 = new Meta(310, 1, 1, 1, 16, 126);
        } public class UAVCAN_NODE_INFO : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal UAVCAN_NODE_INFO() : base(meta311, 0) { }
            internal UAVCAN_NODE_INFO(int bytes) : base(meta311, bytes) { }
            public uint uptime_sec //The number of seconds since the start-up of the node
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  0);}
            }

            public uint sw_vcs_commit //Version control system (VCS) revision identifier (e.g. git short commit hash). Zero if unknown
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  4, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  4);}
            }

            public ulong time_usec //Timestamp (microseconds since UNIX epoch or microseconds since system boot
            {
                get {  return (BitUtils.get_bytes(data,  8, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  8);}
            }

            public byte hw_version_major //Hardware major version number
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  16, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  16);}
            }

            public byte hw_version_minor //Hardware minor version number
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  17, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  17);}
            }

            public byte[] hw_unique_id //Hardware unique 128-bit ID
            {
                get {return hw_unique_id_GET(new byte[16], 0);}
                set {hw_unique_id_SET(value, 0)  ;}
            }
            public byte[]hw_unique_id_GET(byte[] dst_ch, int pos)  //Hardware unique 128-bit ID
            {
                for(int BYTE = 18, dst_max = pos + 16; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }

            public void hw_unique_id_SET(byte[] src, int pos)  //Hardware unique 128-bit ID
            {
                for(int BYTE =  18, src_max = pos + 16; pos < src_max; pos++, BYTE += 1)
                    BitUtils.set_bytes((ulong)(src[pos]), 1, data,  BYTE);
            }


            public byte sw_version_major //Software major version number
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  34, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  34);}
            }

            public byte sw_version_minor //Software minor version number
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  35, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  35);}
            }
            public string name_TRY(Inside ph)//Node name string. For example, "sapog.px4.io"
            {
                if(ph.field_bit !=  288 && !try_visit_field(ph, 288)  ||  !try_visit_item(ph, 0)) return null;
                return new string(name_GET(ph, new char[ph.items], 0));
            }
            public char[]name_GET(Inside ph, char[] dst_ch, int pos) //Node name string. For example, "sapog.px4.io"
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int name_LEN(Inside ph)
            {
                return (ph.field_bit !=  288 && !try_visit_field(ph, 288)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            } public void name_SET(string src, Inside ph) //Node name string. For example, "sapog.px4.io"
            {name_SET(src.ToCharArray(), 0, src.Length, ph);} public void name_SET(char[] src, int pos, int items, Inside ph) //Node name string. For example, "sapog.px4.io"
            {
                if(ph.field_bit != 288 && insert_field(ph, 288, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            } static readonly Meta meta311 = new Meta(311, 0, 2, 1, 37, 288, 0, _Iz);
        } public class PARAM_EXT_REQUEST_READ : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal PARAM_EXT_REQUEST_READ() : base(meta320, 0) { }
            internal PARAM_EXT_REQUEST_READ(int bytes) : base(meta320, bytes) { }
            public byte target_system //System I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            public short param_index //Parameter index. Set to -1 to use the Parameter ID field as identifier (else param_id will be ignore
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  2, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  2);}
            }
            public string param_id_TRY(Inside ph)
            {
                if(ph.field_bit !=  32 && !try_visit_field(ph, 32)  ||  !try_visit_item(ph, 0)) return null;
                return new string(param_id_GET(ph, new char[ph.items], 0));
            }
            public char[]param_id_GET(Inside ph, char[] dst_ch, int pos)
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int param_id_LEN(Inside ph)
            {
                return (ph.field_bit !=  32 && !try_visit_field(ph, 32)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            } public void param_id_SET(string src, Inside ph) {param_id_SET(src.ToCharArray(), 0, src.Length, ph);} public void param_id_SET(char[] src, int pos, int items, Inside ph)
            {
                if(ph.field_bit != 32 && insert_field(ph, 32, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            } static readonly Meta meta320 = new Meta(320, 0, 0, 0, 5, 32, 0, _Cz);
        } public class PARAM_EXT_REQUEST_LIST : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal PARAM_EXT_REQUEST_LIST() : base(meta321, 0) { }
            internal PARAM_EXT_REQUEST_LIST(int bytes) : base(meta321, bytes) { }
            public byte target_system //System I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }
            static readonly Meta meta321 = new Meta(321, 0, 0, 0, 2, 16);
        } public class PARAM_EXT_VALUE : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal PARAM_EXT_VALUE() : base(meta322, 0) { }
            internal PARAM_EXT_VALUE(int bytes) : base(meta322, bytes) { }
            public ushort param_count //Total number of parameter
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort param_index //Index of this paramete
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public MAV_PARAM_EXT_TYPE param_type //Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types
            {
                get {  return (MAV_PARAM_EXT_TYPE)(1 +  BitUtils.get_bits(data, 32, 4));}
                set {  BitUtils.set_bits((ulong)(- 1 +   value), 4, data, 32);}
            }
            public string param_id_TRY(Inside ph)
            {
                if(ph.field_bit !=  38 && !try_visit_field(ph, 38)  ||  !try_visit_item(ph, 0)) return null;
                return new string(param_id_GET(ph, new char[ph.items], 0));
            }
            public char[]param_id_GET(Inside ph, char[] dst_ch, int pos)
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int param_id_LEN(Inside ph)
            {
                return (ph.field_bit !=  38 && !try_visit_field(ph, 38)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            } public void param_id_SET(string src, Inside ph) {param_id_SET(src.ToCharArray(), 0, src.Length, ph);} public void param_id_SET(char[] src, int pos, int items, Inside ph)
            {
                if(ph.field_bit != 38 && insert_field(ph, 38, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            } public string param_value_TRY(Inside ph)//Parameter valu
            {
                if(ph.field_bit !=  39 && !try_visit_field(ph, 39)  ||  !try_visit_item(ph, 0)) return null;
                return new string(param_value_GET(ph, new char[ph.items], 0));
            }
            public char[]param_value_GET(Inside ph, char[] dst_ch, int pos) //Parameter valu
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int param_value_LEN(Inside ph)
            {
                return (ph.field_bit !=  39 && !try_visit_field(ph, 39)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            } public void param_value_SET(string src, Inside ph) //Parameter valu
            {param_value_SET(src.ToCharArray(), 0, src.Length, ph);} public void param_value_SET(char[] src, int pos, int items, Inside ph) //Parameter valu
            {
                if(ph.field_bit != 39 && insert_field(ph, 39, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            } static readonly Meta meta322 = new Meta(322, 2, 0, 0, 5, 38, 2, _zz, _Vz);
        } public class PARAM_EXT_SET : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal PARAM_EXT_SET() : base(meta323, 0) { }
            internal PARAM_EXT_SET(int bytes) : base(meta323, bytes) { }
            public byte target_system //System I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  0);}
            }

            public byte target_component //Component I
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  1);}
            }

            public MAV_PARAM_EXT_TYPE param_type //Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types
            {
                get {  return (MAV_PARAM_EXT_TYPE)(1 +  BitUtils.get_bits(data, 16, 4));}
                set {  BitUtils.set_bits((ulong)(- 1 +   value), 4, data, 16);}
            }
            public string param_id_TRY(Inside ph)
            {
                if(ph.field_bit !=  22 && !try_visit_field(ph, 22)  ||  !try_visit_item(ph, 0)) return null;
                return new string(param_id_GET(ph, new char[ph.items], 0));
            }
            public char[]param_id_GET(Inside ph, char[] dst_ch, int pos)
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int param_id_LEN(Inside ph)
            {
                return (ph.field_bit !=  22 && !try_visit_field(ph, 22)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            } public void param_id_SET(string src, Inside ph) {param_id_SET(src.ToCharArray(), 0, src.Length, ph);} public void param_id_SET(char[] src, int pos, int items, Inside ph)
            {
                if(ph.field_bit != 22 && insert_field(ph, 22, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            } public string param_value_TRY(Inside ph)//Parameter valu
            {
                if(ph.field_bit !=  23 && !try_visit_field(ph, 23)  ||  !try_visit_item(ph, 0)) return null;
                return new string(param_value_GET(ph, new char[ph.items], 0));
            }
            public char[]param_value_GET(Inside ph, char[] dst_ch, int pos) //Parameter valu
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int param_value_LEN(Inside ph)
            {
                return (ph.field_bit !=  23 && !try_visit_field(ph, 23)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            } public void param_value_SET(string src, Inside ph) //Parameter valu
            {param_value_SET(src.ToCharArray(), 0, src.Length, ph);} public void param_value_SET(char[] src, int pos, int items, Inside ph) //Parameter valu
            {
                if(ph.field_bit != 23 && insert_field(ph, 23, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            } static readonly Meta meta323 = new Meta(323, 0, 0, 0, 3, 22, 2, _Zz, _Rz);
        } public class PARAM_EXT_ACK : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal PARAM_EXT_ACK() : base(meta324, 0) { }
            internal PARAM_EXT_ACK(int bytes) : base(meta324, bytes) { }
            public MAV_PARAM_EXT_TYPE param_type //Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types
            {
                get {  return (MAV_PARAM_EXT_TYPE)(1 +  BitUtils.get_bits(data, 0, 4));}
                set {  BitUtils.set_bits((ulong)(- 1 +   value), 4, data, 0);}
            }

            public PARAM_ACK param_result //Result code: see the PARAM_ACK enum for possible codes
            {
                get {  return (PARAM_ACK)(0 +  BitUtils.get_bits(data, 4, 3));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 3, data, 4);}
            }
            public string param_id_TRY(Inside ph)
            {
                if(ph.field_bit !=  9 && !try_visit_field(ph, 9)  ||  !try_visit_item(ph, 0)) return null;
                return new string(param_id_GET(ph, new char[ph.items], 0));
            }
            public char[]param_id_GET(Inside ph, char[] dst_ch, int pos)
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int param_id_LEN(Inside ph)
            {
                return (ph.field_bit !=  9 && !try_visit_field(ph, 9)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            } public void param_id_SET(string src, Inside ph) {param_id_SET(src.ToCharArray(), 0, src.Length, ph);} public void param_id_SET(char[] src, int pos, int items, Inside ph)
            {
                if(ph.field_bit != 9 && insert_field(ph, 9, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            } public string param_value_TRY(Inside ph)//Parameter value (new value if PARAM_ACK_ACCEPTED, current value otherwise
            {
                if(ph.field_bit !=  10 && !try_visit_field(ph, 10)  ||  !try_visit_item(ph, 0)) return null;
                return new string(param_value_GET(ph, new char[ph.items], 0));
            }
            public char[]param_value_GET(Inside ph, char[] dst_ch, int pos) //Parameter value (new value if PARAM_ACK_ACCEPTED, current value otherwise
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int param_value_LEN(Inside ph)
            {
                return (ph.field_bit !=  10 && !try_visit_field(ph, 10)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            } public void param_value_SET(string src, Inside ph) //Parameter value (new value if PARAM_ACK_ACCEPTED, current value otherwise
            {param_value_SET(src.ToCharArray(), 0, src.Length, ph);} public void param_value_SET(char[] src, int pos, int items, Inside ph) //Parameter value (new value if PARAM_ACK_ACCEPTED, current value otherwise
            {
                if(ph.field_bit != 10 && insert_field(ph, 10, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            } static readonly Meta meta324 = new Meta(324, 0, 0, 0, 2, 9, 2, _oz, _lz);
        } public class OBSTACLE_DISTANCE : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal OBSTACLE_DISTANCE() : base(meta330, 0) { }
            internal OBSTACLE_DISTANCE(int bytes) : base(meta330, bytes) { }
            public ushort[] distances
            {
                get {return distances_GET(new ushort[72], 0);}
                set {distances_SET(value, 0)  ;}
            }
            public ushort[]distances_GET(ushort[] dst_ch, int pos)
            {
                for(int BYTE = 0, dst_max = pos + 72; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (ushort)((ushort) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }

            public void distances_SET(ushort[] src, int pos)
            {
                for(int BYTE =  0, src_max = pos + 72; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ulong)(src[pos]), 2, data,  BYTE);
            }


            public ushort min_distance //Minimum distance the sensor can measure in centimeter
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  144, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  144);}
            }

            public ushort max_distance //Maximum distance the sensor can measure in centimeter
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  146, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  146);}
            }

            public ulong time_usec //Timestamp (microseconds since system boot or since UNIX epoch
            {
                get {  return (BitUtils.get_bytes(data,  148, 8));}
                set {  BitUtils.set_bytes((ulong)(value), 8, data,  148);}
            }

            public byte increment //Angular width in degrees of each array element
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  156, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  156);}
            }

            public MAV_DISTANCE_SENSOR sensor_type //Class id of the distance sensor type
            {
                get {  return (MAV_DISTANCE_SENSOR)(0 +  BitUtils.get_bits(data, 1256, 3));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 3, data, 1256);}
            }
            static readonly Meta meta330 = new Meta(330, 74, 0, 1, 158, 1259);
        } public class UAVIONIX_ADSB_OUT_CFG : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal UAVIONIX_ADSB_OUT_CFG() : base(meta10001, 0) { }
            internal UAVIONIX_ADSB_OUT_CFG(int bytes) : base(meta10001, bytes) { }
            public ushort stallSpeed //Aircraft stall speed in cm/
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public uint ICAO //Vehicle address (24 bit
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  2, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  2);}
            }

            public ADSB_EMITTER_TYPE emitterType //Transmitting vehicle type. See ADSB_EMITTER_TYPE enu
            {
                get {  return (ADSB_EMITTER_TYPE)(0 +  BitUtils.get_bits(data, 48, 5));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 5, data, 48);}
            }

            public UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE aircraftSize //Aircraft length and width encoding (table 2-35 of DO-282B
            {
                get {  return (UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE)(0 +  BitUtils.get_bits(data, 53, 5));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 5, data, 53);}
            }

            public UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT gpsOffsetLat //GPS antenna lateral offset (table 2-36 of DO-282B
            {
                get {  return (UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT)(0 +  BitUtils.get_bits(data, 58, 4));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 58);}
            }

            public UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON gpsOffsetLon
            {
                get {  return (UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON)(0 +  BitUtils.get_bits(data, 62, 2));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 2, data, 62);}
            }

            public UAVIONIX_ADSB_OUT_RF_SELECT rfSelect //ADS-B transponder reciever and transmit enable flag
            {
                get {  return (UAVIONIX_ADSB_OUT_RF_SELECT)(0 +  BitUtils.get_bits(data, 64, 2));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 2, data, 64);}
            }
            public string callsign_TRY(Inside ph)//Vehicle identifier (8 characters, null terminated, valid characters are A-Z, 0-9, " " only
            {
                if(ph.field_bit !=  66 && !try_visit_field(ph, 66)  ||  !try_visit_item(ph, 0)) return null;
                return new string(callsign_GET(ph, new char[ph.items], 0));
            }
            public char[]callsign_GET(Inside ph, char[] dst_ch, int pos) //Vehicle identifier (8 characters, null terminated, valid characters are A-Z, 0-9, " " only
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int callsign_LEN(Inside ph)
            {
                return (ph.field_bit !=  66 && !try_visit_field(ph, 66)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            } public void callsign_SET(string src, Inside ph) //Vehicle identifier (8 characters, null terminated, valid characters are A-Z, 0-9, " " only
            {callsign_SET(src.ToCharArray(), 0, src.Length, ph);} public void callsign_SET(char[] src, int pos, int items, Inside ph) //Vehicle identifier (8 characters, null terminated, valid characters are A-Z, 0-9, " " only
            {
                if(ph.field_bit != 66 && insert_field(ph, 66, items) ||
                        ! try_visit_item(ph, 0)) insert_item(ph, 0, items);
                for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                    BitUtils.set_bytes((ushort)(src[pos]), 2, data,  BYTE);
            } static readonly Meta meta10001 = new Meta(10001, 1, 1, 0, 10, 66, 0, _ez);
        } public class UAVIONIX_ADSB_OUT_DYNAMIC : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal UAVIONIX_ADSB_OUT_DYNAMIC() : base(meta10002, 0) { }
            internal UAVIONIX_ADSB_OUT_DYNAMIC(int bytes) : base(meta10002, bytes) { }
            public ushort accuracyVert //Vertical accuracy in cm. If unknown set to UINT16_MA
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  0);}
            }

            public ushort accuracyVel //Velocity accuracy in mm/s (m * 1E-3). If unknown set to UINT16_MA
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  2);}
            }

            public ushort squawk //Mode A code (typically 1200 [0x04B0] for VFR
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
                set {  BitUtils.set_bytes((ulong)(value), 2, data,  4);}
            }

            public uint utcTime //UTC time in seconds since GPS epoch (Jan 6, 1980). If unknown set to UINT32_MA
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  6, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  6);}
            }

            public uint accuracyHor //Horizontal accuracy in mm (m * 1E-3). If unknown set to UINT32_MA
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  10, 4));}
                set {  BitUtils.set_bytes((ulong)(value), 4, data,  10);}
            }

            public int gpsLat //Latitude WGS84 (deg * 1E7). If unknown set to INT32_MA
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  14, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  14);}
            }

            public int gpsLon //Longitude WGS84 (deg * 1E7). If unknown set to INT32_MA
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  18, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  18);}
            }

            public int gpsAlt //Altitude in mm (m * 1E-3) UP +ve. WGS84 altitude. If unknown set to INT32_MA
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  22, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  22);}
            }

            public byte numSats //Number of satellites visible. If unknown set to UINT8_MA
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  26, 1));}
                set {  BitUtils.set_bytes((ulong)(value), 1, data,  26);}
            }

            public int baroAltMSL
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  27, 4));}
                set {  BitUtils.set_bytes((uint)(value), 4, data,  27);}
            }

            public short velVert //GPS vertical speed in cm/s. If unknown set to INT16_MA
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  31, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  31);}
            }

            public short velNS //North-South velocity over ground in cm/s North +ve. If unknown set to INT16_MA
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  33, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  33);}
            }

            public short VelEW //East-West velocity over ground in cm/s East +ve. If unknown set to INT16_MA
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  35, 2));}
                set {  BitUtils.set_bytes((ushort)(value), 2, data,  35);}
            }

            public UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX gpsFix //0-1: no fix, 2: 2D fix, 3: 3D fix, 4: DGPS, 5: RT
            {
                get {  return (UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX)(0 +  BitUtils.get_bits(data, 296, 3));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 3, data, 296);}
            }

            public UAVIONIX_ADSB_EMERGENCY_STATUS emergencyStatus //Emergency statu
            {
                get {  return (UAVIONIX_ADSB_EMERGENCY_STATUS)(0 +  BitUtils.get_bits(data, 299, 4));}
                set {  BitUtils.set_bits((ulong)(- 0 +   value), 4, data, 299);}
            }

            public UAVIONIX_ADSB_OUT_DYNAMIC_STATE state //ADS-B transponder dynamic input state flag
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
                set
                {
                    ulong id = 0;
                    switch(value)
                    {
                        case UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_INTENT_CHANGE:
                            id = 0;
                            break;
                        case UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_AUTOPILOT_ENABLED:
                            id = 1;
                            break;
                        case UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_NICBARO_CROSSCHECKED:
                            id = 2;
                            break;
                        case UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_ON_GROUND:
                            id = 3;
                            break;
                        case UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_IDENT:
                            id = 4;
                            break;
                        default:
                            throw  new ArgumentException("Unknown enum " + value);
                    }
                    BitUtils.set_bits(id, 3, data, 303);
                }
            }
            static readonly Meta meta10002 = new Meta(10002, 3, 2, 0, 39, 306);
        } public class UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT : Pack, LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
        {
            internal UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT() : base(meta10003, 0) { }
            internal UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT(int bytes) : base(meta10003, bytes) { }
            public UAVIONIX_ADSB_RF_HEALTH rfHealth //ADS-B transponder message
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
                set
                {
                    ulong id = 0;
                    switch(value)
                    {
                        case UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_INITIALIZING:
                            id = 0;
                            break;
                        case UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_OK:
                            id = 1;
                            break;
                        case UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_FAIL_TX:
                            id = 2;
                            break;
                        case UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_FAIL_RX:
                            id = 3;
                            break;
                        default:
                            throw  new ArgumentException("Unknown enum " + value);
                    }
                    BitUtils.set_bits(id, 3, data, 0);
                }
            }
            static readonly Meta meta10003 = new Meta(10003, 0, 0, 0, 1, 3);
        }

        public class LoopBackDemoChannel : Channel
        {
            public override bool CanRead { get { return true ; } }
            public override bool CanWrite { get { return true; } }
            static LoopBackDemoChannel() {pack_id_bytes = 2;}

            public static  LoopBackDemoChannel instance = new LoopBackDemoChannel();

            //interface-to-mark of sendable through this channel packs_Schs_Rchs
            public interface Sendable {}
            public static HEARTBEAT new_HEARTBEAT() {return new  HEARTBEAT();}
            public static SYS_STATUS new_SYS_STATUS() {return new  SYS_STATUS();}
            public static SYSTEM_TIME new_SYSTEM_TIME() {return new  SYSTEM_TIME();}
            public static POSITION_TARGET_LOCAL_NED new_POSITION_TARGET_LOCAL_NED() {return new  POSITION_TARGET_LOCAL_NED();}
            public static PING new_PING() {return new  PING();}
            public static CHANGE_OPERATOR_CONTROL new_CHANGE_OPERATOR_CONTROL() {return new  CHANGE_OPERATOR_CONTROL();}
            public static CHANGE_OPERATOR_CONTROL_ACK new_CHANGE_OPERATOR_CONTROL_ACK() {return new  CHANGE_OPERATOR_CONTROL_ACK();}
            public static AUTH_KEY new_AUTH_KEY() {return new  AUTH_KEY();}
            public static SET_MODE new_SET_MODE() {return new  SET_MODE();}
            public static PARAM_REQUEST_READ new_PARAM_REQUEST_READ() {return new  PARAM_REQUEST_READ();}
            public static PARAM_REQUEST_LIST new_PARAM_REQUEST_LIST() {return new  PARAM_REQUEST_LIST();}
            public static PARAM_VALUE new_PARAM_VALUE() {return new  PARAM_VALUE();}
            public static PARAM_SET new_PARAM_SET() {return new  PARAM_SET();}
            public static GPS_RAW_INT new_GPS_RAW_INT() {return new  GPS_RAW_INT();}
            public static GPS_STATUS new_GPS_STATUS() {return new  GPS_STATUS();}
            public static SCALED_IMU new_SCALED_IMU() {return new  SCALED_IMU();}
            public static RAW_IMU new_RAW_IMU() {return new  RAW_IMU();}
            public static RAW_PRESSURE new_RAW_PRESSURE() {return new  RAW_PRESSURE();}
            public static SCALED_PRESSURE new_SCALED_PRESSURE() {return new  SCALED_PRESSURE();}
            public static ATTITUDE new_ATTITUDE() {return new  ATTITUDE();}
            public static ATTITUDE_QUATERNION new_ATTITUDE_QUATERNION() {return new  ATTITUDE_QUATERNION();}
            public static LOCAL_POSITION_NED new_LOCAL_POSITION_NED() {return new  LOCAL_POSITION_NED();}
            public static GLOBAL_POSITION_INT new_GLOBAL_POSITION_INT() {return new  GLOBAL_POSITION_INT();}
            public static RC_CHANNELS_SCALED new_RC_CHANNELS_SCALED() {return new  RC_CHANNELS_SCALED();}
            public static RC_CHANNELS_RAW new_RC_CHANNELS_RAW() {return new  RC_CHANNELS_RAW();}
            public static SERVO_OUTPUT_RAW new_SERVO_OUTPUT_RAW() {return new  SERVO_OUTPUT_RAW();}
            public static MISSION_REQUEST_PARTIAL_LIST new_MISSION_REQUEST_PARTIAL_LIST() {return new  MISSION_REQUEST_PARTIAL_LIST();}
            public static MISSION_WRITE_PARTIAL_LIST new_MISSION_WRITE_PARTIAL_LIST() {return new  MISSION_WRITE_PARTIAL_LIST();}
            public static MISSION_ITEM new_MISSION_ITEM() {return new  MISSION_ITEM();}
            public static MISSION_REQUEST new_MISSION_REQUEST() {return new  MISSION_REQUEST();}
            public static MISSION_SET_CURRENT new_MISSION_SET_CURRENT() {return new  MISSION_SET_CURRENT();}
            public static MISSION_CURRENT new_MISSION_CURRENT() {return new  MISSION_CURRENT();}
            public static MISSION_REQUEST_LIST new_MISSION_REQUEST_LIST() {return new  MISSION_REQUEST_LIST();}
            public static MISSION_COUNT new_MISSION_COUNT() {return new  MISSION_COUNT();}
            public static MISSION_CLEAR_ALL new_MISSION_CLEAR_ALL() {return new  MISSION_CLEAR_ALL();}
            public static MISSION_ITEM_REACHED new_MISSION_ITEM_REACHED() {return new  MISSION_ITEM_REACHED();}
            public static MISSION_ACK new_MISSION_ACK() {return new  MISSION_ACK();}
            public static SET_GPS_GLOBAL_ORIGIN new_SET_GPS_GLOBAL_ORIGIN() {return new  SET_GPS_GLOBAL_ORIGIN();}
            public static GPS_GLOBAL_ORIGIN new_GPS_GLOBAL_ORIGIN() {return new  GPS_GLOBAL_ORIGIN();}
            public static PARAM_MAP_RC new_PARAM_MAP_RC() {return new  PARAM_MAP_RC();}
            public static MISSION_REQUEST_INT new_MISSION_REQUEST_INT() {return new  MISSION_REQUEST_INT();}
            public static SAFETY_SET_ALLOWED_AREA new_SAFETY_SET_ALLOWED_AREA() {return new  SAFETY_SET_ALLOWED_AREA();}
            public static SAFETY_ALLOWED_AREA new_SAFETY_ALLOWED_AREA() {return new  SAFETY_ALLOWED_AREA();}
            public static ATTITUDE_QUATERNION_COV new_ATTITUDE_QUATERNION_COV() {return new  ATTITUDE_QUATERNION_COV();}
            public static NAV_CONTROLLER_OUTPUT new_NAV_CONTROLLER_OUTPUT() {return new  NAV_CONTROLLER_OUTPUT();}
            public static GLOBAL_POSITION_INT_COV new_GLOBAL_POSITION_INT_COV() {return new  GLOBAL_POSITION_INT_COV();}
            public static LOCAL_POSITION_NED_COV new_LOCAL_POSITION_NED_COV() {return new  LOCAL_POSITION_NED_COV();}
            public static RC_CHANNELS new_RC_CHANNELS() {return new  RC_CHANNELS();}
            public static REQUEST_DATA_STREAM new_REQUEST_DATA_STREAM() {return new  REQUEST_DATA_STREAM();}
            public static DATA_STREAM new_DATA_STREAM() {return new  DATA_STREAM();}
            public static MANUAL_CONTROL new_MANUAL_CONTROL() {return new  MANUAL_CONTROL();}
            public static RC_CHANNELS_OVERRIDE new_RC_CHANNELS_OVERRIDE() {return new  RC_CHANNELS_OVERRIDE();}
            public static MISSION_ITEM_INT new_MISSION_ITEM_INT() {return new  MISSION_ITEM_INT();}
            public static VFR_HUD new_VFR_HUD() {return new  VFR_HUD();}
            public static COMMAND_INT new_COMMAND_INT() {return new  COMMAND_INT();}
            public static COMMAND_LONG new_COMMAND_LONG() {return new  COMMAND_LONG();}
            public static COMMAND_ACK new_COMMAND_ACK() {return new  COMMAND_ACK();}
            public static MANUAL_SETPOINT new_MANUAL_SETPOINT() {return new  MANUAL_SETPOINT();}
            public static SET_ATTITUDE_TARGET new_SET_ATTITUDE_TARGET() {return new  SET_ATTITUDE_TARGET();}
            public static ATTITUDE_TARGET new_ATTITUDE_TARGET() {return new  ATTITUDE_TARGET();}
            public static SET_POSITION_TARGET_LOCAL_NED new_SET_POSITION_TARGET_LOCAL_NED() {return new  SET_POSITION_TARGET_LOCAL_NED();}
            public static SET_POSITION_TARGET_GLOBAL_INT new_SET_POSITION_TARGET_GLOBAL_INT() {return new  SET_POSITION_TARGET_GLOBAL_INT();}
            public static POSITION_TARGET_GLOBAL_INT new_POSITION_TARGET_GLOBAL_INT() {return new  POSITION_TARGET_GLOBAL_INT();}
            public static LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET() {return new  LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();}
            public static HIL_STATE new_HIL_STATE() {return new  HIL_STATE();}
            public static HIL_CONTROLS new_HIL_CONTROLS() {return new  HIL_CONTROLS();}
            public static HIL_RC_INPUTS_RAW new_HIL_RC_INPUTS_RAW() {return new  HIL_RC_INPUTS_RAW();}
            public static HIL_ACTUATOR_CONTROLS new_HIL_ACTUATOR_CONTROLS() {return new  HIL_ACTUATOR_CONTROLS();}
            public static OPTICAL_FLOW new_OPTICAL_FLOW() {return new  OPTICAL_FLOW();}
            public static GLOBAL_VISION_POSITION_ESTIMATE new_GLOBAL_VISION_POSITION_ESTIMATE() {return new  GLOBAL_VISION_POSITION_ESTIMATE();}
            public static VISION_POSITION_ESTIMATE new_VISION_POSITION_ESTIMATE() {return new  VISION_POSITION_ESTIMATE();}
            public static VISION_SPEED_ESTIMATE new_VISION_SPEED_ESTIMATE() {return new  VISION_SPEED_ESTIMATE();}
            public static VICON_POSITION_ESTIMATE new_VICON_POSITION_ESTIMATE() {return new  VICON_POSITION_ESTIMATE();}
            public static HIGHRES_IMU new_HIGHRES_IMU() {return new  HIGHRES_IMU();}
            public static OPTICAL_FLOW_RAD new_OPTICAL_FLOW_RAD() {return new  OPTICAL_FLOW_RAD();}
            public static HIL_SENSOR new_HIL_SENSOR() {return new  HIL_SENSOR();}
            public static SIM_STATE new_SIM_STATE() {return new  SIM_STATE();}
            public static RADIO_STATUS new_RADIO_STATUS() {return new  RADIO_STATUS();}
            public static FILE_TRANSFER_PROTOCOL new_FILE_TRANSFER_PROTOCOL() {return new  FILE_TRANSFER_PROTOCOL();}
            public static TIMESYNC new_TIMESYNC() {return new  TIMESYNC();}
            public static CAMERA_TRIGGER new_CAMERA_TRIGGER() {return new  CAMERA_TRIGGER();}
            public static HIL_GPS new_HIL_GPS() {return new  HIL_GPS();}
            public static HIL_OPTICAL_FLOW new_HIL_OPTICAL_FLOW() {return new  HIL_OPTICAL_FLOW();}
            public static HIL_STATE_QUATERNION new_HIL_STATE_QUATERNION() {return new  HIL_STATE_QUATERNION();}
            public static SCALED_IMU2 new_SCALED_IMU2() {return new  SCALED_IMU2();}
            public static LOG_REQUEST_LIST new_LOG_REQUEST_LIST() {return new  LOG_REQUEST_LIST();}
            public static LOG_ENTRY new_LOG_ENTRY() {return new  LOG_ENTRY();}
            public static LOG_REQUEST_DATA new_LOG_REQUEST_DATA() {return new  LOG_REQUEST_DATA();}
            public static LOG_DATA new_LOG_DATA() {return new  LOG_DATA();}
            public static LOG_ERASE new_LOG_ERASE() {return new  LOG_ERASE();}
            public static LOG_REQUEST_END new_LOG_REQUEST_END() {return new  LOG_REQUEST_END();}
            public static GPS_INJECT_DATA new_GPS_INJECT_DATA() {return new  GPS_INJECT_DATA();}
            public static GPS2_RAW new_GPS2_RAW() {return new  GPS2_RAW();}
            public static POWER_STATUS new_POWER_STATUS() {return new  POWER_STATUS();}
            public static SERIAL_CONTROL new_SERIAL_CONTROL() {return new  SERIAL_CONTROL();}
            public static GPS_RTK new_GPS_RTK() {return new  GPS_RTK();}
            public static GPS2_RTK new_GPS2_RTK() {return new  GPS2_RTK();}
            public static SCALED_IMU3 new_SCALED_IMU3() {return new  SCALED_IMU3();}
            public static DATA_TRANSMISSION_HANDSHAKE new_DATA_TRANSMISSION_HANDSHAKE() {return new  DATA_TRANSMISSION_HANDSHAKE();}
            public static ENCAPSULATED_DATA new_ENCAPSULATED_DATA() {return new  ENCAPSULATED_DATA();}
            public static DISTANCE_SENSOR new_DISTANCE_SENSOR() {return new  DISTANCE_SENSOR();}
            public static TERRAIN_REQUEST new_TERRAIN_REQUEST() {return new  TERRAIN_REQUEST();}
            public static TERRAIN_DATA new_TERRAIN_DATA() {return new  TERRAIN_DATA();}
            public static TERRAIN_CHECK new_TERRAIN_CHECK() {return new  TERRAIN_CHECK();}
            public static TERRAIN_REPORT new_TERRAIN_REPORT() {return new  TERRAIN_REPORT();}
            public static SCALED_PRESSURE2 new_SCALED_PRESSURE2() {return new  SCALED_PRESSURE2();}
            public static ATT_POS_MOCAP new_ATT_POS_MOCAP() {return new  ATT_POS_MOCAP();}
            public static SET_ACTUATOR_CONTROL_TARGET new_SET_ACTUATOR_CONTROL_TARGET() {return new  SET_ACTUATOR_CONTROL_TARGET();}
            public static ACTUATOR_CONTROL_TARGET new_ACTUATOR_CONTROL_TARGET() {return new  ACTUATOR_CONTROL_TARGET();}
            public static ALTITUDE new_ALTITUDE() {return new  ALTITUDE();}
            public static RESOURCE_REQUEST new_RESOURCE_REQUEST() {return new  RESOURCE_REQUEST();}
            public static SCALED_PRESSURE3 new_SCALED_PRESSURE3() {return new  SCALED_PRESSURE3();}
            public static FOLLOW_TARGET new_FOLLOW_TARGET() {return new  FOLLOW_TARGET();}
            public static CONTROL_SYSTEM_STATE new_CONTROL_SYSTEM_STATE() {return new  CONTROL_SYSTEM_STATE();}
            public static BATTERY_STATUS new_BATTERY_STATUS() {return new  BATTERY_STATUS();}
            public static AUTOPILOT_VERSION new_AUTOPILOT_VERSION() {return new  AUTOPILOT_VERSION();}
            public static LANDING_TARGET new_LANDING_TARGET() {return new  LANDING_TARGET();}
            public static ESTIMATOR_STATUS new_ESTIMATOR_STATUS() {return new  ESTIMATOR_STATUS();}
            public static WIND_COV new_WIND_COV() {return new  WIND_COV();}
            public static GPS_INPUT new_GPS_INPUT() {return new  GPS_INPUT();}
            public static GPS_RTCM_DATA new_GPS_RTCM_DATA() {return new  GPS_RTCM_DATA();}
            public static HIGH_LATENCY new_HIGH_LATENCY() {return new  HIGH_LATENCY();}
            public static VIBRATION new_VIBRATION() {return new  VIBRATION();}
            public static HOME_POSITION new_HOME_POSITION() {return new  HOME_POSITION();}
            public static SET_HOME_POSITION new_SET_HOME_POSITION() {return new  SET_HOME_POSITION();}
            public static MESSAGE_INTERVAL new_MESSAGE_INTERVAL() {return new  MESSAGE_INTERVAL();}
            public static EXTENDED_SYS_STATE new_EXTENDED_SYS_STATE() {return new  EXTENDED_SYS_STATE();}
            public static ADSB_VEHICLE new_ADSB_VEHICLE() {return new  ADSB_VEHICLE();}
            public static COLLISION new_COLLISION() {return new  COLLISION();}
            public static V2_EXTENSION new_V2_EXTENSION() {return new  V2_EXTENSION();}
            public static MEMORY_VECT new_MEMORY_VECT() {return new  MEMORY_VECT();}
            public static DEBUG_VECT new_DEBUG_VECT() {return new  DEBUG_VECT();}
            public static NAMED_VALUE_FLOAT new_NAMED_VALUE_FLOAT() {return new  NAMED_VALUE_FLOAT();}
            public static NAMED_VALUE_INT new_NAMED_VALUE_INT() {return new  NAMED_VALUE_INT();}
            public static STATUSTEXT new_STATUSTEXT() {return new  STATUSTEXT();}
            public static DEBUG new_DEBUG() {return new  DEBUG();}
            public static SETUP_SIGNING new_SETUP_SIGNING() {return new  SETUP_SIGNING();}
            public static BUTTON_CHANGE new_BUTTON_CHANGE() {return new  BUTTON_CHANGE();}
            public static PLAY_TUNE new_PLAY_TUNE() {return new  PLAY_TUNE();}
            public static CAMERA_INFORMATION new_CAMERA_INFORMATION() {return new  CAMERA_INFORMATION();}
            public static CAMERA_SETTINGS new_CAMERA_SETTINGS() {return new  CAMERA_SETTINGS();}
            public static STORAGE_INFORMATION new_STORAGE_INFORMATION() {return new  STORAGE_INFORMATION();}
            public static CAMERA_CAPTURE_STATUS new_CAMERA_CAPTURE_STATUS() {return new  CAMERA_CAPTURE_STATUS();}
            public static CAMERA_IMAGE_CAPTURED new_CAMERA_IMAGE_CAPTURED() {return new  CAMERA_IMAGE_CAPTURED();}
            public static FLIGHT_INFORMATION new_FLIGHT_INFORMATION() {return new  FLIGHT_INFORMATION();}
            public static MOUNT_ORIENTATION new_MOUNT_ORIENTATION() {return new  MOUNT_ORIENTATION();}
            public static LOGGING_DATA new_LOGGING_DATA() {return new  LOGGING_DATA();}
            public static LOGGING_DATA_ACKED new_LOGGING_DATA_ACKED() {return new  LOGGING_DATA_ACKED();}
            public static LOGGING_ACK new_LOGGING_ACK() {return new  LOGGING_ACK();}
            public static VIDEO_STREAM_INFORMATION new_VIDEO_STREAM_INFORMATION() {return new  VIDEO_STREAM_INFORMATION();}
            public static SET_VIDEO_STREAM_SETTINGS new_SET_VIDEO_STREAM_SETTINGS() {return new  SET_VIDEO_STREAM_SETTINGS();}
            public static WIFI_CONFIG_AP new_WIFI_CONFIG_AP() {return new  WIFI_CONFIG_AP();}
            public static PROTOCOL_VERSION new_PROTOCOL_VERSION() {return new  PROTOCOL_VERSION();}
            public static UAVCAN_NODE_STATUS new_UAVCAN_NODE_STATUS() {return new  UAVCAN_NODE_STATUS();}
            public static UAVCAN_NODE_INFO new_UAVCAN_NODE_INFO() {return new  UAVCAN_NODE_INFO();}
            public static PARAM_EXT_REQUEST_READ new_PARAM_EXT_REQUEST_READ() {return new  PARAM_EXT_REQUEST_READ();}
            public static PARAM_EXT_REQUEST_LIST new_PARAM_EXT_REQUEST_LIST() {return new  PARAM_EXT_REQUEST_LIST();}
            public static PARAM_EXT_VALUE new_PARAM_EXT_VALUE() {return new  PARAM_EXT_VALUE();}
            public static PARAM_EXT_SET new_PARAM_EXT_SET() {return new  PARAM_EXT_SET();}
            public static PARAM_EXT_ACK new_PARAM_EXT_ACK() {return new  PARAM_EXT_ACK();}
            public static OBSTACLE_DISTANCE new_OBSTACLE_DISTANCE() {return new  OBSTACLE_DISTANCE();}
            public static UAVIONIX_ADSB_OUT_CFG new_UAVIONIX_ADSB_OUT_CFG() {return new  UAVIONIX_ADSB_OUT_CFG();}
            public static UAVIONIX_ADSB_OUT_DYNAMIC new_UAVIONIX_ADSB_OUT_DYNAMIC() {return new  UAVIONIX_ADSB_OUT_DYNAMIC();}
            public static UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT new_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT() {return new  UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT();}

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
                        case 148:
                            if(pack == null) return new AUTOPILOT_VERSION(-1);
                            if(OnAUTOPILOT_VERSIONReceive == null) return null;
                            ph.setPack(pack);
                            OnAUTOPILOT_VERSIONReceive(this, ph, (AUTOPILOT_VERSION) pack);
                            if(LOOP) break;
                            return null;
                        case 149:
                            if(pack == null) return new LANDING_TARGET(-1);
                            if(OnLANDING_TARGETReceive == null) return null;
                            ph.setPack(pack);
                            OnLANDING_TARGETReceive(this, ph, (LANDING_TARGET) pack);
                            if(LOOP) break;
                            return null;
                        case 230:
                            if(pack == null) return new ESTIMATOR_STATUS();
                            if(OnESTIMATOR_STATUSReceive == null) return null;
                            ph.setPack(pack);
                            OnESTIMATOR_STATUSReceive(this, ph, (ESTIMATOR_STATUS) pack);
                            if(LOOP) break;
                            return null;
                        case 231:
                            if(pack == null) return new WIND_COV();
                            if(OnWIND_COVReceive == null) return null;
                            ph.setPack(pack);
                            OnWIND_COVReceive(this, ph, (WIND_COV) pack);
                            if(LOOP) break;
                            return null;
                        case 232:
                            if(pack == null) return new GPS_INPUT();
                            if(OnGPS_INPUTReceive == null) return null;
                            ph.setPack(pack);
                            OnGPS_INPUTReceive(this, ph, (GPS_INPUT) pack);
                            if(LOOP) break;
                            return null;
                        case 233:
                            if(pack == null) return new GPS_RTCM_DATA();
                            if(OnGPS_RTCM_DATAReceive == null) return null;
                            ph.setPack(pack);
                            OnGPS_RTCM_DATAReceive(this, ph, (GPS_RTCM_DATA) pack);
                            if(LOOP) break;
                            return null;
                        case 234:
                            if(pack == null) return new HIGH_LATENCY();
                            if(OnHIGH_LATENCYReceive == null) return null;
                            ph.setPack(pack);
                            OnHIGH_LATENCYReceive(this, ph, (HIGH_LATENCY) pack);
                            if(LOOP) break;
                            return null;
                        case 241:
                            if(pack == null) return new VIBRATION();
                            if(OnVIBRATIONReceive == null) return null;
                            ph.setPack(pack);
                            OnVIBRATIONReceive(this, ph, (VIBRATION) pack);
                            if(LOOP) break;
                            return null;
                        case 242:
                            if(pack == null) return new HOME_POSITION(-1);
                            if(OnHOME_POSITIONReceive == null) return null;
                            ph.setPack(pack);
                            OnHOME_POSITIONReceive(this, ph, (HOME_POSITION) pack);
                            if(LOOP) break;
                            return null;
                        case 243:
                            if(pack == null) return new SET_HOME_POSITION(-1);
                            if(OnSET_HOME_POSITIONReceive == null) return null;
                            ph.setPack(pack);
                            OnSET_HOME_POSITIONReceive(this, ph, (SET_HOME_POSITION) pack);
                            if(LOOP) break;
                            return null;
                        case 244:
                            if(pack == null) return new MESSAGE_INTERVAL();
                            if(OnMESSAGE_INTERVALReceive == null) return null;
                            ph.setPack(pack);
                            OnMESSAGE_INTERVALReceive(this, ph, (MESSAGE_INTERVAL) pack);
                            if(LOOP) break;
                            return null;
                        case 245:
                            if(pack == null) return new EXTENDED_SYS_STATE();
                            if(OnEXTENDED_SYS_STATEReceive == null) return null;
                            ph.setPack(pack);
                            OnEXTENDED_SYS_STATEReceive(this, ph, (EXTENDED_SYS_STATE) pack);
                            if(LOOP) break;
                            return null;
                        case 246:
                            if(pack == null) return new ADSB_VEHICLE(-1);
                            if(OnADSB_VEHICLEReceive == null) return null;
                            ph.setPack(pack);
                            OnADSB_VEHICLEReceive(this, ph, (ADSB_VEHICLE) pack);
                            if(LOOP) break;
                            return null;
                        case 247:
                            if(pack == null) return new COLLISION();
                            if(OnCOLLISIONReceive == null) return null;
                            ph.setPack(pack);
                            OnCOLLISIONReceive(this, ph, (COLLISION) pack);
                            if(LOOP) break;
                            return null;
                        case 248:
                            if(pack == null) return new V2_EXTENSION();
                            if(OnV2_EXTENSIONReceive == null) return null;
                            ph.setPack(pack);
                            OnV2_EXTENSIONReceive(this, ph, (V2_EXTENSION) pack);
                            if(LOOP) break;
                            return null;
                        case 249:
                            if(pack == null) return new MEMORY_VECT();
                            if(OnMEMORY_VECTReceive == null) return null;
                            ph.setPack(pack);
                            OnMEMORY_VECTReceive(this, ph, (MEMORY_VECT) pack);
                            if(LOOP) break;
                            return null;
                        case 250:
                            if(pack == null) return new DEBUG_VECT(-1);
                            if(OnDEBUG_VECTReceive == null) return null;
                            ph.setPack(pack);
                            OnDEBUG_VECTReceive(this, ph, (DEBUG_VECT) pack);
                            if(LOOP) break;
                            return null;
                        case 251:
                            if(pack == null) return new NAMED_VALUE_FLOAT(-1);
                            if(OnNAMED_VALUE_FLOATReceive == null) return null;
                            ph.setPack(pack);
                            OnNAMED_VALUE_FLOATReceive(this, ph, (NAMED_VALUE_FLOAT) pack);
                            if(LOOP) break;
                            return null;
                        case 252:
                            if(pack == null) return new NAMED_VALUE_INT(-1);
                            if(OnNAMED_VALUE_INTReceive == null) return null;
                            ph.setPack(pack);
                            OnNAMED_VALUE_INTReceive(this, ph, (NAMED_VALUE_INT) pack);
                            if(LOOP) break;
                            return null;
                        case 253:
                            if(pack == null) return new STATUSTEXT(-1);
                            if(OnSTATUSTEXTReceive == null) return null;
                            ph.setPack(pack);
                            OnSTATUSTEXTReceive(this, ph, (STATUSTEXT) pack);
                            if(LOOP) break;
                            return null;
                        case 254:
                            if(pack == null) return new DEBUG();
                            if(OnDEBUGReceive == null) return null;
                            ph.setPack(pack);
                            OnDEBUGReceive(this, ph, (DEBUG) pack);
                            if(LOOP) break;
                            return null;
                        case 256:
                            if(pack == null) return new SETUP_SIGNING();
                            if(OnSETUP_SIGNINGReceive == null) return null;
                            ph.setPack(pack);
                            OnSETUP_SIGNINGReceive(this, ph, (SETUP_SIGNING) pack);
                            if(LOOP) break;
                            return null;
                        case 257:
                            if(pack == null) return new BUTTON_CHANGE();
                            if(OnBUTTON_CHANGEReceive == null) return null;
                            ph.setPack(pack);
                            OnBUTTON_CHANGEReceive(this, ph, (BUTTON_CHANGE) pack);
                            if(LOOP) break;
                            return null;
                        case 258:
                            if(pack == null) return new PLAY_TUNE(-1);
                            if(OnPLAY_TUNEReceive == null) return null;
                            ph.setPack(pack);
                            OnPLAY_TUNEReceive(this, ph, (PLAY_TUNE) pack);
                            if(LOOP) break;
                            return null;
                        case 259:
                            if(pack == null) return new CAMERA_INFORMATION(-1);
                            if(OnCAMERA_INFORMATIONReceive == null) return null;
                            ph.setPack(pack);
                            OnCAMERA_INFORMATIONReceive(this, ph, (CAMERA_INFORMATION) pack);
                            if(LOOP) break;
                            return null;
                        case 260:
                            if(pack == null) return new CAMERA_SETTINGS();
                            if(OnCAMERA_SETTINGSReceive == null) return null;
                            ph.setPack(pack);
                            OnCAMERA_SETTINGSReceive(this, ph, (CAMERA_SETTINGS) pack);
                            if(LOOP) break;
                            return null;
                        case 261:
                            if(pack == null) return new STORAGE_INFORMATION();
                            if(OnSTORAGE_INFORMATIONReceive == null) return null;
                            ph.setPack(pack);
                            OnSTORAGE_INFORMATIONReceive(this, ph, (STORAGE_INFORMATION) pack);
                            if(LOOP) break;
                            return null;
                        case 262:
                            if(pack == null) return new CAMERA_CAPTURE_STATUS();
                            if(OnCAMERA_CAPTURE_STATUSReceive == null) return null;
                            ph.setPack(pack);
                            OnCAMERA_CAPTURE_STATUSReceive(this, ph, (CAMERA_CAPTURE_STATUS) pack);
                            if(LOOP) break;
                            return null;
                        case 263:
                            if(pack == null) return new CAMERA_IMAGE_CAPTURED(-1);
                            if(OnCAMERA_IMAGE_CAPTUREDReceive == null) return null;
                            ph.setPack(pack);
                            OnCAMERA_IMAGE_CAPTUREDReceive(this, ph, (CAMERA_IMAGE_CAPTURED) pack);
                            if(LOOP) break;
                            return null;
                        case 264:
                            if(pack == null) return new FLIGHT_INFORMATION();
                            if(OnFLIGHT_INFORMATIONReceive == null) return null;
                            ph.setPack(pack);
                            OnFLIGHT_INFORMATIONReceive(this, ph, (FLIGHT_INFORMATION) pack);
                            if(LOOP) break;
                            return null;
                        case 265:
                            if(pack == null) return new MOUNT_ORIENTATION();
                            if(OnMOUNT_ORIENTATIONReceive == null) return null;
                            ph.setPack(pack);
                            OnMOUNT_ORIENTATIONReceive(this, ph, (MOUNT_ORIENTATION) pack);
                            if(LOOP) break;
                            return null;
                        case 266:
                            if(pack == null) return new LOGGING_DATA();
                            if(OnLOGGING_DATAReceive == null) return null;
                            ph.setPack(pack);
                            OnLOGGING_DATAReceive(this, ph, (LOGGING_DATA) pack);
                            if(LOOP) break;
                            return null;
                        case 267:
                            if(pack == null) return new LOGGING_DATA_ACKED();
                            if(OnLOGGING_DATA_ACKEDReceive == null) return null;
                            ph.setPack(pack);
                            OnLOGGING_DATA_ACKEDReceive(this, ph, (LOGGING_DATA_ACKED) pack);
                            if(LOOP) break;
                            return null;
                        case 268:
                            if(pack == null) return new LOGGING_ACK();
                            if(OnLOGGING_ACKReceive == null) return null;
                            ph.setPack(pack);
                            OnLOGGING_ACKReceive(this, ph, (LOGGING_ACK) pack);
                            if(LOOP) break;
                            return null;
                        case 269:
                            if(pack == null) return new VIDEO_STREAM_INFORMATION(-1);
                            if(OnVIDEO_STREAM_INFORMATIONReceive == null) return null;
                            ph.setPack(pack);
                            OnVIDEO_STREAM_INFORMATIONReceive(this, ph, (VIDEO_STREAM_INFORMATION) pack);
                            if(LOOP) break;
                            return null;
                        case 270:
                            if(pack == null) return new SET_VIDEO_STREAM_SETTINGS(-1);
                            if(OnSET_VIDEO_STREAM_SETTINGSReceive == null) return null;
                            ph.setPack(pack);
                            OnSET_VIDEO_STREAM_SETTINGSReceive(this, ph, (SET_VIDEO_STREAM_SETTINGS) pack);
                            if(LOOP) break;
                            return null;
                        case 299:
                            if(pack == null) return new WIFI_CONFIG_AP(-1);
                            if(OnWIFI_CONFIG_APReceive == null) return null;
                            ph.setPack(pack);
                            OnWIFI_CONFIG_APReceive(this, ph, (WIFI_CONFIG_AP) pack);
                            if(LOOP) break;
                            return null;
                        case 300:
                            if(pack == null) return new PROTOCOL_VERSION();
                            if(OnPROTOCOL_VERSIONReceive == null) return null;
                            ph.setPack(pack);
                            OnPROTOCOL_VERSIONReceive(this, ph, (PROTOCOL_VERSION) pack);
                            if(LOOP) break;
                            return null;
                        case 310:
                            if(pack == null) return new UAVCAN_NODE_STATUS();
                            if(OnUAVCAN_NODE_STATUSReceive == null) return null;
                            ph.setPack(pack);
                            OnUAVCAN_NODE_STATUSReceive(this, ph, (UAVCAN_NODE_STATUS) pack);
                            if(LOOP) break;
                            return null;
                        case 311:
                            if(pack == null) return new UAVCAN_NODE_INFO(-1);
                            if(OnUAVCAN_NODE_INFOReceive == null) return null;
                            ph.setPack(pack);
                            OnUAVCAN_NODE_INFOReceive(this, ph, (UAVCAN_NODE_INFO) pack);
                            if(LOOP) break;
                            return null;
                        case 320:
                            if(pack == null) return new PARAM_EXT_REQUEST_READ(-1);
                            if(OnPARAM_EXT_REQUEST_READReceive == null) return null;
                            ph.setPack(pack);
                            OnPARAM_EXT_REQUEST_READReceive(this, ph, (PARAM_EXT_REQUEST_READ) pack);
                            if(LOOP) break;
                            return null;
                        case 321:
                            if(pack == null) return new PARAM_EXT_REQUEST_LIST();
                            if(OnPARAM_EXT_REQUEST_LISTReceive == null) return null;
                            ph.setPack(pack);
                            OnPARAM_EXT_REQUEST_LISTReceive(this, ph, (PARAM_EXT_REQUEST_LIST) pack);
                            if(LOOP) break;
                            return null;
                        case 322:
                            if(pack == null) return new PARAM_EXT_VALUE(-1);
                            if(OnPARAM_EXT_VALUEReceive == null) return null;
                            ph.setPack(pack);
                            OnPARAM_EXT_VALUEReceive(this, ph, (PARAM_EXT_VALUE) pack);
                            if(LOOP) break;
                            return null;
                        case 323:
                            if(pack == null) return new PARAM_EXT_SET(-1);
                            if(OnPARAM_EXT_SETReceive == null) return null;
                            ph.setPack(pack);
                            OnPARAM_EXT_SETReceive(this, ph, (PARAM_EXT_SET) pack);
                            if(LOOP) break;
                            return null;
                        case 324:
                            if(pack == null) return new PARAM_EXT_ACK(-1);
                            if(OnPARAM_EXT_ACKReceive == null) return null;
                            ph.setPack(pack);
                            OnPARAM_EXT_ACKReceive(this, ph, (PARAM_EXT_ACK) pack);
                            if(LOOP) break;
                            return null;
                        case 330:
                            if(pack == null) return new OBSTACLE_DISTANCE();
                            if(OnOBSTACLE_DISTANCEReceive == null) return null;
                            ph.setPack(pack);
                            OnOBSTACLE_DISTANCEReceive(this, ph, (OBSTACLE_DISTANCE) pack);
                            if(LOOP) break;
                            return null;
                        case 10001:
                            if(pack == null) return new UAVIONIX_ADSB_OUT_CFG(-1);
                            if(OnUAVIONIX_ADSB_OUT_CFGReceive == null) return null;
                            ph.setPack(pack);
                            OnUAVIONIX_ADSB_OUT_CFGReceive(this, ph, (UAVIONIX_ADSB_OUT_CFG) pack);
                            if(LOOP) break;
                            return null;
                        case 10002:
                            if(pack == null) return new UAVIONIX_ADSB_OUT_DYNAMIC();
                            if(OnUAVIONIX_ADSB_OUT_DYNAMICReceive == null) return null;
                            ph.setPack(pack);
                            OnUAVIONIX_ADSB_OUT_DYNAMICReceive(this, ph, (UAVIONIX_ADSB_OUT_DYNAMIC) pack);
                            if(LOOP) break;
                            return null;
                        case 10003:
                            if(pack == null) return new UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT();
                            if(OnUAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORTReceive == null) return null;
                            ph.setPack(pack);
                            OnUAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORTReceive(this, ph, (UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT) pack);
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
            public event HEARTBEATReceiveHandler OnHEARTBEATReceive;
            public delegate void HEARTBEATReceiveHandler(Channel src, Inside ph, HEARTBEAT pack);

            public event SYS_STATUSReceiveHandler OnSYS_STATUSReceive;
            public delegate void SYS_STATUSReceiveHandler(Channel src, Inside ph, SYS_STATUS pack);

            public event SYSTEM_TIMEReceiveHandler OnSYSTEM_TIMEReceive;
            public delegate void SYSTEM_TIMEReceiveHandler(Channel src, Inside ph, SYSTEM_TIME pack);

            public event POSITION_TARGET_LOCAL_NEDReceiveHandler OnPOSITION_TARGET_LOCAL_NEDReceive;
            public delegate void POSITION_TARGET_LOCAL_NEDReceiveHandler(Channel src, Inside ph, POSITION_TARGET_LOCAL_NED pack);

            public event PINGReceiveHandler OnPINGReceive;
            public delegate void PINGReceiveHandler(Channel src, Inside ph, PING pack);

            public event CHANGE_OPERATOR_CONTROLReceiveHandler OnCHANGE_OPERATOR_CONTROLReceive;
            public delegate void CHANGE_OPERATOR_CONTROLReceiveHandler(Channel src, Inside ph, CHANGE_OPERATOR_CONTROL pack);

            public event CHANGE_OPERATOR_CONTROL_ACKReceiveHandler OnCHANGE_OPERATOR_CONTROL_ACKReceive;
            public delegate void CHANGE_OPERATOR_CONTROL_ACKReceiveHandler(Channel src, Inside ph, CHANGE_OPERATOR_CONTROL_ACK pack);

            public event AUTH_KEYReceiveHandler OnAUTH_KEYReceive;
            public delegate void AUTH_KEYReceiveHandler(Channel src, Inside ph, AUTH_KEY pack);

            public event SET_MODEReceiveHandler OnSET_MODEReceive;
            public delegate void SET_MODEReceiveHandler(Channel src, Inside ph, SET_MODE pack);

            public event PARAM_REQUEST_READReceiveHandler OnPARAM_REQUEST_READReceive;
            public delegate void PARAM_REQUEST_READReceiveHandler(Channel src, Inside ph, PARAM_REQUEST_READ pack);

            public event PARAM_REQUEST_LISTReceiveHandler OnPARAM_REQUEST_LISTReceive;
            public delegate void PARAM_REQUEST_LISTReceiveHandler(Channel src, Inside ph, PARAM_REQUEST_LIST pack);

            public event PARAM_VALUEReceiveHandler OnPARAM_VALUEReceive;
            public delegate void PARAM_VALUEReceiveHandler(Channel src, Inside ph, PARAM_VALUE pack);

            public event PARAM_SETReceiveHandler OnPARAM_SETReceive;
            public delegate void PARAM_SETReceiveHandler(Channel src, Inside ph, PARAM_SET pack);

            public event GPS_RAW_INTReceiveHandler OnGPS_RAW_INTReceive;
            public delegate void GPS_RAW_INTReceiveHandler(Channel src, Inside ph, GPS_RAW_INT pack);

            public event GPS_STATUSReceiveHandler OnGPS_STATUSReceive;
            public delegate void GPS_STATUSReceiveHandler(Channel src, Inside ph, GPS_STATUS pack);

            public event SCALED_IMUReceiveHandler OnSCALED_IMUReceive;
            public delegate void SCALED_IMUReceiveHandler(Channel src, Inside ph, SCALED_IMU pack);

            public event RAW_IMUReceiveHandler OnRAW_IMUReceive;
            public delegate void RAW_IMUReceiveHandler(Channel src, Inside ph, RAW_IMU pack);

            public event RAW_PRESSUREReceiveHandler OnRAW_PRESSUREReceive;
            public delegate void RAW_PRESSUREReceiveHandler(Channel src, Inside ph, RAW_PRESSURE pack);

            public event SCALED_PRESSUREReceiveHandler OnSCALED_PRESSUREReceive;
            public delegate void SCALED_PRESSUREReceiveHandler(Channel src, Inside ph, SCALED_PRESSURE pack);

            public event ATTITUDEReceiveHandler OnATTITUDEReceive;
            public delegate void ATTITUDEReceiveHandler(Channel src, Inside ph, ATTITUDE pack);

            public event ATTITUDE_QUATERNIONReceiveHandler OnATTITUDE_QUATERNIONReceive;
            public delegate void ATTITUDE_QUATERNIONReceiveHandler(Channel src, Inside ph, ATTITUDE_QUATERNION pack);

            public event LOCAL_POSITION_NEDReceiveHandler OnLOCAL_POSITION_NEDReceive;
            public delegate void LOCAL_POSITION_NEDReceiveHandler(Channel src, Inside ph, LOCAL_POSITION_NED pack);

            public event GLOBAL_POSITION_INTReceiveHandler OnGLOBAL_POSITION_INTReceive;
            public delegate void GLOBAL_POSITION_INTReceiveHandler(Channel src, Inside ph, GLOBAL_POSITION_INT pack);

            public event RC_CHANNELS_SCALEDReceiveHandler OnRC_CHANNELS_SCALEDReceive;
            public delegate void RC_CHANNELS_SCALEDReceiveHandler(Channel src, Inside ph, RC_CHANNELS_SCALED pack);

            public event RC_CHANNELS_RAWReceiveHandler OnRC_CHANNELS_RAWReceive;
            public delegate void RC_CHANNELS_RAWReceiveHandler(Channel src, Inside ph, RC_CHANNELS_RAW pack);

            public event SERVO_OUTPUT_RAWReceiveHandler OnSERVO_OUTPUT_RAWReceive;
            public delegate void SERVO_OUTPUT_RAWReceiveHandler(Channel src, Inside ph, SERVO_OUTPUT_RAW pack);

            public event MISSION_REQUEST_PARTIAL_LISTReceiveHandler OnMISSION_REQUEST_PARTIAL_LISTReceive;
            public delegate void MISSION_REQUEST_PARTIAL_LISTReceiveHandler(Channel src, Inside ph, MISSION_REQUEST_PARTIAL_LIST pack);

            public event MISSION_WRITE_PARTIAL_LISTReceiveHandler OnMISSION_WRITE_PARTIAL_LISTReceive;
            public delegate void MISSION_WRITE_PARTIAL_LISTReceiveHandler(Channel src, Inside ph, MISSION_WRITE_PARTIAL_LIST pack);

            public event MISSION_ITEMReceiveHandler OnMISSION_ITEMReceive;
            public delegate void MISSION_ITEMReceiveHandler(Channel src, Inside ph, MISSION_ITEM pack);

            public event MISSION_REQUESTReceiveHandler OnMISSION_REQUESTReceive;
            public delegate void MISSION_REQUESTReceiveHandler(Channel src, Inside ph, MISSION_REQUEST pack);

            public event MISSION_SET_CURRENTReceiveHandler OnMISSION_SET_CURRENTReceive;
            public delegate void MISSION_SET_CURRENTReceiveHandler(Channel src, Inside ph, MISSION_SET_CURRENT pack);

            public event MISSION_CURRENTReceiveHandler OnMISSION_CURRENTReceive;
            public delegate void MISSION_CURRENTReceiveHandler(Channel src, Inside ph, MISSION_CURRENT pack);

            public event MISSION_REQUEST_LISTReceiveHandler OnMISSION_REQUEST_LISTReceive;
            public delegate void MISSION_REQUEST_LISTReceiveHandler(Channel src, Inside ph, MISSION_REQUEST_LIST pack);

            public event MISSION_COUNTReceiveHandler OnMISSION_COUNTReceive;
            public delegate void MISSION_COUNTReceiveHandler(Channel src, Inside ph, MISSION_COUNT pack);

            public event MISSION_CLEAR_ALLReceiveHandler OnMISSION_CLEAR_ALLReceive;
            public delegate void MISSION_CLEAR_ALLReceiveHandler(Channel src, Inside ph, MISSION_CLEAR_ALL pack);

            public event MISSION_ITEM_REACHEDReceiveHandler OnMISSION_ITEM_REACHEDReceive;
            public delegate void MISSION_ITEM_REACHEDReceiveHandler(Channel src, Inside ph, MISSION_ITEM_REACHED pack);

            public event MISSION_ACKReceiveHandler OnMISSION_ACKReceive;
            public delegate void MISSION_ACKReceiveHandler(Channel src, Inside ph, MISSION_ACK pack);

            public event SET_GPS_GLOBAL_ORIGINReceiveHandler OnSET_GPS_GLOBAL_ORIGINReceive;
            public delegate void SET_GPS_GLOBAL_ORIGINReceiveHandler(Channel src, Inside ph, SET_GPS_GLOBAL_ORIGIN pack);

            public event GPS_GLOBAL_ORIGINReceiveHandler OnGPS_GLOBAL_ORIGINReceive;
            public delegate void GPS_GLOBAL_ORIGINReceiveHandler(Channel src, Inside ph, GPS_GLOBAL_ORIGIN pack);

            public event PARAM_MAP_RCReceiveHandler OnPARAM_MAP_RCReceive;
            public delegate void PARAM_MAP_RCReceiveHandler(Channel src, Inside ph, PARAM_MAP_RC pack);

            public event MISSION_REQUEST_INTReceiveHandler OnMISSION_REQUEST_INTReceive;
            public delegate void MISSION_REQUEST_INTReceiveHandler(Channel src, Inside ph, MISSION_REQUEST_INT pack);

            public event SAFETY_SET_ALLOWED_AREAReceiveHandler OnSAFETY_SET_ALLOWED_AREAReceive;
            public delegate void SAFETY_SET_ALLOWED_AREAReceiveHandler(Channel src, Inside ph, SAFETY_SET_ALLOWED_AREA pack);

            public event SAFETY_ALLOWED_AREAReceiveHandler OnSAFETY_ALLOWED_AREAReceive;
            public delegate void SAFETY_ALLOWED_AREAReceiveHandler(Channel src, Inside ph, SAFETY_ALLOWED_AREA pack);

            public event ATTITUDE_QUATERNION_COVReceiveHandler OnATTITUDE_QUATERNION_COVReceive;
            public delegate void ATTITUDE_QUATERNION_COVReceiveHandler(Channel src, Inside ph, ATTITUDE_QUATERNION_COV pack);

            public event NAV_CONTROLLER_OUTPUTReceiveHandler OnNAV_CONTROLLER_OUTPUTReceive;
            public delegate void NAV_CONTROLLER_OUTPUTReceiveHandler(Channel src, Inside ph, NAV_CONTROLLER_OUTPUT pack);

            public event GLOBAL_POSITION_INT_COVReceiveHandler OnGLOBAL_POSITION_INT_COVReceive;
            public delegate void GLOBAL_POSITION_INT_COVReceiveHandler(Channel src, Inside ph, GLOBAL_POSITION_INT_COV pack);

            public event LOCAL_POSITION_NED_COVReceiveHandler OnLOCAL_POSITION_NED_COVReceive;
            public delegate void LOCAL_POSITION_NED_COVReceiveHandler(Channel src, Inside ph, LOCAL_POSITION_NED_COV pack);

            public event RC_CHANNELSReceiveHandler OnRC_CHANNELSReceive;
            public delegate void RC_CHANNELSReceiveHandler(Channel src, Inside ph, RC_CHANNELS pack);

            public event REQUEST_DATA_STREAMReceiveHandler OnREQUEST_DATA_STREAMReceive;
            public delegate void REQUEST_DATA_STREAMReceiveHandler(Channel src, Inside ph, REQUEST_DATA_STREAM pack);

            public event DATA_STREAMReceiveHandler OnDATA_STREAMReceive;
            public delegate void DATA_STREAMReceiveHandler(Channel src, Inside ph, DATA_STREAM pack);

            public event MANUAL_CONTROLReceiveHandler OnMANUAL_CONTROLReceive;
            public delegate void MANUAL_CONTROLReceiveHandler(Channel src, Inside ph, MANUAL_CONTROL pack);

            public event RC_CHANNELS_OVERRIDEReceiveHandler OnRC_CHANNELS_OVERRIDEReceive;
            public delegate void RC_CHANNELS_OVERRIDEReceiveHandler(Channel src, Inside ph, RC_CHANNELS_OVERRIDE pack);

            public event MISSION_ITEM_INTReceiveHandler OnMISSION_ITEM_INTReceive;
            public delegate void MISSION_ITEM_INTReceiveHandler(Channel src, Inside ph, MISSION_ITEM_INT pack);

            public event VFR_HUDReceiveHandler OnVFR_HUDReceive;
            public delegate void VFR_HUDReceiveHandler(Channel src, Inside ph, VFR_HUD pack);

            public event COMMAND_INTReceiveHandler OnCOMMAND_INTReceive;
            public delegate void COMMAND_INTReceiveHandler(Channel src, Inside ph, COMMAND_INT pack);

            public event COMMAND_LONGReceiveHandler OnCOMMAND_LONGReceive;
            public delegate void COMMAND_LONGReceiveHandler(Channel src, Inside ph, COMMAND_LONG pack);

            public event COMMAND_ACKReceiveHandler OnCOMMAND_ACKReceive;
            public delegate void COMMAND_ACKReceiveHandler(Channel src, Inside ph, COMMAND_ACK pack);

            public event MANUAL_SETPOINTReceiveHandler OnMANUAL_SETPOINTReceive;
            public delegate void MANUAL_SETPOINTReceiveHandler(Channel src, Inside ph, MANUAL_SETPOINT pack);

            public event SET_ATTITUDE_TARGETReceiveHandler OnSET_ATTITUDE_TARGETReceive;
            public delegate void SET_ATTITUDE_TARGETReceiveHandler(Channel src, Inside ph, SET_ATTITUDE_TARGET pack);

            public event ATTITUDE_TARGETReceiveHandler OnATTITUDE_TARGETReceive;
            public delegate void ATTITUDE_TARGETReceiveHandler(Channel src, Inside ph, ATTITUDE_TARGET pack);

            public event SET_POSITION_TARGET_LOCAL_NEDReceiveHandler OnSET_POSITION_TARGET_LOCAL_NEDReceive;
            public delegate void SET_POSITION_TARGET_LOCAL_NEDReceiveHandler(Channel src, Inside ph, SET_POSITION_TARGET_LOCAL_NED pack);

            public event SET_POSITION_TARGET_GLOBAL_INTReceiveHandler OnSET_POSITION_TARGET_GLOBAL_INTReceive;
            public delegate void SET_POSITION_TARGET_GLOBAL_INTReceiveHandler(Channel src, Inside ph, SET_POSITION_TARGET_GLOBAL_INT pack);

            public event POSITION_TARGET_GLOBAL_INTReceiveHandler OnPOSITION_TARGET_GLOBAL_INTReceive;
            public delegate void POSITION_TARGET_GLOBAL_INTReceiveHandler(Channel src, Inside ph, POSITION_TARGET_GLOBAL_INT pack);

            public event LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceiveHandler OnLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceive;
            public delegate void LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceiveHandler(Channel src, Inside ph, LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET pack);

            public event HIL_STATEReceiveHandler OnHIL_STATEReceive;
            public delegate void HIL_STATEReceiveHandler(Channel src, Inside ph, HIL_STATE pack);

            public event HIL_CONTROLSReceiveHandler OnHIL_CONTROLSReceive;
            public delegate void HIL_CONTROLSReceiveHandler(Channel src, Inside ph, HIL_CONTROLS pack);

            public event HIL_RC_INPUTS_RAWReceiveHandler OnHIL_RC_INPUTS_RAWReceive;
            public delegate void HIL_RC_INPUTS_RAWReceiveHandler(Channel src, Inside ph, HIL_RC_INPUTS_RAW pack);

            public event HIL_ACTUATOR_CONTROLSReceiveHandler OnHIL_ACTUATOR_CONTROLSReceive;
            public delegate void HIL_ACTUATOR_CONTROLSReceiveHandler(Channel src, Inside ph, HIL_ACTUATOR_CONTROLS pack);

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

            public event HIGHRES_IMUReceiveHandler OnHIGHRES_IMUReceive;
            public delegate void HIGHRES_IMUReceiveHandler(Channel src, Inside ph, HIGHRES_IMU pack);

            public event OPTICAL_FLOW_RADReceiveHandler OnOPTICAL_FLOW_RADReceive;
            public delegate void OPTICAL_FLOW_RADReceiveHandler(Channel src, Inside ph, OPTICAL_FLOW_RAD pack);

            public event HIL_SENSORReceiveHandler OnHIL_SENSORReceive;
            public delegate void HIL_SENSORReceiveHandler(Channel src, Inside ph, HIL_SENSOR pack);

            public event SIM_STATEReceiveHandler OnSIM_STATEReceive;
            public delegate void SIM_STATEReceiveHandler(Channel src, Inside ph, SIM_STATE pack);

            public event RADIO_STATUSReceiveHandler OnRADIO_STATUSReceive;
            public delegate void RADIO_STATUSReceiveHandler(Channel src, Inside ph, RADIO_STATUS pack);

            public event FILE_TRANSFER_PROTOCOLReceiveHandler OnFILE_TRANSFER_PROTOCOLReceive;
            public delegate void FILE_TRANSFER_PROTOCOLReceiveHandler(Channel src, Inside ph, FILE_TRANSFER_PROTOCOL pack);

            public event TIMESYNCReceiveHandler OnTIMESYNCReceive;
            public delegate void TIMESYNCReceiveHandler(Channel src, Inside ph, TIMESYNC pack);

            public event CAMERA_TRIGGERReceiveHandler OnCAMERA_TRIGGERReceive;
            public delegate void CAMERA_TRIGGERReceiveHandler(Channel src, Inside ph, CAMERA_TRIGGER pack);

            public event HIL_GPSReceiveHandler OnHIL_GPSReceive;
            public delegate void HIL_GPSReceiveHandler(Channel src, Inside ph, HIL_GPS pack);

            public event HIL_OPTICAL_FLOWReceiveHandler OnHIL_OPTICAL_FLOWReceive;
            public delegate void HIL_OPTICAL_FLOWReceiveHandler(Channel src, Inside ph, HIL_OPTICAL_FLOW pack);

            public event HIL_STATE_QUATERNIONReceiveHandler OnHIL_STATE_QUATERNIONReceive;
            public delegate void HIL_STATE_QUATERNIONReceiveHandler(Channel src, Inside ph, HIL_STATE_QUATERNION pack);

            public event SCALED_IMU2ReceiveHandler OnSCALED_IMU2Receive;
            public delegate void SCALED_IMU2ReceiveHandler(Channel src, Inside ph, SCALED_IMU2 pack);

            public event LOG_REQUEST_LISTReceiveHandler OnLOG_REQUEST_LISTReceive;
            public delegate void LOG_REQUEST_LISTReceiveHandler(Channel src, Inside ph, LOG_REQUEST_LIST pack);

            public event LOG_ENTRYReceiveHandler OnLOG_ENTRYReceive;
            public delegate void LOG_ENTRYReceiveHandler(Channel src, Inside ph, LOG_ENTRY pack);

            public event LOG_REQUEST_DATAReceiveHandler OnLOG_REQUEST_DATAReceive;
            public delegate void LOG_REQUEST_DATAReceiveHandler(Channel src, Inside ph, LOG_REQUEST_DATA pack);

            public event LOG_DATAReceiveHandler OnLOG_DATAReceive;
            public delegate void LOG_DATAReceiveHandler(Channel src, Inside ph, LOG_DATA pack);

            public event LOG_ERASEReceiveHandler OnLOG_ERASEReceive;
            public delegate void LOG_ERASEReceiveHandler(Channel src, Inside ph, LOG_ERASE pack);

            public event LOG_REQUEST_ENDReceiveHandler OnLOG_REQUEST_ENDReceive;
            public delegate void LOG_REQUEST_ENDReceiveHandler(Channel src, Inside ph, LOG_REQUEST_END pack);

            public event GPS_INJECT_DATAReceiveHandler OnGPS_INJECT_DATAReceive;
            public delegate void GPS_INJECT_DATAReceiveHandler(Channel src, Inside ph, GPS_INJECT_DATA pack);

            public event GPS2_RAWReceiveHandler OnGPS2_RAWReceive;
            public delegate void GPS2_RAWReceiveHandler(Channel src, Inside ph, GPS2_RAW pack);

            public event POWER_STATUSReceiveHandler OnPOWER_STATUSReceive;
            public delegate void POWER_STATUSReceiveHandler(Channel src, Inside ph, POWER_STATUS pack);

            public event SERIAL_CONTROLReceiveHandler OnSERIAL_CONTROLReceive;
            public delegate void SERIAL_CONTROLReceiveHandler(Channel src, Inside ph, SERIAL_CONTROL pack);

            public event GPS_RTKReceiveHandler OnGPS_RTKReceive;
            public delegate void GPS_RTKReceiveHandler(Channel src, Inside ph, GPS_RTK pack);

            public event GPS2_RTKReceiveHandler OnGPS2_RTKReceive;
            public delegate void GPS2_RTKReceiveHandler(Channel src, Inside ph, GPS2_RTK pack);

            public event SCALED_IMU3ReceiveHandler OnSCALED_IMU3Receive;
            public delegate void SCALED_IMU3ReceiveHandler(Channel src, Inside ph, SCALED_IMU3 pack);

            public event DATA_TRANSMISSION_HANDSHAKEReceiveHandler OnDATA_TRANSMISSION_HANDSHAKEReceive;
            public delegate void DATA_TRANSMISSION_HANDSHAKEReceiveHandler(Channel src, Inside ph, DATA_TRANSMISSION_HANDSHAKE pack);

            public event ENCAPSULATED_DATAReceiveHandler OnENCAPSULATED_DATAReceive;
            public delegate void ENCAPSULATED_DATAReceiveHandler(Channel src, Inside ph, ENCAPSULATED_DATA pack);

            public event DISTANCE_SENSORReceiveHandler OnDISTANCE_SENSORReceive;
            public delegate void DISTANCE_SENSORReceiveHandler(Channel src, Inside ph, DISTANCE_SENSOR pack);

            public event TERRAIN_REQUESTReceiveHandler OnTERRAIN_REQUESTReceive;
            public delegate void TERRAIN_REQUESTReceiveHandler(Channel src, Inside ph, TERRAIN_REQUEST pack);

            public event TERRAIN_DATAReceiveHandler OnTERRAIN_DATAReceive;
            public delegate void TERRAIN_DATAReceiveHandler(Channel src, Inside ph, TERRAIN_DATA pack);

            public event TERRAIN_CHECKReceiveHandler OnTERRAIN_CHECKReceive;
            public delegate void TERRAIN_CHECKReceiveHandler(Channel src, Inside ph, TERRAIN_CHECK pack);

            public event TERRAIN_REPORTReceiveHandler OnTERRAIN_REPORTReceive;
            public delegate void TERRAIN_REPORTReceiveHandler(Channel src, Inside ph, TERRAIN_REPORT pack);

            public event SCALED_PRESSURE2ReceiveHandler OnSCALED_PRESSURE2Receive;
            public delegate void SCALED_PRESSURE2ReceiveHandler(Channel src, Inside ph, SCALED_PRESSURE2 pack);

            public event ATT_POS_MOCAPReceiveHandler OnATT_POS_MOCAPReceive;
            public delegate void ATT_POS_MOCAPReceiveHandler(Channel src, Inside ph, ATT_POS_MOCAP pack);

            public event SET_ACTUATOR_CONTROL_TARGETReceiveHandler OnSET_ACTUATOR_CONTROL_TARGETReceive;
            public delegate void SET_ACTUATOR_CONTROL_TARGETReceiveHandler(Channel src, Inside ph, SET_ACTUATOR_CONTROL_TARGET pack);

            public event ACTUATOR_CONTROL_TARGETReceiveHandler OnACTUATOR_CONTROL_TARGETReceive;
            public delegate void ACTUATOR_CONTROL_TARGETReceiveHandler(Channel src, Inside ph, ACTUATOR_CONTROL_TARGET pack);

            public event ALTITUDEReceiveHandler OnALTITUDEReceive;
            public delegate void ALTITUDEReceiveHandler(Channel src, Inside ph, ALTITUDE pack);

            public event RESOURCE_REQUESTReceiveHandler OnRESOURCE_REQUESTReceive;
            public delegate void RESOURCE_REQUESTReceiveHandler(Channel src, Inside ph, RESOURCE_REQUEST pack);

            public event SCALED_PRESSURE3ReceiveHandler OnSCALED_PRESSURE3Receive;
            public delegate void SCALED_PRESSURE3ReceiveHandler(Channel src, Inside ph, SCALED_PRESSURE3 pack);

            public event FOLLOW_TARGETReceiveHandler OnFOLLOW_TARGETReceive;
            public delegate void FOLLOW_TARGETReceiveHandler(Channel src, Inside ph, FOLLOW_TARGET pack);

            public event CONTROL_SYSTEM_STATEReceiveHandler OnCONTROL_SYSTEM_STATEReceive;
            public delegate void CONTROL_SYSTEM_STATEReceiveHandler(Channel src, Inside ph, CONTROL_SYSTEM_STATE pack);

            public event BATTERY_STATUSReceiveHandler OnBATTERY_STATUSReceive;
            public delegate void BATTERY_STATUSReceiveHandler(Channel src, Inside ph, BATTERY_STATUS pack);

            public event AUTOPILOT_VERSIONReceiveHandler OnAUTOPILOT_VERSIONReceive;
            public delegate void AUTOPILOT_VERSIONReceiveHandler(Channel src, Inside ph, AUTOPILOT_VERSION pack);

            public event LANDING_TARGETReceiveHandler OnLANDING_TARGETReceive;
            public delegate void LANDING_TARGETReceiveHandler(Channel src, Inside ph, LANDING_TARGET pack);

            public event ESTIMATOR_STATUSReceiveHandler OnESTIMATOR_STATUSReceive;
            public delegate void ESTIMATOR_STATUSReceiveHandler(Channel src, Inside ph, ESTIMATOR_STATUS pack);

            public event WIND_COVReceiveHandler OnWIND_COVReceive;
            public delegate void WIND_COVReceiveHandler(Channel src, Inside ph, WIND_COV pack);

            public event GPS_INPUTReceiveHandler OnGPS_INPUTReceive;
            public delegate void GPS_INPUTReceiveHandler(Channel src, Inside ph, GPS_INPUT pack);

            public event GPS_RTCM_DATAReceiveHandler OnGPS_RTCM_DATAReceive;
            public delegate void GPS_RTCM_DATAReceiveHandler(Channel src, Inside ph, GPS_RTCM_DATA pack);

            public event HIGH_LATENCYReceiveHandler OnHIGH_LATENCYReceive;
            public delegate void HIGH_LATENCYReceiveHandler(Channel src, Inside ph, HIGH_LATENCY pack);

            public event VIBRATIONReceiveHandler OnVIBRATIONReceive;
            public delegate void VIBRATIONReceiveHandler(Channel src, Inside ph, VIBRATION pack);

            public event HOME_POSITIONReceiveHandler OnHOME_POSITIONReceive;
            public delegate void HOME_POSITIONReceiveHandler(Channel src, Inside ph, HOME_POSITION pack);

            public event SET_HOME_POSITIONReceiveHandler OnSET_HOME_POSITIONReceive;
            public delegate void SET_HOME_POSITIONReceiveHandler(Channel src, Inside ph, SET_HOME_POSITION pack);

            public event MESSAGE_INTERVALReceiveHandler OnMESSAGE_INTERVALReceive;
            public delegate void MESSAGE_INTERVALReceiveHandler(Channel src, Inside ph, MESSAGE_INTERVAL pack);

            public event EXTENDED_SYS_STATEReceiveHandler OnEXTENDED_SYS_STATEReceive;
            public delegate void EXTENDED_SYS_STATEReceiveHandler(Channel src, Inside ph, EXTENDED_SYS_STATE pack);

            public event ADSB_VEHICLEReceiveHandler OnADSB_VEHICLEReceive;
            public delegate void ADSB_VEHICLEReceiveHandler(Channel src, Inside ph, ADSB_VEHICLE pack);

            public event COLLISIONReceiveHandler OnCOLLISIONReceive;
            public delegate void COLLISIONReceiveHandler(Channel src, Inside ph, COLLISION pack);

            public event V2_EXTENSIONReceiveHandler OnV2_EXTENSIONReceive;
            public delegate void V2_EXTENSIONReceiveHandler(Channel src, Inside ph, V2_EXTENSION pack);

            public event MEMORY_VECTReceiveHandler OnMEMORY_VECTReceive;
            public delegate void MEMORY_VECTReceiveHandler(Channel src, Inside ph, MEMORY_VECT pack);

            public event DEBUG_VECTReceiveHandler OnDEBUG_VECTReceive;
            public delegate void DEBUG_VECTReceiveHandler(Channel src, Inside ph, DEBUG_VECT pack);

            public event NAMED_VALUE_FLOATReceiveHandler OnNAMED_VALUE_FLOATReceive;
            public delegate void NAMED_VALUE_FLOATReceiveHandler(Channel src, Inside ph, NAMED_VALUE_FLOAT pack);

            public event NAMED_VALUE_INTReceiveHandler OnNAMED_VALUE_INTReceive;
            public delegate void NAMED_VALUE_INTReceiveHandler(Channel src, Inside ph, NAMED_VALUE_INT pack);

            public event STATUSTEXTReceiveHandler OnSTATUSTEXTReceive;
            public delegate void STATUSTEXTReceiveHandler(Channel src, Inside ph, STATUSTEXT pack);

            public event DEBUGReceiveHandler OnDEBUGReceive;
            public delegate void DEBUGReceiveHandler(Channel src, Inside ph, DEBUG pack);

            public event SETUP_SIGNINGReceiveHandler OnSETUP_SIGNINGReceive;
            public delegate void SETUP_SIGNINGReceiveHandler(Channel src, Inside ph, SETUP_SIGNING pack);

            public event BUTTON_CHANGEReceiveHandler OnBUTTON_CHANGEReceive;
            public delegate void BUTTON_CHANGEReceiveHandler(Channel src, Inside ph, BUTTON_CHANGE pack);

            public event PLAY_TUNEReceiveHandler OnPLAY_TUNEReceive;
            public delegate void PLAY_TUNEReceiveHandler(Channel src, Inside ph, PLAY_TUNE pack);

            public event CAMERA_INFORMATIONReceiveHandler OnCAMERA_INFORMATIONReceive;
            public delegate void CAMERA_INFORMATIONReceiveHandler(Channel src, Inside ph, CAMERA_INFORMATION pack);

            public event CAMERA_SETTINGSReceiveHandler OnCAMERA_SETTINGSReceive;
            public delegate void CAMERA_SETTINGSReceiveHandler(Channel src, Inside ph, CAMERA_SETTINGS pack);

            public event STORAGE_INFORMATIONReceiveHandler OnSTORAGE_INFORMATIONReceive;
            public delegate void STORAGE_INFORMATIONReceiveHandler(Channel src, Inside ph, STORAGE_INFORMATION pack);

            public event CAMERA_CAPTURE_STATUSReceiveHandler OnCAMERA_CAPTURE_STATUSReceive;
            public delegate void CAMERA_CAPTURE_STATUSReceiveHandler(Channel src, Inside ph, CAMERA_CAPTURE_STATUS pack);

            public event CAMERA_IMAGE_CAPTUREDReceiveHandler OnCAMERA_IMAGE_CAPTUREDReceive;
            public delegate void CAMERA_IMAGE_CAPTUREDReceiveHandler(Channel src, Inside ph, CAMERA_IMAGE_CAPTURED pack);

            public event FLIGHT_INFORMATIONReceiveHandler OnFLIGHT_INFORMATIONReceive;
            public delegate void FLIGHT_INFORMATIONReceiveHandler(Channel src, Inside ph, FLIGHT_INFORMATION pack);

            public event MOUNT_ORIENTATIONReceiveHandler OnMOUNT_ORIENTATIONReceive;
            public delegate void MOUNT_ORIENTATIONReceiveHandler(Channel src, Inside ph, MOUNT_ORIENTATION pack);

            public event LOGGING_DATAReceiveHandler OnLOGGING_DATAReceive;
            public delegate void LOGGING_DATAReceiveHandler(Channel src, Inside ph, LOGGING_DATA pack);

            public event LOGGING_DATA_ACKEDReceiveHandler OnLOGGING_DATA_ACKEDReceive;
            public delegate void LOGGING_DATA_ACKEDReceiveHandler(Channel src, Inside ph, LOGGING_DATA_ACKED pack);

            public event LOGGING_ACKReceiveHandler OnLOGGING_ACKReceive;
            public delegate void LOGGING_ACKReceiveHandler(Channel src, Inside ph, LOGGING_ACK pack);

            public event VIDEO_STREAM_INFORMATIONReceiveHandler OnVIDEO_STREAM_INFORMATIONReceive;
            public delegate void VIDEO_STREAM_INFORMATIONReceiveHandler(Channel src, Inside ph, VIDEO_STREAM_INFORMATION pack);

            public event SET_VIDEO_STREAM_SETTINGSReceiveHandler OnSET_VIDEO_STREAM_SETTINGSReceive;
            public delegate void SET_VIDEO_STREAM_SETTINGSReceiveHandler(Channel src, Inside ph, SET_VIDEO_STREAM_SETTINGS pack);

            public event WIFI_CONFIG_APReceiveHandler OnWIFI_CONFIG_APReceive;
            public delegate void WIFI_CONFIG_APReceiveHandler(Channel src, Inside ph, WIFI_CONFIG_AP pack);

            public event PROTOCOL_VERSIONReceiveHandler OnPROTOCOL_VERSIONReceive;
            public delegate void PROTOCOL_VERSIONReceiveHandler(Channel src, Inside ph, PROTOCOL_VERSION pack);

            public event UAVCAN_NODE_STATUSReceiveHandler OnUAVCAN_NODE_STATUSReceive;
            public delegate void UAVCAN_NODE_STATUSReceiveHandler(Channel src, Inside ph, UAVCAN_NODE_STATUS pack);

            public event UAVCAN_NODE_INFOReceiveHandler OnUAVCAN_NODE_INFOReceive;
            public delegate void UAVCAN_NODE_INFOReceiveHandler(Channel src, Inside ph, UAVCAN_NODE_INFO pack);

            public event PARAM_EXT_REQUEST_READReceiveHandler OnPARAM_EXT_REQUEST_READReceive;
            public delegate void PARAM_EXT_REQUEST_READReceiveHandler(Channel src, Inside ph, PARAM_EXT_REQUEST_READ pack);

            public event PARAM_EXT_REQUEST_LISTReceiveHandler OnPARAM_EXT_REQUEST_LISTReceive;
            public delegate void PARAM_EXT_REQUEST_LISTReceiveHandler(Channel src, Inside ph, PARAM_EXT_REQUEST_LIST pack);

            public event PARAM_EXT_VALUEReceiveHandler OnPARAM_EXT_VALUEReceive;
            public delegate void PARAM_EXT_VALUEReceiveHandler(Channel src, Inside ph, PARAM_EXT_VALUE pack);

            public event PARAM_EXT_SETReceiveHandler OnPARAM_EXT_SETReceive;
            public delegate void PARAM_EXT_SETReceiveHandler(Channel src, Inside ph, PARAM_EXT_SET pack);

            public event PARAM_EXT_ACKReceiveHandler OnPARAM_EXT_ACKReceive;
            public delegate void PARAM_EXT_ACKReceiveHandler(Channel src, Inside ph, PARAM_EXT_ACK pack);

            public event OBSTACLE_DISTANCEReceiveHandler OnOBSTACLE_DISTANCEReceive;
            public delegate void OBSTACLE_DISTANCEReceiveHandler(Channel src, Inside ph, OBSTACLE_DISTANCE pack);

            public event UAVIONIX_ADSB_OUT_CFGReceiveHandler OnUAVIONIX_ADSB_OUT_CFGReceive;
            public delegate void UAVIONIX_ADSB_OUT_CFGReceiveHandler(Channel src, Inside ph, UAVIONIX_ADSB_OUT_CFG pack);

            public event UAVIONIX_ADSB_OUT_DYNAMICReceiveHandler OnUAVIONIX_ADSB_OUT_DYNAMICReceive;
            public delegate void UAVIONIX_ADSB_OUT_DYNAMICReceiveHandler(Channel src, Inside ph, UAVIONIX_ADSB_OUT_DYNAMIC pack);

            public event UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORTReceiveHandler OnUAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORTReceive;
            public delegate void UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORTReceiveHandler(Channel src, Inside ph, UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT pack);
        }
        public class LoopBackDemoChannel_ADV : Channel.Advanced
        {
            public override bool CanRead { get { return false ; } }
            public override bool CanWrite { get { return true; } }
            static LoopBackDemoChannel_ADV() {pack_id_bytes = 2;}

            public static  LoopBackDemoChannel_ADV instance = new LoopBackDemoChannel_ADV();

            //interface-to-mark of sendable through this channel packs_Schs_Rchs
            public interface Sendable {}
            public static HEARTBEAT new_HEARTBEAT() {return new  HEARTBEAT();}
            public static SYS_STATUS new_SYS_STATUS() {return new  SYS_STATUS();}
            public static SYSTEM_TIME new_SYSTEM_TIME() {return new  SYSTEM_TIME();}
            public static POSITION_TARGET_LOCAL_NED new_POSITION_TARGET_LOCAL_NED() {return new  POSITION_TARGET_LOCAL_NED();}
            public static PING new_PING() {return new  PING();}
            public static CHANGE_OPERATOR_CONTROL new_CHANGE_OPERATOR_CONTROL() {return new  CHANGE_OPERATOR_CONTROL();}
            public static CHANGE_OPERATOR_CONTROL_ACK new_CHANGE_OPERATOR_CONTROL_ACK() {return new  CHANGE_OPERATOR_CONTROL_ACK();}
            public static AUTH_KEY new_AUTH_KEY() {return new  AUTH_KEY();}
            public static SET_MODE new_SET_MODE() {return new  SET_MODE();}
            public static PARAM_REQUEST_READ new_PARAM_REQUEST_READ() {return new  PARAM_REQUEST_READ();}
            public static PARAM_REQUEST_LIST new_PARAM_REQUEST_LIST() {return new  PARAM_REQUEST_LIST();}
            public static PARAM_VALUE new_PARAM_VALUE() {return new  PARAM_VALUE();}
            public static PARAM_SET new_PARAM_SET() {return new  PARAM_SET();}
            public static GPS_RAW_INT new_GPS_RAW_INT() {return new  GPS_RAW_INT();}
            public static GPS_STATUS new_GPS_STATUS() {return new  GPS_STATUS();}
            public static SCALED_IMU new_SCALED_IMU() {return new  SCALED_IMU();}
            public static RAW_IMU new_RAW_IMU() {return new  RAW_IMU();}
            public static RAW_PRESSURE new_RAW_PRESSURE() {return new  RAW_PRESSURE();}
            public static SCALED_PRESSURE new_SCALED_PRESSURE() {return new  SCALED_PRESSURE();}
            public static ATTITUDE new_ATTITUDE() {return new  ATTITUDE();}
            public static ATTITUDE_QUATERNION new_ATTITUDE_QUATERNION() {return new  ATTITUDE_QUATERNION();}
            public static LOCAL_POSITION_NED new_LOCAL_POSITION_NED() {return new  LOCAL_POSITION_NED();}
            public static GLOBAL_POSITION_INT new_GLOBAL_POSITION_INT() {return new  GLOBAL_POSITION_INT();}
            public static RC_CHANNELS_SCALED new_RC_CHANNELS_SCALED() {return new  RC_CHANNELS_SCALED();}
            public static RC_CHANNELS_RAW new_RC_CHANNELS_RAW() {return new  RC_CHANNELS_RAW();}
            public static SERVO_OUTPUT_RAW new_SERVO_OUTPUT_RAW() {return new  SERVO_OUTPUT_RAW();}
            public static MISSION_REQUEST_PARTIAL_LIST new_MISSION_REQUEST_PARTIAL_LIST() {return new  MISSION_REQUEST_PARTIAL_LIST();}
            public static MISSION_WRITE_PARTIAL_LIST new_MISSION_WRITE_PARTIAL_LIST() {return new  MISSION_WRITE_PARTIAL_LIST();}
            public static MISSION_ITEM new_MISSION_ITEM() {return new  MISSION_ITEM();}
            public static MISSION_REQUEST new_MISSION_REQUEST() {return new  MISSION_REQUEST();}
            public static MISSION_SET_CURRENT new_MISSION_SET_CURRENT() {return new  MISSION_SET_CURRENT();}
            public static MISSION_CURRENT new_MISSION_CURRENT() {return new  MISSION_CURRENT();}
            public static MISSION_REQUEST_LIST new_MISSION_REQUEST_LIST() {return new  MISSION_REQUEST_LIST();}
            public static MISSION_COUNT new_MISSION_COUNT() {return new  MISSION_COUNT();}
            public static MISSION_CLEAR_ALL new_MISSION_CLEAR_ALL() {return new  MISSION_CLEAR_ALL();}
            public static MISSION_ITEM_REACHED new_MISSION_ITEM_REACHED() {return new  MISSION_ITEM_REACHED();}
            public static MISSION_ACK new_MISSION_ACK() {return new  MISSION_ACK();}
            public static SET_GPS_GLOBAL_ORIGIN new_SET_GPS_GLOBAL_ORIGIN() {return new  SET_GPS_GLOBAL_ORIGIN();}
            public static GPS_GLOBAL_ORIGIN new_GPS_GLOBAL_ORIGIN() {return new  GPS_GLOBAL_ORIGIN();}
            public static PARAM_MAP_RC new_PARAM_MAP_RC() {return new  PARAM_MAP_RC();}
            public static MISSION_REQUEST_INT new_MISSION_REQUEST_INT() {return new  MISSION_REQUEST_INT();}
            public static SAFETY_SET_ALLOWED_AREA new_SAFETY_SET_ALLOWED_AREA() {return new  SAFETY_SET_ALLOWED_AREA();}
            public static SAFETY_ALLOWED_AREA new_SAFETY_ALLOWED_AREA() {return new  SAFETY_ALLOWED_AREA();}
            public static ATTITUDE_QUATERNION_COV new_ATTITUDE_QUATERNION_COV() {return new  ATTITUDE_QUATERNION_COV();}
            public static NAV_CONTROLLER_OUTPUT new_NAV_CONTROLLER_OUTPUT() {return new  NAV_CONTROLLER_OUTPUT();}
            public static GLOBAL_POSITION_INT_COV new_GLOBAL_POSITION_INT_COV() {return new  GLOBAL_POSITION_INT_COV();}
            public static LOCAL_POSITION_NED_COV new_LOCAL_POSITION_NED_COV() {return new  LOCAL_POSITION_NED_COV();}
            public static RC_CHANNELS new_RC_CHANNELS() {return new  RC_CHANNELS();}
            public static REQUEST_DATA_STREAM new_REQUEST_DATA_STREAM() {return new  REQUEST_DATA_STREAM();}
            public static DATA_STREAM new_DATA_STREAM() {return new  DATA_STREAM();}
            public static MANUAL_CONTROL new_MANUAL_CONTROL() {return new  MANUAL_CONTROL();}
            public static RC_CHANNELS_OVERRIDE new_RC_CHANNELS_OVERRIDE() {return new  RC_CHANNELS_OVERRIDE();}
            public static MISSION_ITEM_INT new_MISSION_ITEM_INT() {return new  MISSION_ITEM_INT();}
            public static VFR_HUD new_VFR_HUD() {return new  VFR_HUD();}
            public static COMMAND_INT new_COMMAND_INT() {return new  COMMAND_INT();}
            public static COMMAND_LONG new_COMMAND_LONG() {return new  COMMAND_LONG();}
            public static COMMAND_ACK new_COMMAND_ACK() {return new  COMMAND_ACK();}
            public static MANUAL_SETPOINT new_MANUAL_SETPOINT() {return new  MANUAL_SETPOINT();}
            public static SET_ATTITUDE_TARGET new_SET_ATTITUDE_TARGET() {return new  SET_ATTITUDE_TARGET();}
            public static ATTITUDE_TARGET new_ATTITUDE_TARGET() {return new  ATTITUDE_TARGET();}
            public static SET_POSITION_TARGET_LOCAL_NED new_SET_POSITION_TARGET_LOCAL_NED() {return new  SET_POSITION_TARGET_LOCAL_NED();}
            public static SET_POSITION_TARGET_GLOBAL_INT new_SET_POSITION_TARGET_GLOBAL_INT() {return new  SET_POSITION_TARGET_GLOBAL_INT();}
            public static POSITION_TARGET_GLOBAL_INT new_POSITION_TARGET_GLOBAL_INT() {return new  POSITION_TARGET_GLOBAL_INT();}
            public static LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET() {return new  LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();}
            public static HIL_STATE new_HIL_STATE() {return new  HIL_STATE();}
            public static HIL_CONTROLS new_HIL_CONTROLS() {return new  HIL_CONTROLS();}
            public static HIL_RC_INPUTS_RAW new_HIL_RC_INPUTS_RAW() {return new  HIL_RC_INPUTS_RAW();}
            public static HIL_ACTUATOR_CONTROLS new_HIL_ACTUATOR_CONTROLS() {return new  HIL_ACTUATOR_CONTROLS();}
            public static OPTICAL_FLOW new_OPTICAL_FLOW() {return new  OPTICAL_FLOW();}
            public static GLOBAL_VISION_POSITION_ESTIMATE new_GLOBAL_VISION_POSITION_ESTIMATE() {return new  GLOBAL_VISION_POSITION_ESTIMATE();}
            public static VISION_POSITION_ESTIMATE new_VISION_POSITION_ESTIMATE() {return new  VISION_POSITION_ESTIMATE();}
            public static VISION_SPEED_ESTIMATE new_VISION_SPEED_ESTIMATE() {return new  VISION_SPEED_ESTIMATE();}
            public static VICON_POSITION_ESTIMATE new_VICON_POSITION_ESTIMATE() {return new  VICON_POSITION_ESTIMATE();}
            public static HIGHRES_IMU new_HIGHRES_IMU() {return new  HIGHRES_IMU();}
            public static OPTICAL_FLOW_RAD new_OPTICAL_FLOW_RAD() {return new  OPTICAL_FLOW_RAD();}
            public static HIL_SENSOR new_HIL_SENSOR() {return new  HIL_SENSOR();}
            public static SIM_STATE new_SIM_STATE() {return new  SIM_STATE();}
            public static RADIO_STATUS new_RADIO_STATUS() {return new  RADIO_STATUS();}
            public static FILE_TRANSFER_PROTOCOL new_FILE_TRANSFER_PROTOCOL() {return new  FILE_TRANSFER_PROTOCOL();}
            public static TIMESYNC new_TIMESYNC() {return new  TIMESYNC();}
            public static CAMERA_TRIGGER new_CAMERA_TRIGGER() {return new  CAMERA_TRIGGER();}
            public static HIL_GPS new_HIL_GPS() {return new  HIL_GPS();}
            public static HIL_OPTICAL_FLOW new_HIL_OPTICAL_FLOW() {return new  HIL_OPTICAL_FLOW();}
            public static HIL_STATE_QUATERNION new_HIL_STATE_QUATERNION() {return new  HIL_STATE_QUATERNION();}
            public static SCALED_IMU2 new_SCALED_IMU2() {return new  SCALED_IMU2();}
            public static LOG_REQUEST_LIST new_LOG_REQUEST_LIST() {return new  LOG_REQUEST_LIST();}
            public static LOG_ENTRY new_LOG_ENTRY() {return new  LOG_ENTRY();}
            public static LOG_REQUEST_DATA new_LOG_REQUEST_DATA() {return new  LOG_REQUEST_DATA();}
            public static LOG_DATA new_LOG_DATA() {return new  LOG_DATA();}
            public static LOG_ERASE new_LOG_ERASE() {return new  LOG_ERASE();}
            public static LOG_REQUEST_END new_LOG_REQUEST_END() {return new  LOG_REQUEST_END();}
            public static GPS_INJECT_DATA new_GPS_INJECT_DATA() {return new  GPS_INJECT_DATA();}
            public static GPS2_RAW new_GPS2_RAW() {return new  GPS2_RAW();}
            public static POWER_STATUS new_POWER_STATUS() {return new  POWER_STATUS();}
            public static SERIAL_CONTROL new_SERIAL_CONTROL() {return new  SERIAL_CONTROL();}
            public static GPS_RTK new_GPS_RTK() {return new  GPS_RTK();}
            public static GPS2_RTK new_GPS2_RTK() {return new  GPS2_RTK();}
            public static SCALED_IMU3 new_SCALED_IMU3() {return new  SCALED_IMU3();}
            public static DATA_TRANSMISSION_HANDSHAKE new_DATA_TRANSMISSION_HANDSHAKE() {return new  DATA_TRANSMISSION_HANDSHAKE();}
            public static ENCAPSULATED_DATA new_ENCAPSULATED_DATA() {return new  ENCAPSULATED_DATA();}
            public static DISTANCE_SENSOR new_DISTANCE_SENSOR() {return new  DISTANCE_SENSOR();}
            public static TERRAIN_REQUEST new_TERRAIN_REQUEST() {return new  TERRAIN_REQUEST();}
            public static TERRAIN_DATA new_TERRAIN_DATA() {return new  TERRAIN_DATA();}
            public static TERRAIN_CHECK new_TERRAIN_CHECK() {return new  TERRAIN_CHECK();}
            public static TERRAIN_REPORT new_TERRAIN_REPORT() {return new  TERRAIN_REPORT();}
            public static SCALED_PRESSURE2 new_SCALED_PRESSURE2() {return new  SCALED_PRESSURE2();}
            public static ATT_POS_MOCAP new_ATT_POS_MOCAP() {return new  ATT_POS_MOCAP();}
            public static SET_ACTUATOR_CONTROL_TARGET new_SET_ACTUATOR_CONTROL_TARGET() {return new  SET_ACTUATOR_CONTROL_TARGET();}
            public static ACTUATOR_CONTROL_TARGET new_ACTUATOR_CONTROL_TARGET() {return new  ACTUATOR_CONTROL_TARGET();}
            public static ALTITUDE new_ALTITUDE() {return new  ALTITUDE();}
            public static RESOURCE_REQUEST new_RESOURCE_REQUEST() {return new  RESOURCE_REQUEST();}
            public static SCALED_PRESSURE3 new_SCALED_PRESSURE3() {return new  SCALED_PRESSURE3();}
            public static FOLLOW_TARGET new_FOLLOW_TARGET() {return new  FOLLOW_TARGET();}
            public static CONTROL_SYSTEM_STATE new_CONTROL_SYSTEM_STATE() {return new  CONTROL_SYSTEM_STATE();}
            public static BATTERY_STATUS new_BATTERY_STATUS() {return new  BATTERY_STATUS();}
            public static AUTOPILOT_VERSION new_AUTOPILOT_VERSION() {return new  AUTOPILOT_VERSION();}
            public static LANDING_TARGET new_LANDING_TARGET() {return new  LANDING_TARGET();}
            public static ESTIMATOR_STATUS new_ESTIMATOR_STATUS() {return new  ESTIMATOR_STATUS();}
            public static WIND_COV new_WIND_COV() {return new  WIND_COV();}
            public static GPS_INPUT new_GPS_INPUT() {return new  GPS_INPUT();}
            public static GPS_RTCM_DATA new_GPS_RTCM_DATA() {return new  GPS_RTCM_DATA();}
            public static HIGH_LATENCY new_HIGH_LATENCY() {return new  HIGH_LATENCY();}
            public static VIBRATION new_VIBRATION() {return new  VIBRATION();}
            public static HOME_POSITION new_HOME_POSITION() {return new  HOME_POSITION();}
            public static SET_HOME_POSITION new_SET_HOME_POSITION() {return new  SET_HOME_POSITION();}
            public static MESSAGE_INTERVAL new_MESSAGE_INTERVAL() {return new  MESSAGE_INTERVAL();}
            public static EXTENDED_SYS_STATE new_EXTENDED_SYS_STATE() {return new  EXTENDED_SYS_STATE();}
            public static ADSB_VEHICLE new_ADSB_VEHICLE() {return new  ADSB_VEHICLE();}
            public static COLLISION new_COLLISION() {return new  COLLISION();}
            public static V2_EXTENSION new_V2_EXTENSION() {return new  V2_EXTENSION();}
            public static MEMORY_VECT new_MEMORY_VECT() {return new  MEMORY_VECT();}
            public static DEBUG_VECT new_DEBUG_VECT() {return new  DEBUG_VECT();}
            public static NAMED_VALUE_FLOAT new_NAMED_VALUE_FLOAT() {return new  NAMED_VALUE_FLOAT();}
            public static NAMED_VALUE_INT new_NAMED_VALUE_INT() {return new  NAMED_VALUE_INT();}
            public static STATUSTEXT new_STATUSTEXT() {return new  STATUSTEXT();}
            public static DEBUG new_DEBUG() {return new  DEBUG();}
            public static SETUP_SIGNING new_SETUP_SIGNING() {return new  SETUP_SIGNING();}
            public static BUTTON_CHANGE new_BUTTON_CHANGE() {return new  BUTTON_CHANGE();}
            public static PLAY_TUNE new_PLAY_TUNE() {return new  PLAY_TUNE();}
            public static CAMERA_INFORMATION new_CAMERA_INFORMATION() {return new  CAMERA_INFORMATION();}
            public static CAMERA_SETTINGS new_CAMERA_SETTINGS() {return new  CAMERA_SETTINGS();}
            public static STORAGE_INFORMATION new_STORAGE_INFORMATION() {return new  STORAGE_INFORMATION();}
            public static CAMERA_CAPTURE_STATUS new_CAMERA_CAPTURE_STATUS() {return new  CAMERA_CAPTURE_STATUS();}
            public static CAMERA_IMAGE_CAPTURED new_CAMERA_IMAGE_CAPTURED() {return new  CAMERA_IMAGE_CAPTURED();}
            public static FLIGHT_INFORMATION new_FLIGHT_INFORMATION() {return new  FLIGHT_INFORMATION();}
            public static MOUNT_ORIENTATION new_MOUNT_ORIENTATION() {return new  MOUNT_ORIENTATION();}
            public static LOGGING_DATA new_LOGGING_DATA() {return new  LOGGING_DATA();}
            public static LOGGING_DATA_ACKED new_LOGGING_DATA_ACKED() {return new  LOGGING_DATA_ACKED();}
            public static LOGGING_ACK new_LOGGING_ACK() {return new  LOGGING_ACK();}
            public static VIDEO_STREAM_INFORMATION new_VIDEO_STREAM_INFORMATION() {return new  VIDEO_STREAM_INFORMATION();}
            public static SET_VIDEO_STREAM_SETTINGS new_SET_VIDEO_STREAM_SETTINGS() {return new  SET_VIDEO_STREAM_SETTINGS();}
            public static WIFI_CONFIG_AP new_WIFI_CONFIG_AP() {return new  WIFI_CONFIG_AP();}
            public static PROTOCOL_VERSION new_PROTOCOL_VERSION() {return new  PROTOCOL_VERSION();}
            public static UAVCAN_NODE_STATUS new_UAVCAN_NODE_STATUS() {return new  UAVCAN_NODE_STATUS();}
            public static UAVCAN_NODE_INFO new_UAVCAN_NODE_INFO() {return new  UAVCAN_NODE_INFO();}
            public static PARAM_EXT_REQUEST_READ new_PARAM_EXT_REQUEST_READ() {return new  PARAM_EXT_REQUEST_READ();}
            public static PARAM_EXT_REQUEST_LIST new_PARAM_EXT_REQUEST_LIST() {return new  PARAM_EXT_REQUEST_LIST();}
            public static PARAM_EXT_VALUE new_PARAM_EXT_VALUE() {return new  PARAM_EXT_VALUE();}
            public static PARAM_EXT_SET new_PARAM_EXT_SET() {return new  PARAM_EXT_SET();}
            public static PARAM_EXT_ACK new_PARAM_EXT_ACK() {return new  PARAM_EXT_ACK();}
            public static OBSTACLE_DISTANCE new_OBSTACLE_DISTANCE() {return new  OBSTACLE_DISTANCE();}
            public static UAVIONIX_ADSB_OUT_CFG new_UAVIONIX_ADSB_OUT_CFG() {return new  UAVIONIX_ADSB_OUT_CFG();}
            public static UAVIONIX_ADSB_OUT_DYNAMIC new_UAVIONIX_ADSB_OUT_DYNAMIC() {return new  UAVIONIX_ADSB_OUT_DYNAMIC();}
            public static UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT new_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT() {return new  UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT();}

            public void send(Sendable pack) {lock(sendout_packs) {sendout_packs.Enqueue((Pack) pack); Monitor.PulseAll(sendout_packs);}}
            protected readonly Queue<Pack> sendout_packs = new Queue<Pack>();
            protected internal override Pack process(Pack pack, int id)
            {
                switch(id)
                {
                    default:
                        Debug.Assert(false);
                        return null;
                    case Channel.PROCESS_CHANNEL_REQEST:
                        if(pack == null) return sendout_packs.Count == 0 ? null : sendout_packs.Dequeue();
                        return null;
                    case Channel.PROCESS_HOST_REQEST:
                        lock(sendout_packs) { sendout_packs.Enqueue(pack); Monitor.PulseAll(sendout_packs); }
                        return null;
                }
            }
        }
        ; /*
				                                              */

        public enum MAV_TYPE
        {
            MAV_TYPE_GENERIC = 0,
            MAV_TYPE_FIXED_WING = 1,
            MAV_TYPE_QUADROTOR = 2,
            MAV_TYPE_COAXIAL = 3,
            MAV_TYPE_HELICOPTER = 4,
            MAV_TYPE_ANTENNA_TRACKER = 5,
            MAV_TYPE_GCS = 6,
            MAV_TYPE_AIRSHIP = 7,
            MAV_TYPE_FREE_BALLOON = 8,
            MAV_TYPE_ROCKET = 9,
            MAV_TYPE_GROUND_ROVER = 10,
            MAV_TYPE_SURFACE_BOAT = 11,
            MAV_TYPE_SUBMARINE = 12,
            MAV_TYPE_HEXAROTOR = 13,
            MAV_TYPE_OCTOROTOR = 14,
            MAV_TYPE_TRICOPTER = 15,
            MAV_TYPE_FLAPPING_WING = 16,
            MAV_TYPE_KITE = 17,
            MAV_TYPE_ONBOARD_CONTROLLER = 18,
            MAV_TYPE_VTOL_DUOROTOR = 19,
            MAV_TYPE_VTOL_QUADROTOR = 20,
            MAV_TYPE_VTOL_TILTROTOR = 21,
            MAV_TYPE_VTOL_RESERVED2 = 22,
            MAV_TYPE_VTOL_RESERVED3 = 23,
            MAV_TYPE_VTOL_RESERVED4 = 24,
            MAV_TYPE_VTOL_RESERVED5 = 25,
            MAV_TYPE_GIMBAL = 26,
            MAV_TYPE_ADSB = 27,
            MAV_TYPE_PARAFOIL = 28
        }
        ; /*
				                                              */

        public enum MAV_AUTOPILOT
        {
            MAV_AUTOPILOT_GENERIC = 0,
            MAV_AUTOPILOT_RESERVED = 1,
            MAV_AUTOPILOT_SLUGS = 2,
            MAV_AUTOPILOT_ARDUPILOTMEGA = 3,
            MAV_AUTOPILOT_OPENPILOT = 4,
            MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY = 5,
            MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY = 6,
            MAV_AUTOPILOT_GENERIC_MISSION_FULL = 7,
            MAV_AUTOPILOT_INVALID = 8,
            MAV_AUTOPILOT_PPZ = 9,
            MAV_AUTOPILOT_UDB = 10,
            MAV_AUTOPILOT_FP = 11,
            MAV_AUTOPILOT_PX4 = 12,
            MAV_AUTOPILOT_SMACCMPILOT = 13,
            MAV_AUTOPILOT_AUTOQUAD = 14,
            MAV_AUTOPILOT_ARMAZILA = 15,
            MAV_AUTOPILOT_AEROB = 16,
            MAV_AUTOPILOT_ASLUAV = 17,
            MAV_AUTOPILOT_SMARTAP = 18
        }
        ; /*
				                                              */

        public enum MAV_MODE_FLAG
        {
            MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1,
            MAV_MODE_FLAG_TEST_ENABLED = 2,
            MAV_MODE_FLAG_AUTO_ENABLED = 4,
            MAV_MODE_FLAG_GUIDED_ENABLED = 8,
            MAV_MODE_FLAG_STABILIZE_ENABLED = 16,
            MAV_MODE_FLAG_HIL_ENABLED = 32,
            MAV_MODE_FLAG_MANUAL_INPUT_ENABLED = 64,
            MAV_MODE_FLAG_SAFETY_ARMED = 128
        }
        ; /*
				                                              */

        public enum MAV_STATE
        {
            MAV_STATE_UNINIT = 0,
            MAV_STATE_BOOT = 1,
            MAV_STATE_CALIBRATING = 2,
            MAV_STATE_STANDBY = 3,
            MAV_STATE_ACTIVE = 4,
            MAV_STATE_CRITICAL = 5,
            MAV_STATE_EMERGENCY = 6,
            MAV_STATE_POWEROFF = 7,
            MAV_STATE_FLIGHT_TERMINATION = 8
        }
        ; /*
				                                              */

        public enum MAV_SYS_STATUS_SENSOR
        {
            MAV_SYS_STATUS_SENSOR_3D_GYRO = 1,
            MAV_SYS_STATUS_SENSOR_3D_ACCEL = 2,
            MAV_SYS_STATUS_SENSOR_3D_MAG = 4,
            MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE = 8,
            MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE = 16,
            MAV_SYS_STATUS_SENSOR_GPS = 32,
            MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW = 64,
            MAV_SYS_STATUS_SENSOR_VISION_POSITION = 128,
            MAV_SYS_STATUS_SENSOR_LASER_POSITION = 256,
            MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH = 512,
            MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL = 1024,
            MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION = 2048,
            MAV_SYS_STATUS_SENSOR_YAW_POSITION = 4096,
            MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL = 8192,
            MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL = 16384,
            MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS = 32768,
            MAV_SYS_STATUS_SENSOR_RC_RECEIVER = 65536,
            MAV_SYS_STATUS_SENSOR_3D_GYRO2 = 131072,
            MAV_SYS_STATUS_SENSOR_3D_ACCEL2 = 262144,
            MAV_SYS_STATUS_SENSOR_3D_MAG2 = 524288,
            MAV_SYS_STATUS_GEOFENCE = 1048576,
            MAV_SYS_STATUS_AHRS = 2097152,
            MAV_SYS_STATUS_TERRAIN = 4194304,
            MAV_SYS_STATUS_REVERSE_MOTOR = 8388608,
            MAV_SYS_STATUS_LOGGING = 16777216,
            MAV_SYS_STATUS_SENSOR_BATTERY = 33554432
        }
        ; /*
				                                              */

        public enum MAV_FRAME
        {
            MAV_FRAME_GLOBAL = 0,
            MAV_FRAME_LOCAL_NED = 1,
            MAV_FRAME_MISSION = 2,
            MAV_FRAME_GLOBAL_RELATIVE_ALT = 3,
            MAV_FRAME_LOCAL_ENU = 4,
            MAV_FRAME_GLOBAL_INT = 5,
            MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6,
            MAV_FRAME_LOCAL_OFFSET_NED = 7,
            MAV_FRAME_BODY_NED = 8,
            MAV_FRAME_BODY_OFFSET_NED = 9,
            MAV_FRAME_GLOBAL_TERRAIN_ALT = 10,
            MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11
        }
        ; /*
				                                              */

        public enum MAV_MODE
        {
            MAV_MODE_PREFLIGHT = 0,
            MAV_MODE_MANUAL_DISARMED = 64,
            MAV_MODE_TEST_DISARMED = 66,
            MAV_MODE_STABILIZE_DISARMED = 80,
            MAV_MODE_GUIDED_DISARMED = 88,
            MAV_MODE_AUTO_DISARMED = 92,
            MAV_MODE_MANUAL_ARMED = 192,
            MAV_MODE_TEST_ARMED = 194,
            MAV_MODE_STABILIZE_ARMED = 208,
            MAV_MODE_GUIDED_ARMED = 216,
            MAV_MODE_AUTO_ARMED = 220
        }
        ; /*
				                                              */

        public enum MAV_PARAM_TYPE
        {
            MAV_PARAM_TYPE_UINT8 = 1,
            MAV_PARAM_TYPE_INT8 = 2,
            MAV_PARAM_TYPE_UINT16 = 3,
            MAV_PARAM_TYPE_INT16 = 4,
            MAV_PARAM_TYPE_UINT32 = 5,
            MAV_PARAM_TYPE_INT32 = 6,
            MAV_PARAM_TYPE_UINT64 = 7,
            MAV_PARAM_TYPE_INT64 = 8,
            MAV_PARAM_TYPE_REAL32 = 9,
            MAV_PARAM_TYPE_REAL64 = 10
        }
        ; /*
				                                              */

        public enum GPS_FIX_TYPE
        {
            GPS_FIX_TYPE_NO_GPS = 0,
            GPS_FIX_TYPE_NO_FIX = 1,
            GPS_FIX_TYPE_2D_FIX = 2,
            GPS_FIX_TYPE_3D_FIX = 3,
            GPS_FIX_TYPE_DGPS = 4,
            GPS_FIX_TYPE_RTK_FLOAT = 5,
            GPS_FIX_TYPE_RTK_FIXED = 6,
            GPS_FIX_TYPE_STATIC = 7,
            GPS_FIX_TYPE_PPP = 8
        }
        ; /*
				                                              */

        public enum MAV_MISSION_TYPE
        {
            MAV_MISSION_TYPE_MISSION = 0,
            MAV_MISSION_TYPE_FENCE = 1,
            MAV_MISSION_TYPE_RALLY = 2,
            MAV_MISSION_TYPE_ALL = 255
        }
        ; /*
				                                              */

        public enum MAV_CMD
        {
            MAV_CMD_NAV_WAYPOINT = 16,
            MAV_CMD_NAV_LOITER_UNLIM = 17,
            MAV_CMD_NAV_LOITER_TURNS = 18,
            MAV_CMD_NAV_LOITER_TIME = 19,
            MAV_CMD_NAV_RETURN_TO_LAUNCH = 20,
            MAV_CMD_NAV_LAND = 21,
            MAV_CMD_NAV_TAKEOFF = 22,
            MAV_CMD_NAV_LAND_LOCAL = 23,
            MAV_CMD_NAV_TAKEOFF_LOCAL = 24,
            MAV_CMD_NAV_FOLLOW = 25,
            MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT = 30,
            MAV_CMD_NAV_LOITER_TO_ALT = 31,
            MAV_CMD_DO_FOLLOW = 32,
            MAV_CMD_DO_FOLLOW_REPOSITION = 33,
            MAV_CMD_NAV_ROI = 80,
            MAV_CMD_NAV_PATHPLANNING = 81,
            MAV_CMD_NAV_SPLINE_WAYPOINT = 82,
            MAV_CMD_NAV_VTOL_TAKEOFF = 84,
            MAV_CMD_NAV_VTOL_LAND = 85,
            MAV_CMD_NAV_GUIDED_ENABLE = 92,
            MAV_CMD_NAV_DELAY = 93,
            MAV_CMD_NAV_PAYLOAD_PLACE = 94,
            MAV_CMD_NAV_LAST = 95,
            MAV_CMD_CONDITION_DELAY = 112,
            MAV_CMD_CONDITION_CHANGE_ALT = 113,
            MAV_CMD_CONDITION_DISTANCE = 114,
            MAV_CMD_CONDITION_YAW = 115,
            MAV_CMD_CONDITION_LAST = 159,
            MAV_CMD_DO_SET_MODE = 176,
            MAV_CMD_DO_JUMP = 177,
            MAV_CMD_DO_CHANGE_SPEED = 178,
            MAV_CMD_DO_SET_HOME = 179,
            MAV_CMD_DO_SET_PARAMETER = 180,
            MAV_CMD_DO_SET_RELAY = 181,
            MAV_CMD_DO_REPEAT_RELAY = 182,
            MAV_CMD_DO_SET_SERVO = 183,
            MAV_CMD_DO_REPEAT_SERVO = 184,
            MAV_CMD_DO_FLIGHTTERMINATION = 185,
            MAV_CMD_DO_CHANGE_ALTITUDE = 186,
            MAV_CMD_DO_LAND_START = 189,
            MAV_CMD_DO_RALLY_LAND = 190,
            MAV_CMD_DO_GO_AROUND = 191,
            MAV_CMD_DO_REPOSITION = 192,
            MAV_CMD_DO_PAUSE_CONTINUE = 193,
            MAV_CMD_DO_SET_REVERSE = 194,
            MAV_CMD_DO_CONTROL_VIDEO = 200,
            MAV_CMD_DO_SET_ROI = 201,
            MAV_CMD_DO_DIGICAM_CONFIGURE = 202,
            MAV_CMD_DO_DIGICAM_CONTROL = 203,
            MAV_CMD_DO_MOUNT_CONFIGURE = 204,
            MAV_CMD_DO_MOUNT_CONTROL = 205,
            MAV_CMD_DO_SET_CAM_TRIGG_DIST = 206,
            MAV_CMD_DO_FENCE_ENABLE = 207,
            MAV_CMD_DO_PARACHUTE = 208,
            MAV_CMD_DO_MOTOR_TEST = 209,
            MAV_CMD_DO_INVERTED_FLIGHT = 210,
            MAV_CMD_NAV_SET_YAW_SPEED = 213,
            MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL = 214,
            MAV_CMD_DO_MOUNT_CONTROL_QUAT = 220,
            MAV_CMD_DO_GUIDED_MASTER = 221,
            MAV_CMD_DO_GUIDED_LIMITS = 222,
            MAV_CMD_DO_ENGINE_CONTROL = 223,
            MAV_CMD_DO_LAST = 240,
            MAV_CMD_PREFLIGHT_CALIBRATION = 241,
            MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS = 242,
            MAV_CMD_PREFLIGHT_UAVCAN = 243,
            MAV_CMD_PREFLIGHT_STORAGE = 245,
            MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246,
            MAV_CMD_OVERRIDE_GOTO = 252,
            MAV_CMD_MISSION_START = 300,
            MAV_CMD_COMPONENT_ARM_DISARM = 400,
            MAV_CMD_GET_HOME_POSITION = 410,
            MAV_CMD_START_RX_PAIR = 500,
            MAV_CMD_GET_MESSAGE_INTERVAL = 510,
            MAV_CMD_SET_MESSAGE_INTERVAL = 511,
            MAV_CMD_REQUEST_PROTOCOL_VERSION = 519,
            MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES = 520,
            MAV_CMD_REQUEST_CAMERA_INFORMATION = 521,
            MAV_CMD_REQUEST_CAMERA_SETTINGS = 522,
            MAV_CMD_REQUEST_STORAGE_INFORMATION = 525,
            MAV_CMD_STORAGE_FORMAT = 526,
            MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS = 527,
            MAV_CMD_REQUEST_FLIGHT_INFORMATION = 528,
            MAV_CMD_RESET_CAMERA_SETTINGS = 529,
            MAV_CMD_SET_CAMERA_MODE = 530,
            MAV_CMD_IMAGE_START_CAPTURE = 2000,
            MAV_CMD_IMAGE_STOP_CAPTURE = 2001,
            MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE = 2002,
            MAV_CMD_DO_TRIGGER_CONTROL = 2003,
            MAV_CMD_VIDEO_START_CAPTURE = 2500,
            MAV_CMD_VIDEO_STOP_CAPTURE = 2501,
            MAV_CMD_VIDEO_START_STREAMING = 2502,
            MAV_CMD_VIDEO_STOP_STREAMING = 2503,
            MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION = 2504,
            MAV_CMD_LOGGING_START = 2510,
            MAV_CMD_LOGGING_STOP = 2511,
            MAV_CMD_AIRFRAME_CONFIGURATION = 2520,
            MAV_CMD_PANORAMA_CREATE = 2800,
            MAV_CMD_DO_VTOL_TRANSITION = 3000,
            MAV_CMD_ARM_AUTHORIZATION_REQUEST = 3001,
            MAV_CMD_SET_GUIDED_SUBMODE_STANDARD = 4000,
            MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE = 4001,
            MAV_CMD_CONDITION_GATE = 4501,
            MAV_CMD_NAV_FENCE_RETURN_POINT = 5000,
            MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION = 5001,
            MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION = 5002,
            MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION = 5003,
            MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION = 5004,
            MAV_CMD_NAV_RALLY_POINT = 5100,
            MAV_CMD_UAVCAN_GET_NODE_INFO = 5200,
            MAV_CMD_PAYLOAD_PREPARE_DEPLOY = 30001,
            MAV_CMD_PAYLOAD_CONTROL_DEPLOY = 30002,
            MAV_CMD_WAYPOINT_USER_1 = 31000,
            MAV_CMD_WAYPOINT_USER_2 = 31001,
            MAV_CMD_WAYPOINT_USER_3 = 31002,
            MAV_CMD_WAYPOINT_USER_4 = 31003,
            MAV_CMD_WAYPOINT_USER_5 = 31004,
            MAV_CMD_SPATIAL_USER_1 = 31005,
            MAV_CMD_SPATIAL_USER_2 = 31006,
            MAV_CMD_SPATIAL_USER_3 = 31007,
            MAV_CMD_SPATIAL_USER_4 = 31008,
            MAV_CMD_SPATIAL_USER_5 = 31009,
            MAV_CMD_USER_1 = 31010,
            MAV_CMD_USER_2 = 31011,
            MAV_CMD_USER_3 = 31012,
            MAV_CMD_USER_4 = 31013,
            MAV_CMD_USER_5 = 31014
        }
        ; /*
				                                              */

        public enum MAV_MISSION_RESULT
        {
            MAV_MISSION_ACCEPTED = 0,
            MAV_MISSION_ERROR = 1,
            MAV_MISSION_UNSUPPORTED_FRAME = 2,
            MAV_MISSION_UNSUPPORTED = 3,
            MAV_MISSION_NO_SPACE = 4,
            MAV_MISSION_INVALID = 5,
            MAV_MISSION_INVALID_PARAM1 = 6,
            MAV_MISSION_INVALID_PARAM2 = 7,
            MAV_MISSION_INVALID_PARAM3 = 8,
            MAV_MISSION_INVALID_PARAM4 = 9,
            MAV_MISSION_INVALID_PARAM5_X = 10,
            MAV_MISSION_INVALID_PARAM6_Y = 11,
            MAV_MISSION_INVALID_PARAM7 = 12,
            MAV_MISSION_INVALID_SEQUENCE = 13,
            MAV_MISSION_DENIED = 14
        }
        ; /*
				                                              */

        public enum MAV_ESTIMATOR_TYPE
        {
            MAV_ESTIMATOR_TYPE_NAIVE = 1,
            MAV_ESTIMATOR_TYPE_VISION = 2,
            MAV_ESTIMATOR_TYPE_VIO = 3,
            MAV_ESTIMATOR_TYPE_GPS = 4,
            MAV_ESTIMATOR_TYPE_GPS_INS = 5
        }
        ; /*
				                                              */

        public enum MAV_RESULT
        {
            MAV_RESULT_ACCEPTED = 0,
            MAV_RESULT_TEMPORARILY_REJECTED = 1,
            MAV_RESULT_DENIED = 2,
            MAV_RESULT_UNSUPPORTED = 3,
            MAV_RESULT_FAILED = 4,
            MAV_RESULT_IN_PROGRESS = 5
        }
        ; /*
				                                              */

        public enum MAV_POWER_STATUS
        {
            MAV_POWER_STATUS_BRICK_VALID = 1,
            MAV_POWER_STATUS_SERVO_VALID = 2,
            MAV_POWER_STATUS_USB_CONNECTED = 4,
            MAV_POWER_STATUS_PERIPH_OVERCURRENT = 8,
            MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT = 16,
            MAV_POWER_STATUS_CHANGED = 32
        }
        ; /*
				                                              */

        public enum SERIAL_CONTROL_DEV
        {
            SERIAL_CONTROL_DEV_TELEM1 = 0,
            SERIAL_CONTROL_DEV_TELEM2 = 1,
            SERIAL_CONTROL_DEV_GPS1 = 2,
            SERIAL_CONTROL_DEV_GPS2 = 3,
            SERIAL_CONTROL_DEV_SHELL = 10
        }
        ; /*
				                                              */

        public enum SERIAL_CONTROL_FLAG
        {
            SERIAL_CONTROL_FLAG_REPLY = 1,
            SERIAL_CONTROL_FLAG_RESPOND = 2,
            SERIAL_CONTROL_FLAG_EXCLUSIVE = 4,
            SERIAL_CONTROL_FLAG_BLOCKING = 8,
            SERIAL_CONTROL_FLAG_MULTI = 16
        }
        ; /*
				                                              */

        public enum MAV_DISTANCE_SENSOR
        {
            MAV_DISTANCE_SENSOR_LASER = 0,
            MAV_DISTANCE_SENSOR_ULTRASOUND = 1,
            MAV_DISTANCE_SENSOR_INFRARED = 2,
            MAV_DISTANCE_SENSOR_RADAR = 3,
            MAV_DISTANCE_SENSOR_UNKNOWN = 4
        }
        ; /*
				                                              */

        public enum MAV_SENSOR_ORIENTATION
        {
            MAV_SENSOR_ROTATION_NONE = 0,
            MAV_SENSOR_ROTATION_YAW_45 = 1,
            MAV_SENSOR_ROTATION_YAW_90 = 2,
            MAV_SENSOR_ROTATION_YAW_135 = 3,
            MAV_SENSOR_ROTATION_YAW_180 = 4,
            MAV_SENSOR_ROTATION_YAW_225 = 5,
            MAV_SENSOR_ROTATION_YAW_270 = 6,
            MAV_SENSOR_ROTATION_YAW_315 = 7,
            MAV_SENSOR_ROTATION_ROLL_180 = 8,
            MAV_SENSOR_ROTATION_ROLL_180_YAW_45 = 9,
            MAV_SENSOR_ROTATION_ROLL_180_YAW_90 = 10,
            MAV_SENSOR_ROTATION_ROLL_180_YAW_135 = 11,
            MAV_SENSOR_ROTATION_PITCH_180 = 12,
            MAV_SENSOR_ROTATION_ROLL_180_YAW_225 = 13,
            MAV_SENSOR_ROTATION_ROLL_180_YAW_270 = 14,
            MAV_SENSOR_ROTATION_ROLL_180_YAW_315 = 15,
            MAV_SENSOR_ROTATION_ROLL_90 = 16,
            MAV_SENSOR_ROTATION_ROLL_90_YAW_45 = 17,
            MAV_SENSOR_ROTATION_ROLL_90_YAW_90 = 18,
            MAV_SENSOR_ROTATION_ROLL_90_YAW_135 = 19,
            MAV_SENSOR_ROTATION_ROLL_270 = 20,
            MAV_SENSOR_ROTATION_ROLL_270_YAW_45 = 21,
            MAV_SENSOR_ROTATION_ROLL_270_YAW_90 = 22,
            MAV_SENSOR_ROTATION_ROLL_270_YAW_135 = 23,
            MAV_SENSOR_ROTATION_PITCH_90 = 24,
            MAV_SENSOR_ROTATION_PITCH_270 = 25,
            MAV_SENSOR_ROTATION_PITCH_180_YAW_90 = 26,
            MAV_SENSOR_ROTATION_PITCH_180_YAW_270 = 27,
            MAV_SENSOR_ROTATION_ROLL_90_PITCH_90 = 28,
            MAV_SENSOR_ROTATION_ROLL_180_PITCH_90 = 29,
            MAV_SENSOR_ROTATION_ROLL_270_PITCH_90 = 30,
            MAV_SENSOR_ROTATION_ROLL_90_PITCH_180 = 31,
            MAV_SENSOR_ROTATION_ROLL_270_PITCH_180 = 32,
            MAV_SENSOR_ROTATION_ROLL_90_PITCH_270 = 33,
            MAV_SENSOR_ROTATION_ROLL_180_PITCH_270 = 34,
            MAV_SENSOR_ROTATION_ROLL_270_PITCH_270 = 35,
            MAV_SENSOR_ROTATION_ROLL_90_PITCH_180_YAW_90 = 36,
            MAV_SENSOR_ROTATION_ROLL_90_YAW_270 = 37,
            MAV_SENSOR_ROTATION_ROLL_315_PITCH_315_YAW_315 = 38
        }
        ; /*
				                                              */

        public enum MAV_BATTERY_FUNCTION
        {
            MAV_BATTERY_FUNCTION_UNKNOWN = 0,
            MAV_BATTERY_FUNCTION_ALL = 1,
            MAV_BATTERY_FUNCTION_PROPULSION = 2,
            MAV_BATTERY_FUNCTION_AVIONICS = 3,
            MAV_BATTERY_TYPE_PAYLOAD = 4
        }
        ; /*
				                                              */

        public enum MAV_BATTERY_TYPE
        {
            MAV_BATTERY_TYPE_UNKNOWN = 0,
            MAV_BATTERY_TYPE_LIPO = 1,
            MAV_BATTERY_TYPE_LIFE = 2,
            MAV_BATTERY_TYPE_LION = 3,
            MAV_BATTERY_TYPE_NIMH = 4
        }
        ; /*
				                                              */

        public enum MAV_PROTOCOL_CAPABILITY
        {
            MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT = 1,
            MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT = 2,
            MAV_PROTOCOL_CAPABILITY_MISSION_INT = 4,
            MAV_PROTOCOL_CAPABILITY_COMMAND_INT = 8,
            MAV_PROTOCOL_CAPABILITY_PARAM_UNION = 16,
            MAV_PROTOCOL_CAPABILITY_FTP = 32,
            MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET = 64,
            MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED = 128,
            MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT = 256,
            MAV_PROTOCOL_CAPABILITY_TERRAIN = 512,
            MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET = 1024,
            MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION = 2048,
            MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION = 4096,
            MAV_PROTOCOL_CAPABILITY_MAVLINK2 = 8192,
            MAV_PROTOCOL_CAPABILITY_MISSION_FENCE = 16384,
            MAV_PROTOCOL_CAPABILITY_MISSION_RALLY = 32768,
            MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION = 65536
        }
        ; /*
				                                              */

        public enum LANDING_TARGET_TYPE
        {
            LANDING_TARGET_TYPE_LIGHT_BEACON = 0,
            LANDING_TARGET_TYPE_RADIO_BEACON = 1,
            LANDING_TARGET_TYPE_VISION_FIDUCIAL = 2,
            LANDING_TARGET_TYPE_VISION_OTHER = 3
        }
        ; /*
				                                              */

        public enum ESTIMATOR_STATUS_FLAGS
        {
            ESTIMATOR_ATTITUDE = 1,
            ESTIMATOR_VELOCITY_HORIZ = 2,
            ESTIMATOR_VELOCITY_VERT = 4,
            ESTIMATOR_POS_HORIZ_REL = 8,
            ESTIMATOR_POS_HORIZ_ABS = 16,
            ESTIMATOR_POS_VERT_ABS = 32,
            ESTIMATOR_POS_VERT_AGL = 64,
            ESTIMATOR_CONST_POS_MODE = 128,
            ESTIMATOR_PRED_POS_HORIZ_REL = 256,
            ESTIMATOR_PRED_POS_HORIZ_ABS = 512,
            ESTIMATOR_GPS_GLITCH = 1024
        }
        ; /*
				                                              */

        public enum GPS_INPUT_IGNORE_FLAGS
        {
            GPS_INPUT_IGNORE_FLAG_ALT = 1,
            GPS_INPUT_IGNORE_FLAG_HDOP = 2,
            GPS_INPUT_IGNORE_FLAG_VDOP = 4,
            GPS_INPUT_IGNORE_FLAG_VEL_HORIZ = 8,
            GPS_INPUT_IGNORE_FLAG_VEL_VERT = 16,
            GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY = 32,
            GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY = 64,
            GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY = 128
        }
        ; /*
				                                              */

        public enum MAV_LANDED_STATE
        {
            MAV_LANDED_STATE_UNDEFINED = 0,
            MAV_LANDED_STATE_ON_GROUND = 1,
            MAV_LANDED_STATE_IN_AIR = 2,
            MAV_LANDED_STATE_TAKEOFF = 3,
            MAV_LANDED_STATE_LANDING = 4
        }
        ; /*
				                                              */

        public enum MAV_VTOL_STATE
        {
            MAV_VTOL_STATE_UNDEFINED = 0,
            MAV_VTOL_STATE_TRANSITION_TO_FW = 1,
            MAV_VTOL_STATE_TRANSITION_TO_MC = 2,
            MAV_VTOL_STATE_MC = 3,
            MAV_VTOL_STATE_FW = 4
        }
        ; /*
				                                              */

        public enum ADSB_ALTITUDE_TYPE
        {
            ADSB_ALTITUDE_TYPE_PRESSURE_QNH = 0,
            ADSB_ALTITUDE_TYPE_GEOMETRIC = 1
        }
        ; /*
				                                              */

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
        ; /*
				                                              */

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
        ; /*
				                                              */

        public enum MAV_COLLISION_SRC
        {
            MAV_COLLISION_SRC_ADSB = 0,
            MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT = 1
        }
        ; /*
				                                              */

        public enum MAV_COLLISION_ACTION
        {
            MAV_COLLISION_ACTION_NONE = 0,
            MAV_COLLISION_ACTION_REPORT = 1,
            MAV_COLLISION_ACTION_ASCEND_OR_DESCEND = 2,
            MAV_COLLISION_ACTION_MOVE_HORIZONTALLY = 3,
            MAV_COLLISION_ACTION_MOVE_PERPENDICULAR = 4,
            MAV_COLLISION_ACTION_RTL = 5,
            MAV_COLLISION_ACTION_HOVER = 6
        }
        ; /*
				                                              */

        public enum MAV_COLLISION_THREAT_LEVEL
        {
            MAV_COLLISION_THREAT_LEVEL_NONE = 0,
            MAV_COLLISION_THREAT_LEVEL_LOW = 1,
            MAV_COLLISION_THREAT_LEVEL_HIGH = 2
        }
        ; /*
				                                              */

        public enum MAV_SEVERITY
        {
            MAV_SEVERITY_EMERGENCY = 0,
            MAV_SEVERITY_ALERT = 1,
            MAV_SEVERITY_CRITICAL = 2,
            MAV_SEVERITY_ERROR = 3,
            MAV_SEVERITY_WARNING = 4,
            MAV_SEVERITY_NOTICE = 5,
            MAV_SEVERITY_INFO = 6,
            MAV_SEVERITY_DEBUG = 7
        }
        ; /*
				                                              */

        public enum CAMERA_CAP_FLAGS
        {
            CAMERA_CAP_FLAGS_CAPTURE_VIDEO = 1,
            CAMERA_CAP_FLAGS_CAPTURE_IMAGE = 2,
            CAMERA_CAP_FLAGS_HAS_MODES = 4,
            CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE = 8,
            CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE = 16,
            CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE = 32
        }
        ; /*
				                                              */

        public enum CAMERA_MODE
        {
            CAMERA_MODE_IMAGE = 0,
            CAMERA_MODE_VIDEO = 1,
            CAMERA_MODE_IMAGE_SURVEY = 2
        }
        ; /*
				                                              */

        public enum UAVCAN_NODE_HEALTH
        {
            UAVCAN_NODE_HEALTH_OK = 0,
            UAVCAN_NODE_HEALTH_WARNING = 1,
            UAVCAN_NODE_HEALTH_ERROR = 2,
            UAVCAN_NODE_HEALTH_CRITICAL = 3
        }
        ; /*
				                                              */

        public enum UAVCAN_NODE_MODE
        {
            UAVCAN_NODE_MODE_OPERATIONAL = 0,
            UAVCAN_NODE_MODE_INITIALIZATION = 1,
            UAVCAN_NODE_MODE_MAINTENANCE = 2,
            UAVCAN_NODE_MODE_SOFTWARE_UPDATE = 3,
            UAVCAN_NODE_MODE_OFFLINE = 7
        }
        ; /*
				                                              */

        public enum MAV_PARAM_EXT_TYPE
        {
            MAV_PARAM_EXT_TYPE_UINT8 = 1,
            MAV_PARAM_EXT_TYPE_INT8 = 2,
            MAV_PARAM_EXT_TYPE_UINT16 = 3,
            MAV_PARAM_EXT_TYPE_INT16 = 4,
            MAV_PARAM_EXT_TYPE_UINT32 = 5,
            MAV_PARAM_EXT_TYPE_INT32 = 6,
            MAV_PARAM_EXT_TYPE_UINT64 = 7,
            MAV_PARAM_EXT_TYPE_INT64 = 8,
            MAV_PARAM_EXT_TYPE_REAL32 = 9,
            MAV_PARAM_EXT_TYPE_REAL64 = 10,
            MAV_PARAM_EXT_TYPE_CUSTOM = 11
        }
        ; /*
				                                              */

        public enum PARAM_ACK
        {
            PARAM_ACK_ACCEPTED = 0,
            PARAM_ACK_VALUE_UNSUPPORTED = 1,
            PARAM_ACK_FAILED = 2,
            PARAM_ACK_IN_PROGRESS = 3
        }
        ; /*
				                                              */

        public enum UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE
        {
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
            UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L85_W90M = 15
        }
        ; /*
				                                              */

        public enum UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT
        {
            UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_NO_DATA = 0,
            UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_LEFT_2M = 1,
            UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_LEFT_4M = 2,
            UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_LEFT_6M = 3,
            UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_0M = 4,
            UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_2M = 5,
            UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_4M = 6,
            UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_6M = 7
        }
        ; /*
				                                              */

        public enum UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON
        {
            UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_NO_DATA = 0,
            UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_APPLIED_BY_SENSOR = 1
        }
        ; /*
				                                              */

        public enum UAVIONIX_ADSB_OUT_RF_SELECT
        {
            UAVIONIX_ADSB_OUT_RF_SELECT_STANDBY = 0,
            UAVIONIX_ADSB_OUT_RF_SELECT_RX_ENABLED = 1,
            UAVIONIX_ADSB_OUT_RF_SELECT_TX_ENABLED = 2
        }
        ; /*
				                                              */

        public enum UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX
        {
            UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_NONE_0 = 0,
            UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_NONE_1 = 1,
            UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_2D = 2,
            UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_3D = 3,
            UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_DGPS = 4,
            UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_RTK = 5
        }
        ; /*
				                                              */

        public enum UAVIONIX_ADSB_EMERGENCY_STATUS
        {
            UAVIONIX_ADSB_OUT_NO_EMERGENCY = 0,
            UAVIONIX_ADSB_OUT_GENERAL_EMERGENCY = 1,
            UAVIONIX_ADSB_OUT_LIFEGUARD_EMERGENCY = 2,
            UAVIONIX_ADSB_OUT_MINIMUM_FUEL_EMERGENCY = 3,
            UAVIONIX_ADSB_OUT_NO_COMM_EMERGENCY = 4,
            UAVIONIX_ADSB_OUT_UNLAWFUL_INTERFERANCE_EMERGENCY = 5,
            UAVIONIX_ADSB_OUT_DOWNED_AIRCRAFT_EMERGENCY = 6,
            UAVIONIX_ADSB_OUT_RESERVED = 7
        }
        ; /*
				                                              */

        public enum UAVIONIX_ADSB_OUT_DYNAMIC_STATE
        {
            UAVIONIX_ADSB_OUT_DYNAMIC_STATE_INTENT_CHANGE = 1,
            UAVIONIX_ADSB_OUT_DYNAMIC_STATE_AUTOPILOT_ENABLED = 2,
            UAVIONIX_ADSB_OUT_DYNAMIC_STATE_NICBARO_CROSSCHECKED = 4,
            UAVIONIX_ADSB_OUT_DYNAMIC_STATE_ON_GROUND = 8,
            UAVIONIX_ADSB_OUT_DYNAMIC_STATE_IDENT = 16
        }
        ; /*
				                                              */

        public enum UAVIONIX_ADSB_RF_HEALTH
        {
            UAVIONIX_ADSB_RF_HEALTH_INITIALIZING = 0,
            UAVIONIX_ADSB_RF_HEALTH_OK = 1,
            UAVIONIX_ADSB_RF_HEALTH_FAIL_TX = 2,
            UAVIONIX_ADSB_RF_HEALTH_FAIL_RX = 16
        }

        static readonly Field _g = new Field(5, true, -5, 2, 1, 0, 0, 0, 1);
        static readonly Field _o = new Field(5, true, -6, 2, 1, 0, 0, 0, 1);
        static readonly Field _n = new Field(5, true, -5, 2, 1, 0, 0, 0, 1);
        static readonly Field _T = new Field(5, true, -5, 2, 1, 0, 0, 0, 1);
        static readonly Field _H = new Field(5, true, -5, 2, 1, 0, 0, 0, 1);
        static readonly Field _tA = new Field(0, false, 1, 4, 1, 0, 0, 0);
        static readonly Field _EA = new Field(0, true, 1, 4, 1, 0, 0, 0);
        static readonly Field _KA = new Field(0, true, 1, 4, 1, 0, 0, 0);
        static readonly Field _kA = new Field(0, true, 1, 4, 1, 0, 0, 0);
        static readonly Field _xA = new Field(0, true, 1, 4, 1, 0, 0, 0);
        static readonly Field _IJ = new Field(0, true, 1, 2, 1, 0, 0, 0);
        static readonly Field _tJ = new Field(0, true, 1, 2, 1, 0, 0, 0);
        static readonly Field _EJ = new Field(0, true, 1, 2, 1, 0, 0, 0);
        static readonly Field _KJ = new Field(0, true, 1, 2, 1, 0, 0, 0);
        static readonly Field _kJ = new Field(0, true, 1, 2, 1, 0, 0, 0);
        static readonly Field _xJ = new Field(0, true, 1, 2, 1, 0, 0, 0);
        static readonly Field _mJ = new Field(0, true, 1, 2, 1, 0, 0, 0);
        static readonly Field _rJ = new Field(0, true, 1, 2, 1, 0, 0, 0);
        static readonly Field _ib = new Field(0, true, 1, 8, 1, 0, 0, 0);
        static readonly Field _Lb = new Field(0, true, 1, 8, 1, 0, 0, 0);
        static readonly Field _hb = new Field(5, true, -5, 2, 1, 0, 0, 0, 1);
        static readonly Field _VF = new Field(0, false, 1, 1, 1, 0, 0, 0);
        static readonly Field _hF = new Field(0, false, 1, 4, 1, 0, 0, 0);
        static readonly Field _QF = new Field(0, false, 1, 1, 1, 0, 0, 0);
        static readonly Field _vF = new Field(0, false, 1, 1, 1, 0, 0, 0);
        static readonly Field _xt = new Field(0, false, 1, 4, 1, 0, 0, 0);
        static readonly Field _mt = new Field(0, false, 1, 4, 1, 0, 0, 0);
        static readonly Field _Gi = new Field(0, false, 18, 1, 1, 0, 0, 0);
        static readonly Field _xi = new Field(0, false, 1, 4, 1, 0, 0, 0);
        static readonly Field _mi = new Field(0, false, 1, 4, 1, 0, 0, 0);
        static readonly Field _ri = new Field(0, false, 1, 4, 1, 0, 0, 0);
        static readonly Field _ii = new Field(0, false, 4, 4, 1, 0, 0, 0);
        static readonly Field _Di = new Field(0, false, 1, 1, 1, 0, 0, 0);
        static readonly Field _HC = new Field(0, true, 1, 8, 1, 0, 0, 0);
        static readonly Field _ID = new Field(0, true, 1, 8, 1, 0, 0, 0);
        static readonly Field _zD = new Field(5, true, -4, 2, 1, 0, 0, 0, 1);
        static readonly Field _eD = new Field(5, true, -4, 2, 1, 0, 0, 0, 1);
        static readonly Field _HD = new Field(5, true, -4, 2, 1, 0, 0, 0, 1);
        static readonly Field _fM = new Field(5, true, -4, 2, 1, 0, 0, 0, 1);
        static readonly Field _JM = new Field(5, true, -6, 2, 1, 0, 0, 0, 1);
        static readonly Field _rM = new Field(5, true, -5, 2, 1, 0, 0, 0, 1);
        static readonly Field _ZM = new Field(5, true, -8, 2, 1, 0, 0, 0, 1);
        static readonly Field _GL = new Field(5, true, -8, 2, 1, 0, 0, 0, 1);
        static readonly Field _uL = new Field(5, true, -8, 2, 1, 0, 0, 0, 1);
        static readonly Field _yL = new Field(5, true, -8, 2, 1, 0, 0, 0, 1);
        static readonly Field _pL = new Field(5, true, -6, 2, 1, 0, 0, 0, 1);
        static readonly Field _qL = new Field(5, true, -7, 2, 1, 0, 0, 0, 1);
        static readonly Field _Iz = new Field(5, true, -7, 2, 1, 0, 0, 0, 1);
        static readonly Field _Cz = new Field(5, true, -5, 2, 1, 0, 0, 0, 1);
        static readonly Field _zz = new Field(5, true, -5, 2, 1, 0, 0, 0, 1);
        static readonly Field _Vz = new Field(5, true, -8, 2, 1, 0, 0, 0, 1);
        static readonly Field _Zz = new Field(5, true, -5, 2, 1, 0, 0, 0, 1);
        static readonly Field _Rz = new Field(5, true, -8, 2, 1, 0, 0, 0, 1);
        static readonly Field _oz = new Field(5, true, -5, 2, 1, 0, 0, 0, 1);
        static readonly Field _lz = new Field(5, true, -8, 2, 1, 0, 0, 0, 1);
        static readonly Field _ez = new Field(5, true, -4, 2, 1, 0, 0, 0, 1);

    }
}
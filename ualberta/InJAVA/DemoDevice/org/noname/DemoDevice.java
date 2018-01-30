package org.noname;
import java.util.*;
import org.unirail.BlackBox.Host;
import org.unirail.BlackBox.Host.Pack.Meta.Field;
import org.unirail.BlackBox.Host.Pack.Meta.Field.Bounds;
import static org.unirail.BlackBox.BitUtils.*;
import java.util.concurrent.ConcurrentLinkedQueue;


public class DemoDevice extends Host
{
    public static class HEARTBEAT extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        HEARTBEAT() { super(meta, 0); }
        HEARTBEAT(int bytes) { super(meta, bytes); }
        public long custom_mode_GET()//A bitfield for use for autopilot-specific flags
        {  return (get_bytes(data,  0, 4)); }
        public void custom_mode_SET(long  src) //A bitfield for use for autopilot-specific flags
        {  set_bytes((src) & -1L, 4, data,  0); }
        public char mavlink_version_GET()//MAVLink version, not writable by user, gets added by protocol because of magic data type: char_mavlink_versi
        {  return (char)((char) get_bytes(data,  4, 1)); }
        public void mavlink_version_SET(char  src) //MAVLink version, not writable by user, gets added by protocol because of magic data type: char_mavlink_versi
        {  set_bytes((char)(src) & -1L, 1, data,  4); }
        public @MAV_TYPE int type_GET()//Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM
        {  return  0 + (int)get_bits(data, 40, 5); }
        public void type_SET(@MAV_TYPE int  src) //Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM
        {  set_bits(- 0 +   src, 5, data, 40); }
        public @MAV_AUTOPILOT int autopilot_GET()//Autopilot type / class. defined in MAV_AUTOPILOT ENU
        {  return  0 + (int)get_bits(data, 45, 5); }
        public void autopilot_SET(@MAV_AUTOPILOT int  src) //Autopilot type / class. defined in MAV_AUTOPILOT ENU
        {  set_bits(- 0 +   src, 5, data, 45); }
        public @MAV_MODE_FLAG int base_mode_GET()//System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.
        {
            switch((int)get_bits(data, 50, 4))
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
            assert(false);//("Unknown enum ID " + id);
            return  Integer.MIN_VALUE;
        }
        public void base_mode_SET(@MAV_MODE_FLAG int  src) //System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.
        {
            long id = 0;
            switch(src)
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
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 4, data, 50);
        }
        public @MAV_STATE int system_status_GET()//System status flag, see MAV_STATE ENU
        {  return  0 + (int)get_bits(data, 54, 4); }
        public void system_status_SET(@MAV_STATE int  src) //System status flag, see MAV_STATE ENU
        {  set_bits(- 0 +   src, 4, data, 54); }
        static final Meta meta = new Meta(0, 0, 1, 0, 8, 58);
    } public static class SYS_STATUS extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        SYS_STATUS() { super(meta, 0); }
        SYS_STATUS(int bytes) { super(meta, bytes); }
        public char load_GET()//Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 100
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void load_SET(char  src) //Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 100
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char voltage_battery_GET()//Battery voltage, in millivolts (1 = 1 millivolt
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public void voltage_battery_SET(char  src) //Battery voltage, in millivolts (1 = 1 millivolt
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public char drop_rate_comm_GET() {  return (char)((char) get_bytes(data,  4, 2)); }
        public void drop_rate_comm_SET(char  src) {  set_bytes((char)(src) & -1L, 2, data,  4); }
        public char errors_comm_GET() {  return (char)((char) get_bytes(data,  6, 2)); }
        public void errors_comm_SET(char  src) {  set_bytes((char)(src) & -1L, 2, data,  6); }
        public char errors_count1_GET()//Autopilot-specific error
        {  return (char)((char) get_bytes(data,  8, 2)); }
        public void errors_count1_SET(char  src) //Autopilot-specific error
        {  set_bytes((char)(src) & -1L, 2, data,  8); }
        public char errors_count2_GET()//Autopilot-specific error
        {  return (char)((char) get_bytes(data,  10, 2)); }
        public void errors_count2_SET(char  src) //Autopilot-specific error
        {  set_bytes((char)(src) & -1L, 2, data,  10); }
        public char errors_count3_GET()//Autopilot-specific error
        {  return (char)((char) get_bytes(data,  12, 2)); }
        public void errors_count3_SET(char  src) //Autopilot-specific error
        {  set_bytes((char)(src) & -1L, 2, data,  12); }
        public char errors_count4_GET()//Autopilot-specific error
        {  return (char)((char) get_bytes(data,  14, 2)); }
        public void errors_count4_SET(char  src) //Autopilot-specific error
        {  set_bytes((char)(src) & -1L, 2, data,  14); }
        public short current_battery_GET()//Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the curre
        {  return (short)((short) get_bytes(data,  16, 2)); }
        public void current_battery_SET(short  src) //Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the curre
        {  set_bytes((short)(src) & -1L, 2, data,  16); }
        public byte battery_remaining_GET()//Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining batter
        {  return (byte)((byte) get_bytes(data,  18, 1)); }
        public void battery_remaining_SET(byte  src) //Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining batter
        {  set_bytes((byte)(src) & -1L, 1, data,  18); }
        public @MAV_SYS_STATUS_SENSOR int onboard_control_sensors_present_GET()
        {
            switch((int)get_bits(data, 152, 5))
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
            assert(false);//("Unknown enum ID " + id);
            return  Integer.MIN_VALUE;
        }
        public void onboard_control_sensors_present_SET(@MAV_SYS_STATUS_SENSOR int  src)
        {
            long id = 0;
            switch(src)
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
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 5, data, 152);
        }
        public @MAV_SYS_STATUS_SENSOR int onboard_control_sensors_enabled_GET()
        {
            switch((int)get_bits(data, 157, 5))
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
            assert(false);//("Unknown enum ID " + id);
            return  Integer.MIN_VALUE;
        }
        public void onboard_control_sensors_enabled_SET(@MAV_SYS_STATUS_SENSOR int  src)
        {
            long id = 0;
            switch(src)
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
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 5, data, 157);
        }
        public @MAV_SYS_STATUS_SENSOR int onboard_control_sensors_health_GET()
        {
            switch((int)get_bits(data, 162, 5))
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
            assert(false);//("Unknown enum ID " + id);
            return  Integer.MIN_VALUE;
        }
        public void onboard_control_sensors_health_SET(@MAV_SYS_STATUS_SENSOR int  src)
        {
            long id = 0;
            switch(src)
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
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 5, data, 162);
        }
        static final Meta meta = new Meta(1, 8, 0, 0, 21, 167);
    } public static class SYSTEM_TIME extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        SYSTEM_TIME() { super(meta, 0); }
        SYSTEM_TIME(int bytes) { super(meta, bytes); }
        public long time_boot_ms_GET()//Timestamp of the component clock since boot time in milliseconds
        {  return (get_bytes(data,  0, 4)); }
        public void time_boot_ms_SET(long  src) //Timestamp of the component clock since boot time in milliseconds
        {  set_bytes((src) & -1L, 4, data,  0); }
        public long time_unix_usec_GET()//Timestamp of the master clock in microseconds since UNIX epoch
        {  return (get_bytes(data,  4, 8)); }
        public void time_unix_usec_SET(long  src) //Timestamp of the master clock in microseconds since UNIX epoch
        {  set_bytes((src) & -1L, 8, data,  4); }
        static final Meta meta = new Meta(2, 0, 1, 1, 12, 96);
    } public static class POSITION_TARGET_LOCAL_NED extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        POSITION_TARGET_LOCAL_NED() { super(meta, 0); }
        POSITION_TARGET_LOCAL_NED(int bytes) { super(meta, bytes); }
        public char type_mask_GET() {  return (char)((char) get_bytes(data,  0, 2)); }
        public void type_mask_SET(char  src) {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public long time_boot_ms_GET()//Timestamp in milliseconds since system boo
        {  return (get_bytes(data,  2, 4)); }
        public void time_boot_ms_SET(long  src) //Timestamp in milliseconds since system boo
        {  set_bytes((src) & -1L, 4, data,  2); }
        public float x_GET()//X Position in NED frame in meter
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  6, 4))); }
        public void x_SET(float  src) //X Position in NED frame in meter
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 6); }
        public float y_GET()//Y Position in NED frame in meter
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  10, 4))); }
        public void y_SET(float  src) //Y Position in NED frame in meter
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 10); }
        public float z_GET()//Z Position in NED frame in meters (note, altitude is negative in NED
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  14, 4))); }
        public void z_SET(float  src) //Z Position in NED frame in meters (note, altitude is negative in NED
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 14); }
        public float vx_GET()//X velocity in NED frame in meter /
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  18, 4))); }
        public void vx_SET(float  src) //X velocity in NED frame in meter /
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 18); }
        public float vy_GET()//Y velocity in NED frame in meter /
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  22, 4))); }
        public void vy_SET(float  src) //Y velocity in NED frame in meter /
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 22); }
        public float vz_GET()//Z velocity in NED frame in meter /
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  26, 4))); }
        public void vz_SET(float  src) //Z velocity in NED frame in meter /
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 26); }
        public float afx_GET()//X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  30, 4))); }
        public void afx_SET(float  src) //X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 30); }
        public float afy_GET()//Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  34, 4))); }
        public void afy_SET(float  src) //Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 34); }
        public float afz_GET()//Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  38, 4))); }
        public void afz_SET(float  src) //Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 38); }
        public float yaw_GET()//yaw setpoint in ra
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  42, 4))); }
        public void yaw_SET(float  src) //yaw setpoint in ra
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 42); }
        public float yaw_rate_GET()//yaw rate setpoint in rad/
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  46, 4))); }
        public void yaw_rate_SET(float  src) //yaw rate setpoint in rad/
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 46); }
        public @MAV_FRAME int coordinate_frame_GET() {  return  0 + (int)get_bits(data, 400, 4); }
        public void coordinate_frame_SET(@MAV_FRAME int  src) {  set_bits(- 0 +   src, 4, data, 400); }
        static final Meta meta = new Meta(3, 1, 1, 0, 51, 404);
    } public static class PING extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        PING() { super(meta, 0); }
        PING(int bytes) { super(meta, bytes); }
        public long seq_GET()//PING sequenc
        {  return (get_bytes(data,  0, 4)); }
        public void seq_SET(long  src) //PING sequenc
        {  set_bytes((src) & -1L, 4, data,  0); }
        public long time_usec_GET()//Unix timestamp in microseconds or since system boot if smaller than MAVLink epoch (1.1.2009
        {  return (get_bytes(data,  4, 8)); }
        public void time_usec_SET(long  src) //Unix timestamp in microseconds or since system boot if smaller than MAVLink epoch (1.1.2009
        {  set_bytes((src) & -1L, 8, data,  4); }
        public char target_system_GET() {  return (char)((char) get_bytes(data,  12, 1)); }
        public void target_system_SET(char  src) {  set_bytes((char)(src) & -1L, 1, data,  12); }
        public char target_component_GET() {  return (char)((char) get_bytes(data,  13, 1)); }
        public void target_component_SET(char  src) {  set_bytes((char)(src) & -1L, 1, data,  13); }
        static final Meta meta = new Meta(4, 0, 1, 1, 14, 112);
    } public static class CHANGE_OPERATOR_CONTROL extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        CHANGE_OPERATOR_CONTROL() { super(meta, 0); }
        CHANGE_OPERATOR_CONTROL(int bytes) { super(meta, bytes); }
        public char target_system_GET()//System the GCS requests control fo
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public void target_system_SET(char  src) //System the GCS requests control fo
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public char control_request_GET()//0: request control of this MAV, 1: Release control of this MA
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public void control_request_SET(char  src) //0: request control of this MAV, 1: Release control of this MA
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public char version_GET() {  return (char)((char) get_bytes(data,  2, 1)); }
        public void version_SET(char  src) {  set_bytes((char)(src) & -1L, 1, data,  2); }
        public String passkey_TRY(Bounds.Inside ph)
        {
            if(ph.field_bit !=  24 && !try_visit_field(ph, 24)  ||  !try_visit_item(ph, 0)) return null;
            return new String(passkey_GET(ph, new char[ph.items], 0));
        }
        public char[] passkey_GET(Bounds.Inside ph, char[]  dst_ch, int pos)
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int passkey_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  24 && !try_visit_field(ph, 24)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        } public void passkey_SET(String src, Bounds.Inside ph) {passkey_SET(src.toCharArray(), 0, src.length(), ph);} public void passkey_SET(char[]  src, int pos, int items, Bounds.Inside ph)
        {
            if(ph.field_bit != 24 && insert_field(ph, 24, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } static final Meta meta = new Meta(5, 0, 0, 0, 4, 24, 0, _I);
    } public static class CHANGE_OPERATOR_CONTROL_ACK extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        CHANGE_OPERATOR_CONTROL_ACK() { super(meta, 0); }
        CHANGE_OPERATOR_CONTROL_ACK(int bytes) { super(meta, bytes); }
        public char gcs_system_id_GET()//ID of the GCS this message
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public void gcs_system_id_SET(char  src) //ID of the GCS this message
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public char control_request_GET()//0: request control of this MAV, 1: Release control of this MA
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public void control_request_SET(char  src) //0: request control of this MAV, 1: Release control of this MA
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public char ack_GET() {  return (char)((char) get_bytes(data,  2, 1)); }
        public void ack_SET(char  src) {  set_bytes((char)(src) & -1L, 1, data,  2); }
        static final Meta meta = new Meta(6, 0, 0, 0, 3, 24);
    } public static class AUTH_KEY extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        AUTH_KEY() { super(meta, 0); }
        AUTH_KEY(int bytes) { super(meta, bytes); }
        public String key_TRY(Bounds.Inside ph)//ke
//ke
        {
            if(ph.field_bit !=  0 && !try_visit_field(ph, 0)  ||  !try_visit_item(ph, 0)) return null;
            return new String(key_GET(ph, new char[ph.items], 0));
        }
        public char[] key_GET(Bounds.Inside ph, char[]  dst_ch, int pos) //ke
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int key_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  0 && !try_visit_field(ph, 0)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        } public void key_SET(String src, Bounds.Inside ph) //ke
        {key_SET(src.toCharArray(), 0, src.length(), ph);} public void key_SET(char[]  src, int pos, int items, Bounds.Inside ph) //ke
        {
            if(ph.field_bit != 0 && insert_field(ph, 0, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } static final Meta meta = new Meta(7, 0, 0, 0, 1, 0, 0, _k);
    } public static class SET_MODE extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        SET_MODE() { super(meta, 0); }
        SET_MODE(int bytes) { super(meta, bytes); }
        public long custom_mode_GET()//The new autopilot-specific mode. This field can be ignored by an autopilot
        {  return (get_bytes(data,  0, 4)); }
        public void custom_mode_SET(long  src) //The new autopilot-specific mode. This field can be ignored by an autopilot
        {  set_bytes((src) & -1L, 4, data,  0); }
        public char target_system_GET()//The system setting the mod
        {  return (char)((char) get_bytes(data,  4, 1)); }
        public void target_system_SET(char  src) //The system setting the mod
        {  set_bytes((char)(src) & -1L, 1, data,  4); }
        public @MAV_MODE int base_mode_GET()//The new base mod
        {
            switch((int)get_bits(data, 40, 4))
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
            assert(false);//("Unknown enum ID " + id);
            return  Integer.MIN_VALUE;
        }
        public void base_mode_SET(@MAV_MODE int  src) //The new base mod
        {
            long id = 0;
            switch(src)
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
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 4, data, 40);
        }
        static final Meta meta = new Meta(11, 0, 1, 0, 6, 44);
    } public static class PARAM_REQUEST_READ extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        PARAM_REQUEST_READ() { super(meta, 0); }
        PARAM_REQUEST_READ(int bytes) { super(meta, bytes); }
        public char target_system_GET()//System I
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public void target_system_SET(char  src) //System I
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public char target_component_GET()//Component I
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public void target_component_SET(char  src) //Component I
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public short param_index_GET()//Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignore
        {  return (short)((short) get_bytes(data,  2, 2)); }
        public void param_index_SET(short  src) //Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignore
        {  set_bytes((short)(src) & -1L, 2, data,  2); }
        public String param_id_TRY(Bounds.Inside ph)
        {
            if(ph.field_bit !=  32 && !try_visit_field(ph, 32)  ||  !try_visit_item(ph, 0)) return null;
            return new String(param_id_GET(ph, new char[ph.items], 0));
        }
        public char[] param_id_GET(Bounds.Inside ph, char[]  dst_ch, int pos)
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int param_id_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  32 && !try_visit_field(ph, 32)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        } public void param_id_SET(String src, Bounds.Inside ph) {param_id_SET(src.toCharArray(), 0, src.length(), ph);} public void param_id_SET(char[]  src, int pos, int items, Bounds.Inside ph)
        {
            if(ph.field_bit != 32 && insert_field(ph, 32, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } static final Meta meta = new Meta(20, 0, 0, 0, 5, 32, 0, _i);
    } public static class PARAM_REQUEST_LIST extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        PARAM_REQUEST_LIST() { super(meta, 0); }
        PARAM_REQUEST_LIST(int bytes) { super(meta, bytes); }
        public char target_system_GET()//System I
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public void target_system_SET(char  src) //System I
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public char target_component_GET()//Component I
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public void target_component_SET(char  src) //Component I
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        static final Meta meta = new Meta(21, 0, 0, 0, 2, 16);
    } public static class PARAM_VALUE extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        PARAM_VALUE() { super(meta, 0); }
        PARAM_VALUE(int bytes) { super(meta, bytes); }
        public char param_count_GET()//Total number of onboard parameter
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void param_count_SET(char  src) //Total number of onboard parameter
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char param_index_GET()//Index of this onboard paramete
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public void param_index_SET(char  src) //Index of this onboard paramete
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public float param_value_GET()//Onboard parameter valu
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public void param_value_SET(float  src) //Onboard parameter valu
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }
        public @MAV_PARAM_TYPE int param_type_GET()//Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types
        {  return  1 + (int)get_bits(data, 64, 4); }
        public void param_type_SET(@MAV_PARAM_TYPE int  src) //Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types
        {  set_bits(- 1 +   src, 4, data, 64); }
        public String param_id_TRY(Bounds.Inside ph)
        {
            if(ph.field_bit !=  68 && !try_visit_field(ph, 68)  ||  !try_visit_item(ph, 0)) return null;
            return new String(param_id_GET(ph, new char[ph.items], 0));
        }
        public char[] param_id_GET(Bounds.Inside ph, char[]  dst_ch, int pos)
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int param_id_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  68 && !try_visit_field(ph, 68)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        } public void param_id_SET(String src, Bounds.Inside ph) {param_id_SET(src.toCharArray(), 0, src.length(), ph);} public void param_id_SET(char[]  src, int pos, int items, Bounds.Inside ph)
        {
            if(ph.field_bit != 68 && insert_field(ph, 68, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } static final Meta meta = new Meta(22, 2, 0, 0, 10, 68, 0, _D);
    } public static class PARAM_SET extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        PARAM_SET() { super(meta, 0); }
        PARAM_SET(int bytes) { super(meta, bytes); }
        public char target_system_GET()//System I
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public void target_system_SET(char  src) //System I
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public char target_component_GET()//Component I
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public void target_component_SET(char  src) //Component I
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public float param_value_GET()//Onboard parameter valu
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  2, 4))); }
        public void param_value_SET(float  src) //Onboard parameter valu
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 2); }
        public @MAV_PARAM_TYPE int param_type_GET()//Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types
        {  return  1 + (int)get_bits(data, 48, 4); }
        public void param_type_SET(@MAV_PARAM_TYPE int  src) //Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types
        {  set_bits(- 1 +   src, 4, data, 48); }
        public String param_id_TRY(Bounds.Inside ph)
        {
            if(ph.field_bit !=  52 && !try_visit_field(ph, 52)  ||  !try_visit_item(ph, 0)) return null;
            return new String(param_id_GET(ph, new char[ph.items], 0));
        }
        public char[] param_id_GET(Bounds.Inside ph, char[]  dst_ch, int pos)
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int param_id_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  52 && !try_visit_field(ph, 52)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        } public void param_id_SET(String src, Bounds.Inside ph) {param_id_SET(src.toCharArray(), 0, src.length(), ph);} public void param_id_SET(char[]  src, int pos, int items, Bounds.Inside ph)
        {
            if(ph.field_bit != 52 && insert_field(ph, 52, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } static final Meta meta = new Meta(23, 0, 0, 0, 8, 52, 0, _T);
    } public static class GPS_RAW_INT extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        GPS_RAW_INT() { super(meta, 0); }
        GPS_RAW_INT(int bytes) { super(meta, bytes); }
        public char eph_GET()//GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MA
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void eph_SET(char  src) //GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MA
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char epv_GET()//GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MA
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public void epv_SET(char  src) //GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MA
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public char vel_GET()//GPS ground speed (m/s * 100). If unknown, set to: UINT16_MA
        {  return (char)((char) get_bytes(data,  4, 2)); }
        public void vel_SET(char  src) //GPS ground speed (m/s * 100). If unknown, set to: UINT16_MA
        {  set_bytes((char)(src) & -1L, 2, data,  4); }
        public char cog_GET() {  return (char)((char) get_bytes(data,  6, 2)); }
        public void cog_SET(char  src) {  set_bytes((char)(src) & -1L, 2, data,  6); }
        public long time_usec_GET()//Timestamp (microseconds since UNIX epoch or microseconds since system boot
        {  return (get_bytes(data,  8, 8)); }
        public void time_usec_SET(long  src) //Timestamp (microseconds since UNIX epoch or microseconds since system boot
        {  set_bytes((src) & -1L, 8, data,  8); }
        public int lat_GET()//Latitude (WGS84, EGM96 ellipsoid), in degrees * 1E
        {  return (int)((int) get_bytes(data,  16, 4)); }
        public void lat_SET(int  src) //Latitude (WGS84, EGM96 ellipsoid), in degrees * 1E
        {  set_bytes((int)(src) & -1L, 4, data,  16); }
        public int lon_GET()//Longitude (WGS84, EGM96 ellipsoid), in degrees * 1E
        {  return (int)((int) get_bytes(data,  20, 4)); }
        public void lon_SET(int  src) //Longitude (WGS84, EGM96 ellipsoid), in degrees * 1E
        {  set_bytes((int)(src) & -1L, 4, data,  20); }
        public int alt_GET() {  return (int)((int) get_bytes(data,  24, 4)); }
        public void alt_SET(int  src) {  set_bytes((int)(src) & -1L, 4, data,  24); }
        public char satellites_visible_GET()//Number of satellites visible. If unknown, set to 25
        {  return (char)((char) get_bytes(data,  28, 1)); }
        public void satellites_visible_SET(char  src) //Number of satellites visible. If unknown, set to 25
        {  set_bytes((char)(src) & -1L, 1, data,  28); }
        public @GPS_FIX_TYPE int fix_type_GET()//See the GPS_FIX_TYPE enum
        {  return  0 + (int)get_bits(data, 232, 4); }
        public void fix_type_SET(@GPS_FIX_TYPE int  src) //See the GPS_FIX_TYPE enum
        {  set_bits(- 0 +   src, 4, data, 232); }
        public int  alt_ellipsoid_TRY(Bounds.Inside ph)//Altitude (above WGS84, EGM96 ellipsoid), in meters * 1000 (positive for up)
//Altitude (above WGS84, EGM96 ellipsoid), in meters * 1000 (positive for up)
        {
            if(ph.field_bit !=  236 && !try_visit_field(ph, 236)) return 0;
            return (int)((int) get_bytes(data,  ph.BYTE, 4));
        }
        public void alt_ellipsoid_SET(int  src, Bounds.Inside ph)//Altitude (above WGS84, EGM96 ellipsoid), in meters * 1000 (positive for up)
        {
            if(ph.field_bit != 236)insert_field(ph, 236, 0);
            set_bytes((int)(src) & -1L, 4, data,  ph.BYTE);
        } public long  h_acc_TRY(Bounds.Inside ph) //Position uncertainty in meters * 1000 (positive for up)
//Position uncertainty in meters * 1000 (positive for up)
        {
            if(ph.field_bit !=  237 && !try_visit_field(ph, 237)) return 0;
            return (get_bytes(data,  ph.BYTE, 4));
        }
        public void h_acc_SET(long  src, Bounds.Inside ph)//Position uncertainty in meters * 1000 (positive for up)
        {
            if(ph.field_bit != 237)insert_field(ph, 237, 0);
            set_bytes((src) & -1L, 4, data,  ph.BYTE);
        } public long  v_acc_TRY(Bounds.Inside ph) //Altitude uncertainty in meters * 1000 (positive for up)
//Altitude uncertainty in meters * 1000 (positive for up)
        {
            if(ph.field_bit !=  238 && !try_visit_field(ph, 238)) return 0;
            return (get_bytes(data,  ph.BYTE, 4));
        }
        public void v_acc_SET(long  src, Bounds.Inside ph)//Altitude uncertainty in meters * 1000 (positive for up)
        {
            if(ph.field_bit != 238)insert_field(ph, 238, 0);
            set_bytes((src) & -1L, 4, data,  ph.BYTE);
        } public long  vel_acc_TRY(Bounds.Inside ph) //Speed uncertainty in meters * 1000 (positive for up)
//Speed uncertainty in meters * 1000 (positive for up)
        {
            if(ph.field_bit !=  239 && !try_visit_field(ph, 239)) return 0;
            return (get_bytes(data,  ph.BYTE, 4));
        }
        public void vel_acc_SET(long  src, Bounds.Inside ph)//Speed uncertainty in meters * 1000 (positive for up)
        {
            if(ph.field_bit != 239)insert_field(ph, 239, 0);
            set_bytes((src) & -1L, 4, data,  ph.BYTE);
        } public long  hdg_acc_TRY(Bounds.Inside ph) //Heading / track uncertainty in degrees * 1e5
//Heading / track uncertainty in degrees * 1e5
        {
            if(ph.field_bit !=  240 && !try_visit_field(ph, 240)) return 0;
            return (get_bytes(data,  ph.BYTE, 4));
        }
        public void hdg_acc_SET(long  src, Bounds.Inside ph)//Heading / track uncertainty in degrees * 1e5
        {
            if(ph.field_bit != 240)insert_field(ph, 240, 0);
            set_bytes((src) & -1L, 4, data,  ph.BYTE);
        } static final Meta meta = new Meta(24, 4, 0, 1, 31, 236, 0, _uE, _GE, _FE, _BE, _VE);
    } public static class GPS_STATUS extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        GPS_STATUS() { super(meta, 0); }
        GPS_STATUS(int bytes) { super(meta, bytes); }
        public char satellites_visible_GET()//Number of satellites visibl
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public void satellites_visible_SET(char  src) //Number of satellites visibl
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public char[] satellite_prn_GET(char[]  dst_ch, int pos)  //Global satellite I
        {
            for(int BYTE = 1, dst_max = pos + 20; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] satellite_prn_GET()//Global satellite I
        {return satellite_prn_GET(new char[20], 0);} public void satellite_prn_SET(char[]  src, int pos)  //Global satellite I
        {
            for(int BYTE =  1, src_max = pos + 20; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        public char[] satellite_used_GET(char[]  dst_ch, int pos)  //0: Satellite not used, 1: used for localizatio
        {
            for(int BYTE = 21, dst_max = pos + 20; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] satellite_used_GET()//0: Satellite not used, 1: used for localizatio
        {return satellite_used_GET(new char[20], 0);} public void satellite_used_SET(char[]  src, int pos)  //0: Satellite not used, 1: used for localizatio
        {
            for(int BYTE =  21, src_max = pos + 20; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        public char[] satellite_elevation_GET(char[]  dst_ch, int pos)  //Elevation (0: right on top of receiver, 90: on the horizon) of satellit
        {
            for(int BYTE = 41, dst_max = pos + 20; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] satellite_elevation_GET()//Elevation (0: right on top of receiver, 90: on the horizon) of satellit
        {return satellite_elevation_GET(new char[20], 0);} public void satellite_elevation_SET(char[]  src, int pos)  //Elevation (0: right on top of receiver, 90: on the horizon) of satellit
        {
            for(int BYTE =  41, src_max = pos + 20; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        public char[] satellite_azimuth_GET(char[]  dst_ch, int pos)  //Direction of satellite, 0: 0 deg, 255: 360 deg
        {
            for(int BYTE = 61, dst_max = pos + 20; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] satellite_azimuth_GET()//Direction of satellite, 0: 0 deg, 255: 360 deg
        {return satellite_azimuth_GET(new char[20], 0);} public void satellite_azimuth_SET(char[]  src, int pos)  //Direction of satellite, 0: 0 deg, 255: 360 deg
        {
            for(int BYTE =  61, src_max = pos + 20; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        public char[] satellite_snr_GET(char[]  dst_ch, int pos)  //Signal to noise ratio of satellit
        {
            for(int BYTE = 81, dst_max = pos + 20; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] satellite_snr_GET()//Signal to noise ratio of satellit
        {return satellite_snr_GET(new char[20], 0);} public void satellite_snr_SET(char[]  src, int pos)  //Signal to noise ratio of satellit
        {
            for(int BYTE =  81, src_max = pos + 20; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        static final Meta meta = new Meta(25, 0, 0, 0, 101, 808);
    } public static class SCALED_IMU extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        SCALED_IMU() { super(meta, 0); }
        SCALED_IMU(int bytes) { super(meta, bytes); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot
        {  return (get_bytes(data,  0, 4)); }
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot
        {  set_bytes((src) & -1L, 4, data,  0); }
        public short xacc_GET()//X acceleration (mg
        {  return (short)((short) get_bytes(data,  4, 2)); }
        public void xacc_SET(short  src) //X acceleration (mg
        {  set_bytes((short)(src) & -1L, 2, data,  4); }
        public short yacc_GET()//Y acceleration (mg
        {  return (short)((short) get_bytes(data,  6, 2)); }
        public void yacc_SET(short  src) //Y acceleration (mg
        {  set_bytes((short)(src) & -1L, 2, data,  6); }
        public short zacc_GET()//Z acceleration (mg
        {  return (short)((short) get_bytes(data,  8, 2)); }
        public void zacc_SET(short  src) //Z acceleration (mg
        {  set_bytes((short)(src) & -1L, 2, data,  8); }
        public short xgyro_GET()//Angular speed around X axis (millirad /sec
        {  return (short)((short) get_bytes(data,  10, 2)); }
        public void xgyro_SET(short  src) //Angular speed around X axis (millirad /sec
        {  set_bytes((short)(src) & -1L, 2, data,  10); }
        public short ygyro_GET()//Angular speed around Y axis (millirad /sec
        {  return (short)((short) get_bytes(data,  12, 2)); }
        public void ygyro_SET(short  src) //Angular speed around Y axis (millirad /sec
        {  set_bytes((short)(src) & -1L, 2, data,  12); }
        public short zgyro_GET()//Angular speed around Z axis (millirad /sec
        {  return (short)((short) get_bytes(data,  14, 2)); }
        public void zgyro_SET(short  src) //Angular speed around Z axis (millirad /sec
        {  set_bytes((short)(src) & -1L, 2, data,  14); }
        public short xmag_GET()//X Magnetic field (milli tesla
        {  return (short)((short) get_bytes(data,  16, 2)); }
        public void xmag_SET(short  src) //X Magnetic field (milli tesla
        {  set_bytes((short)(src) & -1L, 2, data,  16); }
        public short ymag_GET()//Y Magnetic field (milli tesla
        {  return (short)((short) get_bytes(data,  18, 2)); }
        public void ymag_SET(short  src) //Y Magnetic field (milli tesla
        {  set_bytes((short)(src) & -1L, 2, data,  18); }
        public short zmag_GET()//Z Magnetic field (milli tesla
        {  return (short)((short) get_bytes(data,  20, 2)); }
        public void zmag_SET(short  src) //Z Magnetic field (milli tesla
        {  set_bytes((short)(src) & -1L, 2, data,  20); }
        static final Meta meta = new Meta(26, 0, 1, 0, 22, 176);
    } public static class RAW_IMU extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        RAW_IMU() { super(meta, 0); }
        RAW_IMU(int bytes) { super(meta, bytes); }
        public long time_usec_GET()//Timestamp (microseconds since UNIX epoch or microseconds since system boot
        {  return (get_bytes(data,  0, 8)); }
        public void time_usec_SET(long  src) //Timestamp (microseconds since UNIX epoch or microseconds since system boot
        {  set_bytes((src) & -1L, 8, data,  0); }
        public short xacc_GET()//X acceleration (raw
        {  return (short)((short) get_bytes(data,  8, 2)); }
        public void xacc_SET(short  src) //X acceleration (raw
        {  set_bytes((short)(src) & -1L, 2, data,  8); }
        public short yacc_GET()//Y acceleration (raw
        {  return (short)((short) get_bytes(data,  10, 2)); }
        public void yacc_SET(short  src) //Y acceleration (raw
        {  set_bytes((short)(src) & -1L, 2, data,  10); }
        public short zacc_GET()//Z acceleration (raw
        {  return (short)((short) get_bytes(data,  12, 2)); }
        public void zacc_SET(short  src) //Z acceleration (raw
        {  set_bytes((short)(src) & -1L, 2, data,  12); }
        public short xgyro_GET()//Angular speed around X axis (raw
        {  return (short)((short) get_bytes(data,  14, 2)); }
        public void xgyro_SET(short  src) //Angular speed around X axis (raw
        {  set_bytes((short)(src) & -1L, 2, data,  14); }
        public short ygyro_GET()//Angular speed around Y axis (raw
        {  return (short)((short) get_bytes(data,  16, 2)); }
        public void ygyro_SET(short  src) //Angular speed around Y axis (raw
        {  set_bytes((short)(src) & -1L, 2, data,  16); }
        public short zgyro_GET()//Angular speed around Z axis (raw
        {  return (short)((short) get_bytes(data,  18, 2)); }
        public void zgyro_SET(short  src) //Angular speed around Z axis (raw
        {  set_bytes((short)(src) & -1L, 2, data,  18); }
        public short xmag_GET()//X Magnetic field (raw
        {  return (short)((short) get_bytes(data,  20, 2)); }
        public void xmag_SET(short  src) //X Magnetic field (raw
        {  set_bytes((short)(src) & -1L, 2, data,  20); }
        public short ymag_GET()//Y Magnetic field (raw
        {  return (short)((short) get_bytes(data,  22, 2)); }
        public void ymag_SET(short  src) //Y Magnetic field (raw
        {  set_bytes((short)(src) & -1L, 2, data,  22); }
        public short zmag_GET()//Z Magnetic field (raw
        {  return (short)((short) get_bytes(data,  24, 2)); }
        public void zmag_SET(short  src) //Z Magnetic field (raw
        {  set_bytes((short)(src) & -1L, 2, data,  24); }
        static final Meta meta = new Meta(27, 0, 0, 1, 26, 208);
    } public static class RAW_PRESSURE extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        RAW_PRESSURE() { super(meta, 0); }
        RAW_PRESSURE(int bytes) { super(meta, bytes); }
        public long time_usec_GET()//Timestamp (microseconds since UNIX epoch or microseconds since system boot
        {  return (get_bytes(data,  0, 8)); }
        public void time_usec_SET(long  src) //Timestamp (microseconds since UNIX epoch or microseconds since system boot
        {  set_bytes((src) & -1L, 8, data,  0); }
        public short press_abs_GET()//Absolute pressure (raw
        {  return (short)((short) get_bytes(data,  8, 2)); }
        public void press_abs_SET(short  src) //Absolute pressure (raw
        {  set_bytes((short)(src) & -1L, 2, data,  8); }
        public short press_diff1_GET()//Differential pressure 1 (raw, 0 if nonexistant
        {  return (short)((short) get_bytes(data,  10, 2)); }
        public void press_diff1_SET(short  src) //Differential pressure 1 (raw, 0 if nonexistant
        {  set_bytes((short)(src) & -1L, 2, data,  10); }
        public short press_diff2_GET()//Differential pressure 2 (raw, 0 if nonexistant
        {  return (short)((short) get_bytes(data,  12, 2)); }
        public void press_diff2_SET(short  src) //Differential pressure 2 (raw, 0 if nonexistant
        {  set_bytes((short)(src) & -1L, 2, data,  12); }
        public short temperature_GET()//Raw Temperature measurement (raw
        {  return (short)((short) get_bytes(data,  14, 2)); }
        public void temperature_SET(short  src) //Raw Temperature measurement (raw
        {  set_bytes((short)(src) & -1L, 2, data,  14); }
        static final Meta meta = new Meta(28, 0, 0, 1, 16, 128);
    } public static class SCALED_PRESSURE extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        SCALED_PRESSURE() { super(meta, 0); }
        SCALED_PRESSURE(int bytes) { super(meta, bytes); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot
        {  return (get_bytes(data,  0, 4)); }
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot
        {  set_bytes((src) & -1L, 4, data,  0); }
        public float press_abs_GET()//Absolute pressure (hectopascal
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public void press_abs_SET(float  src) //Absolute pressure (hectopascal
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }
        public float press_diff_GET()//Differential pressure 1 (hectopascal
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public void press_diff_SET(float  src) //Differential pressure 1 (hectopascal
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public short temperature_GET()//Temperature measurement (0.01 degrees celsius
        {  return (short)((short) get_bytes(data,  12, 2)); }
        public void temperature_SET(short  src) //Temperature measurement (0.01 degrees celsius
        {  set_bytes((short)(src) & -1L, 2, data,  12); }
        static final Meta meta = new Meta(29, 0, 1, 0, 14, 112);
    } public static class ATTITUDE extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        ATTITUDE() { super(meta, 0); }
        ATTITUDE(int bytes) { super(meta, bytes); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot
        {  return (get_bytes(data,  0, 4)); }
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot
        {  set_bytes((src) & -1L, 4, data,  0); }
        public float roll_GET()//Roll angle (rad, -pi..+pi
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public void roll_SET(float  src) //Roll angle (rad, -pi..+pi
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }
        public float pitch_GET()//Pitch angle (rad, -pi..+pi
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public void pitch_SET(float  src) //Pitch angle (rad, -pi..+pi
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public float yaw_GET()//Yaw angle (rad, -pi..+pi
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public void yaw_SET(float  src) //Yaw angle (rad, -pi..+pi
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public float rollspeed_GET()//Roll angular speed (rad/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public void rollspeed_SET(float  src) //Roll angular speed (rad/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public float pitchspeed_GET()//Pitch angular speed (rad/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public void pitchspeed_SET(float  src) //Pitch angular speed (rad/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }
        public float yawspeed_GET()//Yaw angular speed (rad/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public void yawspeed_SET(float  src) //Yaw angular speed (rad/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        static final Meta meta = new Meta(30, 0, 1, 0, 28, 224);
    } public static class ATTITUDE_QUATERNION extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        ATTITUDE_QUATERNION() { super(meta, 0); }
        ATTITUDE_QUATERNION(int bytes) { super(meta, bytes); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot
        {  return (get_bytes(data,  0, 4)); }
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot
        {  set_bytes((src) & -1L, 4, data,  0); }
        public float q1_GET()//Quaternion component 1, w (1 in null-rotation
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public void q1_SET(float  src) //Quaternion component 1, w (1 in null-rotation
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }
        public float q2_GET()//Quaternion component 2, x (0 in null-rotation
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public void q2_SET(float  src) //Quaternion component 2, x (0 in null-rotation
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public float q3_GET()//Quaternion component 3, y (0 in null-rotation
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public void q3_SET(float  src) //Quaternion component 3, y (0 in null-rotation
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public float q4_GET()//Quaternion component 4, z (0 in null-rotation
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public void q4_SET(float  src) //Quaternion component 4, z (0 in null-rotation
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public float rollspeed_GET()//Roll angular speed (rad/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public void rollspeed_SET(float  src) //Roll angular speed (rad/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }
        public float pitchspeed_GET()//Pitch angular speed (rad/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public void pitchspeed_SET(float  src) //Pitch angular speed (rad/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        public float yawspeed_GET()//Yaw angular speed (rad/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public void yawspeed_SET(float  src) //Yaw angular speed (rad/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
        static final Meta meta = new Meta(31, 0, 1, 0, 32, 256);
    } public static class LOCAL_POSITION_NED extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        LOCAL_POSITION_NED() { super(meta, 0); }
        LOCAL_POSITION_NED(int bytes) { super(meta, bytes); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot
        {  return (get_bytes(data,  0, 4)); }
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot
        {  set_bytes((src) & -1L, 4, data,  0); }
        public float x_GET()//X Positio
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public void x_SET(float  src) //X Positio
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }
        public float y_GET()//Y Positio
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public void y_SET(float  src) //Y Positio
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public float z_GET()//Z Positio
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public void z_SET(float  src) //Z Positio
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public float vx_GET()//X Spee
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public void vx_SET(float  src) //X Spee
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public float vy_GET()//Y Spee
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public void vy_SET(float  src) //Y Spee
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }
        public float vz_GET()//Z Spee
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public void vz_SET(float  src) //Z Spee
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        static final Meta meta = new Meta(32, 0, 1, 0, 28, 224);
    } public static class GLOBAL_POSITION_INT extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        GLOBAL_POSITION_INT() { super(meta, 0); }
        GLOBAL_POSITION_INT(int bytes) { super(meta, bytes); }
        public char hdg_GET()//Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MA
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void hdg_SET(char  src) //Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MA
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot
        {  return (get_bytes(data,  2, 4)); }
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot
        {  set_bytes((src) & -1L, 4, data,  2); }
        public int lat_GET()//Latitude, expressed as degrees * 1E
        {  return (int)((int) get_bytes(data,  6, 4)); }
        public void lat_SET(int  src) //Latitude, expressed as degrees * 1E
        {  set_bytes((int)(src) & -1L, 4, data,  6); }
        public int lon_GET()//Longitude, expressed as degrees * 1E
        {  return (int)((int) get_bytes(data,  10, 4)); }
        public void lon_SET(int  src) //Longitude, expressed as degrees * 1E
        {  set_bytes((int)(src) & -1L, 4, data,  10); }
        public int alt_GET() {  return (int)((int) get_bytes(data,  14, 4)); }
        public void alt_SET(int  src) {  set_bytes((int)(src) & -1L, 4, data,  14); }
        public int relative_alt_GET()//Altitude above ground in meters, expressed as * 1000 (millimeters
        {  return (int)((int) get_bytes(data,  18, 4)); }
        public void relative_alt_SET(int  src) //Altitude above ground in meters, expressed as * 1000 (millimeters
        {  set_bytes((int)(src) & -1L, 4, data,  18); }
        public short vx_GET()//Ground X Speed (Latitude, positive north), expressed as m/s * 10
        {  return (short)((short) get_bytes(data,  22, 2)); }
        public void vx_SET(short  src) //Ground X Speed (Latitude, positive north), expressed as m/s * 10
        {  set_bytes((short)(src) & -1L, 2, data,  22); }
        public short vy_GET()//Ground Y Speed (Longitude, positive east), expressed as m/s * 10
        {  return (short)((short) get_bytes(data,  24, 2)); }
        public void vy_SET(short  src) //Ground Y Speed (Longitude, positive east), expressed as m/s * 10
        {  set_bytes((short)(src) & -1L, 2, data,  24); }
        public short vz_GET()//Ground Z Speed (Altitude, positive down), expressed as m/s * 10
        {  return (short)((short) get_bytes(data,  26, 2)); }
        public void vz_SET(short  src) //Ground Z Speed (Altitude, positive down), expressed as m/s * 10
        {  set_bytes((short)(src) & -1L, 2, data,  26); }
        static final Meta meta = new Meta(33, 1, 1, 0, 28, 224);
    } public static class RC_CHANNELS_SCALED extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        RC_CHANNELS_SCALED() { super(meta, 0); }
        RC_CHANNELS_SCALED(int bytes) { super(meta, bytes); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot
        {  return (get_bytes(data,  0, 4)); }
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot
        {  set_bytes((src) & -1L, 4, data,  0); }
        public char port_GET() {  return (char)((char) get_bytes(data,  4, 1)); }
        public void port_SET(char  src) {  set_bytes((char)(src) & -1L, 1, data,  4); }
        public short chan1_scaled_GET()//RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX
        {  return (short)((short) get_bytes(data,  5, 2)); }
        public void chan1_scaled_SET(short  src) //RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX
        {  set_bytes((short)(src) & -1L, 2, data,  5); }
        public short chan2_scaled_GET()//RC channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX
        {  return (short)((short) get_bytes(data,  7, 2)); }
        public void chan2_scaled_SET(short  src) //RC channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX
        {  set_bytes((short)(src) & -1L, 2, data,  7); }
        public short chan3_scaled_GET()//RC channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX
        {  return (short)((short) get_bytes(data,  9, 2)); }
        public void chan3_scaled_SET(short  src) //RC channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX
        {  set_bytes((short)(src) & -1L, 2, data,  9); }
        public short chan4_scaled_GET()//RC channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX
        {  return (short)((short) get_bytes(data,  11, 2)); }
        public void chan4_scaled_SET(short  src) //RC channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX
        {  set_bytes((short)(src) & -1L, 2, data,  11); }
        public short chan5_scaled_GET()//RC channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX
        {  return (short)((short) get_bytes(data,  13, 2)); }
        public void chan5_scaled_SET(short  src) //RC channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX
        {  set_bytes((short)(src) & -1L, 2, data,  13); }
        public short chan6_scaled_GET()//RC channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX
        {  return (short)((short) get_bytes(data,  15, 2)); }
        public void chan6_scaled_SET(short  src) //RC channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX
        {  set_bytes((short)(src) & -1L, 2, data,  15); }
        public short chan7_scaled_GET()//RC channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX
        {  return (short)((short) get_bytes(data,  17, 2)); }
        public void chan7_scaled_SET(short  src) //RC channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX
        {  set_bytes((short)(src) & -1L, 2, data,  17); }
        public short chan8_scaled_GET()//RC channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX
        {  return (short)((short) get_bytes(data,  19, 2)); }
        public void chan8_scaled_SET(short  src) //RC channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX
        {  set_bytes((short)(src) & -1L, 2, data,  19); }
        public char rssi_GET()//Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown
        {  return (char)((char) get_bytes(data,  21, 1)); }
        public void rssi_SET(char  src) //Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown
        {  set_bytes((char)(src) & -1L, 1, data,  21); }
        static final Meta meta = new Meta(34, 0, 1, 0, 22, 176);
    } public static class RC_CHANNELS_RAW extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        RC_CHANNELS_RAW() { super(meta, 0); }
        RC_CHANNELS_RAW(int bytes) { super(meta, bytes); }
        public char chan1_raw_GET()//RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void chan1_raw_SET(char  src) //RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char chan2_raw_GET()//RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public void chan2_raw_SET(char  src) //RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public char chan3_raw_GET()//RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  return (char)((char) get_bytes(data,  4, 2)); }
        public void chan3_raw_SET(char  src) //RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  set_bytes((char)(src) & -1L, 2, data,  4); }
        public char chan4_raw_GET()//RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  return (char)((char) get_bytes(data,  6, 2)); }
        public void chan4_raw_SET(char  src) //RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  set_bytes((char)(src) & -1L, 2, data,  6); }
        public char chan5_raw_GET()//RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  return (char)((char) get_bytes(data,  8, 2)); }
        public void chan5_raw_SET(char  src) //RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  set_bytes((char)(src) & -1L, 2, data,  8); }
        public char chan6_raw_GET()//RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  return (char)((char) get_bytes(data,  10, 2)); }
        public void chan6_raw_SET(char  src) //RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  set_bytes((char)(src) & -1L, 2, data,  10); }
        public char chan7_raw_GET()//RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  return (char)((char) get_bytes(data,  12, 2)); }
        public void chan7_raw_SET(char  src) //RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  set_bytes((char)(src) & -1L, 2, data,  12); }
        public char chan8_raw_GET()//RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  return (char)((char) get_bytes(data,  14, 2)); }
        public void chan8_raw_SET(char  src) //RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  set_bytes((char)(src) & -1L, 2, data,  14); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot
        {  return (get_bytes(data,  16, 4)); }
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot
        {  set_bytes((src) & -1L, 4, data,  16); }
        public char port_GET() {  return (char)((char) get_bytes(data,  20, 1)); }
        public void port_SET(char  src) {  set_bytes((char)(src) & -1L, 1, data,  20); }
        public char rssi_GET()//Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown
        {  return (char)((char) get_bytes(data,  21, 1)); }
        public void rssi_SET(char  src) //Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown
        {  set_bytes((char)(src) & -1L, 1, data,  21); }
        static final Meta meta = new Meta(35, 8, 1, 0, 22, 176);
    } public static class SERVO_OUTPUT_RAW extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        SERVO_OUTPUT_RAW() { super(meta, 0); }
        SERVO_OUTPUT_RAW(int bytes) { super(meta, bytes); }
        public char servo1_raw_GET()//Servo output 1 value, in microsecond
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void servo1_raw_SET(char  src) //Servo output 1 value, in microsecond
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char servo2_raw_GET()//Servo output 2 value, in microsecond
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public void servo2_raw_SET(char  src) //Servo output 2 value, in microsecond
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public char servo3_raw_GET()//Servo output 3 value, in microsecond
        {  return (char)((char) get_bytes(data,  4, 2)); }
        public void servo3_raw_SET(char  src) //Servo output 3 value, in microsecond
        {  set_bytes((char)(src) & -1L, 2, data,  4); }
        public char servo4_raw_GET()//Servo output 4 value, in microsecond
        {  return (char)((char) get_bytes(data,  6, 2)); }
        public void servo4_raw_SET(char  src) //Servo output 4 value, in microsecond
        {  set_bytes((char)(src) & -1L, 2, data,  6); }
        public char servo5_raw_GET()//Servo output 5 value, in microsecond
        {  return (char)((char) get_bytes(data,  8, 2)); }
        public void servo5_raw_SET(char  src) //Servo output 5 value, in microsecond
        {  set_bytes((char)(src) & -1L, 2, data,  8); }
        public char servo6_raw_GET()//Servo output 6 value, in microsecond
        {  return (char)((char) get_bytes(data,  10, 2)); }
        public void servo6_raw_SET(char  src) //Servo output 6 value, in microsecond
        {  set_bytes((char)(src) & -1L, 2, data,  10); }
        public char servo7_raw_GET()//Servo output 7 value, in microsecond
        {  return (char)((char) get_bytes(data,  12, 2)); }
        public void servo7_raw_SET(char  src) //Servo output 7 value, in microsecond
        {  set_bytes((char)(src) & -1L, 2, data,  12); }
        public char servo8_raw_GET()//Servo output 8 value, in microsecond
        {  return (char)((char) get_bytes(data,  14, 2)); }
        public void servo8_raw_SET(char  src) //Servo output 8 value, in microsecond
        {  set_bytes((char)(src) & -1L, 2, data,  14); }
        public long time_usec_GET()//Timestamp (microseconds since system boot
        {  return (get_bytes(data,  16, 4)); }
        public void time_usec_SET(long  src) //Timestamp (microseconds since system boot
        {  set_bytes((src) & -1L, 4, data,  16); }
        public char port_GET() {  return (char)((char) get_bytes(data,  20, 1)); }
        public void port_SET(char  src) {  set_bytes((char)(src) & -1L, 1, data,  20); }
        public char  servo9_raw_TRY(Bounds.Inside ph)//Servo output 9 value, in microsecond
//Servo output 9 value, in microsecond
        {
            if(ph.field_bit !=  168 && !try_visit_field(ph, 168)) return 0;
            return (char)((char) get_bytes(data,  ph.BYTE, 2));
        }
        public void servo9_raw_SET(char  src, Bounds.Inside ph)//Servo output 9 value, in microsecond
        {
            if(ph.field_bit != 168)insert_field(ph, 168, 0);
            set_bytes((char)(src) & -1L, 2, data,  ph.BYTE);
        } public char  servo10_raw_TRY(Bounds.Inside ph) //Servo output 10 value, in microsecond
//Servo output 10 value, in microsecond
        {
            if(ph.field_bit !=  169 && !try_visit_field(ph, 169)) return 0;
            return (char)((char) get_bytes(data,  ph.BYTE, 2));
        }
        public void servo10_raw_SET(char  src, Bounds.Inside ph)//Servo output 10 value, in microsecond
        {
            if(ph.field_bit != 169)insert_field(ph, 169, 0);
            set_bytes((char)(src) & -1L, 2, data,  ph.BYTE);
        } public char  servo11_raw_TRY(Bounds.Inside ph) //Servo output 11 value, in microsecond
//Servo output 11 value, in microsecond
        {
            if(ph.field_bit !=  170 && !try_visit_field(ph, 170)) return 0;
            return (char)((char) get_bytes(data,  ph.BYTE, 2));
        }
        public void servo11_raw_SET(char  src, Bounds.Inside ph)//Servo output 11 value, in microsecond
        {
            if(ph.field_bit != 170)insert_field(ph, 170, 0);
            set_bytes((char)(src) & -1L, 2, data,  ph.BYTE);
        } public char  servo12_raw_TRY(Bounds.Inside ph) //Servo output 12 value, in microsecond
//Servo output 12 value, in microsecond
        {
            if(ph.field_bit !=  171 && !try_visit_field(ph, 171)) return 0;
            return (char)((char) get_bytes(data,  ph.BYTE, 2));
        }
        public void servo12_raw_SET(char  src, Bounds.Inside ph)//Servo output 12 value, in microsecond
        {
            if(ph.field_bit != 171)insert_field(ph, 171, 0);
            set_bytes((char)(src) & -1L, 2, data,  ph.BYTE);
        } public char  servo13_raw_TRY(Bounds.Inside ph) //Servo output 13 value, in microsecond
//Servo output 13 value, in microsecond
        {
            if(ph.field_bit !=  172 && !try_visit_field(ph, 172)) return 0;
            return (char)((char) get_bytes(data,  ph.BYTE, 2));
        }
        public void servo13_raw_SET(char  src, Bounds.Inside ph)//Servo output 13 value, in microsecond
        {
            if(ph.field_bit != 172)insert_field(ph, 172, 0);
            set_bytes((char)(src) & -1L, 2, data,  ph.BYTE);
        } public char  servo14_raw_TRY(Bounds.Inside ph) //Servo output 14 value, in microsecond
//Servo output 14 value, in microsecond
        {
            if(ph.field_bit !=  173 && !try_visit_field(ph, 173)) return 0;
            return (char)((char) get_bytes(data,  ph.BYTE, 2));
        }
        public void servo14_raw_SET(char  src, Bounds.Inside ph)//Servo output 14 value, in microsecond
        {
            if(ph.field_bit != 173)insert_field(ph, 173, 0);
            set_bytes((char)(src) & -1L, 2, data,  ph.BYTE);
        } public char  servo15_raw_TRY(Bounds.Inside ph) //Servo output 15 value, in microsecond
//Servo output 15 value, in microsecond
        {
            if(ph.field_bit !=  174 && !try_visit_field(ph, 174)) return 0;
            return (char)((char) get_bytes(data,  ph.BYTE, 2));
        }
        public void servo15_raw_SET(char  src, Bounds.Inside ph)//Servo output 15 value, in microsecond
        {
            if(ph.field_bit != 174)insert_field(ph, 174, 0);
            set_bytes((char)(src) & -1L, 2, data,  ph.BYTE);
        } public char  servo16_raw_TRY(Bounds.Inside ph) //Servo output 16 value, in microsecond
//Servo output 16 value, in microsecond
        {
            if(ph.field_bit !=  175 && !try_visit_field(ph, 175)) return 0;
            return (char)((char) get_bytes(data,  ph.BYTE, 2));
        }
        public void servo16_raw_SET(char  src, Bounds.Inside ph)//Servo output 16 value, in microsecond
        {
            if(ph.field_bit != 175)insert_field(ph, 175, 0);
            set_bytes((char)(src) & -1L, 2, data,  ph.BYTE);
        } static final Meta meta = new Meta(36, 8, 1, 0, 22, 168, 0, _JW, _uW, _GW, _FW, _BW, _VW, _jW, _zW);
    } public static class MISSION_REQUEST_PARTIAL_LIST extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        MISSION_REQUEST_PARTIAL_LIST() { super(meta, 0); }
        MISSION_REQUEST_PARTIAL_LIST(int bytes) { super(meta, bytes); }
        public char target_system_GET()//System I
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public void target_system_SET(char  src) //System I
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public char target_component_GET()//Component I
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public void target_component_SET(char  src) //Component I
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public short start_index_GET()//Start index, 0 by defaul
        {  return (short)((short) get_bytes(data,  2, 2)); }
        public void start_index_SET(short  src) //Start index, 0 by defaul
        {  set_bytes((short)(src) & -1L, 2, data,  2); }
        public short end_index_GET()//End index, -1 by default (-1: send list to end). Else a valid index of the lis
        {  return (short)((short) get_bytes(data,  4, 2)); }
        public void end_index_SET(short  src) //End index, -1 by default (-1: send list to end). Else a valid index of the lis
        {  set_bytes((short)(src) & -1L, 2, data,  4); }
        public @MAV_MISSION_TYPE int mission_type_GET()//Mission type, see MAV_MISSION_TYP
        {
            switch((int)get_bits(data, 48, 3))
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
            assert(false);//("Unknown enum ID " + id);
            return  Integer.MIN_VALUE;
        }
        public void mission_type_SET(@MAV_MISSION_TYPE int  src) //Mission type, see MAV_MISSION_TYP
        {
            long id = 0;
            switch(src)
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
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 3, data, 48);
        }
        static final Meta meta = new Meta(37, 0, 0, 0, 7, 51);
    } public static class MISSION_WRITE_PARTIAL_LIST extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        MISSION_WRITE_PARTIAL_LIST() { super(meta, 0); }
        MISSION_WRITE_PARTIAL_LIST(int bytes) { super(meta, bytes); }
        public char target_system_GET()//System I
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public void target_system_SET(char  src) //System I
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public char target_component_GET()//Component I
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public void target_component_SET(char  src) //Component I
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public short start_index_GET()//Start index, 0 by default and smaller / equal to the largest index of the current onboard list
        {  return (short)((short) get_bytes(data,  2, 2)); }
        public void start_index_SET(short  src) //Start index, 0 by default and smaller / equal to the largest index of the current onboard list
        {  set_bytes((short)(src) & -1L, 2, data,  2); }
        public short end_index_GET()//End index, equal or greater than start index
        {  return (short)((short) get_bytes(data,  4, 2)); }
        public void end_index_SET(short  src) //End index, equal or greater than start index
        {  set_bytes((short)(src) & -1L, 2, data,  4); }
        public @MAV_MISSION_TYPE int mission_type_GET()//Mission type, see MAV_MISSION_TYP
        {
            switch((int)get_bits(data, 48, 3))
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
            assert(false);//("Unknown enum ID " + id);
            return  Integer.MIN_VALUE;
        }
        public void mission_type_SET(@MAV_MISSION_TYPE int  src) //Mission type, see MAV_MISSION_TYP
        {
            long id = 0;
            switch(src)
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
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 3, data, 48);
        }
        static final Meta meta = new Meta(38, 0, 0, 0, 7, 51);
    } public static class MISSION_ITEM extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        MISSION_ITEM() { super(meta, 0); }
        MISSION_ITEM(int bytes) { super(meta, bytes); }
        public char seq_GET()//Sequenc
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void seq_SET(char  src) //Sequenc
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char target_system_GET()//System I
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public void target_system_SET(char  src) //System I
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        public char target_component_GET()//Component I
        {  return (char)((char) get_bytes(data,  3, 1)); }
        public void target_component_SET(char  src) //Component I
        {  set_bytes((char)(src) & -1L, 1, data,  3); }
        public char current_GET()//false:0, true:
        {  return (char)((char) get_bytes(data,  4, 1)); }
        public void current_SET(char  src) //false:0, true:
        {  set_bytes((char)(src) & -1L, 1, data,  4); }
        public char autocontinue_GET()//autocontinue to next w
        {  return (char)((char) get_bytes(data,  5, 1)); }
        public void autocontinue_SET(char  src) //autocontinue to next w
        {  set_bytes((char)(src) & -1L, 1, data,  5); }
        public float param1_GET()//PARAM1, see MAV_CMD enu
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  6, 4))); }
        public void param1_SET(float  src) //PARAM1, see MAV_CMD enu
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 6); }
        public float param2_GET()//PARAM2, see MAV_CMD enu
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  10, 4))); }
        public void param2_SET(float  src) //PARAM2, see MAV_CMD enu
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 10); }
        public float param3_GET()//PARAM3, see MAV_CMD enu
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  14, 4))); }
        public void param3_SET(float  src) //PARAM3, see MAV_CMD enu
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 14); }
        public float param4_GET()//PARAM4, see MAV_CMD enu
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  18, 4))); }
        public void param4_SET(float  src) //PARAM4, see MAV_CMD enu
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 18); }
        public float x_GET()//PARAM5 / local: x position, global: latitud
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  22, 4))); }
        public void x_SET(float  src) //PARAM5 / local: x position, global: latitud
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 22); }
        public float y_GET()//PARAM6 / y position: global: longitud
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  26, 4))); }
        public void y_SET(float  src) //PARAM6 / y position: global: longitud
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 26); }
        public float z_GET()//PARAM7 / z position: global: altitude (relative or absolute, depending on frame
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  30, 4))); }
        public void z_SET(float  src) //PARAM7 / z position: global: altitude (relative or absolute, depending on frame
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 30); }
        public @MAV_FRAME int frame_GET()//The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.
        {  return  0 + (int)get_bits(data, 272, 4); }
        public void frame_SET(@MAV_FRAME int  src) //The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.
        {  set_bits(- 0 +   src, 4, data, 272); }
        public @MAV_CMD int command_GET()//The scheduled action for the waypoint. see MAV_CMD in common.xml MAVLink spec
        {
            switch((int)get_bits(data, 276, 7))
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
            assert(false);//("Unknown enum ID " + id);
            return  Integer.MIN_VALUE;
        }
        public void command_SET(@MAV_CMD int  src) //The scheduled action for the waypoint. see MAV_CMD in common.xml MAVLink spec
        {
            long id = 0;
            switch(src)
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
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 7, data, 276);
        }
        public @MAV_MISSION_TYPE int mission_type_GET()//Mission type, see MAV_MISSION_TYP
        {
            switch((int)get_bits(data, 283, 3))
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
            assert(false);//("Unknown enum ID " + id);
            return  Integer.MIN_VALUE;
        }
        public void mission_type_SET(@MAV_MISSION_TYPE int  src) //Mission type, see MAV_MISSION_TYP
        {
            long id = 0;
            switch(src)
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
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 3, data, 283);
        }
        static final Meta meta = new Meta(39, 1, 0, 0, 36, 286);
    } public static class MISSION_REQUEST extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        MISSION_REQUEST() { super(meta, 0); }
        MISSION_REQUEST(int bytes) { super(meta, bytes); }
        public char seq_GET()//Sequenc
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void seq_SET(char  src) //Sequenc
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char target_system_GET()//System I
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public void target_system_SET(char  src) //System I
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        public char target_component_GET()//Component I
        {  return (char)((char) get_bytes(data,  3, 1)); }
        public void target_component_SET(char  src) //Component I
        {  set_bytes((char)(src) & -1L, 1, data,  3); }
        public @MAV_MISSION_TYPE int mission_type_GET()//Mission type, see MAV_MISSION_TYP
        {
            switch((int)get_bits(data, 32, 3))
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
            assert(false);//("Unknown enum ID " + id);
            return  Integer.MIN_VALUE;
        }
        public void mission_type_SET(@MAV_MISSION_TYPE int  src) //Mission type, see MAV_MISSION_TYP
        {
            long id = 0;
            switch(src)
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
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 3, data, 32);
        }
        static final Meta meta = new Meta(40, 1, 0, 0, 5, 35);
    } public static class MISSION_SET_CURRENT extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        MISSION_SET_CURRENT() { super(meta, 0); }
        MISSION_SET_CURRENT(int bytes) { super(meta, bytes); }
        public char seq_GET()//Sequenc
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void seq_SET(char  src) //Sequenc
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char target_system_GET()//System I
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public void target_system_SET(char  src) //System I
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        public char target_component_GET()//Component I
        {  return (char)((char) get_bytes(data,  3, 1)); }
        public void target_component_SET(char  src) //Component I
        {  set_bytes((char)(src) & -1L, 1, data,  3); }
        static final Meta meta = new Meta(41, 1, 0, 0, 4, 32);
    } public static class MISSION_CURRENT extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        MISSION_CURRENT() { super(meta, 0); }
        MISSION_CURRENT(int bytes) { super(meta, bytes); }
        public char seq_GET()//Sequenc
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void seq_SET(char  src) //Sequenc
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        static final Meta meta = new Meta(42, 1, 0, 0, 2, 16);
    } public static class MISSION_REQUEST_LIST extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        MISSION_REQUEST_LIST() { super(meta, 0); }
        MISSION_REQUEST_LIST(int bytes) { super(meta, bytes); }
        public char target_system_GET()//System I
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public void target_system_SET(char  src) //System I
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public char target_component_GET()//Component I
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public void target_component_SET(char  src) //Component I
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public @MAV_MISSION_TYPE int mission_type_GET()//Mission type, see MAV_MISSION_TYP
        {
            switch((int)get_bits(data, 16, 3))
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
            assert(false);//("Unknown enum ID " + id);
            return  Integer.MIN_VALUE;
        }
        public void mission_type_SET(@MAV_MISSION_TYPE int  src) //Mission type, see MAV_MISSION_TYP
        {
            long id = 0;
            switch(src)
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
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 3, data, 16);
        }
        static final Meta meta = new Meta(43, 0, 0, 0, 3, 19);
    } public static class MISSION_COUNT extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        MISSION_COUNT() { super(meta, 0); }
        MISSION_COUNT(int bytes) { super(meta, bytes); }
        public char count_GET()//Number of mission items in the sequenc
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void count_SET(char  src) //Number of mission items in the sequenc
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char target_system_GET()//System I
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public void target_system_SET(char  src) //System I
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        public char target_component_GET()//Component I
        {  return (char)((char) get_bytes(data,  3, 1)); }
        public void target_component_SET(char  src) //Component I
        {  set_bytes((char)(src) & -1L, 1, data,  3); }
        public @MAV_MISSION_TYPE int mission_type_GET()//Mission type, see MAV_MISSION_TYP
        {
            switch((int)get_bits(data, 32, 3))
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
            assert(false);//("Unknown enum ID " + id);
            return  Integer.MIN_VALUE;
        }
        public void mission_type_SET(@MAV_MISSION_TYPE int  src) //Mission type, see MAV_MISSION_TYP
        {
            long id = 0;
            switch(src)
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
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 3, data, 32);
        }
        static final Meta meta = new Meta(44, 1, 0, 0, 5, 35);
    } public static class MISSION_CLEAR_ALL extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        MISSION_CLEAR_ALL() { super(meta, 0); }
        MISSION_CLEAR_ALL(int bytes) { super(meta, bytes); }
        public char target_system_GET()//System I
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public void target_system_SET(char  src) //System I
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public char target_component_GET()//Component I
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public void target_component_SET(char  src) //Component I
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public @MAV_MISSION_TYPE int mission_type_GET()//Mission type, see MAV_MISSION_TYP
        {
            switch((int)get_bits(data, 16, 3))
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
            assert(false);//("Unknown enum ID " + id);
            return  Integer.MIN_VALUE;
        }
        public void mission_type_SET(@MAV_MISSION_TYPE int  src) //Mission type, see MAV_MISSION_TYP
        {
            long id = 0;
            switch(src)
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
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 3, data, 16);
        }
        static final Meta meta = new Meta(45, 0, 0, 0, 3, 19);
    } public static class MISSION_ITEM_REACHED extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        MISSION_ITEM_REACHED() { super(meta, 0); }
        MISSION_ITEM_REACHED(int bytes) { super(meta, bytes); }
        public char seq_GET()//Sequenc
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void seq_SET(char  src) //Sequenc
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        static final Meta meta = new Meta(46, 1, 0, 0, 2, 16);
    } public static class MISSION_ACK extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        MISSION_ACK() { super(meta, 0); }
        MISSION_ACK(int bytes) { super(meta, bytes); }
        public char target_system_GET()//System I
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public void target_system_SET(char  src) //System I
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public char target_component_GET()//Component I
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public void target_component_SET(char  src) //Component I
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public @MAV_MISSION_RESULT int type_GET()//See MAV_MISSION_RESULT enu
        {  return  0 + (int)get_bits(data, 16, 4); }
        public void type_SET(@MAV_MISSION_RESULT int  src) //See MAV_MISSION_RESULT enu
        {  set_bits(- 0 +   src, 4, data, 16); }
        public @MAV_MISSION_TYPE int mission_type_GET()//Mission type, see MAV_MISSION_TYP
        {
            switch((int)get_bits(data, 20, 3))
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
            assert(false);//("Unknown enum ID " + id);
            return  Integer.MIN_VALUE;
        }
        public void mission_type_SET(@MAV_MISSION_TYPE int  src) //Mission type, see MAV_MISSION_TYP
        {
            long id = 0;
            switch(src)
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
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 3, data, 20);
        }
        static final Meta meta = new Meta(47, 0, 0, 0, 3, 23);
    } public static class SET_GPS_GLOBAL_ORIGIN extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        SET_GPS_GLOBAL_ORIGIN() { super(meta, 0); }
        SET_GPS_GLOBAL_ORIGIN(int bytes) { super(meta, bytes); }
        public char target_system_GET()//System I
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public void target_system_SET(char  src) //System I
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public int latitude_GET()//Latitude (WGS84), in degrees * 1E
        {  return (int)((int) get_bytes(data,  1, 4)); }
        public void latitude_SET(int  src) //Latitude (WGS84), in degrees * 1E
        {  set_bytes((int)(src) & -1L, 4, data,  1); }
        public int longitude_GET()//Longitude (WGS84, in degrees * 1E
        {  return (int)((int) get_bytes(data,  5, 4)); }
        public void longitude_SET(int  src) //Longitude (WGS84, in degrees * 1E
        {  set_bytes((int)(src) & -1L, 4, data,  5); }
        public int altitude_GET()//Altitude (AMSL), in meters * 1000 (positive for up
        {  return (int)((int) get_bytes(data,  9, 4)); }
        public void altitude_SET(int  src) //Altitude (AMSL), in meters * 1000 (positive for up
        {  set_bytes((int)(src) & -1L, 4, data,  9); }
        public long  time_usec_TRY(Bounds.Inside ph)//Timestamp (microseconds since UNIX epoch or microseconds since system boot
//Timestamp (microseconds since UNIX epoch or microseconds since system boot
        {
            if(ph.field_bit !=  104 && !try_visit_field(ph, 104)) return 0;
            return (get_bytes(data,  ph.BYTE, 8));
        }
        public void time_usec_SET(long  src, Bounds.Inside ph)//Timestamp (microseconds since UNIX epoch or microseconds since system boot
        {
            if(ph.field_bit != 104)insert_field(ph, 104, 0);
            set_bytes((src) & -1L, 8, data,  ph.BYTE);
        } static final Meta meta = new Meta(48, 0, 0, 0, 14, 104, 0, _ey);
    } public static class GPS_GLOBAL_ORIGIN extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        GPS_GLOBAL_ORIGIN() { super(meta, 0); }
        GPS_GLOBAL_ORIGIN(int bytes) { super(meta, bytes); }
        public int latitude_GET()//Latitude (WGS84), in degrees * 1E
        {  return (int)((int) get_bytes(data,  0, 4)); }
        public void latitude_SET(int  src) //Latitude (WGS84), in degrees * 1E
        {  set_bytes((int)(src) & -1L, 4, data,  0); }
        public int longitude_GET()//Longitude (WGS84), in degrees * 1E
        {  return (int)((int) get_bytes(data,  4, 4)); }
        public void longitude_SET(int  src) //Longitude (WGS84), in degrees * 1E
        {  set_bytes((int)(src) & -1L, 4, data,  4); }
        public int altitude_GET()//Altitude (AMSL), in meters * 1000 (positive for up
        {  return (int)((int) get_bytes(data,  8, 4)); }
        public void altitude_SET(int  src) //Altitude (AMSL), in meters * 1000 (positive for up
        {  set_bytes((int)(src) & -1L, 4, data,  8); }
        public long  time_usec_TRY(Bounds.Inside ph)//Timestamp (microseconds since UNIX epoch or microseconds since system boot
//Timestamp (microseconds since UNIX epoch or microseconds since system boot
        {
            if(ph.field_bit !=  96 && !try_visit_field(ph, 96)) return 0;
            return (get_bytes(data,  ph.BYTE, 8));
        }
        public void time_usec_SET(long  src, Bounds.Inside ph)//Timestamp (microseconds since UNIX epoch or microseconds since system boot
        {
            if(ph.field_bit != 96)insert_field(ph, 96, 0);
            set_bytes((src) & -1L, 8, data,  ph.BYTE);
        } static final Meta meta = new Meta(49, 0, 0, 0, 13, 96, 0, _Oy);
    } public static class PARAM_MAP_RC extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        PARAM_MAP_RC() { super(meta, 0); }
        PARAM_MAP_RC(int bytes) { super(meta, bytes); }
        public char target_system_GET()//System I
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public void target_system_SET(char  src) //System I
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public char target_component_GET()//Component I
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public void target_component_SET(char  src) //Component I
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public short param_index_GET() {  return (short)((short) get_bytes(data,  2, 2)); }
        public void param_index_SET(short  src) {  set_bytes((short)(src) & -1L, 2, data,  2); }
        public char parameter_rc_channel_index_GET() {  return (char)((char) get_bytes(data,  4, 1)); }
        public void parameter_rc_channel_index_SET(char  src) {  set_bytes((char)(src) & -1L, 1, data,  4); }
        public float param_value0_GET()//Initial parameter valu
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  5, 4))); }
        public void param_value0_SET(float  src) //Initial parameter valu
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 5); }
        public float scale_GET()//Scale, maps the RC range [-1, 1] to a parameter valu
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  9, 4))); }
        public void scale_SET(float  src) //Scale, maps the RC range [-1, 1] to a parameter valu
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 9); }
        public float param_value_min_GET() {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  13, 4))); }
        public void param_value_min_SET(float  src) {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 13); }
        public float param_value_max_GET() {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  17, 4))); }
        public void param_value_max_SET(float  src) {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 17); }
        public String param_id_TRY(Bounds.Inside ph)
        {
            if(ph.field_bit !=  168 && !try_visit_field(ph, 168)  ||  !try_visit_item(ph, 0)) return null;
            return new String(param_id_GET(ph, new char[ph.items], 0));
        }
        public char[] param_id_GET(Bounds.Inside ph, char[]  dst_ch, int pos)
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int param_id_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  168 && !try_visit_field(ph, 168)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        } public void param_id_SET(String src, Bounds.Inside ph) {param_id_SET(src.toCharArray(), 0, src.length(), ph);} public void param_id_SET(char[]  src, int pos, int items, Bounds.Inside ph)
        {
            if(ph.field_bit != 168 && insert_field(ph, 168, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } static final Meta meta = new Meta(50, 0, 0, 0, 22, 168, 0, _Uy);
    } public static class MISSION_REQUEST_INT extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        MISSION_REQUEST_INT() { super(meta, 0); }
        MISSION_REQUEST_INT(int bytes) { super(meta, bytes); }
        public char seq_GET()//Sequenc
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void seq_SET(char  src) //Sequenc
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char target_system_GET()//System I
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public void target_system_SET(char  src) //System I
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        public char target_component_GET()//Component I
        {  return (char)((char) get_bytes(data,  3, 1)); }
        public void target_component_SET(char  src) //Component I
        {  set_bytes((char)(src) & -1L, 1, data,  3); }
        public @MAV_MISSION_TYPE int mission_type_GET()//Mission type, see MAV_MISSION_TYP
        {
            switch((int)get_bits(data, 32, 3))
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
            assert(false);//("Unknown enum ID " + id);
            return  Integer.MIN_VALUE;
        }
        public void mission_type_SET(@MAV_MISSION_TYPE int  src) //Mission type, see MAV_MISSION_TYP
        {
            long id = 0;
            switch(src)
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
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 3, data, 32);
        }
        static final Meta meta = new Meta(51, 1, 0, 0, 5, 35);
    } public static class SAFETY_SET_ALLOWED_AREA extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        SAFETY_SET_ALLOWED_AREA() { super(meta, 0); }
        SAFETY_SET_ALLOWED_AREA(int bytes) { super(meta, bytes); }
        public char target_system_GET()//System I
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public void target_system_SET(char  src) //System I
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public char target_component_GET()//Component I
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public void target_component_SET(char  src) //Component I
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public float p1x_GET()//x position 1 / Latitude
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  2, 4))); }
        public void p1x_SET(float  src) //x position 1 / Latitude
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 2); }
        public float p1y_GET()//y position 1 / Longitude
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  6, 4))); }
        public void p1y_SET(float  src) //y position 1 / Longitude
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 6); }
        public float p1z_GET()//z position 1 / Altitude
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  10, 4))); }
        public void p1z_SET(float  src) //z position 1 / Altitude
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 10); }
        public float p2x_GET()//x position 2 / Latitude
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  14, 4))); }
        public void p2x_SET(float  src) //x position 2 / Latitude
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 14); }
        public float p2y_GET()//y position 2 / Longitude
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  18, 4))); }
        public void p2y_SET(float  src) //y position 2 / Longitude
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 18); }
        public float p2z_GET()//z position 2 / Altitude
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  22, 4))); }
        public void p2z_SET(float  src) //z position 2 / Altitude
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 22); }
        public @MAV_FRAME int frame_GET() {  return  0 + (int)get_bits(data, 208, 4); }
        public void frame_SET(@MAV_FRAME int  src) {  set_bits(- 0 +   src, 4, data, 208); }
        static final Meta meta = new Meta(54, 0, 0, 0, 27, 212);
    } public static class SAFETY_ALLOWED_AREA extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        SAFETY_ALLOWED_AREA() { super(meta, 0); }
        SAFETY_ALLOWED_AREA(int bytes) { super(meta, bytes); }
        public float p1x_GET()//x position 1 / Latitude
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  0, 4))); }
        public void p1x_SET(float  src) //x position 1 / Latitude
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 0); }
        public float p1y_GET()//y position 1 / Longitude
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public void p1y_SET(float  src) //y position 1 / Longitude
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }
        public float p1z_GET()//z position 1 / Altitude
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public void p1z_SET(float  src) //z position 1 / Altitude
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public float p2x_GET()//x position 2 / Latitude
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public void p2x_SET(float  src) //x position 2 / Latitude
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public float p2y_GET()//y position 2 / Longitude
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public void p2y_SET(float  src) //y position 2 / Longitude
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public float p2z_GET()//z position 2 / Altitude
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public void p2z_SET(float  src) //z position 2 / Altitude
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }
        public @MAV_FRAME int frame_GET() {  return  0 + (int)get_bits(data, 192, 4); }
        public void frame_SET(@MAV_FRAME int  src) {  set_bits(- 0 +   src, 4, data, 192); }
        static final Meta meta = new Meta(55, 0, 0, 0, 25, 196);
    } public static class ATTITUDE_QUATERNION_COV extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        ATTITUDE_QUATERNION_COV() { super(meta, 0); }
        ATTITUDE_QUATERNION_COV(int bytes) { super(meta, bytes); }
        public long time_usec_GET()//Timestamp (microseconds since system boot or since UNIX epoch
        {  return (get_bytes(data,  0, 8)); }
        public void time_usec_SET(long  src) //Timestamp (microseconds since system boot or since UNIX epoch
        {  set_bytes((src) & -1L, 8, data,  0); }
        public float[] q_GET(float[]  dst_ch, int pos)  //Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation
        {
            for(int BYTE = 8, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] q_GET()//Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation
        {return q_GET(new float[4], 0);} public void q_SET(float[]  src, int pos)  //Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation
        {
            for(int BYTE =  8, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
                set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
        }
        public float rollspeed_GET()//Roll angular speed (rad/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public void rollspeed_SET(float  src) //Roll angular speed (rad/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        public float pitchspeed_GET()//Pitch angular speed (rad/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public void pitchspeed_SET(float  src) //Pitch angular speed (rad/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
        public float yawspeed_GET()//Yaw angular speed (rad/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  32, 4))); }
        public void yawspeed_SET(float  src) //Yaw angular speed (rad/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 32); }
        public float[] covariance_GET(float[]  dst_ch, int pos)  //Attitude covarianc
        {
            for(int BYTE = 36, dst_max = pos + 9; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] covariance_GET()//Attitude covarianc
        {return covariance_GET(new float[9], 0);} public void covariance_SET(float[]  src, int pos)  //Attitude covarianc
        {
            for(int BYTE =  36, src_max = pos + 9; pos < src_max; pos++, BYTE += 4)
                set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
        }
        static final Meta meta = new Meta(61, 0, 0, 1, 72, 576);
    } public static class NAV_CONTROLLER_OUTPUT extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        NAV_CONTROLLER_OUTPUT() { super(meta, 0); }
        NAV_CONTROLLER_OUTPUT(int bytes) { super(meta, bytes); }
        public char wp_dist_GET()//Distance to active waypoint in meter
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void wp_dist_SET(char  src) //Distance to active waypoint in meter
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public float nav_roll_GET()//Current desired roll in degree
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  2, 4))); }
        public void nav_roll_SET(float  src) //Current desired roll in degree
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 2); }
        public float nav_pitch_GET()//Current desired pitch in degree
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  6, 4))); }
        public void nav_pitch_SET(float  src) //Current desired pitch in degree
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 6); }
        public short nav_bearing_GET()//Current desired heading in degree
        {  return (short)((short) get_bytes(data,  10, 2)); }
        public void nav_bearing_SET(short  src) //Current desired heading in degree
        {  set_bytes((short)(src) & -1L, 2, data,  10); }
        public short target_bearing_GET()//Bearing to current waypoint/target in degree
        {  return (short)((short) get_bytes(data,  12, 2)); }
        public void target_bearing_SET(short  src) //Bearing to current waypoint/target in degree
        {  set_bytes((short)(src) & -1L, 2, data,  12); }
        public float alt_error_GET()//Current altitude error in meter
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  14, 4))); }
        public void alt_error_SET(float  src) //Current altitude error in meter
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 14); }
        public float aspd_error_GET()//Current airspeed error in meters/secon
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  18, 4))); }
        public void aspd_error_SET(float  src) //Current airspeed error in meters/secon
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 18); }
        public float xtrack_error_GET()//Current crosstrack error on x-y plane in meter
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  22, 4))); }
        public void xtrack_error_SET(float  src) //Current crosstrack error on x-y plane in meter
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 22); }
        static final Meta meta = new Meta(62, 1, 0, 0, 26, 208);
    } public static class GLOBAL_POSITION_INT_COV extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        GLOBAL_POSITION_INT_COV() { super(meta, 0); }
        GLOBAL_POSITION_INT_COV(int bytes) { super(meta, bytes); }
        public long time_usec_GET()//Timestamp (microseconds since system boot or since UNIX epoch
        {  return (get_bytes(data,  0, 8)); }
        public void time_usec_SET(long  src) //Timestamp (microseconds since system boot or since UNIX epoch
        {  set_bytes((src) & -1L, 8, data,  0); }
        public int lat_GET()//Latitude, expressed as degrees * 1E
        {  return (int)((int) get_bytes(data,  8, 4)); }
        public void lat_SET(int  src) //Latitude, expressed as degrees * 1E
        {  set_bytes((int)(src) & -1L, 4, data,  8); }
        public int lon_GET()//Longitude, expressed as degrees * 1E
        {  return (int)((int) get_bytes(data,  12, 4)); }
        public void lon_SET(int  src) //Longitude, expressed as degrees * 1E
        {  set_bytes((int)(src) & -1L, 4, data,  12); }
        public int alt_GET()//Altitude in meters, expressed as * 1000 (millimeters), above MS
        {  return (int)((int) get_bytes(data,  16, 4)); }
        public void alt_SET(int  src) //Altitude in meters, expressed as * 1000 (millimeters), above MS
        {  set_bytes((int)(src) & -1L, 4, data,  16); }
        public int relative_alt_GET()//Altitude above ground in meters, expressed as * 1000 (millimeters
        {  return (int)((int) get_bytes(data,  20, 4)); }
        public void relative_alt_SET(int  src) //Altitude above ground in meters, expressed as * 1000 (millimeters
        {  set_bytes((int)(src) & -1L, 4, data,  20); }
        public float vx_GET()//Ground X Speed (Latitude), expressed as m/
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public void vx_SET(float  src) //Ground X Speed (Latitude), expressed as m/
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        public float vy_GET()//Ground Y Speed (Longitude), expressed as m/
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public void vy_SET(float  src) //Ground Y Speed (Longitude), expressed as m/
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
        public float vz_GET()//Ground Z Speed (Altitude), expressed as m/
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  32, 4))); }
        public void vz_SET(float  src) //Ground Z Speed (Altitude), expressed as m/
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 32); }
        public float[] covariance_GET(float[]  dst_ch, int pos)  //Covariance matrix (first six entries are the first ROW, next six entries are the second row, etc.
        {
            for(int BYTE = 36, dst_max = pos + 36; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] covariance_GET()//Covariance matrix (first six entries are the first ROW, next six entries are the second row, etc.
        {return covariance_GET(new float[36], 0);} public void covariance_SET(float[]  src, int pos)  //Covariance matrix (first six entries are the first ROW, next six entries are the second row, etc.
        {
            for(int BYTE =  36, src_max = pos + 36; pos < src_max; pos++, BYTE += 4)
                set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
        }
        public @MAV_ESTIMATOR_TYPE int estimator_type_GET()//Class id of the estimator this estimate originated from
        {  return  1 + (int)get_bits(data, 1440, 3); }
        public void estimator_type_SET(@MAV_ESTIMATOR_TYPE int  src) //Class id of the estimator this estimate originated from
        {  set_bits(- 1 +   src, 3, data, 1440); }
        static final Meta meta = new Meta(63, 0, 0, 1, 181, 1443);
    } public static class LOCAL_POSITION_NED_COV extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        LOCAL_POSITION_NED_COV() { super(meta, 0); }
        LOCAL_POSITION_NED_COV(int bytes) { super(meta, bytes); }
        public long time_usec_GET()//Timestamp (microseconds since system boot or since UNIX epoch
        {  return (get_bytes(data,  0, 8)); }
        public void time_usec_SET(long  src) //Timestamp (microseconds since system boot or since UNIX epoch
        {  set_bytes((src) & -1L, 8, data,  0); }
        public float x_GET()//X Positio
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public void x_SET(float  src) //X Positio
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public float y_GET()//Y Positio
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public void y_SET(float  src) //Y Positio
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public float z_GET()//Z Positio
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public void z_SET(float  src) //Z Positio
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public float vx_GET()//X Speed (m/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public void vx_SET(float  src) //X Speed (m/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }
        public float vy_GET()//Y Speed (m/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public void vy_SET(float  src) //Y Speed (m/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        public float vz_GET()//Z Speed (m/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public void vz_SET(float  src) //Z Speed (m/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
        public float ax_GET()//X Acceleration (m/s^2
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  32, 4))); }
        public void ax_SET(float  src) //X Acceleration (m/s^2
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 32); }
        public float ay_GET()//Y Acceleration (m/s^2
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  36, 4))); }
        public void ay_SET(float  src) //Y Acceleration (m/s^2
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 36); }
        public float az_GET()//Z Acceleration (m/s^2
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  40, 4))); }
        public void az_SET(float  src) //Z Acceleration (m/s^2
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 40); }
        public float[] covariance_GET(float[]  dst_ch, int pos)
        {
            for(int BYTE = 44, dst_max = pos + 45; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] covariance_GET() {return covariance_GET(new float[45], 0);} public void covariance_SET(float[]  src, int pos)
        {
            for(int BYTE =  44, src_max = pos + 45; pos < src_max; pos++, BYTE += 4)
                set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
        }
        public @MAV_ESTIMATOR_TYPE int estimator_type_GET()//Class id of the estimator this estimate originated from
        {  return  1 + (int)get_bits(data, 1792, 3); }
        public void estimator_type_SET(@MAV_ESTIMATOR_TYPE int  src) //Class id of the estimator this estimate originated from
        {  set_bits(- 1 +   src, 3, data, 1792); }
        static final Meta meta = new Meta(64, 0, 0, 1, 225, 1795);
    } public static class RC_CHANNELS extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        RC_CHANNELS() { super(meta, 0); }
        RC_CHANNELS(int bytes) { super(meta, bytes); }
        public char chan1_raw_GET()//RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void chan1_raw_SET(char  src) //RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char chan2_raw_GET()//RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public void chan2_raw_SET(char  src) //RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public char chan3_raw_GET()//RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  return (char)((char) get_bytes(data,  4, 2)); }
        public void chan3_raw_SET(char  src) //RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  set_bytes((char)(src) & -1L, 2, data,  4); }
        public char chan4_raw_GET()//RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  return (char)((char) get_bytes(data,  6, 2)); }
        public void chan4_raw_SET(char  src) //RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  set_bytes((char)(src) & -1L, 2, data,  6); }
        public char chan5_raw_GET()//RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  return (char)((char) get_bytes(data,  8, 2)); }
        public void chan5_raw_SET(char  src) //RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  set_bytes((char)(src) & -1L, 2, data,  8); }
        public char chan6_raw_GET()//RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  return (char)((char) get_bytes(data,  10, 2)); }
        public void chan6_raw_SET(char  src) //RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  set_bytes((char)(src) & -1L, 2, data,  10); }
        public char chan7_raw_GET()//RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  return (char)((char) get_bytes(data,  12, 2)); }
        public void chan7_raw_SET(char  src) //RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  set_bytes((char)(src) & -1L, 2, data,  12); }
        public char chan8_raw_GET()//RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  return (char)((char) get_bytes(data,  14, 2)); }
        public void chan8_raw_SET(char  src) //RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  set_bytes((char)(src) & -1L, 2, data,  14); }
        public char chan9_raw_GET()//RC channel 9 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  return (char)((char) get_bytes(data,  16, 2)); }
        public void chan9_raw_SET(char  src) //RC channel 9 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  set_bytes((char)(src) & -1L, 2, data,  16); }
        public char chan10_raw_GET()//RC channel 10 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  return (char)((char) get_bytes(data,  18, 2)); }
        public void chan10_raw_SET(char  src) //RC channel 10 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  set_bytes((char)(src) & -1L, 2, data,  18); }
        public char chan11_raw_GET()//RC channel 11 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  return (char)((char) get_bytes(data,  20, 2)); }
        public void chan11_raw_SET(char  src) //RC channel 11 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  set_bytes((char)(src) & -1L, 2, data,  20); }
        public char chan12_raw_GET()//RC channel 12 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  return (char)((char) get_bytes(data,  22, 2)); }
        public void chan12_raw_SET(char  src) //RC channel 12 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  set_bytes((char)(src) & -1L, 2, data,  22); }
        public char chan13_raw_GET()//RC channel 13 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  return (char)((char) get_bytes(data,  24, 2)); }
        public void chan13_raw_SET(char  src) //RC channel 13 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  set_bytes((char)(src) & -1L, 2, data,  24); }
        public char chan14_raw_GET()//RC channel 14 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  return (char)((char) get_bytes(data,  26, 2)); }
        public void chan14_raw_SET(char  src) //RC channel 14 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  set_bytes((char)(src) & -1L, 2, data,  26); }
        public char chan15_raw_GET()//RC channel 15 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  return (char)((char) get_bytes(data,  28, 2)); }
        public void chan15_raw_SET(char  src) //RC channel 15 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  set_bytes((char)(src) & -1L, 2, data,  28); }
        public char chan16_raw_GET()//RC channel 16 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  return (char)((char) get_bytes(data,  30, 2)); }
        public void chan16_raw_SET(char  src) //RC channel 16 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  set_bytes((char)(src) & -1L, 2, data,  30); }
        public char chan17_raw_GET()//RC channel 17 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  return (char)((char) get_bytes(data,  32, 2)); }
        public void chan17_raw_SET(char  src) //RC channel 17 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  set_bytes((char)(src) & -1L, 2, data,  32); }
        public char chan18_raw_GET()//RC channel 18 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  return (char)((char) get_bytes(data,  34, 2)); }
        public void chan18_raw_SET(char  src) //RC channel 18 value, in microseconds. A value of UINT16_MAX implies the channel is unused
        {  set_bytes((char)(src) & -1L, 2, data,  34); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot
        {  return (get_bytes(data,  36, 4)); }
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot
        {  set_bytes((src) & -1L, 4, data,  36); }
        public char chancount_GET() {  return (char)((char) get_bytes(data,  40, 1)); }
        public void chancount_SET(char  src) {  set_bytes((char)(src) & -1L, 1, data,  40); }
        public char rssi_GET()//Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown
        {  return (char)((char) get_bytes(data,  41, 1)); }
        public void rssi_SET(char  src) //Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown
        {  set_bytes((char)(src) & -1L, 1, data,  41); }
        static final Meta meta = new Meta(65, 18, 1, 0, 42, 336);
    } public static class REQUEST_DATA_STREAM extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        REQUEST_DATA_STREAM() { super(meta, 0); }
        REQUEST_DATA_STREAM(int bytes) { super(meta, bytes); }
        public char req_message_rate_GET()//The requested message rat
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void req_message_rate_SET(char  src) //The requested message rat
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char target_system_GET()//The target requested to send the message stream
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public void target_system_SET(char  src) //The target requested to send the message stream
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        public char target_component_GET()//The target requested to send the message stream
        {  return (char)((char) get_bytes(data,  3, 1)); }
        public void target_component_SET(char  src) //The target requested to send the message stream
        {  set_bytes((char)(src) & -1L, 1, data,  3); }
        public char req_stream_id_GET()//The ID of the requested data strea
        {  return (char)((char) get_bytes(data,  4, 1)); }
        public void req_stream_id_SET(char  src) //The ID of the requested data strea
        {  set_bytes((char)(src) & -1L, 1, data,  4); }
        public char start_stop_GET()//1 to start sending, 0 to stop sending
        {  return (char)((char) get_bytes(data,  5, 1)); }
        public void start_stop_SET(char  src) //1 to start sending, 0 to stop sending
        {  set_bytes((char)(src) & -1L, 1, data,  5); }
        static final Meta meta = new Meta(66, 1, 0, 0, 6, 48);
    } public static class DATA_STREAM extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        DATA_STREAM() { super(meta, 0); }
        DATA_STREAM(int bytes) { super(meta, bytes); }
        public char message_rate_GET()//The message rat
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void message_rate_SET(char  src) //The message rat
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char stream_id_GET()//The ID of the requested data strea
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public void stream_id_SET(char  src) //The ID of the requested data strea
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        public char on_off_GET()//1 stream is enabled, 0 stream is stopped
        {  return (char)((char) get_bytes(data,  3, 1)); }
        public void on_off_SET(char  src) //1 stream is enabled, 0 stream is stopped
        {  set_bytes((char)(src) & -1L, 1, data,  3); }
        static final Meta meta = new Meta(67, 1, 0, 0, 4, 32);
    } public static class MANUAL_CONTROL extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        MANUAL_CONTROL() { super(meta, 0); }
        MANUAL_CONTROL(int bytes) { super(meta, bytes); }
        public char buttons_GET() {  return (char)((char) get_bytes(data,  0, 2)); }
        public void buttons_SET(char  src) {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char target_GET()//The system to be controlled
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public void target_SET(char  src) //The system to be controlled
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        public short x_GET() {  return (short)((short) get_bytes(data,  3, 2)); }
        public void x_SET(short  src) {  set_bytes((short)(src) & -1L, 2, data,  3); }
        public short y_GET() {  return (short)((short) get_bytes(data,  5, 2)); }
        public void y_SET(short  src) {  set_bytes((short)(src) & -1L, 2, data,  5); }
        public short z_GET() {  return (short)((short) get_bytes(data,  7, 2)); }
        public void z_SET(short  src) {  set_bytes((short)(src) & -1L, 2, data,  7); }
        public short r_GET() {  return (short)((short) get_bytes(data,  9, 2)); }
        public void r_SET(short  src) {  set_bytes((short)(src) & -1L, 2, data,  9); }
        static final Meta meta = new Meta(69, 1, 0, 0, 11, 88);
    } public static class RC_CHANNELS_OVERRIDE extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        RC_CHANNELS_OVERRIDE() { super(meta, 0); }
        RC_CHANNELS_OVERRIDE(int bytes) { super(meta, bytes); }
        public char chan1_raw_GET()//RC channel 1 value, in microseconds. A value of UINT16_MAX means to ignore this field
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void chan1_raw_SET(char  src) //RC channel 1 value, in microseconds. A value of UINT16_MAX means to ignore this field
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char chan2_raw_GET()//RC channel 2 value, in microseconds. A value of UINT16_MAX means to ignore this field
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public void chan2_raw_SET(char  src) //RC channel 2 value, in microseconds. A value of UINT16_MAX means to ignore this field
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public char chan3_raw_GET()//RC channel 3 value, in microseconds. A value of UINT16_MAX means to ignore this field
        {  return (char)((char) get_bytes(data,  4, 2)); }
        public void chan3_raw_SET(char  src) //RC channel 3 value, in microseconds. A value of UINT16_MAX means to ignore this field
        {  set_bytes((char)(src) & -1L, 2, data,  4); }
        public char chan4_raw_GET()//RC channel 4 value, in microseconds. A value of UINT16_MAX means to ignore this field
        {  return (char)((char) get_bytes(data,  6, 2)); }
        public void chan4_raw_SET(char  src) //RC channel 4 value, in microseconds. A value of UINT16_MAX means to ignore this field
        {  set_bytes((char)(src) & -1L, 2, data,  6); }
        public char chan5_raw_GET()//RC channel 5 value, in microseconds. A value of UINT16_MAX means to ignore this field
        {  return (char)((char) get_bytes(data,  8, 2)); }
        public void chan5_raw_SET(char  src) //RC channel 5 value, in microseconds. A value of UINT16_MAX means to ignore this field
        {  set_bytes((char)(src) & -1L, 2, data,  8); }
        public char chan6_raw_GET()//RC channel 6 value, in microseconds. A value of UINT16_MAX means to ignore this field
        {  return (char)((char) get_bytes(data,  10, 2)); }
        public void chan6_raw_SET(char  src) //RC channel 6 value, in microseconds. A value of UINT16_MAX means to ignore this field
        {  set_bytes((char)(src) & -1L, 2, data,  10); }
        public char chan7_raw_GET()//RC channel 7 value, in microseconds. A value of UINT16_MAX means to ignore this field
        {  return (char)((char) get_bytes(data,  12, 2)); }
        public void chan7_raw_SET(char  src) //RC channel 7 value, in microseconds. A value of UINT16_MAX means to ignore this field
        {  set_bytes((char)(src) & -1L, 2, data,  12); }
        public char chan8_raw_GET()//RC channel 8 value, in microseconds. A value of UINT16_MAX means to ignore this field
        {  return (char)((char) get_bytes(data,  14, 2)); }
        public void chan8_raw_SET(char  src) //RC channel 8 value, in microseconds. A value of UINT16_MAX means to ignore this field
        {  set_bytes((char)(src) & -1L, 2, data,  14); }
        public char target_system_GET()//System I
        {  return (char)((char) get_bytes(data,  16, 1)); }
        public void target_system_SET(char  src) //System I
        {  set_bytes((char)(src) & -1L, 1, data,  16); }
        public char target_component_GET()//Component I
        {  return (char)((char) get_bytes(data,  17, 1)); }
        public void target_component_SET(char  src) //Component I
        {  set_bytes((char)(src) & -1L, 1, data,  17); }
        static final Meta meta = new Meta(70, 8, 0, 0, 18, 144);
    } public static class MISSION_ITEM_INT extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        MISSION_ITEM_INT() { super(meta, 0); }
        MISSION_ITEM_INT(int bytes) { super(meta, bytes); }
        public char seq_GET() {  return (char)((char) get_bytes(data,  0, 2)); }
        public void seq_SET(char  src) {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char target_system_GET()//System I
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public void target_system_SET(char  src) //System I
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        public char target_component_GET()//Component I
        {  return (char)((char) get_bytes(data,  3, 1)); }
        public void target_component_SET(char  src) //Component I
        {  set_bytes((char)(src) & -1L, 1, data,  3); }
        public char current_GET()//false:0, true:
        {  return (char)((char) get_bytes(data,  4, 1)); }
        public void current_SET(char  src) //false:0, true:
        {  set_bytes((char)(src) & -1L, 1, data,  4); }
        public char autocontinue_GET()//autocontinue to next w
        {  return (char)((char) get_bytes(data,  5, 1)); }
        public void autocontinue_SET(char  src) //autocontinue to next w
        {  set_bytes((char)(src) & -1L, 1, data,  5); }
        public float param1_GET()//PARAM1, see MAV_CMD enu
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  6, 4))); }
        public void param1_SET(float  src) //PARAM1, see MAV_CMD enu
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 6); }
        public float param2_GET()//PARAM2, see MAV_CMD enu
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  10, 4))); }
        public void param2_SET(float  src) //PARAM2, see MAV_CMD enu
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 10); }
        public float param3_GET()//PARAM3, see MAV_CMD enu
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  14, 4))); }
        public void param3_SET(float  src) //PARAM3, see MAV_CMD enu
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 14); }
        public float param4_GET()//PARAM4, see MAV_CMD enu
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  18, 4))); }
        public void param4_SET(float  src) //PARAM4, see MAV_CMD enu
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 18); }
        public int x_GET()//PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^
        {  return (int)((int) get_bytes(data,  22, 4)); }
        public void x_SET(int  src) //PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^
        {  set_bytes((int)(src) & -1L, 4, data,  22); }
        public int y_GET()//PARAM6 / y position: local: x position in meters * 1e4, global: longitude in degrees *10^
        {  return (int)((int) get_bytes(data,  26, 4)); }
        public void y_SET(int  src) //PARAM6 / y position: local: x position in meters * 1e4, global: longitude in degrees *10^
        {  set_bytes((int)(src) & -1L, 4, data,  26); }
        public float z_GET()//PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  30, 4))); }
        public void z_SET(float  src) //PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 30); }
        public @MAV_FRAME int frame_GET()//The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.
        {  return  0 + (int)get_bits(data, 272, 4); }
        public void frame_SET(@MAV_FRAME int  src) //The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.
        {  set_bits(- 0 +   src, 4, data, 272); }
        public @MAV_CMD int command_GET()//The scheduled action for the waypoint. see MAV_CMD in common.xml MAVLink spec
        {
            switch((int)get_bits(data, 276, 7))
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
            assert(false);//("Unknown enum ID " + id);
            return  Integer.MIN_VALUE;
        }
        public void command_SET(@MAV_CMD int  src) //The scheduled action for the waypoint. see MAV_CMD in common.xml MAVLink spec
        {
            long id = 0;
            switch(src)
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
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 7, data, 276);
        }
        public @MAV_MISSION_TYPE int mission_type_GET()//Mission type, see MAV_MISSION_TYP
        {
            switch((int)get_bits(data, 283, 3))
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
            assert(false);//("Unknown enum ID " + id);
            return  Integer.MIN_VALUE;
        }
        public void mission_type_SET(@MAV_MISSION_TYPE int  src) //Mission type, see MAV_MISSION_TYP
        {
            long id = 0;
            switch(src)
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
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 3, data, 283);
        }
        static final Meta meta = new Meta(73, 1, 0, 0, 36, 286);
    } public static class VFR_HUD extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        VFR_HUD() { super(meta, 0); }
        VFR_HUD(int bytes) { super(meta, bytes); }
        public char throttle_GET()//Current throttle setting in integer percent, 0 to 10
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void throttle_SET(char  src) //Current throttle setting in integer percent, 0 to 10
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public float airspeed_GET()//Current airspeed in m/
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  2, 4))); }
        public void airspeed_SET(float  src) //Current airspeed in m/
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 2); }
        public float groundspeed_GET()//Current ground speed in m/
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  6, 4))); }
        public void groundspeed_SET(float  src) //Current ground speed in m/
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 6); }
        public short heading_GET()//Current heading in degrees, in compass units (0..360, 0=north
        {  return (short)((short) get_bytes(data,  10, 2)); }
        public void heading_SET(short  src) //Current heading in degrees, in compass units (0..360, 0=north
        {  set_bytes((short)(src) & -1L, 2, data,  10); }
        public float alt_GET()//Current altitude (MSL), in meter
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public void alt_SET(float  src) //Current altitude (MSL), in meter
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public float climb_GET()//Current climb rate in meters/secon
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public void climb_SET(float  src) //Current climb rate in meters/secon
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        static final Meta meta = new Meta(74, 1, 0, 0, 20, 160);
    } public static class COMMAND_INT extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        COMMAND_INT() { super(meta, 0); }
        COMMAND_INT(int bytes) { super(meta, bytes); }
        public char target_system_GET()//System I
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public void target_system_SET(char  src) //System I
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public char target_component_GET()//Component I
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public void target_component_SET(char  src) //Component I
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public char current_GET()//false:0, true:
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public void current_SET(char  src) //false:0, true:
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        public char autocontinue_GET()//autocontinue to next w
        {  return (char)((char) get_bytes(data,  3, 1)); }
        public void autocontinue_SET(char  src) //autocontinue to next w
        {  set_bytes((char)(src) & -1L, 1, data,  3); }
        public float param1_GET()//PARAM1, see MAV_CMD enu
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public void param1_SET(float  src) //PARAM1, see MAV_CMD enu
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }
        public float param2_GET()//PARAM2, see MAV_CMD enu
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public void param2_SET(float  src) //PARAM2, see MAV_CMD enu
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public float param3_GET()//PARAM3, see MAV_CMD enu
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public void param3_SET(float  src) //PARAM3, see MAV_CMD enu
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public float param4_GET()//PARAM4, see MAV_CMD enu
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public void param4_SET(float  src) //PARAM4, see MAV_CMD enu
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public int x_GET()//PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^
        {  return (int)((int) get_bytes(data,  20, 4)); }
        public void x_SET(int  src) //PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^
        {  set_bytes((int)(src) & -1L, 4, data,  20); }
        public int y_GET()//PARAM6 / local: y position in meters * 1e4, global: longitude in degrees * 10^
        {  return (int)((int) get_bytes(data,  24, 4)); }
        public void y_SET(int  src) //PARAM6 / local: y position in meters * 1e4, global: longitude in degrees * 10^
        {  set_bytes((int)(src) & -1L, 4, data,  24); }
        public float z_GET()//PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public void z_SET(float  src) //PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
        public @MAV_FRAME int frame_GET()//The coordinate system of the COMMAND. see MAV_FRAME in mavlink_types.
        {  return  0 + (int)get_bits(data, 256, 4); }
        public void frame_SET(@MAV_FRAME int  src) //The coordinate system of the COMMAND. see MAV_FRAME in mavlink_types.
        {  set_bits(- 0 +   src, 4, data, 256); }
        public @MAV_CMD int command_GET()//The scheduled action for the mission item. see MAV_CMD in common.xml MAVLink spec
        {
            switch((int)get_bits(data, 260, 7))
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
            assert(false);//("Unknown enum ID " + id);
            return  Integer.MIN_VALUE;
        }
        public void command_SET(@MAV_CMD int  src) //The scheduled action for the mission item. see MAV_CMD in common.xml MAVLink spec
        {
            long id = 0;
            switch(src)
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
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 7, data, 260);
        }
        static final Meta meta = new Meta(75, 0, 0, 0, 34, 267);
    } public static class COMMAND_LONG extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        COMMAND_LONG() { super(meta, 0); }
        COMMAND_LONG(int bytes) { super(meta, bytes); }
        public char target_system_GET()//System which should execute the comman
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public void target_system_SET(char  src) //System which should execute the comman
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public char target_component_GET()//Component which should execute the command, 0 for all component
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public void target_component_SET(char  src) //Component which should execute the command, 0 for all component
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public char confirmation_GET()//0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public void confirmation_SET(char  src) //0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        public float param1_GET()//Parameter 1, as defined by MAV_CMD enum
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  3, 4))); }
        public void param1_SET(float  src) //Parameter 1, as defined by MAV_CMD enum
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 3); }
        public float param2_GET()//Parameter 2, as defined by MAV_CMD enum
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  7, 4))); }
        public void param2_SET(float  src) //Parameter 2, as defined by MAV_CMD enum
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 7); }
        public float param3_GET()//Parameter 3, as defined by MAV_CMD enum
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  11, 4))); }
        public void param3_SET(float  src) //Parameter 3, as defined by MAV_CMD enum
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 11); }
        public float param4_GET()//Parameter 4, as defined by MAV_CMD enum
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  15, 4))); }
        public void param4_SET(float  src) //Parameter 4, as defined by MAV_CMD enum
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 15); }
        public float param5_GET()//Parameter 5, as defined by MAV_CMD enum
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  19, 4))); }
        public void param5_SET(float  src) //Parameter 5, as defined by MAV_CMD enum
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 19); }
        public float param6_GET()//Parameter 6, as defined by MAV_CMD enum
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  23, 4))); }
        public void param6_SET(float  src) //Parameter 6, as defined by MAV_CMD enum
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 23); }
        public float param7_GET()//Parameter 7, as defined by MAV_CMD enum
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  27, 4))); }
        public void param7_SET(float  src) //Parameter 7, as defined by MAV_CMD enum
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 27); }
        public @MAV_CMD int command_GET()//Command ID, as defined by MAV_CMD enum
        {
            switch((int)get_bits(data, 248, 7))
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
            assert(false);//("Unknown enum ID " + id);
            return  Integer.MIN_VALUE;
        }
        public void command_SET(@MAV_CMD int  src) //Command ID, as defined by MAV_CMD enum
        {
            long id = 0;
            switch(src)
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
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 7, data, 248);
        }
        static final Meta meta = new Meta(76, 0, 0, 0, 32, 255);
    } public static class COMMAND_ACK extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        COMMAND_ACK() { super(meta, 0); }
        COMMAND_ACK(int bytes) { super(meta, bytes); }
        public @MAV_CMD int command_GET()//Command ID, as defined by MAV_CMD enum
        {
            switch((int)get_bits(data, 0, 7))
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
            assert(false);//("Unknown enum ID " + id);
            return  Integer.MIN_VALUE;
        }
        public void command_SET(@MAV_CMD int  src) //Command ID, as defined by MAV_CMD enum
        {
            long id = 0;
            switch(src)
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
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 7, data, 0);
        }
        public @MAV_RESULT int result_GET()//See MAV_RESULT enu
        {  return  0 + (int)get_bits(data, 7, 3); }
        public void result_SET(@MAV_RESULT int  src) //See MAV_RESULT enu
        {  set_bits(- 0 +   src, 3, data, 7); }
        public char  progress_TRY(Bounds.Inside ph)
        {
            if(ph.field_bit !=  10 && !try_visit_field(ph, 10)) return 0;
            return (char)((char) get_bytes(data,  ph.BYTE, 1));
        }
        public void progress_SET(char  src, Bounds.Inside ph)
        {
            if(ph.field_bit != 10)insert_field(ph, 10, 0);
            set_bytes((char)(src) & -1L, 1, data,  ph.BYTE);
        } public int  result_param2_TRY(Bounds.Inside ph)
        {
            if(ph.field_bit !=  11 && !try_visit_field(ph, 11)) return 0;
            return (int)((int) get_bytes(data,  ph.BYTE, 4));
        }
        public void result_param2_SET(int  src, Bounds.Inside ph)
        {
            if(ph.field_bit != 11)insert_field(ph, 11, 0);
            set_bytes((int)(src) & -1L, 4, data,  ph.BYTE);
        } public char  target_system_TRY(Bounds.Inside ph) //WIP: System which requested the command to be execute
//WIP: System which requested the command to be execute
        {
            if(ph.field_bit !=  12 && !try_visit_field(ph, 12)) return 0;
            return (char)((char) get_bytes(data,  ph.BYTE, 1));
        }
        public void target_system_SET(char  src, Bounds.Inside ph)//WIP: System which requested the command to be execute
        {
            if(ph.field_bit != 12)insert_field(ph, 12, 0);
            set_bytes((char)(src) & -1L, 1, data,  ph.BYTE);
        } public char  target_component_TRY(Bounds.Inside ph) //WIP: Component which requested the command to be execute
//WIP: Component which requested the command to be execute
        {
            if(ph.field_bit !=  13 && !try_visit_field(ph, 13)) return 0;
            return (char)((char) get_bytes(data,  ph.BYTE, 1));
        }
        public void target_component_SET(char  src, Bounds.Inside ph)//WIP: Component which requested the command to be execute
        {
            if(ph.field_bit != 13)insert_field(ph, 13, 0);
            set_bytes((char)(src) & -1L, 1, data,  ph.BYTE);
        } static final Meta meta = new Meta(77, 0, 0, 0, 3, 10, 0, _sv, _Uv, _pv, _Rv);
    } public static class MANUAL_SETPOINT extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        MANUAL_SETPOINT() { super(meta, 0); }
        MANUAL_SETPOINT(int bytes) { super(meta, bytes); }
        public long time_boot_ms_GET()//Timestamp in milliseconds since system boo
        {  return (get_bytes(data,  0, 4)); }
        public void time_boot_ms_SET(long  src) //Timestamp in milliseconds since system boo
        {  set_bytes((src) & -1L, 4, data,  0); }
        public float roll_GET()//Desired roll rate in radians per secon
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public void roll_SET(float  src) //Desired roll rate in radians per secon
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }
        public float pitch_GET()//Desired pitch rate in radians per secon
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public void pitch_SET(float  src) //Desired pitch rate in radians per secon
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public float yaw_GET()//Desired yaw rate in radians per secon
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public void yaw_SET(float  src) //Desired yaw rate in radians per secon
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public float thrust_GET()//Collective thrust, normalized to 0 ..
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public void thrust_SET(float  src) //Collective thrust, normalized to 0 ..
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public char mode_switch_GET()//Flight mode switch position, 0.. 25
        {  return (char)((char) get_bytes(data,  20, 1)); }
        public void mode_switch_SET(char  src) //Flight mode switch position, 0.. 25
        {  set_bytes((char)(src) & -1L, 1, data,  20); }
        public char manual_override_switch_GET()//Override mode switch position, 0.. 25
        {  return (char)((char) get_bytes(data,  21, 1)); }
        public void manual_override_switch_SET(char  src) //Override mode switch position, 0.. 25
        {  set_bytes((char)(src) & -1L, 1, data,  21); }
        static final Meta meta = new Meta(81, 0, 1, 0, 22, 176);
    } public static class SET_ATTITUDE_TARGET extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        SET_ATTITUDE_TARGET() { super(meta, 0); }
        SET_ATTITUDE_TARGET(int bytes) { super(meta, bytes); }
        public long time_boot_ms_GET()//Timestamp in milliseconds since system boo
        {  return (get_bytes(data,  0, 4)); }
        public void time_boot_ms_SET(long  src) //Timestamp in milliseconds since system boo
        {  set_bytes((src) & -1L, 4, data,  0); }
        public char target_system_GET()//System I
        {  return (char)((char) get_bytes(data,  4, 1)); }
        public void target_system_SET(char  src) //System I
        {  set_bytes((char)(src) & -1L, 1, data,  4); }
        public char target_component_GET()//Component I
        {  return (char)((char) get_bytes(data,  5, 1)); }
        public void target_component_SET(char  src) //Component I
        {  set_bytes((char)(src) & -1L, 1, data,  5); }
        public char type_mask_GET() {  return (char)((char) get_bytes(data,  6, 1)); }
        public void type_mask_SET(char  src) {  set_bytes((char)(src) & -1L, 1, data,  6); }
        public float[] q_GET(float[]  dst_ch, int pos)  //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0
        {
            for(int BYTE = 7, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] q_GET()//Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0
        {return q_GET(new float[4], 0);} public void q_SET(float[]  src, int pos)  //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0
        {
            for(int BYTE =  7, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
                set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
        }
        public float body_roll_rate_GET()//Body roll rate in radians per secon
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  23, 4))); }
        public void body_roll_rate_SET(float  src) //Body roll rate in radians per secon
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 23); }
        public float body_pitch_rate_GET()//Body roll rate in radians per secon
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  27, 4))); }
        public void body_pitch_rate_SET(float  src) //Body roll rate in radians per secon
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 27); }
        public float body_yaw_rate_GET()//Body roll rate in radians per secon
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  31, 4))); }
        public void body_yaw_rate_SET(float  src) //Body roll rate in radians per secon
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 31); }
        public float thrust_GET()//Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  35, 4))); }
        public void thrust_SET(float  src) //Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 35); }
        static final Meta meta = new Meta(82, 0, 1, 0, 39, 312);
    } public static class ATTITUDE_TARGET extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        ATTITUDE_TARGET() { super(meta, 0); }
        ATTITUDE_TARGET(int bytes) { super(meta, bytes); }
        public long time_boot_ms_GET()//Timestamp in milliseconds since system boo
        {  return (get_bytes(data,  0, 4)); }
        public void time_boot_ms_SET(long  src) //Timestamp in milliseconds since system boo
        {  set_bytes((src) & -1L, 4, data,  0); }
        public char type_mask_GET() {  return (char)((char) get_bytes(data,  4, 1)); }
        public void type_mask_SET(char  src) {  set_bytes((char)(src) & -1L, 1, data,  4); }
        public float[] q_GET(float[]  dst_ch, int pos)  //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0
        {
            for(int BYTE = 5, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] q_GET()//Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0
        {return q_GET(new float[4], 0);} public void q_SET(float[]  src, int pos)  //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0
        {
            for(int BYTE =  5, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
                set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
        }
        public float body_roll_rate_GET()//Body roll rate in radians per secon
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  21, 4))); }
        public void body_roll_rate_SET(float  src) //Body roll rate in radians per secon
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 21); }
        public float body_pitch_rate_GET()//Body pitch rate in radians per secon
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  25, 4))); }
        public void body_pitch_rate_SET(float  src) //Body pitch rate in radians per secon
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 25); }
        public float body_yaw_rate_GET()//Body yaw rate in radians per secon
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  29, 4))); }
        public void body_yaw_rate_SET(float  src) //Body yaw rate in radians per secon
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 29); }
        public float thrust_GET()//Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  33, 4))); }
        public void thrust_SET(float  src) //Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 33); }
        static final Meta meta = new Meta(83, 0, 1, 0, 37, 296);
    } public static class SET_POSITION_TARGET_LOCAL_NED extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        SET_POSITION_TARGET_LOCAL_NED() { super(meta, 0); }
        SET_POSITION_TARGET_LOCAL_NED(int bytes) { super(meta, bytes); }
        public char type_mask_GET() {  return (char)((char) get_bytes(data,  0, 2)); }
        public void type_mask_SET(char  src) {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public long time_boot_ms_GET()//Timestamp in milliseconds since system boo
        {  return (get_bytes(data,  2, 4)); }
        public void time_boot_ms_SET(long  src) //Timestamp in milliseconds since system boo
        {  set_bytes((src) & -1L, 4, data,  2); }
        public char target_system_GET()//System I
        {  return (char)((char) get_bytes(data,  6, 1)); }
        public void target_system_SET(char  src) //System I
        {  set_bytes((char)(src) & -1L, 1, data,  6); }
        public char target_component_GET()//Component I
        {  return (char)((char) get_bytes(data,  7, 1)); }
        public void target_component_SET(char  src) //Component I
        {  set_bytes((char)(src) & -1L, 1, data,  7); }
        public float x_GET()//X Position in NED frame in meter
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public void x_SET(float  src) //X Position in NED frame in meter
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public float y_GET()//Y Position in NED frame in meter
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public void y_SET(float  src) //Y Position in NED frame in meter
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public float z_GET()//Z Position in NED frame in meters (note, altitude is negative in NED
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public void z_SET(float  src) //Z Position in NED frame in meters (note, altitude is negative in NED
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public float vx_GET()//X velocity in NED frame in meter /
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public void vx_SET(float  src) //X velocity in NED frame in meter /
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }
        public float vy_GET()//Y velocity in NED frame in meter /
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public void vy_SET(float  src) //Y velocity in NED frame in meter /
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        public float vz_GET()//Z velocity in NED frame in meter /
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public void vz_SET(float  src) //Z velocity in NED frame in meter /
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
        public float afx_GET()//X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  32, 4))); }
        public void afx_SET(float  src) //X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 32); }
        public float afy_GET()//Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  36, 4))); }
        public void afy_SET(float  src) //Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 36); }
        public float afz_GET()//Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  40, 4))); }
        public void afz_SET(float  src) //Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 40); }
        public float yaw_GET()//yaw setpoint in ra
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  44, 4))); }
        public void yaw_SET(float  src) //yaw setpoint in ra
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 44); }
        public float yaw_rate_GET()//yaw rate setpoint in rad/
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  48, 4))); }
        public void yaw_rate_SET(float  src) //yaw rate setpoint in rad/
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 48); }
        public @MAV_FRAME int coordinate_frame_GET() {  return  0 + (int)get_bits(data, 416, 4); }
        public void coordinate_frame_SET(@MAV_FRAME int  src) {  set_bits(- 0 +   src, 4, data, 416); }
        static final Meta meta = new Meta(84, 1, 1, 0, 53, 420);
    } public static class SET_POSITION_TARGET_GLOBAL_INT extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        SET_POSITION_TARGET_GLOBAL_INT() { super(meta, 0); }
        SET_POSITION_TARGET_GLOBAL_INT(int bytes) { super(meta, bytes); }
        public char type_mask_GET() {  return (char)((char) get_bytes(data,  0, 2)); }
        public void type_mask_SET(char  src) {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public long time_boot_ms_GET() {  return (get_bytes(data,  2, 4)); }
        public void time_boot_ms_SET(long  src) {  set_bytes((src) & -1L, 4, data,  2); }
        public char target_system_GET()//System I
        {  return (char)((char) get_bytes(data,  6, 1)); }
        public void target_system_SET(char  src) //System I
        {  set_bytes((char)(src) & -1L, 1, data,  6); }
        public char target_component_GET()//Component I
        {  return (char)((char) get_bytes(data,  7, 1)); }
        public void target_component_SET(char  src) //Component I
        {  set_bytes((char)(src) & -1L, 1, data,  7); }
        public int lat_int_GET()//X Position in WGS84 frame in 1e7 * meter
        {  return (int)((int) get_bytes(data,  8, 4)); }
        public void lat_int_SET(int  src) //X Position in WGS84 frame in 1e7 * meter
        {  set_bytes((int)(src) & -1L, 4, data,  8); }
        public int lon_int_GET()//Y Position in WGS84 frame in 1e7 * meter
        {  return (int)((int) get_bytes(data,  12, 4)); }
        public void lon_int_SET(int  src) //Y Position in WGS84 frame in 1e7 * meter
        {  set_bytes((int)(src) & -1L, 4, data,  12); }
        public float alt_GET()//Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_I
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public void alt_SET(float  src) //Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_I
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public float vx_GET()//X velocity in NED frame in meter /
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public void vx_SET(float  src) //X velocity in NED frame in meter /
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }
        public float vy_GET()//Y velocity in NED frame in meter /
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public void vy_SET(float  src) //Y velocity in NED frame in meter /
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        public float vz_GET()//Z velocity in NED frame in meter /
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public void vz_SET(float  src) //Z velocity in NED frame in meter /
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
        public float afx_GET()//X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  32, 4))); }
        public void afx_SET(float  src) //X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 32); }
        public float afy_GET()//Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  36, 4))); }
        public void afy_SET(float  src) //Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 36); }
        public float afz_GET()//Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  40, 4))); }
        public void afz_SET(float  src) //Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 40); }
        public float yaw_GET()//yaw setpoint in ra
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  44, 4))); }
        public void yaw_SET(float  src) //yaw setpoint in ra
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 44); }
        public float yaw_rate_GET()//yaw rate setpoint in rad/
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  48, 4))); }
        public void yaw_rate_SET(float  src) //yaw rate setpoint in rad/
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 48); }
        public @MAV_FRAME int coordinate_frame_GET() {  return  0 + (int)get_bits(data, 416, 4); }
        public void coordinate_frame_SET(@MAV_FRAME int  src) {  set_bits(- 0 +   src, 4, data, 416); }
        static final Meta meta = new Meta(86, 1, 1, 0, 53, 420);
    } public static class POSITION_TARGET_GLOBAL_INT extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        POSITION_TARGET_GLOBAL_INT() { super(meta, 0); }
        POSITION_TARGET_GLOBAL_INT(int bytes) { super(meta, bytes); }
        public char type_mask_GET() {  return (char)((char) get_bytes(data,  0, 2)); }
        public void type_mask_SET(char  src) {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public long time_boot_ms_GET() {  return (get_bytes(data,  2, 4)); }
        public void time_boot_ms_SET(long  src) {  set_bytes((src) & -1L, 4, data,  2); }
        public int lat_int_GET()//X Position in WGS84 frame in 1e7 * meter
        {  return (int)((int) get_bytes(data,  6, 4)); }
        public void lat_int_SET(int  src) //X Position in WGS84 frame in 1e7 * meter
        {  set_bytes((int)(src) & -1L, 4, data,  6); }
        public int lon_int_GET()//Y Position in WGS84 frame in 1e7 * meter
        {  return (int)((int) get_bytes(data,  10, 4)); }
        public void lon_int_SET(int  src) //Y Position in WGS84 frame in 1e7 * meter
        {  set_bytes((int)(src) & -1L, 4, data,  10); }
        public float alt_GET()//Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_I
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  14, 4))); }
        public void alt_SET(float  src) //Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_I
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 14); }
        public float vx_GET()//X velocity in NED frame in meter /
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  18, 4))); }
        public void vx_SET(float  src) //X velocity in NED frame in meter /
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 18); }
        public float vy_GET()//Y velocity in NED frame in meter /
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  22, 4))); }
        public void vy_SET(float  src) //Y velocity in NED frame in meter /
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 22); }
        public float vz_GET()//Z velocity in NED frame in meter /
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  26, 4))); }
        public void vz_SET(float  src) //Z velocity in NED frame in meter /
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 26); }
        public float afx_GET()//X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  30, 4))); }
        public void afx_SET(float  src) //X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 30); }
        public float afy_GET()//Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  34, 4))); }
        public void afy_SET(float  src) //Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 34); }
        public float afz_GET()//Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  38, 4))); }
        public void afz_SET(float  src) //Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 38); }
        public float yaw_GET()//yaw setpoint in ra
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  42, 4))); }
        public void yaw_SET(float  src) //yaw setpoint in ra
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 42); }
        public float yaw_rate_GET()//yaw rate setpoint in rad/
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  46, 4))); }
        public void yaw_rate_SET(float  src) //yaw rate setpoint in rad/
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 46); }
        public @MAV_FRAME int coordinate_frame_GET() {  return  0 + (int)get_bits(data, 400, 4); }
        public void coordinate_frame_SET(@MAV_FRAME int  src) {  set_bits(- 0 +   src, 4, data, 400); }
        static final Meta meta = new Meta(87, 1, 1, 0, 51, 404);
    } public static class LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET() { super(meta, 0); }
        LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET(int bytes) { super(meta, bytes); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot
        {  return (get_bytes(data,  0, 4)); }
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot
        {  set_bytes((src) & -1L, 4, data,  0); }
        public float x_GET()//X Positio
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public void x_SET(float  src) //X Positio
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }
        public float y_GET()//Y Positio
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public void y_SET(float  src) //Y Positio
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public float z_GET()//Z Positio
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public void z_SET(float  src) //Z Positio
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public float roll_GET()//Rol
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public void roll_SET(float  src) //Rol
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public float pitch_GET()//Pitc
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public void pitch_SET(float  src) //Pitc
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }
        public float yaw_GET()//Ya
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public void yaw_SET(float  src) //Ya
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        static final Meta meta = new Meta(89, 0, 1, 0, 28, 224);
    } public static class HIL_STATE extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        HIL_STATE() { super(meta, 0); }
        HIL_STATE(int bytes) { super(meta, bytes); }
        public long time_usec_GET()//Timestamp (microseconds since UNIX epoch or microseconds since system boot
        {  return (get_bytes(data,  0, 8)); }
        public void time_usec_SET(long  src) //Timestamp (microseconds since UNIX epoch or microseconds since system boot
        {  set_bytes((src) & -1L, 8, data,  0); }
        public float roll_GET()//Roll angle (rad
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public void roll_SET(float  src) //Roll angle (rad
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public float pitch_GET()//Pitch angle (rad
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public void pitch_SET(float  src) //Pitch angle (rad
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public float yaw_GET()//Yaw angle (rad
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public void yaw_SET(float  src) //Yaw angle (rad
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public float rollspeed_GET()//Body frame roll / phi angular speed (rad/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public void rollspeed_SET(float  src) //Body frame roll / phi angular speed (rad/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }
        public float pitchspeed_GET()//Body frame pitch / theta angular speed (rad/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public void pitchspeed_SET(float  src) //Body frame pitch / theta angular speed (rad/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        public float yawspeed_GET()//Body frame yaw / psi angular speed (rad/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public void yawspeed_SET(float  src) //Body frame yaw / psi angular speed (rad/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
        public int lat_GET()//Latitude, expressed as * 1E
        {  return (int)((int) get_bytes(data,  32, 4)); }
        public void lat_SET(int  src) //Latitude, expressed as * 1E
        {  set_bytes((int)(src) & -1L, 4, data,  32); }
        public int lon_GET()//Longitude, expressed as * 1E
        {  return (int)((int) get_bytes(data,  36, 4)); }
        public void lon_SET(int  src) //Longitude, expressed as * 1E
        {  set_bytes((int)(src) & -1L, 4, data,  36); }
        public int alt_GET()//Altitude in meters, expressed as * 1000 (millimeters
        {  return (int)((int) get_bytes(data,  40, 4)); }
        public void alt_SET(int  src) //Altitude in meters, expressed as * 1000 (millimeters
        {  set_bytes((int)(src) & -1L, 4, data,  40); }
        public short vx_GET()//Ground X Speed (Latitude), expressed as m/s * 10
        {  return (short)((short) get_bytes(data,  44, 2)); }
        public void vx_SET(short  src) //Ground X Speed (Latitude), expressed as m/s * 10
        {  set_bytes((short)(src) & -1L, 2, data,  44); }
        public short vy_GET()//Ground Y Speed (Longitude), expressed as m/s * 10
        {  return (short)((short) get_bytes(data,  46, 2)); }
        public void vy_SET(short  src) //Ground Y Speed (Longitude), expressed as m/s * 10
        {  set_bytes((short)(src) & -1L, 2, data,  46); }
        public short vz_GET()//Ground Z Speed (Altitude), expressed as m/s * 10
        {  return (short)((short) get_bytes(data,  48, 2)); }
        public void vz_SET(short  src) //Ground Z Speed (Altitude), expressed as m/s * 10
        {  set_bytes((short)(src) & -1L, 2, data,  48); }
        public short xacc_GET()//X acceleration (mg
        {  return (short)((short) get_bytes(data,  50, 2)); }
        public void xacc_SET(short  src) //X acceleration (mg
        {  set_bytes((short)(src) & -1L, 2, data,  50); }
        public short yacc_GET()//Y acceleration (mg
        {  return (short)((short) get_bytes(data,  52, 2)); }
        public void yacc_SET(short  src) //Y acceleration (mg
        {  set_bytes((short)(src) & -1L, 2, data,  52); }
        public short zacc_GET()//Z acceleration (mg
        {  return (short)((short) get_bytes(data,  54, 2)); }
        public void zacc_SET(short  src) //Z acceleration (mg
        {  set_bytes((short)(src) & -1L, 2, data,  54); }
        static final Meta meta = new Meta(90, 0, 0, 1, 56, 448);
    } public static class HIL_CONTROLS extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        HIL_CONTROLS() { super(meta, 0); }
        HIL_CONTROLS(int bytes) { super(meta, bytes); }
        public long time_usec_GET()//Timestamp (microseconds since UNIX epoch or microseconds since system boot
        {  return (get_bytes(data,  0, 8)); }
        public void time_usec_SET(long  src) //Timestamp (microseconds since UNIX epoch or microseconds since system boot
        {  set_bytes((src) & -1L, 8, data,  0); }
        public float roll_ailerons_GET()//Control output -1 ..
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public void roll_ailerons_SET(float  src) //Control output -1 ..
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public float pitch_elevator_GET()//Control output -1 ..
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public void pitch_elevator_SET(float  src) //Control output -1 ..
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public float yaw_rudder_GET()//Control output -1 ..
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public void yaw_rudder_SET(float  src) //Control output -1 ..
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public float throttle_GET()//Throttle 0 ..
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public void throttle_SET(float  src) //Throttle 0 ..
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }
        public float aux1_GET()//Aux 1, -1 ..
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public void aux1_SET(float  src) //Aux 1, -1 ..
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        public float aux2_GET()//Aux 2, -1 ..
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public void aux2_SET(float  src) //Aux 2, -1 ..
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
        public float aux3_GET()//Aux 3, -1 ..
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  32, 4))); }
        public void aux3_SET(float  src) //Aux 3, -1 ..
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 32); }
        public float aux4_GET()//Aux 4, -1 ..
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  36, 4))); }
        public void aux4_SET(float  src) //Aux 4, -1 ..
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 36); }
        public char nav_mode_GET()//Navigation mode (MAV_NAV_MODE
        {  return (char)((char) get_bytes(data,  40, 1)); }
        public void nav_mode_SET(char  src) //Navigation mode (MAV_NAV_MODE
        {  set_bytes((char)(src) & -1L, 1, data,  40); }
        public @MAV_MODE int mode_GET()//System mode (MAV_MODE
        {
            switch((int)get_bits(data, 328, 4))
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
            assert(false);//("Unknown enum ID " + id);
            return  Integer.MIN_VALUE;
        }
        public void mode_SET(@MAV_MODE int  src) //System mode (MAV_MODE
        {
            long id = 0;
            switch(src)
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
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 4, data, 328);
        }
        static final Meta meta = new Meta(91, 0, 0, 1, 42, 332);
    } public static class HIL_RC_INPUTS_RAW extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        HIL_RC_INPUTS_RAW() { super(meta, 0); }
        HIL_RC_INPUTS_RAW(int bytes) { super(meta, bytes); }
        public char chan1_raw_GET()//RC channel 1 value, in microsecond
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void chan1_raw_SET(char  src) //RC channel 1 value, in microsecond
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char chan2_raw_GET()//RC channel 2 value, in microsecond
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public void chan2_raw_SET(char  src) //RC channel 2 value, in microsecond
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public char chan3_raw_GET()//RC channel 3 value, in microsecond
        {  return (char)((char) get_bytes(data,  4, 2)); }
        public void chan3_raw_SET(char  src) //RC channel 3 value, in microsecond
        {  set_bytes((char)(src) & -1L, 2, data,  4); }
        public char chan4_raw_GET()//RC channel 4 value, in microsecond
        {  return (char)((char) get_bytes(data,  6, 2)); }
        public void chan4_raw_SET(char  src) //RC channel 4 value, in microsecond
        {  set_bytes((char)(src) & -1L, 2, data,  6); }
        public char chan5_raw_GET()//RC channel 5 value, in microsecond
        {  return (char)((char) get_bytes(data,  8, 2)); }
        public void chan5_raw_SET(char  src) //RC channel 5 value, in microsecond
        {  set_bytes((char)(src) & -1L, 2, data,  8); }
        public char chan6_raw_GET()//RC channel 6 value, in microsecond
        {  return (char)((char) get_bytes(data,  10, 2)); }
        public void chan6_raw_SET(char  src) //RC channel 6 value, in microsecond
        {  set_bytes((char)(src) & -1L, 2, data,  10); }
        public char chan7_raw_GET()//RC channel 7 value, in microsecond
        {  return (char)((char) get_bytes(data,  12, 2)); }
        public void chan7_raw_SET(char  src) //RC channel 7 value, in microsecond
        {  set_bytes((char)(src) & -1L, 2, data,  12); }
        public char chan8_raw_GET()//RC channel 8 value, in microsecond
        {  return (char)((char) get_bytes(data,  14, 2)); }
        public void chan8_raw_SET(char  src) //RC channel 8 value, in microsecond
        {  set_bytes((char)(src) & -1L, 2, data,  14); }
        public char chan9_raw_GET()//RC channel 9 value, in microsecond
        {  return (char)((char) get_bytes(data,  16, 2)); }
        public void chan9_raw_SET(char  src) //RC channel 9 value, in microsecond
        {  set_bytes((char)(src) & -1L, 2, data,  16); }
        public char chan10_raw_GET()//RC channel 10 value, in microsecond
        {  return (char)((char) get_bytes(data,  18, 2)); }
        public void chan10_raw_SET(char  src) //RC channel 10 value, in microsecond
        {  set_bytes((char)(src) & -1L, 2, data,  18); }
        public char chan11_raw_GET()//RC channel 11 value, in microsecond
        {  return (char)((char) get_bytes(data,  20, 2)); }
        public void chan11_raw_SET(char  src) //RC channel 11 value, in microsecond
        {  set_bytes((char)(src) & -1L, 2, data,  20); }
        public char chan12_raw_GET()//RC channel 12 value, in microsecond
        {  return (char)((char) get_bytes(data,  22, 2)); }
        public void chan12_raw_SET(char  src) //RC channel 12 value, in microsecond
        {  set_bytes((char)(src) & -1L, 2, data,  22); }
        public long time_usec_GET()//Timestamp (microseconds since UNIX epoch or microseconds since system boot
        {  return (get_bytes(data,  24, 8)); }
        public void time_usec_SET(long  src) //Timestamp (microseconds since UNIX epoch or microseconds since system boot
        {  set_bytes((src) & -1L, 8, data,  24); }
        public char rssi_GET()//Receive signal strength indicator, 0: 0%, 255: 100
        {  return (char)((char) get_bytes(data,  32, 1)); }
        public void rssi_SET(char  src) //Receive signal strength indicator, 0: 0%, 255: 100
        {  set_bytes((char)(src) & -1L, 1, data,  32); }
        static final Meta meta = new Meta(92, 12, 0, 1, 33, 264);
    } public static class HIL_ACTUATOR_CONTROLS extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        HIL_ACTUATOR_CONTROLS() { super(meta, 0); }
        HIL_ACTUATOR_CONTROLS(int bytes) { super(meta, bytes); }
        public long time_usec_GET()//Timestamp (microseconds since UNIX epoch or microseconds since system boot
        {  return (get_bytes(data,  0, 8)); }
        public void time_usec_SET(long  src) //Timestamp (microseconds since UNIX epoch or microseconds since system boot
        {  set_bytes((src) & -1L, 8, data,  0); }
        public long flags_GET()//Flags as bitfield, reserved for future use
        {  return (get_bytes(data,  8, 8)); }
        public void flags_SET(long  src) //Flags as bitfield, reserved for future use
        {  set_bytes((src) & -1L, 8, data,  8); }
        public float[] controls_GET(float[]  dst_ch, int pos)  //Control outputs -1 .. 1. Channel assignment depends on the simulated hardware
        {
            for(int BYTE = 16, dst_max = pos + 16; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] controls_GET()//Control outputs -1 .. 1. Channel assignment depends on the simulated hardware
        {return controls_GET(new float[16], 0);} public void controls_SET(float[]  src, int pos)  //Control outputs -1 .. 1. Channel assignment depends on the simulated hardware
        {
            for(int BYTE =  16, src_max = pos + 16; pos < src_max; pos++, BYTE += 4)
                set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
        }
        public @MAV_MODE int mode_GET()//System mode (MAV_MODE), includes arming state
        {
            switch((int)get_bits(data, 640, 4))
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
            assert(false);//("Unknown enum ID " + id);
            return  Integer.MIN_VALUE;
        }
        public void mode_SET(@MAV_MODE int  src) //System mode (MAV_MODE), includes arming state
        {
            long id = 0;
            switch(src)
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
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 4, data, 640);
        }
        static final Meta meta = new Meta(93, 0, 0, 2, 81, 644);
    } public static class OPTICAL_FLOW extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        OPTICAL_FLOW() { super(meta, 0); }
        OPTICAL_FLOW(int bytes) { super(meta, bytes); }
        public long time_usec_GET()//Timestamp (UNIX
        {  return (get_bytes(data,  0, 8)); }
        public void time_usec_SET(long  src) //Timestamp (UNIX
        {  set_bytes((src) & -1L, 8, data,  0); }
        public char sensor_id_GET()//Sensor I
        {  return (char)((char) get_bytes(data,  8, 1)); }
        public void sensor_id_SET(char  src) //Sensor I
        {  set_bytes((char)(src) & -1L, 1, data,  8); }
        public short flow_x_GET()//Flow in pixels * 10 in x-sensor direction (dezi-pixels
        {  return (short)((short) get_bytes(data,  9, 2)); }
        public void flow_x_SET(short  src) //Flow in pixels * 10 in x-sensor direction (dezi-pixels
        {  set_bytes((short)(src) & -1L, 2, data,  9); }
        public short flow_y_GET()//Flow in pixels * 10 in y-sensor direction (dezi-pixels
        {  return (short)((short) get_bytes(data,  11, 2)); }
        public void flow_y_SET(short  src) //Flow in pixels * 10 in y-sensor direction (dezi-pixels
        {  set_bytes((short)(src) & -1L, 2, data,  11); }
        public float flow_comp_m_x_GET()//Flow in meters in x-sensor direction, angular-speed compensate
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  13, 4))); }
        public void flow_comp_m_x_SET(float  src) //Flow in meters in x-sensor direction, angular-speed compensate
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 13); }
        public float flow_comp_m_y_GET()//Flow in meters in y-sensor direction, angular-speed compensate
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  17, 4))); }
        public void flow_comp_m_y_SET(float  src) //Flow in meters in y-sensor direction, angular-speed compensate
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 17); }
        public char quality_GET()//Optical flow quality / confidence. 0: bad, 255: maximum qualit
        {  return (char)((char) get_bytes(data,  21, 1)); }
        public void quality_SET(char  src) //Optical flow quality / confidence. 0: bad, 255: maximum qualit
        {  set_bytes((char)(src) & -1L, 1, data,  21); }
        public float ground_distance_GET()//Ground distance in meters. Positive value: distance known. Negative value: Unknown distanc
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  22, 4))); }
        public void ground_distance_SET(float  src) //Ground distance in meters. Positive value: distance known. Negative value: Unknown distanc
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 22); }
        public float  flow_rate_x_TRY(Bounds.Inside ph)//Flow rate in radians/second about X axi
//Flow rate in radians/second about X axi
        {
            if(ph.field_bit !=  208 && !try_visit_field(ph, 208)) return 0;
            return (float)(Float.intBitsToFloat((int) get_bytes(data,  ph.BYTE, 4)));
        }
        public void flow_rate_x_SET(float  src, Bounds.Inside ph)//Flow rate in radians/second about X axi
        {
            if(ph.field_bit != 208)insert_field(ph, 208, 0);
            set_bytes(Float.floatToIntBits(src) & -1L, 4, data, ph.BYTE);
        } public float  flow_rate_y_TRY(Bounds.Inside ph) //Flow rate in radians/second about Y axi
//Flow rate in radians/second about Y axi
        {
            if(ph.field_bit !=  209 && !try_visit_field(ph, 209)) return 0;
            return (float)(Float.intBitsToFloat((int) get_bytes(data,  ph.BYTE, 4)));
        }
        public void flow_rate_y_SET(float  src, Bounds.Inside ph)//Flow rate in radians/second about Y axi
        {
            if(ph.field_bit != 209)insert_field(ph, 209, 0);
            set_bytes(Float.floatToIntBits(src) & -1L, 4, data, ph.BYTE);
        } static final Meta meta = new Meta(100, 0, 0, 1, 27, 208, 0, _Vu, _ju);
    } public static class GLOBAL_VISION_POSITION_ESTIMATE extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        GLOBAL_VISION_POSITION_ESTIMATE() { super(meta, 0); }
        GLOBAL_VISION_POSITION_ESTIMATE(int bytes) { super(meta, bytes); }
        public long usec_GET()//Timestamp (microseconds, synced to UNIX time or since system boot
        {  return (get_bytes(data,  0, 8)); }
        public void usec_SET(long  src) //Timestamp (microseconds, synced to UNIX time or since system boot
        {  set_bytes((src) & -1L, 8, data,  0); }
        public float x_GET()//Global X positio
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public void x_SET(float  src) //Global X positio
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public float y_GET()//Global Y positio
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public void y_SET(float  src) //Global Y positio
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public float z_GET()//Global Z positio
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public void z_SET(float  src) //Global Z positio
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public float roll_GET()//Roll angle in ra
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public void roll_SET(float  src) //Roll angle in ra
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }
        public float pitch_GET()//Pitch angle in ra
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public void pitch_SET(float  src) //Pitch angle in ra
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        public float yaw_GET()//Yaw angle in ra
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public void yaw_SET(float  src) //Yaw angle in ra
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
        static final Meta meta = new Meta(101, 0, 0, 1, 32, 256);
    } public static class VISION_POSITION_ESTIMATE extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        VISION_POSITION_ESTIMATE() { super(meta, 0); }
        VISION_POSITION_ESTIMATE(int bytes) { super(meta, bytes); }
        public long usec_GET()//Timestamp (microseconds, synced to UNIX time or since system boot
        {  return (get_bytes(data,  0, 8)); }
        public void usec_SET(long  src) //Timestamp (microseconds, synced to UNIX time or since system boot
        {  set_bytes((src) & -1L, 8, data,  0); }
        public float x_GET()//Global X positio
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public void x_SET(float  src) //Global X positio
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public float y_GET()//Global Y positio
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public void y_SET(float  src) //Global Y positio
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public float z_GET()//Global Z positio
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public void z_SET(float  src) //Global Z positio
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public float roll_GET()//Roll angle in ra
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public void roll_SET(float  src) //Roll angle in ra
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }
        public float pitch_GET()//Pitch angle in ra
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public void pitch_SET(float  src) //Pitch angle in ra
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        public float yaw_GET()//Yaw angle in ra
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public void yaw_SET(float  src) //Yaw angle in ra
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
        static final Meta meta = new Meta(102, 0, 0, 1, 32, 256);
    } public static class VISION_SPEED_ESTIMATE extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        VISION_SPEED_ESTIMATE() { super(meta, 0); }
        VISION_SPEED_ESTIMATE(int bytes) { super(meta, bytes); }
        public long usec_GET()//Timestamp (microseconds, synced to UNIX time or since system boot
        {  return (get_bytes(data,  0, 8)); }
        public void usec_SET(long  src) //Timestamp (microseconds, synced to UNIX time or since system boot
        {  set_bytes((src) & -1L, 8, data,  0); }
        public float x_GET()//Global X spee
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public void x_SET(float  src) //Global X spee
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public float y_GET()//Global Y spee
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public void y_SET(float  src) //Global Y spee
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public float z_GET()//Global Z spee
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public void z_SET(float  src) //Global Z spee
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        static final Meta meta = new Meta(103, 0, 0, 1, 20, 160);
    } public static class VICON_POSITION_ESTIMATE extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        VICON_POSITION_ESTIMATE() { super(meta, 0); }
        VICON_POSITION_ESTIMATE(int bytes) { super(meta, bytes); }
        public long usec_GET()//Timestamp (microseconds, synced to UNIX time or since system boot
        {  return (get_bytes(data,  0, 8)); }
        public void usec_SET(long  src) //Timestamp (microseconds, synced to UNIX time or since system boot
        {  set_bytes((src) & -1L, 8, data,  0); }
        public float x_GET()//Global X positio
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public void x_SET(float  src) //Global X positio
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public float y_GET()//Global Y positio
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public void y_SET(float  src) //Global Y positio
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public float z_GET()//Global Z positio
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public void z_SET(float  src) //Global Z positio
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public float roll_GET()//Roll angle in ra
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public void roll_SET(float  src) //Roll angle in ra
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }
        public float pitch_GET()//Pitch angle in ra
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public void pitch_SET(float  src) //Pitch angle in ra
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        public float yaw_GET()//Yaw angle in ra
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public void yaw_SET(float  src) //Yaw angle in ra
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
        static final Meta meta = new Meta(104, 0, 0, 1, 32, 256);
    } public static class HIGHRES_IMU extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        HIGHRES_IMU() { super(meta, 0); }
        HIGHRES_IMU(int bytes) { super(meta, bytes); }
        public char fields_updated_GET()//Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperatur
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void fields_updated_SET(char  src) //Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperatur
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public long time_usec_GET()//Timestamp (microseconds, synced to UNIX time or since system boot
        {  return (get_bytes(data,  2, 8)); }
        public void time_usec_SET(long  src) //Timestamp (microseconds, synced to UNIX time or since system boot
        {  set_bytes((src) & -1L, 8, data,  2); }
        public float xacc_GET()//X acceleration (m/s^2
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  10, 4))); }
        public void xacc_SET(float  src) //X acceleration (m/s^2
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 10); }
        public float yacc_GET()//Y acceleration (m/s^2
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  14, 4))); }
        public void yacc_SET(float  src) //Y acceleration (m/s^2
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 14); }
        public float zacc_GET()//Z acceleration (m/s^2
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  18, 4))); }
        public void zacc_SET(float  src) //Z acceleration (m/s^2
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 18); }
        public float xgyro_GET()//Angular speed around X axis (rad / sec
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  22, 4))); }
        public void xgyro_SET(float  src) //Angular speed around X axis (rad / sec
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 22); }
        public float ygyro_GET()//Angular speed around Y axis (rad / sec
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  26, 4))); }
        public void ygyro_SET(float  src) //Angular speed around Y axis (rad / sec
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 26); }
        public float zgyro_GET()//Angular speed around Z axis (rad / sec
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  30, 4))); }
        public void zgyro_SET(float  src) //Angular speed around Z axis (rad / sec
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 30); }
        public float xmag_GET()//X Magnetic field (Gauss
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  34, 4))); }
        public void xmag_SET(float  src) //X Magnetic field (Gauss
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 34); }
        public float ymag_GET()//Y Magnetic field (Gauss
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  38, 4))); }
        public void ymag_SET(float  src) //Y Magnetic field (Gauss
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 38); }
        public float zmag_GET()//Z Magnetic field (Gauss
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  42, 4))); }
        public void zmag_SET(float  src) //Z Magnetic field (Gauss
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 42); }
        public float abs_pressure_GET()//Absolute pressure in milliba
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  46, 4))); }
        public void abs_pressure_SET(float  src) //Absolute pressure in milliba
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 46); }
        public float diff_pressure_GET()//Differential pressure in milliba
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  50, 4))); }
        public void diff_pressure_SET(float  src) //Differential pressure in milliba
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 50); }
        public float pressure_alt_GET()//Altitude calculated from pressur
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  54, 4))); }
        public void pressure_alt_SET(float  src) //Altitude calculated from pressur
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 54); }
        public float temperature_GET()//Temperature in degrees celsiu
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  58, 4))); }
        public void temperature_SET(float  src) //Temperature in degrees celsiu
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 58); }
        static final Meta meta = new Meta(105, 1, 0, 1, 62, 496);
    } public static class OPTICAL_FLOW_RAD extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        OPTICAL_FLOW_RAD() { super(meta, 0); }
        OPTICAL_FLOW_RAD(int bytes) { super(meta, bytes); }
        public long integration_time_us_GET() {  return (get_bytes(data,  0, 4)); }
        public void integration_time_us_SET(long  src) {  set_bytes((src) & -1L, 4, data,  0); }
        public long time_delta_distance_us_GET()//Time in microseconds since the distance was sampled
        {  return (get_bytes(data,  4, 4)); }
        public void time_delta_distance_us_SET(long  src) //Time in microseconds since the distance was sampled
        {  set_bytes((src) & -1L, 4, data,  4); }
        public long time_usec_GET()//Timestamp (microseconds, synced to UNIX time or since system boot
        {  return (get_bytes(data,  8, 8)); }
        public void time_usec_SET(long  src) //Timestamp (microseconds, synced to UNIX time or since system boot
        {  set_bytes((src) & -1L, 8, data,  8); }
        public char sensor_id_GET()//Sensor I
        {  return (char)((char) get_bytes(data,  16, 1)); }
        public void sensor_id_SET(char  src) //Sensor I
        {  set_bytes((char)(src) & -1L, 1, data,  16); }
        public float integrated_x_GET() {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  17, 4))); }
        public void integrated_x_SET(float  src) {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 17); }
        public float integrated_y_GET() {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  21, 4))); }
        public void integrated_y_SET(float  src) {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 21); }
        public float integrated_xgyro_GET()//RH rotation around X axis (rad
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  25, 4))); }
        public void integrated_xgyro_SET(float  src) //RH rotation around X axis (rad
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 25); }
        public float integrated_ygyro_GET()//RH rotation around Y axis (rad
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  29, 4))); }
        public void integrated_ygyro_SET(float  src) //RH rotation around Y axis (rad
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 29); }
        public float integrated_zgyro_GET()//RH rotation around Z axis (rad
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  33, 4))); }
        public void integrated_zgyro_SET(float  src) //RH rotation around Z axis (rad
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 33); }
        public short temperature_GET()//Temperature * 100 in centi-degrees Celsiu
        {  return (short)((short) get_bytes(data,  37, 2)); }
        public void temperature_SET(short  src) //Temperature * 100 in centi-degrees Celsiu
        {  set_bytes((short)(src) & -1L, 2, data,  37); }
        public char quality_GET()//Optical flow quality / confidence. 0: no valid flow, 255: maximum qualit
        {  return (char)((char) get_bytes(data,  39, 1)); }
        public void quality_SET(char  src) //Optical flow quality / confidence. 0: no valid flow, 255: maximum qualit
        {  set_bytes((char)(src) & -1L, 1, data,  39); }
        public float distance_GET() {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  40, 4))); }
        public void distance_SET(float  src) {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 40); }
        static final Meta meta = new Meta(106, 0, 2, 1, 44, 352);
    } public static class HIL_SENSOR extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        HIL_SENSOR() { super(meta, 0); }
        HIL_SENSOR(int bytes) { super(meta, bytes); }
        public long fields_updated_GET() {  return (get_bytes(data,  0, 4)); }
        public void fields_updated_SET(long  src) {  set_bytes((src) & -1L, 4, data,  0); }
        public long time_usec_GET()//Timestamp (microseconds, synced to UNIX time or since system boot
        {  return (get_bytes(data,  4, 8)); }
        public void time_usec_SET(long  src) //Timestamp (microseconds, synced to UNIX time or since system boot
        {  set_bytes((src) & -1L, 8, data,  4); }
        public float xacc_GET()//X acceleration (m/s^2
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public void xacc_SET(float  src) //X acceleration (m/s^2
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public float yacc_GET()//Y acceleration (m/s^2
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public void yacc_SET(float  src) //Y acceleration (m/s^2
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public float zacc_GET()//Z acceleration (m/s^2
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public void zacc_SET(float  src) //Z acceleration (m/s^2
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }
        public float xgyro_GET()//Angular speed around X axis in body frame (rad / sec
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public void xgyro_SET(float  src) //Angular speed around X axis in body frame (rad / sec
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        public float ygyro_GET()//Angular speed around Y axis in body frame (rad / sec
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public void ygyro_SET(float  src) //Angular speed around Y axis in body frame (rad / sec
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
        public float zgyro_GET()//Angular speed around Z axis in body frame (rad / sec
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  32, 4))); }
        public void zgyro_SET(float  src) //Angular speed around Z axis in body frame (rad / sec
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 32); }
        public float xmag_GET()//X Magnetic field (Gauss
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  36, 4))); }
        public void xmag_SET(float  src) //X Magnetic field (Gauss
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 36); }
        public float ymag_GET()//Y Magnetic field (Gauss
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  40, 4))); }
        public void ymag_SET(float  src) //Y Magnetic field (Gauss
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 40); }
        public float zmag_GET()//Z Magnetic field (Gauss
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  44, 4))); }
        public void zmag_SET(float  src) //Z Magnetic field (Gauss
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 44); }
        public float abs_pressure_GET()//Absolute pressure in milliba
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  48, 4))); }
        public void abs_pressure_SET(float  src) //Absolute pressure in milliba
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 48); }
        public float diff_pressure_GET()//Differential pressure (airspeed) in milliba
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  52, 4))); }
        public void diff_pressure_SET(float  src) //Differential pressure (airspeed) in milliba
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 52); }
        public float pressure_alt_GET()//Altitude calculated from pressur
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  56, 4))); }
        public void pressure_alt_SET(float  src) //Altitude calculated from pressur
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 56); }
        public float temperature_GET()//Temperature in degrees celsiu
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  60, 4))); }
        public void temperature_SET(float  src) //Temperature in degrees celsiu
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 60); }
        static final Meta meta = new Meta(107, 0, 1, 1, 64, 512);
    } public static class SIM_STATE extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        SIM_STATE() { super(meta, 0); }
        SIM_STATE(int bytes) { super(meta, bytes); }
        public float q1_GET()//True attitude quaternion component 1, w (1 in null-rotation
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  0, 4))); }
        public void q1_SET(float  src) //True attitude quaternion component 1, w (1 in null-rotation
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 0); }
        public float q2_GET()//True attitude quaternion component 2, x (0 in null-rotation
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public void q2_SET(float  src) //True attitude quaternion component 2, x (0 in null-rotation
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }
        public float q3_GET()//True attitude quaternion component 3, y (0 in null-rotation
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public void q3_SET(float  src) //True attitude quaternion component 3, y (0 in null-rotation
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public float q4_GET()//True attitude quaternion component 4, z (0 in null-rotation
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public void q4_SET(float  src) //True attitude quaternion component 4, z (0 in null-rotation
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public float roll_GET()//Attitude roll expressed as Euler angles, not recommended except for human-readable output
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public void roll_SET(float  src) //Attitude roll expressed as Euler angles, not recommended except for human-readable output
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public float pitch_GET()//Attitude pitch expressed as Euler angles, not recommended except for human-readable output
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public void pitch_SET(float  src) //Attitude pitch expressed as Euler angles, not recommended except for human-readable output
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }
        public float yaw_GET()//Attitude yaw expressed as Euler angles, not recommended except for human-readable output
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public void yaw_SET(float  src) //Attitude yaw expressed as Euler angles, not recommended except for human-readable output
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        public float xacc_GET()//X acceleration m/s/
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public void xacc_SET(float  src) //X acceleration m/s/
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
        public float yacc_GET()//Y acceleration m/s/
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  32, 4))); }
        public void yacc_SET(float  src) //Y acceleration m/s/
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 32); }
        public float zacc_GET()//Z acceleration m/s/
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  36, 4))); }
        public void zacc_SET(float  src) //Z acceleration m/s/
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 36); }
        public float xgyro_GET()//Angular speed around X axis rad/
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  40, 4))); }
        public void xgyro_SET(float  src) //Angular speed around X axis rad/
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 40); }
        public float ygyro_GET()//Angular speed around Y axis rad/
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  44, 4))); }
        public void ygyro_SET(float  src) //Angular speed around Y axis rad/
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 44); }
        public float zgyro_GET()//Angular speed around Z axis rad/
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  48, 4))); }
        public void zgyro_SET(float  src) //Angular speed around Z axis rad/
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 48); }
        public float lat_GET()//Latitude in degree
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  52, 4))); }
        public void lat_SET(float  src) //Latitude in degree
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 52); }
        public float lon_GET()//Longitude in degree
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  56, 4))); }
        public void lon_SET(float  src) //Longitude in degree
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 56); }
        public float alt_GET()//Altitude in meter
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  60, 4))); }
        public void alt_SET(float  src) //Altitude in meter
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 60); }
        public float std_dev_horz_GET()//Horizontal position standard deviatio
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  64, 4))); }
        public void std_dev_horz_SET(float  src) //Horizontal position standard deviatio
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 64); }
        public float std_dev_vert_GET()//Vertical position standard deviatio
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  68, 4))); }
        public void std_dev_vert_SET(float  src) //Vertical position standard deviatio
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 68); }
        public float vn_GET()//True velocity in m/s in NORTH direction in earth-fixed NED fram
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  72, 4))); }
        public void vn_SET(float  src) //True velocity in m/s in NORTH direction in earth-fixed NED fram
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 72); }
        public float ve_GET()//True velocity in m/s in EAST direction in earth-fixed NED fram
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  76, 4))); }
        public void ve_SET(float  src) //True velocity in m/s in EAST direction in earth-fixed NED fram
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 76); }
        public float vd_GET()//True velocity in m/s in DOWN direction in earth-fixed NED fram
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  80, 4))); }
        public void vd_SET(float  src) //True velocity in m/s in DOWN direction in earth-fixed NED fram
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 80); }
        static final Meta meta = new Meta(108, 0, 0, 0, 84, 672);
    } public static class RADIO_STATUS extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        RADIO_STATUS() { super(meta, 0); }
        RADIO_STATUS(int bytes) { super(meta, bytes); }
        public char rxerrors_GET()//Receive error
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void rxerrors_SET(char  src) //Receive error
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char fixed__GET()//Count of error corrected packet
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public void fixed__SET(char  src) //Count of error corrected packet
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public char rssi_GET()//Local signal strengt
        {  return (char)((char) get_bytes(data,  4, 1)); }
        public void rssi_SET(char  src) //Local signal strengt
        {  set_bytes((char)(src) & -1L, 1, data,  4); }
        public char remrssi_GET()//Remote signal strengt
        {  return (char)((char) get_bytes(data,  5, 1)); }
        public void remrssi_SET(char  src) //Remote signal strengt
        {  set_bytes((char)(src) & -1L, 1, data,  5); }
        public char txbuf_GET()//Remaining free buffer space in percent
        {  return (char)((char) get_bytes(data,  6, 1)); }
        public void txbuf_SET(char  src) //Remaining free buffer space in percent
        {  set_bytes((char)(src) & -1L, 1, data,  6); }
        public char noise_GET()//Background noise leve
        {  return (char)((char) get_bytes(data,  7, 1)); }
        public void noise_SET(char  src) //Background noise leve
        {  set_bytes((char)(src) & -1L, 1, data,  7); }
        public char remnoise_GET()//Remote background noise leve
        {  return (char)((char) get_bytes(data,  8, 1)); }
        public void remnoise_SET(char  src) //Remote background noise leve
        {  set_bytes((char)(src) & -1L, 1, data,  8); }
        static final Meta meta = new Meta(109, 2, 0, 0, 9, 72);
    } public static class FILE_TRANSFER_PROTOCOL extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        FILE_TRANSFER_PROTOCOL() { super(meta, 0); }
        FILE_TRANSFER_PROTOCOL(int bytes) { super(meta, bytes); }
        public char target_network_GET()//Network ID (0 for broadcast
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public void target_network_SET(char  src) //Network ID (0 for broadcast
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public char target_system_GET()//System ID (0 for broadcast
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public void target_system_SET(char  src) //System ID (0 for broadcast
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public char target_component_GET()//Component ID (0 for broadcast
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public void target_component_SET(char  src) //Component ID (0 for broadcast
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        public char[] payload_GET(char[]  dst_ch, int pos)
        {
            for(int BYTE = 3, dst_max = pos + 251; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] payload_GET() {return payload_GET(new char[251], 0);} public void payload_SET(char[]  src, int pos)
        {
            for(int BYTE =  3, src_max = pos + 251; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        static final Meta meta = new Meta(110, 0, 0, 0, 254, 2032);
    } public static class TIMESYNC extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        TIMESYNC() { super(meta, 0); }
        TIMESYNC(int bytes) { super(meta, bytes); }
        public long tc1_GET()//Time sync timestamp
        {  return (get_bytes(data,  0, 8)); }
        public void tc1_SET(long  src) //Time sync timestamp
        {  set_bytes((src) & -1L, 8, data,  0); }
        public long ts1_GET()//Time sync timestamp
        {  return (get_bytes(data,  8, 8)); }
        public void ts1_SET(long  src) //Time sync timestamp
        {  set_bytes((src) & -1L, 8, data,  8); }
        static final Meta meta = new Meta(111, 0, 0, 0, 16, 128);
    } public static class CAMERA_TRIGGER extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        CAMERA_TRIGGER() { super(meta, 0); }
        CAMERA_TRIGGER(int bytes) { super(meta, bytes); }
        public long seq_GET()//Image frame sequenc
        {  return (get_bytes(data,  0, 4)); }
        public void seq_SET(long  src) //Image frame sequenc
        {  set_bytes((src) & -1L, 4, data,  0); }
        public long time_usec_GET()//Timestamp for the image frame in microsecond
        {  return (get_bytes(data,  4, 8)); }
        public void time_usec_SET(long  src) //Timestamp for the image frame in microsecond
        {  set_bytes((src) & -1L, 8, data,  4); }
        static final Meta meta = new Meta(112, 0, 1, 1, 12, 96);
    } public static class HIL_GPS extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        HIL_GPS() { super(meta, 0); }
        HIL_GPS(int bytes) { super(meta, bytes); }
        public char eph_GET()//GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 6553
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void eph_SET(char  src) //GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 6553
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char epv_GET()//GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: 6553
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public void epv_SET(char  src) //GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: 6553
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public char vel_GET()//GPS ground speed in cm/s. If unknown, set to: 6553
        {  return (char)((char) get_bytes(data,  4, 2)); }
        public void vel_SET(char  src) //GPS ground speed in cm/s. If unknown, set to: 6553
        {  set_bytes((char)(src) & -1L, 2, data,  4); }
        public char cog_GET() {  return (char)((char) get_bytes(data,  6, 2)); }
        public void cog_SET(char  src) {  set_bytes((char)(src) & -1L, 2, data,  6); }
        public long time_usec_GET()//Timestamp (microseconds since UNIX epoch or microseconds since system boot
        {  return (get_bytes(data,  8, 8)); }
        public void time_usec_SET(long  src) //Timestamp (microseconds since UNIX epoch or microseconds since system boot
        {  set_bytes((src) & -1L, 8, data,  8); }
        public char fix_type_GET() {  return (char)((char) get_bytes(data,  16, 1)); }
        public void fix_type_SET(char  src) {  set_bytes((char)(src) & -1L, 1, data,  16); }
        public int lat_GET()//Latitude (WGS84), in degrees * 1E
        {  return (int)((int) get_bytes(data,  17, 4)); }
        public void lat_SET(int  src) //Latitude (WGS84), in degrees * 1E
        {  set_bytes((int)(src) & -1L, 4, data,  17); }
        public int lon_GET()//Longitude (WGS84), in degrees * 1E
        {  return (int)((int) get_bytes(data,  21, 4)); }
        public void lon_SET(int  src) //Longitude (WGS84), in degrees * 1E
        {  set_bytes((int)(src) & -1L, 4, data,  21); }
        public int alt_GET()//Altitude (AMSL, not WGS84), in meters * 1000 (positive for up
        {  return (int)((int) get_bytes(data,  25, 4)); }
        public void alt_SET(int  src) //Altitude (AMSL, not WGS84), in meters * 1000 (positive for up
        {  set_bytes((int)(src) & -1L, 4, data,  25); }
        public short vn_GET()//GPS velocity in cm/s in NORTH direction in earth-fixed NED fram
        {  return (short)((short) get_bytes(data,  29, 2)); }
        public void vn_SET(short  src) //GPS velocity in cm/s in NORTH direction in earth-fixed NED fram
        {  set_bytes((short)(src) & -1L, 2, data,  29); }
        public short ve_GET()//GPS velocity in cm/s in EAST direction in earth-fixed NED fram
        {  return (short)((short) get_bytes(data,  31, 2)); }
        public void ve_SET(short  src) //GPS velocity in cm/s in EAST direction in earth-fixed NED fram
        {  set_bytes((short)(src) & -1L, 2, data,  31); }
        public short vd_GET()//GPS velocity in cm/s in DOWN direction in earth-fixed NED fram
        {  return (short)((short) get_bytes(data,  33, 2)); }
        public void vd_SET(short  src) //GPS velocity in cm/s in DOWN direction in earth-fixed NED fram
        {  set_bytes((short)(src) & -1L, 2, data,  33); }
        public char satellites_visible_GET()//Number of satellites visible. If unknown, set to 25
        {  return (char)((char) get_bytes(data,  35, 1)); }
        public void satellites_visible_SET(char  src) //Number of satellites visible. If unknown, set to 25
        {  set_bytes((char)(src) & -1L, 1, data,  35); }
        static final Meta meta = new Meta(113, 4, 0, 1, 36, 288);
    } public static class HIL_OPTICAL_FLOW extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        HIL_OPTICAL_FLOW() { super(meta, 0); }
        HIL_OPTICAL_FLOW(int bytes) { super(meta, bytes); }
        public long integration_time_us_GET() {  return (get_bytes(data,  0, 4)); }
        public void integration_time_us_SET(long  src) {  set_bytes((src) & -1L, 4, data,  0); }
        public long time_delta_distance_us_GET()//Time in microseconds since the distance was sampled
        {  return (get_bytes(data,  4, 4)); }
        public void time_delta_distance_us_SET(long  src) //Time in microseconds since the distance was sampled
        {  set_bytes((src) & -1L, 4, data,  4); }
        public long time_usec_GET()//Timestamp (microseconds, synced to UNIX time or since system boot
        {  return (get_bytes(data,  8, 8)); }
        public void time_usec_SET(long  src) //Timestamp (microseconds, synced to UNIX time or since system boot
        {  set_bytes((src) & -1L, 8, data,  8); }
        public char sensor_id_GET()//Sensor I
        {  return (char)((char) get_bytes(data,  16, 1)); }
        public void sensor_id_SET(char  src) //Sensor I
        {  set_bytes((char)(src) & -1L, 1, data,  16); }
        public float integrated_x_GET() {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  17, 4))); }
        public void integrated_x_SET(float  src) {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 17); }
        public float integrated_y_GET() {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  21, 4))); }
        public void integrated_y_SET(float  src) {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 21); }
        public float integrated_xgyro_GET()//RH rotation around X axis (rad
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  25, 4))); }
        public void integrated_xgyro_SET(float  src) //RH rotation around X axis (rad
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 25); }
        public float integrated_ygyro_GET()//RH rotation around Y axis (rad
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  29, 4))); }
        public void integrated_ygyro_SET(float  src) //RH rotation around Y axis (rad
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 29); }
        public float integrated_zgyro_GET()//RH rotation around Z axis (rad
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  33, 4))); }
        public void integrated_zgyro_SET(float  src) //RH rotation around Z axis (rad
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 33); }
        public short temperature_GET()//Temperature * 100 in centi-degrees Celsiu
        {  return (short)((short) get_bytes(data,  37, 2)); }
        public void temperature_SET(short  src) //Temperature * 100 in centi-degrees Celsiu
        {  set_bytes((short)(src) & -1L, 2, data,  37); }
        public char quality_GET()//Optical flow quality / confidence. 0: no valid flow, 255: maximum qualit
        {  return (char)((char) get_bytes(data,  39, 1)); }
        public void quality_SET(char  src) //Optical flow quality / confidence. 0: no valid flow, 255: maximum qualit
        {  set_bytes((char)(src) & -1L, 1, data,  39); }
        public float distance_GET() {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  40, 4))); }
        public void distance_SET(float  src) {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 40); }
        static final Meta meta = new Meta(114, 0, 2, 1, 44, 352);
    } public static class HIL_STATE_QUATERNION extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        HIL_STATE_QUATERNION() { super(meta, 0); }
        HIL_STATE_QUATERNION(int bytes) { super(meta, bytes); }
        public char ind_airspeed_GET()//Indicated airspeed, expressed as cm/
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void ind_airspeed_SET(char  src) //Indicated airspeed, expressed as cm/
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char true_airspeed_GET()//True airspeed, expressed as cm/
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public void true_airspeed_SET(char  src) //True airspeed, expressed as cm/
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public long time_usec_GET()//Timestamp (microseconds since UNIX epoch or microseconds since system boot
        {  return (get_bytes(data,  4, 8)); }
        public void time_usec_SET(long  src) //Timestamp (microseconds since UNIX epoch or microseconds since system boot
        {  set_bytes((src) & -1L, 8, data,  4); }
        public float[] attitude_quaternion_GET(float[]  dst_ch, int pos)  //Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotatio
        {
            for(int BYTE = 12, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] attitude_quaternion_GET()//Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotatio
        {return attitude_quaternion_GET(new float[4], 0);} public void attitude_quaternion_SET(float[]  src, int pos)  //Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotatio
        {
            for(int BYTE =  12, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
                set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
        }
        public float rollspeed_GET()//Body frame roll / phi angular speed (rad/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public void rollspeed_SET(float  src) //Body frame roll / phi angular speed (rad/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
        public float pitchspeed_GET()//Body frame pitch / theta angular speed (rad/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  32, 4))); }
        public void pitchspeed_SET(float  src) //Body frame pitch / theta angular speed (rad/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 32); }
        public float yawspeed_GET()//Body frame yaw / psi angular speed (rad/s
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  36, 4))); }
        public void yawspeed_SET(float  src) //Body frame yaw / psi angular speed (rad/s
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 36); }
        public int lat_GET()//Latitude, expressed as * 1E
        {  return (int)((int) get_bytes(data,  40, 4)); }
        public void lat_SET(int  src) //Latitude, expressed as * 1E
        {  set_bytes((int)(src) & -1L, 4, data,  40); }
        public int lon_GET()//Longitude, expressed as * 1E
        {  return (int)((int) get_bytes(data,  44, 4)); }
        public void lon_SET(int  src) //Longitude, expressed as * 1E
        {  set_bytes((int)(src) & -1L, 4, data,  44); }
        public int alt_GET()//Altitude in meters, expressed as * 1000 (millimeters
        {  return (int)((int) get_bytes(data,  48, 4)); }
        public void alt_SET(int  src) //Altitude in meters, expressed as * 1000 (millimeters
        {  set_bytes((int)(src) & -1L, 4, data,  48); }
        public short vx_GET()//Ground X Speed (Latitude), expressed as cm/
        {  return (short)((short) get_bytes(data,  52, 2)); }
        public void vx_SET(short  src) //Ground X Speed (Latitude), expressed as cm/
        {  set_bytes((short)(src) & -1L, 2, data,  52); }
        public short vy_GET()//Ground Y Speed (Longitude), expressed as cm/
        {  return (short)((short) get_bytes(data,  54, 2)); }
        public void vy_SET(short  src) //Ground Y Speed (Longitude), expressed as cm/
        {  set_bytes((short)(src) & -1L, 2, data,  54); }
        public short vz_GET()//Ground Z Speed (Altitude), expressed as cm/
        {  return (short)((short) get_bytes(data,  56, 2)); }
        public void vz_SET(short  src) //Ground Z Speed (Altitude), expressed as cm/
        {  set_bytes((short)(src) & -1L, 2, data,  56); }
        public short xacc_GET()//X acceleration (mg
        {  return (short)((short) get_bytes(data,  58, 2)); }
        public void xacc_SET(short  src) //X acceleration (mg
        {  set_bytes((short)(src) & -1L, 2, data,  58); }
        public short yacc_GET()//Y acceleration (mg
        {  return (short)((short) get_bytes(data,  60, 2)); }
        public void yacc_SET(short  src) //Y acceleration (mg
        {  set_bytes((short)(src) & -1L, 2, data,  60); }
        public short zacc_GET()//Z acceleration (mg
        {  return (short)((short) get_bytes(data,  62, 2)); }
        public void zacc_SET(short  src) //Z acceleration (mg
        {  set_bytes((short)(src) & -1L, 2, data,  62); }
        static final Meta meta = new Meta(115, 2, 0, 1, 64, 512);
    } public static class SCALED_IMU2 extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        SCALED_IMU2() { super(meta, 0); }
        SCALED_IMU2(int bytes) { super(meta, bytes); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot
        {  return (get_bytes(data,  0, 4)); }
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot
        {  set_bytes((src) & -1L, 4, data,  0); }
        public short xacc_GET()//X acceleration (mg
        {  return (short)((short) get_bytes(data,  4, 2)); }
        public void xacc_SET(short  src) //X acceleration (mg
        {  set_bytes((short)(src) & -1L, 2, data,  4); }
        public short yacc_GET()//Y acceleration (mg
        {  return (short)((short) get_bytes(data,  6, 2)); }
        public void yacc_SET(short  src) //Y acceleration (mg
        {  set_bytes((short)(src) & -1L, 2, data,  6); }
        public short zacc_GET()//Z acceleration (mg
        {  return (short)((short) get_bytes(data,  8, 2)); }
        public void zacc_SET(short  src) //Z acceleration (mg
        {  set_bytes((short)(src) & -1L, 2, data,  8); }
        public short xgyro_GET()//Angular speed around X axis (millirad /sec
        {  return (short)((short) get_bytes(data,  10, 2)); }
        public void xgyro_SET(short  src) //Angular speed around X axis (millirad /sec
        {  set_bytes((short)(src) & -1L, 2, data,  10); }
        public short ygyro_GET()//Angular speed around Y axis (millirad /sec
        {  return (short)((short) get_bytes(data,  12, 2)); }
        public void ygyro_SET(short  src) //Angular speed around Y axis (millirad /sec
        {  set_bytes((short)(src) & -1L, 2, data,  12); }
        public short zgyro_GET()//Angular speed around Z axis (millirad /sec
        {  return (short)((short) get_bytes(data,  14, 2)); }
        public void zgyro_SET(short  src) //Angular speed around Z axis (millirad /sec
        {  set_bytes((short)(src) & -1L, 2, data,  14); }
        public short xmag_GET()//X Magnetic field (milli tesla
        {  return (short)((short) get_bytes(data,  16, 2)); }
        public void xmag_SET(short  src) //X Magnetic field (milli tesla
        {  set_bytes((short)(src) & -1L, 2, data,  16); }
        public short ymag_GET()//Y Magnetic field (milli tesla
        {  return (short)((short) get_bytes(data,  18, 2)); }
        public void ymag_SET(short  src) //Y Magnetic field (milli tesla
        {  set_bytes((short)(src) & -1L, 2, data,  18); }
        public short zmag_GET()//Z Magnetic field (milli tesla
        {  return (short)((short) get_bytes(data,  20, 2)); }
        public void zmag_SET(short  src) //Z Magnetic field (milli tesla
        {  set_bytes((short)(src) & -1L, 2, data,  20); }
        static final Meta meta = new Meta(116, 0, 1, 0, 22, 176);
    } public static class LOG_REQUEST_LIST extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        LOG_REQUEST_LIST() { super(meta, 0); }
        LOG_REQUEST_LIST(int bytes) { super(meta, bytes); }
        public char start_GET()//First log id (0 for first available
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void start_SET(char  src) //First log id (0 for first available
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char end_GET()//Last log id (0xffff for last available
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public void end_SET(char  src) //Last log id (0xffff for last available
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public char target_system_GET()//System I
        {  return (char)((char) get_bytes(data,  4, 1)); }
        public void target_system_SET(char  src) //System I
        {  set_bytes((char)(src) & -1L, 1, data,  4); }
        public char target_component_GET()//Component I
        {  return (char)((char) get_bytes(data,  5, 1)); }
        public void target_component_SET(char  src) //Component I
        {  set_bytes((char)(src) & -1L, 1, data,  5); }
        static final Meta meta = new Meta(117, 2, 0, 0, 6, 48);
    } public static class LOG_ENTRY extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        LOG_ENTRY() { super(meta, 0); }
        LOG_ENTRY(int bytes) { super(meta, bytes); }
        public char id_GET()//Log i
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void id_SET(char  src) //Log i
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char num_logs_GET()//Total number of log
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public void num_logs_SET(char  src) //Total number of log
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public char last_log_num_GET()//High log numbe
        {  return (char)((char) get_bytes(data,  4, 2)); }
        public void last_log_num_SET(char  src) //High log numbe
        {  set_bytes((char)(src) & -1L, 2, data,  4); }
        public long time_utc_GET()//UTC timestamp of log in seconds since 1970, or 0 if not availabl
        {  return (get_bytes(data,  6, 4)); }
        public void time_utc_SET(long  src) //UTC timestamp of log in seconds since 1970, or 0 if not availabl
        {  set_bytes((src) & -1L, 4, data,  6); }
        public long size_GET()//Size of the log (may be approximate) in byte
        {  return (get_bytes(data,  10, 4)); }
        public void size_SET(long  src) //Size of the log (may be approximate) in byte
        {  set_bytes((src) & -1L, 4, data,  10); }
        static final Meta meta = new Meta(118, 3, 2, 0, 14, 112);
    } public static class LOG_REQUEST_DATA extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        LOG_REQUEST_DATA() { super(meta, 0); }
        LOG_REQUEST_DATA(int bytes) { super(meta, bytes); }
        public char id_GET()//Log id (from LOG_ENTRY reply
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void id_SET(char  src) //Log id (from LOG_ENTRY reply
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public long ofs_GET()//Offset into the lo
        {  return (get_bytes(data,  2, 4)); }
        public void ofs_SET(long  src) //Offset into the lo
        {  set_bytes((src) & -1L, 4, data,  2); }
        public long count_GET()//Number of byte
        {  return (get_bytes(data,  6, 4)); }
        public void count_SET(long  src) //Number of byte
        {  set_bytes((src) & -1L, 4, data,  6); }
        public char target_system_GET()//System I
        {  return (char)((char) get_bytes(data,  10, 1)); }
        public void target_system_SET(char  src) //System I
        {  set_bytes((char)(src) & -1L, 1, data,  10); }
        public char target_component_GET()//Component I
        {  return (char)((char) get_bytes(data,  11, 1)); }
        public void target_component_SET(char  src) //Component I
        {  set_bytes((char)(src) & -1L, 1, data,  11); }
        static final Meta meta = new Meta(119, 1, 2, 0, 12, 96);
    } public static class LOG_DATA extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        LOG_DATA() { super(meta, 0); }
        LOG_DATA(int bytes) { super(meta, bytes); }
        public char id_GET()//Log id (from LOG_ENTRY reply
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void id_SET(char  src) //Log id (from LOG_ENTRY reply
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public long ofs_GET()//Offset into the lo
        {  return (get_bytes(data,  2, 4)); }
        public void ofs_SET(long  src) //Offset into the lo
        {  set_bytes((src) & -1L, 4, data,  2); }
        public char count_GET()//Number of bytes (zero for end of log
        {  return (char)((char) get_bytes(data,  6, 1)); }
        public void count_SET(char  src) //Number of bytes (zero for end of log
        {  set_bytes((char)(src) & -1L, 1, data,  6); }
        public char[] data__GET(char[]  dst_ch, int pos)  //log dat
        {
            for(int BYTE = 7, dst_max = pos + 90; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] data__GET()//log dat
        {return data__GET(new char[90], 0);} public void data__SET(char[]  src, int pos)  //log dat
        {
            for(int BYTE =  7, src_max = pos + 90; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        static final Meta meta = new Meta(120, 1, 1, 0, 97, 776);
    } public static class LOG_ERASE extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        LOG_ERASE() { super(meta, 0); }
        LOG_ERASE(int bytes) { super(meta, bytes); }
        public char target_system_GET()//System I
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public void target_system_SET(char  src) //System I
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public char target_component_GET()//Component I
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public void target_component_SET(char  src) //Component I
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        static final Meta meta = new Meta(121, 0, 0, 0, 2, 16);
    } public static class LOG_REQUEST_END extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        LOG_REQUEST_END() { super(meta, 0); }
        LOG_REQUEST_END(int bytes) { super(meta, bytes); }
        public char target_system_GET()//System I
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public void target_system_SET(char  src) //System I
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public char target_component_GET()//Component I
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public void target_component_SET(char  src) //Component I
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        static final Meta meta = new Meta(122, 0, 0, 0, 2, 16);
    } public static class GPS_INJECT_DATA extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        GPS_INJECT_DATA() { super(meta, 0); }
        GPS_INJECT_DATA(int bytes) { super(meta, bytes); }
        public char target_system_GET()//System I
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public void target_system_SET(char  src) //System I
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public char target_component_GET()//Component I
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public void target_component_SET(char  src) //Component I
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public char len_GET()//data lengt
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public void len_SET(char  src) //data lengt
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        public char[] data__GET(char[]  dst_ch, int pos)  //raw data (110 is enough for 12 satellites of RTCMv2
        {
            for(int BYTE = 3, dst_max = pos + 110; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] data__GET()//raw data (110 is enough for 12 satellites of RTCMv2
        {return data__GET(new char[110], 0);} public void data__SET(char[]  src, int pos)  //raw data (110 is enough for 12 satellites of RTCMv2
        {
            for(int BYTE =  3, src_max = pos + 110; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        static final Meta meta = new Meta(123, 0, 0, 0, 113, 904);
    } public static class GPS2_RAW extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        GPS2_RAW() { super(meta, 0); }
        GPS2_RAW(int bytes) { super(meta, bytes); }
        public char eph_GET()//GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: UINT16_MA
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void eph_SET(char  src) //GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: UINT16_MA
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char epv_GET()//GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: UINT16_MA
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public void epv_SET(char  src) //GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: UINT16_MA
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public char vel_GET()//GPS ground speed (m/s * 100). If unknown, set to: UINT16_MA
        {  return (char)((char) get_bytes(data,  4, 2)); }
        public void vel_SET(char  src) //GPS ground speed (m/s * 100). If unknown, set to: UINT16_MA
        {  set_bytes((char)(src) & -1L, 2, data,  4); }
        public char cog_GET() {  return (char)((char) get_bytes(data,  6, 2)); }
        public void cog_SET(char  src) {  set_bytes((char)(src) & -1L, 2, data,  6); }
        public long dgps_age_GET()//Age of DGPS inf
        {  return (get_bytes(data,  8, 4)); }
        public void dgps_age_SET(long  src) //Age of DGPS inf
        {  set_bytes((src) & -1L, 4, data,  8); }
        public long time_usec_GET()//Timestamp (microseconds since UNIX epoch or microseconds since system boot
        {  return (get_bytes(data,  12, 8)); }
        public void time_usec_SET(long  src) //Timestamp (microseconds since UNIX epoch or microseconds since system boot
        {  set_bytes((src) & -1L, 8, data,  12); }
        public int lat_GET()//Latitude (WGS84), in degrees * 1E
        {  return (int)((int) get_bytes(data,  20, 4)); }
        public void lat_SET(int  src) //Latitude (WGS84), in degrees * 1E
        {  set_bytes((int)(src) & -1L, 4, data,  20); }
        public int lon_GET()//Longitude (WGS84), in degrees * 1E
        {  return (int)((int) get_bytes(data,  24, 4)); }
        public void lon_SET(int  src) //Longitude (WGS84), in degrees * 1E
        {  set_bytes((int)(src) & -1L, 4, data,  24); }
        public int alt_GET()//Altitude (AMSL, not WGS84), in meters * 1000 (positive for up
        {  return (int)((int) get_bytes(data,  28, 4)); }
        public void alt_SET(int  src) //Altitude (AMSL, not WGS84), in meters * 1000 (positive for up
        {  set_bytes((int)(src) & -1L, 4, data,  28); }
        public char satellites_visible_GET()//Number of satellites visible. If unknown, set to 25
        {  return (char)((char) get_bytes(data,  32, 1)); }
        public void satellites_visible_SET(char  src) //Number of satellites visible. If unknown, set to 25
        {  set_bytes((char)(src) & -1L, 1, data,  32); }
        public char dgps_numch_GET()//Number of DGPS satellite
        {  return (char)((char) get_bytes(data,  33, 1)); }
        public void dgps_numch_SET(char  src) //Number of DGPS satellite
        {  set_bytes((char)(src) & -1L, 1, data,  33); }
        public @GPS_FIX_TYPE int fix_type_GET()//See the GPS_FIX_TYPE enum
        {  return  0 + (int)get_bits(data, 272, 4); }
        public void fix_type_SET(@GPS_FIX_TYPE int  src) //See the GPS_FIX_TYPE enum
        {  set_bits(- 0 +   src, 4, data, 272); }
        static final Meta meta = new Meta(124, 4, 1, 1, 35, 276);
    } public static class POWER_STATUS extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        POWER_STATUS() { super(meta, 0); }
        POWER_STATUS(int bytes) { super(meta, bytes); }
        public char Vcc_GET()//5V rail voltage in millivolt
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void Vcc_SET(char  src) //5V rail voltage in millivolt
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char Vservo_GET()//servo rail voltage in millivolt
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public void Vservo_SET(char  src) //servo rail voltage in millivolt
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public @MAV_POWER_STATUS int flags_GET()//power supply status flags (see MAV_POWER_STATUS enum
        {
            switch((int)get_bits(data, 32, 3))
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
            assert(false);//("Unknown enum ID " + id);
            return  Integer.MIN_VALUE;
        }
        public void flags_SET(@MAV_POWER_STATUS int  src) //power supply status flags (see MAV_POWER_STATUS enum
        {
            long id = 0;
            switch(src)
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
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 3, data, 32);
        }
        static final Meta meta = new Meta(125, 2, 0, 0, 5, 35);
    } public static class SERIAL_CONTROL extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        SERIAL_CONTROL() { super(meta, 0); }
        SERIAL_CONTROL(int bytes) { super(meta, bytes); }
        public char timeout_GET()//Timeout for reply data in millisecond
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void timeout_SET(char  src) //Timeout for reply data in millisecond
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public long baudrate_GET()//Baudrate of transfer. Zero means no change
        {  return (get_bytes(data,  2, 4)); }
        public void baudrate_SET(long  src) //Baudrate of transfer. Zero means no change
        {  set_bytes((src) & -1L, 4, data,  2); }
        public char count_GET()//how many bytes in this transfe
        {  return (char)((char) get_bytes(data,  6, 1)); }
        public void count_SET(char  src) //how many bytes in this transfe
        {  set_bytes((char)(src) & -1L, 1, data,  6); }
        public char[] data__GET(char[]  dst_ch, int pos)  //serial dat
        {
            for(int BYTE = 7, dst_max = pos + 70; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] data__GET()//serial dat
        {return data__GET(new char[70], 0);} public void data__SET(char[]  src, int pos)  //serial dat
        {
            for(int BYTE =  7, src_max = pos + 70; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        public @SERIAL_CONTROL_DEV int device_GET()//See SERIAL_CONTROL_DEV enu
        {
            switch((int)get_bits(data, 616, 3))
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
            assert(false);//("Unknown enum ID " + id);
            return  Integer.MIN_VALUE;
        }
        public void device_SET(@SERIAL_CONTROL_DEV int  src) //See SERIAL_CONTROL_DEV enu
        {
            long id = 0;
            switch(src)
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
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 3, data, 616);
        }
        public @SERIAL_CONTROL_FLAG int flags_GET()//See SERIAL_CONTROL_FLAG enu
        {
            switch((int)get_bits(data, 619, 3))
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
            assert(false);//("Unknown enum ID " + id);
            return  Integer.MIN_VALUE;
        }
        public void flags_SET(@SERIAL_CONTROL_FLAG int  src) //See SERIAL_CONTROL_FLAG enu
        {
            long id = 0;
            switch(src)
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
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 3, data, 619);
        }
        static final Meta meta = new Meta(126, 1, 1, 0, 78, 622);
    } public static class GPS_RTK extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        GPS_RTK() { super(meta, 0); }
        GPS_RTK(int bytes) { super(meta, bytes); }
        public char wn_GET()//GPS Week Number of last baselin
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void wn_SET(char  src) //GPS Week Number of last baselin
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public long time_last_baseline_ms_GET()//Time since boot of last baseline message received in ms
        {  return (get_bytes(data,  2, 4)); }
        public void time_last_baseline_ms_SET(long  src) //Time since boot of last baseline message received in ms
        {  set_bytes((src) & -1L, 4, data,  2); }
        public long tow_GET()//GPS Time of Week of last baselin
        {  return (get_bytes(data,  6, 4)); }
        public void tow_SET(long  src) //GPS Time of Week of last baselin
        {  set_bytes((src) & -1L, 4, data,  6); }
        public long accuracy_GET()//Current estimate of baseline accuracy
        {  return (get_bytes(data,  10, 4)); }
        public void accuracy_SET(long  src) //Current estimate of baseline accuracy
        {  set_bytes((src) & -1L, 4, data,  10); }
        public char rtk_receiver_id_GET()//Identification of connected RTK receiver
        {  return (char)((char) get_bytes(data,  14, 1)); }
        public void rtk_receiver_id_SET(char  src) //Identification of connected RTK receiver
        {  set_bytes((char)(src) & -1L, 1, data,  14); }
        public char rtk_health_GET()//GPS-specific health report for RTK data
        {  return (char)((char) get_bytes(data,  15, 1)); }
        public void rtk_health_SET(char  src) //GPS-specific health report for RTK data
        {  set_bytes((char)(src) & -1L, 1, data,  15); }
        public char rtk_rate_GET()//Rate of baseline messages being received by GPS, in H
        {  return (char)((char) get_bytes(data,  16, 1)); }
        public void rtk_rate_SET(char  src) //Rate of baseline messages being received by GPS, in H
        {  set_bytes((char)(src) & -1L, 1, data,  16); }
        public char nsats_GET()//Current number of sats used for RTK calculation
        {  return (char)((char) get_bytes(data,  17, 1)); }
        public void nsats_SET(char  src) //Current number of sats used for RTK calculation
        {  set_bytes((char)(src) & -1L, 1, data,  17); }
        public char baseline_coords_type_GET()//Coordinate system of baseline. 0 == ECEF, 1 == NE
        {  return (char)((char) get_bytes(data,  18, 1)); }
        public void baseline_coords_type_SET(char  src) //Coordinate system of baseline. 0 == ECEF, 1 == NE
        {  set_bytes((char)(src) & -1L, 1, data,  18); }
        public int baseline_a_mm_GET()//Current baseline in ECEF x or NED north component in mm
        {  return (int)((int) get_bytes(data,  19, 4)); }
        public void baseline_a_mm_SET(int  src) //Current baseline in ECEF x or NED north component in mm
        {  set_bytes((int)(src) & -1L, 4, data,  19); }
        public int baseline_b_mm_GET()//Current baseline in ECEF y or NED east component in mm
        {  return (int)((int) get_bytes(data,  23, 4)); }
        public void baseline_b_mm_SET(int  src) //Current baseline in ECEF y or NED east component in mm
        {  set_bytes((int)(src) & -1L, 4, data,  23); }
        public int baseline_c_mm_GET()//Current baseline in ECEF z or NED down component in mm
        {  return (int)((int) get_bytes(data,  27, 4)); }
        public void baseline_c_mm_SET(int  src) //Current baseline in ECEF z or NED down component in mm
        {  set_bytes((int)(src) & -1L, 4, data,  27); }
        public int iar_num_hypotheses_GET()//Current number of integer ambiguity hypotheses
        {  return (int)((int) get_bytes(data,  31, 4)); }
        public void iar_num_hypotheses_SET(int  src) //Current number of integer ambiguity hypotheses
        {  set_bytes((int)(src) & -1L, 4, data,  31); }
        static final Meta meta = new Meta(127, 1, 3, 0, 35, 280);
    } public static class GPS2_RTK extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        GPS2_RTK() { super(meta, 0); }
        GPS2_RTK(int bytes) { super(meta, bytes); }
        public char wn_GET()//GPS Week Number of last baselin
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void wn_SET(char  src) //GPS Week Number of last baselin
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public long time_last_baseline_ms_GET()//Time since boot of last baseline message received in ms
        {  return (get_bytes(data,  2, 4)); }
        public void time_last_baseline_ms_SET(long  src) //Time since boot of last baseline message received in ms
        {  set_bytes((src) & -1L, 4, data,  2); }
        public long tow_GET()//GPS Time of Week of last baselin
        {  return (get_bytes(data,  6, 4)); }
        public void tow_SET(long  src) //GPS Time of Week of last baselin
        {  set_bytes((src) & -1L, 4, data,  6); }
        public long accuracy_GET()//Current estimate of baseline accuracy
        {  return (get_bytes(data,  10, 4)); }
        public void accuracy_SET(long  src) //Current estimate of baseline accuracy
        {  set_bytes((src) & -1L, 4, data,  10); }
        public char rtk_receiver_id_GET()//Identification of connected RTK receiver
        {  return (char)((char) get_bytes(data,  14, 1)); }
        public void rtk_receiver_id_SET(char  src) //Identification of connected RTK receiver
        {  set_bytes((char)(src) & -1L, 1, data,  14); }
        public char rtk_health_GET()//GPS-specific health report for RTK data
        {  return (char)((char) get_bytes(data,  15, 1)); }
        public void rtk_health_SET(char  src) //GPS-specific health report for RTK data
        {  set_bytes((char)(src) & -1L, 1, data,  15); }
        public char rtk_rate_GET()//Rate of baseline messages being received by GPS, in H
        {  return (char)((char) get_bytes(data,  16, 1)); }
        public void rtk_rate_SET(char  src) //Rate of baseline messages being received by GPS, in H
        {  set_bytes((char)(src) & -1L, 1, data,  16); }
        public char nsats_GET()//Current number of sats used for RTK calculation
        {  return (char)((char) get_bytes(data,  17, 1)); }
        public void nsats_SET(char  src) //Current number of sats used for RTK calculation
        {  set_bytes((char)(src) & -1L, 1, data,  17); }
        public char baseline_coords_type_GET()//Coordinate system of baseline. 0 == ECEF, 1 == NE
        {  return (char)((char) get_bytes(data,  18, 1)); }
        public void baseline_coords_type_SET(char  src) //Coordinate system of baseline. 0 == ECEF, 1 == NE
        {  set_bytes((char)(src) & -1L, 1, data,  18); }
        public int baseline_a_mm_GET()//Current baseline in ECEF x or NED north component in mm
        {  return (int)((int) get_bytes(data,  19, 4)); }
        public void baseline_a_mm_SET(int  src) //Current baseline in ECEF x or NED north component in mm
        {  set_bytes((int)(src) & -1L, 4, data,  19); }
        public int baseline_b_mm_GET()//Current baseline in ECEF y or NED east component in mm
        {  return (int)((int) get_bytes(data,  23, 4)); }
        public void baseline_b_mm_SET(int  src) //Current baseline in ECEF y or NED east component in mm
        {  set_bytes((int)(src) & -1L, 4, data,  23); }
        public int baseline_c_mm_GET()//Current baseline in ECEF z or NED down component in mm
        {  return (int)((int) get_bytes(data,  27, 4)); }
        public void baseline_c_mm_SET(int  src) //Current baseline in ECEF z or NED down component in mm
        {  set_bytes((int)(src) & -1L, 4, data,  27); }
        public int iar_num_hypotheses_GET()//Current number of integer ambiguity hypotheses
        {  return (int)((int) get_bytes(data,  31, 4)); }
        public void iar_num_hypotheses_SET(int  src) //Current number of integer ambiguity hypotheses
        {  set_bytes((int)(src) & -1L, 4, data,  31); }
        static final Meta meta = new Meta(128, 1, 3, 0, 35, 280);
    } public static class SCALED_IMU3 extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        SCALED_IMU3() { super(meta, 0); }
        SCALED_IMU3(int bytes) { super(meta, bytes); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot
        {  return (get_bytes(data,  0, 4)); }
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot
        {  set_bytes((src) & -1L, 4, data,  0); }
        public short xacc_GET()//X acceleration (mg
        {  return (short)((short) get_bytes(data,  4, 2)); }
        public void xacc_SET(short  src) //X acceleration (mg
        {  set_bytes((short)(src) & -1L, 2, data,  4); }
        public short yacc_GET()//Y acceleration (mg
        {  return (short)((short) get_bytes(data,  6, 2)); }
        public void yacc_SET(short  src) //Y acceleration (mg
        {  set_bytes((short)(src) & -1L, 2, data,  6); }
        public short zacc_GET()//Z acceleration (mg
        {  return (short)((short) get_bytes(data,  8, 2)); }
        public void zacc_SET(short  src) //Z acceleration (mg
        {  set_bytes((short)(src) & -1L, 2, data,  8); }
        public short xgyro_GET()//Angular speed around X axis (millirad /sec
        {  return (short)((short) get_bytes(data,  10, 2)); }
        public void xgyro_SET(short  src) //Angular speed around X axis (millirad /sec
        {  set_bytes((short)(src) & -1L, 2, data,  10); }
        public short ygyro_GET()//Angular speed around Y axis (millirad /sec
        {  return (short)((short) get_bytes(data,  12, 2)); }
        public void ygyro_SET(short  src) //Angular speed around Y axis (millirad /sec
        {  set_bytes((short)(src) & -1L, 2, data,  12); }
        public short zgyro_GET()//Angular speed around Z axis (millirad /sec
        {  return (short)((short) get_bytes(data,  14, 2)); }
        public void zgyro_SET(short  src) //Angular speed around Z axis (millirad /sec
        {  set_bytes((short)(src) & -1L, 2, data,  14); }
        public short xmag_GET()//X Magnetic field (milli tesla
        {  return (short)((short) get_bytes(data,  16, 2)); }
        public void xmag_SET(short  src) //X Magnetic field (milli tesla
        {  set_bytes((short)(src) & -1L, 2, data,  16); }
        public short ymag_GET()//Y Magnetic field (milli tesla
        {  return (short)((short) get_bytes(data,  18, 2)); }
        public void ymag_SET(short  src) //Y Magnetic field (milli tesla
        {  set_bytes((short)(src) & -1L, 2, data,  18); }
        public short zmag_GET()//Z Magnetic field (milli tesla
        {  return (short)((short) get_bytes(data,  20, 2)); }
        public void zmag_SET(short  src) //Z Magnetic field (milli tesla
        {  set_bytes((short)(src) & -1L, 2, data,  20); }
        static final Meta meta = new Meta(129, 0, 1, 0, 22, 176);
    } public static class DATA_TRANSMISSION_HANDSHAKE extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        DATA_TRANSMISSION_HANDSHAKE() { super(meta, 0); }
        DATA_TRANSMISSION_HANDSHAKE(int bytes) { super(meta, bytes); }
        public char width_GET()//Width of a matrix or imag
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void width_SET(char  src) //Width of a matrix or imag
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char height_GET()//Height of a matrix or imag
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public void height_SET(char  src) //Height of a matrix or imag
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public char packets_GET()//number of packets beeing sent (set on ACK only
        {  return (char)((char) get_bytes(data,  4, 2)); }
        public void packets_SET(char  src) //number of packets beeing sent (set on ACK only
        {  set_bytes((char)(src) & -1L, 2, data,  4); }
        public long size_GET()//total data size in bytes (set on ACK only
        {  return (get_bytes(data,  6, 4)); }
        public void size_SET(long  src) //total data size in bytes (set on ACK only
        {  set_bytes((src) & -1L, 4, data,  6); }
        public char type_GET()//type of requested/acknowledged data (as defined in ENUM DATA_TYPES in mavlink/include/mavlink_types.
        {  return (char)((char) get_bytes(data,  10, 1)); }
        public void type_SET(char  src) //type of requested/acknowledged data (as defined in ENUM DATA_TYPES in mavlink/include/mavlink_types.
        {  set_bytes((char)(src) & -1L, 1, data,  10); }
        public char payload_GET() {  return (char)((char) get_bytes(data,  11, 1)); }
        public void payload_SET(char  src) {  set_bytes((char)(src) & -1L, 1, data,  11); }
        public char jpg_quality_GET()//JPEG quality out of [1,100
        {  return (char)((char) get_bytes(data,  12, 1)); }
        public void jpg_quality_SET(char  src) //JPEG quality out of [1,100
        {  set_bytes((char)(src) & -1L, 1, data,  12); }
        static final Meta meta = new Meta(130, 3, 1, 0, 13, 104);
    } public static class ENCAPSULATED_DATA extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        ENCAPSULATED_DATA() { super(meta, 0); }
        ENCAPSULATED_DATA(int bytes) { super(meta, bytes); }
        public char seqnr_GET()//sequence number (starting with 0 on every transmission
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void seqnr_SET(char  src) //sequence number (starting with 0 on every transmission
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char[] data__GET(char[]  dst_ch, int pos)  //image data byte
        {
            for(int BYTE = 2, dst_max = pos + 253; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] data__GET()//image data byte
        {return data__GET(new char[253], 0);} public void data__SET(char[]  src, int pos)  //image data byte
        {
            for(int BYTE =  2, src_max = pos + 253; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        static final Meta meta = new Meta(131, 1, 0, 0, 255, 2040);
    } public static class DISTANCE_SENSOR extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        DISTANCE_SENSOR() { super(meta, 0); }
        DISTANCE_SENSOR(int bytes) { super(meta, bytes); }
        public char min_distance_GET()//Minimum distance the sensor can measure in centimeter
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void min_distance_SET(char  src) //Minimum distance the sensor can measure in centimeter
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char max_distance_GET()//Maximum distance the sensor can measure in centimeter
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public void max_distance_SET(char  src) //Maximum distance the sensor can measure in centimeter
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public char current_distance_GET()//Current distance readin
        {  return (char)((char) get_bytes(data,  4, 2)); }
        public void current_distance_SET(char  src) //Current distance readin
        {  set_bytes((char)(src) & -1L, 2, data,  4); }
        public long time_boot_ms_GET()//Time since system boo
        {  return (get_bytes(data,  6, 4)); }
        public void time_boot_ms_SET(long  src) //Time since system boo
        {  set_bytes((src) & -1L, 4, data,  6); }
        public char id_GET()//Onboard ID of the senso
        {  return (char)((char) get_bytes(data,  10, 1)); }
        public void id_SET(char  src) //Onboard ID of the senso
        {  set_bytes((char)(src) & -1L, 1, data,  10); }
        public char covariance_GET()//Measurement covariance in centimeters, 0 for unknown / invalid reading
        {  return (char)((char) get_bytes(data,  11, 1)); }
        public void covariance_SET(char  src) //Measurement covariance in centimeters, 0 for unknown / invalid reading
        {  set_bytes((char)(src) & -1L, 1, data,  11); }
        public @MAV_DISTANCE_SENSOR int type_GET()//Type from MAV_DISTANCE_SENSOR enum
        {  return  0 + (int)get_bits(data, 96, 3); }
        public void type_SET(@MAV_DISTANCE_SENSOR int  src) //Type from MAV_DISTANCE_SENSOR enum
        {  set_bits(- 0 +   src, 3, data, 96); }
        public @MAV_SENSOR_ORIENTATION int orientation_GET() {  return  0 + (int)get_bits(data, 99, 6); }
        public void orientation_SET(@MAV_SENSOR_ORIENTATION int  src) {  set_bits(- 0 +   src, 6, data, 99); }
        static final Meta meta = new Meta(132, 3, 1, 0, 14, 105);
    } public static class TERRAIN_REQUEST extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        TERRAIN_REQUEST() { super(meta, 0); }
        TERRAIN_REQUEST(int bytes) { super(meta, bytes); }
        public char grid_spacing_GET()//Grid spacing in meter
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void grid_spacing_SET(char  src) //Grid spacing in meter
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public long mask_GET()//Bitmask of requested 4x4 grids (row major 8x7 array of grids, 56 bits
        {  return (get_bytes(data,  2, 8)); }
        public void mask_SET(long  src) //Bitmask of requested 4x4 grids (row major 8x7 array of grids, 56 bits
        {  set_bytes((src) & -1L, 8, data,  2); }
        public int lat_GET()//Latitude of SW corner of first grid (degrees *10^7
        {  return (int)((int) get_bytes(data,  10, 4)); }
        public void lat_SET(int  src) //Latitude of SW corner of first grid (degrees *10^7
        {  set_bytes((int)(src) & -1L, 4, data,  10); }
        public int lon_GET()//Longitude of SW corner of first grid (in degrees *10^7
        {  return (int)((int) get_bytes(data,  14, 4)); }
        public void lon_SET(int  src) //Longitude of SW corner of first grid (in degrees *10^7
        {  set_bytes((int)(src) & -1L, 4, data,  14); }
        static final Meta meta = new Meta(133, 1, 0, 1, 18, 144);
    } public static class TERRAIN_DATA extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        TERRAIN_DATA() { super(meta, 0); }
        TERRAIN_DATA(int bytes) { super(meta, bytes); }
        public char grid_spacing_GET()//Grid spacing in meter
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void grid_spacing_SET(char  src) //Grid spacing in meter
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public int lat_GET()//Latitude of SW corner of first grid (degrees *10^7
        {  return (int)((int) get_bytes(data,  2, 4)); }
        public void lat_SET(int  src) //Latitude of SW corner of first grid (degrees *10^7
        {  set_bytes((int)(src) & -1L, 4, data,  2); }
        public int lon_GET()//Longitude of SW corner of first grid (in degrees *10^7
        {  return (int)((int) get_bytes(data,  6, 4)); }
        public void lon_SET(int  src) //Longitude of SW corner of first grid (in degrees *10^7
        {  set_bytes((int)(src) & -1L, 4, data,  6); }
        public char gridbit_GET()//bit within the terrain request mas
        {  return (char)((char) get_bytes(data,  10, 1)); }
        public void gridbit_SET(char  src) //bit within the terrain request mas
        {  set_bytes((char)(src) & -1L, 1, data,  10); }
        public short[] data__GET(short[]  dst_ch, int pos)  //Terrain data in meters AMS
        {
            for(int BYTE = 11, dst_max = pos + 16; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (short)((short) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public short[] data__GET()//Terrain data in meters AMS
        {return data__GET(new short[16], 0);} public void data__SET(short[]  src, int pos)  //Terrain data in meters AMS
        {
            for(int BYTE =  11, src_max = pos + 16; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        }
        static final Meta meta = new Meta(134, 1, 0, 0, 43, 344);
    } public static class TERRAIN_CHECK extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        TERRAIN_CHECK() { super(meta, 0); }
        TERRAIN_CHECK(int bytes) { super(meta, bytes); }
        public int lat_GET()//Latitude (degrees *10^7
        {  return (int)((int) get_bytes(data,  0, 4)); }
        public void lat_SET(int  src) //Latitude (degrees *10^7
        {  set_bytes((int)(src) & -1L, 4, data,  0); }
        public int lon_GET()//Longitude (degrees *10^7
        {  return (int)((int) get_bytes(data,  4, 4)); }
        public void lon_SET(int  src) //Longitude (degrees *10^7
        {  set_bytes((int)(src) & -1L, 4, data,  4); }
        static final Meta meta = new Meta(135, 0, 0, 0, 8, 64);
    } public static class TERRAIN_REPORT extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        TERRAIN_REPORT() { super(meta, 0); }
        TERRAIN_REPORT(int bytes) { super(meta, bytes); }
        public char spacing_GET()//grid spacing (zero if terrain at this location unavailable
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void spacing_SET(char  src) //grid spacing (zero if terrain at this location unavailable
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char pending_GET()//Number of 4x4 terrain blocks waiting to be received or read from dis
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public void pending_SET(char  src) //Number of 4x4 terrain blocks waiting to be received or read from dis
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public char loaded_GET()//Number of 4x4 terrain blocks in memor
        {  return (char)((char) get_bytes(data,  4, 2)); }
        public void loaded_SET(char  src) //Number of 4x4 terrain blocks in memor
        {  set_bytes((char)(src) & -1L, 2, data,  4); }
        public int lat_GET()//Latitude (degrees *10^7
        {  return (int)((int) get_bytes(data,  6, 4)); }
        public void lat_SET(int  src) //Latitude (degrees *10^7
        {  set_bytes((int)(src) & -1L, 4, data,  6); }
        public int lon_GET()//Longitude (degrees *10^7
        {  return (int)((int) get_bytes(data,  10, 4)); }
        public void lon_SET(int  src) //Longitude (degrees *10^7
        {  set_bytes((int)(src) & -1L, 4, data,  10); }
        public float terrain_height_GET()//Terrain height in meters AMS
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  14, 4))); }
        public void terrain_height_SET(float  src) //Terrain height in meters AMS
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 14); }
        public float current_height_GET()//Current vehicle height above lat/lon terrain height (meters
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  18, 4))); }
        public void current_height_SET(float  src) //Current vehicle height above lat/lon terrain height (meters
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 18); }
        static final Meta meta = new Meta(136, 3, 0, 0, 22, 176);
    } public static class SCALED_PRESSURE2 extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        SCALED_PRESSURE2() { super(meta, 0); }
        SCALED_PRESSURE2(int bytes) { super(meta, bytes); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot
        {  return (get_bytes(data,  0, 4)); }
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot
        {  set_bytes((src) & -1L, 4, data,  0); }
        public float press_abs_GET()//Absolute pressure (hectopascal
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public void press_abs_SET(float  src) //Absolute pressure (hectopascal
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }
        public float press_diff_GET()//Differential pressure 1 (hectopascal
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public void press_diff_SET(float  src) //Differential pressure 1 (hectopascal
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public short temperature_GET()//Temperature measurement (0.01 degrees celsius
        {  return (short)((short) get_bytes(data,  12, 2)); }
        public void temperature_SET(short  src) //Temperature measurement (0.01 degrees celsius
        {  set_bytes((short)(src) & -1L, 2, data,  12); }
        static final Meta meta = new Meta(137, 0, 1, 0, 14, 112);
    } public static class ATT_POS_MOCAP extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        ATT_POS_MOCAP() { super(meta, 0); }
        ATT_POS_MOCAP(int bytes) { super(meta, bytes); }
        public long time_usec_GET()//Timestamp (micros since boot or Unix epoch
        {  return (get_bytes(data,  0, 8)); }
        public void time_usec_SET(long  src) //Timestamp (micros since boot or Unix epoch
        {  set_bytes((src) & -1L, 8, data,  0); }
        public float[] q_GET(float[]  dst_ch, int pos)  //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0
        {
            for(int BYTE = 8, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] q_GET()//Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0
        {return q_GET(new float[4], 0);} public void q_SET(float[]  src, int pos)  //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0
        {
            for(int BYTE =  8, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
                set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
        }
        public float x_GET()//X position in meters (NED
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public void x_SET(float  src) //X position in meters (NED
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        public float y_GET()//Y position in meters (NED
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public void y_SET(float  src) //Y position in meters (NED
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
        public float z_GET()//Z position in meters (NED
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  32, 4))); }
        public void z_SET(float  src) //Z position in meters (NED
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 32); }
        static final Meta meta = new Meta(138, 0, 0, 1, 36, 288);
    } public static class SET_ACTUATOR_CONTROL_TARGET extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        SET_ACTUATOR_CONTROL_TARGET() { super(meta, 0); }
        SET_ACTUATOR_CONTROL_TARGET(int bytes) { super(meta, bytes); }
        public long time_usec_GET()//Timestamp (micros since boot or Unix epoch
        {  return (get_bytes(data,  0, 8)); }
        public void time_usec_SET(long  src) //Timestamp (micros since boot or Unix epoch
        {  set_bytes((src) & -1L, 8, data,  0); }
        public char group_mlx_GET() {  return (char)((char) get_bytes(data,  8, 1)); }
        public void group_mlx_SET(char  src) {  set_bytes((char)(src) & -1L, 1, data,  8); }
        public char target_system_GET()//System I
        {  return (char)((char) get_bytes(data,  9, 1)); }
        public void target_system_SET(char  src) //System I
        {  set_bytes((char)(src) & -1L, 1, data,  9); }
        public char target_component_GET()//Component I
        {  return (char)((char) get_bytes(data,  10, 1)); }
        public void target_component_SET(char  src) //Component I
        {  set_bytes((char)(src) & -1L, 1, data,  10); }
        public float[] controls_GET(float[]  dst_ch, int pos)
        {
            for(int BYTE = 11, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] controls_GET() {return controls_GET(new float[8], 0);} public void controls_SET(float[]  src, int pos)
        {
            for(int BYTE =  11, src_max = pos + 8; pos < src_max; pos++, BYTE += 4)
                set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
        }
        static final Meta meta = new Meta(139, 0, 0, 1, 43, 344);
    } public static class ACTUATOR_CONTROL_TARGET extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        ACTUATOR_CONTROL_TARGET() { super(meta, 0); }
        ACTUATOR_CONTROL_TARGET(int bytes) { super(meta, bytes); }
        public long time_usec_GET()//Timestamp (micros since boot or Unix epoch
        {  return (get_bytes(data,  0, 8)); }
        public void time_usec_SET(long  src) //Timestamp (micros since boot or Unix epoch
        {  set_bytes((src) & -1L, 8, data,  0); }
        public char group_mlx_GET() {  return (char)((char) get_bytes(data,  8, 1)); }
        public void group_mlx_SET(char  src) {  set_bytes((char)(src) & -1L, 1, data,  8); }
        public float[] controls_GET(float[]  dst_ch, int pos)
        {
            for(int BYTE = 9, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] controls_GET() {return controls_GET(new float[8], 0);} public void controls_SET(float[]  src, int pos)
        {
            for(int BYTE =  9, src_max = pos + 8; pos < src_max; pos++, BYTE += 4)
                set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
        }
        static final Meta meta = new Meta(140, 0, 0, 1, 41, 328);
    } public static class ALTITUDE extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        ALTITUDE() { super(meta, 0); }
        ALTITUDE(int bytes) { super(meta, bytes); }
        public long time_usec_GET()//Timestamp (micros since boot or Unix epoch
        {  return (get_bytes(data,  0, 8)); }
        public void time_usec_SET(long  src) //Timestamp (micros since boot or Unix epoch
        {  set_bytes((src) & -1L, 8, data,  0); }
        public float altitude_monotonic_GET() {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public void altitude_monotonic_SET(float  src) {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public float altitude_amsl_GET() {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public void altitude_amsl_SET(float  src) {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public float altitude_local_GET() {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public void altitude_local_SET(float  src) {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public float altitude_relative_GET()//This is the altitude above the home position. It resets on each change of the current home positio
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public void altitude_relative_SET(float  src) //This is the altitude above the home position. It resets on each change of the current home positio
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }
        public float altitude_terrain_GET() {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public void altitude_terrain_SET(float  src) {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        public float bottom_clearance_GET() {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public void bottom_clearance_SET(float  src) {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
        static final Meta meta = new Meta(141, 0, 0, 1, 32, 256);
    } public static class RESOURCE_REQUEST extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        RESOURCE_REQUEST() { super(meta, 0); }
        RESOURCE_REQUEST(int bytes) { super(meta, bytes); }
        public char request_id_GET()//Request ID. This ID should be re-used when sending back URI content
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public void request_id_SET(char  src) //Request ID. This ID should be re-used when sending back URI content
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public char uri_type_GET()//The type of requested URI. 0 = a file via URL. 1 = a UAVCAN binar
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public void uri_type_SET(char  src) //The type of requested URI. 0 = a file via URL. 1 = a UAVCAN binar
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public char[] uri_GET(char[]  dst_ch, int pos)
        {
            for(int BYTE = 2, dst_max = pos + 120; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] uri_GET() {return uri_GET(new char[120], 0);} public void uri_SET(char[]  src, int pos)
        {
            for(int BYTE =  2, src_max = pos + 120; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        public char transfer_type_GET()//The way the autopilot wants to receive the URI. 0 = MAVLink FTP. 1 = binary stream
        {  return (char)((char) get_bytes(data,  122, 1)); }
        public void transfer_type_SET(char  src) //The way the autopilot wants to receive the URI. 0 = MAVLink FTP. 1 = binary stream
        {  set_bytes((char)(src) & -1L, 1, data,  122); }
        public char[] storage_GET(char[]  dst_ch, int pos)
        {
            for(int BYTE = 123, dst_max = pos + 120; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] storage_GET() {return storage_GET(new char[120], 0);} public void storage_SET(char[]  src, int pos)
        {
            for(int BYTE =  123, src_max = pos + 120; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        static final Meta meta = new Meta(142, 0, 0, 0, 243, 1944);
    } public static class SCALED_PRESSURE3 extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        SCALED_PRESSURE3() { super(meta, 0); }
        SCALED_PRESSURE3(int bytes) { super(meta, bytes); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot
        {  return (get_bytes(data,  0, 4)); }
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot
        {  set_bytes((src) & -1L, 4, data,  0); }
        public float press_abs_GET()//Absolute pressure (hectopascal
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public void press_abs_SET(float  src) //Absolute pressure (hectopascal
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }
        public float press_diff_GET()//Differential pressure 1 (hectopascal
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public void press_diff_SET(float  src) //Differential pressure 1 (hectopascal
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public short temperature_GET()//Temperature measurement (0.01 degrees celsius
        {  return (short)((short) get_bytes(data,  12, 2)); }
        public void temperature_SET(short  src) //Temperature measurement (0.01 degrees celsius
        {  set_bytes((short)(src) & -1L, 2, data,  12); }
        static final Meta meta = new Meta(143, 0, 1, 0, 14, 112);
    } public static class FOLLOW_TARGET extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        FOLLOW_TARGET() { super(meta, 0); }
        FOLLOW_TARGET(int bytes) { super(meta, bytes); }
        public long timestamp_GET()//Timestamp in milliseconds since system boo
        {  return (get_bytes(data,  0, 8)); }
        public void timestamp_SET(long  src) //Timestamp in milliseconds since system boo
        {  set_bytes((src) & -1L, 8, data,  0); }
        public long custom_state_GET()//button states or switches of a tracker devic
        {  return (get_bytes(data,  8, 8)); }
        public void custom_state_SET(long  src) //button states or switches of a tracker devic
        {  set_bytes((src) & -1L, 8, data,  8); }
        public char est_capabilities_GET()//bit positions for tracker reporting capabilities (POS = 0, VEL = 1, ACCEL = 2, ATT + RATES = 3
        {  return (char)((char) get_bytes(data,  16, 1)); }
        public void est_capabilities_SET(char  src) //bit positions for tracker reporting capabilities (POS = 0, VEL = 1, ACCEL = 2, ATT + RATES = 3
        {  set_bytes((char)(src) & -1L, 1, data,  16); }
        public int lat_GET()//Latitude (WGS84), in degrees * 1E
        {  return (int)((int) get_bytes(data,  17, 4)); }
        public void lat_SET(int  src) //Latitude (WGS84), in degrees * 1E
        {  set_bytes((int)(src) & -1L, 4, data,  17); }
        public int lon_GET()//Longitude (WGS84), in degrees * 1E
        {  return (int)((int) get_bytes(data,  21, 4)); }
        public void lon_SET(int  src) //Longitude (WGS84), in degrees * 1E
        {  set_bytes((int)(src) & -1L, 4, data,  21); }
        public float alt_GET()//AMSL, in meter
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  25, 4))); }
        public void alt_SET(float  src) //AMSL, in meter
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 25); }
        public float[] vel_GET(float[]  dst_ch, int pos)  //target velocity (0,0,0) for unknow
        {
            for(int BYTE = 29, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] vel_GET()//target velocity (0,0,0) for unknow
        {return vel_GET(new float[3], 0);} public void vel_SET(float[]  src, int pos)  //target velocity (0,0,0) for unknow
        {
            for(int BYTE =  29, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
                set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
        }
        public float[] acc_GET(float[]  dst_ch, int pos)  //linear target acceleration (0,0,0) for unknow
        {
            for(int BYTE = 41, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] acc_GET()//linear target acceleration (0,0,0) for unknow
        {return acc_GET(new float[3], 0);} public void acc_SET(float[]  src, int pos)  //linear target acceleration (0,0,0) for unknow
        {
            for(int BYTE =  41, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
                set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
        }
        public float[] attitude_q_GET(float[]  dst_ch, int pos)  //(1 0 0 0 for unknown
        {
            for(int BYTE = 53, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] attitude_q_GET()//(1 0 0 0 for unknown
        {return attitude_q_GET(new float[4], 0);} public void attitude_q_SET(float[]  src, int pos)  //(1 0 0 0 for unknown
        {
            for(int BYTE =  53, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
                set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
        }
        public float[] rates_GET(float[]  dst_ch, int pos)  //(0 0 0 for unknown
        {
            for(int BYTE = 69, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] rates_GET()//(0 0 0 for unknown
        {return rates_GET(new float[3], 0);} public void rates_SET(float[]  src, int pos)  //(0 0 0 for unknown
        {
            for(int BYTE =  69, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
                set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
        }
        public float[] position_cov_GET(float[]  dst_ch, int pos)  //eph ep
        {
            for(int BYTE = 81, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] position_cov_GET()//eph ep
        {return position_cov_GET(new float[3], 0);} public void position_cov_SET(float[]  src, int pos)  //eph ep
        {
            for(int BYTE =  81, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
                set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
        }
        static final Meta meta = new Meta(144, 0, 0, 2, 93, 744);
    } public static class CONTROL_SYSTEM_STATE extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        CONTROL_SYSTEM_STATE() { super(meta, 0); }
        CONTROL_SYSTEM_STATE(int bytes) { super(meta, bytes); }
        public long time_usec_GET()//Timestamp (micros since boot or Unix epoch
        {  return (get_bytes(data,  0, 8)); }
        public void time_usec_SET(long  src) //Timestamp (micros since boot or Unix epoch
        {  set_bytes((src) & -1L, 8, data,  0); }
        public float x_acc_GET()//X acceleration in body fram
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public void x_acc_SET(float  src) //X acceleration in body fram
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public float y_acc_GET()//Y acceleration in body fram
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public void y_acc_SET(float  src) //Y acceleration in body fram
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public float z_acc_GET()//Z acceleration in body fram
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public void z_acc_SET(float  src) //Z acceleration in body fram
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public float x_vel_GET()//X velocity in body fram
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public void x_vel_SET(float  src) //X velocity in body fram
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }
        public float y_vel_GET()//Y velocity in body fram
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public void y_vel_SET(float  src) //Y velocity in body fram
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        public float z_vel_GET()//Z velocity in body fram
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public void z_vel_SET(float  src) //Z velocity in body fram
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
        public float x_pos_GET()//X position in local fram
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  32, 4))); }
        public void x_pos_SET(float  src) //X position in local fram
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 32); }
        public float y_pos_GET()//Y position in local fram
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  36, 4))); }
        public void y_pos_SET(float  src) //Y position in local fram
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 36); }
        public float z_pos_GET()//Z position in local fram
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  40, 4))); }
        public void z_pos_SET(float  src) //Z position in local fram
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 40); }
        public float airspeed_GET()//Airspeed, set to -1 if unknow
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  44, 4))); }
        public void airspeed_SET(float  src) //Airspeed, set to -1 if unknow
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 44); }
        public float[] vel_variance_GET(float[]  dst_ch, int pos)  //Variance of body velocity estimat
        {
            for(int BYTE = 48, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] vel_variance_GET()//Variance of body velocity estimat
        {return vel_variance_GET(new float[3], 0);} public void vel_variance_SET(float[]  src, int pos)  //Variance of body velocity estimat
        {
            for(int BYTE =  48, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
                set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
        }
        public float[] pos_variance_GET(float[]  dst_ch, int pos)  //Variance in local positio
        {
            for(int BYTE = 60, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] pos_variance_GET()//Variance in local positio
        {return pos_variance_GET(new float[3], 0);} public void pos_variance_SET(float[]  src, int pos)  //Variance in local positio
        {
            for(int BYTE =  60, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
                set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
        }
        public float[] q_GET(float[]  dst_ch, int pos)  //The attitude, represented as Quaternio
        {
            for(int BYTE = 72, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] q_GET()//The attitude, represented as Quaternio
        {return q_GET(new float[4], 0);} public void q_SET(float[]  src, int pos)  //The attitude, represented as Quaternio
        {
            for(int BYTE =  72, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
                set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
        }
        public float roll_rate_GET()//Angular rate in roll axi
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  88, 4))); }
        public void roll_rate_SET(float  src) //Angular rate in roll axi
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 88); }
        public float pitch_rate_GET()//Angular rate in pitch axi
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  92, 4))); }
        public void pitch_rate_SET(float  src) //Angular rate in pitch axi
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 92); }
        public float yaw_rate_GET()//Angular rate in yaw axi
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  96, 4))); }
        public void yaw_rate_SET(float  src) //Angular rate in yaw axi
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 96); }
        static final Meta meta = new Meta(146, 0, 0, 1, 100, 800);
    } public static class BATTERY_STATUS extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        BATTERY_STATUS() { super(meta, 0); }
        BATTERY_STATUS(int bytes) { super(meta, bytes); }
        public char[] voltages_GET(char[]  dst_ch, int pos)
        {
            for(int BYTE = 0, dst_max = pos + 10; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public char[] voltages_GET() {return voltages_GET(new char[10], 0);} public void voltages_SET(char[]  src, int pos)
        {
            for(int BYTE =  0, src_max = pos + 10; pos < src_max; pos++, BYTE += 2)
                set_bytes((char)(src[pos]) & -1L, 2, data,  BYTE);
        }
        public char id_GET()//Battery I
        {  return (char)((char) get_bytes(data,  20, 1)); }
        public void id_SET(char  src) //Battery I
        {  set_bytes((char)(src) & -1L, 1, data,  20); }
        public short temperature_GET()//Temperature of the battery in centi-degrees celsius. INT16_MAX for unknown temperature
        {  return (short)((short) get_bytes(data,  21, 2)); }
        public void temperature_SET(short  src) //Temperature of the battery in centi-degrees celsius. INT16_MAX for unknown temperature
        {  set_bytes((short)(src) & -1L, 2, data,  21); }
        public short current_battery_GET()//Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the curre
        {  return (short)((short) get_bytes(data,  23, 2)); }
        public void current_battery_SET(short  src) //Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the curre
        {  set_bytes((short)(src) & -1L, 2, data,  23); }
        public int current_consumed_GET()//Consumed charge, in milliampere hours (1 = 1 mAh), -1: autopilot does not provide mAh consumption estima
        {  return (int)((int) get_bytes(data,  25, 4)); }
        public void current_consumed_SET(int  src) //Consumed charge, in milliampere hours (1 = 1 mAh), -1: autopilot does not provide mAh consumption estima
        {  set_bytes((int)(src) & -1L, 4, data,  25); }
        public int energy_consumed_GET() {  return (int)((int) get_bytes(data,  29, 4)); }
        public void energy_consumed_SET(int  src) {  set_bytes((int)(src) & -1L, 4, data,  29); }
        public byte battery_remaining_GET()//Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot does not estimate the remaining batter
        {  return (byte)((byte) get_bytes(data,  33, 1)); }
        public void battery_remaining_SET(byte  src) //Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot does not estimate the remaining batter
        {  set_bytes((byte)(src) & -1L, 1, data,  33); }
        public @MAV_BATTERY_FUNCTION int battery_function_GET()//Function of the batter
        {  return  0 + (int)get_bits(data, 272, 3); }
        public void battery_function_SET(@MAV_BATTERY_FUNCTION int  src) //Function of the batter
        {  set_bits(- 0 +   src, 3, data, 272); }
        public @MAV_BATTERY_TYPE int type_GET()//Type (chemistry) of the batter
        {  return  0 + (int)get_bits(data, 275, 3); }
        public void type_SET(@MAV_BATTERY_TYPE int  src) //Type (chemistry) of the batter
        {  set_bits(- 0 +   src, 3, data, 275); }
        static final Meta meta = new Meta(147, 10, 0, 0, 35, 278);
    } public static class AUTOPILOT_VERSION extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        AUTOPILOT_VERSION() { super(meta, 0); }
        AUTOPILOT_VERSION(int bytes) { super(meta, bytes); }
        public char vendor_id_GET()//ID of the board vendo
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void vendor_id_SET(char  src) //ID of the board vendo
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char product_id_GET()//ID of the produc
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public void product_id_SET(char  src) //ID of the produc
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public long flight_sw_version_GET()//Firmware version numbe
        {  return (get_bytes(data,  4, 4)); }
        public void flight_sw_version_SET(long  src) //Firmware version numbe
        {  set_bytes((src) & -1L, 4, data,  4); }
        public long middleware_sw_version_GET()//Middleware version numbe
        {  return (get_bytes(data,  8, 4)); }
        public void middleware_sw_version_SET(long  src) //Middleware version numbe
        {  set_bytes((src) & -1L, 4, data,  8); }
        public long os_sw_version_GET()//Operating system version numbe
        {  return (get_bytes(data,  12, 4)); }
        public void os_sw_version_SET(long  src) //Operating system version numbe
        {  set_bytes((src) & -1L, 4, data,  12); }
        public long board_version_GET()//HW / board version (last 8 bytes should be silicon ID, if any
        {  return (get_bytes(data,  16, 4)); }
        public void board_version_SET(long  src) //HW / board version (last 8 bytes should be silicon ID, if any
        {  set_bytes((src) & -1L, 4, data,  16); }
        public long uid_GET()//UID if provided by hardware (see uid2
        {  return (get_bytes(data,  20, 8)); }
        public void uid_SET(long  src) //UID if provided by hardware (see uid2
        {  set_bytes((src) & -1L, 8, data,  20); }
        public char[] flight_custom_version_GET(char[]  dst_ch, int pos)
        {
            for(int BYTE = 28, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] flight_custom_version_GET() {return flight_custom_version_GET(new char[8], 0);} public void flight_custom_version_SET(char[]  src, int pos)
        {
            for(int BYTE =  28, src_max = pos + 8; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        public char[] middleware_custom_version_GET(char[]  dst_ch, int pos)
        {
            for(int BYTE = 36, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] middleware_custom_version_GET() {return middleware_custom_version_GET(new char[8], 0);} public void middleware_custom_version_SET(char[]  src, int pos)
        {
            for(int BYTE =  36, src_max = pos + 8; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        public char[] os_custom_version_GET(char[]  dst_ch, int pos)
        {
            for(int BYTE = 44, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] os_custom_version_GET() {return os_custom_version_GET(new char[8], 0);} public void os_custom_version_SET(char[]  src, int pos)
        {
            for(int BYTE =  44, src_max = pos + 8; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        public @MAV_PROTOCOL_CAPABILITY int capabilities_GET()//bitmask of capabilities (see MAV_PROTOCOL_CAPABILITY enum
        {
            switch((int)get_bits(data, 416, 5))
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
            assert(false);//("Unknown enum ID " + id);
            return  Integer.MIN_VALUE;
        }
        public void capabilities_SET(@MAV_PROTOCOL_CAPABILITY int  src) //bitmask of capabilities (see MAV_PROTOCOL_CAPABILITY enum
        {
            long id = 0;
            switch(src)
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
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 5, data, 416);
        }
        public char[]  uid2_TRY(Bounds.Inside ph)
        {
            if(ph.field_bit !=  421 && !try_visit_field(ph, 421)) return null;
            return uid2_GET(ph, new char[ph.items], 0);
        }
        public char[] uid2_GET(Bounds.Inside ph, char[]  dst_ch, int pos)
        {
            for(int BYTE = ph.BYTE, dst_max = pos + 18; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public int uid2_LEN()
        {
            return 18;
        } public void uid2_SET(char[]  src, int pos, Bounds.Inside ph)
        {
            if(ph.field_bit != 421)insert_field(ph, 421, 0);
            for(int BYTE =  ph.BYTE, src_max = pos + 18; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        } static final Meta meta = new Meta(148, 2, 4, 1, 54, 421, 0, _Se);
    } public static class LANDING_TARGET extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        LANDING_TARGET() { super(meta, 0); }
        LANDING_TARGET(int bytes) { super(meta, bytes); }
        public long time_usec_GET()//Timestamp (micros since boot or Unix epoch
        {  return (get_bytes(data,  0, 8)); }
        public void time_usec_SET(long  src) //Timestamp (micros since boot or Unix epoch
        {  set_bytes((src) & -1L, 8, data,  0); }
        public char target_num_GET()//The ID of the target if multiple targets are presen
        {  return (char)((char) get_bytes(data,  8, 1)); }
        public void target_num_SET(char  src) //The ID of the target if multiple targets are presen
        {  set_bytes((char)(src) & -1L, 1, data,  8); }
        public float angle_x_GET()//X-axis angular offset (in radians) of the target from the center of the imag
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  9, 4))); }
        public void angle_x_SET(float  src) //X-axis angular offset (in radians) of the target from the center of the imag
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 9); }
        public float angle_y_GET()//Y-axis angular offset (in radians) of the target from the center of the imag
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  13, 4))); }
        public void angle_y_SET(float  src) //Y-axis angular offset (in radians) of the target from the center of the imag
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 13); }
        public float distance_GET()//Distance to the target from the vehicle in meter
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  17, 4))); }
        public void distance_SET(float  src) //Distance to the target from the vehicle in meter
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 17); }
        public float size_x_GET()//Size in radians of target along x-axi
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  21, 4))); }
        public void size_x_SET(float  src) //Size in radians of target along x-axi
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 21); }
        public float size_y_GET()//Size in radians of target along y-axi
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  25, 4))); }
        public void size_y_SET(float  src) //Size in radians of target along y-axi
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 25); }
        public @MAV_FRAME int frame_GET()//MAV_FRAME enum specifying the whether the following feilds are earth-frame, body-frame, etc
        {  return  0 + (int)get_bits(data, 232, 4); }
        public void frame_SET(@MAV_FRAME int  src) //MAV_FRAME enum specifying the whether the following feilds are earth-frame, body-frame, etc
        {  set_bits(- 0 +   src, 4, data, 232); }
        public @LANDING_TARGET_TYPE int type_GET()//LANDING_TARGET_TYPE enum specifying the type of landing targe
        {  return  0 + (int)get_bits(data, 236, 3); }
        public void type_SET(@LANDING_TARGET_TYPE int  src) //LANDING_TARGET_TYPE enum specifying the type of landing targe
        {  set_bits(- 0 +   src, 3, data, 236); }
        public float  x_TRY(Bounds.Inside ph)//X Position of the landing target on MAV_FRAM
//X Position of the landing target on MAV_FRAM
        {
            if(ph.field_bit !=  239 && !try_visit_field(ph, 239)) return 0;
            return (float)(Float.intBitsToFloat((int) get_bytes(data,  ph.BYTE, 4)));
        }
        public void x_SET(float  src, Bounds.Inside ph)//X Position of the landing target on MAV_FRAM
        {
            if(ph.field_bit != 239)insert_field(ph, 239, 0);
            set_bytes(Float.floatToIntBits(src) & -1L, 4, data, ph.BYTE);
        } public float  y_TRY(Bounds.Inside ph) //Y Position of the landing target on MAV_FRAM
//Y Position of the landing target on MAV_FRAM
        {
            if(ph.field_bit !=  240 && !try_visit_field(ph, 240)) return 0;
            return (float)(Float.intBitsToFloat((int) get_bytes(data,  ph.BYTE, 4)));
        }
        public void y_SET(float  src, Bounds.Inside ph)//Y Position of the landing target on MAV_FRAM
        {
            if(ph.field_bit != 240)insert_field(ph, 240, 0);
            set_bytes(Float.floatToIntBits(src) & -1L, 4, data, ph.BYTE);
        } public float  z_TRY(Bounds.Inside ph) //Z Position of the landing target on MAV_FRAM
//Z Position of the landing target on MAV_FRAM
        {
            if(ph.field_bit !=  241 && !try_visit_field(ph, 241)) return 0;
            return (float)(Float.intBitsToFloat((int) get_bytes(data,  ph.BYTE, 4)));
        }
        public void z_SET(float  src, Bounds.Inside ph)//Z Position of the landing target on MAV_FRAM
        {
            if(ph.field_bit != 241)insert_field(ph, 241, 0);
            set_bytes(Float.floatToIntBits(src) & -1L, 4, data, ph.BYTE);
        } public float[]  q_TRY(Bounds.Inside ph) //Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0
//Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0
        {
            if(ph.field_bit !=  242 && !try_visit_field(ph, 242)) return null;
            return q_GET(ph, new float[ph.items], 0);
        }
        public float[] q_GET(Bounds.Inside ph, float[]  dst_ch, int pos) //Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0
        {
            for(int BYTE = ph.BYTE, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public int q_LEN()
        {
            return 4;
        } public void q_SET(float[]  src, int pos, Bounds.Inside ph)//Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0
        {
            if(ph.field_bit != 242)insert_field(ph, 242, 0);
            for(int BYTE =  ph.BYTE, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
                set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
        } public char  position_valid_TRY(Bounds.Inside ph)
        {
            if(ph.field_bit !=  243 && !try_visit_field(ph, 243)) return 0;
            return (char)((char) get_bytes(data,  ph.BYTE, 1));
        }
        public void position_valid_SET(char  src, Bounds.Inside ph)
        {
            if(ph.field_bit != 243)insert_field(ph, 243, 0);
            set_bytes((char)(src) & -1L, 1, data,  ph.BYTE);
        } static final Meta meta = new Meta(149, 0, 0, 1, 31, 239, 0, _Ve, _je, _ze, _ee, _qe);
    } public static class NAV_FILTER_BIAS extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        NAV_FILTER_BIAS() { super(meta, 0); }
        NAV_FILTER_BIAS(int bytes) { super(meta, bytes); }
        public long usec_GET()//Timestamp (microseconds
        {  return (get_bytes(data,  0, 8)); }
        public void usec_SET(long  src) //Timestamp (microseconds
        {  set_bytes((src) & -1L, 8, data,  0); }
        public float accel_0_GET()//b_f[0
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public void accel_0_SET(float  src) //b_f[0
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public float accel_1_GET()//b_f[1
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public void accel_1_SET(float  src) //b_f[1
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public float accel_2_GET()//b_f[2
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public void accel_2_SET(float  src) //b_f[2
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public float gyro_0_GET()//b_f[0
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public void gyro_0_SET(float  src) //b_f[0
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }
        public float gyro_1_GET()//b_f[1
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public void gyro_1_SET(float  src) //b_f[1
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        public float gyro_2_GET()//b_f[2
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public void gyro_2_SET(float  src) //b_f[2
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
        static final Meta meta = new Meta(220, 0, 0, 1, 32, 256);
    } public static class RADIO_CALIBRATION extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        RADIO_CALIBRATION() { super(meta, 0); }
        RADIO_CALIBRATION(int bytes) { super(meta, bytes); }
        public char[] aileron_GET(char[]  dst_ch, int pos)  //Aileron setpoints: left, center, righ
        {
            for(int BYTE = 0, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public char[] aileron_GET()//Aileron setpoints: left, center, righ
        {return aileron_GET(new char[3], 0);} public void aileron_SET(char[]  src, int pos)  //Aileron setpoints: left, center, righ
        {
            for(int BYTE =  0, src_max = pos + 3; pos < src_max; pos++, BYTE += 2)
                set_bytes((char)(src[pos]) & -1L, 2, data,  BYTE);
        }
        public char[] elevator_GET(char[]  dst_ch, int pos)  //Elevator setpoints: nose down, center, nose u
        {
            for(int BYTE = 6, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public char[] elevator_GET()//Elevator setpoints: nose down, center, nose u
        {return elevator_GET(new char[3], 0);} public void elevator_SET(char[]  src, int pos)  //Elevator setpoints: nose down, center, nose u
        {
            for(int BYTE =  6, src_max = pos + 3; pos < src_max; pos++, BYTE += 2)
                set_bytes((char)(src[pos]) & -1L, 2, data,  BYTE);
        }
        public char[] rudder_GET(char[]  dst_ch, int pos)  //Rudder setpoints: nose left, center, nose righ
        {
            for(int BYTE = 12, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public char[] rudder_GET()//Rudder setpoints: nose left, center, nose righ
        {return rudder_GET(new char[3], 0);} public void rudder_SET(char[]  src, int pos)  //Rudder setpoints: nose left, center, nose righ
        {
            for(int BYTE =  12, src_max = pos + 3; pos < src_max; pos++, BYTE += 2)
                set_bytes((char)(src[pos]) & -1L, 2, data,  BYTE);
        }
        public char[] gyro_GET(char[]  dst_ch, int pos)  //Tail gyro mode/gain setpoints: heading hold, rate mod
        {
            for(int BYTE = 18, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public char[] gyro_GET()//Tail gyro mode/gain setpoints: heading hold, rate mod
        {return gyro_GET(new char[2], 0);} public void gyro_SET(char[]  src, int pos)  //Tail gyro mode/gain setpoints: heading hold, rate mod
        {
            for(int BYTE =  18, src_max = pos + 2; pos < src_max; pos++, BYTE += 2)
                set_bytes((char)(src[pos]) & -1L, 2, data,  BYTE);
        }
        public char[] pitch_GET(char[]  dst_ch, int pos)  //Pitch curve setpoints (every 25%
        {
            for(int BYTE = 22, dst_max = pos + 5; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public char[] pitch_GET()//Pitch curve setpoints (every 25%
        {return pitch_GET(new char[5], 0);} public void pitch_SET(char[]  src, int pos)  //Pitch curve setpoints (every 25%
        {
            for(int BYTE =  22, src_max = pos + 5; pos < src_max; pos++, BYTE += 2)
                set_bytes((char)(src[pos]) & -1L, 2, data,  BYTE);
        }
        public char[] throttle_GET(char[]  dst_ch, int pos)  //Throttle curve setpoints (every 25%
        {
            for(int BYTE = 32, dst_max = pos + 5; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public char[] throttle_GET()//Throttle curve setpoints (every 25%
        {return throttle_GET(new char[5], 0);} public void throttle_SET(char[]  src, int pos)  //Throttle curve setpoints (every 25%
        {
            for(int BYTE =  32, src_max = pos + 5; pos < src_max; pos++, BYTE += 2)
                set_bytes((char)(src[pos]) & -1L, 2, data,  BYTE);
        }
        static final Meta meta = new Meta(221, 21, 0, 0, 42, 336);
    } public static class UALBERTA_SYS_STATUS extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        UALBERTA_SYS_STATUS() { super(meta, 0); }
        UALBERTA_SYS_STATUS(int bytes) { super(meta, bytes); }
        public char mode_GET()//System mode, see UALBERTA_AUTOPILOT_MODE ENU
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public void mode_SET(char  src) //System mode, see UALBERTA_AUTOPILOT_MODE ENU
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public char nav_mode_GET()//Navigation mode, see UALBERTA_NAV_MODE ENU
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public void nav_mode_SET(char  src) //Navigation mode, see UALBERTA_NAV_MODE ENU
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public char pilot_GET()//Pilot mode, see UALBERTA_PILOT_MOD
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public void pilot_SET(char  src) //Pilot mode, see UALBERTA_PILOT_MOD
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        static final Meta meta = new Meta(222, 0, 0, 0, 3, 24);
    } public static class ESTIMATOR_STATUS extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        ESTIMATOR_STATUS() { super(meta, 0); }
        ESTIMATOR_STATUS(int bytes) { super(meta, bytes); }
        public long time_usec_GET()//Timestamp (micros since boot or Unix epoch
        {  return (get_bytes(data,  0, 8)); }
        public void time_usec_SET(long  src) //Timestamp (micros since boot or Unix epoch
        {  set_bytes((src) & -1L, 8, data,  0); }
        public float vel_ratio_GET()//Velocity innovation test rati
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public void vel_ratio_SET(float  src) //Velocity innovation test rati
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public float pos_horiz_ratio_GET()//Horizontal position innovation test rati
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public void pos_horiz_ratio_SET(float  src) //Horizontal position innovation test rati
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public float pos_vert_ratio_GET()//Vertical position innovation test rati
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public void pos_vert_ratio_SET(float  src) //Vertical position innovation test rati
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public float mag_ratio_GET()//Magnetometer innovation test rati
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public void mag_ratio_SET(float  src) //Magnetometer innovation test rati
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }
        public float hagl_ratio_GET()//Height above terrain innovation test rati
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public void hagl_ratio_SET(float  src) //Height above terrain innovation test rati
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        public float tas_ratio_GET()//True airspeed innovation test rati
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public void tas_ratio_SET(float  src) //True airspeed innovation test rati
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
        public float pos_horiz_accuracy_GET()//Horizontal position 1-STD accuracy relative to the EKF local origin (m
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  32, 4))); }
        public void pos_horiz_accuracy_SET(float  src) //Horizontal position 1-STD accuracy relative to the EKF local origin (m
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 32); }
        public float pos_vert_accuracy_GET()//Vertical position 1-STD accuracy relative to the EKF local origin (m
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  36, 4))); }
        public void pos_vert_accuracy_SET(float  src) //Vertical position 1-STD accuracy relative to the EKF local origin (m
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 36); }
        public @ESTIMATOR_STATUS_FLAGS int flags_GET()//Integer bitmask indicating which EKF outputs are valid. See definition for ESTIMATOR_STATUS_FLAGS
        {
            switch((int)get_bits(data, 320, 4))
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
            assert(false);//("Unknown enum ID " + id);
            return  Integer.MIN_VALUE;
        }
        public void flags_SET(@ESTIMATOR_STATUS_FLAGS int  src) //Integer bitmask indicating which EKF outputs are valid. See definition for ESTIMATOR_STATUS_FLAGS
        {
            long id = 0;
            switch(src)
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
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 4, data, 320);
        }
        static final Meta meta = new Meta(230, 0, 0, 1, 41, 324);
    } public static class WIND_COV extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        WIND_COV() { super(meta, 0); }
        WIND_COV(int bytes) { super(meta, bytes); }
        public long time_usec_GET()//Timestamp (micros since boot or Unix epoch
        {  return (get_bytes(data,  0, 8)); }
        public void time_usec_SET(long  src) //Timestamp (micros since boot or Unix epoch
        {  set_bytes((src) & -1L, 8, data,  0); }
        public float wind_x_GET()//Wind in X (NED) direction in m/
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public void wind_x_SET(float  src) //Wind in X (NED) direction in m/
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public float wind_y_GET()//Wind in Y (NED) direction in m/
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public void wind_y_SET(float  src) //Wind in Y (NED) direction in m/
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public float wind_z_GET()//Wind in Z (NED) direction in m/
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public void wind_z_SET(float  src) //Wind in Z (NED) direction in m/
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public float var_horiz_GET()//Variability of the wind in XY. RMS of a 1 Hz lowpassed wind estimate
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public void var_horiz_SET(float  src) //Variability of the wind in XY. RMS of a 1 Hz lowpassed wind estimate
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }
        public float var_vert_GET()//Variability of the wind in Z. RMS of a 1 Hz lowpassed wind estimate
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public void var_vert_SET(float  src) //Variability of the wind in Z. RMS of a 1 Hz lowpassed wind estimate
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        public float wind_alt_GET()//AMSL altitude (m) this measurement was taken a
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public void wind_alt_SET(float  src) //AMSL altitude (m) this measurement was taken a
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
        public float horiz_accuracy_GET()//Horizontal speed 1-STD accurac
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  32, 4))); }
        public void horiz_accuracy_SET(float  src) //Horizontal speed 1-STD accurac
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 32); }
        public float vert_accuracy_GET()//Vertical speed 1-STD accurac
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  36, 4))); }
        public void vert_accuracy_SET(float  src) //Vertical speed 1-STD accurac
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 36); }
        static final Meta meta = new Meta(231, 0, 0, 1, 40, 320);
    } public static class GPS_INPUT extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        GPS_INPUT() { super(meta, 0); }
        GPS_INPUT(int bytes) { super(meta, bytes); }
        public char time_week_GET()//GPS week numbe
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void time_week_SET(char  src) //GPS week numbe
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public long time_week_ms_GET()//GPS time (milliseconds from start of GPS week
        {  return (get_bytes(data,  2, 4)); }
        public void time_week_ms_SET(long  src) //GPS time (milliseconds from start of GPS week
        {  set_bytes((src) & -1L, 4, data,  2); }
        public long time_usec_GET()//Timestamp (micros since boot or Unix epoch
        {  return (get_bytes(data,  6, 8)); }
        public void time_usec_SET(long  src) //Timestamp (micros since boot or Unix epoch
        {  set_bytes((src) & -1L, 8, data,  6); }
        public char gps_id_GET()//ID of the GPS for multiple GPS input
        {  return (char)((char) get_bytes(data,  14, 1)); }
        public void gps_id_SET(char  src) //ID of the GPS for multiple GPS input
        {  set_bytes((char)(src) & -1L, 1, data,  14); }
        public char fix_type_GET()//0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RT
        {  return (char)((char) get_bytes(data,  15, 1)); }
        public void fix_type_SET(char  src) //0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RT
        {  set_bytes((char)(src) & -1L, 1, data,  15); }
        public int lat_GET()//Latitude (WGS84), in degrees * 1E
        {  return (int)((int) get_bytes(data,  16, 4)); }
        public void lat_SET(int  src) //Latitude (WGS84), in degrees * 1E
        {  set_bytes((int)(src) & -1L, 4, data,  16); }
        public int lon_GET()//Longitude (WGS84), in degrees * 1E
        {  return (int)((int) get_bytes(data,  20, 4)); }
        public void lon_SET(int  src) //Longitude (WGS84), in degrees * 1E
        {  set_bytes((int)(src) & -1L, 4, data,  20); }
        public float alt_GET()//Altitude (AMSL, not WGS84), in m (positive for up
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public void alt_SET(float  src) //Altitude (AMSL, not WGS84), in m (positive for up
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        public float hdop_GET()//GPS HDOP horizontal dilution of position in
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public void hdop_SET(float  src) //GPS HDOP horizontal dilution of position in
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
        public float vdop_GET()//GPS VDOP vertical dilution of position in
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  32, 4))); }
        public void vdop_SET(float  src) //GPS VDOP vertical dilution of position in
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 32); }
        public float vn_GET()//GPS velocity in m/s in NORTH direction in earth-fixed NED fram
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  36, 4))); }
        public void vn_SET(float  src) //GPS velocity in m/s in NORTH direction in earth-fixed NED fram
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 36); }
        public float ve_GET()//GPS velocity in m/s in EAST direction in earth-fixed NED fram
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  40, 4))); }
        public void ve_SET(float  src) //GPS velocity in m/s in EAST direction in earth-fixed NED fram
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 40); }
        public float vd_GET()//GPS velocity in m/s in DOWN direction in earth-fixed NED fram
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  44, 4))); }
        public void vd_SET(float  src) //GPS velocity in m/s in DOWN direction in earth-fixed NED fram
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 44); }
        public float speed_accuracy_GET()//GPS speed accuracy in m/
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  48, 4))); }
        public void speed_accuracy_SET(float  src) //GPS speed accuracy in m/
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 48); }
        public float horiz_accuracy_GET()//GPS horizontal accuracy in
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  52, 4))); }
        public void horiz_accuracy_SET(float  src) //GPS horizontal accuracy in
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 52); }
        public float vert_accuracy_GET()//GPS vertical accuracy in
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  56, 4))); }
        public void vert_accuracy_SET(float  src) //GPS vertical accuracy in
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 56); }
        public char satellites_visible_GET()//Number of satellites visible
        {  return (char)((char) get_bytes(data,  60, 1)); }
        public void satellites_visible_SET(char  src) //Number of satellites visible
        {  set_bytes((char)(src) & -1L, 1, data,  60); }
        public @GPS_INPUT_IGNORE_FLAGS int ignore_flags_GET()//Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum).  All other fields must be provide
        {
            switch((int)get_bits(data, 488, 4))
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
            assert(false);//("Unknown enum ID " + id);
            return  Integer.MIN_VALUE;
        }
        public void ignore_flags_SET(@GPS_INPUT_IGNORE_FLAGS int  src) //Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum).  All other fields must be provide
        {
            long id = 0;
            switch(src)
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
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 4, data, 488);
        }
        static final Meta meta = new Meta(232, 1, 1, 1, 62, 492);
    } public static class GPS_RTCM_DATA extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        GPS_RTCM_DATA() { super(meta, 0); }
        GPS_RTCM_DATA(int bytes) { super(meta, bytes); }
        public char flags_GET() {  return (char)((char) get_bytes(data,  0, 1)); }
        public void flags_SET(char  src) {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public char len_GET()//data lengt
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public void len_SET(char  src) //data lengt
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public char[] data__GET(char[]  dst_ch, int pos)  //RTCM message (may be fragmented
        {
            for(int BYTE = 2, dst_max = pos + 180; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] data__GET()//RTCM message (may be fragmented
        {return data__GET(new char[180], 0);} public void data__SET(char[]  src, int pos)  //RTCM message (may be fragmented
        {
            for(int BYTE =  2, src_max = pos + 180; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        static final Meta meta = new Meta(233, 0, 0, 0, 182, 1456);
    } public static class HIGH_LATENCY extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        HIGH_LATENCY() { super(meta, 0); }
        HIGH_LATENCY(int bytes) { super(meta, bytes); }
        public char heading_GET()//heading (centidegrees
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void heading_SET(char  src) //heading (centidegrees
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char wp_distance_GET()//distance to target (meters
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public void wp_distance_SET(char  src) //distance to target (meters
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public long custom_mode_GET()//A bitfield for use for autopilot-specific flags
        {  return (get_bytes(data,  4, 4)); }
        public void custom_mode_SET(long  src) //A bitfield for use for autopilot-specific flags
        {  set_bytes((src) & -1L, 4, data,  4); }
        public short roll_GET()//roll (centidegrees
        {  return (short)((short) get_bytes(data,  8, 2)); }
        public void roll_SET(short  src) //roll (centidegrees
        {  set_bytes((short)(src) & -1L, 2, data,  8); }
        public short pitch_GET()//pitch (centidegrees
        {  return (short)((short) get_bytes(data,  10, 2)); }
        public void pitch_SET(short  src) //pitch (centidegrees
        {  set_bytes((short)(src) & -1L, 2, data,  10); }
        public byte throttle_GET()//throttle (percentage
        {  return (byte)((byte) get_bytes(data,  12, 1)); }
        public void throttle_SET(byte  src) //throttle (percentage
        {  set_bytes((byte)(src) & -1L, 1, data,  12); }
        public short heading_sp_GET()//heading setpoint (centidegrees
        {  return (short)((short) get_bytes(data,  13, 2)); }
        public void heading_sp_SET(short  src) //heading setpoint (centidegrees
        {  set_bytes((short)(src) & -1L, 2, data,  13); }
        public int latitude_GET()//Latitude, expressed as degrees * 1E
        {  return (int)((int) get_bytes(data,  15, 4)); }
        public void latitude_SET(int  src) //Latitude, expressed as degrees * 1E
        {  set_bytes((int)(src) & -1L, 4, data,  15); }
        public int longitude_GET()//Longitude, expressed as degrees * 1E
        {  return (int)((int) get_bytes(data,  19, 4)); }
        public void longitude_SET(int  src) //Longitude, expressed as degrees * 1E
        {  set_bytes((int)(src) & -1L, 4, data,  19); }
        public short altitude_amsl_GET()//Altitude above mean sea level (meters
        {  return (short)((short) get_bytes(data,  23, 2)); }
        public void altitude_amsl_SET(short  src) //Altitude above mean sea level (meters
        {  set_bytes((short)(src) & -1L, 2, data,  23); }
        public short altitude_sp_GET()//Altitude setpoint relative to the home position (meters
        {  return (short)((short) get_bytes(data,  25, 2)); }
        public void altitude_sp_SET(short  src) //Altitude setpoint relative to the home position (meters
        {  set_bytes((short)(src) & -1L, 2, data,  25); }
        public char airspeed_GET()//airspeed (m/s
        {  return (char)((char) get_bytes(data,  27, 1)); }
        public void airspeed_SET(char  src) //airspeed (m/s
        {  set_bytes((char)(src) & -1L, 1, data,  27); }
        public char airspeed_sp_GET()//airspeed setpoint (m/s
        {  return (char)((char) get_bytes(data,  28, 1)); }
        public void airspeed_sp_SET(char  src) //airspeed setpoint (m/s
        {  set_bytes((char)(src) & -1L, 1, data,  28); }
        public char groundspeed_GET()//groundspeed (m/s
        {  return (char)((char) get_bytes(data,  29, 1)); }
        public void groundspeed_SET(char  src) //groundspeed (m/s
        {  set_bytes((char)(src) & -1L, 1, data,  29); }
        public byte climb_rate_GET()//climb rate (m/s
        {  return (byte)((byte) get_bytes(data,  30, 1)); }
        public void climb_rate_SET(byte  src) //climb rate (m/s
        {  set_bytes((byte)(src) & -1L, 1, data,  30); }
        public char gps_nsat_GET()//Number of satellites visible. If unknown, set to 25
        {  return (char)((char) get_bytes(data,  31, 1)); }
        public void gps_nsat_SET(char  src) //Number of satellites visible. If unknown, set to 25
        {  set_bytes((char)(src) & -1L, 1, data,  31); }
        public char battery_remaining_GET()//Remaining battery (percentage
        {  return (char)((char) get_bytes(data,  32, 1)); }
        public void battery_remaining_SET(char  src) //Remaining battery (percentage
        {  set_bytes((char)(src) & -1L, 1, data,  32); }
        public byte temperature_GET()//Autopilot temperature (degrees C
        {  return (byte)((byte) get_bytes(data,  33, 1)); }
        public void temperature_SET(byte  src) //Autopilot temperature (degrees C
        {  set_bytes((byte)(src) & -1L, 1, data,  33); }
        public byte temperature_air_GET()//Air temperature (degrees C) from airspeed senso
        {  return (byte)((byte) get_bytes(data,  34, 1)); }
        public void temperature_air_SET(byte  src) //Air temperature (degrees C) from airspeed senso
        {  set_bytes((byte)(src) & -1L, 1, data,  34); }
        public char failsafe_GET() {  return (char)((char) get_bytes(data,  35, 1)); }
        public void failsafe_SET(char  src) {  set_bytes((char)(src) & -1L, 1, data,  35); }
        public char wp_num_GET()//current waypoint numbe
        {  return (char)((char) get_bytes(data,  36, 1)); }
        public void wp_num_SET(char  src) //current waypoint numbe
        {  set_bytes((char)(src) & -1L, 1, data,  36); }
        public @MAV_MODE_FLAG int base_mode_GET()//System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.
        {
            switch((int)get_bits(data, 296, 4))
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
            assert(false);//("Unknown enum ID " + id);
            return  Integer.MIN_VALUE;
        }
        public void base_mode_SET(@MAV_MODE_FLAG int  src) //System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.
        {
            long id = 0;
            switch(src)
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
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 4, data, 296);
        }
        public @MAV_LANDED_STATE int landed_state_GET()//The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown
        {  return  0 + (int)get_bits(data, 300, 3); }
        public void landed_state_SET(@MAV_LANDED_STATE int  src) //The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown
        {  set_bits(- 0 +   src, 3, data, 300); }
        public @GPS_FIX_TYPE int gps_fix_type_GET()//See the GPS_FIX_TYPE enum
        {  return  0 + (int)get_bits(data, 303, 4); }
        public void gps_fix_type_SET(@GPS_FIX_TYPE int  src) //See the GPS_FIX_TYPE enum
        {  set_bits(- 0 +   src, 4, data, 303); }
        static final Meta meta = new Meta(234, 2, 1, 0, 39, 307);
    } public static class VIBRATION extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        VIBRATION() { super(meta, 0); }
        VIBRATION(int bytes) { super(meta, bytes); }
        public long clipping_0_GET()//first accelerometer clipping coun
        {  return (get_bytes(data,  0, 4)); }
        public void clipping_0_SET(long  src) //first accelerometer clipping coun
        {  set_bytes((src) & -1L, 4, data,  0); }
        public long clipping_1_GET()//second accelerometer clipping coun
        {  return (get_bytes(data,  4, 4)); }
        public void clipping_1_SET(long  src) //second accelerometer clipping coun
        {  set_bytes((src) & -1L, 4, data,  4); }
        public long clipping_2_GET()//third accelerometer clipping coun
        {  return (get_bytes(data,  8, 4)); }
        public void clipping_2_SET(long  src) //third accelerometer clipping coun
        {  set_bytes((src) & -1L, 4, data,  8); }
        public long time_usec_GET()//Timestamp (micros since boot or Unix epoch
        {  return (get_bytes(data,  12, 8)); }
        public void time_usec_SET(long  src) //Timestamp (micros since boot or Unix epoch
        {  set_bytes((src) & -1L, 8, data,  12); }
        public float vibration_x_GET()//Vibration levels on X-axi
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public void vibration_x_SET(float  src) //Vibration levels on X-axi
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }
        public float vibration_y_GET()//Vibration levels on Y-axi
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  24, 4))); }
        public void vibration_y_SET(float  src) //Vibration levels on Y-axi
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 24); }
        public float vibration_z_GET()//Vibration levels on Z-axi
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  28, 4))); }
        public void vibration_z_SET(float  src) //Vibration levels on Z-axi
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 28); }
        static final Meta meta = new Meta(241, 0, 3, 1, 32, 256);
    } public static class HOME_POSITION extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        HOME_POSITION() { super(meta, 0); }
        HOME_POSITION(int bytes) { super(meta, bytes); }
        public int latitude_GET()//Latitude (WGS84), in degrees * 1E
        {  return (int)((int) get_bytes(data,  0, 4)); }
        public void latitude_SET(int  src) //Latitude (WGS84), in degrees * 1E
        {  set_bytes((int)(src) & -1L, 4, data,  0); }
        public int longitude_GET()//Longitude (WGS84, in degrees * 1E
        {  return (int)((int) get_bytes(data,  4, 4)); }
        public void longitude_SET(int  src) //Longitude (WGS84, in degrees * 1E
        {  set_bytes((int)(src) & -1L, 4, data,  4); }
        public int altitude_GET()//Altitude (AMSL), in meters * 1000 (positive for up
        {  return (int)((int) get_bytes(data,  8, 4)); }
        public void altitude_SET(int  src) //Altitude (AMSL), in meters * 1000 (positive for up
        {  set_bytes((int)(src) & -1L, 4, data,  8); }
        public float x_GET()//Local X position of this position in the local coordinate fram
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public void x_SET(float  src) //Local X position of this position in the local coordinate fram
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public float y_GET()//Local Y position of this position in the local coordinate fram
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public void y_SET(float  src) //Local Y position of this position in the local coordinate fram
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public float z_GET()//Local Z position of this position in the local coordinate fram
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  20, 4))); }
        public void z_SET(float  src) //Local Z position of this position in the local coordinate fram
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 20); }
        public float[] q_GET(float[]  dst_ch, int pos)
        {
            for(int BYTE = 24, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] q_GET() {return q_GET(new float[4], 0);} public void q_SET(float[]  src, int pos)
        {
            for(int BYTE =  24, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
                set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
        }
        public float approach_x_GET() {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  40, 4))); }
        public void approach_x_SET(float  src) {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 40); }
        public float approach_y_GET() {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  44, 4))); }
        public void approach_y_SET(float  src) {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 44); }
        public float approach_z_GET() {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  48, 4))); }
        public void approach_z_SET(float  src) {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 48); }
        public long  time_usec_TRY(Bounds.Inside ph)//Timestamp (microseconds since UNIX epoch or microseconds since system boot
//Timestamp (microseconds since UNIX epoch or microseconds since system boot
        {
            if(ph.field_bit !=  416 && !try_visit_field(ph, 416)) return 0;
            return (get_bytes(data,  ph.BYTE, 8));
        }
        public void time_usec_SET(long  src, Bounds.Inside ph)//Timestamp (microseconds since UNIX epoch or microseconds since system boot
        {
            if(ph.field_bit != 416)insert_field(ph, 416, 0);
            set_bytes((src) & -1L, 8, data,  ph.BYTE);
        } static final Meta meta = new Meta(242, 0, 0, 0, 53, 416, 0, _Tn);
    } public static class SET_HOME_POSITION extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        SET_HOME_POSITION() { super(meta, 0); }
        SET_HOME_POSITION(int bytes) { super(meta, bytes); }
        public char target_system_GET()//System ID
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public void target_system_SET(char  src) //System ID
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public int latitude_GET()//Latitude (WGS84), in degrees * 1E
        {  return (int)((int) get_bytes(data,  1, 4)); }
        public void latitude_SET(int  src) //Latitude (WGS84), in degrees * 1E
        {  set_bytes((int)(src) & -1L, 4, data,  1); }
        public int longitude_GET()//Longitude (WGS84, in degrees * 1E
        {  return (int)((int) get_bytes(data,  5, 4)); }
        public void longitude_SET(int  src) //Longitude (WGS84, in degrees * 1E
        {  set_bytes((int)(src) & -1L, 4, data,  5); }
        public int altitude_GET()//Altitude (AMSL), in meters * 1000 (positive for up
        {  return (int)((int) get_bytes(data,  9, 4)); }
        public void altitude_SET(int  src) //Altitude (AMSL), in meters * 1000 (positive for up
        {  set_bytes((int)(src) & -1L, 4, data,  9); }
        public float x_GET()//Local X position of this position in the local coordinate fram
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  13, 4))); }
        public void x_SET(float  src) //Local X position of this position in the local coordinate fram
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 13); }
        public float y_GET()//Local Y position of this position in the local coordinate fram
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  17, 4))); }
        public void y_SET(float  src) //Local Y position of this position in the local coordinate fram
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 17); }
        public float z_GET()//Local Z position of this position in the local coordinate fram
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  21, 4))); }
        public void z_SET(float  src) //Local Z position of this position in the local coordinate fram
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 21); }
        public float[] q_GET(float[]  dst_ch, int pos)
        {
            for(int BYTE = 25, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] q_GET() {return q_GET(new float[4], 0);} public void q_SET(float[]  src, int pos)
        {
            for(int BYTE =  25, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
                set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
        }
        public float approach_x_GET() {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  41, 4))); }
        public void approach_x_SET(float  src) {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 41); }
        public float approach_y_GET() {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  45, 4))); }
        public void approach_y_SET(float  src) {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 45); }
        public float approach_z_GET() {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  49, 4))); }
        public void approach_z_SET(float  src) {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 49); }
        public long  time_usec_TRY(Bounds.Inside ph)//Timestamp (microseconds since UNIX epoch or microseconds since system boot
//Timestamp (microseconds since UNIX epoch or microseconds since system boot
        {
            if(ph.field_bit !=  424 && !try_visit_field(ph, 424)) return 0;
            return (get_bytes(data,  ph.BYTE, 8));
        }
        public void time_usec_SET(long  src, Bounds.Inside ph)//Timestamp (microseconds since UNIX epoch or microseconds since system boot
        {
            if(ph.field_bit != 424)insert_field(ph, 424, 0);
            set_bytes((src) & -1L, 8, data,  ph.BYTE);
        } static final Meta meta = new Meta(243, 0, 0, 0, 54, 424, 0, _Jq);
    } public static class MESSAGE_INTERVAL extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        MESSAGE_INTERVAL() { super(meta, 0); }
        MESSAGE_INTERVAL(int bytes) { super(meta, bytes); }
        public char message_id_GET()//The ID of the requested MAVLink message. v1.0 is limited to 254 messages
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void message_id_SET(char  src) //The ID of the requested MAVLink message. v1.0 is limited to 254 messages
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public int interval_us_GET()//0 indicates the interval at which it is sent
        {  return (int)((int) get_bytes(data,  2, 4)); }
        public void interval_us_SET(int  src) //0 indicates the interval at which it is sent
        {  set_bytes((int)(src) & -1L, 4, data,  2); }
        static final Meta meta = new Meta(244, 1, 0, 0, 6, 48);
    } public static class EXTENDED_SYS_STATE extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        EXTENDED_SYS_STATE() { super(meta, 0); }
        EXTENDED_SYS_STATE(int bytes) { super(meta, bytes); }
        public @MAV_VTOL_STATE int vtol_state_GET()//The VTOL state if applicable. Is set to MAV_VTOL_STATE_UNDEFINED if UAV is not in VTOL configuratio
        {  return  0 + (int)get_bits(data, 0, 3); }
        public void vtol_state_SET(@MAV_VTOL_STATE int  src) //The VTOL state if applicable. Is set to MAV_VTOL_STATE_UNDEFINED if UAV is not in VTOL configuratio
        {  set_bits(- 0 +   src, 3, data, 0); }
        public @MAV_LANDED_STATE int landed_state_GET()//The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown
        {  return  0 + (int)get_bits(data, 3, 3); }
        public void landed_state_SET(@MAV_LANDED_STATE int  src) //The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown
        {  set_bits(- 0 +   src, 3, data, 3); }
        static final Meta meta = new Meta(245, 0, 0, 0, 1, 6);
    } public static class ADSB_VEHICLE extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        ADSB_VEHICLE() { super(meta, 0); }
        ADSB_VEHICLE(int bytes) { super(meta, bytes); }
        public char heading_GET()//Course over ground in centidegree
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void heading_SET(char  src) //Course over ground in centidegree
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char hor_velocity_GET()//The horizontal velocity in centimeters/secon
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public void hor_velocity_SET(char  src) //The horizontal velocity in centimeters/secon
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public char squawk_GET()//Squawk cod
        {  return (char)((char) get_bytes(data,  4, 2)); }
        public void squawk_SET(char  src) //Squawk cod
        {  set_bytes((char)(src) & -1L, 2, data,  4); }
        public long ICAO_address_GET()//ICAO addres
        {  return (get_bytes(data,  6, 4)); }
        public void ICAO_address_SET(long  src) //ICAO addres
        {  set_bytes((src) & -1L, 4, data,  6); }
        public int lat_GET()//Latitude, expressed as degrees * 1E
        {  return (int)((int) get_bytes(data,  10, 4)); }
        public void lat_SET(int  src) //Latitude, expressed as degrees * 1E
        {  set_bytes((int)(src) & -1L, 4, data,  10); }
        public int lon_GET()//Longitude, expressed as degrees * 1E
        {  return (int)((int) get_bytes(data,  14, 4)); }
        public void lon_SET(int  src) //Longitude, expressed as degrees * 1E
        {  set_bytes((int)(src) & -1L, 4, data,  14); }
        public int altitude_GET()//Altitude(ASL) in millimeter
        {  return (int)((int) get_bytes(data,  18, 4)); }
        public void altitude_SET(int  src) //Altitude(ASL) in millimeter
        {  set_bytes((int)(src) & -1L, 4, data,  18); }
        public short ver_velocity_GET()//The vertical velocity in centimeters/second, positive is u
        {  return (short)((short) get_bytes(data,  22, 2)); }
        public void ver_velocity_SET(short  src) //The vertical velocity in centimeters/second, positive is u
        {  set_bytes((short)(src) & -1L, 2, data,  22); }
        public char tslc_GET()//Time since last communication in second
        {  return (char)((char) get_bytes(data,  24, 1)); }
        public void tslc_SET(char  src) //Time since last communication in second
        {  set_bytes((char)(src) & -1L, 1, data,  24); }
        public @ADSB_ALTITUDE_TYPE int altitude_type_GET()//Type from ADSB_ALTITUDE_TYPE enu
        {  return  0 + (int)get_bits(data, 200, 2); }
        public void altitude_type_SET(@ADSB_ALTITUDE_TYPE int  src) //Type from ADSB_ALTITUDE_TYPE enu
        {  set_bits(- 0 +   src, 2, data, 200); }
        public @ADSB_EMITTER_TYPE int emitter_type_GET()//Type from ADSB_EMITTER_TYPE enu
        {  return  0 + (int)get_bits(data, 202, 5); }
        public void emitter_type_SET(@ADSB_EMITTER_TYPE int  src) //Type from ADSB_EMITTER_TYPE enu
        {  set_bits(- 0 +   src, 5, data, 202); }
        public @ADSB_FLAGS int flags_GET()//Flags to indicate various statuses including valid data field
        {
            switch((int)get_bits(data, 207, 3))
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
            assert(false);//("Unknown enum ID " + id);
            return  Integer.MIN_VALUE;
        }
        public void flags_SET(@ADSB_FLAGS int  src) //Flags to indicate various statuses including valid data field
        {
            long id = 0;
            switch(src)
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
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 3, data, 207);
        }
        public String callsign_TRY(Bounds.Inside ph)//The callsign, 8+nul
//The callsign, 8+nul
        {
            if(ph.field_bit !=  210 && !try_visit_field(ph, 210)  ||  !try_visit_item(ph, 0)) return null;
            return new String(callsign_GET(ph, new char[ph.items], 0));
        }
        public char[] callsign_GET(Bounds.Inside ph, char[]  dst_ch, int pos) //The callsign, 8+nul
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int callsign_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  210 && !try_visit_field(ph, 210)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        } public void callsign_SET(String src, Bounds.Inside ph) //The callsign, 8+nul
        {callsign_SET(src.toCharArray(), 0, src.length(), ph);} public void callsign_SET(char[]  src, int pos, int items, Bounds.Inside ph) //The callsign, 8+nul
        {
            if(ph.field_bit != 210 && insert_field(ph, 210, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } static final Meta meta = new Meta(246, 3, 1, 0, 28, 210, 0, _Xq);
    } public static class COLLISION extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        COLLISION() { super(meta, 0); }
        COLLISION(int bytes) { super(meta, bytes); }
        public long id_GET()//Unique identifier, domain based on src fiel
        {  return (get_bytes(data,  0, 4)); }
        public void id_SET(long  src) //Unique identifier, domain based on src fiel
        {  set_bytes((src) & -1L, 4, data,  0); }
        public float time_to_minimum_delta_GET()//Estimated time until collision occurs (seconds
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public void time_to_minimum_delta_SET(float  src) //Estimated time until collision occurs (seconds
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }
        public float altitude_minimum_delta_GET()//Closest vertical distance in meters between vehicle and objec
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public void altitude_minimum_delta_SET(float  src) //Closest vertical distance in meters between vehicle and objec
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public float horizontal_minimum_delta_GET()//Closest horizontal distance in meteres between vehicle and objec
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public void horizontal_minimum_delta_SET(float  src) //Closest horizontal distance in meteres between vehicle and objec
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public @MAV_COLLISION_SRC int src__GET()//Collision data sourc
        {  return  0 + (int)get_bits(data, 128, 2); }
        public void src__SET(@MAV_COLLISION_SRC int  src) //Collision data sourc
        {  set_bits(- 0 +   src, 2, data, 128); }
        public @MAV_COLLISION_ACTION int action_GET()//Action that is being taken to avoid this collisio
        {  return  0 + (int)get_bits(data, 130, 3); }
        public void action_SET(@MAV_COLLISION_ACTION int  src) //Action that is being taken to avoid this collisio
        {  set_bits(- 0 +   src, 3, data, 130); }
        public @MAV_COLLISION_THREAT_LEVEL int threat_level_GET()//How concerned the aircraft is about this collisio
        {  return  0 + (int)get_bits(data, 133, 2); }
        public void threat_level_SET(@MAV_COLLISION_THREAT_LEVEL int  src) //How concerned the aircraft is about this collisio
        {  set_bits(- 0 +   src, 2, data, 133); }
        static final Meta meta = new Meta(247, 0, 1, 0, 17, 135);
    } public static class V2_EXTENSION extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        V2_EXTENSION() { super(meta, 0); }
        V2_EXTENSION(int bytes) { super(meta, bytes); }
        public char message_type_GET() {  return (char)((char) get_bytes(data,  0, 2)); }
        public void message_type_SET(char  src) {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char target_network_GET()//Network ID (0 for broadcast
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public void target_network_SET(char  src) //Network ID (0 for broadcast
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        public char target_system_GET()//System ID (0 for broadcast
        {  return (char)((char) get_bytes(data,  3, 1)); }
        public void target_system_SET(char  src) //System ID (0 for broadcast
        {  set_bytes((char)(src) & -1L, 1, data,  3); }
        public char target_component_GET()//Component ID (0 for broadcast
        {  return (char)((char) get_bytes(data,  4, 1)); }
        public void target_component_SET(char  src) //Component ID (0 for broadcast
        {  set_bytes((char)(src) & -1L, 1, data,  4); }
        public char[] payload_GET(char[]  dst_ch, int pos)
        {
            for(int BYTE = 5, dst_max = pos + 249; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] payload_GET() {return payload_GET(new char[249], 0);} public void payload_SET(char[]  src, int pos)
        {
            for(int BYTE =  5, src_max = pos + 249; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        static final Meta meta = new Meta(248, 1, 0, 0, 254, 2032);
    } public static class MEMORY_VECT extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        MEMORY_VECT() { super(meta, 0); }
        MEMORY_VECT(int bytes) { super(meta, bytes); }
        public char address_GET()//Starting address of the debug variable
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void address_SET(char  src) //Starting address of the debug variable
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char ver_GET()//Version code of the type variable. 0=unknown, type ignored and assumed short. 1=as belo
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public void ver_SET(char  src) //Version code of the type variable. 0=unknown, type ignored and assumed short. 1=as belo
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        public char type_GET()//Type code of the memory variables. for ver = 1: 0=16 x short, 1=16 x char, 2=16 x Q15, 3=16 x 1Q
        {  return (char)((char) get_bytes(data,  3, 1)); }
        public void type_SET(char  src) //Type code of the memory variables. for ver = 1: 0=16 x short, 1=16 x char, 2=16 x Q15, 3=16 x 1Q
        {  set_bytes((char)(src) & -1L, 1, data,  3); }
        public byte[] value_GET(byte[]  dst_ch, int pos)  //Memory contents at specified addres
        {
            for(int BYTE = 4, dst_max = pos + 32; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (byte)((byte) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public byte[] value_GET()//Memory contents at specified addres
        {return value_GET(new byte[32], 0);} public void value_SET(byte[]  src, int pos)  //Memory contents at specified addres
        {
            for(int BYTE =  4, src_max = pos + 32; pos < src_max; pos++, BYTE += 1)
                set_bytes((byte)(src[pos]) & -1L, 1, data,  BYTE);
        }
        static final Meta meta = new Meta(249, 1, 0, 0, 36, 288);
    } public static class DEBUG_VECT extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        DEBUG_VECT() { super(meta, 0); }
        DEBUG_VECT(int bytes) { super(meta, bytes); }
        public long time_usec_GET()//Timestam
        {  return (get_bytes(data,  0, 8)); }
        public void time_usec_SET(long  src) //Timestam
        {  set_bytes((src) & -1L, 8, data,  0); }
        public float x_GET() {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public void x_SET(float  src) {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public float y_GET() {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public void y_SET(float  src) {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public float z_GET() {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  16, 4))); }
        public void z_SET(float  src) {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 16); }
        public String name_TRY(Bounds.Inside ph)//Nam
//Nam
        {
            if(ph.field_bit !=  160 && !try_visit_field(ph, 160)  ||  !try_visit_item(ph, 0)) return null;
            return new String(name_GET(ph, new char[ph.items], 0));
        }
        public char[] name_GET(Bounds.Inside ph, char[]  dst_ch, int pos) //Nam
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int name_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  160 && !try_visit_field(ph, 160)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        } public void name_SET(String src, Bounds.Inside ph) //Nam
        {name_SET(src.toCharArray(), 0, src.length(), ph);} public void name_SET(char[]  src, int pos, int items, Bounds.Inside ph) //Nam
        {
            if(ph.field_bit != 160 && insert_field(ph, 160, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } static final Meta meta = new Meta(250, 0, 0, 1, 21, 160, 0, _bq);
    } public static class NAMED_VALUE_FLOAT extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        NAMED_VALUE_FLOAT() { super(meta, 0); }
        NAMED_VALUE_FLOAT(int bytes) { super(meta, bytes); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot
        {  return (get_bytes(data,  0, 4)); }
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot
        {  set_bytes((src) & -1L, 4, data,  0); }
        public float value_GET()//Floating point valu
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public void value_SET(float  src) //Floating point valu
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }
        public String name_TRY(Bounds.Inside ph)//Name of the debug variabl
//Name of the debug variabl
        {
            if(ph.field_bit !=  64 && !try_visit_field(ph, 64)  ||  !try_visit_item(ph, 0)) return null;
            return new String(name_GET(ph, new char[ph.items], 0));
        }
        public char[] name_GET(Bounds.Inside ph, char[]  dst_ch, int pos) //Name of the debug variabl
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int name_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  64 && !try_visit_field(ph, 64)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        } public void name_SET(String src, Bounds.Inside ph) //Name of the debug variabl
        {name_SET(src.toCharArray(), 0, src.length(), ph);} public void name_SET(char[]  src, int pos, int items, Bounds.Inside ph) //Name of the debug variabl
        {
            if(ph.field_bit != 64 && insert_field(ph, 64, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } static final Meta meta = new Meta(251, 0, 1, 0, 9, 64, 0, _Tq);
    } public static class NAMED_VALUE_INT extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        NAMED_VALUE_INT() { super(meta, 0); }
        NAMED_VALUE_INT(int bytes) { super(meta, bytes); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot
        {  return (get_bytes(data,  0, 4)); }
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot
        {  set_bytes((src) & -1L, 4, data,  0); }
        public int value_GET()//Signed integer valu
        {  return (int)((int) get_bytes(data,  4, 4)); }
        public void value_SET(int  src) //Signed integer valu
        {  set_bytes((int)(src) & -1L, 4, data,  4); }
        public String name_TRY(Bounds.Inside ph)//Name of the debug variabl
//Name of the debug variabl
        {
            if(ph.field_bit !=  64 && !try_visit_field(ph, 64)  ||  !try_visit_item(ph, 0)) return null;
            return new String(name_GET(ph, new char[ph.items], 0));
        }
        public char[] name_GET(Bounds.Inside ph, char[]  dst_ch, int pos) //Name of the debug variabl
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int name_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  64 && !try_visit_field(ph, 64)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        } public void name_SET(String src, Bounds.Inside ph) //Name of the debug variabl
        {name_SET(src.toCharArray(), 0, src.length(), ph);} public void name_SET(char[]  src, int pos, int items, Bounds.Inside ph) //Name of the debug variabl
        {
            if(ph.field_bit != 64 && insert_field(ph, 64, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } static final Meta meta = new Meta(252, 0, 1, 0, 9, 64, 0, _xM);
    } public static class STATUSTEXT extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        STATUSTEXT() { super(meta, 0); }
        STATUSTEXT(int bytes) { super(meta, bytes); }
        public @MAV_SEVERITY int severity_GET()//Severity of status. Relies on the definitions within RFC-5424. See enum MAV_SEVERITY
        {  return  0 + (int)get_bits(data, 0, 4); }
        public void severity_SET(@MAV_SEVERITY int  src) //Severity of status. Relies on the definitions within RFC-5424. See enum MAV_SEVERITY
        {  set_bits(- 0 +   src, 4, data, 0); }
        public String text_TRY(Bounds.Inside ph)//Status text message, without null termination characte
//Status text message, without null termination characte
        {
            if(ph.field_bit !=  4 && !try_visit_field(ph, 4)  ||  !try_visit_item(ph, 0)) return null;
            return new String(text_GET(ph, new char[ph.items], 0));
        }
        public char[] text_GET(Bounds.Inside ph, char[]  dst_ch, int pos) //Status text message, without null termination characte
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int text_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  4 && !try_visit_field(ph, 4)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        } public void text_SET(String src, Bounds.Inside ph) //Status text message, without null termination characte
        {text_SET(src.toCharArray(), 0, src.length(), ph);} public void text_SET(char[]  src, int pos, int items, Bounds.Inside ph) //Status text message, without null termination characte
        {
            if(ph.field_bit != 4 && insert_field(ph, 4, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } static final Meta meta = new Meta(253, 0, 0, 0, 2, 4, 0, _WM);
    } public static class DEBUG extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        DEBUG() { super(meta, 0); }
        DEBUG(int bytes) { super(meta, bytes); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot
        {  return (get_bytes(data,  0, 4)); }
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot
        {  set_bytes((src) & -1L, 4, data,  0); }
        public char ind_GET()//index of debug variabl
        {  return (char)((char) get_bytes(data,  4, 1)); }
        public void ind_SET(char  src) //index of debug variabl
        {  set_bytes((char)(src) & -1L, 1, data,  4); }
        public float value_GET()//DEBUG valu
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  5, 4))); }
        public void value_SET(float  src) //DEBUG valu
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 5); }
        static final Meta meta = new Meta(254, 0, 1, 0, 9, 72);
    } public static class SETUP_SIGNING extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        SETUP_SIGNING() { super(meta, 0); }
        SETUP_SIGNING(int bytes) { super(meta, bytes); }
        public long initial_timestamp_GET()//initial timestam
        {  return (get_bytes(data,  0, 8)); }
        public void initial_timestamp_SET(long  src) //initial timestam
        {  set_bytes((src) & -1L, 8, data,  0); }
        public char target_system_GET()//system id of the targe
        {  return (char)((char) get_bytes(data,  8, 1)); }
        public void target_system_SET(char  src) //system id of the targe
        {  set_bytes((char)(src) & -1L, 1, data,  8); }
        public char target_component_GET()//component ID of the targe
        {  return (char)((char) get_bytes(data,  9, 1)); }
        public void target_component_SET(char  src) //component ID of the targe
        {  set_bytes((char)(src) & -1L, 1, data,  9); }
        public char[] secret_key_GET(char[]  dst_ch, int pos)  //signing ke
        {
            for(int BYTE = 10, dst_max = pos + 32; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] secret_key_GET()//signing ke
        {return secret_key_GET(new char[32], 0);} public void secret_key_SET(char[]  src, int pos)  //signing ke
        {
            for(int BYTE =  10, src_max = pos + 32; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        static final Meta meta = new Meta(256, 0, 0, 1, 42, 336);
    } public static class BUTTON_CHANGE extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        BUTTON_CHANGE() { super(meta, 0); }
        BUTTON_CHANGE(int bytes) { super(meta, bytes); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot
        {  return (get_bytes(data,  0, 4)); }
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot
        {  set_bytes((src) & -1L, 4, data,  0); }
        public long last_change_ms_GET()//Time of last change of button stat
        {  return (get_bytes(data,  4, 4)); }
        public void last_change_ms_SET(long  src) //Time of last change of button stat
        {  set_bytes((src) & -1L, 4, data,  4); }
        public char state_GET()//Bitmap state of button
        {  return (char)((char) get_bytes(data,  8, 1)); }
        public void state_SET(char  src) //Bitmap state of button
        {  set_bytes((char)(src) & -1L, 1, data,  8); }
        static final Meta meta = new Meta(257, 0, 2, 0, 9, 72);
    } public static class PLAY_TUNE extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        PLAY_TUNE() { super(meta, 0); }
        PLAY_TUNE(int bytes) { super(meta, bytes); }
        public char target_system_GET()//System I
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public void target_system_SET(char  src) //System I
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public char target_component_GET()//Component I
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public void target_component_SET(char  src) //Component I
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public String tune_TRY(Bounds.Inside ph)//tune in board specific forma
//tune in board specific forma
        {
            if(ph.field_bit !=  16 && !try_visit_field(ph, 16)  ||  !try_visit_item(ph, 0)) return null;
            return new String(tune_GET(ph, new char[ph.items], 0));
        }
        public char[] tune_GET(Bounds.Inside ph, char[]  dst_ch, int pos) //tune in board specific forma
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int tune_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  16 && !try_visit_field(ph, 16)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        } public void tune_SET(String src, Bounds.Inside ph) //tune in board specific forma
        {tune_SET(src.toCharArray(), 0, src.length(), ph);} public void tune_SET(char[]  src, int pos, int items, Bounds.Inside ph) //tune in board specific forma
        {
            if(ph.field_bit != 16 && insert_field(ph, 16, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } static final Meta meta = new Meta(258, 0, 0, 0, 3, 16, 0, _zM);
    } public static class CAMERA_INFORMATION extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        CAMERA_INFORMATION() { super(meta, 0); }
        CAMERA_INFORMATION(int bytes) { super(meta, bytes); }
        public char resolution_h_GET()//Image resolution in pixels horizonta
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void resolution_h_SET(char  src) //Image resolution in pixels horizonta
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char resolution_v_GET()//Image resolution in pixels vertica
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public void resolution_v_SET(char  src) //Image resolution in pixels vertica
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public char cam_definition_version_GET()//Camera definition version (iteration
        {  return (char)((char) get_bytes(data,  4, 2)); }
        public void cam_definition_version_SET(char  src) //Camera definition version (iteration
        {  set_bytes((char)(src) & -1L, 2, data,  4); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot
        {  return (get_bytes(data,  6, 4)); }
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot
        {  set_bytes((src) & -1L, 4, data,  6); }
        public long firmware_version_GET()//0xff = Major
        {  return (get_bytes(data,  10, 4)); }
        public void firmware_version_SET(long  src) //0xff = Major
        {  set_bytes((src) & -1L, 4, data,  10); }
        public char[] vendor_name_GET(char[]  dst_ch, int pos)  //Name of the camera vendo
        {
            for(int BYTE = 14, dst_max = pos + 32; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] vendor_name_GET()//Name of the camera vendo
        {return vendor_name_GET(new char[32], 0);} public void vendor_name_SET(char[]  src, int pos)  //Name of the camera vendo
        {
            for(int BYTE =  14, src_max = pos + 32; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        public char[] model_name_GET(char[]  dst_ch, int pos)  //Name of the camera mode
        {
            for(int BYTE = 46, dst_max = pos + 32; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] model_name_GET()//Name of the camera mode
        {return model_name_GET(new char[32], 0);} public void model_name_SET(char[]  src, int pos)  //Name of the camera mode
        {
            for(int BYTE =  46, src_max = pos + 32; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        public float focal_length_GET()//Focal length in m
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  78, 4))); }
        public void focal_length_SET(float  src) //Focal length in m
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 78); }
        public float sensor_size_h_GET()//Image sensor size horizontal in m
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  82, 4))); }
        public void sensor_size_h_SET(float  src) //Image sensor size horizontal in m
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 82); }
        public float sensor_size_v_GET()//Image sensor size vertical in m
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  86, 4))); }
        public void sensor_size_v_SET(float  src) //Image sensor size vertical in m
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 86); }
        public char lens_id_GET()//Reserved for a lens I
        {  return (char)((char) get_bytes(data,  90, 1)); }
        public void lens_id_SET(char  src) //Reserved for a lens I
        {  set_bytes((char)(src) & -1L, 1, data,  90); }
        public @CAMERA_CAP_FLAGS int flags_GET()//CAMERA_CAP_FLAGS enum flags (bitmap) describing camera capabilities
        {
            switch((int)get_bits(data, 728, 3))
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
            assert(false);//("Unknown enum ID " + id);
            return  Integer.MIN_VALUE;
        }
        public void flags_SET(@CAMERA_CAP_FLAGS int  src) //CAMERA_CAP_FLAGS enum flags (bitmap) describing camera capabilities
        {
            long id = 0;
            switch(src)
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
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 3, data, 728);
        }
        public String cam_definition_uri_TRY(Bounds.Inside ph)//Camera definition URI (if any, otherwise only basic functions will be available)
//Camera definition URI (if any, otherwise only basic functions will be available)
        {
            if(ph.field_bit !=  731 && !try_visit_field(ph, 731)  ||  !try_visit_item(ph, 0)) return null;
            return new String(cam_definition_uri_GET(ph, new char[ph.items], 0));
        }
        public char[] cam_definition_uri_GET(Bounds.Inside ph, char[]  dst_ch, int pos) //Camera definition URI (if any, otherwise only basic functions will be available)
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int cam_definition_uri_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  731 && !try_visit_field(ph, 731)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        } public void cam_definition_uri_SET(String src, Bounds.Inside ph) //Camera definition URI (if any, otherwise only basic functions will be available)
        {cam_definition_uri_SET(src.toCharArray(), 0, src.length(), ph);} public void cam_definition_uri_SET(char[]  src, int pos, int items, Bounds.Inside ph) //Camera definition URI (if any, otherwise only basic functions will be available)
        {
            if(ph.field_bit != 731 && insert_field(ph, 731, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } static final Meta meta = new Meta(259, 3, 2, 0, 93, 731, 0, _gM);
    } public static class CAMERA_SETTINGS extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        CAMERA_SETTINGS() { super(meta, 0); }
        CAMERA_SETTINGS(int bytes) { super(meta, bytes); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot
        {  return (get_bytes(data,  0, 4)); }
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot
        {  set_bytes((src) & -1L, 4, data,  0); }
        public @CAMERA_MODE int mode_id_GET()//Camera mode (CAMERA_MODE
        {  return  0 + (int)get_bits(data, 32, 2); }
        public void mode_id_SET(@CAMERA_MODE int  src) //Camera mode (CAMERA_MODE
        {  set_bits(- 0 +   src, 2, data, 32); }
        static final Meta meta = new Meta(260, 0, 1, 0, 5, 34);
    } public static class STORAGE_INFORMATION extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        STORAGE_INFORMATION() { super(meta, 0); }
        STORAGE_INFORMATION(int bytes) { super(meta, bytes); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot
        {  return (get_bytes(data,  0, 4)); }
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot
        {  set_bytes((src) & -1L, 4, data,  0); }
        public char storage_id_GET()//Storage ID (1 for first, 2 for second, etc.
        {  return (char)((char) get_bytes(data,  4, 1)); }
        public void storage_id_SET(char  src) //Storage ID (1 for first, 2 for second, etc.
        {  set_bytes((char)(src) & -1L, 1, data,  4); }
        public char storage_count_GET()//Number of storage device
        {  return (char)((char) get_bytes(data,  5, 1)); }
        public void storage_count_SET(char  src) //Number of storage device
        {  set_bytes((char)(src) & -1L, 1, data,  5); }
        public char status_GET()//Status of storage (0 not available, 1 unformatted, 2 formatted
        {  return (char)((char) get_bytes(data,  6, 1)); }
        public void status_SET(char  src) //Status of storage (0 not available, 1 unformatted, 2 formatted
        {  set_bytes((char)(src) & -1L, 1, data,  6); }
        public float total_capacity_GET()//Total capacity in Mi
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  7, 4))); }
        public void total_capacity_SET(float  src) //Total capacity in Mi
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 7); }
        public float used_capacity_GET()//Used capacity in Mi
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  11, 4))); }
        public void used_capacity_SET(float  src) //Used capacity in Mi
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 11); }
        public float available_capacity_GET()//Available capacity in Mi
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  15, 4))); }
        public void available_capacity_SET(float  src) //Available capacity in Mi
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 15); }
        public float read_speed_GET()//Read speed in MiB/
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  19, 4))); }
        public void read_speed_SET(float  src) //Read speed in MiB/
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 19); }
        public float write_speed_GET()//Write speed in MiB/
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  23, 4))); }
        public void write_speed_SET(float  src) //Write speed in MiB/
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 23); }
        static final Meta meta = new Meta(261, 0, 1, 0, 27, 216);
    } public static class CAMERA_CAPTURE_STATUS extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        CAMERA_CAPTURE_STATUS() { super(meta, 0); }
        CAMERA_CAPTURE_STATUS(int bytes) { super(meta, bytes); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot
        {  return (get_bytes(data,  0, 4)); }
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot
        {  set_bytes((src) & -1L, 4, data,  0); }
        public long recording_time_ms_GET()//Time in milliseconds since recording starte
        {  return (get_bytes(data,  4, 4)); }
        public void recording_time_ms_SET(long  src) //Time in milliseconds since recording starte
        {  set_bytes((src) & -1L, 4, data,  4); }
        public char image_status_GET() {  return (char)((char) get_bytes(data,  8, 1)); }
        public void image_status_SET(char  src) {  set_bytes((char)(src) & -1L, 1, data,  8); }
        public char video_status_GET()//Current status of video capturing (0: idle, 1: capture in progress
        {  return (char)((char) get_bytes(data,  9, 1)); }
        public void video_status_SET(char  src) //Current status of video capturing (0: idle, 1: capture in progress
        {  set_bytes((char)(src) & -1L, 1, data,  9); }
        public float image_interval_GET()//Image capture interval in second
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  10, 4))); }
        public void image_interval_SET(float  src) //Image capture interval in second
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 10); }
        public float available_capacity_GET()//Available storage capacity in Mi
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  14, 4))); }
        public void available_capacity_SET(float  src) //Available storage capacity in Mi
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 14); }
        static final Meta meta = new Meta(262, 0, 2, 0, 18, 144);
    } public static class CAMERA_IMAGE_CAPTURED extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        CAMERA_IMAGE_CAPTURED() { super(meta, 0); }
        CAMERA_IMAGE_CAPTURED(int bytes) { super(meta, bytes); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot
        {  return (get_bytes(data,  0, 4)); }
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot
        {  set_bytes((src) & -1L, 4, data,  0); }
        public long time_utc_GET()//Timestamp (microseconds since UNIX epoch) in UTC. 0 for unknown
        {  return (get_bytes(data,  4, 8)); }
        public void time_utc_SET(long  src) //Timestamp (microseconds since UNIX epoch) in UTC. 0 for unknown
        {  set_bytes((src) & -1L, 8, data,  4); }
        public char camera_id_GET()//Camera ID (1 for first, 2 for second, etc.
        {  return (char)((char) get_bytes(data,  12, 1)); }
        public void camera_id_SET(char  src) //Camera ID (1 for first, 2 for second, etc.
        {  set_bytes((char)(src) & -1L, 1, data,  12); }
        public int lat_GET()//Latitude, expressed as degrees * 1E7 where image was take
        {  return (int)((int) get_bytes(data,  13, 4)); }
        public void lat_SET(int  src) //Latitude, expressed as degrees * 1E7 where image was take
        {  set_bytes((int)(src) & -1L, 4, data,  13); }
        public int lon_GET()//Longitude, expressed as degrees * 1E7 where capture was take
        {  return (int)((int) get_bytes(data,  17, 4)); }
        public void lon_SET(int  src) //Longitude, expressed as degrees * 1E7 where capture was take
        {  set_bytes((int)(src) & -1L, 4, data,  17); }
        public int alt_GET()//Altitude in meters, expressed as * 1E3 (AMSL, not WGS84) where image was take
        {  return (int)((int) get_bytes(data,  21, 4)); }
        public void alt_SET(int  src) //Altitude in meters, expressed as * 1E3 (AMSL, not WGS84) where image was take
        {  set_bytes((int)(src) & -1L, 4, data,  21); }
        public int relative_alt_GET()//Altitude above ground in meters, expressed as * 1E3 where image was take
        {  return (int)((int) get_bytes(data,  25, 4)); }
        public void relative_alt_SET(int  src) //Altitude above ground in meters, expressed as * 1E3 where image was take
        {  set_bytes((int)(src) & -1L, 4, data,  25); }
        public float[] q_GET(float[]  dst_ch, int pos)  //Quaternion of camera orientation (w, x, y, z order, zero-rotation is 0, 0, 0, 0
        {
            for(int BYTE = 29, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                dst_ch[pos] = (float)(Float.intBitsToFloat((int) get_bytes(data,  BYTE, 4)));
            return dst_ch;
        }
        public float[] q_GET()//Quaternion of camera orientation (w, x, y, z order, zero-rotation is 0, 0, 0, 0
        {return q_GET(new float[4], 0);} public void q_SET(float[]  src, int pos)  //Quaternion of camera orientation (w, x, y, z order, zero-rotation is 0, 0, 0, 0
        {
            for(int BYTE =  29, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
                set_bytes(Float.floatToIntBits(src[pos]) & -1L, 4, data, BYTE);
        }
        public int image_index_GET()//Zero based index of this image (image count since armed -1
        {  return (int)((int) get_bytes(data,  45, 4)); }
        public void image_index_SET(int  src) //Zero based index of this image (image count since armed -1
        {  set_bytes((int)(src) & -1L, 4, data,  45); }
        public byte capture_result_GET()//Boolean indicating success (1) or failure (0) while capturing this image
        {  return (byte)((byte) get_bytes(data,  49, 1)); }
        public void capture_result_SET(byte  src) //Boolean indicating success (1) or failure (0) while capturing this image
        {  set_bytes((byte)(src) & -1L, 1, data,  49); }
        public String file_url_TRY(Bounds.Inside ph)//foo.jpg if camera provides an HTTP interface
//foo.jpg if camera provides an HTTP interface
        {
            if(ph.field_bit !=  402 && !try_visit_field(ph, 402)  ||  !try_visit_item(ph, 0)) return null;
            return new String(file_url_GET(ph, new char[ph.items], 0));
        }
        public char[] file_url_GET(Bounds.Inside ph, char[]  dst_ch, int pos) //foo.jpg if camera provides an HTTP interface
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int file_url_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  402 && !try_visit_field(ph, 402)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        } public void file_url_SET(String src, Bounds.Inside ph) //foo.jpg if camera provides an HTTP interface
        {file_url_SET(src.toCharArray(), 0, src.length(), ph);} public void file_url_SET(char[]  src, int pos, int items, Bounds.Inside ph) //foo.jpg if camera provides an HTTP interface
        {
            if(ph.field_bit != 402 && insert_field(ph, 402, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } static final Meta meta = new Meta(263, 0, 1, 1, 51, 402, 2, _SO);
    } public static class FLIGHT_INFORMATION extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        FLIGHT_INFORMATION() { super(meta, 0); }
        FLIGHT_INFORMATION(int bytes) { super(meta, bytes); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot
        {  return (get_bytes(data,  0, 4)); }
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot
        {  set_bytes((src) & -1L, 4, data,  0); }
        public long arming_time_utc_GET()//Timestamp at arming (microseconds since UNIX epoch) in UTC, 0 for unknow
        {  return (get_bytes(data,  4, 8)); }
        public void arming_time_utc_SET(long  src) //Timestamp at arming (microseconds since UNIX epoch) in UTC, 0 for unknow
        {  set_bytes((src) & -1L, 8, data,  4); }
        public long takeoff_time_utc_GET()//Timestamp at takeoff (microseconds since UNIX epoch) in UTC, 0 for unknow
        {  return (get_bytes(data,  12, 8)); }
        public void takeoff_time_utc_SET(long  src) //Timestamp at takeoff (microseconds since UNIX epoch) in UTC, 0 for unknow
        {  set_bytes((src) & -1L, 8, data,  12); }
        public long flight_uuid_GET()//Universally unique identifier (UUID) of flight, should correspond to name of logfile
        {  return (get_bytes(data,  20, 8)); }
        public void flight_uuid_SET(long  src) //Universally unique identifier (UUID) of flight, should correspond to name of logfile
        {  set_bytes((src) & -1L, 8, data,  20); }
        static final Meta meta = new Meta(264, 0, 1, 3, 28, 224);
    } public static class MOUNT_ORIENTATION extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        MOUNT_ORIENTATION() { super(meta, 0); }
        MOUNT_ORIENTATION(int bytes) { super(meta, bytes); }
        public long time_boot_ms_GET()//Timestamp (milliseconds since system boot
        {  return (get_bytes(data,  0, 4)); }
        public void time_boot_ms_SET(long  src) //Timestamp (milliseconds since system boot
        {  set_bytes((src) & -1L, 4, data,  0); }
        public float roll_GET()//Roll in degree
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  4, 4))); }
        public void roll_SET(float  src) //Roll in degree
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 4); }
        public float pitch_GET()//Pitch in degree
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  8, 4))); }
        public void pitch_SET(float  src) //Pitch in degree
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 8); }
        public float yaw_GET()//Yaw in degree
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public void yaw_SET(float  src) //Yaw in degree
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        static final Meta meta = new Meta(265, 0, 1, 0, 16, 128);
    } public static class LOGGING_DATA extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        LOGGING_DATA() { super(meta, 0); }
        LOGGING_DATA(int bytes) { super(meta, bytes); }
        public char sequence_GET()//sequence number (can wrap
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void sequence_SET(char  src) //sequence number (can wrap
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char target_system_GET()//system ID of the targe
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public void target_system_SET(char  src) //system ID of the targe
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        public char target_component_GET()//component ID of the targe
        {  return (char)((char) get_bytes(data,  3, 1)); }
        public void target_component_SET(char  src) //component ID of the targe
        {  set_bytes((char)(src) & -1L, 1, data,  3); }
        public char length_GET()//data lengt
        {  return (char)((char) get_bytes(data,  4, 1)); }
        public void length_SET(char  src) //data lengt
        {  set_bytes((char)(src) & -1L, 1, data,  4); }
        public char first_message_offset_GET() {  return (char)((char) get_bytes(data,  5, 1)); }
        public void first_message_offset_SET(char  src) {  set_bytes((char)(src) & -1L, 1, data,  5); }
        public char[] data__GET(char[]  dst_ch, int pos)  //logged dat
        {
            for(int BYTE = 6, dst_max = pos + 249; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] data__GET()//logged dat
        {return data__GET(new char[249], 0);} public void data__SET(char[]  src, int pos)  //logged dat
        {
            for(int BYTE =  6, src_max = pos + 249; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        static final Meta meta = new Meta(266, 1, 0, 0, 255, 2040);
    } public static class LOGGING_DATA_ACKED extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        LOGGING_DATA_ACKED() { super(meta, 0); }
        LOGGING_DATA_ACKED(int bytes) { super(meta, bytes); }
        public char sequence_GET()//sequence number (can wrap
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void sequence_SET(char  src) //sequence number (can wrap
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char target_system_GET()//system ID of the targe
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public void target_system_SET(char  src) //system ID of the targe
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        public char target_component_GET()//component ID of the targe
        {  return (char)((char) get_bytes(data,  3, 1)); }
        public void target_component_SET(char  src) //component ID of the targe
        {  set_bytes((char)(src) & -1L, 1, data,  3); }
        public char length_GET()//data lengt
        {  return (char)((char) get_bytes(data,  4, 1)); }
        public void length_SET(char  src) //data lengt
        {  set_bytes((char)(src) & -1L, 1, data,  4); }
        public char first_message_offset_GET() {  return (char)((char) get_bytes(data,  5, 1)); }
        public void first_message_offset_SET(char  src) {  set_bytes((char)(src) & -1L, 1, data,  5); }
        public char[] data__GET(char[]  dst_ch, int pos)  //logged dat
        {
            for(int BYTE = 6, dst_max = pos + 249; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] data__GET()//logged dat
        {return data__GET(new char[249], 0);} public void data__SET(char[]  src, int pos)  //logged dat
        {
            for(int BYTE =  6, src_max = pos + 249; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        static final Meta meta = new Meta(267, 1, 0, 0, 255, 2040);
    } public static class LOGGING_ACK extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        LOGGING_ACK() { super(meta, 0); }
        LOGGING_ACK(int bytes) { super(meta, bytes); }
        public char sequence_GET()//sequence number (must match the one in LOGGING_DATA_ACKED
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void sequence_SET(char  src) //sequence number (must match the one in LOGGING_DATA_ACKED
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char target_system_GET()//system ID of the targe
        {  return (char)((char) get_bytes(data,  2, 1)); }
        public void target_system_SET(char  src) //system ID of the targe
        {  set_bytes((char)(src) & -1L, 1, data,  2); }
        public char target_component_GET()//component ID of the targe
        {  return (char)((char) get_bytes(data,  3, 1)); }
        public void target_component_SET(char  src) //component ID of the targe
        {  set_bytes((char)(src) & -1L, 1, data,  3); }
        static final Meta meta = new Meta(268, 1, 0, 0, 4, 32);
    } public static class VIDEO_STREAM_INFORMATION extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        VIDEO_STREAM_INFORMATION() { super(meta, 0); }
        VIDEO_STREAM_INFORMATION(int bytes) { super(meta, bytes); }
        public char resolution_h_GET()//Resolution horizontal in pixel
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void resolution_h_SET(char  src) //Resolution horizontal in pixel
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char resolution_v_GET()//Resolution vertical in pixel
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public void resolution_v_SET(char  src) //Resolution vertical in pixel
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public char rotation_GET()//Video image rotation clockwis
        {  return (char)((char) get_bytes(data,  4, 2)); }
        public void rotation_SET(char  src) //Video image rotation clockwis
        {  set_bytes((char)(src) & -1L, 2, data,  4); }
        public long bitrate_GET()//Bit rate in bits per secon
        {  return (get_bytes(data,  6, 4)); }
        public void bitrate_SET(long  src) //Bit rate in bits per secon
        {  set_bytes((src) & -1L, 4, data,  6); }
        public char camera_id_GET()//Camera ID (1 for first, 2 for second, etc.
        {  return (char)((char) get_bytes(data,  10, 1)); }
        public void camera_id_SET(char  src) //Camera ID (1 for first, 2 for second, etc.
        {  set_bytes((char)(src) & -1L, 1, data,  10); }
        public char status_GET()//Current status of video streaming (0: not running, 1: in progress
        {  return (char)((char) get_bytes(data,  11, 1)); }
        public void status_SET(char  src) //Current status of video streaming (0: not running, 1: in progress
        {  set_bytes((char)(src) & -1L, 1, data,  11); }
        public float framerate_GET()//Frames per secon
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  12, 4))); }
        public void framerate_SET(float  src) //Frames per secon
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 12); }
        public String uri_TRY(Bounds.Inside ph)//Video stream UR
//Video stream UR
        {
            if(ph.field_bit !=  130 && !try_visit_field(ph, 130)  ||  !try_visit_item(ph, 0)) return null;
            return new String(uri_GET(ph, new char[ph.items], 0));
        }
        public char[] uri_GET(Bounds.Inside ph, char[]  dst_ch, int pos) //Video stream UR
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int uri_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  130 && !try_visit_field(ph, 130)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        } public void uri_SET(String src, Bounds.Inside ph) //Video stream UR
        {uri_SET(src.toCharArray(), 0, src.length(), ph);} public void uri_SET(char[]  src, int pos, int items, Bounds.Inside ph) //Video stream UR
        {
            if(ph.field_bit != 130 && insert_field(ph, 130, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } static final Meta meta = new Meta(269, 3, 1, 0, 17, 130, 2, _rO);
    } public static class SET_VIDEO_STREAM_SETTINGS extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        SET_VIDEO_STREAM_SETTINGS() { super(meta, 0); }
        SET_VIDEO_STREAM_SETTINGS(int bytes) { super(meta, bytes); }
        public char resolution_h_GET()//Resolution horizontal in pixels (set to -1 for highest resolution possible
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void resolution_h_SET(char  src) //Resolution horizontal in pixels (set to -1 for highest resolution possible
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char resolution_v_GET()//Resolution vertical in pixels (set to -1 for highest resolution possible
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public void resolution_v_SET(char  src) //Resolution vertical in pixels (set to -1 for highest resolution possible
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public char rotation_GET()//Video image rotation clockwise (0-359 degrees
        {  return (char)((char) get_bytes(data,  4, 2)); }
        public void rotation_SET(char  src) //Video image rotation clockwise (0-359 degrees
        {  set_bytes((char)(src) & -1L, 2, data,  4); }
        public long bitrate_GET()//Bit rate in bits per second (set to -1 for auto
        {  return (get_bytes(data,  6, 4)); }
        public void bitrate_SET(long  src) //Bit rate in bits per second (set to -1 for auto
        {  set_bytes((src) & -1L, 4, data,  6); }
        public char target_system_GET()//system ID of the targe
        {  return (char)((char) get_bytes(data,  10, 1)); }
        public void target_system_SET(char  src) //system ID of the targe
        {  set_bytes((char)(src) & -1L, 1, data,  10); }
        public char target_component_GET()//component ID of the targe
        {  return (char)((char) get_bytes(data,  11, 1)); }
        public void target_component_SET(char  src) //component ID of the targe
        {  set_bytes((char)(src) & -1L, 1, data,  11); }
        public char camera_id_GET()//Camera ID (1 for first, 2 for second, etc.
        {  return (char)((char) get_bytes(data,  12, 1)); }
        public void camera_id_SET(char  src) //Camera ID (1 for first, 2 for second, etc.
        {  set_bytes((char)(src) & -1L, 1, data,  12); }
        public float framerate_GET()//Frames per second (set to -1 for highest framerate possible
        {  return (float)(Float.intBitsToFloat((int) get_bytes(data,  13, 4))); }
        public void framerate_SET(float  src) //Frames per second (set to -1 for highest framerate possible
        {  set_bytes(Float.floatToIntBits(src) & -1L, 4, data, 13); }
        public String uri_TRY(Bounds.Inside ph)//Video stream UR
//Video stream UR
        {
            if(ph.field_bit !=  138 && !try_visit_field(ph, 138)  ||  !try_visit_item(ph, 0)) return null;
            return new String(uri_GET(ph, new char[ph.items], 0));
        }
        public char[] uri_GET(Bounds.Inside ph, char[]  dst_ch, int pos) //Video stream UR
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int uri_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  138 && !try_visit_field(ph, 138)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        } public void uri_SET(String src, Bounds.Inside ph) //Video stream UR
        {uri_SET(src.toCharArray(), 0, src.length(), ph);} public void uri_SET(char[]  src, int pos, int items, Bounds.Inside ph) //Video stream UR
        {
            if(ph.field_bit != 138 && insert_field(ph, 138, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } static final Meta meta = new Meta(270, 3, 1, 0, 18, 138, 2, _lO);
    } public static class WIFI_CONFIG_AP extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        WIFI_CONFIG_AP() { super(meta, 0); }
        WIFI_CONFIG_AP(int bytes) { super(meta, bytes); }
        public String ssid_TRY(Bounds.Inside ph)//Name of Wi-Fi network (SSID). Leave it blank to leave it unchanged
//Name of Wi-Fi network (SSID). Leave it blank to leave it unchanged
        {
            if(ph.field_bit !=  2 && !try_visit_field(ph, 2)  ||  !try_visit_item(ph, 0)) return null;
            return new String(ssid_GET(ph, new char[ph.items], 0));
        }
        public char[] ssid_GET(Bounds.Inside ph, char[]  dst_ch, int pos) //Name of Wi-Fi network (SSID). Leave it blank to leave it unchanged
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int ssid_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  2 && !try_visit_field(ph, 2)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        } public void ssid_SET(String src, Bounds.Inside ph) //Name of Wi-Fi network (SSID). Leave it blank to leave it unchanged
        {ssid_SET(src.toCharArray(), 0, src.length(), ph);} public void ssid_SET(char[]  src, int pos, int items, Bounds.Inside ph) //Name of Wi-Fi network (SSID). Leave it blank to leave it unchanged
        {
            if(ph.field_bit != 2 && insert_field(ph, 2, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } public String password_TRY(Bounds.Inside ph)//Password. Leave it blank for an open AP
//Password. Leave it blank for an open AP
        {
            if(ph.field_bit !=  3 && !try_visit_field(ph, 3)  ||  !try_visit_item(ph, 0)) return null;
            return new String(password_GET(ph, new char[ph.items], 0));
        }
        public char[] password_GET(Bounds.Inside ph, char[]  dst_ch, int pos) //Password. Leave it blank for an open AP
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int password_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  3 && !try_visit_field(ph, 3)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        } public void password_SET(String src, Bounds.Inside ph) //Password. Leave it blank for an open AP
        {password_SET(src.toCharArray(), 0, src.length(), ph);} public void password_SET(char[]  src, int pos, int items, Bounds.Inside ph) //Password. Leave it blank for an open AP
        {
            if(ph.field_bit != 3 && insert_field(ph, 3, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } static final Meta meta = new Meta(299, 0, 0, 0, 1, 2, 2, _YO, _LO);
    } public static class PROTOCOL_VERSION extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        PROTOCOL_VERSION() { super(meta, 0); }
        PROTOCOL_VERSION(int bytes) { super(meta, bytes); }
        public char version_GET()//Currently active MAVLink version number * 100: v1.0 is 100, v2.0 is 200, etc
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void version_SET(char  src) //Currently active MAVLink version number * 100: v1.0 is 100, v2.0 is 200, etc
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char min_version_GET()//Minimum MAVLink version supporte
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public void min_version_SET(char  src) //Minimum MAVLink version supporte
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public char max_version_GET()//Maximum MAVLink version supported (set to the same value as version by default
        {  return (char)((char) get_bytes(data,  4, 2)); }
        public void max_version_SET(char  src) //Maximum MAVLink version supported (set to the same value as version by default
        {  set_bytes((char)(src) & -1L, 2, data,  4); }
        public char[] spec_version_hash_GET(char[]  dst_ch, int pos)  //The first 8 bytes (not characters printed in hex!) of the git hash
        {
            for(int BYTE = 6, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] spec_version_hash_GET()//The first 8 bytes (not characters printed in hex!) of the git hash
        {return spec_version_hash_GET(new char[8], 0);} public void spec_version_hash_SET(char[]  src, int pos)  //The first 8 bytes (not characters printed in hex!) of the git hash
        {
            for(int BYTE =  6, src_max = pos + 8; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        public char[] library_version_hash_GET(char[]  dst_ch, int pos)  //The first 8 bytes (not characters printed in hex!) of the git hash
        {
            for(int BYTE = 14, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] library_version_hash_GET()//The first 8 bytes (not characters printed in hex!) of the git hash
        {return library_version_hash_GET(new char[8], 0);} public void library_version_hash_SET(char[]  src, int pos)  //The first 8 bytes (not characters printed in hex!) of the git hash
        {
            for(int BYTE =  14, src_max = pos + 8; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        static final Meta meta = new Meta(300, 3, 0, 0, 22, 176);
    } public static class UAVCAN_NODE_STATUS extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        UAVCAN_NODE_STATUS() { super(meta, 0); }
        UAVCAN_NODE_STATUS(int bytes) { super(meta, bytes); }
        public char vendor_specific_status_code_GET()//Vendor-specific status information
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void vendor_specific_status_code_SET(char  src) //Vendor-specific status information
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public long uptime_sec_GET()//The number of seconds since the start-up of the node
        {  return (get_bytes(data,  2, 4)); }
        public void uptime_sec_SET(long  src) //The number of seconds since the start-up of the node
        {  set_bytes((src) & -1L, 4, data,  2); }
        public long time_usec_GET()//Timestamp (microseconds since UNIX epoch or microseconds since system boot
        {  return (get_bytes(data,  6, 8)); }
        public void time_usec_SET(long  src) //Timestamp (microseconds since UNIX epoch or microseconds since system boot
        {  set_bytes((src) & -1L, 8, data,  6); }
        public char sub_mode_GET()//Not used currently
        {  return (char)((char) get_bytes(data,  14, 1)); }
        public void sub_mode_SET(char  src) //Not used currently
        {  set_bytes((char)(src) & -1L, 1, data,  14); }
        public @UAVCAN_NODE_HEALTH int health_GET()//Generalized node health status
        {  return  0 + (int)get_bits(data, 120, 3); }
        public void health_SET(@UAVCAN_NODE_HEALTH int  src) //Generalized node health status
        {  set_bits(- 0 +   src, 3, data, 120); }
        public @UAVCAN_NODE_MODE int mode_GET()//Generalized operating mode
        {
            switch((int)get_bits(data, 123, 3))
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
            assert(false);//("Unknown enum ID " + id);
            return  Integer.MIN_VALUE;
        }
        public void mode_SET(@UAVCAN_NODE_MODE int  src) //Generalized operating mode
        {
            long id = 0;
            switch(src)
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
                    assert(false);//("Unknown enum" + id);
            }
            set_bits(id, 3, data, 123);
        }
        static final Meta meta = new Meta(310, 1, 1, 1, 16, 126);
    } public static class UAVCAN_NODE_INFO extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        UAVCAN_NODE_INFO() { super(meta, 0); }
        UAVCAN_NODE_INFO(int bytes) { super(meta, bytes); }
        public long uptime_sec_GET()//The number of seconds since the start-up of the node
        {  return (get_bytes(data,  0, 4)); }
        public void uptime_sec_SET(long  src) //The number of seconds since the start-up of the node
        {  set_bytes((src) & -1L, 4, data,  0); }
        public long sw_vcs_commit_GET()//Version control system (VCS) revision identifier (e.g. git short commit hash). Zero if unknown
        {  return (get_bytes(data,  4, 4)); }
        public void sw_vcs_commit_SET(long  src) //Version control system (VCS) revision identifier (e.g. git short commit hash). Zero if unknown
        {  set_bytes((src) & -1L, 4, data,  4); }
        public long time_usec_GET()//Timestamp (microseconds since UNIX epoch or microseconds since system boot
        {  return (get_bytes(data,  8, 8)); }
        public void time_usec_SET(long  src) //Timestamp (microseconds since UNIX epoch or microseconds since system boot
        {  set_bytes((src) & -1L, 8, data,  8); }
        public char hw_version_major_GET()//Hardware major version number
        {  return (char)((char) get_bytes(data,  16, 1)); }
        public void hw_version_major_SET(char  src) //Hardware major version number
        {  set_bytes((char)(src) & -1L, 1, data,  16); }
        public char hw_version_minor_GET()//Hardware minor version number
        {  return (char)((char) get_bytes(data,  17, 1)); }
        public void hw_version_minor_SET(char  src) //Hardware minor version number
        {  set_bytes((char)(src) & -1L, 1, data,  17); }
        public char[] hw_unique_id_GET(char[]  dst_ch, int pos)  //Hardware unique 128-bit ID
        {
            for(int BYTE = 18, dst_max = pos + 16; pos < dst_max ; pos++,  BYTE += 1)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 1));
            return dst_ch;
        }
        public char[] hw_unique_id_GET()//Hardware unique 128-bit ID
        {return hw_unique_id_GET(new char[16], 0);} public void hw_unique_id_SET(char[]  src, int pos)  //Hardware unique 128-bit ID
        {
            for(int BYTE =  18, src_max = pos + 16; pos < src_max; pos++, BYTE += 1)
                set_bytes((char)(src[pos]) & -1L, 1, data,  BYTE);
        }
        public char sw_version_major_GET()//Software major version number
        {  return (char)((char) get_bytes(data,  34, 1)); }
        public void sw_version_major_SET(char  src) //Software major version number
        {  set_bytes((char)(src) & -1L, 1, data,  34); }
        public char sw_version_minor_GET()//Software minor version number
        {  return (char)((char) get_bytes(data,  35, 1)); }
        public void sw_version_minor_SET(char  src) //Software minor version number
        {  set_bytes((char)(src) & -1L, 1, data,  35); }
        public String name_TRY(Bounds.Inside ph)//Node name string. For example, "sapog.px4.io"
//Node name string. For example, "sapog.px4.io"
        {
            if(ph.field_bit !=  288 && !try_visit_field(ph, 288)  ||  !try_visit_item(ph, 0)) return null;
            return new String(name_GET(ph, new char[ph.items], 0));
        }
        public char[] name_GET(Bounds.Inside ph, char[]  dst_ch, int pos) //Node name string. For example, "sapog.px4.io"
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int name_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  288 && !try_visit_field(ph, 288)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        } public void name_SET(String src, Bounds.Inside ph) //Node name string. For example, "sapog.px4.io"
        {name_SET(src.toCharArray(), 0, src.length(), ph);} public void name_SET(char[]  src, int pos, int items, Bounds.Inside ph) //Node name string. For example, "sapog.px4.io"
        {
            if(ph.field_bit != 288 && insert_field(ph, 288, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } static final Meta meta = new Meta(311, 0, 2, 1, 37, 288, 0, _JX);
    } public static class PARAM_EXT_REQUEST_READ extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        PARAM_EXT_REQUEST_READ() { super(meta, 0); }
        PARAM_EXT_REQUEST_READ(int bytes) { super(meta, bytes); }
        public char target_system_GET()//System I
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public void target_system_SET(char  src) //System I
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public char target_component_GET()//Component I
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public void target_component_SET(char  src) //Component I
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public short param_index_GET()//Parameter index. Set to -1 to use the Parameter ID field as identifier (else param_id will be ignore
        {  return (short)((short) get_bytes(data,  2, 2)); }
        public void param_index_SET(short  src) //Parameter index. Set to -1 to use the Parameter ID field as identifier (else param_id will be ignore
        {  set_bytes((short)(src) & -1L, 2, data,  2); }
        public String param_id_TRY(Bounds.Inside ph)
        {
            if(ph.field_bit !=  32 && !try_visit_field(ph, 32)  ||  !try_visit_item(ph, 0)) return null;
            return new String(param_id_GET(ph, new char[ph.items], 0));
        }
        public char[] param_id_GET(Bounds.Inside ph, char[]  dst_ch, int pos)
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int param_id_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  32 && !try_visit_field(ph, 32)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        } public void param_id_SET(String src, Bounds.Inside ph) {param_id_SET(src.toCharArray(), 0, src.length(), ph);} public void param_id_SET(char[]  src, int pos, int items, Bounds.Inside ph)
        {
            if(ph.field_bit != 32 && insert_field(ph, 32, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } static final Meta meta = new Meta(320, 0, 0, 0, 5, 32, 0, _nX);
    } public static class PARAM_EXT_REQUEST_LIST extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        PARAM_EXT_REQUEST_LIST() { super(meta, 0); }
        PARAM_EXT_REQUEST_LIST(int bytes) { super(meta, bytes); }
        public char target_system_GET()//System I
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public void target_system_SET(char  src) //System I
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public char target_component_GET()//Component I
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public void target_component_SET(char  src) //Component I
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        static final Meta meta = new Meta(321, 0, 0, 0, 2, 16);
    } public static class PARAM_EXT_VALUE extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        PARAM_EXT_VALUE() { super(meta, 0); }
        PARAM_EXT_VALUE(int bytes) { super(meta, bytes); }
        public char param_count_GET()//Total number of parameter
        {  return (char)((char) get_bytes(data,  0, 2)); }
        public void param_count_SET(char  src) //Total number of parameter
        {  set_bytes((char)(src) & -1L, 2, data,  0); }
        public char param_index_GET()//Index of this paramete
        {  return (char)((char) get_bytes(data,  2, 2)); }
        public void param_index_SET(char  src) //Index of this paramete
        {  set_bytes((char)(src) & -1L, 2, data,  2); }
        public @MAV_PARAM_EXT_TYPE int param_type_GET()//Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types
        {  return  1 + (int)get_bits(data, 32, 4); }
        public void param_type_SET(@MAV_PARAM_EXT_TYPE int  src) //Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types
        {  set_bits(- 1 +   src, 4, data, 32); }
        public String param_id_TRY(Bounds.Inside ph)
        {
            if(ph.field_bit !=  38 && !try_visit_field(ph, 38)  ||  !try_visit_item(ph, 0)) return null;
            return new String(param_id_GET(ph, new char[ph.items], 0));
        }
        public char[] param_id_GET(Bounds.Inside ph, char[]  dst_ch, int pos)
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int param_id_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  38 && !try_visit_field(ph, 38)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        } public void param_id_SET(String src, Bounds.Inside ph) {param_id_SET(src.toCharArray(), 0, src.length(), ph);} public void param_id_SET(char[]  src, int pos, int items, Bounds.Inside ph)
        {
            if(ph.field_bit != 38 && insert_field(ph, 38, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } public String param_value_TRY(Bounds.Inside ph)//Parameter valu
//Parameter valu
        {
            if(ph.field_bit !=  39 && !try_visit_field(ph, 39)  ||  !try_visit_item(ph, 0)) return null;
            return new String(param_value_GET(ph, new char[ph.items], 0));
        }
        public char[] param_value_GET(Bounds.Inside ph, char[]  dst_ch, int pos) //Parameter valu
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int param_value_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  39 && !try_visit_field(ph, 39)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        } public void param_value_SET(String src, Bounds.Inside ph) //Parameter valu
        {param_value_SET(src.toCharArray(), 0, src.length(), ph);} public void param_value_SET(char[]  src, int pos, int items, Bounds.Inside ph) //Parameter valu
        {
            if(ph.field_bit != 39 && insert_field(ph, 39, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } static final Meta meta = new Meta(322, 2, 0, 0, 5, 38, 2, _XX, _sX);
    } public static class PARAM_EXT_SET extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        PARAM_EXT_SET() { super(meta, 0); }
        PARAM_EXT_SET(int bytes) { super(meta, bytes); }
        public char target_system_GET()//System I
        {  return (char)((char) get_bytes(data,  0, 1)); }
        public void target_system_SET(char  src) //System I
        {  set_bytes((char)(src) & -1L, 1, data,  0); }
        public char target_component_GET()//Component I
        {  return (char)((char) get_bytes(data,  1, 1)); }
        public void target_component_SET(char  src) //Component I
        {  set_bytes((char)(src) & -1L, 1, data,  1); }
        public @MAV_PARAM_EXT_TYPE int param_type_GET()//Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types
        {  return  1 + (int)get_bits(data, 16, 4); }
        public void param_type_SET(@MAV_PARAM_EXT_TYPE int  src) //Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types
        {  set_bits(- 1 +   src, 4, data, 16); }
        public String param_id_TRY(Bounds.Inside ph)
        {
            if(ph.field_bit !=  22 && !try_visit_field(ph, 22)  ||  !try_visit_item(ph, 0)) return null;
            return new String(param_id_GET(ph, new char[ph.items], 0));
        }
        public char[] param_id_GET(Bounds.Inside ph, char[]  dst_ch, int pos)
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int param_id_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  22 && !try_visit_field(ph, 22)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        } public void param_id_SET(String src, Bounds.Inside ph) {param_id_SET(src.toCharArray(), 0, src.length(), ph);} public void param_id_SET(char[]  src, int pos, int items, Bounds.Inside ph)
        {
            if(ph.field_bit != 22 && insert_field(ph, 22, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } public String param_value_TRY(Bounds.Inside ph)//Parameter valu
//Parameter valu
        {
            if(ph.field_bit !=  23 && !try_visit_field(ph, 23)  ||  !try_visit_item(ph, 0)) return null;
            return new String(param_value_GET(ph, new char[ph.items], 0));
        }
        public char[] param_value_GET(Bounds.Inside ph, char[]  dst_ch, int pos) //Parameter valu
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int param_value_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  23 && !try_visit_field(ph, 23)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        } public void param_value_SET(String src, Bounds.Inside ph) //Parameter valu
        {param_value_SET(src.toCharArray(), 0, src.length(), ph);} public void param_value_SET(char[]  src, int pos, int items, Bounds.Inside ph) //Parameter valu
        {
            if(ph.field_bit != 23 && insert_field(ph, 23, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } static final Meta meta = new Meta(323, 0, 0, 0, 3, 22, 2, _gX, _wX);
    } public static class PARAM_EXT_ACK extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        PARAM_EXT_ACK() { super(meta, 0); }
        PARAM_EXT_ACK(int bytes) { super(meta, bytes); }
        public @MAV_PARAM_EXT_TYPE int param_type_GET()//Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types
        {  return  1 + (int)get_bits(data, 0, 4); }
        public void param_type_SET(@MAV_PARAM_EXT_TYPE int  src) //Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types
        {  set_bits(- 1 +   src, 4, data, 0); }
        public @PARAM_ACK int param_result_GET()//Result code: see the PARAM_ACK enum for possible codes
        {  return  0 + (int)get_bits(data, 4, 3); }
        public void param_result_SET(@PARAM_ACK int  src) //Result code: see the PARAM_ACK enum for possible codes
        {  set_bits(- 0 +   src, 3, data, 4); }
        public String param_id_TRY(Bounds.Inside ph)
        {
            if(ph.field_bit !=  9 && !try_visit_field(ph, 9)  ||  !try_visit_item(ph, 0)) return null;
            return new String(param_id_GET(ph, new char[ph.items], 0));
        }
        public char[] param_id_GET(Bounds.Inside ph, char[]  dst_ch, int pos)
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int param_id_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  9 && !try_visit_field(ph, 9)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        } public void param_id_SET(String src, Bounds.Inside ph) {param_id_SET(src.toCharArray(), 0, src.length(), ph);} public void param_id_SET(char[]  src, int pos, int items, Bounds.Inside ph)
        {
            if(ph.field_bit != 9 && insert_field(ph, 9, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } public String param_value_TRY(Bounds.Inside ph)//Parameter value (new value if PARAM_ACK_ACCEPTED, current value otherwise
//Parameter value (new value if PARAM_ACK_ACCEPTED, current value otherwise
        {
            if(ph.field_bit !=  10 && !try_visit_field(ph, 10)  ||  !try_visit_item(ph, 0)) return null;
            return new String(param_value_GET(ph, new char[ph.items], 0));
        }
        public char[] param_value_GET(Bounds.Inside ph, char[]  dst_ch, int pos) //Parameter value (new value if PARAM_ACK_ACCEPTED, current value otherwise
        {
            for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public int param_value_LEN(Bounds.Inside ph)
        {
            return (ph.field_bit !=  10 && !try_visit_field(ph, 10)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
        } public void param_value_SET(String src, Bounds.Inside ph) //Parameter value (new value if PARAM_ACK_ACCEPTED, current value otherwise
        {param_value_SET(src.toCharArray(), 0, src.length(), ph);} public void param_value_SET(char[]  src, int pos, int items, Bounds.Inside ph) //Parameter value (new value if PARAM_ACK_ACCEPTED, current value otherwise
        {
            if(ph.field_bit != 10 && insert_field(ph, 10, items) || ! try_visit_item(ph, 0))
                insert_item(ph, 0, items);
            for(int BYTE =  ph.BYTE, src_max = pos + ph.items; pos < src_max; pos++, BYTE += 2)
                set_bytes((short)(src[pos]) & -1L, 2, data,  BYTE);
        } static final Meta meta = new Meta(324, 0, 0, 0, 2, 9, 2, _kX, _tX);
    } public static class OBSTACLE_DISTANCE extends Pack  implements LoopBackDemoChannel.Sendable, LoopBackDemoChannel_ADV.Sendable
    {
        OBSTACLE_DISTANCE() { super(meta, 0); }
        OBSTACLE_DISTANCE(int bytes) { super(meta, bytes); }
        public char[] distances_GET(char[]  dst_ch, int pos)
        {
            for(int BYTE = 0, dst_max = pos + 72; pos < dst_max ; pos++,  BYTE += 2)
                dst_ch[pos] = (char)((char) get_bytes(data,  BYTE, 2));
            return dst_ch;
        }
        public char[] distances_GET() {return distances_GET(new char[72], 0);} public void distances_SET(char[]  src, int pos)
        {
            for(int BYTE =  0, src_max = pos + 72; pos < src_max; pos++, BYTE += 2)
                set_bytes((char)(src[pos]) & -1L, 2, data,  BYTE);
        }
        public char min_distance_GET()//Minimum distance the sensor can measure in centimeter
        {  return (char)((char) get_bytes(data,  144, 2)); }
        public void min_distance_SET(char  src) //Minimum distance the sensor can measure in centimeter
        {  set_bytes((char)(src) & -1L, 2, data,  144); }
        public char max_distance_GET()//Maximum distance the sensor can measure in centimeter
        {  return (char)((char) get_bytes(data,  146, 2)); }
        public void max_distance_SET(char  src) //Maximum distance the sensor can measure in centimeter
        {  set_bytes((char)(src) & -1L, 2, data,  146); }
        public long time_usec_GET()//Timestamp (microseconds since system boot or since UNIX epoch
        {  return (get_bytes(data,  148, 8)); }
        public void time_usec_SET(long  src) //Timestamp (microseconds since system boot or since UNIX epoch
        {  set_bytes((src) & -1L, 8, data,  148); }
        public char increment_GET()//Angular width in degrees of each array element
        {  return (char)((char) get_bytes(data,  156, 1)); }
        public void increment_SET(char  src) //Angular width in degrees of each array element
        {  set_bytes((char)(src) & -1L, 1, data,  156); }
        public @MAV_DISTANCE_SENSOR int sensor_type_GET()//Class id of the distance sensor type
        {  return  0 + (int)get_bits(data, 1256, 3); }
        public void sensor_type_SET(@MAV_DISTANCE_SENSOR int  src) //Class id of the distance sensor type
        {  set_bits(- 0 +   src, 3, data, 1256); }
        static final Meta meta = new Meta(330, 74, 0, 1, 158, 1259);
    }

    public static class LoopBackDemoChannel  extends Channel
    {
        static {pack_id_bytes = 2; }

        public static  LoopBackDemoChannel instance = new LoopBackDemoChannel();

        public final java.io.InputStream inputStream = new  InputStream();
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
        public static NAV_FILTER_BIAS new_NAV_FILTER_BIAS() {return new  NAV_FILTER_BIAS();}
        public static RADIO_CALIBRATION new_RADIO_CALIBRATION() {return new  RADIO_CALIBRATION();}
        public static UALBERTA_SYS_STATUS new_UALBERTA_SYS_STATUS() {return new  UALBERTA_SYS_STATUS();}
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

        public void send(Sendable pack) { sendout_packs.add((Pack) pack);}
        protected final Queue<Pack> sendout_packs = new ConcurrentLinkedQueue<Pack>()
        {
            @Override public boolean add(Pack pack)
            {
                synchronized(this)
                {
                    boolean ret = super.add(pack);
                    this.notify();
                    return ret;
                }
            }
        };

        public boolean waitingSendoutPack()
        {
            try
            {
                synchronized(sendout_packs)
                {
                    while(sendout_packs.size() == 0) sendout_packs.wait();
                }
                return true;
            }
            catch(InterruptedException e) {}
            return false;
        }

        public final java.io.OutputStream outputStream = new  OutputStream();
        protected final Queue<Pack> received_packs = new ConcurrentLinkedQueue<Pack>()
        {
            @Override public boolean add(Pack pack)
            {
                synchronized(this)
                {
                    boolean ret = super.add(pack);
                    this.notify();
                    return ret;
                }
            }
        };

        public boolean waitingReceivedPack()
        {
            try
            {
                synchronized(received_packs)
                {
                    while(received_packs.size() == 0) received_packs.wait();
                    return true;
                }
            }
            catch(InterruptedException e) {}
            return false;
        }


        private final Bounds.Inside ph    = new Bounds.Inside();


        @Override protected Pack process(Pack pack, int id)
        {
            for(boolean LOOP = false; ;)
            {
                switch(id)
                {
                    default:
                        assert(false);
                        return null;
                    case 0:
                        if(pack == null) return new HEARTBEAT();
                        final HEARTBEAT pi0 = (HEARTBEAT) pack;
                        ph.setPack(pi0);
                        on_HEARTBEAT.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi0));
                        if(LOOP) break;
                        return null;
                    case 1:
                        if(pack == null) return new SYS_STATUS();
                        final SYS_STATUS pi1 = (SYS_STATUS) pack;
                        ph.setPack(pi1);
                        on_SYS_STATUS.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi1));
                        if(LOOP) break;
                        return null;
                    case 2:
                        if(pack == null) return new SYSTEM_TIME();
                        final SYSTEM_TIME pi2 = (SYSTEM_TIME) pack;
                        ph.setPack(pi2);
                        on_SYSTEM_TIME.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi2));
                        if(LOOP) break;
                        return null;
                    case 3:
                        if(pack == null) return new POSITION_TARGET_LOCAL_NED();
                        final POSITION_TARGET_LOCAL_NED pi3 = (POSITION_TARGET_LOCAL_NED) pack;
                        ph.setPack(pi3);
                        on_POSITION_TARGET_LOCAL_NED.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi3));
                        if(LOOP) break;
                        return null;
                    case 4:
                        if(pack == null) return new PING();
                        final PING pi4 = (PING) pack;
                        ph.setPack(pi4);
                        on_PING.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi4));
                        if(LOOP) break;
                        return null;
                    case 5:
                        if(pack == null) return new CHANGE_OPERATOR_CONTROL(-1);
                        final CHANGE_OPERATOR_CONTROL pi5 = (CHANGE_OPERATOR_CONTROL) pack;
                        ph.setPack(pi5);
                        on_CHANGE_OPERATOR_CONTROL.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi5));
                        if(LOOP) break;
                        return null;
                    case 6:
                        if(pack == null) return new CHANGE_OPERATOR_CONTROL_ACK();
                        final CHANGE_OPERATOR_CONTROL_ACK pi6 = (CHANGE_OPERATOR_CONTROL_ACK) pack;
                        ph.setPack(pi6);
                        on_CHANGE_OPERATOR_CONTROL_ACK.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi6));
                        if(LOOP) break;
                        return null;
                    case 7:
                        if(pack == null) return new AUTH_KEY(-1);
                        final AUTH_KEY pi7 = (AUTH_KEY) pack;
                        ph.setPack(pi7);
                        on_AUTH_KEY.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi7));
                        if(LOOP) break;
                        return null;
                    case 11:
                        if(pack == null) return new SET_MODE();
                        final SET_MODE pi11 = (SET_MODE) pack;
                        ph.setPack(pi11);
                        on_SET_MODE.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi11));
                        if(LOOP) break;
                        return null;
                    case 20:
                        if(pack == null) return new PARAM_REQUEST_READ(-1);
                        final PARAM_REQUEST_READ pi20 = (PARAM_REQUEST_READ) pack;
                        ph.setPack(pi20);
                        on_PARAM_REQUEST_READ.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi20));
                        if(LOOP) break;
                        return null;
                    case 21:
                        if(pack == null) return new PARAM_REQUEST_LIST();
                        final PARAM_REQUEST_LIST pi21 = (PARAM_REQUEST_LIST) pack;
                        ph.setPack(pi21);
                        on_PARAM_REQUEST_LIST.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi21));
                        if(LOOP) break;
                        return null;
                    case 22:
                        if(pack == null) return new PARAM_VALUE(-1);
                        final PARAM_VALUE pi22 = (PARAM_VALUE) pack;
                        ph.setPack(pi22);
                        on_PARAM_VALUE.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi22));
                        if(LOOP) break;
                        return null;
                    case 23:
                        if(pack == null) return new PARAM_SET(-1);
                        final PARAM_SET pi23 = (PARAM_SET) pack;
                        ph.setPack(pi23);
                        on_PARAM_SET.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi23));
                        if(LOOP) break;
                        return null;
                    case 24:
                        if(pack == null) return new GPS_RAW_INT(-1);
                        final GPS_RAW_INT pi24 = (GPS_RAW_INT) pack;
                        ph.setPack(pi24);
                        on_GPS_RAW_INT.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi24));
                        if(LOOP) break;
                        return null;
                    case 25:
                        if(pack == null) return new GPS_STATUS();
                        final GPS_STATUS pi25 = (GPS_STATUS) pack;
                        ph.setPack(pi25);
                        on_GPS_STATUS.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi25));
                        if(LOOP) break;
                        return null;
                    case 26:
                        if(pack == null) return new SCALED_IMU();
                        final SCALED_IMU pi26 = (SCALED_IMU) pack;
                        ph.setPack(pi26);
                        on_SCALED_IMU.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi26));
                        if(LOOP) break;
                        return null;
                    case 27:
                        if(pack == null) return new RAW_IMU();
                        final RAW_IMU pi27 = (RAW_IMU) pack;
                        ph.setPack(pi27);
                        on_RAW_IMU.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi27));
                        if(LOOP) break;
                        return null;
                    case 28:
                        if(pack == null) return new RAW_PRESSURE();
                        final RAW_PRESSURE pi28 = (RAW_PRESSURE) pack;
                        ph.setPack(pi28);
                        on_RAW_PRESSURE.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi28));
                        if(LOOP) break;
                        return null;
                    case 29:
                        if(pack == null) return new SCALED_PRESSURE();
                        final SCALED_PRESSURE pi29 = (SCALED_PRESSURE) pack;
                        ph.setPack(pi29);
                        on_SCALED_PRESSURE.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi29));
                        if(LOOP) break;
                        return null;
                    case 30:
                        if(pack == null) return new ATTITUDE();
                        final ATTITUDE pi30 = (ATTITUDE) pack;
                        ph.setPack(pi30);
                        on_ATTITUDE.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi30));
                        if(LOOP) break;
                        return null;
                    case 31:
                        if(pack == null) return new ATTITUDE_QUATERNION();
                        final ATTITUDE_QUATERNION pi31 = (ATTITUDE_QUATERNION) pack;
                        ph.setPack(pi31);
                        on_ATTITUDE_QUATERNION.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi31));
                        if(LOOP) break;
                        return null;
                    case 32:
                        if(pack == null) return new LOCAL_POSITION_NED();
                        final LOCAL_POSITION_NED pi32 = (LOCAL_POSITION_NED) pack;
                        ph.setPack(pi32);
                        on_LOCAL_POSITION_NED.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi32));
                        if(LOOP) break;
                        return null;
                    case 33:
                        if(pack == null) return new GLOBAL_POSITION_INT();
                        final GLOBAL_POSITION_INT pi33 = (GLOBAL_POSITION_INT) pack;
                        ph.setPack(pi33);
                        on_GLOBAL_POSITION_INT.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi33));
                        if(LOOP) break;
                        return null;
                    case 34:
                        if(pack == null) return new RC_CHANNELS_SCALED();
                        final RC_CHANNELS_SCALED pi34 = (RC_CHANNELS_SCALED) pack;
                        ph.setPack(pi34);
                        on_RC_CHANNELS_SCALED.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi34));
                        if(LOOP) break;
                        return null;
                    case 35:
                        if(pack == null) return new RC_CHANNELS_RAW();
                        final RC_CHANNELS_RAW pi35 = (RC_CHANNELS_RAW) pack;
                        ph.setPack(pi35);
                        on_RC_CHANNELS_RAW.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi35));
                        if(LOOP) break;
                        return null;
                    case 36:
                        if(pack == null) return new SERVO_OUTPUT_RAW(-1);
                        final SERVO_OUTPUT_RAW pi36 = (SERVO_OUTPUT_RAW) pack;
                        ph.setPack(pi36);
                        on_SERVO_OUTPUT_RAW.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi36));
                        if(LOOP) break;
                        return null;
                    case 37:
                        if(pack == null) return new MISSION_REQUEST_PARTIAL_LIST();
                        final MISSION_REQUEST_PARTIAL_LIST pi37 = (MISSION_REQUEST_PARTIAL_LIST) pack;
                        ph.setPack(pi37);
                        on_MISSION_REQUEST_PARTIAL_LIST.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi37));
                        if(LOOP) break;
                        return null;
                    case 38:
                        if(pack == null) return new MISSION_WRITE_PARTIAL_LIST();
                        final MISSION_WRITE_PARTIAL_LIST pi38 = (MISSION_WRITE_PARTIAL_LIST) pack;
                        ph.setPack(pi38);
                        on_MISSION_WRITE_PARTIAL_LIST.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi38));
                        if(LOOP) break;
                        return null;
                    case 39:
                        if(pack == null) return new MISSION_ITEM();
                        final MISSION_ITEM pi39 = (MISSION_ITEM) pack;
                        ph.setPack(pi39);
                        on_MISSION_ITEM.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi39));
                        if(LOOP) break;
                        return null;
                    case 40:
                        if(pack == null) return new MISSION_REQUEST();
                        final MISSION_REQUEST pi40 = (MISSION_REQUEST) pack;
                        ph.setPack(pi40);
                        on_MISSION_REQUEST.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi40));
                        if(LOOP) break;
                        return null;
                    case 41:
                        if(pack == null) return new MISSION_SET_CURRENT();
                        final MISSION_SET_CURRENT pi41 = (MISSION_SET_CURRENT) pack;
                        ph.setPack(pi41);
                        on_MISSION_SET_CURRENT.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi41));
                        if(LOOP) break;
                        return null;
                    case 42:
                        if(pack == null) return new MISSION_CURRENT();
                        final MISSION_CURRENT pi42 = (MISSION_CURRENT) pack;
                        ph.setPack(pi42);
                        on_MISSION_CURRENT.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi42));
                        if(LOOP) break;
                        return null;
                    case 43:
                        if(pack == null) return new MISSION_REQUEST_LIST();
                        final MISSION_REQUEST_LIST pi43 = (MISSION_REQUEST_LIST) pack;
                        ph.setPack(pi43);
                        on_MISSION_REQUEST_LIST.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi43));
                        if(LOOP) break;
                        return null;
                    case 44:
                        if(pack == null) return new MISSION_COUNT();
                        final MISSION_COUNT pi44 = (MISSION_COUNT) pack;
                        ph.setPack(pi44);
                        on_MISSION_COUNT.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi44));
                        if(LOOP) break;
                        return null;
                    case 45:
                        if(pack == null) return new MISSION_CLEAR_ALL();
                        final MISSION_CLEAR_ALL pi45 = (MISSION_CLEAR_ALL) pack;
                        ph.setPack(pi45);
                        on_MISSION_CLEAR_ALL.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi45));
                        if(LOOP) break;
                        return null;
                    case 46:
                        if(pack == null) return new MISSION_ITEM_REACHED();
                        final MISSION_ITEM_REACHED pi46 = (MISSION_ITEM_REACHED) pack;
                        ph.setPack(pi46);
                        on_MISSION_ITEM_REACHED.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi46));
                        if(LOOP) break;
                        return null;
                    case 47:
                        if(pack == null) return new MISSION_ACK();
                        final MISSION_ACK pi47 = (MISSION_ACK) pack;
                        ph.setPack(pi47);
                        on_MISSION_ACK.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi47));
                        if(LOOP) break;
                        return null;
                    case 48:
                        if(pack == null) return new SET_GPS_GLOBAL_ORIGIN(-1);
                        final SET_GPS_GLOBAL_ORIGIN pi48 = (SET_GPS_GLOBAL_ORIGIN) pack;
                        ph.setPack(pi48);
                        on_SET_GPS_GLOBAL_ORIGIN.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi48));
                        if(LOOP) break;
                        return null;
                    case 49:
                        if(pack == null) return new GPS_GLOBAL_ORIGIN(-1);
                        final GPS_GLOBAL_ORIGIN pi49 = (GPS_GLOBAL_ORIGIN) pack;
                        ph.setPack(pi49);
                        on_GPS_GLOBAL_ORIGIN.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi49));
                        if(LOOP) break;
                        return null;
                    case 50:
                        if(pack == null) return new PARAM_MAP_RC(-1);
                        final PARAM_MAP_RC pi50 = (PARAM_MAP_RC) pack;
                        ph.setPack(pi50);
                        on_PARAM_MAP_RC.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi50));
                        if(LOOP) break;
                        return null;
                    case 51:
                        if(pack == null) return new MISSION_REQUEST_INT();
                        final MISSION_REQUEST_INT pi51 = (MISSION_REQUEST_INT) pack;
                        ph.setPack(pi51);
                        on_MISSION_REQUEST_INT.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi51));
                        if(LOOP) break;
                        return null;
                    case 54:
                        if(pack == null) return new SAFETY_SET_ALLOWED_AREA();
                        final SAFETY_SET_ALLOWED_AREA pi54 = (SAFETY_SET_ALLOWED_AREA) pack;
                        ph.setPack(pi54);
                        on_SAFETY_SET_ALLOWED_AREA.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi54));
                        if(LOOP) break;
                        return null;
                    case 55:
                        if(pack == null) return new SAFETY_ALLOWED_AREA();
                        final SAFETY_ALLOWED_AREA pi55 = (SAFETY_ALLOWED_AREA) pack;
                        ph.setPack(pi55);
                        on_SAFETY_ALLOWED_AREA.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi55));
                        if(LOOP) break;
                        return null;
                    case 61:
                        if(pack == null) return new ATTITUDE_QUATERNION_COV();
                        final ATTITUDE_QUATERNION_COV pi61 = (ATTITUDE_QUATERNION_COV) pack;
                        ph.setPack(pi61);
                        on_ATTITUDE_QUATERNION_COV.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi61));
                        if(LOOP) break;
                        return null;
                    case 62:
                        if(pack == null) return new NAV_CONTROLLER_OUTPUT();
                        final NAV_CONTROLLER_OUTPUT pi62 = (NAV_CONTROLLER_OUTPUT) pack;
                        ph.setPack(pi62);
                        on_NAV_CONTROLLER_OUTPUT.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi62));
                        if(LOOP) break;
                        return null;
                    case 63:
                        if(pack == null) return new GLOBAL_POSITION_INT_COV();
                        final GLOBAL_POSITION_INT_COV pi63 = (GLOBAL_POSITION_INT_COV) pack;
                        ph.setPack(pi63);
                        on_GLOBAL_POSITION_INT_COV.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi63));
                        if(LOOP) break;
                        return null;
                    case 64:
                        if(pack == null) return new LOCAL_POSITION_NED_COV();
                        final LOCAL_POSITION_NED_COV pi64 = (LOCAL_POSITION_NED_COV) pack;
                        ph.setPack(pi64);
                        on_LOCAL_POSITION_NED_COV.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi64));
                        if(LOOP) break;
                        return null;
                    case 65:
                        if(pack == null) return new RC_CHANNELS();
                        final RC_CHANNELS pi65 = (RC_CHANNELS) pack;
                        ph.setPack(pi65);
                        on_RC_CHANNELS.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi65));
                        if(LOOP) break;
                        return null;
                    case 66:
                        if(pack == null) return new REQUEST_DATA_STREAM();
                        final REQUEST_DATA_STREAM pi66 = (REQUEST_DATA_STREAM) pack;
                        ph.setPack(pi66);
                        on_REQUEST_DATA_STREAM.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi66));
                        if(LOOP) break;
                        return null;
                    case 67:
                        if(pack == null) return new DATA_STREAM();
                        final DATA_STREAM pi67 = (DATA_STREAM) pack;
                        ph.setPack(pi67);
                        on_DATA_STREAM.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi67));
                        if(LOOP) break;
                        return null;
                    case 69:
                        if(pack == null) return new MANUAL_CONTROL();
                        final MANUAL_CONTROL pi69 = (MANUAL_CONTROL) pack;
                        ph.setPack(pi69);
                        on_MANUAL_CONTROL.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi69));
                        if(LOOP) break;
                        return null;
                    case 70:
                        if(pack == null) return new RC_CHANNELS_OVERRIDE();
                        final RC_CHANNELS_OVERRIDE pi70 = (RC_CHANNELS_OVERRIDE) pack;
                        ph.setPack(pi70);
                        on_RC_CHANNELS_OVERRIDE.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi70));
                        if(LOOP) break;
                        return null;
                    case 73:
                        if(pack == null) return new MISSION_ITEM_INT();
                        final MISSION_ITEM_INT pi73 = (MISSION_ITEM_INT) pack;
                        ph.setPack(pi73);
                        on_MISSION_ITEM_INT.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi73));
                        if(LOOP) break;
                        return null;
                    case 74:
                        if(pack == null) return new VFR_HUD();
                        final VFR_HUD pi74 = (VFR_HUD) pack;
                        ph.setPack(pi74);
                        on_VFR_HUD.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi74));
                        if(LOOP) break;
                        return null;
                    case 75:
                        if(pack == null) return new COMMAND_INT();
                        final COMMAND_INT pi75 = (COMMAND_INT) pack;
                        ph.setPack(pi75);
                        on_COMMAND_INT.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi75));
                        if(LOOP) break;
                        return null;
                    case 76:
                        if(pack == null) return new COMMAND_LONG();
                        final COMMAND_LONG pi76 = (COMMAND_LONG) pack;
                        ph.setPack(pi76);
                        on_COMMAND_LONG.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi76));
                        if(LOOP) break;
                        return null;
                    case 77:
                        if(pack == null) return new COMMAND_ACK(-1);
                        final COMMAND_ACK pi77 = (COMMAND_ACK) pack;
                        ph.setPack(pi77);
                        on_COMMAND_ACK.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi77));
                        if(LOOP) break;
                        return null;
                    case 81:
                        if(pack == null) return new MANUAL_SETPOINT();
                        final MANUAL_SETPOINT pi81 = (MANUAL_SETPOINT) pack;
                        ph.setPack(pi81);
                        on_MANUAL_SETPOINT.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi81));
                        if(LOOP) break;
                        return null;
                    case 82:
                        if(pack == null) return new SET_ATTITUDE_TARGET();
                        final SET_ATTITUDE_TARGET pi82 = (SET_ATTITUDE_TARGET) pack;
                        ph.setPack(pi82);
                        on_SET_ATTITUDE_TARGET.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi82));
                        if(LOOP) break;
                        return null;
                    case 83:
                        if(pack == null) return new ATTITUDE_TARGET();
                        final ATTITUDE_TARGET pi83 = (ATTITUDE_TARGET) pack;
                        ph.setPack(pi83);
                        on_ATTITUDE_TARGET.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi83));
                        if(LOOP) break;
                        return null;
                    case 84:
                        if(pack == null) return new SET_POSITION_TARGET_LOCAL_NED();
                        final SET_POSITION_TARGET_LOCAL_NED pi84 = (SET_POSITION_TARGET_LOCAL_NED) pack;
                        ph.setPack(pi84);
                        on_SET_POSITION_TARGET_LOCAL_NED.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi84));
                        if(LOOP) break;
                        return null;
                    case 86:
                        if(pack == null) return new SET_POSITION_TARGET_GLOBAL_INT();
                        final SET_POSITION_TARGET_GLOBAL_INT pi86 = (SET_POSITION_TARGET_GLOBAL_INT) pack;
                        ph.setPack(pi86);
                        on_SET_POSITION_TARGET_GLOBAL_INT.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi86));
                        if(LOOP) break;
                        return null;
                    case 87:
                        if(pack == null) return new POSITION_TARGET_GLOBAL_INT();
                        final POSITION_TARGET_GLOBAL_INT pi87 = (POSITION_TARGET_GLOBAL_INT) pack;
                        ph.setPack(pi87);
                        on_POSITION_TARGET_GLOBAL_INT.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi87));
                        if(LOOP) break;
                        return null;
                    case 89:
                        if(pack == null) return new LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
                        final LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET pi89 = (LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET) pack;
                        ph.setPack(pi89);
                        on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi89));
                        if(LOOP) break;
                        return null;
                    case 90:
                        if(pack == null) return new HIL_STATE();
                        final HIL_STATE pi90 = (HIL_STATE) pack;
                        ph.setPack(pi90);
                        on_HIL_STATE.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi90));
                        if(LOOP) break;
                        return null;
                    case 91:
                        if(pack == null) return new HIL_CONTROLS();
                        final HIL_CONTROLS pi91 = (HIL_CONTROLS) pack;
                        ph.setPack(pi91);
                        on_HIL_CONTROLS.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi91));
                        if(LOOP) break;
                        return null;
                    case 92:
                        if(pack == null) return new HIL_RC_INPUTS_RAW();
                        final HIL_RC_INPUTS_RAW pi92 = (HIL_RC_INPUTS_RAW) pack;
                        ph.setPack(pi92);
                        on_HIL_RC_INPUTS_RAW.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi92));
                        if(LOOP) break;
                        return null;
                    case 93:
                        if(pack == null) return new HIL_ACTUATOR_CONTROLS();
                        final HIL_ACTUATOR_CONTROLS pi93 = (HIL_ACTUATOR_CONTROLS) pack;
                        ph.setPack(pi93);
                        on_HIL_ACTUATOR_CONTROLS.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi93));
                        if(LOOP) break;
                        return null;
                    case 100:
                        if(pack == null) return new OPTICAL_FLOW(-1);
                        final OPTICAL_FLOW pi100 = (OPTICAL_FLOW) pack;
                        ph.setPack(pi100);
                        on_OPTICAL_FLOW.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi100));
                        if(LOOP) break;
                        return null;
                    case 101:
                        if(pack == null) return new GLOBAL_VISION_POSITION_ESTIMATE();
                        final GLOBAL_VISION_POSITION_ESTIMATE pi101 = (GLOBAL_VISION_POSITION_ESTIMATE) pack;
                        ph.setPack(pi101);
                        on_GLOBAL_VISION_POSITION_ESTIMATE.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi101));
                        if(LOOP) break;
                        return null;
                    case 102:
                        if(pack == null) return new VISION_POSITION_ESTIMATE();
                        final VISION_POSITION_ESTIMATE pi102 = (VISION_POSITION_ESTIMATE) pack;
                        ph.setPack(pi102);
                        on_VISION_POSITION_ESTIMATE.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi102));
                        if(LOOP) break;
                        return null;
                    case 103:
                        if(pack == null) return new VISION_SPEED_ESTIMATE();
                        final VISION_SPEED_ESTIMATE pi103 = (VISION_SPEED_ESTIMATE) pack;
                        ph.setPack(pi103);
                        on_VISION_SPEED_ESTIMATE.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi103));
                        if(LOOP) break;
                        return null;
                    case 104:
                        if(pack == null) return new VICON_POSITION_ESTIMATE();
                        final VICON_POSITION_ESTIMATE pi104 = (VICON_POSITION_ESTIMATE) pack;
                        ph.setPack(pi104);
                        on_VICON_POSITION_ESTIMATE.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi104));
                        if(LOOP) break;
                        return null;
                    case 105:
                        if(pack == null) return new HIGHRES_IMU();
                        final HIGHRES_IMU pi105 = (HIGHRES_IMU) pack;
                        ph.setPack(pi105);
                        on_HIGHRES_IMU.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi105));
                        if(LOOP) break;
                        return null;
                    case 106:
                        if(pack == null) return new OPTICAL_FLOW_RAD();
                        final OPTICAL_FLOW_RAD pi106 = (OPTICAL_FLOW_RAD) pack;
                        ph.setPack(pi106);
                        on_OPTICAL_FLOW_RAD.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi106));
                        if(LOOP) break;
                        return null;
                    case 107:
                        if(pack == null) return new HIL_SENSOR();
                        final HIL_SENSOR pi107 = (HIL_SENSOR) pack;
                        ph.setPack(pi107);
                        on_HIL_SENSOR.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi107));
                        if(LOOP) break;
                        return null;
                    case 108:
                        if(pack == null) return new SIM_STATE();
                        final SIM_STATE pi108 = (SIM_STATE) pack;
                        ph.setPack(pi108);
                        on_SIM_STATE.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi108));
                        if(LOOP) break;
                        return null;
                    case 109:
                        if(pack == null) return new RADIO_STATUS();
                        final RADIO_STATUS pi109 = (RADIO_STATUS) pack;
                        ph.setPack(pi109);
                        on_RADIO_STATUS.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi109));
                        if(LOOP) break;
                        return null;
                    case 110:
                        if(pack == null) return new FILE_TRANSFER_PROTOCOL();
                        final FILE_TRANSFER_PROTOCOL pi110 = (FILE_TRANSFER_PROTOCOL) pack;
                        ph.setPack(pi110);
                        on_FILE_TRANSFER_PROTOCOL.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi110));
                        if(LOOP) break;
                        return null;
                    case 111:
                        if(pack == null) return new TIMESYNC();
                        final TIMESYNC pi111 = (TIMESYNC) pack;
                        ph.setPack(pi111);
                        on_TIMESYNC.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi111));
                        if(LOOP) break;
                        return null;
                    case 112:
                        if(pack == null) return new CAMERA_TRIGGER();
                        final CAMERA_TRIGGER pi112 = (CAMERA_TRIGGER) pack;
                        ph.setPack(pi112);
                        on_CAMERA_TRIGGER.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi112));
                        if(LOOP) break;
                        return null;
                    case 113:
                        if(pack == null) return new HIL_GPS();
                        final HIL_GPS pi113 = (HIL_GPS) pack;
                        ph.setPack(pi113);
                        on_HIL_GPS.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi113));
                        if(LOOP) break;
                        return null;
                    case 114:
                        if(pack == null) return new HIL_OPTICAL_FLOW();
                        final HIL_OPTICAL_FLOW pi114 = (HIL_OPTICAL_FLOW) pack;
                        ph.setPack(pi114);
                        on_HIL_OPTICAL_FLOW.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi114));
                        if(LOOP) break;
                        return null;
                    case 115:
                        if(pack == null) return new HIL_STATE_QUATERNION();
                        final HIL_STATE_QUATERNION pi115 = (HIL_STATE_QUATERNION) pack;
                        ph.setPack(pi115);
                        on_HIL_STATE_QUATERNION.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi115));
                        if(LOOP) break;
                        return null;
                    case 116:
                        if(pack == null) return new SCALED_IMU2();
                        final SCALED_IMU2 pi116 = (SCALED_IMU2) pack;
                        ph.setPack(pi116);
                        on_SCALED_IMU2.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi116));
                        if(LOOP) break;
                        return null;
                    case 117:
                        if(pack == null) return new LOG_REQUEST_LIST();
                        final LOG_REQUEST_LIST pi117 = (LOG_REQUEST_LIST) pack;
                        ph.setPack(pi117);
                        on_LOG_REQUEST_LIST.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi117));
                        if(LOOP) break;
                        return null;
                    case 118:
                        if(pack == null) return new LOG_ENTRY();
                        final LOG_ENTRY pi118 = (LOG_ENTRY) pack;
                        ph.setPack(pi118);
                        on_LOG_ENTRY.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi118));
                        if(LOOP) break;
                        return null;
                    case 119:
                        if(pack == null) return new LOG_REQUEST_DATA();
                        final LOG_REQUEST_DATA pi119 = (LOG_REQUEST_DATA) pack;
                        ph.setPack(pi119);
                        on_LOG_REQUEST_DATA.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi119));
                        if(LOOP) break;
                        return null;
                    case 120:
                        if(pack == null) return new LOG_DATA();
                        final LOG_DATA pi120 = (LOG_DATA) pack;
                        ph.setPack(pi120);
                        on_LOG_DATA.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi120));
                        if(LOOP) break;
                        return null;
                    case 121:
                        if(pack == null) return new LOG_ERASE();
                        final LOG_ERASE pi121 = (LOG_ERASE) pack;
                        ph.setPack(pi121);
                        on_LOG_ERASE.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi121));
                        if(LOOP) break;
                        return null;
                    case 122:
                        if(pack == null) return new LOG_REQUEST_END();
                        final LOG_REQUEST_END pi122 = (LOG_REQUEST_END) pack;
                        ph.setPack(pi122);
                        on_LOG_REQUEST_END.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi122));
                        if(LOOP) break;
                        return null;
                    case 123:
                        if(pack == null) return new GPS_INJECT_DATA();
                        final GPS_INJECT_DATA pi123 = (GPS_INJECT_DATA) pack;
                        ph.setPack(pi123);
                        on_GPS_INJECT_DATA.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi123));
                        if(LOOP) break;
                        return null;
                    case 124:
                        if(pack == null) return new GPS2_RAW();
                        final GPS2_RAW pi124 = (GPS2_RAW) pack;
                        ph.setPack(pi124);
                        on_GPS2_RAW.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi124));
                        if(LOOP) break;
                        return null;
                    case 125:
                        if(pack == null) return new POWER_STATUS();
                        final POWER_STATUS pi125 = (POWER_STATUS) pack;
                        ph.setPack(pi125);
                        on_POWER_STATUS.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi125));
                        if(LOOP) break;
                        return null;
                    case 126:
                        if(pack == null) return new SERIAL_CONTROL();
                        final SERIAL_CONTROL pi126 = (SERIAL_CONTROL) pack;
                        ph.setPack(pi126);
                        on_SERIAL_CONTROL.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi126));
                        if(LOOP) break;
                        return null;
                    case 127:
                        if(pack == null) return new GPS_RTK();
                        final GPS_RTK pi127 = (GPS_RTK) pack;
                        ph.setPack(pi127);
                        on_GPS_RTK.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi127));
                        if(LOOP) break;
                        return null;
                    case 128:
                        if(pack == null) return new GPS2_RTK();
                        final GPS2_RTK pi128 = (GPS2_RTK) pack;
                        ph.setPack(pi128);
                        on_GPS2_RTK.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi128));
                        if(LOOP) break;
                        return null;
                    case 129:
                        if(pack == null) return new SCALED_IMU3();
                        final SCALED_IMU3 pi129 = (SCALED_IMU3) pack;
                        ph.setPack(pi129);
                        on_SCALED_IMU3.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi129));
                        if(LOOP) break;
                        return null;
                    case 130:
                        if(pack == null) return new DATA_TRANSMISSION_HANDSHAKE();
                        final DATA_TRANSMISSION_HANDSHAKE pi130 = (DATA_TRANSMISSION_HANDSHAKE) pack;
                        ph.setPack(pi130);
                        on_DATA_TRANSMISSION_HANDSHAKE.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi130));
                        if(LOOP) break;
                        return null;
                    case 131:
                        if(pack == null) return new ENCAPSULATED_DATA();
                        final ENCAPSULATED_DATA pi131 = (ENCAPSULATED_DATA) pack;
                        ph.setPack(pi131);
                        on_ENCAPSULATED_DATA.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi131));
                        if(LOOP) break;
                        return null;
                    case 132:
                        if(pack == null) return new DISTANCE_SENSOR();
                        final DISTANCE_SENSOR pi132 = (DISTANCE_SENSOR) pack;
                        ph.setPack(pi132);
                        on_DISTANCE_SENSOR.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi132));
                        if(LOOP) break;
                        return null;
                    case 133:
                        if(pack == null) return new TERRAIN_REQUEST();
                        final TERRAIN_REQUEST pi133 = (TERRAIN_REQUEST) pack;
                        ph.setPack(pi133);
                        on_TERRAIN_REQUEST.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi133));
                        if(LOOP) break;
                        return null;
                    case 134:
                        if(pack == null) return new TERRAIN_DATA();
                        final TERRAIN_DATA pi134 = (TERRAIN_DATA) pack;
                        ph.setPack(pi134);
                        on_TERRAIN_DATA.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi134));
                        if(LOOP) break;
                        return null;
                    case 135:
                        if(pack == null) return new TERRAIN_CHECK();
                        final TERRAIN_CHECK pi135 = (TERRAIN_CHECK) pack;
                        ph.setPack(pi135);
                        on_TERRAIN_CHECK.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi135));
                        if(LOOP) break;
                        return null;
                    case 136:
                        if(pack == null) return new TERRAIN_REPORT();
                        final TERRAIN_REPORT pi136 = (TERRAIN_REPORT) pack;
                        ph.setPack(pi136);
                        on_TERRAIN_REPORT.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi136));
                        if(LOOP) break;
                        return null;
                    case 137:
                        if(pack == null) return new SCALED_PRESSURE2();
                        final SCALED_PRESSURE2 pi137 = (SCALED_PRESSURE2) pack;
                        ph.setPack(pi137);
                        on_SCALED_PRESSURE2.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi137));
                        if(LOOP) break;
                        return null;
                    case 138:
                        if(pack == null) return new ATT_POS_MOCAP();
                        final ATT_POS_MOCAP pi138 = (ATT_POS_MOCAP) pack;
                        ph.setPack(pi138);
                        on_ATT_POS_MOCAP.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi138));
                        if(LOOP) break;
                        return null;
                    case 139:
                        if(pack == null) return new SET_ACTUATOR_CONTROL_TARGET();
                        final SET_ACTUATOR_CONTROL_TARGET pi139 = (SET_ACTUATOR_CONTROL_TARGET) pack;
                        ph.setPack(pi139);
                        on_SET_ACTUATOR_CONTROL_TARGET.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi139));
                        if(LOOP) break;
                        return null;
                    case 140:
                        if(pack == null) return new ACTUATOR_CONTROL_TARGET();
                        final ACTUATOR_CONTROL_TARGET pi140 = (ACTUATOR_CONTROL_TARGET) pack;
                        ph.setPack(pi140);
                        on_ACTUATOR_CONTROL_TARGET.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi140));
                        if(LOOP) break;
                        return null;
                    case 141:
                        if(pack == null) return new ALTITUDE();
                        final ALTITUDE pi141 = (ALTITUDE) pack;
                        ph.setPack(pi141);
                        on_ALTITUDE.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi141));
                        if(LOOP) break;
                        return null;
                    case 142:
                        if(pack == null) return new RESOURCE_REQUEST();
                        final RESOURCE_REQUEST pi142 = (RESOURCE_REQUEST) pack;
                        ph.setPack(pi142);
                        on_RESOURCE_REQUEST.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi142));
                        if(LOOP) break;
                        return null;
                    case 143:
                        if(pack == null) return new SCALED_PRESSURE3();
                        final SCALED_PRESSURE3 pi143 = (SCALED_PRESSURE3) pack;
                        ph.setPack(pi143);
                        on_SCALED_PRESSURE3.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi143));
                        if(LOOP) break;
                        return null;
                    case 144:
                        if(pack == null) return new FOLLOW_TARGET();
                        final FOLLOW_TARGET pi144 = (FOLLOW_TARGET) pack;
                        ph.setPack(pi144);
                        on_FOLLOW_TARGET.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi144));
                        if(LOOP) break;
                        return null;
                    case 146:
                        if(pack == null) return new CONTROL_SYSTEM_STATE();
                        final CONTROL_SYSTEM_STATE pi146 = (CONTROL_SYSTEM_STATE) pack;
                        ph.setPack(pi146);
                        on_CONTROL_SYSTEM_STATE.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi146));
                        if(LOOP) break;
                        return null;
                    case 147:
                        if(pack == null) return new BATTERY_STATUS();
                        final BATTERY_STATUS pi147 = (BATTERY_STATUS) pack;
                        ph.setPack(pi147);
                        on_BATTERY_STATUS.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi147));
                        if(LOOP) break;
                        return null;
                    case 148:
                        if(pack == null) return new AUTOPILOT_VERSION(-1);
                        final AUTOPILOT_VERSION pi148 = (AUTOPILOT_VERSION) pack;
                        ph.setPack(pi148);
                        on_AUTOPILOT_VERSION.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi148));
                        if(LOOP) break;
                        return null;
                    case 149:
                        if(pack == null) return new LANDING_TARGET(-1);
                        final LANDING_TARGET pi149 = (LANDING_TARGET) pack;
                        ph.setPack(pi149);
                        on_LANDING_TARGET.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi149));
                        if(LOOP) break;
                        return null;
                    case 220:
                        if(pack == null) return new NAV_FILTER_BIAS();
                        final NAV_FILTER_BIAS pi220 = (NAV_FILTER_BIAS) pack;
                        ph.setPack(pi220);
                        on_NAV_FILTER_BIAS.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi220));
                        if(LOOP) break;
                        return null;
                    case 221:
                        if(pack == null) return new RADIO_CALIBRATION();
                        final RADIO_CALIBRATION pi221 = (RADIO_CALIBRATION) pack;
                        ph.setPack(pi221);
                        on_RADIO_CALIBRATION.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi221));
                        if(LOOP) break;
                        return null;
                    case 222:
                        if(pack == null) return new UALBERTA_SYS_STATUS();
                        final UALBERTA_SYS_STATUS pi222 = (UALBERTA_SYS_STATUS) pack;
                        ph.setPack(pi222);
                        on_UALBERTA_SYS_STATUS.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi222));
                        if(LOOP) break;
                        return null;
                    case 230:
                        if(pack == null) return new ESTIMATOR_STATUS();
                        final ESTIMATOR_STATUS pi230 = (ESTIMATOR_STATUS) pack;
                        ph.setPack(pi230);
                        on_ESTIMATOR_STATUS.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi230));
                        if(LOOP) break;
                        return null;
                    case 231:
                        if(pack == null) return new WIND_COV();
                        final WIND_COV pi231 = (WIND_COV) pack;
                        ph.setPack(pi231);
                        on_WIND_COV.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi231));
                        if(LOOP) break;
                        return null;
                    case 232:
                        if(pack == null) return new GPS_INPUT();
                        final GPS_INPUT pi232 = (GPS_INPUT) pack;
                        ph.setPack(pi232);
                        on_GPS_INPUT.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi232));
                        if(LOOP) break;
                        return null;
                    case 233:
                        if(pack == null) return new GPS_RTCM_DATA();
                        final GPS_RTCM_DATA pi233 = (GPS_RTCM_DATA) pack;
                        ph.setPack(pi233);
                        on_GPS_RTCM_DATA.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi233));
                        if(LOOP) break;
                        return null;
                    case 234:
                        if(pack == null) return new HIGH_LATENCY();
                        final HIGH_LATENCY pi234 = (HIGH_LATENCY) pack;
                        ph.setPack(pi234);
                        on_HIGH_LATENCY.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi234));
                        if(LOOP) break;
                        return null;
                    case 241:
                        if(pack == null) return new VIBRATION();
                        final VIBRATION pi241 = (VIBRATION) pack;
                        ph.setPack(pi241);
                        on_VIBRATION.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi241));
                        if(LOOP) break;
                        return null;
                    case 242:
                        if(pack == null) return new HOME_POSITION(-1);
                        final HOME_POSITION pi242 = (HOME_POSITION) pack;
                        ph.setPack(pi242);
                        on_HOME_POSITION.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi242));
                        if(LOOP) break;
                        return null;
                    case 243:
                        if(pack == null) return new SET_HOME_POSITION(-1);
                        final SET_HOME_POSITION pi243 = (SET_HOME_POSITION) pack;
                        ph.setPack(pi243);
                        on_SET_HOME_POSITION.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi243));
                        if(LOOP) break;
                        return null;
                    case 244:
                        if(pack == null) return new MESSAGE_INTERVAL();
                        final MESSAGE_INTERVAL pi244 = (MESSAGE_INTERVAL) pack;
                        ph.setPack(pi244);
                        on_MESSAGE_INTERVAL.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi244));
                        if(LOOP) break;
                        return null;
                    case 245:
                        if(pack == null) return new EXTENDED_SYS_STATE();
                        final EXTENDED_SYS_STATE pi245 = (EXTENDED_SYS_STATE) pack;
                        ph.setPack(pi245);
                        on_EXTENDED_SYS_STATE.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi245));
                        if(LOOP) break;
                        return null;
                    case 246:
                        if(pack == null) return new ADSB_VEHICLE(-1);
                        final ADSB_VEHICLE pi246 = (ADSB_VEHICLE) pack;
                        ph.setPack(pi246);
                        on_ADSB_VEHICLE.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi246));
                        if(LOOP) break;
                        return null;
                    case 247:
                        if(pack == null) return new COLLISION();
                        final COLLISION pi247 = (COLLISION) pack;
                        ph.setPack(pi247);
                        on_COLLISION.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi247));
                        if(LOOP) break;
                        return null;
                    case 248:
                        if(pack == null) return new V2_EXTENSION();
                        final V2_EXTENSION pi248 = (V2_EXTENSION) pack;
                        ph.setPack(pi248);
                        on_V2_EXTENSION.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi248));
                        if(LOOP) break;
                        return null;
                    case 249:
                        if(pack == null) return new MEMORY_VECT();
                        final MEMORY_VECT pi249 = (MEMORY_VECT) pack;
                        ph.setPack(pi249);
                        on_MEMORY_VECT.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi249));
                        if(LOOP) break;
                        return null;
                    case 250:
                        if(pack == null) return new DEBUG_VECT(-1);
                        final DEBUG_VECT pi250 = (DEBUG_VECT) pack;
                        ph.setPack(pi250);
                        on_DEBUG_VECT.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi250));
                        if(LOOP) break;
                        return null;
                    case 251:
                        if(pack == null) return new NAMED_VALUE_FLOAT(-1);
                        final NAMED_VALUE_FLOAT pi251 = (NAMED_VALUE_FLOAT) pack;
                        ph.setPack(pi251);
                        on_NAMED_VALUE_FLOAT.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi251));
                        if(LOOP) break;
                        return null;
                    case 252:
                        if(pack == null) return new NAMED_VALUE_INT(-1);
                        final NAMED_VALUE_INT pi252 = (NAMED_VALUE_INT) pack;
                        ph.setPack(pi252);
                        on_NAMED_VALUE_INT.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi252));
                        if(LOOP) break;
                        return null;
                    case 253:
                        if(pack == null) return new STATUSTEXT(-1);
                        final STATUSTEXT pi253 = (STATUSTEXT) pack;
                        ph.setPack(pi253);
                        on_STATUSTEXT.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi253));
                        if(LOOP) break;
                        return null;
                    case 254:
                        if(pack == null) return new DEBUG();
                        final DEBUG pi254 = (DEBUG) pack;
                        ph.setPack(pi254);
                        on_DEBUG.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi254));
                        if(LOOP) break;
                        return null;
                    case 256:
                        if(pack == null) return new SETUP_SIGNING();
                        final SETUP_SIGNING pi256 = (SETUP_SIGNING) pack;
                        ph.setPack(pi256);
                        on_SETUP_SIGNING.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi256));
                        if(LOOP) break;
                        return null;
                    case 257:
                        if(pack == null) return new BUTTON_CHANGE();
                        final BUTTON_CHANGE pi257 = (BUTTON_CHANGE) pack;
                        ph.setPack(pi257);
                        on_BUTTON_CHANGE.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi257));
                        if(LOOP) break;
                        return null;
                    case 258:
                        if(pack == null) return new PLAY_TUNE(-1);
                        final PLAY_TUNE pi258 = (PLAY_TUNE) pack;
                        ph.setPack(pi258);
                        on_PLAY_TUNE.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi258));
                        if(LOOP) break;
                        return null;
                    case 259:
                        if(pack == null) return new CAMERA_INFORMATION(-1);
                        final CAMERA_INFORMATION pi259 = (CAMERA_INFORMATION) pack;
                        ph.setPack(pi259);
                        on_CAMERA_INFORMATION.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi259));
                        if(LOOP) break;
                        return null;
                    case 260:
                        if(pack == null) return new CAMERA_SETTINGS();
                        final CAMERA_SETTINGS pi260 = (CAMERA_SETTINGS) pack;
                        ph.setPack(pi260);
                        on_CAMERA_SETTINGS.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi260));
                        if(LOOP) break;
                        return null;
                    case 261:
                        if(pack == null) return new STORAGE_INFORMATION();
                        final STORAGE_INFORMATION pi261 = (STORAGE_INFORMATION) pack;
                        ph.setPack(pi261);
                        on_STORAGE_INFORMATION.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi261));
                        if(LOOP) break;
                        return null;
                    case 262:
                        if(pack == null) return new CAMERA_CAPTURE_STATUS();
                        final CAMERA_CAPTURE_STATUS pi262 = (CAMERA_CAPTURE_STATUS) pack;
                        ph.setPack(pi262);
                        on_CAMERA_CAPTURE_STATUS.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi262));
                        if(LOOP) break;
                        return null;
                    case 263:
                        if(pack == null) return new CAMERA_IMAGE_CAPTURED(-1);
                        final CAMERA_IMAGE_CAPTURED pi263 = (CAMERA_IMAGE_CAPTURED) pack;
                        ph.setPack(pi263);
                        on_CAMERA_IMAGE_CAPTURED.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi263));
                        if(LOOP) break;
                        return null;
                    case 264:
                        if(pack == null) return new FLIGHT_INFORMATION();
                        final FLIGHT_INFORMATION pi264 = (FLIGHT_INFORMATION) pack;
                        ph.setPack(pi264);
                        on_FLIGHT_INFORMATION.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi264));
                        if(LOOP) break;
                        return null;
                    case 265:
                        if(pack == null) return new MOUNT_ORIENTATION();
                        final MOUNT_ORIENTATION pi265 = (MOUNT_ORIENTATION) pack;
                        ph.setPack(pi265);
                        on_MOUNT_ORIENTATION.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi265));
                        if(LOOP) break;
                        return null;
                    case 266:
                        if(pack == null) return new LOGGING_DATA();
                        final LOGGING_DATA pi266 = (LOGGING_DATA) pack;
                        ph.setPack(pi266);
                        on_LOGGING_DATA.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi266));
                        if(LOOP) break;
                        return null;
                    case 267:
                        if(pack == null) return new LOGGING_DATA_ACKED();
                        final LOGGING_DATA_ACKED pi267 = (LOGGING_DATA_ACKED) pack;
                        ph.setPack(pi267);
                        on_LOGGING_DATA_ACKED.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi267));
                        if(LOOP) break;
                        return null;
                    case 268:
                        if(pack == null) return new LOGGING_ACK();
                        final LOGGING_ACK pi268 = (LOGGING_ACK) pack;
                        ph.setPack(pi268);
                        on_LOGGING_ACK.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi268));
                        if(LOOP) break;
                        return null;
                    case 269:
                        if(pack == null) return new VIDEO_STREAM_INFORMATION(-1);
                        final VIDEO_STREAM_INFORMATION pi269 = (VIDEO_STREAM_INFORMATION) pack;
                        ph.setPack(pi269);
                        on_VIDEO_STREAM_INFORMATION.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi269));
                        if(LOOP) break;
                        return null;
                    case 270:
                        if(pack == null) return new SET_VIDEO_STREAM_SETTINGS(-1);
                        final SET_VIDEO_STREAM_SETTINGS pi270 = (SET_VIDEO_STREAM_SETTINGS) pack;
                        ph.setPack(pi270);
                        on_SET_VIDEO_STREAM_SETTINGS.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi270));
                        if(LOOP) break;
                        return null;
                    case 299:
                        if(pack == null) return new WIFI_CONFIG_AP(-1);
                        final WIFI_CONFIG_AP pi299 = (WIFI_CONFIG_AP) pack;
                        ph.setPack(pi299);
                        on_WIFI_CONFIG_AP.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi299));
                        if(LOOP) break;
                        return null;
                    case 300:
                        if(pack == null) return new PROTOCOL_VERSION();
                        final PROTOCOL_VERSION pi300 = (PROTOCOL_VERSION) pack;
                        ph.setPack(pi300);
                        on_PROTOCOL_VERSION.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi300));
                        if(LOOP) break;
                        return null;
                    case 310:
                        if(pack == null) return new UAVCAN_NODE_STATUS();
                        final UAVCAN_NODE_STATUS pi310 = (UAVCAN_NODE_STATUS) pack;
                        ph.setPack(pi310);
                        on_UAVCAN_NODE_STATUS.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi310));
                        if(LOOP) break;
                        return null;
                    case 311:
                        if(pack == null) return new UAVCAN_NODE_INFO(-1);
                        final UAVCAN_NODE_INFO pi311 = (UAVCAN_NODE_INFO) pack;
                        ph.setPack(pi311);
                        on_UAVCAN_NODE_INFO.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi311));
                        if(LOOP) break;
                        return null;
                    case 320:
                        if(pack == null) return new PARAM_EXT_REQUEST_READ(-1);
                        final PARAM_EXT_REQUEST_READ pi320 = (PARAM_EXT_REQUEST_READ) pack;
                        ph.setPack(pi320);
                        on_PARAM_EXT_REQUEST_READ.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi320));
                        if(LOOP) break;
                        return null;
                    case 321:
                        if(pack == null) return new PARAM_EXT_REQUEST_LIST();
                        final PARAM_EXT_REQUEST_LIST pi321 = (PARAM_EXT_REQUEST_LIST) pack;
                        ph.setPack(pi321);
                        on_PARAM_EXT_REQUEST_LIST.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi321));
                        if(LOOP) break;
                        return null;
                    case 322:
                        if(pack == null) return new PARAM_EXT_VALUE(-1);
                        final PARAM_EXT_VALUE pi322 = (PARAM_EXT_VALUE) pack;
                        ph.setPack(pi322);
                        on_PARAM_EXT_VALUE.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi322));
                        if(LOOP) break;
                        return null;
                    case 323:
                        if(pack == null) return new PARAM_EXT_SET(-1);
                        final PARAM_EXT_SET pi323 = (PARAM_EXT_SET) pack;
                        ph.setPack(pi323);
                        on_PARAM_EXT_SET.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi323));
                        if(LOOP) break;
                        return null;
                    case 324:
                        if(pack == null) return new PARAM_EXT_ACK(-1);
                        final PARAM_EXT_ACK pi324 = (PARAM_EXT_ACK) pack;
                        ph.setPack(pi324);
                        on_PARAM_EXT_ACK.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi324));
                        if(LOOP) break;
                        return null;
                    case 330:
                        if(pack == null) return new OBSTACLE_DISTANCE();
                        final OBSTACLE_DISTANCE pi330 = (OBSTACLE_DISTANCE) pack;
                        ph.setPack(pi330);
                        on_OBSTACLE_DISTANCE.forEach(h -> h.handle(LoopBackDemoChannel.this, ph, pi330));
                        if(LOOP) break;
                        return null;
                    case Channel.PROCESS_CHANNEL_REQEST:
                        if(pack == null) return sendout_packs.poll();
                        received_packs.add(pack);
                        return null;
                    case Channel.PROCESS_HOST_REQEST:
                        if(pack == null) return received_packs.poll();
                        sendout_packs.add(pack);
                        return null;
                    case Channel.PROCESS_RECEIVED:
                        LOOP = true;
                }
                if(received_packs.isEmpty()) return null;
                pack = received_packs.remove();
                id = pack.meta.id;
            }
        }
        public final Collection<OnReceive.Handler<HEARTBEAT, LoopBackDemoChannel>> on_HEARTBEAT = new OnReceive<>();
        public final Collection<OnReceive.Handler<SYS_STATUS, LoopBackDemoChannel>> on_SYS_STATUS = new OnReceive<>();
        public final Collection<OnReceive.Handler<SYSTEM_TIME, LoopBackDemoChannel>> on_SYSTEM_TIME = new OnReceive<>();
        public final Collection<OnReceive.Handler<POSITION_TARGET_LOCAL_NED, LoopBackDemoChannel>> on_POSITION_TARGET_LOCAL_NED = new OnReceive<>();
        public final Collection<OnReceive.Handler<PING, LoopBackDemoChannel>> on_PING = new OnReceive<>();
        public final Collection<OnReceive.Handler<CHANGE_OPERATOR_CONTROL, LoopBackDemoChannel>> on_CHANGE_OPERATOR_CONTROL = new OnReceive<>();
        public final Collection<OnReceive.Handler<CHANGE_OPERATOR_CONTROL_ACK, LoopBackDemoChannel>> on_CHANGE_OPERATOR_CONTROL_ACK = new OnReceive<>();
        public final Collection<OnReceive.Handler<AUTH_KEY, LoopBackDemoChannel>> on_AUTH_KEY = new OnReceive<>();
        public final Collection<OnReceive.Handler<SET_MODE, LoopBackDemoChannel>> on_SET_MODE = new OnReceive<>();
        public final Collection<OnReceive.Handler<PARAM_REQUEST_READ, LoopBackDemoChannel>> on_PARAM_REQUEST_READ = new OnReceive<>();
        public final Collection<OnReceive.Handler<PARAM_REQUEST_LIST, LoopBackDemoChannel>> on_PARAM_REQUEST_LIST = new OnReceive<>();
        public final Collection<OnReceive.Handler<PARAM_VALUE, LoopBackDemoChannel>> on_PARAM_VALUE = new OnReceive<>();
        public final Collection<OnReceive.Handler<PARAM_SET, LoopBackDemoChannel>> on_PARAM_SET = new OnReceive<>();
        public final Collection<OnReceive.Handler<GPS_RAW_INT, LoopBackDemoChannel>> on_GPS_RAW_INT = new OnReceive<>();
        public final Collection<OnReceive.Handler<GPS_STATUS, LoopBackDemoChannel>> on_GPS_STATUS = new OnReceive<>();
        public final Collection<OnReceive.Handler<SCALED_IMU, LoopBackDemoChannel>> on_SCALED_IMU = new OnReceive<>();
        public final Collection<OnReceive.Handler<RAW_IMU, LoopBackDemoChannel>> on_RAW_IMU = new OnReceive<>();
        public final Collection<OnReceive.Handler<RAW_PRESSURE, LoopBackDemoChannel>> on_RAW_PRESSURE = new OnReceive<>();
        public final Collection<OnReceive.Handler<SCALED_PRESSURE, LoopBackDemoChannel>> on_SCALED_PRESSURE = new OnReceive<>();
        public final Collection<OnReceive.Handler<ATTITUDE, LoopBackDemoChannel>> on_ATTITUDE = new OnReceive<>();
        public final Collection<OnReceive.Handler<ATTITUDE_QUATERNION, LoopBackDemoChannel>> on_ATTITUDE_QUATERNION = new OnReceive<>();
        public final Collection<OnReceive.Handler<LOCAL_POSITION_NED, LoopBackDemoChannel>> on_LOCAL_POSITION_NED = new OnReceive<>();
        public final Collection<OnReceive.Handler<GLOBAL_POSITION_INT, LoopBackDemoChannel>> on_GLOBAL_POSITION_INT = new OnReceive<>();
        public final Collection<OnReceive.Handler<RC_CHANNELS_SCALED, LoopBackDemoChannel>> on_RC_CHANNELS_SCALED = new OnReceive<>();
        public final Collection<OnReceive.Handler<RC_CHANNELS_RAW, LoopBackDemoChannel>> on_RC_CHANNELS_RAW = new OnReceive<>();
        public final Collection<OnReceive.Handler<SERVO_OUTPUT_RAW, LoopBackDemoChannel>> on_SERVO_OUTPUT_RAW = new OnReceive<>();
        public final Collection<OnReceive.Handler<MISSION_REQUEST_PARTIAL_LIST, LoopBackDemoChannel>> on_MISSION_REQUEST_PARTIAL_LIST = new OnReceive<>();
        public final Collection<OnReceive.Handler<MISSION_WRITE_PARTIAL_LIST, LoopBackDemoChannel>> on_MISSION_WRITE_PARTIAL_LIST = new OnReceive<>();
        public final Collection<OnReceive.Handler<MISSION_ITEM, LoopBackDemoChannel>> on_MISSION_ITEM = new OnReceive<>();
        public final Collection<OnReceive.Handler<MISSION_REQUEST, LoopBackDemoChannel>> on_MISSION_REQUEST = new OnReceive<>();
        public final Collection<OnReceive.Handler<MISSION_SET_CURRENT, LoopBackDemoChannel>> on_MISSION_SET_CURRENT = new OnReceive<>();
        public final Collection<OnReceive.Handler<MISSION_CURRENT, LoopBackDemoChannel>> on_MISSION_CURRENT = new OnReceive<>();
        public final Collection<OnReceive.Handler<MISSION_REQUEST_LIST, LoopBackDemoChannel>> on_MISSION_REQUEST_LIST = new OnReceive<>();
        public final Collection<OnReceive.Handler<MISSION_COUNT, LoopBackDemoChannel>> on_MISSION_COUNT = new OnReceive<>();
        public final Collection<OnReceive.Handler<MISSION_CLEAR_ALL, LoopBackDemoChannel>> on_MISSION_CLEAR_ALL = new OnReceive<>();
        public final Collection<OnReceive.Handler<MISSION_ITEM_REACHED, LoopBackDemoChannel>> on_MISSION_ITEM_REACHED = new OnReceive<>();
        public final Collection<OnReceive.Handler<MISSION_ACK, LoopBackDemoChannel>> on_MISSION_ACK = new OnReceive<>();
        public final Collection<OnReceive.Handler<SET_GPS_GLOBAL_ORIGIN, LoopBackDemoChannel>> on_SET_GPS_GLOBAL_ORIGIN = new OnReceive<>();
        public final Collection<OnReceive.Handler<GPS_GLOBAL_ORIGIN, LoopBackDemoChannel>> on_GPS_GLOBAL_ORIGIN = new OnReceive<>();
        public final Collection<OnReceive.Handler<PARAM_MAP_RC, LoopBackDemoChannel>> on_PARAM_MAP_RC = new OnReceive<>();
        public final Collection<OnReceive.Handler<MISSION_REQUEST_INT, LoopBackDemoChannel>> on_MISSION_REQUEST_INT = new OnReceive<>();
        public final Collection<OnReceive.Handler<SAFETY_SET_ALLOWED_AREA, LoopBackDemoChannel>> on_SAFETY_SET_ALLOWED_AREA = new OnReceive<>();
        public final Collection<OnReceive.Handler<SAFETY_ALLOWED_AREA, LoopBackDemoChannel>> on_SAFETY_ALLOWED_AREA = new OnReceive<>();
        public final Collection<OnReceive.Handler<ATTITUDE_QUATERNION_COV, LoopBackDemoChannel>> on_ATTITUDE_QUATERNION_COV = new OnReceive<>();
        public final Collection<OnReceive.Handler<NAV_CONTROLLER_OUTPUT, LoopBackDemoChannel>> on_NAV_CONTROLLER_OUTPUT = new OnReceive<>();
        public final Collection<OnReceive.Handler<GLOBAL_POSITION_INT_COV, LoopBackDemoChannel>> on_GLOBAL_POSITION_INT_COV = new OnReceive<>();
        public final Collection<OnReceive.Handler<LOCAL_POSITION_NED_COV, LoopBackDemoChannel>> on_LOCAL_POSITION_NED_COV = new OnReceive<>();
        public final Collection<OnReceive.Handler<RC_CHANNELS, LoopBackDemoChannel>> on_RC_CHANNELS = new OnReceive<>();
        public final Collection<OnReceive.Handler<REQUEST_DATA_STREAM, LoopBackDemoChannel>> on_REQUEST_DATA_STREAM = new OnReceive<>();
        public final Collection<OnReceive.Handler<DATA_STREAM, LoopBackDemoChannel>> on_DATA_STREAM = new OnReceive<>();
        public final Collection<OnReceive.Handler<MANUAL_CONTROL, LoopBackDemoChannel>> on_MANUAL_CONTROL = new OnReceive<>();
        public final Collection<OnReceive.Handler<RC_CHANNELS_OVERRIDE, LoopBackDemoChannel>> on_RC_CHANNELS_OVERRIDE = new OnReceive<>();
        public final Collection<OnReceive.Handler<MISSION_ITEM_INT, LoopBackDemoChannel>> on_MISSION_ITEM_INT = new OnReceive<>();
        public final Collection<OnReceive.Handler<VFR_HUD, LoopBackDemoChannel>> on_VFR_HUD = new OnReceive<>();
        public final Collection<OnReceive.Handler<COMMAND_INT, LoopBackDemoChannel>> on_COMMAND_INT = new OnReceive<>();
        public final Collection<OnReceive.Handler<COMMAND_LONG, LoopBackDemoChannel>> on_COMMAND_LONG = new OnReceive<>();
        public final Collection<OnReceive.Handler<COMMAND_ACK, LoopBackDemoChannel>> on_COMMAND_ACK = new OnReceive<>();
        public final Collection<OnReceive.Handler<MANUAL_SETPOINT, LoopBackDemoChannel>> on_MANUAL_SETPOINT = new OnReceive<>();
        public final Collection<OnReceive.Handler<SET_ATTITUDE_TARGET, LoopBackDemoChannel>> on_SET_ATTITUDE_TARGET = new OnReceive<>();
        public final Collection<OnReceive.Handler<ATTITUDE_TARGET, LoopBackDemoChannel>> on_ATTITUDE_TARGET = new OnReceive<>();
        public final Collection<OnReceive.Handler<SET_POSITION_TARGET_LOCAL_NED, LoopBackDemoChannel>> on_SET_POSITION_TARGET_LOCAL_NED = new OnReceive<>();
        public final Collection<OnReceive.Handler<SET_POSITION_TARGET_GLOBAL_INT, LoopBackDemoChannel>> on_SET_POSITION_TARGET_GLOBAL_INT = new OnReceive<>();
        public final Collection<OnReceive.Handler<POSITION_TARGET_GLOBAL_INT, LoopBackDemoChannel>> on_POSITION_TARGET_GLOBAL_INT = new OnReceive<>();
        public final Collection<OnReceive.Handler<LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET, LoopBackDemoChannel>> on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET = new OnReceive<>();
        public final Collection<OnReceive.Handler<HIL_STATE, LoopBackDemoChannel>> on_HIL_STATE = new OnReceive<>();
        public final Collection<OnReceive.Handler<HIL_CONTROLS, LoopBackDemoChannel>> on_HIL_CONTROLS = new OnReceive<>();
        public final Collection<OnReceive.Handler<HIL_RC_INPUTS_RAW, LoopBackDemoChannel>> on_HIL_RC_INPUTS_RAW = new OnReceive<>();
        public final Collection<OnReceive.Handler<HIL_ACTUATOR_CONTROLS, LoopBackDemoChannel>> on_HIL_ACTUATOR_CONTROLS = new OnReceive<>();
        public final Collection<OnReceive.Handler<OPTICAL_FLOW, LoopBackDemoChannel>> on_OPTICAL_FLOW = new OnReceive<>();
        public final Collection<OnReceive.Handler<GLOBAL_VISION_POSITION_ESTIMATE, LoopBackDemoChannel>> on_GLOBAL_VISION_POSITION_ESTIMATE = new OnReceive<>();
        public final Collection<OnReceive.Handler<VISION_POSITION_ESTIMATE, LoopBackDemoChannel>> on_VISION_POSITION_ESTIMATE = new OnReceive<>();
        public final Collection<OnReceive.Handler<VISION_SPEED_ESTIMATE, LoopBackDemoChannel>> on_VISION_SPEED_ESTIMATE = new OnReceive<>();
        public final Collection<OnReceive.Handler<VICON_POSITION_ESTIMATE, LoopBackDemoChannel>> on_VICON_POSITION_ESTIMATE = new OnReceive<>();
        public final Collection<OnReceive.Handler<HIGHRES_IMU, LoopBackDemoChannel>> on_HIGHRES_IMU = new OnReceive<>();
        public final Collection<OnReceive.Handler<OPTICAL_FLOW_RAD, LoopBackDemoChannel>> on_OPTICAL_FLOW_RAD = new OnReceive<>();
        public final Collection<OnReceive.Handler<HIL_SENSOR, LoopBackDemoChannel>> on_HIL_SENSOR = new OnReceive<>();
        public final Collection<OnReceive.Handler<SIM_STATE, LoopBackDemoChannel>> on_SIM_STATE = new OnReceive<>();
        public final Collection<OnReceive.Handler<RADIO_STATUS, LoopBackDemoChannel>> on_RADIO_STATUS = new OnReceive<>();
        public final Collection<OnReceive.Handler<FILE_TRANSFER_PROTOCOL, LoopBackDemoChannel>> on_FILE_TRANSFER_PROTOCOL = new OnReceive<>();
        public final Collection<OnReceive.Handler<TIMESYNC, LoopBackDemoChannel>> on_TIMESYNC = new OnReceive<>();
        public final Collection<OnReceive.Handler<CAMERA_TRIGGER, LoopBackDemoChannel>> on_CAMERA_TRIGGER = new OnReceive<>();
        public final Collection<OnReceive.Handler<HIL_GPS, LoopBackDemoChannel>> on_HIL_GPS = new OnReceive<>();
        public final Collection<OnReceive.Handler<HIL_OPTICAL_FLOW, LoopBackDemoChannel>> on_HIL_OPTICAL_FLOW = new OnReceive<>();
        public final Collection<OnReceive.Handler<HIL_STATE_QUATERNION, LoopBackDemoChannel>> on_HIL_STATE_QUATERNION = new OnReceive<>();
        public final Collection<OnReceive.Handler<SCALED_IMU2, LoopBackDemoChannel>> on_SCALED_IMU2 = new OnReceive<>();
        public final Collection<OnReceive.Handler<LOG_REQUEST_LIST, LoopBackDemoChannel>> on_LOG_REQUEST_LIST = new OnReceive<>();
        public final Collection<OnReceive.Handler<LOG_ENTRY, LoopBackDemoChannel>> on_LOG_ENTRY = new OnReceive<>();
        public final Collection<OnReceive.Handler<LOG_REQUEST_DATA, LoopBackDemoChannel>> on_LOG_REQUEST_DATA = new OnReceive<>();
        public final Collection<OnReceive.Handler<LOG_DATA, LoopBackDemoChannel>> on_LOG_DATA = new OnReceive<>();
        public final Collection<OnReceive.Handler<LOG_ERASE, LoopBackDemoChannel>> on_LOG_ERASE = new OnReceive<>();
        public final Collection<OnReceive.Handler<LOG_REQUEST_END, LoopBackDemoChannel>> on_LOG_REQUEST_END = new OnReceive<>();
        public final Collection<OnReceive.Handler<GPS_INJECT_DATA, LoopBackDemoChannel>> on_GPS_INJECT_DATA = new OnReceive<>();
        public final Collection<OnReceive.Handler<GPS2_RAW, LoopBackDemoChannel>> on_GPS2_RAW = new OnReceive<>();
        public final Collection<OnReceive.Handler<POWER_STATUS, LoopBackDemoChannel>> on_POWER_STATUS = new OnReceive<>();
        public final Collection<OnReceive.Handler<SERIAL_CONTROL, LoopBackDemoChannel>> on_SERIAL_CONTROL = new OnReceive<>();
        public final Collection<OnReceive.Handler<GPS_RTK, LoopBackDemoChannel>> on_GPS_RTK = new OnReceive<>();
        public final Collection<OnReceive.Handler<GPS2_RTK, LoopBackDemoChannel>> on_GPS2_RTK = new OnReceive<>();
        public final Collection<OnReceive.Handler<SCALED_IMU3, LoopBackDemoChannel>> on_SCALED_IMU3 = new OnReceive<>();
        public final Collection<OnReceive.Handler<DATA_TRANSMISSION_HANDSHAKE, LoopBackDemoChannel>> on_DATA_TRANSMISSION_HANDSHAKE = new OnReceive<>();
        public final Collection<OnReceive.Handler<ENCAPSULATED_DATA, LoopBackDemoChannel>> on_ENCAPSULATED_DATA = new OnReceive<>();
        public final Collection<OnReceive.Handler<DISTANCE_SENSOR, LoopBackDemoChannel>> on_DISTANCE_SENSOR = new OnReceive<>();
        public final Collection<OnReceive.Handler<TERRAIN_REQUEST, LoopBackDemoChannel>> on_TERRAIN_REQUEST = new OnReceive<>();
        public final Collection<OnReceive.Handler<TERRAIN_DATA, LoopBackDemoChannel>> on_TERRAIN_DATA = new OnReceive<>();
        public final Collection<OnReceive.Handler<TERRAIN_CHECK, LoopBackDemoChannel>> on_TERRAIN_CHECK = new OnReceive<>();
        public final Collection<OnReceive.Handler<TERRAIN_REPORT, LoopBackDemoChannel>> on_TERRAIN_REPORT = new OnReceive<>();
        public final Collection<OnReceive.Handler<SCALED_PRESSURE2, LoopBackDemoChannel>> on_SCALED_PRESSURE2 = new OnReceive<>();
        public final Collection<OnReceive.Handler<ATT_POS_MOCAP, LoopBackDemoChannel>> on_ATT_POS_MOCAP = new OnReceive<>();
        public final Collection<OnReceive.Handler<SET_ACTUATOR_CONTROL_TARGET, LoopBackDemoChannel>> on_SET_ACTUATOR_CONTROL_TARGET = new OnReceive<>();
        public final Collection<OnReceive.Handler<ACTUATOR_CONTROL_TARGET, LoopBackDemoChannel>> on_ACTUATOR_CONTROL_TARGET = new OnReceive<>();
        public final Collection<OnReceive.Handler<ALTITUDE, LoopBackDemoChannel>> on_ALTITUDE = new OnReceive<>();
        public final Collection<OnReceive.Handler<RESOURCE_REQUEST, LoopBackDemoChannel>> on_RESOURCE_REQUEST = new OnReceive<>();
        public final Collection<OnReceive.Handler<SCALED_PRESSURE3, LoopBackDemoChannel>> on_SCALED_PRESSURE3 = new OnReceive<>();
        public final Collection<OnReceive.Handler<FOLLOW_TARGET, LoopBackDemoChannel>> on_FOLLOW_TARGET = new OnReceive<>();
        public final Collection<OnReceive.Handler<CONTROL_SYSTEM_STATE, LoopBackDemoChannel>> on_CONTROL_SYSTEM_STATE = new OnReceive<>();
        public final Collection<OnReceive.Handler<BATTERY_STATUS, LoopBackDemoChannel>> on_BATTERY_STATUS = new OnReceive<>();
        public final Collection<OnReceive.Handler<AUTOPILOT_VERSION, LoopBackDemoChannel>> on_AUTOPILOT_VERSION = new OnReceive<>();
        public final Collection<OnReceive.Handler<LANDING_TARGET, LoopBackDemoChannel>> on_LANDING_TARGET = new OnReceive<>();
        public final Collection<OnReceive.Handler<NAV_FILTER_BIAS, LoopBackDemoChannel>> on_NAV_FILTER_BIAS = new OnReceive<>();
        public final Collection<OnReceive.Handler<RADIO_CALIBRATION, LoopBackDemoChannel>> on_RADIO_CALIBRATION = new OnReceive<>();
        public final Collection<OnReceive.Handler<UALBERTA_SYS_STATUS, LoopBackDemoChannel>> on_UALBERTA_SYS_STATUS = new OnReceive<>();
        public final Collection<OnReceive.Handler<ESTIMATOR_STATUS, LoopBackDemoChannel>> on_ESTIMATOR_STATUS = new OnReceive<>();
        public final Collection<OnReceive.Handler<WIND_COV, LoopBackDemoChannel>> on_WIND_COV = new OnReceive<>();
        public final Collection<OnReceive.Handler<GPS_INPUT, LoopBackDemoChannel>> on_GPS_INPUT = new OnReceive<>();
        public final Collection<OnReceive.Handler<GPS_RTCM_DATA, LoopBackDemoChannel>> on_GPS_RTCM_DATA = new OnReceive<>();
        public final Collection<OnReceive.Handler<HIGH_LATENCY, LoopBackDemoChannel>> on_HIGH_LATENCY = new OnReceive<>();
        public final Collection<OnReceive.Handler<VIBRATION, LoopBackDemoChannel>> on_VIBRATION = new OnReceive<>();
        public final Collection<OnReceive.Handler<HOME_POSITION, LoopBackDemoChannel>> on_HOME_POSITION = new OnReceive<>();
        public final Collection<OnReceive.Handler<SET_HOME_POSITION, LoopBackDemoChannel>> on_SET_HOME_POSITION = new OnReceive<>();
        public final Collection<OnReceive.Handler<MESSAGE_INTERVAL, LoopBackDemoChannel>> on_MESSAGE_INTERVAL = new OnReceive<>();
        public final Collection<OnReceive.Handler<EXTENDED_SYS_STATE, LoopBackDemoChannel>> on_EXTENDED_SYS_STATE = new OnReceive<>();
        public final Collection<OnReceive.Handler<ADSB_VEHICLE, LoopBackDemoChannel>> on_ADSB_VEHICLE = new OnReceive<>();
        public final Collection<OnReceive.Handler<COLLISION, LoopBackDemoChannel>> on_COLLISION = new OnReceive<>();
        public final Collection<OnReceive.Handler<V2_EXTENSION, LoopBackDemoChannel>> on_V2_EXTENSION = new OnReceive<>();
        public final Collection<OnReceive.Handler<MEMORY_VECT, LoopBackDemoChannel>> on_MEMORY_VECT = new OnReceive<>();
        public final Collection<OnReceive.Handler<DEBUG_VECT, LoopBackDemoChannel>> on_DEBUG_VECT = new OnReceive<>();
        public final Collection<OnReceive.Handler<NAMED_VALUE_FLOAT, LoopBackDemoChannel>> on_NAMED_VALUE_FLOAT = new OnReceive<>();
        public final Collection<OnReceive.Handler<NAMED_VALUE_INT, LoopBackDemoChannel>> on_NAMED_VALUE_INT = new OnReceive<>();
        public final Collection<OnReceive.Handler<STATUSTEXT, LoopBackDemoChannel>> on_STATUSTEXT = new OnReceive<>();
        public final Collection<OnReceive.Handler<DEBUG, LoopBackDemoChannel>> on_DEBUG = new OnReceive<>();
        public final Collection<OnReceive.Handler<SETUP_SIGNING, LoopBackDemoChannel>> on_SETUP_SIGNING = new OnReceive<>();
        public final Collection<OnReceive.Handler<BUTTON_CHANGE, LoopBackDemoChannel>> on_BUTTON_CHANGE = new OnReceive<>();
        public final Collection<OnReceive.Handler<PLAY_TUNE, LoopBackDemoChannel>> on_PLAY_TUNE = new OnReceive<>();
        public final Collection<OnReceive.Handler<CAMERA_INFORMATION, LoopBackDemoChannel>> on_CAMERA_INFORMATION = new OnReceive<>();
        public final Collection<OnReceive.Handler<CAMERA_SETTINGS, LoopBackDemoChannel>> on_CAMERA_SETTINGS = new OnReceive<>();
        public final Collection<OnReceive.Handler<STORAGE_INFORMATION, LoopBackDemoChannel>> on_STORAGE_INFORMATION = new OnReceive<>();
        public final Collection<OnReceive.Handler<CAMERA_CAPTURE_STATUS, LoopBackDemoChannel>> on_CAMERA_CAPTURE_STATUS = new OnReceive<>();
        public final Collection<OnReceive.Handler<CAMERA_IMAGE_CAPTURED, LoopBackDemoChannel>> on_CAMERA_IMAGE_CAPTURED = new OnReceive<>();
        public final Collection<OnReceive.Handler<FLIGHT_INFORMATION, LoopBackDemoChannel>> on_FLIGHT_INFORMATION = new OnReceive<>();
        public final Collection<OnReceive.Handler<MOUNT_ORIENTATION, LoopBackDemoChannel>> on_MOUNT_ORIENTATION = new OnReceive<>();
        public final Collection<OnReceive.Handler<LOGGING_DATA, LoopBackDemoChannel>> on_LOGGING_DATA = new OnReceive<>();
        public final Collection<OnReceive.Handler<LOGGING_DATA_ACKED, LoopBackDemoChannel>> on_LOGGING_DATA_ACKED = new OnReceive<>();
        public final Collection<OnReceive.Handler<LOGGING_ACK, LoopBackDemoChannel>> on_LOGGING_ACK = new OnReceive<>();
        public final Collection<OnReceive.Handler<VIDEO_STREAM_INFORMATION, LoopBackDemoChannel>> on_VIDEO_STREAM_INFORMATION = new OnReceive<>();
        public final Collection<OnReceive.Handler<SET_VIDEO_STREAM_SETTINGS, LoopBackDemoChannel>> on_SET_VIDEO_STREAM_SETTINGS = new OnReceive<>();
        public final Collection<OnReceive.Handler<WIFI_CONFIG_AP, LoopBackDemoChannel>> on_WIFI_CONFIG_AP = new OnReceive<>();
        public final Collection<OnReceive.Handler<PROTOCOL_VERSION, LoopBackDemoChannel>> on_PROTOCOL_VERSION = new OnReceive<>();
        public final Collection<OnReceive.Handler<UAVCAN_NODE_STATUS, LoopBackDemoChannel>> on_UAVCAN_NODE_STATUS = new OnReceive<>();
        public final Collection<OnReceive.Handler<UAVCAN_NODE_INFO, LoopBackDemoChannel>> on_UAVCAN_NODE_INFO = new OnReceive<>();
        public final Collection<OnReceive.Handler<PARAM_EXT_REQUEST_READ, LoopBackDemoChannel>> on_PARAM_EXT_REQUEST_READ = new OnReceive<>();
        public final Collection<OnReceive.Handler<PARAM_EXT_REQUEST_LIST, LoopBackDemoChannel>> on_PARAM_EXT_REQUEST_LIST = new OnReceive<>();
        public final Collection<OnReceive.Handler<PARAM_EXT_VALUE, LoopBackDemoChannel>> on_PARAM_EXT_VALUE = new OnReceive<>();
        public final Collection<OnReceive.Handler<PARAM_EXT_SET, LoopBackDemoChannel>> on_PARAM_EXT_SET = new OnReceive<>();
        public final Collection<OnReceive.Handler<PARAM_EXT_ACK, LoopBackDemoChannel>> on_PARAM_EXT_ACK = new OnReceive<>();
        public final Collection<OnReceive.Handler<OBSTACLE_DISTANCE, LoopBackDemoChannel>> on_OBSTACLE_DISTANCE = new OnReceive<>();
    }

    public static class LoopBackDemoChannel_ADV  extends Channel
    {
        static {pack_id_bytes = 2; }

        public static  LoopBackDemoChannel_ADV instance = new LoopBackDemoChannel_ADV();

        public final java.io.InputStream inputStream = new  AdvancedInputStream();
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
        public static NAV_FILTER_BIAS new_NAV_FILTER_BIAS() {return new  NAV_FILTER_BIAS();}
        public static RADIO_CALIBRATION new_RADIO_CALIBRATION() {return new  RADIO_CALIBRATION();}
        public static UALBERTA_SYS_STATUS new_UALBERTA_SYS_STATUS() {return new  UALBERTA_SYS_STATUS();}
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

        public void send(Sendable pack) { sendout_packs.add((Pack) pack);}
        protected final Queue<Pack> sendout_packs = new ConcurrentLinkedQueue<Pack>()
        {
            @Override public boolean add(Pack pack)
            {
                synchronized(this)
                {
                    boolean ret = super.add(pack);
                    this.notify();
                    return ret;
                }
            }
        };

        public boolean waitingSendoutPack()
        {
            try
            {
                synchronized(sendout_packs)
                {
                    while(sendout_packs.size() == 0) sendout_packs.wait();
                }
                return true;
            }
            catch(InterruptedException e) {}
            return false;
        }


        @Override protected Pack process(Pack pack, int id)
        {
            switch(id)
            {
                default:
                    assert(false);
                    return null;
                case Channel.PROCESS_CHANNEL_REQEST:
                    if(pack == null) return sendout_packs.poll();
                    return null;
                case Channel.PROCESS_HOST_REQEST:
                    sendout_packs.add(pack);
                    return null;
            }
        }

    }

    /*
                  */
    public @interface MAV_TYPE
    {
        int
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
        MAV_TYPE_PARAFOIL = 28;
    }
    /*
    				                                              */
    public @interface MAV_AUTOPILOT
    {
        int
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
        MAV_AUTOPILOT_SMARTAP = 18;
    }
    /*
    				                                              */
    public @interface MAV_MODE_FLAG
    {
        int
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1,
        MAV_MODE_FLAG_TEST_ENABLED = 2,
        MAV_MODE_FLAG_AUTO_ENABLED = 4,
        MAV_MODE_FLAG_GUIDED_ENABLED = 8,
        MAV_MODE_FLAG_STABILIZE_ENABLED = 16,
        MAV_MODE_FLAG_HIL_ENABLED = 32,
        MAV_MODE_FLAG_MANUAL_INPUT_ENABLED = 64,
        MAV_MODE_FLAG_SAFETY_ARMED = 128;
    }
    /*
    				                                              */
    public @interface MAV_STATE
    {
        int
        MAV_STATE_UNINIT = 0,
        MAV_STATE_BOOT = 1,
        MAV_STATE_CALIBRATING = 2,
        MAV_STATE_STANDBY = 3,
        MAV_STATE_ACTIVE = 4,
        MAV_STATE_CRITICAL = 5,
        MAV_STATE_EMERGENCY = 6,
        MAV_STATE_POWEROFF = 7,
        MAV_STATE_FLIGHT_TERMINATION = 8;
    }
    /*
    				                                              */
    public @interface MAV_SYS_STATUS_SENSOR
    {
        int
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
        MAV_SYS_STATUS_SENSOR_BATTERY = 33554432;
    }
    /*
    				                                              */
    public @interface MAV_FRAME
    {
        int
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
        MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11;
    }
    /*
    				                                              */
    public @interface MAV_MODE
    {
        int
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
        MAV_MODE_AUTO_ARMED = 220;
    }
    /*
    				                                              */
    public @interface MAV_PARAM_TYPE
    {
        int
        MAV_PARAM_TYPE_UINT8 = 1,
        MAV_PARAM_TYPE_INT8 = 2,
        MAV_PARAM_TYPE_UINT16 = 3,
        MAV_PARAM_TYPE_INT16 = 4,
        MAV_PARAM_TYPE_UINT32 = 5,
        MAV_PARAM_TYPE_INT32 = 6,
        MAV_PARAM_TYPE_UINT64 = 7,
        MAV_PARAM_TYPE_INT64 = 8,
        MAV_PARAM_TYPE_REAL32 = 9,
        MAV_PARAM_TYPE_REAL64 = 10;
    }
    /*
    				                                              */
    public @interface GPS_FIX_TYPE
    {
        int
        GPS_FIX_TYPE_NO_GPS = 0,
        GPS_FIX_TYPE_NO_FIX = 1,
        GPS_FIX_TYPE_2D_FIX = 2,
        GPS_FIX_TYPE_3D_FIX = 3,
        GPS_FIX_TYPE_DGPS = 4,
        GPS_FIX_TYPE_RTK_FLOAT = 5,
        GPS_FIX_TYPE_RTK_FIXED = 6,
        GPS_FIX_TYPE_STATIC = 7,
        GPS_FIX_TYPE_PPP = 8;
    }
    /*
    				                                              */
    public @interface MAV_MISSION_TYPE
    {
        int
        MAV_MISSION_TYPE_MISSION = 0,
        MAV_MISSION_TYPE_FENCE = 1,
        MAV_MISSION_TYPE_RALLY = 2,
        MAV_MISSION_TYPE_ALL = 255;
    }
    /*
    				                                              */
    public @interface MAV_CMD
    {
        int
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
        MAV_CMD_USER_5 = 31014;
    }
    /*
    				                                              */
    public @interface MAV_MISSION_RESULT
    {
        int
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
        MAV_MISSION_DENIED = 14;
    }
    /*
    				                                              */
    public @interface MAV_ESTIMATOR_TYPE
    {
        int
        MAV_ESTIMATOR_TYPE_NAIVE = 1,
        MAV_ESTIMATOR_TYPE_VISION = 2,
        MAV_ESTIMATOR_TYPE_VIO = 3,
        MAV_ESTIMATOR_TYPE_GPS = 4,
        MAV_ESTIMATOR_TYPE_GPS_INS = 5;
    }
    /*
    				                                              */
    public @interface MAV_RESULT
    {
        int
        MAV_RESULT_ACCEPTED = 0,
        MAV_RESULT_TEMPORARILY_REJECTED = 1,
        MAV_RESULT_DENIED = 2,
        MAV_RESULT_UNSUPPORTED = 3,
        MAV_RESULT_FAILED = 4,
        MAV_RESULT_IN_PROGRESS = 5;
    }
    /*
    				                                              */
    public @interface MAV_POWER_STATUS
    {
        int
        MAV_POWER_STATUS_BRICK_VALID = 1,
        MAV_POWER_STATUS_SERVO_VALID = 2,
        MAV_POWER_STATUS_USB_CONNECTED = 4,
        MAV_POWER_STATUS_PERIPH_OVERCURRENT = 8,
        MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT = 16,
        MAV_POWER_STATUS_CHANGED = 32;
    }
    /*
    				                                              */
    public @interface SERIAL_CONTROL_DEV
    {
        int
        SERIAL_CONTROL_DEV_TELEM1 = 0,
        SERIAL_CONTROL_DEV_TELEM2 = 1,
        SERIAL_CONTROL_DEV_GPS1 = 2,
        SERIAL_CONTROL_DEV_GPS2 = 3,
        SERIAL_CONTROL_DEV_SHELL = 10;
    }
    /*
    				                                              */
    public @interface SERIAL_CONTROL_FLAG
    {
        int
        SERIAL_CONTROL_FLAG_REPLY = 1,
        SERIAL_CONTROL_FLAG_RESPOND = 2,
        SERIAL_CONTROL_FLAG_EXCLUSIVE = 4,
        SERIAL_CONTROL_FLAG_BLOCKING = 8,
        SERIAL_CONTROL_FLAG_MULTI = 16;
    }
    /*
    				                                              */
    public @interface MAV_DISTANCE_SENSOR
    {
        int
        MAV_DISTANCE_SENSOR_LASER = 0,
        MAV_DISTANCE_SENSOR_ULTRASOUND = 1,
        MAV_DISTANCE_SENSOR_INFRARED = 2,
        MAV_DISTANCE_SENSOR_RADAR = 3,
        MAV_DISTANCE_SENSOR_UNKNOWN = 4;
    }
    /*
    				                                              */
    public @interface MAV_SENSOR_ORIENTATION
    {
        int
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
        MAV_SENSOR_ROTATION_ROLL_315_PITCH_315_YAW_315 = 38;
    }
    /*
    				                                              */
    public @interface MAV_BATTERY_FUNCTION
    {
        int
        MAV_BATTERY_FUNCTION_UNKNOWN = 0,
        MAV_BATTERY_FUNCTION_ALL = 1,
        MAV_BATTERY_FUNCTION_PROPULSION = 2,
        MAV_BATTERY_FUNCTION_AVIONICS = 3,
        MAV_BATTERY_TYPE_PAYLOAD = 4;
    }
    /*
    				                                              */
    public @interface MAV_BATTERY_TYPE
    {
        int
        MAV_BATTERY_TYPE_UNKNOWN = 0,
        MAV_BATTERY_TYPE_LIPO = 1,
        MAV_BATTERY_TYPE_LIFE = 2,
        MAV_BATTERY_TYPE_LION = 3,
        MAV_BATTERY_TYPE_NIMH = 4;
    }
    /*
    				                                              */
    public @interface MAV_PROTOCOL_CAPABILITY
    {
        int
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
        MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION = 65536;
    }
    /*
    				                                              */
    public @interface LANDING_TARGET_TYPE
    {
        int
        LANDING_TARGET_TYPE_LIGHT_BEACON = 0,
        LANDING_TARGET_TYPE_RADIO_BEACON = 1,
        LANDING_TARGET_TYPE_VISION_FIDUCIAL = 2,
        LANDING_TARGET_TYPE_VISION_OTHER = 3;
    }
    /*
    				                                              */
    public @interface ESTIMATOR_STATUS_FLAGS
    {
        int
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
        ESTIMATOR_GPS_GLITCH = 1024;
    }
    /*
    				                                              */
    public @interface GPS_INPUT_IGNORE_FLAGS
    {
        int
        GPS_INPUT_IGNORE_FLAG_ALT = 1,
        GPS_INPUT_IGNORE_FLAG_HDOP = 2,
        GPS_INPUT_IGNORE_FLAG_VDOP = 4,
        GPS_INPUT_IGNORE_FLAG_VEL_HORIZ = 8,
        GPS_INPUT_IGNORE_FLAG_VEL_VERT = 16,
        GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY = 32,
        GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY = 64,
        GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY = 128;
    }
    /*
    				                                              */
    public @interface MAV_LANDED_STATE
    {
        int
        MAV_LANDED_STATE_UNDEFINED = 0,
        MAV_LANDED_STATE_ON_GROUND = 1,
        MAV_LANDED_STATE_IN_AIR = 2,
        MAV_LANDED_STATE_TAKEOFF = 3,
        MAV_LANDED_STATE_LANDING = 4;
    }
    /*
    				                                              */
    public @interface MAV_VTOL_STATE
    {
        int
        MAV_VTOL_STATE_UNDEFINED = 0,
        MAV_VTOL_STATE_TRANSITION_TO_FW = 1,
        MAV_VTOL_STATE_TRANSITION_TO_MC = 2,
        MAV_VTOL_STATE_MC = 3,
        MAV_VTOL_STATE_FW = 4;
    }
    /*
    				                                              */
    public @interface ADSB_ALTITUDE_TYPE
    {
        int
        ADSB_ALTITUDE_TYPE_PRESSURE_QNH = 0,
        ADSB_ALTITUDE_TYPE_GEOMETRIC = 1;
    }
    /*
    				                                              */
    public @interface ADSB_EMITTER_TYPE
    {
        int
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
        ADSB_EMITTER_TYPE_POINT_OBSTACLE = 19;
    }
    /*
    				                                              */
    public @interface ADSB_FLAGS
    {
        int
        ADSB_FLAGS_VALID_COORDS = 1,
        ADSB_FLAGS_VALID_ALTITUDE = 2,
        ADSB_FLAGS_VALID_HEADING = 4,
        ADSB_FLAGS_VALID_VELOCITY = 8,
        ADSB_FLAGS_VALID_CALLSIGN = 16,
        ADSB_FLAGS_VALID_SQUAWK = 32,
        ADSB_FLAGS_SIMULATED = 64;
    }
    /*
    				                                              */
    public @interface MAV_COLLISION_SRC
    {
        int
        MAV_COLLISION_SRC_ADSB = 0,
        MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT = 1;
    }
    /*
    				                                              */
    public @interface MAV_COLLISION_ACTION
    {
        int
        MAV_COLLISION_ACTION_NONE = 0,
        MAV_COLLISION_ACTION_REPORT = 1,
        MAV_COLLISION_ACTION_ASCEND_OR_DESCEND = 2,
        MAV_COLLISION_ACTION_MOVE_HORIZONTALLY = 3,
        MAV_COLLISION_ACTION_MOVE_PERPENDICULAR = 4,
        MAV_COLLISION_ACTION_RTL = 5,
        MAV_COLLISION_ACTION_HOVER = 6;
    }
    /*
    				                                              */
    public @interface MAV_COLLISION_THREAT_LEVEL
    {
        int
        MAV_COLLISION_THREAT_LEVEL_NONE = 0,
        MAV_COLLISION_THREAT_LEVEL_LOW = 1,
        MAV_COLLISION_THREAT_LEVEL_HIGH = 2;
    }
    /*
    				                                              */
    public @interface MAV_SEVERITY
    {
        int
        MAV_SEVERITY_EMERGENCY = 0,
        MAV_SEVERITY_ALERT = 1,
        MAV_SEVERITY_CRITICAL = 2,
        MAV_SEVERITY_ERROR = 3,
        MAV_SEVERITY_WARNING = 4,
        MAV_SEVERITY_NOTICE = 5,
        MAV_SEVERITY_INFO = 6,
        MAV_SEVERITY_DEBUG = 7;
    }
    /*
    				                                              */
    public @interface CAMERA_CAP_FLAGS
    {
        int
        CAMERA_CAP_FLAGS_CAPTURE_VIDEO = 1,
        CAMERA_CAP_FLAGS_CAPTURE_IMAGE = 2,
        CAMERA_CAP_FLAGS_HAS_MODES = 4,
        CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE = 8,
        CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE = 16,
        CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE = 32;
    }
    /*
    				                                              */
    public @interface CAMERA_MODE
    {
        int
        CAMERA_MODE_IMAGE = 0,
        CAMERA_MODE_VIDEO = 1,
        CAMERA_MODE_IMAGE_SURVEY = 2;
    }
    /*
    				                                              */
    public @interface UAVCAN_NODE_HEALTH
    {
        int
        UAVCAN_NODE_HEALTH_OK = 0,
        UAVCAN_NODE_HEALTH_WARNING = 1,
        UAVCAN_NODE_HEALTH_ERROR = 2,
        UAVCAN_NODE_HEALTH_CRITICAL = 3;
    }
    /*
    				                                              */
    public @interface UAVCAN_NODE_MODE
    {
        int
        UAVCAN_NODE_MODE_OPERATIONAL = 0,
        UAVCAN_NODE_MODE_INITIALIZATION = 1,
        UAVCAN_NODE_MODE_MAINTENANCE = 2,
        UAVCAN_NODE_MODE_SOFTWARE_UPDATE = 3,
        UAVCAN_NODE_MODE_OFFLINE = 7;
    }
    /*
    				                                              */
    public @interface MAV_PARAM_EXT_TYPE
    {
        int
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
        MAV_PARAM_EXT_TYPE_CUSTOM = 11;
    }
    /*
    				                                              */
    public @interface PARAM_ACK
    {
        int
        PARAM_ACK_ACCEPTED = 0,
        PARAM_ACK_VALUE_UNSUPPORTED = 1,
        PARAM_ACK_FAILED = 2,
        PARAM_ACK_IN_PROGRESS = 3;
    }


    private static final Field _I = new Field(5, true, -5, 2, 1, 0, 0, 0, 1);
    private static final Field _k = new Field(5, true, -6, 2, 1, 0, 0, 0, 1);
    private static final Field _i = new Field(5, true, -5, 2, 1, 0, 0, 0, 1);
    private static final Field _D = new Field(5, true, -5, 2, 1, 0, 0, 0, 1);
    private static final Field _T = new Field(5, true, -5, 2, 1, 0, 0, 0, 1);
    private static final Field _uE = new Field(0, false, 1, 4, 1, 0, 0, 0);
    private static final Field _GE = new Field(0, true, 1, 4, 1, 0, 0, 0);
    private static final Field _FE = new Field(0, true, 1, 4, 1, 0, 0, 0);
    private static final Field _BE = new Field(0, true, 1, 4, 1, 0, 0, 0);
    private static final Field _VE = new Field(0, true, 1, 4, 1, 0, 0, 0);
    private static final Field _JW = new Field(0, true, 1, 2, 1, 0, 0, 0);
    private static final Field _uW = new Field(0, true, 1, 2, 1, 0, 0, 0);
    private static final Field _GW = new Field(0, true, 1, 2, 1, 0, 0, 0);
    private static final Field _FW = new Field(0, true, 1, 2, 1, 0, 0, 0);
    private static final Field _BW = new Field(0, true, 1, 2, 1, 0, 0, 0);
    private static final Field _VW = new Field(0, true, 1, 2, 1, 0, 0, 0);
    private static final Field _jW = new Field(0, true, 1, 2, 1, 0, 0, 0);
    private static final Field _zW = new Field(0, true, 1, 2, 1, 0, 0, 0);
    private static final Field _ey = new Field(0, true, 1, 8, 1, 0, 0, 0);
    private static final Field _Oy = new Field(0, true, 1, 8, 1, 0, 0, 0);
    private static final Field _Uy = new Field(5, true, -5, 2, 1, 0, 0, 0, 1);
    private static final Field _sv = new Field(0, false, 1, 1, 1, 0, 0, 0);
    private static final Field _Uv = new Field(0, false, 1, 4, 1, 0, 0, 0);
    private static final Field _pv = new Field(0, false, 1, 1, 1, 0, 0, 0);
    private static final Field _Rv = new Field(0, false, 1, 1, 1, 0, 0, 0);
    private static final Field _Vu = new Field(0, false, 1, 4, 1, 0, 0, 0);
    private static final Field _ju = new Field(0, false, 1, 4, 1, 0, 0, 0);
    private static final Field _Se = new Field(0, false, 18, 1, 1, 0, 0, 0);
    private static final Field _Ve = new Field(0, false, 1, 4, 1, 0, 0, 0);
    private static final Field _je = new Field(0, false, 1, 4, 1, 0, 0, 0);
    private static final Field _ze = new Field(0, false, 1, 4, 1, 0, 0, 0);
    private static final Field _ee = new Field(0, false, 4, 4, 1, 0, 0, 0);
    private static final Field _qe = new Field(0, false, 1, 1, 1, 0, 0, 0);
    private static final Field _Tn = new Field(0, true, 1, 8, 1, 0, 0, 0);
    private static final Field _Jq = new Field(0, true, 1, 8, 1, 0, 0, 0);
    private static final Field _Xq = new Field(5, true, -4, 2, 1, 0, 0, 0, 1);
    private static final Field _bq = new Field(5, true, -4, 2, 1, 0, 0, 0, 1);
    private static final Field _Tq = new Field(5, true, -4, 2, 1, 0, 0, 0, 1);
    private static final Field _xM = new Field(5, true, -4, 2, 1, 0, 0, 0, 1);
    private static final Field _WM = new Field(5, true, -6, 2, 1, 0, 0, 0, 1);
    private static final Field _zM = new Field(5, true, -5, 2, 1, 0, 0, 0, 1);
    private static final Field _gM = new Field(5, true, -8, 2, 1, 0, 0, 0, 1);
    private static final Field _SO = new Field(5, true, -8, 2, 1, 0, 0, 0, 1);
    private static final Field _rO = new Field(5, true, -8, 2, 1, 0, 0, 0, 1);
    private static final Field _lO = new Field(5, true, -8, 2, 1, 0, 0, 0, 1);
    private static final Field _YO = new Field(5, true, -6, 2, 1, 0, 0, 0, 1);
    private static final Field _LO = new Field(5, true, -7, 2, 1, 0, 0, 0, 1);
    private static final Field _JX = new Field(5, true, -7, 2, 1, 0, 0, 0, 1);
    private static final Field _nX = new Field(5, true, -5, 2, 1, 0, 0, 0, 1);
    private static final Field _XX = new Field(5, true, -5, 2, 1, 0, 0, 0, 1);
    private static final Field _sX = new Field(5, true, -8, 2, 1, 0, 0, 0, 1);
    private static final Field _gX = new Field(5, true, -5, 2, 1, 0, 0, 0, 1);
    private static final Field _wX = new Field(5, true, -8, 2, 1, 0, 0, 0, 1);
    private static final Field _kX = new Field(5, true, -5, 2, 1, 0, 0, 0, 1);
    private static final Field _tX = new Field(5, true, -8, 2, 1, 0, 0, 0, 1);

}

using System.Diagnostics;
using System.Threading;
using Inside = org.unirail.BlackBox.Host.Pack.Meta.Field.Bounds.Inside;
namespace org.noname
{
    public class Demo :  DemoDevice
    {
        static void Main_(string[] args)
        {
            Inside PH = new Inside();
            LoopBackDemoChannel.instance.OnHEARTBEATReceive += (src, ph, pack) =>
            {
                MAV_TYPE type = pack.type;
                MAV_AUTOPILOT autopilot = pack.autopilot;
                MAV_MODE_FLAG base_mode = pack.base_mode;
                uint custom_mode = pack.custom_mode;
                MAV_STATE system_status = pack.system_status;
                byte mavlink_version = pack.mavlink_version;
            };
            LoopBackDemoChannel.instance.OnSYS_STATUSReceive += (src, ph, pack) =>
            {
                MAV_SYS_STATUS_SENSOR onboard_control_sensors_present = pack.onboard_control_sensors_present;
                MAV_SYS_STATUS_SENSOR onboard_control_sensors_enabled = pack.onboard_control_sensors_enabled;
                MAV_SYS_STATUS_SENSOR onboard_control_sensors_health = pack.onboard_control_sensors_health;
                ushort load = pack.load;
                ushort voltage_battery = pack.voltage_battery;
                short current_battery = pack.current_battery;
                sbyte battery_remaining = pack.battery_remaining;
                ushort drop_rate_comm = pack.drop_rate_comm;
                ushort errors_comm = pack.errors_comm;
                ushort errors_count1 = pack.errors_count1;
                ushort errors_count2 = pack.errors_count2;
                ushort errors_count3 = pack.errors_count3;
                ushort errors_count4 = pack.errors_count4;
            };
            LoopBackDemoChannel.instance.OnSYSTEM_TIMEReceive += (src, ph, pack) =>
            {
                ulong time_unix_usec = pack.time_unix_usec;
                uint time_boot_ms = pack.time_boot_ms;
            };
            LoopBackDemoChannel.instance.OnPOSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                MAV_FRAME coordinate_frame = pack.coordinate_frame;
                ushort type_mask = pack.type_mask;
                float x = pack.x;
                float y = pack.y;
                float z = pack.z;
                float vx = pack.vx;
                float vy = pack.vy;
                float vz = pack.vz;
                float afx = pack.afx;
                float afy = pack.afy;
                float afz = pack.afz;
                float yaw = pack.yaw;
                float yaw_rate = pack.yaw_rate;
            };
            LoopBackDemoChannel.instance.OnPINGReceive += (src, ph, pack) =>
            {
                ulong time_usec = pack.time_usec;
                uint seq = pack.seq;
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
            };
            LoopBackDemoChannel.instance.OnCHANGE_OPERATOR_CONTROLReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte control_request = pack.control_request;
                byte version = pack.version;
                string passkey = pack.passkey_TRY(ph);
            };
            LoopBackDemoChannel.instance.OnCHANGE_OPERATOR_CONTROL_ACKReceive += (src, ph, pack) =>
            {
                byte gcs_system_id = pack.gcs_system_id;
                byte control_request = pack.control_request;
                byte ack = pack.ack;
            };
            LoopBackDemoChannel.instance.OnAUTH_KEYReceive += (src, ph, pack) =>
            {
                string key = pack.key_TRY(ph);
            };
            LoopBackDemoChannel.instance.OnSET_MODEReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                MAV_MODE base_mode = pack.base_mode;
                uint custom_mode = pack.custom_mode;
            };
            LoopBackDemoChannel.instance.OnPARAM_REQUEST_READReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                string param_id = pack.param_id_TRY(ph);
                short param_index = pack.param_index;
            };
            LoopBackDemoChannel.instance.OnPARAM_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
            };
            LoopBackDemoChannel.instance.OnPARAM_VALUEReceive += (src, ph, pack) =>
            {
                string param_id = pack.param_id_TRY(ph);
                float param_value = pack.param_value;
                MAV_PARAM_TYPE param_type = pack.param_type;
                ushort param_count = pack.param_count;
                ushort param_index = pack.param_index;
            };
            LoopBackDemoChannel.instance.OnPARAM_SETReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                string param_id = pack.param_id_TRY(ph);
                float param_value = pack.param_value;
                MAV_PARAM_TYPE param_type = pack.param_type;
            };
            LoopBackDemoChannel.instance.OnGPS_RAW_INTReceive += (src, ph, pack) =>
            {
                ulong time_usec = pack.time_usec;
                GPS_FIX_TYPE fix_type = pack.fix_type;
                int lat = pack.lat;
                int lon = pack.lon;
                int alt = pack.alt;
                ushort eph = pack.eph;
                ushort epv = pack.epv;
                ushort vel = pack.vel;
                ushort cog = pack.cog;
                byte satellites_visible = pack.satellites_visible;
                int alt_ellipsoid = pack.alt_ellipsoid_TRY(ph);
                uint h_acc = pack.h_acc_TRY(ph);
                uint v_acc = pack.v_acc_TRY(ph);
                uint vel_acc = pack.vel_acc_TRY(ph);
                uint hdg_acc = pack.hdg_acc_TRY(ph);
            };
            LoopBackDemoChannel.instance.OnGPS_STATUSReceive += (src, ph, pack) =>
            {
                byte satellites_visible = pack.satellites_visible;
                byte[] satellite_prn = pack.satellite_prn;
                byte[] satellite_used = pack.satellite_used;
                byte[] satellite_elevation = pack.satellite_elevation;
                byte[] satellite_azimuth = pack.satellite_azimuth;
                byte[] satellite_snr = pack.satellite_snr;
            };
            LoopBackDemoChannel.instance.OnSCALED_IMUReceive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                short xacc = pack.xacc;
                short yacc = pack.yacc;
                short zacc = pack.zacc;
                short xgyro = pack.xgyro;
                short ygyro = pack.ygyro;
                short zgyro = pack.zgyro;
                short xmag = pack.xmag;
                short ymag = pack.ymag;
                short zmag = pack.zmag;
            };
            LoopBackDemoChannel.instance.OnRAW_IMUReceive += (src, ph, pack) =>
            {
                ulong time_usec = pack.time_usec;
                short xacc = pack.xacc;
                short yacc = pack.yacc;
                short zacc = pack.zacc;
                short xgyro = pack.xgyro;
                short ygyro = pack.ygyro;
                short zgyro = pack.zgyro;
                short xmag = pack.xmag;
                short ymag = pack.ymag;
                short zmag = pack.zmag;
            };
            LoopBackDemoChannel.instance.OnRAW_PRESSUREReceive += (src, ph, pack) =>
            {
                ulong time_usec = pack.time_usec;
                short press_abs = pack.press_abs;
                short press_diff1 = pack.press_diff1;
                short press_diff2 = pack.press_diff2;
                short temperature = pack.temperature;
            };
            LoopBackDemoChannel.instance.OnSCALED_PRESSUREReceive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                float press_abs = pack.press_abs;
                float press_diff = pack.press_diff;
                short temperature = pack.temperature;
            };
            LoopBackDemoChannel.instance.OnATTITUDEReceive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                float roll = pack.roll;
                float pitch = pack.pitch;
                float yaw = pack.yaw;
                float rollspeed = pack.rollspeed;
                float pitchspeed = pack.pitchspeed;
                float yawspeed = pack.yawspeed;
            };
            LoopBackDemoChannel.instance.OnATTITUDE_QUATERNIONReceive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                float q1 = pack.q1;
                float q2 = pack.q2;
                float q3 = pack.q3;
                float q4 = pack.q4;
                float rollspeed = pack.rollspeed;
                float pitchspeed = pack.pitchspeed;
                float yawspeed = pack.yawspeed;
            };
            LoopBackDemoChannel.instance.OnLOCAL_POSITION_NEDReceive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                float x = pack.x;
                float y = pack.y;
                float z = pack.z;
                float vx = pack.vx;
                float vy = pack.vy;
                float vz = pack.vz;
            };
            LoopBackDemoChannel.instance.OnGLOBAL_POSITION_INTReceive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                int lat = pack.lat;
                int lon = pack.lon;
                int alt = pack.alt;
                int relative_alt = pack.relative_alt;
                short vx = pack.vx;
                short vy = pack.vy;
                short vz = pack.vz;
                ushort hdg = pack.hdg;
            };
            LoopBackDemoChannel.instance.OnRC_CHANNELS_SCALEDReceive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                byte port = pack.port;
                short chan1_scaled = pack.chan1_scaled;
                short chan2_scaled = pack.chan2_scaled;
                short chan3_scaled = pack.chan3_scaled;
                short chan4_scaled = pack.chan4_scaled;
                short chan5_scaled = pack.chan5_scaled;
                short chan6_scaled = pack.chan6_scaled;
                short chan7_scaled = pack.chan7_scaled;
                short chan8_scaled = pack.chan8_scaled;
                byte rssi = pack.rssi;
            };
            LoopBackDemoChannel.instance.OnRC_CHANNELS_RAWReceive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                byte port = pack.port;
                ushort chan1_raw = pack.chan1_raw;
                ushort chan2_raw = pack.chan2_raw;
                ushort chan3_raw = pack.chan3_raw;
                ushort chan4_raw = pack.chan4_raw;
                ushort chan5_raw = pack.chan5_raw;
                ushort chan6_raw = pack.chan6_raw;
                ushort chan7_raw = pack.chan7_raw;
                ushort chan8_raw = pack.chan8_raw;
                byte rssi = pack.rssi;
            };
            LoopBackDemoChannel.instance.OnSERVO_OUTPUT_RAWReceive += (src, ph, pack) =>
            {
                uint time_usec = pack.time_usec;
                byte port = pack.port;
                ushort servo1_raw = pack.servo1_raw;
                ushort servo2_raw = pack.servo2_raw;
                ushort servo3_raw = pack.servo3_raw;
                ushort servo4_raw = pack.servo4_raw;
                ushort servo5_raw = pack.servo5_raw;
                ushort servo6_raw = pack.servo6_raw;
                ushort servo7_raw = pack.servo7_raw;
                ushort servo8_raw = pack.servo8_raw;
                ushort servo9_raw = pack.servo9_raw_TRY(ph);
                ushort servo10_raw = pack.servo10_raw_TRY(ph);
                ushort servo11_raw = pack.servo11_raw_TRY(ph);
                ushort servo12_raw = pack.servo12_raw_TRY(ph);
                ushort servo13_raw = pack.servo13_raw_TRY(ph);
                ushort servo14_raw = pack.servo14_raw_TRY(ph);
                ushort servo15_raw = pack.servo15_raw_TRY(ph);
                ushort servo16_raw = pack.servo16_raw_TRY(ph);
            };
            LoopBackDemoChannel.instance.OnMISSION_REQUEST_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                short start_index = pack.start_index;
                short end_index = pack.end_index;
                MAV_MISSION_TYPE mission_type = pack.mission_type;
            };
            LoopBackDemoChannel.instance.OnMISSION_WRITE_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                short start_index = pack.start_index;
                short end_index = pack.end_index;
                MAV_MISSION_TYPE mission_type = pack.mission_type;
            };
            LoopBackDemoChannel.instance.OnMISSION_ITEMReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                ushort seq = pack.seq;
                MAV_FRAME frame = pack.frame;
                MAV_CMD command = pack.command;
                byte current = pack.current;
                byte autocontinue = pack.autocontinue;
                float param1 = pack.param1;
                float param2 = pack.param2;
                float param3 = pack.param3;
                float param4 = pack.param4;
                float x = pack.x;
                float y = pack.y;
                float z = pack.z;
                MAV_MISSION_TYPE mission_type = pack.mission_type;
            };
            LoopBackDemoChannel.instance.OnMISSION_REQUESTReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                ushort seq = pack.seq;
                MAV_MISSION_TYPE mission_type = pack.mission_type;
            };
            LoopBackDemoChannel.instance.OnMISSION_SET_CURRENTReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                ushort seq = pack.seq;
            };
            LoopBackDemoChannel.instance.OnMISSION_CURRENTReceive += (src, ph, pack) =>
            {
                ushort seq = pack.seq;
            };
            LoopBackDemoChannel.instance.OnMISSION_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                MAV_MISSION_TYPE mission_type = pack.mission_type;
            };
            LoopBackDemoChannel.instance.OnMISSION_COUNTReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                ushort count = pack.count;
                MAV_MISSION_TYPE mission_type = pack.mission_type;
            };
            LoopBackDemoChannel.instance.OnMISSION_CLEAR_ALLReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                MAV_MISSION_TYPE mission_type = pack.mission_type;
            };
            LoopBackDemoChannel.instance.OnMISSION_ITEM_REACHEDReceive += (src, ph, pack) =>
            {
                ushort seq = pack.seq;
            };
            LoopBackDemoChannel.instance.OnMISSION_ACKReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                MAV_MISSION_RESULT type = pack.type;
                MAV_MISSION_TYPE mission_type = pack.mission_type;
            };
            LoopBackDemoChannel.instance.OnSET_GPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                int latitude = pack.latitude;
                int longitude = pack.longitude;
                int altitude = pack.altitude;
                ulong time_usec = pack.time_usec_TRY(ph);
            };
            LoopBackDemoChannel.instance.OnGPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                int latitude = pack.latitude;
                int longitude = pack.longitude;
                int altitude = pack.altitude;
                ulong time_usec = pack.time_usec_TRY(ph);
            };
            LoopBackDemoChannel.instance.OnPARAM_MAP_RCReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                string param_id = pack.param_id_TRY(ph);
                short param_index = pack.param_index;
                byte parameter_rc_channel_index = pack.parameter_rc_channel_index;
                float param_value0 = pack.param_value0;
                float scale = pack.scale;
                float param_value_min = pack.param_value_min;
                float param_value_max = pack.param_value_max;
            };
            LoopBackDemoChannel.instance.OnMISSION_REQUEST_INTReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                ushort seq = pack.seq;
                MAV_MISSION_TYPE mission_type = pack.mission_type;
            };
            LoopBackDemoChannel.instance.OnSAFETY_SET_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                MAV_FRAME frame = pack.frame;
                float p1x = pack.p1x;
                float p1y = pack.p1y;
                float p1z = pack.p1z;
                float p2x = pack.p2x;
                float p2y = pack.p2y;
                float p2z = pack.p2z;
            };
            LoopBackDemoChannel.instance.OnSAFETY_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                MAV_FRAME frame = pack.frame;
                float p1x = pack.p1x;
                float p1y = pack.p1y;
                float p1z = pack.p1z;
                float p2x = pack.p2x;
                float p2y = pack.p2y;
                float p2z = pack.p2z;
            };
            LoopBackDemoChannel.instance.OnATTITUDE_QUATERNION_COVReceive += (src, ph, pack) =>
            {
                ulong time_usec = pack.time_usec;
                float[] q = pack.q;
                float rollspeed = pack.rollspeed;
                float pitchspeed = pack.pitchspeed;
                float yawspeed = pack.yawspeed;
                float[] covariance = pack.covariance;
            };
            LoopBackDemoChannel.instance.OnNAV_CONTROLLER_OUTPUTReceive += (src, ph, pack) =>
            {
                float nav_roll = pack.nav_roll;
                float nav_pitch = pack.nav_pitch;
                short nav_bearing = pack.nav_bearing;
                short target_bearing = pack.target_bearing;
                ushort wp_dist = pack.wp_dist;
                float alt_error = pack.alt_error;
                float aspd_error = pack.aspd_error;
                float xtrack_error = pack.xtrack_error;
            };
            LoopBackDemoChannel.instance.OnGLOBAL_POSITION_INT_COVReceive += (src, ph, pack) =>
            {
                ulong time_usec = pack.time_usec;
                MAV_ESTIMATOR_TYPE estimator_type = pack.estimator_type;
                int lat = pack.lat;
                int lon = pack.lon;
                int alt = pack.alt;
                int relative_alt = pack.relative_alt;
                float vx = pack.vx;
                float vy = pack.vy;
                float vz = pack.vz;
                float[] covariance = pack.covariance;
            };
            LoopBackDemoChannel.instance.OnLOCAL_POSITION_NED_COVReceive += (src, ph, pack) =>
            {
                ulong time_usec = pack.time_usec;
                MAV_ESTIMATOR_TYPE estimator_type = pack.estimator_type;
                float x = pack.x;
                float y = pack.y;
                float z = pack.z;
                float vx = pack.vx;
                float vy = pack.vy;
                float vz = pack.vz;
                float ax = pack.ax;
                float ay = pack.ay;
                float az = pack.az;
                float[] covariance = pack.covariance;
            };
            LoopBackDemoChannel.instance.OnRC_CHANNELSReceive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                byte chancount = pack.chancount;
                ushort chan1_raw = pack.chan1_raw;
                ushort chan2_raw = pack.chan2_raw;
                ushort chan3_raw = pack.chan3_raw;
                ushort chan4_raw = pack.chan4_raw;
                ushort chan5_raw = pack.chan5_raw;
                ushort chan6_raw = pack.chan6_raw;
                ushort chan7_raw = pack.chan7_raw;
                ushort chan8_raw = pack.chan8_raw;
                ushort chan9_raw = pack.chan9_raw;
                ushort chan10_raw = pack.chan10_raw;
                ushort chan11_raw = pack.chan11_raw;
                ushort chan12_raw = pack.chan12_raw;
                ushort chan13_raw = pack.chan13_raw;
                ushort chan14_raw = pack.chan14_raw;
                ushort chan15_raw = pack.chan15_raw;
                ushort chan16_raw = pack.chan16_raw;
                ushort chan17_raw = pack.chan17_raw;
                ushort chan18_raw = pack.chan18_raw;
                byte rssi = pack.rssi;
            };
            LoopBackDemoChannel.instance.OnREQUEST_DATA_STREAMReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                byte req_stream_id = pack.req_stream_id;
                ushort req_message_rate = pack.req_message_rate;
                byte start_stop = pack.start_stop;
            };
            LoopBackDemoChannel.instance.OnDATA_STREAMReceive += (src, ph, pack) =>
            {
                byte stream_id = pack.stream_id;
                ushort message_rate = pack.message_rate;
                byte on_off = pack.on_off;
            };
            LoopBackDemoChannel.instance.OnMANUAL_CONTROLReceive += (src, ph, pack) =>
            {
                byte target = pack.target;
                short x = pack.x;
                short y = pack.y;
                short z = pack.z;
                short r = pack.r;
                ushort buttons = pack.buttons;
            };
            LoopBackDemoChannel.instance.OnRC_CHANNELS_OVERRIDEReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                ushort chan1_raw = pack.chan1_raw;
                ushort chan2_raw = pack.chan2_raw;
                ushort chan3_raw = pack.chan3_raw;
                ushort chan4_raw = pack.chan4_raw;
                ushort chan5_raw = pack.chan5_raw;
                ushort chan6_raw = pack.chan6_raw;
                ushort chan7_raw = pack.chan7_raw;
                ushort chan8_raw = pack.chan8_raw;
            };
            LoopBackDemoChannel.instance.OnMISSION_ITEM_INTReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                ushort seq = pack.seq;
                MAV_FRAME frame = pack.frame;
                MAV_CMD command = pack.command;
                byte current = pack.current;
                byte autocontinue = pack.autocontinue;
                float param1 = pack.param1;
                float param2 = pack.param2;
                float param3 = pack.param3;
                float param4 = pack.param4;
                int x = pack.x;
                int y = pack.y;
                float z = pack.z;
                MAV_MISSION_TYPE mission_type = pack.mission_type;
            };
            LoopBackDemoChannel.instance.OnVFR_HUDReceive += (src, ph, pack) =>
            {
                float airspeed = pack.airspeed;
                float groundspeed = pack.groundspeed;
                short heading = pack.heading;
                ushort throttle = pack.throttle;
                float alt = pack.alt;
                float climb = pack.climb;
            };
            LoopBackDemoChannel.instance.OnCOMMAND_INTReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                MAV_FRAME frame = pack.frame;
                MAV_CMD command = pack.command;
                byte current = pack.current;
                byte autocontinue = pack.autocontinue;
                float param1 = pack.param1;
                float param2 = pack.param2;
                float param3 = pack.param3;
                float param4 = pack.param4;
                int x = pack.x;
                int y = pack.y;
                float z = pack.z;
            };
            LoopBackDemoChannel.instance.OnCOMMAND_LONGReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                MAV_CMD command = pack.command;
                byte confirmation = pack.confirmation;
                float param1 = pack.param1;
                float param2 = pack.param2;
                float param3 = pack.param3;
                float param4 = pack.param4;
                float param5 = pack.param5;
                float param6 = pack.param6;
                float param7 = pack.param7;
            };
            LoopBackDemoChannel.instance.OnCOMMAND_ACKReceive += (src, ph, pack) =>
            {
                MAV_CMD command = pack.command;
                MAV_RESULT result = pack.result;
                byte progress = pack.progress_TRY(ph);
                int result_param2 = pack.result_param2_TRY(ph);
                byte target_system = pack.target_system_TRY(ph);
                byte target_component = pack.target_component_TRY(ph);
            };
            LoopBackDemoChannel.instance.OnMANUAL_SETPOINTReceive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                float roll = pack.roll;
                float pitch = pack.pitch;
                float yaw = pack.yaw;
                float thrust = pack.thrust;
                byte mode_switch = pack.mode_switch;
                byte manual_override_switch = pack.manual_override_switch;
            };
            LoopBackDemoChannel.instance.OnSET_ATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                byte type_mask = pack.type_mask;
                float[] q = pack.q;
                float body_roll_rate = pack.body_roll_rate;
                float body_pitch_rate = pack.body_pitch_rate;
                float body_yaw_rate = pack.body_yaw_rate;
                float thrust = pack.thrust;
            };
            LoopBackDemoChannel.instance.OnATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                byte type_mask = pack.type_mask;
                float[] q = pack.q;
                float body_roll_rate = pack.body_roll_rate;
                float body_pitch_rate = pack.body_pitch_rate;
                float body_yaw_rate = pack.body_yaw_rate;
                float thrust = pack.thrust;
            };
            LoopBackDemoChannel.instance.OnSET_POSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                MAV_FRAME coordinate_frame = pack.coordinate_frame;
                ushort type_mask = pack.type_mask;
                float x = pack.x;
                float y = pack.y;
                float z = pack.z;
                float vx = pack.vx;
                float vy = pack.vy;
                float vz = pack.vz;
                float afx = pack.afx;
                float afy = pack.afy;
                float afz = pack.afz;
                float yaw = pack.yaw;
                float yaw_rate = pack.yaw_rate;
            };
            LoopBackDemoChannel.instance.OnSET_POSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                MAV_FRAME coordinate_frame = pack.coordinate_frame;
                ushort type_mask = pack.type_mask;
                int lat_int = pack.lat_int;
                int lon_int = pack.lon_int;
                float alt = pack.alt;
                float vx = pack.vx;
                float vy = pack.vy;
                float vz = pack.vz;
                float afx = pack.afx;
                float afy = pack.afy;
                float afz = pack.afz;
                float yaw = pack.yaw;
                float yaw_rate = pack.yaw_rate;
            };
            LoopBackDemoChannel.instance.OnPOSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                MAV_FRAME coordinate_frame = pack.coordinate_frame;
                ushort type_mask = pack.type_mask;
                int lat_int = pack.lat_int;
                int lon_int = pack.lon_int;
                float alt = pack.alt;
                float vx = pack.vx;
                float vy = pack.vy;
                float vz = pack.vz;
                float afx = pack.afx;
                float afy = pack.afy;
                float afz = pack.afz;
                float yaw = pack.yaw;
                float yaw_rate = pack.yaw_rate;
            };
            LoopBackDemoChannel.instance.OnLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                float x = pack.x;
                float y = pack.y;
                float z = pack.z;
                float roll = pack.roll;
                float pitch = pack.pitch;
                float yaw = pack.yaw;
            };
            LoopBackDemoChannel.instance.OnHIL_STATEReceive += (src, ph, pack) =>
            {
                ulong time_usec = pack.time_usec;
                float roll = pack.roll;
                float pitch = pack.pitch;
                float yaw = pack.yaw;
                float rollspeed = pack.rollspeed;
                float pitchspeed = pack.pitchspeed;
                float yawspeed = pack.yawspeed;
                int lat = pack.lat;
                int lon = pack.lon;
                int alt = pack.alt;
                short vx = pack.vx;
                short vy = pack.vy;
                short vz = pack.vz;
                short xacc = pack.xacc;
                short yacc = pack.yacc;
                short zacc = pack.zacc;
            };
            LoopBackDemoChannel.instance.OnHIL_CONTROLSReceive += (src, ph, pack) =>
            {
                ulong time_usec = pack.time_usec;
                float roll_ailerons = pack.roll_ailerons;
                float pitch_elevator = pack.pitch_elevator;
                float yaw_rudder = pack.yaw_rudder;
                float throttle = pack.throttle;
                float aux1 = pack.aux1;
                float aux2 = pack.aux2;
                float aux3 = pack.aux3;
                float aux4 = pack.aux4;
                MAV_MODE mode = pack.mode;
                byte nav_mode = pack.nav_mode;
            };
            LoopBackDemoChannel.instance.OnHIL_RC_INPUTS_RAWReceive += (src, ph, pack) =>
            {
                ulong time_usec = pack.time_usec;
                ushort chan1_raw = pack.chan1_raw;
                ushort chan2_raw = pack.chan2_raw;
                ushort chan3_raw = pack.chan3_raw;
                ushort chan4_raw = pack.chan4_raw;
                ushort chan5_raw = pack.chan5_raw;
                ushort chan6_raw = pack.chan6_raw;
                ushort chan7_raw = pack.chan7_raw;
                ushort chan8_raw = pack.chan8_raw;
                ushort chan9_raw = pack.chan9_raw;
                ushort chan10_raw = pack.chan10_raw;
                ushort chan11_raw = pack.chan11_raw;
                ushort chan12_raw = pack.chan12_raw;
                byte rssi = pack.rssi;
            };
            LoopBackDemoChannel.instance.OnHIL_ACTUATOR_CONTROLSReceive += (src, ph, pack) =>
            {
                ulong time_usec = pack.time_usec;
                float[] controls = pack.controls;
                MAV_MODE mode = pack.mode;
                ulong flags = pack.flags;
            };
            LoopBackDemoChannel.instance.OnOPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                ulong time_usec = pack.time_usec;
                byte sensor_id = pack.sensor_id;
                short flow_x = pack.flow_x;
                short flow_y = pack.flow_y;
                float flow_comp_m_x = pack.flow_comp_m_x;
                float flow_comp_m_y = pack.flow_comp_m_y;
                byte quality = pack.quality;
                float ground_distance = pack.ground_distance;
                float flow_rate_x = pack.flow_rate_x_TRY(ph);
                float flow_rate_y = pack.flow_rate_y_TRY(ph);
            };
            LoopBackDemoChannel.instance.OnGLOBAL_VISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                ulong usec = pack.usec;
                float x = pack.x;
                float y = pack.y;
                float z = pack.z;
                float roll = pack.roll;
                float pitch = pack.pitch;
                float yaw = pack.yaw;
            };
            LoopBackDemoChannel.instance.OnVISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                ulong usec = pack.usec;
                float x = pack.x;
                float y = pack.y;
                float z = pack.z;
                float roll = pack.roll;
                float pitch = pack.pitch;
                float yaw = pack.yaw;
            };
            LoopBackDemoChannel.instance.OnVISION_SPEED_ESTIMATEReceive += (src, ph, pack) =>
            {
                ulong usec = pack.usec;
                float x = pack.x;
                float y = pack.y;
                float z = pack.z;
            };
            LoopBackDemoChannel.instance.OnVICON_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                ulong usec = pack.usec;
                float x = pack.x;
                float y = pack.y;
                float z = pack.z;
                float roll = pack.roll;
                float pitch = pack.pitch;
                float yaw = pack.yaw;
            };
            LoopBackDemoChannel.instance.OnHIGHRES_IMUReceive += (src, ph, pack) =>
            {
                ulong time_usec = pack.time_usec;
                float xacc = pack.xacc;
                float yacc = pack.yacc;
                float zacc = pack.zacc;
                float xgyro = pack.xgyro;
                float ygyro = pack.ygyro;
                float zgyro = pack.zgyro;
                float xmag = pack.xmag;
                float ymag = pack.ymag;
                float zmag = pack.zmag;
                float abs_pressure = pack.abs_pressure;
                float diff_pressure = pack.diff_pressure;
                float pressure_alt = pack.pressure_alt;
                float temperature = pack.temperature;
                ushort fields_updated = pack.fields_updated;
            };
            LoopBackDemoChannel.instance.OnOPTICAL_FLOW_RADReceive += (src, ph, pack) =>
            {
                ulong time_usec = pack.time_usec;
                byte sensor_id = pack.sensor_id;
                uint integration_time_us = pack.integration_time_us;
                float integrated_x = pack.integrated_x;
                float integrated_y = pack.integrated_y;
                float integrated_xgyro = pack.integrated_xgyro;
                float integrated_ygyro = pack.integrated_ygyro;
                float integrated_zgyro = pack.integrated_zgyro;
                short temperature = pack.temperature;
                byte quality = pack.quality;
                uint time_delta_distance_us = pack.time_delta_distance_us;
                float distance = pack.distance;
            };
            LoopBackDemoChannel.instance.OnHIL_SENSORReceive += (src, ph, pack) =>
            {
                ulong time_usec = pack.time_usec;
                float xacc = pack.xacc;
                float yacc = pack.yacc;
                float zacc = pack.zacc;
                float xgyro = pack.xgyro;
                float ygyro = pack.ygyro;
                float zgyro = pack.zgyro;
                float xmag = pack.xmag;
                float ymag = pack.ymag;
                float zmag = pack.zmag;
                float abs_pressure = pack.abs_pressure;
                float diff_pressure = pack.diff_pressure;
                float pressure_alt = pack.pressure_alt;
                float temperature = pack.temperature;
                uint fields_updated = pack.fields_updated;
            };
            LoopBackDemoChannel.instance.OnSIM_STATEReceive += (src, ph, pack) =>
            {
                float q1 = pack.q1;
                float q2 = pack.q2;
                float q3 = pack.q3;
                float q4 = pack.q4;
                float roll = pack.roll;
                float pitch = pack.pitch;
                float yaw = pack.yaw;
                float xacc = pack.xacc;
                float yacc = pack.yacc;
                float zacc = pack.zacc;
                float xgyro = pack.xgyro;
                float ygyro = pack.ygyro;
                float zgyro = pack.zgyro;
                float lat = pack.lat;
                float lon = pack.lon;
                float alt = pack.alt;
                float std_dev_horz = pack.std_dev_horz;
                float std_dev_vert = pack.std_dev_vert;
                float vn = pack.vn;
                float ve = pack.ve;
                float vd = pack.vd;
            };
            LoopBackDemoChannel.instance.OnRADIO_STATUSReceive += (src, ph, pack) =>
            {
                byte rssi = pack.rssi;
                byte remrssi = pack.remrssi;
                byte txbuf = pack.txbuf;
                byte noise = pack.noise;
                byte remnoise = pack.remnoise;
                ushort rxerrors = pack.rxerrors;
                ushort fixed_ = pack.fixed_;
            };
            LoopBackDemoChannel.instance.OnFILE_TRANSFER_PROTOCOLReceive += (src, ph, pack) =>
            {
                byte target_network = pack.target_network;
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                byte[] payload = pack.payload;
            };
            LoopBackDemoChannel.instance.OnTIMESYNCReceive += (src, ph, pack) =>
            {
                long tc1 = pack.tc1;
                long ts1 = pack.ts1;
            };
            LoopBackDemoChannel.instance.OnCAMERA_TRIGGERReceive += (src, ph, pack) =>
            {
                ulong time_usec = pack.time_usec;
                uint seq = pack.seq;
            };
            LoopBackDemoChannel.instance.OnHIL_GPSReceive += (src, ph, pack) =>
            {
                ulong time_usec = pack.time_usec;
                byte fix_type = pack.fix_type;
                int lat = pack.lat;
                int lon = pack.lon;
                int alt = pack.alt;
                ushort eph = pack.eph;
                ushort epv = pack.epv;
                ushort vel = pack.vel;
                short vn = pack.vn;
                short ve = pack.ve;
                short vd = pack.vd;
                ushort cog = pack.cog;
                byte satellites_visible = pack.satellites_visible;
            };
            LoopBackDemoChannel.instance.OnHIL_OPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                ulong time_usec = pack.time_usec;
                byte sensor_id = pack.sensor_id;
                uint integration_time_us = pack.integration_time_us;
                float integrated_x = pack.integrated_x;
                float integrated_y = pack.integrated_y;
                float integrated_xgyro = pack.integrated_xgyro;
                float integrated_ygyro = pack.integrated_ygyro;
                float integrated_zgyro = pack.integrated_zgyro;
                short temperature = pack.temperature;
                byte quality = pack.quality;
                uint time_delta_distance_us = pack.time_delta_distance_us;
                float distance = pack.distance;
            };
            LoopBackDemoChannel.instance.OnHIL_STATE_QUATERNIONReceive += (src, ph, pack) =>
            {
                ulong time_usec = pack.time_usec;
                float[] attitude_quaternion = pack.attitude_quaternion;
                float rollspeed = pack.rollspeed;
                float pitchspeed = pack.pitchspeed;
                float yawspeed = pack.yawspeed;
                int lat = pack.lat;
                int lon = pack.lon;
                int alt = pack.alt;
                short vx = pack.vx;
                short vy = pack.vy;
                short vz = pack.vz;
                ushort ind_airspeed = pack.ind_airspeed;
                ushort true_airspeed = pack.true_airspeed;
                short xacc = pack.xacc;
                short yacc = pack.yacc;
                short zacc = pack.zacc;
            };
            LoopBackDemoChannel.instance.OnSCALED_IMU2Receive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                short xacc = pack.xacc;
                short yacc = pack.yacc;
                short zacc = pack.zacc;
                short xgyro = pack.xgyro;
                short ygyro = pack.ygyro;
                short zgyro = pack.zgyro;
                short xmag = pack.xmag;
                short ymag = pack.ymag;
                short zmag = pack.zmag;
            };
            LoopBackDemoChannel.instance.OnLOG_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                ushort start = pack.start;
                ushort end = pack.end;
            };
            LoopBackDemoChannel.instance.OnLOG_ENTRYReceive += (src, ph, pack) =>
            {
                ushort id = pack.id;
                ushort num_logs = pack.num_logs;
                ushort last_log_num = pack.last_log_num;
                uint time_utc = pack.time_utc;
                uint size = pack.size;
            };
            LoopBackDemoChannel.instance.OnLOG_REQUEST_DATAReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                ushort id = pack.id;
                uint ofs = pack.ofs;
                uint count = pack.count;
            };
            LoopBackDemoChannel.instance.OnLOG_DATAReceive += (src, ph, pack) =>
            {
                ushort id = pack.id;
                uint ofs = pack.ofs;
                byte count = pack.count;
                byte[] data_ = pack.data_;
            };
            LoopBackDemoChannel.instance.OnLOG_ERASEReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
            };
            LoopBackDemoChannel.instance.OnLOG_REQUEST_ENDReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
            };
            LoopBackDemoChannel.instance.OnGPS_INJECT_DATAReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                byte len = pack.len;
                byte[] data_ = pack.data_;
            };
            LoopBackDemoChannel.instance.OnGPS2_RAWReceive += (src, ph, pack) =>
            {
                ulong time_usec = pack.time_usec;
                GPS_FIX_TYPE fix_type = pack.fix_type;
                int lat = pack.lat;
                int lon = pack.lon;
                int alt = pack.alt;
                ushort eph = pack.eph;
                ushort epv = pack.epv;
                ushort vel = pack.vel;
                ushort cog = pack.cog;
                byte satellites_visible = pack.satellites_visible;
                byte dgps_numch = pack.dgps_numch;
                uint dgps_age = pack.dgps_age;
            };
            LoopBackDemoChannel.instance.OnPOWER_STATUSReceive += (src, ph, pack) =>
            {
                ushort Vcc = pack.Vcc;
                ushort Vservo = pack.Vservo;
                MAV_POWER_STATUS flags = pack.flags;
            };
            LoopBackDemoChannel.instance.OnSERIAL_CONTROLReceive += (src, ph, pack) =>
            {
                SERIAL_CONTROL_DEV device = pack.device;
                SERIAL_CONTROL_FLAG flags = pack.flags;
                ushort timeout = pack.timeout;
                uint baudrate = pack.baudrate;
                byte count = pack.count;
                byte[] data_ = pack.data_;
            };
            LoopBackDemoChannel.instance.OnGPS_RTKReceive += (src, ph, pack) =>
            {
                uint time_last_baseline_ms = pack.time_last_baseline_ms;
                byte rtk_receiver_id = pack.rtk_receiver_id;
                ushort wn = pack.wn;
                uint tow = pack.tow;
                byte rtk_health = pack.rtk_health;
                byte rtk_rate = pack.rtk_rate;
                byte nsats = pack.nsats;
                byte baseline_coords_type = pack.baseline_coords_type;
                int baseline_a_mm = pack.baseline_a_mm;
                int baseline_b_mm = pack.baseline_b_mm;
                int baseline_c_mm = pack.baseline_c_mm;
                uint accuracy = pack.accuracy;
                int iar_num_hypotheses = pack.iar_num_hypotheses;
            };
            LoopBackDemoChannel.instance.OnGPS2_RTKReceive += (src, ph, pack) =>
            {
                uint time_last_baseline_ms = pack.time_last_baseline_ms;
                byte rtk_receiver_id = pack.rtk_receiver_id;
                ushort wn = pack.wn;
                uint tow = pack.tow;
                byte rtk_health = pack.rtk_health;
                byte rtk_rate = pack.rtk_rate;
                byte nsats = pack.nsats;
                byte baseline_coords_type = pack.baseline_coords_type;
                int baseline_a_mm = pack.baseline_a_mm;
                int baseline_b_mm = pack.baseline_b_mm;
                int baseline_c_mm = pack.baseline_c_mm;
                uint accuracy = pack.accuracy;
                int iar_num_hypotheses = pack.iar_num_hypotheses;
            };
            LoopBackDemoChannel.instance.OnSCALED_IMU3Receive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                short xacc = pack.xacc;
                short yacc = pack.yacc;
                short zacc = pack.zacc;
                short xgyro = pack.xgyro;
                short ygyro = pack.ygyro;
                short zgyro = pack.zgyro;
                short xmag = pack.xmag;
                short ymag = pack.ymag;
                short zmag = pack.zmag;
            };
            LoopBackDemoChannel.instance.OnDATA_TRANSMISSION_HANDSHAKEReceive += (src, ph, pack) =>
            {
                byte type = pack.type;
                uint size = pack.size;
                ushort width = pack.width;
                ushort height = pack.height;
                ushort packets = pack.packets;
                byte payload = pack.payload;
                byte jpg_quality = pack.jpg_quality;
            };
            LoopBackDemoChannel.instance.OnENCAPSULATED_DATAReceive += (src, ph, pack) =>
            {
                ushort seqnr = pack.seqnr;
                byte[] data_ = pack.data_;
            };
            LoopBackDemoChannel.instance.OnDISTANCE_SENSORReceive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                ushort min_distance = pack.min_distance;
                ushort max_distance = pack.max_distance;
                ushort current_distance = pack.current_distance;
                MAV_DISTANCE_SENSOR type = pack.type;
                byte id = pack.id;
                MAV_SENSOR_ORIENTATION orientation = pack.orientation;
                byte covariance = pack.covariance;
            };
            LoopBackDemoChannel.instance.OnTERRAIN_REQUESTReceive += (src, ph, pack) =>
            {
                int lat = pack.lat;
                int lon = pack.lon;
                ushort grid_spacing = pack.grid_spacing;
                ulong mask = pack.mask;
            };
            LoopBackDemoChannel.instance.OnTERRAIN_DATAReceive += (src, ph, pack) =>
            {
                int lat = pack.lat;
                int lon = pack.lon;
                ushort grid_spacing = pack.grid_spacing;
                byte gridbit = pack.gridbit;
                short[] data_ = pack.data_;
            };
            LoopBackDemoChannel.instance.OnTERRAIN_CHECKReceive += (src, ph, pack) =>
            {
                int lat = pack.lat;
                int lon = pack.lon;
            };
            LoopBackDemoChannel.instance.OnTERRAIN_REPORTReceive += (src, ph, pack) =>
            {
                int lat = pack.lat;
                int lon = pack.lon;
                ushort spacing = pack.spacing;
                float terrain_height = pack.terrain_height;
                float current_height = pack.current_height;
                ushort pending = pack.pending;
                ushort loaded = pack.loaded;
            };
            LoopBackDemoChannel.instance.OnSCALED_PRESSURE2Receive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                float press_abs = pack.press_abs;
                float press_diff = pack.press_diff;
                short temperature = pack.temperature;
            };
            LoopBackDemoChannel.instance.OnATT_POS_MOCAPReceive += (src, ph, pack) =>
            {
                ulong time_usec = pack.time_usec;
                float[] q = pack.q;
                float x = pack.x;
                float y = pack.y;
                float z = pack.z;
            };
            LoopBackDemoChannel.instance.OnSET_ACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                ulong time_usec = pack.time_usec;
                byte group_mlx = pack.group_mlx;
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                float[] controls = pack.controls;
            };
            LoopBackDemoChannel.instance.OnACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                ulong time_usec = pack.time_usec;
                byte group_mlx = pack.group_mlx;
                float[] controls = pack.controls;
            };
            LoopBackDemoChannel.instance.OnALTITUDEReceive += (src, ph, pack) =>
            {
                ulong time_usec = pack.time_usec;
                float altitude_monotonic = pack.altitude_monotonic;
                float altitude_amsl = pack.altitude_amsl;
                float altitude_local = pack.altitude_local;
                float altitude_relative = pack.altitude_relative;
                float altitude_terrain = pack.altitude_terrain;
                float bottom_clearance = pack.bottom_clearance;
            };
            LoopBackDemoChannel.instance.OnRESOURCE_REQUESTReceive += (src, ph, pack) =>
            {
                byte request_id = pack.request_id;
                byte uri_type = pack.uri_type;
                byte[] uri = pack.uri;
                byte transfer_type = pack.transfer_type;
                byte[] storage = pack.storage;
            };
            LoopBackDemoChannel.instance.OnSCALED_PRESSURE3Receive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                float press_abs = pack.press_abs;
                float press_diff = pack.press_diff;
                short temperature = pack.temperature;
            };
            LoopBackDemoChannel.instance.OnFOLLOW_TARGETReceive += (src, ph, pack) =>
            {
                ulong timestamp = pack.timestamp;
                byte est_capabilities = pack.est_capabilities;
                int lat = pack.lat;
                int lon = pack.lon;
                float alt = pack.alt;
                float[] vel = pack.vel;
                float[] acc = pack.acc;
                float[] attitude_q = pack.attitude_q;
                float[] rates = pack.rates;
                float[] position_cov = pack.position_cov;
                ulong custom_state = pack.custom_state;
            };
            LoopBackDemoChannel.instance.OnCONTROL_SYSTEM_STATEReceive += (src, ph, pack) =>
            {
                ulong time_usec = pack.time_usec;
                float x_acc = pack.x_acc;
                float y_acc = pack.y_acc;
                float z_acc = pack.z_acc;
                float x_vel = pack.x_vel;
                float y_vel = pack.y_vel;
                float z_vel = pack.z_vel;
                float x_pos = pack.x_pos;
                float y_pos = pack.y_pos;
                float z_pos = pack.z_pos;
                float airspeed = pack.airspeed;
                float[] vel_variance = pack.vel_variance;
                float[] pos_variance = pack.pos_variance;
                float[] q = pack.q;
                float roll_rate = pack.roll_rate;
                float pitch_rate = pack.pitch_rate;
                float yaw_rate = pack.yaw_rate;
            };
            LoopBackDemoChannel.instance.OnBATTERY_STATUSReceive += (src, ph, pack) =>
            {
                byte id = pack.id;
                MAV_BATTERY_FUNCTION battery_function = pack.battery_function;
                MAV_BATTERY_TYPE type = pack.type;
                short temperature = pack.temperature;
                ushort[] voltages = pack.voltages;
                short current_battery = pack.current_battery;
                int current_consumed = pack.current_consumed;
                int energy_consumed = pack.energy_consumed;
                sbyte battery_remaining = pack.battery_remaining;
            };
            LoopBackDemoChannel.instance.OnAUTOPILOT_VERSIONReceive += (src, ph, pack) =>
            {
                MAV_PROTOCOL_CAPABILITY capabilities = pack.capabilities;
                uint flight_sw_version = pack.flight_sw_version;
                uint middleware_sw_version = pack.middleware_sw_version;
                uint os_sw_version = pack.os_sw_version;
                uint board_version = pack.board_version;
                byte[] flight_custom_version = pack.flight_custom_version;
                byte[] middleware_custom_version = pack.middleware_custom_version;
                byte[] os_custom_version = pack.os_custom_version;
                ushort vendor_id = pack.vendor_id;
                ushort product_id = pack.product_id;
                ulong uid = pack.uid;
                byte[] uid2 = pack.uid2_TRY(ph);
            };
            LoopBackDemoChannel.instance.OnLANDING_TARGETReceive += (src, ph, pack) =>
            {
                ulong time_usec = pack.time_usec;
                byte target_num = pack.target_num;
                MAV_FRAME frame = pack.frame;
                float angle_x = pack.angle_x;
                float angle_y = pack.angle_y;
                float distance = pack.distance;
                float size_x = pack.size_x;
                float size_y = pack.size_y;
                float x = pack.x_TRY(ph);
                float y = pack.y_TRY(ph);
                float z = pack.z_TRY(ph);
                float[] q = pack.q_TRY(ph);
                LANDING_TARGET_TYPE type = pack.type;
                byte position_valid = pack.position_valid_TRY(ph);
            };
            LoopBackDemoChannel.instance.OnSENS_POWERReceive += (src, ph, pack) =>
            {
                float adc121_vspb_volt = pack.adc121_vspb_volt;
                float adc121_cspb_amp = pack.adc121_cspb_amp;
                float adc121_cs1_amp = pack.adc121_cs1_amp;
                float adc121_cs2_amp = pack.adc121_cs2_amp;
            };
            LoopBackDemoChannel.instance.OnSENS_MPPTReceive += (src, ph, pack) =>
            {
                ulong mppt_timestamp = pack.mppt_timestamp;
                float mppt1_volt = pack.mppt1_volt;
                float mppt1_amp = pack.mppt1_amp;
                ushort mppt1_pwm = pack.mppt1_pwm;
                byte mppt1_status = pack.mppt1_status;
                float mppt2_volt = pack.mppt2_volt;
                float mppt2_amp = pack.mppt2_amp;
                ushort mppt2_pwm = pack.mppt2_pwm;
                byte mppt2_status = pack.mppt2_status;
                float mppt3_volt = pack.mppt3_volt;
                float mppt3_amp = pack.mppt3_amp;
                ushort mppt3_pwm = pack.mppt3_pwm;
                byte mppt3_status = pack.mppt3_status;
            };
            LoopBackDemoChannel.instance.OnASLCTRL_DATAReceive += (src, ph, pack) =>
            {
                ulong timestamp = pack.timestamp;
                byte aslctrl_mode = pack.aslctrl_mode;
                float h = pack.h;
                float hRef = pack.hRef;
                float hRef_t = pack.hRef_t;
                float PitchAngle = pack.PitchAngle;
                float PitchAngleRef = pack.PitchAngleRef;
                float q = pack.q;
                float qRef = pack.qRef;
                float uElev = pack.uElev;
                float uThrot = pack.uThrot;
                float uThrot2 = pack.uThrot2;
                float nZ = pack.nZ;
                float AirspeedRef = pack.AirspeedRef;
                byte SpoilersEngaged = pack.SpoilersEngaged;
                float YawAngle = pack.YawAngle;
                float YawAngleRef = pack.YawAngleRef;
                float RollAngle = pack.RollAngle;
                float RollAngleRef = pack.RollAngleRef;
                float p = pack.p;
                float pRef = pack.pRef;
                float r = pack.r;
                float rRef = pack.rRef;
                float uAil = pack.uAil;
                float uRud = pack.uRud;
            };
            LoopBackDemoChannel.instance.OnASLCTRL_DEBUGReceive += (src, ph, pack) =>
            {
                uint i32_1 = pack.i32_1;
                byte i8_1 = pack.i8_1;
                byte i8_2 = pack.i8_2;
                float f_1 = pack.f_1;
                float f_2 = pack.f_2;
                float f_3 = pack.f_3;
                float f_4 = pack.f_4;
                float f_5 = pack.f_5;
                float f_6 = pack.f_6;
                float f_7 = pack.f_7;
                float f_8 = pack.f_8;
            };
            LoopBackDemoChannel.instance.OnASLUAV_STATUSReceive += (src, ph, pack) =>
            {
                byte LED_status = pack.LED_status;
                byte SATCOM_status = pack.SATCOM_status;
                byte[] Servo_status = pack.Servo_status;
                float Motor_rpm = pack.Motor_rpm;
            };
            LoopBackDemoChannel.instance.OnEKF_EXTReceive += (src, ph, pack) =>
            {
                ulong timestamp = pack.timestamp;
                float Windspeed = pack.Windspeed;
                float WindDir = pack.WindDir;
                float WindZ = pack.WindZ;
                float Airspeed = pack.Airspeed;
                float beta = pack.beta;
                float alpha = pack.alpha;
            };
            LoopBackDemoChannel.instance.OnASL_OBCTRLReceive += (src, ph, pack) =>
            {
                ulong timestamp = pack.timestamp;
                float uElev = pack.uElev;
                float uThrot = pack.uThrot;
                float uThrot2 = pack.uThrot2;
                float uAilL = pack.uAilL;
                float uAilR = pack.uAilR;
                float uRud = pack.uRud;
                byte obctrl_status = pack.obctrl_status;
            };
            LoopBackDemoChannel.instance.OnSENS_ATMOSReceive += (src, ph, pack) =>
            {
                float TempAmbient = pack.TempAmbient;
                float Humidity = pack.Humidity;
            };
            LoopBackDemoChannel.instance.OnSENS_BATMONReceive += (src, ph, pack) =>
            {
                float temperature = pack.temperature;
                ushort voltage = pack.voltage;
                short current = pack.current;
                byte SoC = pack.SoC;
                ushort batterystatus = pack.batterystatus;
                ushort serialnumber = pack.serialnumber;
                ushort hostfetcontrol = pack.hostfetcontrol;
                ushort cellvoltage1 = pack.cellvoltage1;
                ushort cellvoltage2 = pack.cellvoltage2;
                ushort cellvoltage3 = pack.cellvoltage3;
                ushort cellvoltage4 = pack.cellvoltage4;
                ushort cellvoltage5 = pack.cellvoltage5;
                ushort cellvoltage6 = pack.cellvoltage6;
            };
            LoopBackDemoChannel.instance.OnFW_SOARING_DATAReceive += (src, ph, pack) =>
            {
                ulong timestamp = pack.timestamp;
                ulong timestampModeChanged = pack.timestampModeChanged;
                float xW = pack.xW;
                float xR = pack.xR;
                float xLat = pack.xLat;
                float xLon = pack.xLon;
                float VarW = pack.VarW;
                float VarR = pack.VarR;
                float VarLat = pack.VarLat;
                float VarLon = pack.VarLon;
                float LoiterRadius = pack.LoiterRadius;
                float LoiterDirection = pack.LoiterDirection;
                float DistToSoarPoint = pack.DistToSoarPoint;
                float vSinkExp = pack.vSinkExp;
                float z1_LocalUpdraftSpeed = pack.z1_LocalUpdraftSpeed;
                float z2_DeltaRoll = pack.z2_DeltaRoll;
                float z1_exp = pack.z1_exp;
                float z2_exp = pack.z2_exp;
                float ThermalGSNorth = pack.ThermalGSNorth;
                float ThermalGSEast = pack.ThermalGSEast;
                float TSE_dot = pack.TSE_dot;
                float DebugVar1 = pack.DebugVar1;
                float DebugVar2 = pack.DebugVar2;
                byte ControlMode = pack.ControlMode;
                byte valid = pack.valid;
            };
            LoopBackDemoChannel.instance.OnSENSORPOD_STATUSReceive += (src, ph, pack) =>
            {
                ulong timestamp = pack.timestamp;
                byte visensor_rate_1 = pack.visensor_rate_1;
                byte visensor_rate_2 = pack.visensor_rate_2;
                byte visensor_rate_3 = pack.visensor_rate_3;
                byte visensor_rate_4 = pack.visensor_rate_4;
                byte recording_nodes_count = pack.recording_nodes_count;
                byte cpu_temp = pack.cpu_temp;
                ushort free_space = pack.free_space;
            };
            LoopBackDemoChannel.instance.OnSENS_POWER_BOARDReceive += (src, ph, pack) =>
            {
                ulong timestamp = pack.timestamp;
                byte pwr_brd_status = pack.pwr_brd_status;
                byte pwr_brd_led_status = pack.pwr_brd_led_status;
                float pwr_brd_system_volt = pack.pwr_brd_system_volt;
                float pwr_brd_servo_volt = pack.pwr_brd_servo_volt;
                float pwr_brd_mot_l_amp = pack.pwr_brd_mot_l_amp;
                float pwr_brd_mot_r_amp = pack.pwr_brd_mot_r_amp;
                float pwr_brd_servo_1_amp = pack.pwr_brd_servo_1_amp;
                float pwr_brd_servo_2_amp = pack.pwr_brd_servo_2_amp;
                float pwr_brd_servo_3_amp = pack.pwr_brd_servo_3_amp;
                float pwr_brd_servo_4_amp = pack.pwr_brd_servo_4_amp;
                float pwr_brd_aux_amp = pack.pwr_brd_aux_amp;
            };
            LoopBackDemoChannel.instance.OnESTIMATOR_STATUSReceive += (src, ph, pack) =>
            {
                ulong time_usec = pack.time_usec;
                ESTIMATOR_STATUS_FLAGS flags = pack.flags;
                float vel_ratio = pack.vel_ratio;
                float pos_horiz_ratio = pack.pos_horiz_ratio;
                float pos_vert_ratio = pack.pos_vert_ratio;
                float mag_ratio = pack.mag_ratio;
                float hagl_ratio = pack.hagl_ratio;
                float tas_ratio = pack.tas_ratio;
                float pos_horiz_accuracy = pack.pos_horiz_accuracy;
                float pos_vert_accuracy = pack.pos_vert_accuracy;
            };
            LoopBackDemoChannel.instance.OnWIND_COVReceive += (src, ph, pack) =>
            {
                ulong time_usec = pack.time_usec;
                float wind_x = pack.wind_x;
                float wind_y = pack.wind_y;
                float wind_z = pack.wind_z;
                float var_horiz = pack.var_horiz;
                float var_vert = pack.var_vert;
                float wind_alt = pack.wind_alt;
                float horiz_accuracy = pack.horiz_accuracy;
                float vert_accuracy = pack.vert_accuracy;
            };
            LoopBackDemoChannel.instance.OnGPS_INPUTReceive += (src, ph, pack) =>
            {
                ulong time_usec = pack.time_usec;
                byte gps_id = pack.gps_id;
                GPS_INPUT_IGNORE_FLAGS ignore_flags = pack.ignore_flags;
                uint time_week_ms = pack.time_week_ms;
                ushort time_week = pack.time_week;
                byte fix_type = pack.fix_type;
                int lat = pack.lat;
                int lon = pack.lon;
                float alt = pack.alt;
                float hdop = pack.hdop;
                float vdop = pack.vdop;
                float vn = pack.vn;
                float ve = pack.ve;
                float vd = pack.vd;
                float speed_accuracy = pack.speed_accuracy;
                float horiz_accuracy = pack.horiz_accuracy;
                float vert_accuracy = pack.vert_accuracy;
                byte satellites_visible = pack.satellites_visible;
            };
            LoopBackDemoChannel.instance.OnGPS_RTCM_DATAReceive += (src, ph, pack) =>
            {
                byte flags = pack.flags;
                byte len = pack.len;
                byte[] data_ = pack.data_;
            };
            LoopBackDemoChannel.instance.OnHIGH_LATENCYReceive += (src, ph, pack) =>
            {
                MAV_MODE_FLAG base_mode = pack.base_mode;
                uint custom_mode = pack.custom_mode;
                MAV_LANDED_STATE landed_state = pack.landed_state;
                short roll = pack.roll;
                short pitch = pack.pitch;
                ushort heading = pack.heading;
                sbyte throttle = pack.throttle;
                short heading_sp = pack.heading_sp;
                int latitude = pack.latitude;
                int longitude = pack.longitude;
                short altitude_amsl = pack.altitude_amsl;
                short altitude_sp = pack.altitude_sp;
                byte airspeed = pack.airspeed;
                byte airspeed_sp = pack.airspeed_sp;
                byte groundspeed = pack.groundspeed;
                sbyte climb_rate = pack.climb_rate;
                byte gps_nsat = pack.gps_nsat;
                GPS_FIX_TYPE gps_fix_type = pack.gps_fix_type;
                byte battery_remaining = pack.battery_remaining;
                sbyte temperature = pack.temperature;
                sbyte temperature_air = pack.temperature_air;
                byte failsafe = pack.failsafe;
                byte wp_num = pack.wp_num;
                ushort wp_distance = pack.wp_distance;
            };
            LoopBackDemoChannel.instance.OnVIBRATIONReceive += (src, ph, pack) =>
            {
                ulong time_usec = pack.time_usec;
                float vibration_x = pack.vibration_x;
                float vibration_y = pack.vibration_y;
                float vibration_z = pack.vibration_z;
                uint clipping_0 = pack.clipping_0;
                uint clipping_1 = pack.clipping_1;
                uint clipping_2 = pack.clipping_2;
            };
            LoopBackDemoChannel.instance.OnHOME_POSITIONReceive += (src, ph, pack) =>
            {
                int latitude = pack.latitude;
                int longitude = pack.longitude;
                int altitude = pack.altitude;
                float x = pack.x;
                float y = pack.y;
                float z = pack.z;
                float[] q = pack.q;
                float approach_x = pack.approach_x;
                float approach_y = pack.approach_y;
                float approach_z = pack.approach_z;
                ulong time_usec = pack.time_usec_TRY(ph);
            };
            LoopBackDemoChannel.instance.OnSET_HOME_POSITIONReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                int latitude = pack.latitude;
                int longitude = pack.longitude;
                int altitude = pack.altitude;
                float x = pack.x;
                float y = pack.y;
                float z = pack.z;
                float[] q = pack.q;
                float approach_x = pack.approach_x;
                float approach_y = pack.approach_y;
                float approach_z = pack.approach_z;
                ulong time_usec = pack.time_usec_TRY(ph);
            };
            LoopBackDemoChannel.instance.OnMESSAGE_INTERVALReceive += (src, ph, pack) =>
            {
                ushort message_id = pack.message_id;
                int interval_us = pack.interval_us;
            };
            LoopBackDemoChannel.instance.OnEXTENDED_SYS_STATEReceive += (src, ph, pack) =>
            {
                MAV_VTOL_STATE vtol_state = pack.vtol_state;
                MAV_LANDED_STATE landed_state = pack.landed_state;
            };
            LoopBackDemoChannel.instance.OnADSB_VEHICLEReceive += (src, ph, pack) =>
            {
                uint ICAO_address = pack.ICAO_address;
                int lat = pack.lat;
                int lon = pack.lon;
                ADSB_ALTITUDE_TYPE altitude_type = pack.altitude_type;
                int altitude = pack.altitude;
                ushort heading = pack.heading;
                ushort hor_velocity = pack.hor_velocity;
                short ver_velocity = pack.ver_velocity;
                string callsign = pack.callsign_TRY(ph);
                ADSB_EMITTER_TYPE emitter_type = pack.emitter_type;
                byte tslc = pack.tslc;
                ADSB_FLAGS flags = pack.flags;
                ushort squawk = pack.squawk;
            };
            LoopBackDemoChannel.instance.OnCOLLISIONReceive += (src, ph, pack) =>
            {
                MAV_COLLISION_SRC src_ = pack.src_;
                uint id = pack.id;
                MAV_COLLISION_ACTION action = pack.action;
                MAV_COLLISION_THREAT_LEVEL threat_level = pack.threat_level;
                float time_to_minimum_delta = pack.time_to_minimum_delta;
                float altitude_minimum_delta = pack.altitude_minimum_delta;
                float horizontal_minimum_delta = pack.horizontal_minimum_delta;
            };
            LoopBackDemoChannel.instance.OnV2_EXTENSIONReceive += (src, ph, pack) =>
            {
                byte target_network = pack.target_network;
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                ushort message_type = pack.message_type;
                byte[] payload = pack.payload;
            };
            LoopBackDemoChannel.instance.OnMEMORY_VECTReceive += (src, ph, pack) =>
            {
                ushort address = pack.address;
                byte ver = pack.ver;
                byte type = pack.type;
                sbyte[] value = pack.value;
            };
            LoopBackDemoChannel.instance.OnDEBUG_VECTReceive += (src, ph, pack) =>
            {
                string name = pack.name_TRY(ph);
                ulong time_usec = pack.time_usec;
                float x = pack.x;
                float y = pack.y;
                float z = pack.z;
            };
            LoopBackDemoChannel.instance.OnNAMED_VALUE_FLOATReceive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                string name = pack.name_TRY(ph);
                float value = pack.value;
            };
            LoopBackDemoChannel.instance.OnNAMED_VALUE_INTReceive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                string name = pack.name_TRY(ph);
                int value = pack.value;
            };
            LoopBackDemoChannel.instance.OnSTATUSTEXTReceive += (src, ph, pack) =>
            {
                MAV_SEVERITY severity = pack.severity;
                string text = pack.text_TRY(ph);
            };
            LoopBackDemoChannel.instance.OnDEBUGReceive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                byte ind = pack.ind;
                float value = pack.value;
            };
            LoopBackDemoChannel.instance.OnSETUP_SIGNINGReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                byte[] secret_key = pack.secret_key;
                ulong initial_timestamp = pack.initial_timestamp;
            };
            LoopBackDemoChannel.instance.OnBUTTON_CHANGEReceive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                uint last_change_ms = pack.last_change_ms;
                byte state = pack.state;
            };
            LoopBackDemoChannel.instance.OnPLAY_TUNEReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                string tune = pack.tune_TRY(ph);
            };
            LoopBackDemoChannel.instance.OnCAMERA_INFORMATIONReceive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                byte[] vendor_name = pack.vendor_name;
                byte[] model_name = pack.model_name;
                uint firmware_version = pack.firmware_version;
                float focal_length = pack.focal_length;
                float sensor_size_h = pack.sensor_size_h;
                float sensor_size_v = pack.sensor_size_v;
                ushort resolution_h = pack.resolution_h;
                ushort resolution_v = pack.resolution_v;
                byte lens_id = pack.lens_id;
                CAMERA_CAP_FLAGS flags = pack.flags;
                ushort cam_definition_version = pack.cam_definition_version;
                string cam_definition_uri = pack.cam_definition_uri_TRY(ph);
            };
            LoopBackDemoChannel.instance.OnCAMERA_SETTINGSReceive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                CAMERA_MODE mode_id = pack.mode_id;
            };
            LoopBackDemoChannel.instance.OnSTORAGE_INFORMATIONReceive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                byte storage_id = pack.storage_id;
                byte storage_count = pack.storage_count;
                byte status = pack.status;
                float total_capacity = pack.total_capacity;
                float used_capacity = pack.used_capacity;
                float available_capacity = pack.available_capacity;
                float read_speed = pack.read_speed;
                float write_speed = pack.write_speed;
            };
            LoopBackDemoChannel.instance.OnCAMERA_CAPTURE_STATUSReceive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                byte image_status = pack.image_status;
                byte video_status = pack.video_status;
                float image_interval = pack.image_interval;
                uint recording_time_ms = pack.recording_time_ms;
                float available_capacity = pack.available_capacity;
            };
            LoopBackDemoChannel.instance.OnCAMERA_IMAGE_CAPTUREDReceive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                ulong time_utc = pack.time_utc;
                byte camera_id = pack.camera_id;
                int lat = pack.lat;
                int lon = pack.lon;
                int alt = pack.alt;
                int relative_alt = pack.relative_alt;
                float[] q = pack.q;
                int image_index = pack.image_index;
                sbyte capture_result = pack.capture_result;
                string file_url = pack.file_url_TRY(ph);
            };
            LoopBackDemoChannel.instance.OnFLIGHT_INFORMATIONReceive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                ulong arming_time_utc = pack.arming_time_utc;
                ulong takeoff_time_utc = pack.takeoff_time_utc;
                ulong flight_uuid = pack.flight_uuid;
            };
            LoopBackDemoChannel.instance.OnMOUNT_ORIENTATIONReceive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                float roll = pack.roll;
                float pitch = pack.pitch;
                float yaw = pack.yaw;
            };
            LoopBackDemoChannel.instance.OnLOGGING_DATAReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                ushort sequence = pack.sequence;
                byte length = pack.length;
                byte first_message_offset = pack.first_message_offset;
                byte[] data_ = pack.data_;
            };
            LoopBackDemoChannel.instance.OnLOGGING_DATA_ACKEDReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                ushort sequence = pack.sequence;
                byte length = pack.length;
                byte first_message_offset = pack.first_message_offset;
                byte[] data_ = pack.data_;
            };
            LoopBackDemoChannel.instance.OnLOGGING_ACKReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                ushort sequence = pack.sequence;
            };
            LoopBackDemoChannel.instance.OnVIDEO_STREAM_INFORMATIONReceive += (src, ph, pack) =>
            {
                byte camera_id = pack.camera_id;
                byte status = pack.status;
                float framerate = pack.framerate;
                ushort resolution_h = pack.resolution_h;
                ushort resolution_v = pack.resolution_v;
                uint bitrate = pack.bitrate;
                ushort rotation = pack.rotation;
                string uri = pack.uri_TRY(ph);
            };
            LoopBackDemoChannel.instance.OnSET_VIDEO_STREAM_SETTINGSReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                byte camera_id = pack.camera_id;
                float framerate = pack.framerate;
                ushort resolution_h = pack.resolution_h;
                ushort resolution_v = pack.resolution_v;
                uint bitrate = pack.bitrate;
                ushort rotation = pack.rotation;
                string uri = pack.uri_TRY(ph);
            };
            LoopBackDemoChannel.instance.OnWIFI_CONFIG_APReceive += (src, ph, pack) =>
            {
                string ssid = pack.ssid_TRY(ph);
                string password = pack.password_TRY(ph);
            };
            LoopBackDemoChannel.instance.OnPROTOCOL_VERSIONReceive += (src, ph, pack) =>
            {
                ushort version = pack.version;
                ushort min_version = pack.min_version;
                ushort max_version = pack.max_version;
                byte[] spec_version_hash = pack.spec_version_hash;
                byte[] library_version_hash = pack.library_version_hash;
            };
            LoopBackDemoChannel.instance.OnUAVCAN_NODE_STATUSReceive += (src, ph, pack) =>
            {
                ulong time_usec = pack.time_usec;
                uint uptime_sec = pack.uptime_sec;
                UAVCAN_NODE_HEALTH health = pack.health;
                UAVCAN_NODE_MODE mode = pack.mode;
                byte sub_mode = pack.sub_mode;
                ushort vendor_specific_status_code = pack.vendor_specific_status_code;
            };
            LoopBackDemoChannel.instance.OnUAVCAN_NODE_INFOReceive += (src, ph, pack) =>
            {
                ulong time_usec = pack.time_usec;
                uint uptime_sec = pack.uptime_sec;
                string name = pack.name_TRY(ph);
                byte hw_version_major = pack.hw_version_major;
                byte hw_version_minor = pack.hw_version_minor;
                byte[] hw_unique_id = pack.hw_unique_id;
                byte sw_version_major = pack.sw_version_major;
                byte sw_version_minor = pack.sw_version_minor;
                uint sw_vcs_commit = pack.sw_vcs_commit;
            };
            LoopBackDemoChannel.instance.OnPARAM_EXT_REQUEST_READReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                string param_id = pack.param_id_TRY(ph);
                short param_index = pack.param_index;
            };
            LoopBackDemoChannel.instance.OnPARAM_EXT_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
            };
            LoopBackDemoChannel.instance.OnPARAM_EXT_VALUEReceive += (src, ph, pack) =>
            {
                string param_id = pack.param_id_TRY(ph);
                string param_value = pack.param_value_TRY(ph);
                MAV_PARAM_EXT_TYPE param_type = pack.param_type;
                ushort param_count = pack.param_count;
                ushort param_index = pack.param_index;
            };
            LoopBackDemoChannel.instance.OnPARAM_EXT_SETReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                string param_id = pack.param_id_TRY(ph);
                string param_value = pack.param_value_TRY(ph);
                MAV_PARAM_EXT_TYPE param_type = pack.param_type;
            };
            LoopBackDemoChannel.instance.OnPARAM_EXT_ACKReceive += (src, ph, pack) =>
            {
                string param_id = pack.param_id_TRY(ph);
                string param_value = pack.param_value_TRY(ph);
                MAV_PARAM_EXT_TYPE param_type = pack.param_type;
                PARAM_ACK param_result = pack.param_result;
            };
            LoopBackDemoChannel.instance.OnOBSTACLE_DISTANCEReceive += (src, ph, pack) =>
            {
                ulong time_usec = pack.time_usec;
                MAV_DISTANCE_SENSOR sensor_type = pack.sensor_type;
                ushort[] distances = pack.distances;
                byte increment = pack.increment;
                ushort min_distance = pack.min_distance;
                ushort max_distance = pack.max_distance;
            };
            HEARTBEAT p0 = LoopBackDemoChannel.new_HEARTBEAT();
            PH.setPack(p0);
            p0.type = (MAV_TYPE)MAV_TYPE.MAV_TYPE_ROCKET;
            p0.autopilot = (MAV_AUTOPILOT)MAV_AUTOPILOT.MAV_AUTOPILOT_AUTOQUAD;
            p0.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED;
            p0.custom_mode = (uint)4197306963U;
            p0.system_status = (MAV_STATE)MAV_STATE.MAV_STATE_EMERGENCY;
            p0.mavlink_version = (byte)(byte)143;
            LoopBackDemoChannel.instance.send(p0); //===============================
            SYS_STATUS p1 = LoopBackDemoChannel.new_SYS_STATUS();
            PH.setPack(p1);
            p1.onboard_control_sensors_present = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION;
            p1.onboard_control_sensors_enabled = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH;
            p1.onboard_control_sensors_health = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING;
            p1.load = (ushort)(ushort)44060;
            p1.voltage_battery = (ushort)(ushort)2730;
            p1.current_battery = (short)(short)16778;
            p1.battery_remaining = (sbyte)(sbyte) - 104;
            p1.drop_rate_comm = (ushort)(ushort)9775;
            p1.errors_comm = (ushort)(ushort)15578;
            p1.errors_count1 = (ushort)(ushort)47595;
            p1.errors_count2 = (ushort)(ushort)43079;
            p1.errors_count3 = (ushort)(ushort)38322;
            p1.errors_count4 = (ushort)(ushort)43668;
            LoopBackDemoChannel.instance.send(p1); //===============================
            SYSTEM_TIME p2 = LoopBackDemoChannel.new_SYSTEM_TIME();
            PH.setPack(p2);
            p2.time_unix_usec = (ulong)5404655760980983060L;
            p2.time_boot_ms = (uint)7286946U;
            LoopBackDemoChannel.instance.send(p2); //===============================
            POSITION_TARGET_LOCAL_NED p3 = LoopBackDemoChannel.new_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.time_boot_ms = (uint)517479378U;
            p3.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
            p3.type_mask = (ushort)(ushort)22993;
            p3.x = (float) -1.8734588E38F;
            p3.y = (float)1.2785225E38F;
            p3.z = (float) -2.3097164E38F;
            p3.vx = (float) -1.5481583E38F;
            p3.vy = (float)2.9643036E38F;
            p3.vz = (float)9.962328E37F;
            p3.afx = (float)3.1690307E37F;
            p3.afy = (float)1.0924599E38F;
            p3.afz = (float)4.9524797E37F;
            p3.yaw = (float)1.1837827E38F;
            p3.yaw_rate = (float) -2.8680934E38F;
            LoopBackDemoChannel.instance.send(p3); //===============================
            PING p4 = LoopBackDemoChannel.new_PING();
            PH.setPack(p4);
            p4.time_usec = (ulong)2603662783618762543L;
            p4.seq = (uint)117075383U;
            p4.target_system = (byte)(byte)13;
            p4.target_component = (byte)(byte)214;
            LoopBackDemoChannel.instance.send(p4); //===============================
            CHANGE_OPERATOR_CONTROL p5 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL();
            PH.setPack(p5);
            p5.target_system = (byte)(byte)215;
            p5.control_request = (byte)(byte)78;
            p5.version = (byte)(byte)178;
            p5.passkey_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p5); //===============================
            CHANGE_OPERATOR_CONTROL_ACK p6 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL_ACK();
            PH.setPack(p6);
            p6.gcs_system_id = (byte)(byte)63;
            p6.control_request = (byte)(byte)223;
            p6.ack = (byte)(byte)63;
            LoopBackDemoChannel.instance.send(p6); //===============================
            AUTH_KEY p7 = LoopBackDemoChannel.new_AUTH_KEY();
            PH.setPack(p7);
            p7.key_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p7); //===============================
            SET_MODE p11 = LoopBackDemoChannel.new_SET_MODE();
            PH.setPack(p11);
            p11.target_system = (byte)(byte)171;
            p11.base_mode = (MAV_MODE)MAV_MODE.MAV_MODE_STABILIZE_ARMED;
            p11.custom_mode = (uint)1492878223U;
            LoopBackDemoChannel.instance.send(p11); //===============================
            PARAM_REQUEST_READ p20 = LoopBackDemoChannel.new_PARAM_REQUEST_READ();
            PH.setPack(p20);
            p20.target_system = (byte)(byte)161;
            p20.target_component = (byte)(byte)201;
            p20.param_id_SET("DEMO", PH);
            p20.param_index = (short)(short)14790;
            LoopBackDemoChannel.instance.send(p20); //===============================
            PARAM_REQUEST_LIST p21 = LoopBackDemoChannel.new_PARAM_REQUEST_LIST();
            PH.setPack(p21);
            p21.target_system = (byte)(byte)87;
            p21.target_component = (byte)(byte)182;
            LoopBackDemoChannel.instance.send(p21); //===============================
            PARAM_VALUE p22 = LoopBackDemoChannel.new_PARAM_VALUE();
            PH.setPack(p22);
            p22.param_id_SET("DEMO", PH);
            p22.param_value = (float) -1.7546876E38F;
            p22.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16;
            p22.param_count = (ushort)(ushort)35913;
            p22.param_index = (ushort)(ushort)31571;
            LoopBackDemoChannel.instance.send(p22); //===============================
            PARAM_SET p23 = LoopBackDemoChannel.new_PARAM_SET();
            PH.setPack(p23);
            p23.target_system = (byte)(byte)189;
            p23.target_component = (byte)(byte)42;
            p23.param_id_SET("DEMO", PH);
            p23.param_value = (float)1.5429956E38F;
            p23.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT16;
            LoopBackDemoChannel.instance.send(p23); //===============================
            GPS_RAW_INT p24 = LoopBackDemoChannel.new_GPS_RAW_INT();
            PH.setPack(p24);
            p24.time_usec = (ulong)1287012387305068532L;
            p24.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX;
            p24.lat = (int) -113502675;
            p24.lon = (int)913022113;
            p24.alt = (int)364731062;
            p24.eph = (ushort)(ushort)54659;
            p24.epv = (ushort)(ushort)51659;
            p24.vel = (ushort)(ushort)42998;
            p24.cog = (ushort)(ushort)29997;
            p24.satellites_visible = (byte)(byte)69;
            p24.alt_ellipsoid_SET((int)1277810369, PH);
            p24.h_acc_SET((uint)535936649U, PH);
            p24.v_acc_SET((uint)501932602U, PH);
            p24.vel_acc_SET((uint)362688044U, PH);
            p24.hdg_acc_SET((uint)1903444458U, PH);
            LoopBackDemoChannel.instance.send(p24); //===============================
            GPS_STATUS p25 = LoopBackDemoChannel.new_GPS_STATUS();
            PH.setPack(p25);
            p25.satellites_visible = (byte)(byte)231;
            p25.satellite_prn_SET(new byte[20], 0);
            p25.satellite_used_SET(new byte[20], 0);
            p25.satellite_elevation_SET(new byte[20], 0);
            p25.satellite_azimuth_SET(new byte[20], 0);
            p25.satellite_snr_SET(new byte[20], 0);
            LoopBackDemoChannel.instance.send(p25); //===============================
            SCALED_IMU p26 = LoopBackDemoChannel.new_SCALED_IMU();
            PH.setPack(p26);
            p26.time_boot_ms = (uint)2412081427U;
            p26.xacc = (short)(short) -7859;
            p26.yacc = (short)(short) -29298;
            p26.zacc = (short)(short) -9053;
            p26.xgyro = (short)(short)31712;
            p26.ygyro = (short)(short)26603;
            p26.zgyro = (short)(short)18480;
            p26.xmag = (short)(short)10888;
            p26.ymag = (short)(short)4863;
            p26.zmag = (short)(short)27176;
            LoopBackDemoChannel.instance.send(p26); //===============================
            RAW_IMU p27 = LoopBackDemoChannel.new_RAW_IMU();
            PH.setPack(p27);
            p27.time_usec = (ulong)816491826419124435L;
            p27.xacc = (short)(short)1691;
            p27.yacc = (short)(short) -19780;
            p27.zacc = (short)(short) -22481;
            p27.xgyro = (short)(short)18579;
            p27.ygyro = (short)(short) -12243;
            p27.zgyro = (short)(short) -5989;
            p27.xmag = (short)(short)25803;
            p27.ymag = (short)(short) -7622;
            p27.zmag = (short)(short) -22909;
            LoopBackDemoChannel.instance.send(p27); //===============================
            RAW_PRESSURE p28 = LoopBackDemoChannel.new_RAW_PRESSURE();
            PH.setPack(p28);
            p28.time_usec = (ulong)6004567713681316906L;
            p28.press_abs = (short)(short)14813;
            p28.press_diff1 = (short)(short)1814;
            p28.press_diff2 = (short)(short)1205;
            p28.temperature = (short)(short) -5179;
            LoopBackDemoChannel.instance.send(p28); //===============================
            SCALED_PRESSURE p29 = LoopBackDemoChannel.new_SCALED_PRESSURE();
            PH.setPack(p29);
            p29.time_boot_ms = (uint)3444285466U;
            p29.press_abs = (float)2.8284153E38F;
            p29.press_diff = (float)7.5933346E37F;
            p29.temperature = (short)(short)30207;
            LoopBackDemoChannel.instance.send(p29); //===============================
            ATTITUDE p30 = LoopBackDemoChannel.new_ATTITUDE();
            PH.setPack(p30);
            p30.time_boot_ms = (uint)1506372281U;
            p30.roll = (float) -5.4557593E37F;
            p30.pitch = (float) -2.8334645E37F;
            p30.yaw = (float)1.4760007E38F;
            p30.rollspeed = (float) -3.517889E37F;
            p30.pitchspeed = (float)4.8913896E37F;
            p30.yawspeed = (float) -1.7584072E38F;
            LoopBackDemoChannel.instance.send(p30); //===============================
            ATTITUDE_QUATERNION p31 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION();
            PH.setPack(p31);
            p31.time_boot_ms = (uint)4195419795U;
            p31.q1 = (float)1.6993944E38F;
            p31.q2 = (float)2.1705022E38F;
            p31.q3 = (float) -6.571797E37F;
            p31.q4 = (float)8.697026E36F;
            p31.rollspeed = (float) -7.605196E37F;
            p31.pitchspeed = (float)2.8233114E38F;
            p31.yawspeed = (float)2.7403377E38F;
            LoopBackDemoChannel.instance.send(p31); //===============================
            LOCAL_POSITION_NED p32 = LoopBackDemoChannel.new_LOCAL_POSITION_NED();
            PH.setPack(p32);
            p32.time_boot_ms = (uint)1864372816U;
            p32.x = (float)2.806355E38F;
            p32.y = (float)2.9572222E38F;
            p32.z = (float)3.5486326E36F;
            p32.vx = (float)2.053457E38F;
            p32.vy = (float) -2.3352842E38F;
            p32.vz = (float)8.295201E37F;
            LoopBackDemoChannel.instance.send(p32); //===============================
            GLOBAL_POSITION_INT p33 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT();
            PH.setPack(p33);
            p33.time_boot_ms = (uint)1286764024U;
            p33.lat = (int) -741557875;
            p33.lon = (int) -501501847;
            p33.alt = (int)64614096;
            p33.relative_alt = (int) -1770976679;
            p33.vx = (short)(short) -22246;
            p33.vy = (short)(short)29903;
            p33.vz = (short)(short) -14424;
            p33.hdg = (ushort)(ushort)44249;
            LoopBackDemoChannel.instance.send(p33); //===============================
            RC_CHANNELS_SCALED p34 = LoopBackDemoChannel.new_RC_CHANNELS_SCALED();
            PH.setPack(p34);
            p34.time_boot_ms = (uint)3873045986U;
            p34.port = (byte)(byte)153;
            p34.chan1_scaled = (short)(short)31749;
            p34.chan2_scaled = (short)(short)31832;
            p34.chan3_scaled = (short)(short)99;
            p34.chan4_scaled = (short)(short)21125;
            p34.chan5_scaled = (short)(short) -25680;
            p34.chan6_scaled = (short)(short) -28109;
            p34.chan7_scaled = (short)(short)15154;
            p34.chan8_scaled = (short)(short)10793;
            p34.rssi = (byte)(byte)18;
            LoopBackDemoChannel.instance.send(p34); //===============================
            RC_CHANNELS_RAW p35 = LoopBackDemoChannel.new_RC_CHANNELS_RAW();
            PH.setPack(p35);
            p35.time_boot_ms = (uint)206950075U;
            p35.port = (byte)(byte)206;
            p35.chan1_raw = (ushort)(ushort)36016;
            p35.chan2_raw = (ushort)(ushort)63529;
            p35.chan3_raw = (ushort)(ushort)45986;
            p35.chan4_raw = (ushort)(ushort)7661;
            p35.chan5_raw = (ushort)(ushort)63108;
            p35.chan6_raw = (ushort)(ushort)52094;
            p35.chan7_raw = (ushort)(ushort)30485;
            p35.chan8_raw = (ushort)(ushort)9147;
            p35.rssi = (byte)(byte)128;
            LoopBackDemoChannel.instance.send(p35); //===============================
            SERVO_OUTPUT_RAW p36 = LoopBackDemoChannel.new_SERVO_OUTPUT_RAW();
            PH.setPack(p36);
            p36.time_usec = (uint)4178009384U;
            p36.port = (byte)(byte)168;
            p36.servo1_raw = (ushort)(ushort)23018;
            p36.servo2_raw = (ushort)(ushort)48921;
            p36.servo3_raw = (ushort)(ushort)27721;
            p36.servo4_raw = (ushort)(ushort)20409;
            p36.servo5_raw = (ushort)(ushort)28813;
            p36.servo6_raw = (ushort)(ushort)44947;
            p36.servo7_raw = (ushort)(ushort)7291;
            p36.servo8_raw = (ushort)(ushort)33167;
            p36.servo9_raw_SET((ushort)(ushort)53928, PH);
            p36.servo10_raw_SET((ushort)(ushort)6322, PH);
            p36.servo11_raw_SET((ushort)(ushort)41080, PH);
            p36.servo12_raw_SET((ushort)(ushort)32330, PH);
            p36.servo13_raw_SET((ushort)(ushort)7252, PH);
            p36.servo14_raw_SET((ushort)(ushort)46177, PH);
            p36.servo15_raw_SET((ushort)(ushort)43741, PH);
            p36.servo16_raw_SET((ushort)(ushort)52229, PH);
            LoopBackDemoChannel.instance.send(p36); //===============================
            MISSION_REQUEST_PARTIAL_LIST p37 = LoopBackDemoChannel.new_MISSION_REQUEST_PARTIAL_LIST();
            PH.setPack(p37);
            p37.target_system = (byte)(byte)82;
            p37.target_component = (byte)(byte)176;
            p37.start_index = (short)(short)19923;
            p37.end_index = (short)(short) -15212;
            p37.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            LoopBackDemoChannel.instance.send(p37); //===============================
            MISSION_WRITE_PARTIAL_LIST p38 = LoopBackDemoChannel.new_MISSION_WRITE_PARTIAL_LIST();
            PH.setPack(p38);
            p38.target_system = (byte)(byte)125;
            p38.target_component = (byte)(byte)84;
            p38.start_index = (short)(short) -24463;
            p38.end_index = (short)(short)28862;
            p38.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            LoopBackDemoChannel.instance.send(p38); //===============================
            MISSION_ITEM p39 = LoopBackDemoChannel.new_MISSION_ITEM();
            PH.setPack(p39);
            p39.target_system = (byte)(byte)217;
            p39.target_component = (byte)(byte)51;
            p39.seq = (ushort)(ushort)38188;
            p39.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p39.command = (MAV_CMD)MAV_CMD.MAV_CMD_NAV_PATHPLANNING;
            p39.current = (byte)(byte)83;
            p39.autocontinue = (byte)(byte)132;
            p39.param1 = (float)3.3815636E38F;
            p39.param2 = (float) -2.4107672E38F;
            p39.param3 = (float)3.2922174E38F;
            p39.param4 = (float) -2.9265947E38F;
            p39.x = (float) -8.99526E37F;
            p39.y = (float) -2.263364E38F;
            p39.z = (float)8.2829664E37F;
            p39.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            LoopBackDemoChannel.instance.send(p39); //===============================
            MISSION_REQUEST p40 = LoopBackDemoChannel.new_MISSION_REQUEST();
            PH.setPack(p40);
            p40.target_system = (byte)(byte)49;
            p40.target_component = (byte)(byte)7;
            p40.seq = (ushort)(ushort)178;
            p40.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            LoopBackDemoChannel.instance.send(p40); //===============================
            MISSION_SET_CURRENT p41 = LoopBackDemoChannel.new_MISSION_SET_CURRENT();
            PH.setPack(p41);
            p41.target_system = (byte)(byte)144;
            p41.target_component = (byte)(byte)195;
            p41.seq = (ushort)(ushort)48017;
            LoopBackDemoChannel.instance.send(p41); //===============================
            MISSION_CURRENT p42 = LoopBackDemoChannel.new_MISSION_CURRENT();
            PH.setPack(p42);
            p42.seq = (ushort)(ushort)3300;
            LoopBackDemoChannel.instance.send(p42); //===============================
            MISSION_REQUEST_LIST p43 = LoopBackDemoChannel.new_MISSION_REQUEST_LIST();
            PH.setPack(p43);
            p43.target_system = (byte)(byte)211;
            p43.target_component = (byte)(byte)22;
            p43.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            LoopBackDemoChannel.instance.send(p43); //===============================
            MISSION_COUNT p44 = LoopBackDemoChannel.new_MISSION_COUNT();
            PH.setPack(p44);
            p44.target_system = (byte)(byte)32;
            p44.target_component = (byte)(byte)183;
            p44.count = (ushort)(ushort)63916;
            p44.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            LoopBackDemoChannel.instance.send(p44); //===============================
            MISSION_CLEAR_ALL p45 = LoopBackDemoChannel.new_MISSION_CLEAR_ALL();
            PH.setPack(p45);
            p45.target_system = (byte)(byte)229;
            p45.target_component = (byte)(byte)189;
            p45.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            LoopBackDemoChannel.instance.send(p45); //===============================
            MISSION_ITEM_REACHED p46 = LoopBackDemoChannel.new_MISSION_ITEM_REACHED();
            PH.setPack(p46);
            p46.seq = (ushort)(ushort)7146;
            LoopBackDemoChannel.instance.send(p46); //===============================
            MISSION_ACK p47 = LoopBackDemoChannel.new_MISSION_ACK();
            PH.setPack(p47);
            p47.target_system = (byte)(byte)193;
            p47.target_component = (byte)(byte)202;
            p47.type = (MAV_MISSION_RESULT)MAV_MISSION_RESULT.MAV_MISSION_UNSUPPORTED;
            p47.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            LoopBackDemoChannel.instance.send(p47); //===============================
            SET_GPS_GLOBAL_ORIGIN p48 = LoopBackDemoChannel.new_SET_GPS_GLOBAL_ORIGIN();
            PH.setPack(p48);
            p48.target_system = (byte)(byte)98;
            p48.latitude = (int)564121897;
            p48.longitude = (int)147181843;
            p48.altitude = (int)1523596929;
            p48.time_usec_SET((ulong)8745131604709069154L, PH);
            LoopBackDemoChannel.instance.send(p48); //===============================
            GPS_GLOBAL_ORIGIN p49 = LoopBackDemoChannel.new_GPS_GLOBAL_ORIGIN();
            PH.setPack(p49);
            p49.latitude = (int)294110182;
            p49.longitude = (int) -1924031750;
            p49.altitude = (int) -1511496809;
            p49.time_usec_SET((ulong)625298082457535870L, PH);
            LoopBackDemoChannel.instance.send(p49); //===============================
            PARAM_MAP_RC p50 = LoopBackDemoChannel.new_PARAM_MAP_RC();
            PH.setPack(p50);
            p50.target_system = (byte)(byte)199;
            p50.target_component = (byte)(byte)178;
            p50.param_id_SET("DEMO", PH);
            p50.param_index = (short)(short)729;
            p50.parameter_rc_channel_index = (byte)(byte)88;
            p50.param_value0 = (float)1.7150932E37F;
            p50.scale = (float) -2.2601058E38F;
            p50.param_value_min = (float)3.3906611E38F;
            p50.param_value_max = (float) -3.5676137E37F;
            LoopBackDemoChannel.instance.send(p50); //===============================
            MISSION_REQUEST_INT p51 = LoopBackDemoChannel.new_MISSION_REQUEST_INT();
            PH.setPack(p51);
            p51.target_system = (byte)(byte)111;
            p51.target_component = (byte)(byte)215;
            p51.seq = (ushort)(ushort)64422;
            p51.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            LoopBackDemoChannel.instance.send(p51); //===============================
            SAFETY_SET_ALLOWED_AREA p54 = LoopBackDemoChannel.new_SAFETY_SET_ALLOWED_AREA();
            PH.setPack(p54);
            p54.target_system = (byte)(byte)150;
            p54.target_component = (byte)(byte)196;
            p54.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p54.p1x = (float) -9.737749E37F;
            p54.p1y = (float) -7.5286687E37F;
            p54.p1z = (float) -1.1570296E38F;
            p54.p2x = (float) -3.0659172E38F;
            p54.p2y = (float) -1.9132888E38F;
            p54.p2z = (float)3.3111527E38F;
            LoopBackDemoChannel.instance.send(p54); //===============================
            SAFETY_ALLOWED_AREA p55 = LoopBackDemoChannel.new_SAFETY_ALLOWED_AREA();
            PH.setPack(p55);
            p55.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_ENU;
            p55.p1x = (float)3.2987157E38F;
            p55.p1y = (float)3.226451E38F;
            p55.p1z = (float) -2.9226358E38F;
            p55.p2x = (float)3.3192855E38F;
            p55.p2y = (float)2.8474027E38F;
            p55.p2z = (float)1.0317042E38F;
            LoopBackDemoChannel.instance.send(p55); //===============================
            ATTITUDE_QUATERNION_COV p61 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION_COV();
            PH.setPack(p61);
            p61.time_usec = (ulong)355139884635496279L;
            p61.q_SET(new float[4], 0);
            p61.rollspeed = (float) -3.2915381E38F;
            p61.pitchspeed = (float) -2.7920426E38F;
            p61.yawspeed = (float)1.8072728E38F;
            p61.covariance_SET(new float[9], 0);
            LoopBackDemoChannel.instance.send(p61); //===============================
            NAV_CONTROLLER_OUTPUT p62 = LoopBackDemoChannel.new_NAV_CONTROLLER_OUTPUT();
            PH.setPack(p62);
            p62.nav_roll = (float) -5.2433345E37F;
            p62.nav_pitch = (float) -4.682271E37F;
            p62.nav_bearing = (short)(short)10394;
            p62.target_bearing = (short)(short)26783;
            p62.wp_dist = (ushort)(ushort)41292;
            p62.alt_error = (float)1.6317736E38F;
            p62.aspd_error = (float) -2.999018E38F;
            p62.xtrack_error = (float)3.0178875E38F;
            LoopBackDemoChannel.instance.send(p62); //===============================
            GLOBAL_POSITION_INT_COV p63 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT_COV();
            PH.setPack(p63);
            p63.time_usec = (ulong)3213488020137612355L;
            p63.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION;
            p63.lat = (int)534960725;
            p63.lon = (int)772805987;
            p63.alt = (int) -1993473350;
            p63.relative_alt = (int)1289908361;
            p63.vx = (float) -1.1142895E37F;
            p63.vy = (float)1.9364917E38F;
            p63.vz = (float)2.6695184E38F;
            p63.covariance_SET(new float[36], 0);
            LoopBackDemoChannel.instance.send(p63); //===============================
            LOCAL_POSITION_NED_COV p64 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_COV();
            PH.setPack(p64);
            p64.time_usec = (ulong)4005955423935840083L;
            p64.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS;
            p64.x = (float) -5.7168887E37F;
            p64.y = (float) -8.0997864E37F;
            p64.z = (float)2.4191793E38F;
            p64.vx = (float)1.2975607E38F;
            p64.vy = (float)2.6402917E38F;
            p64.vz = (float)1.3173785E38F;
            p64.ax = (float)2.418242E38F;
            p64.ay = (float) -1.3663731E38F;
            p64.az = (float) -1.3566033E38F;
            p64.covariance_SET(new float[45], 0);
            LoopBackDemoChannel.instance.send(p64); //===============================
            RC_CHANNELS p65 = LoopBackDemoChannel.new_RC_CHANNELS();
            PH.setPack(p65);
            p65.time_boot_ms = (uint)1762590258U;
            p65.chancount = (byte)(byte)136;
            p65.chan1_raw = (ushort)(ushort)2112;
            p65.chan2_raw = (ushort)(ushort)41191;
            p65.chan3_raw = (ushort)(ushort)24545;
            p65.chan4_raw = (ushort)(ushort)29146;
            p65.chan5_raw = (ushort)(ushort)33177;
            p65.chan6_raw = (ushort)(ushort)10674;
            p65.chan7_raw = (ushort)(ushort)23143;
            p65.chan8_raw = (ushort)(ushort)4218;
            p65.chan9_raw = (ushort)(ushort)9588;
            p65.chan10_raw = (ushort)(ushort)19780;
            p65.chan11_raw = (ushort)(ushort)10535;
            p65.chan12_raw = (ushort)(ushort)40329;
            p65.chan13_raw = (ushort)(ushort)6203;
            p65.chan14_raw = (ushort)(ushort)33265;
            p65.chan15_raw = (ushort)(ushort)1261;
            p65.chan16_raw = (ushort)(ushort)6450;
            p65.chan17_raw = (ushort)(ushort)18416;
            p65.chan18_raw = (ushort)(ushort)47678;
            p65.rssi = (byte)(byte)48;
            LoopBackDemoChannel.instance.send(p65); //===============================
            REQUEST_DATA_STREAM p66 = LoopBackDemoChannel.new_REQUEST_DATA_STREAM();
            PH.setPack(p66);
            p66.target_system = (byte)(byte)121;
            p66.target_component = (byte)(byte)235;
            p66.req_stream_id = (byte)(byte)201;
            p66.req_message_rate = (ushort)(ushort)13370;
            p66.start_stop = (byte)(byte)196;
            LoopBackDemoChannel.instance.send(p66); //===============================
            DATA_STREAM p67 = LoopBackDemoChannel.new_DATA_STREAM();
            PH.setPack(p67);
            p67.stream_id = (byte)(byte)111;
            p67.message_rate = (ushort)(ushort)62798;
            p67.on_off = (byte)(byte)66;
            LoopBackDemoChannel.instance.send(p67); //===============================
            MANUAL_CONTROL p69 = LoopBackDemoChannel.new_MANUAL_CONTROL();
            PH.setPack(p69);
            p69.target = (byte)(byte)59;
            p69.x = (short)(short)19222;
            p69.y = (short)(short)4000;
            p69.z = (short)(short) -6554;
            p69.r = (short)(short)24738;
            p69.buttons = (ushort)(ushort)37330;
            LoopBackDemoChannel.instance.send(p69); //===============================
            RC_CHANNELS_OVERRIDE p70 = LoopBackDemoChannel.new_RC_CHANNELS_OVERRIDE();
            PH.setPack(p70);
            p70.target_system = (byte)(byte)138;
            p70.target_component = (byte)(byte)157;
            p70.chan1_raw = (ushort)(ushort)31003;
            p70.chan2_raw = (ushort)(ushort)47294;
            p70.chan3_raw = (ushort)(ushort)31120;
            p70.chan4_raw = (ushort)(ushort)51655;
            p70.chan5_raw = (ushort)(ushort)60658;
            p70.chan6_raw = (ushort)(ushort)37394;
            p70.chan7_raw = (ushort)(ushort)34184;
            p70.chan8_raw = (ushort)(ushort)51148;
            LoopBackDemoChannel.instance.send(p70); //===============================
            MISSION_ITEM_INT p73 = LoopBackDemoChannel.new_MISSION_ITEM_INT();
            PH.setPack(p73);
            p73.target_system = (byte)(byte)251;
            p73.target_component = (byte)(byte)22;
            p73.seq = (ushort)(ushort)59725;
            p73.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT;
            p73.command = (MAV_CMD)MAV_CMD.MAV_CMD_DO_RALLY_LAND;
            p73.current = (byte)(byte)0;
            p73.autocontinue = (byte)(byte)205;
            p73.param1 = (float)2.1562201E38F;
            p73.param2 = (float)5.0839376E37F;
            p73.param3 = (float)1.4769556E38F;
            p73.param4 = (float) -2.0862614E38F;
            p73.x = (int)387222923;
            p73.y = (int) -842619959;
            p73.z = (float)2.8261222E38F;
            p73.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            LoopBackDemoChannel.instance.send(p73); //===============================
            VFR_HUD p74 = LoopBackDemoChannel.new_VFR_HUD();
            PH.setPack(p74);
            p74.airspeed = (float)7.007559E37F;
            p74.groundspeed = (float)3.1665462E38F;
            p74.heading = (short)(short) -30913;
            p74.throttle = (ushort)(ushort)11396;
            p74.alt = (float)2.7151502E38F;
            p74.climb = (float)2.980745E38F;
            LoopBackDemoChannel.instance.send(p74); //===============================
            COMMAND_INT p75 = LoopBackDemoChannel.new_COMMAND_INT();
            PH.setPack(p75);
            p75.target_system = (byte)(byte)175;
            p75.target_component = (byte)(byte)61;
            p75.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
            p75.command = (MAV_CMD)MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION;
            p75.current = (byte)(byte)218;
            p75.autocontinue = (byte)(byte)209;
            p75.param1 = (float)4.306846E37F;
            p75.param2 = (float) -5.7699166E37F;
            p75.param3 = (float) -3.2649946E38F;
            p75.param4 = (float) -2.3196546E37F;
            p75.x = (int)857982957;
            p75.y = (int) -1810696586;
            p75.z = (float) -2.7259627E38F;
            LoopBackDemoChannel.instance.send(p75); //===============================
            COMMAND_LONG p76 = LoopBackDemoChannel.new_COMMAND_LONG();
            PH.setPack(p76);
            p76.target_system = (byte)(byte)154;
            p76.target_component = (byte)(byte)186;
            p76.command = (MAV_CMD)MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
            p76.confirmation = (byte)(byte)229;
            p76.param1 = (float) -2.7004213E38F;
            p76.param2 = (float)2.859666E38F;
            p76.param3 = (float)6.6619643E37F;
            p76.param4 = (float)2.935545E38F;
            p76.param5 = (float) -1.8514625E38F;
            p76.param6 = (float)3.0639622E37F;
            p76.param7 = (float)8.0916516E37F;
            LoopBackDemoChannel.instance.send(p76); //===============================
            COMMAND_ACK p77 = LoopBackDemoChannel.new_COMMAND_ACK();
            PH.setPack(p77);
            p77.command = (MAV_CMD)MAV_CMD.MAV_CMD_START_RX_PAIR;
            p77.result = (MAV_RESULT)MAV_RESULT.MAV_RESULT_IN_PROGRESS;
            p77.progress_SET((byte)(byte)205, PH);
            p77.result_param2_SET((int) -1858491477, PH);
            p77.target_system_SET((byte)(byte)163, PH);
            p77.target_component_SET((byte)(byte)77, PH);
            LoopBackDemoChannel.instance.send(p77); //===============================
            MANUAL_SETPOINT p81 = LoopBackDemoChannel.new_MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.time_boot_ms = (uint)729893102U;
            p81.roll = (float)2.354413E38F;
            p81.pitch = (float) -3.347202E37F;
            p81.yaw = (float)3.116868E38F;
            p81.thrust = (float)3.1559995E38F;
            p81.mode_switch = (byte)(byte)148;
            p81.manual_override_switch = (byte)(byte)72;
            LoopBackDemoChannel.instance.send(p81); //===============================
            SET_ATTITUDE_TARGET p82 = LoopBackDemoChannel.new_SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.time_boot_ms = (uint)4092470943U;
            p82.target_system = (byte)(byte)103;
            p82.target_component = (byte)(byte)59;
            p82.type_mask = (byte)(byte)17;
            p82.q_SET(new float[4], 0);
            p82.body_roll_rate = (float) -2.1246866E37F;
            p82.body_pitch_rate = (float) -1.7267145E38F;
            p82.body_yaw_rate = (float)2.3389504E38F;
            p82.thrust = (float)7.457977E36F;
            LoopBackDemoChannel.instance.send(p82); //===============================
            ATTITUDE_TARGET p83 = LoopBackDemoChannel.new_ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.time_boot_ms = (uint)3260791585U;
            p83.type_mask = (byte)(byte)32;
            p83.q_SET(new float[4], 0);
            p83.body_roll_rate = (float)2.5255667E38F;
            p83.body_pitch_rate = (float)2.898145E38F;
            p83.body_yaw_rate = (float) -2.6729928E38F;
            p83.thrust = (float) -2.1606372E38F;
            LoopBackDemoChannel.instance.send(p83); //===============================
            SET_POSITION_TARGET_LOCAL_NED p84 = LoopBackDemoChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.time_boot_ms = (uint)2893488781U;
            p84.target_system = (byte)(byte)179;
            p84.target_component = (byte)(byte)56;
            p84.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
            p84.type_mask = (ushort)(ushort)22134;
            p84.x = (float)7.2061627E37F;
            p84.y = (float) -7.2271063E37F;
            p84.z = (float)2.46692E38F;
            p84.vx = (float)8.1552953E37F;
            p84.vy = (float)3.2864523E38F;
            p84.vz = (float)2.0040288E38F;
            p84.afx = (float) -2.2444433E38F;
            p84.afy = (float) -2.9361439E38F;
            p84.afz = (float)2.471066E38F;
            p84.yaw = (float) -2.6337776E38F;
            p84.yaw_rate = (float)2.8558527E38F;
            LoopBackDemoChannel.instance.send(p84); //===============================
            SET_POSITION_TARGET_GLOBAL_INT p86 = LoopBackDemoChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.time_boot_ms = (uint)3797521867U;
            p86.target_system = (byte)(byte)8;
            p86.target_component = (byte)(byte)146;
            p86.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_MISSION;
            p86.type_mask = (ushort)(ushort)30452;
            p86.lat_int = (int) -862609952;
            p86.lon_int = (int) -2069514538;
            p86.alt = (float)1.0253318E38F;
            p86.vx = (float)1.7315456E37F;
            p86.vy = (float) -2.0730022E38F;
            p86.vz = (float) -1.9321423E38F;
            p86.afx = (float) -1.6122866E37F;
            p86.afy = (float)3.0395138E38F;
            p86.afz = (float)1.0001976E38F;
            p86.yaw = (float)9.298519E37F;
            p86.yaw_rate = (float)2.7577994E38F;
            LoopBackDemoChannel.instance.send(p86); //===============================
            POSITION_TARGET_GLOBAL_INT p87 = LoopBackDemoChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.time_boot_ms = (uint)55577400U;
            p87.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_NED;
            p87.type_mask = (ushort)(ushort)1909;
            p87.lat_int = (int)192468547;
            p87.lon_int = (int) -599565647;
            p87.alt = (float)2.3556755E38F;
            p87.vx = (float)2.274014E38F;
            p87.vy = (float)2.051412E38F;
            p87.vz = (float) -3.2873697E38F;
            p87.afx = (float) -2.169922E38F;
            p87.afy = (float) -2.4494157E38F;
            p87.afz = (float) -3.0512167E38F;
            p87.yaw = (float)2.107771E38F;
            p87.yaw_rate = (float) -2.9370878E38F;
            LoopBackDemoChannel.instance.send(p87); //===============================
            LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.time_boot_ms = (uint)4120099830U;
            p89.x = (float)9.754888E37F;
            p89.y = (float)2.4153837E38F;
            p89.z = (float) -1.8284647E38F;
            p89.roll = (float) -7.340979E37F;
            p89.pitch = (float) -3.3603933E38F;
            p89.yaw = (float) -6.4538556E37F;
            LoopBackDemoChannel.instance.send(p89); //===============================
            HIL_STATE p90 = LoopBackDemoChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.time_usec = (ulong)4286317552030935154L;
            p90.roll = (float) -2.3267088E38F;
            p90.pitch = (float) -1.5053428E38F;
            p90.yaw = (float)1.5938912E38F;
            p90.rollspeed = (float) -3.3836817E37F;
            p90.pitchspeed = (float) -2.0373774E38F;
            p90.yawspeed = (float) -8.320368E37F;
            p90.lat = (int)1665400083;
            p90.lon = (int) -196673103;
            p90.alt = (int)1268282797;
            p90.vx = (short)(short)3142;
            p90.vy = (short)(short)24906;
            p90.vz = (short)(short)19570;
            p90.xacc = (short)(short)29224;
            p90.yacc = (short)(short) -182;
            p90.zacc = (short)(short) -25192;
            LoopBackDemoChannel.instance.send(p90); //===============================
            HIL_CONTROLS p91 = LoopBackDemoChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.time_usec = (ulong)4022549157929459604L;
            p91.roll_ailerons = (float) -3.1034295E38F;
            p91.pitch_elevator = (float)5.293176E37F;
            p91.yaw_rudder = (float) -3.0775912E38F;
            p91.throttle = (float)2.1130064E38F;
            p91.aux1 = (float)3.527339E37F;
            p91.aux2 = (float) -5.4825376E37F;
            p91.aux3 = (float) -8.160801E37F;
            p91.aux4 = (float)5.5157333E37F;
            p91.mode = (MAV_MODE)MAV_MODE.MAV_MODE_AUTO_DISARMED;
            p91.nav_mode = (byte)(byte)1;
            LoopBackDemoChannel.instance.send(p91); //===============================
            HIL_RC_INPUTS_RAW p92 = LoopBackDemoChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.time_usec = (ulong)8784575708933410873L;
            p92.chan1_raw = (ushort)(ushort)15406;
            p92.chan2_raw = (ushort)(ushort)43718;
            p92.chan3_raw = (ushort)(ushort)55788;
            p92.chan4_raw = (ushort)(ushort)648;
            p92.chan5_raw = (ushort)(ushort)9094;
            p92.chan6_raw = (ushort)(ushort)22473;
            p92.chan7_raw = (ushort)(ushort)48550;
            p92.chan8_raw = (ushort)(ushort)62419;
            p92.chan9_raw = (ushort)(ushort)41460;
            p92.chan10_raw = (ushort)(ushort)6592;
            p92.chan11_raw = (ushort)(ushort)24021;
            p92.chan12_raw = (ushort)(ushort)41583;
            p92.rssi = (byte)(byte)44;
            LoopBackDemoChannel.instance.send(p92); //===============================
            HIL_ACTUATOR_CONTROLS p93 = LoopBackDemoChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.time_usec = (ulong)5885501630648933689L;
            p93.controls_SET(new float[16], 0);
            p93.mode = (MAV_MODE)MAV_MODE.MAV_MODE_STABILIZE_ARMED;
            p93.flags = (ulong)4709691352087712417L;
            LoopBackDemoChannel.instance.send(p93); //===============================
            OPTICAL_FLOW p100 = LoopBackDemoChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.time_usec = (ulong)6766672121654246635L;
            p100.sensor_id = (byte)(byte)146;
            p100.flow_x = (short)(short)29631;
            p100.flow_y = (short)(short) -5947;
            p100.flow_comp_m_x = (float)1.0696925E38F;
            p100.flow_comp_m_y = (float)5.293031E37F;
            p100.quality = (byte)(byte)117;
            p100.ground_distance = (float) -3.345066E38F;
            p100.flow_rate_x_SET((float)2.2410633E38F, PH);
            p100.flow_rate_y_SET((float)1.105037E38F, PH);
            LoopBackDemoChannel.instance.send(p100); //===============================
            GLOBAL_VISION_POSITION_ESTIMATE p101 = LoopBackDemoChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.usec = (ulong)8564975479906361858L;
            p101.x = (float)3.2993653E38F;
            p101.y = (float) -1.6682621E38F;
            p101.z = (float)1.104105E38F;
            p101.roll = (float)2.2685725E38F;
            p101.pitch = (float)1.1807668E38F;
            p101.yaw = (float)2.9372537E37F;
            LoopBackDemoChannel.instance.send(p101); //===============================
            VISION_POSITION_ESTIMATE p102 = LoopBackDemoChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.usec = (ulong)843506136772423991L;
            p102.x = (float) -3.3729264E38F;
            p102.y = (float)2.2258314E38F;
            p102.z = (float)2.1189792E38F;
            p102.roll = (float)2.4530445E38F;
            p102.pitch = (float)1.0455843E38F;
            p102.yaw = (float) -2.700946E37F;
            LoopBackDemoChannel.instance.send(p102); //===============================
            VISION_SPEED_ESTIMATE p103 = LoopBackDemoChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.usec = (ulong)6339188650465302048L;
            p103.x = (float) -6.6275314E37F;
            p103.y = (float) -1.3092603E38F;
            p103.z = (float)2.3661427E38F;
            LoopBackDemoChannel.instance.send(p103); //===============================
            VICON_POSITION_ESTIMATE p104 = LoopBackDemoChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.usec = (ulong)5162240077243070607L;
            p104.x = (float)5.859967E37F;
            p104.y = (float)2.3925124E38F;
            p104.z = (float) -1.0807948E38F;
            p104.roll = (float) -2.4595736E38F;
            p104.pitch = (float)2.0585157E38F;
            p104.yaw = (float)2.4695513E38F;
            LoopBackDemoChannel.instance.send(p104); //===============================
            HIGHRES_IMU p105 = LoopBackDemoChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.time_usec = (ulong)3906054200422632291L;
            p105.xacc = (float)4.2021101E37F;
            p105.yacc = (float)1.6474964E38F;
            p105.zacc = (float)3.1334057E38F;
            p105.xgyro = (float) -2.7428E38F;
            p105.ygyro = (float) -2.288224E38F;
            p105.zgyro = (float)1.4672184E38F;
            p105.xmag = (float) -7.3528785E37F;
            p105.ymag = (float) -9.19158E37F;
            p105.zmag = (float)1.0941147E38F;
            p105.abs_pressure = (float) -1.7670467E38F;
            p105.diff_pressure = (float) -2.6598074E38F;
            p105.pressure_alt = (float)7.7812937E37F;
            p105.temperature = (float) -1.7316121E38F;
            p105.fields_updated = (ushort)(ushort)17935;
            LoopBackDemoChannel.instance.send(p105); //===============================
            OPTICAL_FLOW_RAD p106 = LoopBackDemoChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.time_usec = (ulong)3097229852554219409L;
            p106.sensor_id = (byte)(byte)6;
            p106.integration_time_us = (uint)1460689924U;
            p106.integrated_x = (float)2.485055E38F;
            p106.integrated_y = (float)2.3176095E38F;
            p106.integrated_xgyro = (float) -9.993794E37F;
            p106.integrated_ygyro = (float)7.880242E37F;
            p106.integrated_zgyro = (float) -2.0151162E38F;
            p106.temperature = (short)(short)1057;
            p106.quality = (byte)(byte)83;
            p106.time_delta_distance_us = (uint)3979413029U;
            p106.distance = (float) -2.254352E38F;
            LoopBackDemoChannel.instance.send(p106); //===============================
            HIL_SENSOR p107 = LoopBackDemoChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.time_usec = (ulong)2105229275399060875L;
            p107.xacc = (float)2.7806377E37F;
            p107.yacc = (float) -9.227096E37F;
            p107.zacc = (float) -3.0182134E38F;
            p107.xgyro = (float)2.1638765E38F;
            p107.ygyro = (float)2.052413E38F;
            p107.zgyro = (float)8.991959E37F;
            p107.xmag = (float)1.5275041E38F;
            p107.ymag = (float) -3.3628643E38F;
            p107.zmag = (float) -1.0425454E38F;
            p107.abs_pressure = (float) -2.8297336E38F;
            p107.diff_pressure = (float) -1.2330832E38F;
            p107.pressure_alt = (float)1.1729951E38F;
            p107.temperature = (float) -8.2099756E37F;
            p107.fields_updated = (uint)3648730781U;
            LoopBackDemoChannel.instance.send(p107); //===============================
            SIM_STATE p108 = LoopBackDemoChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.q1 = (float)2.0000597E37F;
            p108.q2 = (float)6.409157E36F;
            p108.q3 = (float)9.972101E37F;
            p108.q4 = (float) -1.7256469E38F;
            p108.roll = (float) -1.1808472E38F;
            p108.pitch = (float) -2.6413186E37F;
            p108.yaw = (float)9.08485E37F;
            p108.xacc = (float) -2.4186394E38F;
            p108.yacc = (float)2.605946E38F;
            p108.zacc = (float) -6.007252E37F;
            p108.xgyro = (float)7.3461686E37F;
            p108.ygyro = (float) -3.1358084E38F;
            p108.zgyro = (float)4.4009695E37F;
            p108.lat = (float)3.095877E38F;
            p108.lon = (float) -1.8591082E38F;
            p108.alt = (float)1.121719E38F;
            p108.std_dev_horz = (float)1.1962507E38F;
            p108.std_dev_vert = (float) -1.2065725E38F;
            p108.vn = (float) -5.146829E37F;
            p108.ve = (float)6.016021E36F;
            p108.vd = (float) -6.521798E37F;
            LoopBackDemoChannel.instance.send(p108); //===============================
            RADIO_STATUS p109 = LoopBackDemoChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.rssi = (byte)(byte)131;
            p109.remrssi = (byte)(byte)91;
            p109.txbuf = (byte)(byte)246;
            p109.noise = (byte)(byte)64;
            p109.remnoise = (byte)(byte)64;
            p109.rxerrors = (ushort)(ushort)30548;
            p109.fixed_ = (ushort)(ushort)7098;
            LoopBackDemoChannel.instance.send(p109); //===============================
            FILE_TRANSFER_PROTOCOL p110 = LoopBackDemoChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_network = (byte)(byte)51;
            p110.target_system = (byte)(byte)164;
            p110.target_component = (byte)(byte)168;
            p110.payload_SET(new byte[251], 0);
            LoopBackDemoChannel.instance.send(p110); //===============================
            TIMESYNC p111 = LoopBackDemoChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.tc1 = (long)4608554666546427180L;
            p111.ts1 = (long)9183398316781460137L;
            LoopBackDemoChannel.instance.send(p111); //===============================
            CAMERA_TRIGGER p112 = LoopBackDemoChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.time_usec = (ulong)3764848189767289240L;
            p112.seq = (uint)2600977132U;
            LoopBackDemoChannel.instance.send(p112); //===============================
            HIL_GPS p113 = LoopBackDemoChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.time_usec = (ulong)5547350229303125260L;
            p113.fix_type = (byte)(byte)168;
            p113.lat = (int)2118664239;
            p113.lon = (int)1697935895;
            p113.alt = (int)1673403615;
            p113.eph = (ushort)(ushort)55289;
            p113.epv = (ushort)(ushort)39381;
            p113.vel = (ushort)(ushort)37377;
            p113.vn = (short)(short)26905;
            p113.ve = (short)(short) -21186;
            p113.vd = (short)(short)26008;
            p113.cog = (ushort)(ushort)51614;
            p113.satellites_visible = (byte)(byte)72;
            LoopBackDemoChannel.instance.send(p113); //===============================
            HIL_OPTICAL_FLOW p114 = LoopBackDemoChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.time_usec = (ulong)5529090462780662780L;
            p114.sensor_id = (byte)(byte)241;
            p114.integration_time_us = (uint)3531182795U;
            p114.integrated_x = (float)5.434999E37F;
            p114.integrated_y = (float) -3.3784596E38F;
            p114.integrated_xgyro = (float)7.4580854E37F;
            p114.integrated_ygyro = (float)1.4535701E38F;
            p114.integrated_zgyro = (float) -2.5625047E37F;
            p114.temperature = (short)(short) -15164;
            p114.quality = (byte)(byte)236;
            p114.time_delta_distance_us = (uint)1797865556U;
            p114.distance = (float) -2.8707493E38F;
            LoopBackDemoChannel.instance.send(p114); //===============================
            HIL_STATE_QUATERNION p115 = LoopBackDemoChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.time_usec = (ulong)1798778339797318575L;
            p115.attitude_quaternion_SET(new float[4], 0);
            p115.rollspeed = (float) -1.6514085E38F;
            p115.pitchspeed = (float)1.3352006E38F;
            p115.yawspeed = (float) -1.7306222E38F;
            p115.lat = (int) -83843612;
            p115.lon = (int)1341247192;
            p115.alt = (int) -1853413036;
            p115.vx = (short)(short) -16967;
            p115.vy = (short)(short) -18452;
            p115.vz = (short)(short)11319;
            p115.ind_airspeed = (ushort)(ushort)11277;
            p115.true_airspeed = (ushort)(ushort)41299;
            p115.xacc = (short)(short)11717;
            p115.yacc = (short)(short) -4854;
            p115.zacc = (short)(short)20743;
            LoopBackDemoChannel.instance.send(p115); //===============================
            SCALED_IMU2 p116 = LoopBackDemoChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.time_boot_ms = (uint)3423230533U;
            p116.xacc = (short)(short)26103;
            p116.yacc = (short)(short)12246;
            p116.zacc = (short)(short)4902;
            p116.xgyro = (short)(short) -27609;
            p116.ygyro = (short)(short) -8994;
            p116.zgyro = (short)(short) -9910;
            p116.xmag = (short)(short) -8829;
            p116.ymag = (short)(short) -24747;
            p116.zmag = (short)(short) -13272;
            LoopBackDemoChannel.instance.send(p116); //===============================
            LOG_REQUEST_LIST p117 = LoopBackDemoChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.target_system = (byte)(byte)187;
            p117.target_component = (byte)(byte)65;
            p117.start = (ushort)(ushort)11800;
            p117.end = (ushort)(ushort)61341;
            LoopBackDemoChannel.instance.send(p117); //===============================
            LOG_ENTRY p118 = LoopBackDemoChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.id = (ushort)(ushort)8456;
            p118.num_logs = (ushort)(ushort)13753;
            p118.last_log_num = (ushort)(ushort)59993;
            p118.time_utc = (uint)3621632130U;
            p118.size = (uint)1929061061U;
            LoopBackDemoChannel.instance.send(p118); //===============================
            LOG_REQUEST_DATA p119 = LoopBackDemoChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.target_system = (byte)(byte)119;
            p119.target_component = (byte)(byte)105;
            p119.id = (ushort)(ushort)59189;
            p119.ofs = (uint)1315172359U;
            p119.count = (uint)201034878U;
            LoopBackDemoChannel.instance.send(p119); //===============================
            LOG_DATA p120 = LoopBackDemoChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.id = (ushort)(ushort)17142;
            p120.ofs = (uint)111956805U;
            p120.count = (byte)(byte)56;
            p120.data__SET(new byte[90], 0);
            LoopBackDemoChannel.instance.send(p120); //===============================
            LOG_ERASE p121 = LoopBackDemoChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_system = (byte)(byte)31;
            p121.target_component = (byte)(byte)245;
            LoopBackDemoChannel.instance.send(p121); //===============================
            LOG_REQUEST_END p122 = LoopBackDemoChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_system = (byte)(byte)31;
            p122.target_component = (byte)(byte)97;
            LoopBackDemoChannel.instance.send(p122); //===============================
            GPS_INJECT_DATA p123 = LoopBackDemoChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.target_system = (byte)(byte)146;
            p123.target_component = (byte)(byte)237;
            p123.len = (byte)(byte)39;
            p123.data__SET(new byte[110], 0);
            LoopBackDemoChannel.instance.send(p123); //===============================
            GPS2_RAW p124 = LoopBackDemoChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.time_usec = (ulong)8427492579339346021L;
            p124.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED;
            p124.lat = (int)527950980;
            p124.lon = (int) -751377887;
            p124.alt = (int)1850605552;
            p124.eph = (ushort)(ushort)61469;
            p124.epv = (ushort)(ushort)12372;
            p124.vel = (ushort)(ushort)29210;
            p124.cog = (ushort)(ushort)33133;
            p124.satellites_visible = (byte)(byte)11;
            p124.dgps_numch = (byte)(byte)180;
            p124.dgps_age = (uint)4062503721U;
            LoopBackDemoChannel.instance.send(p124); //===============================
            POWER_STATUS p125 = LoopBackDemoChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vcc = (ushort)(ushort)34965;
            p125.Vservo = (ushort)(ushort)53790;
            p125.flags = (MAV_POWER_STATUS)MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID;
            LoopBackDemoChannel.instance.send(p125); //===============================
            SERIAL_CONTROL p126 = LoopBackDemoChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.device = (SERIAL_CONTROL_DEV)SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1;
            p126.flags = (SERIAL_CONTROL_FLAG)SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE;
            p126.timeout = (ushort)(ushort)22724;
            p126.baudrate = (uint)1208270648U;
            p126.count = (byte)(byte)24;
            p126.data__SET(new byte[70], 0);
            LoopBackDemoChannel.instance.send(p126); //===============================
            GPS_RTK p127 = LoopBackDemoChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.time_last_baseline_ms = (uint)1887336275U;
            p127.rtk_receiver_id = (byte)(byte)81;
            p127.wn = (ushort)(ushort)38848;
            p127.tow = (uint)1869563808U;
            p127.rtk_health = (byte)(byte)177;
            p127.rtk_rate = (byte)(byte)181;
            p127.nsats = (byte)(byte)214;
            p127.baseline_coords_type = (byte)(byte)155;
            p127.baseline_a_mm = (int)1023141178;
            p127.baseline_b_mm = (int)1302212335;
            p127.baseline_c_mm = (int) -548041223;
            p127.accuracy = (uint)1233342009U;
            p127.iar_num_hypotheses = (int) -304775200;
            LoopBackDemoChannel.instance.send(p127); //===============================
            GPS2_RTK p128 = LoopBackDemoChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.time_last_baseline_ms = (uint)1045641680U;
            p128.rtk_receiver_id = (byte)(byte)84;
            p128.wn = (ushort)(ushort)1033;
            p128.tow = (uint)870940372U;
            p128.rtk_health = (byte)(byte)58;
            p128.rtk_rate = (byte)(byte)36;
            p128.nsats = (byte)(byte)158;
            p128.baseline_coords_type = (byte)(byte)116;
            p128.baseline_a_mm = (int) -455916926;
            p128.baseline_b_mm = (int) -1393012812;
            p128.baseline_c_mm = (int)1427615801;
            p128.accuracy = (uint)1336316652U;
            p128.iar_num_hypotheses = (int) -535772487;
            LoopBackDemoChannel.instance.send(p128); //===============================
            SCALED_IMU3 p129 = LoopBackDemoChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.time_boot_ms = (uint)4286351384U;
            p129.xacc = (short)(short) -30626;
            p129.yacc = (short)(short) -28215;
            p129.zacc = (short)(short)26808;
            p129.xgyro = (short)(short)16600;
            p129.ygyro = (short)(short)10941;
            p129.zgyro = (short)(short) -9633;
            p129.xmag = (short)(short) -9460;
            p129.ymag = (short)(short) -17789;
            p129.zmag = (short)(short) -25386;
            LoopBackDemoChannel.instance.send(p129); //===============================
            DATA_TRANSMISSION_HANDSHAKE p130 = LoopBackDemoChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.type = (byte)(byte)106;
            p130.size = (uint)1629854189U;
            p130.width = (ushort)(ushort)42415;
            p130.height = (ushort)(ushort)20748;
            p130.packets = (ushort)(ushort)43425;
            p130.payload = (byte)(byte)217;
            p130.jpg_quality = (byte)(byte)51;
            LoopBackDemoChannel.instance.send(p130); //===============================
            ENCAPSULATED_DATA p131 = LoopBackDemoChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.seqnr = (ushort)(ushort)7509;
            p131.data__SET(new byte[253], 0);
            LoopBackDemoChannel.instance.send(p131); //===============================
            DISTANCE_SENSOR p132 = LoopBackDemoChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.time_boot_ms = (uint)887932240U;
            p132.min_distance = (ushort)(ushort)12009;
            p132.max_distance = (ushort)(ushort)9956;
            p132.current_distance = (ushort)(ushort)35610;
            p132.type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED;
            p132.id = (byte)(byte)38;
            p132.orientation = (MAV_SENSOR_ORIENTATION)MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_135;
            p132.covariance = (byte)(byte)175;
            LoopBackDemoChannel.instance.send(p132); //===============================
            TERRAIN_REQUEST p133 = LoopBackDemoChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.lat = (int)264400455;
            p133.lon = (int) -672194660;
            p133.grid_spacing = (ushort)(ushort)659;
            p133.mask = (ulong)5877257862439611759L;
            LoopBackDemoChannel.instance.send(p133); //===============================
            TERRAIN_DATA p134 = LoopBackDemoChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.lat = (int)1594122959;
            p134.lon = (int)660742608;
            p134.grid_spacing = (ushort)(ushort)48554;
            p134.gridbit = (byte)(byte)158;
            p134.data__SET(new short[16], 0);
            LoopBackDemoChannel.instance.send(p134); //===============================
            TERRAIN_CHECK p135 = LoopBackDemoChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lat = (int)338331614;
            p135.lon = (int)73129884;
            LoopBackDemoChannel.instance.send(p135); //===============================
            TERRAIN_REPORT p136 = LoopBackDemoChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.lat = (int) -1079377368;
            p136.lon = (int)1052726550;
            p136.spacing = (ushort)(ushort)7156;
            p136.terrain_height = (float)2.362841E38F;
            p136.current_height = (float)3.3487382E38F;
            p136.pending = (ushort)(ushort)22487;
            p136.loaded = (ushort)(ushort)15815;
            LoopBackDemoChannel.instance.send(p136); //===============================
            SCALED_PRESSURE2 p137 = LoopBackDemoChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.time_boot_ms = (uint)772620830U;
            p137.press_abs = (float)1.1805521E38F;
            p137.press_diff = (float)2.0872987E38F;
            p137.temperature = (short)(short)28337;
            LoopBackDemoChannel.instance.send(p137); //===============================
            ATT_POS_MOCAP p138 = LoopBackDemoChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.time_usec = (ulong)7538314888707295946L;
            p138.q_SET(new float[4], 0);
            p138.x = (float) -6.495291E37F;
            p138.y = (float)1.2349194E38F;
            p138.z = (float) -1.3586391E38F;
            LoopBackDemoChannel.instance.send(p138); //===============================
            SET_ACTUATOR_CONTROL_TARGET p139 = LoopBackDemoChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.time_usec = (ulong)3026408696200885644L;
            p139.group_mlx = (byte)(byte)214;
            p139.target_system = (byte)(byte)75;
            p139.target_component = (byte)(byte)185;
            p139.controls_SET(new float[8], 0);
            LoopBackDemoChannel.instance.send(p139); //===============================
            ACTUATOR_CONTROL_TARGET p140 = LoopBackDemoChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.time_usec = (ulong)6378740082585318100L;
            p140.group_mlx = (byte)(byte)53;
            p140.controls_SET(new float[8], 0);
            LoopBackDemoChannel.instance.send(p140); //===============================
            ALTITUDE p141 = LoopBackDemoChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.time_usec = (ulong)957483486905074506L;
            p141.altitude_monotonic = (float) -2.5077048E38F;
            p141.altitude_amsl = (float)2.5620436E37F;
            p141.altitude_local = (float) -1.5864257E38F;
            p141.altitude_relative = (float)2.0220952E38F;
            p141.altitude_terrain = (float) -2.5109065E38F;
            p141.bottom_clearance = (float) -3.3989852E38F;
            LoopBackDemoChannel.instance.send(p141); //===============================
            RESOURCE_REQUEST p142 = LoopBackDemoChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.request_id = (byte)(byte)93;
            p142.uri_type = (byte)(byte)63;
            p142.uri_SET(new byte[120], 0);
            p142.transfer_type = (byte)(byte)101;
            p142.storage_SET(new byte[120], 0);
            LoopBackDemoChannel.instance.send(p142); //===============================
            SCALED_PRESSURE3 p143 = LoopBackDemoChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.time_boot_ms = (uint)1355682087U;
            p143.press_abs = (float) -3.1020623E38F;
            p143.press_diff = (float) -2.4667049E38F;
            p143.temperature = (short)(short) -4304;
            LoopBackDemoChannel.instance.send(p143); //===============================
            FOLLOW_TARGET p144 = LoopBackDemoChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.timestamp = (ulong)626609398960653552L;
            p144.est_capabilities = (byte)(byte)227;
            p144.lat = (int) -1782118607;
            p144.lon = (int)2121551433;
            p144.alt = (float)6.5334403E37F;
            p144.vel_SET(new float[3], 0);
            p144.acc_SET(new float[3], 0);
            p144.attitude_q_SET(new float[4], 0);
            p144.rates_SET(new float[3], 0);
            p144.position_cov_SET(new float[3], 0);
            p144.custom_state = (ulong)4827246599395566378L;
            LoopBackDemoChannel.instance.send(p144); //===============================
            CONTROL_SYSTEM_STATE p146 = LoopBackDemoChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.time_usec = (ulong)5507516715218246268L;
            p146.x_acc = (float)3.3964307E38F;
            p146.y_acc = (float)2.606765E37F;
            p146.z_acc = (float) -1.3310653E38F;
            p146.x_vel = (float)1.968432E38F;
            p146.y_vel = (float)2.9071504E38F;
            p146.z_vel = (float) -1.3793893E38F;
            p146.x_pos = (float)2.1193173E38F;
            p146.y_pos = (float) -2.0995089E38F;
            p146.z_pos = (float)6.4185673E37F;
            p146.airspeed = (float)2.125721E38F;
            p146.vel_variance_SET(new float[3], 0);
            p146.pos_variance_SET(new float[3], 0);
            p146.q_SET(new float[4], 0);
            p146.roll_rate = (float)2.8028235E38F;
            p146.pitch_rate = (float) -2.4768493E38F;
            p146.yaw_rate = (float) -2.9286337E38F;
            LoopBackDemoChannel.instance.send(p146); //===============================
            BATTERY_STATUS p147 = LoopBackDemoChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.id = (byte)(byte)243;
            p147.battery_function = (MAV_BATTERY_FUNCTION)MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_AVIONICS;
            p147.type = (MAV_BATTERY_TYPE)MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIPO;
            p147.temperature = (short)(short)13203;
            p147.voltages_SET(new ushort[10], 0);
            p147.current_battery = (short)(short) -31663;
            p147.current_consumed = (int)1799835039;
            p147.energy_consumed = (int) -1656028726;
            p147.battery_remaining = (sbyte)(sbyte)15;
            LoopBackDemoChannel.instance.send(p147); //===============================
            AUTOPILOT_VERSION p148 = LoopBackDemoChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY)MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT;
            p148.flight_sw_version = (uint)3592767164U;
            p148.middleware_sw_version = (uint)1969320979U;
            p148.os_sw_version = (uint)1862006983U;
            p148.board_version = (uint)3387446590U;
            p148.flight_custom_version_SET(new byte[8], 0);
            p148.middleware_custom_version_SET(new byte[8], 0);
            p148.os_custom_version_SET(new byte[8], 0);
            p148.vendor_id = (ushort)(ushort)62842;
            p148.product_id = (ushort)(ushort)64698;
            p148.uid = (ulong)4460439458541827629L;
            p148.uid2_SET(new byte[18], 0, PH);
            LoopBackDemoChannel.instance.send(p148); //===============================
            LANDING_TARGET p149 = LoopBackDemoChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.time_usec = (ulong)4166928929171804682L;
            p149.target_num = (byte)(byte)164;
            p149.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_NED;
            p149.angle_x = (float) -3.3874851E37F;
            p149.angle_y = (float)2.8656157E38F;
            p149.distance = (float)1.7445685E38F;
            p149.size_x = (float) -2.498794E37F;
            p149.size_y = (float) -1.5974441E38F;
            p149.x_SET((float) -2.0536585E38F, PH);
            p149.y_SET((float)1.2152272E38F, PH);
            p149.z_SET((float)2.46467E38F, PH);
            p149.q_SET(new float[4], 0, PH);
            p149.type = (LANDING_TARGET_TYPE)LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER;
            p149.position_valid_SET((byte)(byte)214, PH);
            LoopBackDemoChannel.instance.send(p149); //===============================
            SENS_POWER p201 = LoopBackDemoChannel.new_SENS_POWER();
            PH.setPack(p201);
            p201.adc121_vspb_volt = (float)3.3066646E38F;
            p201.adc121_cspb_amp = (float) -1.2563544E38F;
            p201.adc121_cs1_amp = (float)3.766106E37F;
            p201.adc121_cs2_amp = (float)1.987529E38F;
            LoopBackDemoChannel.instance.send(p201); //===============================
            SENS_MPPT p202 = LoopBackDemoChannel.new_SENS_MPPT();
            PH.setPack(p202);
            p202.mppt_timestamp = (ulong)4594282532226116613L;
            p202.mppt1_volt = (float) -5.068979E37F;
            p202.mppt1_amp = (float)3.3625479E38F;
            p202.mppt1_pwm = (ushort)(ushort)36442;
            p202.mppt1_status = (byte)(byte)199;
            p202.mppt2_volt = (float) -3.2755836E38F;
            p202.mppt2_amp = (float)1.7707876E38F;
            p202.mppt2_pwm = (ushort)(ushort)6805;
            p202.mppt2_status = (byte)(byte)40;
            p202.mppt3_volt = (float) -3.1914457E38F;
            p202.mppt3_amp = (float)1.2792395E38F;
            p202.mppt3_pwm = (ushort)(ushort)25566;
            p202.mppt3_status = (byte)(byte)128;
            LoopBackDemoChannel.instance.send(p202); //===============================
            ASLCTRL_DATA p203 = LoopBackDemoChannel.new_ASLCTRL_DATA();
            PH.setPack(p203);
            p203.timestamp = (ulong)5879953584745793147L;
            p203.aslctrl_mode = (byte)(byte)198;
            p203.h = (float) -1.6687103E38F;
            p203.hRef = (float)1.042488E38F;
            p203.hRef_t = (float)3.0018516E38F;
            p203.PitchAngle = (float) -2.4184528E38F;
            p203.PitchAngleRef = (float) -2.78437E38F;
            p203.q = (float)8.680552E37F;
            p203.qRef = (float) -3.3904212E38F;
            p203.uElev = (float)1.0194697E38F;
            p203.uThrot = (float)2.7457793E38F;
            p203.uThrot2 = (float)2.5775862E38F;
            p203.nZ = (float) -4.5511623E37F;
            p203.AirspeedRef = (float)6.509309E37F;
            p203.SpoilersEngaged = (byte)(byte)45;
            p203.YawAngle = (float) -3.3426914E38F;
            p203.YawAngleRef = (float)1.7989796E38F;
            p203.RollAngle = (float)1.1573601E38F;
            p203.RollAngleRef = (float) -1.4976689E38F;
            p203.p = (float)2.9861031E38F;
            p203.pRef = (float) -3.5023665E37F;
            p203.r = (float)3.1424809E38F;
            p203.rRef = (float) -1.730124E38F;
            p203.uAil = (float) -2.0258154E38F;
            p203.uRud = (float)2.6981274E37F;
            LoopBackDemoChannel.instance.send(p203); //===============================
            ASLCTRL_DEBUG p204 = LoopBackDemoChannel.new_ASLCTRL_DEBUG();
            PH.setPack(p204);
            p204.i32_1 = (uint)521912475U;
            p204.i8_1 = (byte)(byte)27;
            p204.i8_2 = (byte)(byte)204;
            p204.f_1 = (float)2.5790676E37F;
            p204.f_2 = (float) -7.080586E37F;
            p204.f_3 = (float)2.475108E38F;
            p204.f_4 = (float)1.3003813E38F;
            p204.f_5 = (float)3.3650874E38F;
            p204.f_6 = (float)2.0376818E38F;
            p204.f_7 = (float)3.3098102E38F;
            p204.f_8 = (float) -6.385224E37F;
            LoopBackDemoChannel.instance.send(p204); //===============================
            ASLUAV_STATUS p205 = LoopBackDemoChannel.new_ASLUAV_STATUS();
            PH.setPack(p205);
            p205.LED_status = (byte)(byte)25;
            p205.SATCOM_status = (byte)(byte)241;
            p205.Servo_status_SET(new byte[8], 0);
            p205.Motor_rpm = (float)2.0384138E38F;
            LoopBackDemoChannel.instance.send(p205); //===============================
            EKF_EXT p206 = LoopBackDemoChannel.new_EKF_EXT();
            PH.setPack(p206);
            p206.timestamp = (ulong)1572074887649236644L;
            p206.Windspeed = (float) -3.1454553E38F;
            p206.WindDir = (float) -2.7328048E38F;
            p206.WindZ = (float) -1.7825522E38F;
            p206.Airspeed = (float) -3.1633545E38F;
            p206.beta = (float)1.1171536E38F;
            p206.alpha = (float)1.3785131E38F;
            LoopBackDemoChannel.instance.send(p206); //===============================
            ASL_OBCTRL p207 = LoopBackDemoChannel.new_ASL_OBCTRL();
            PH.setPack(p207);
            p207.timestamp = (ulong)1980589338583778552L;
            p207.uElev = (float)1.6457932E38F;
            p207.uThrot = (float)5.0541787E36F;
            p207.uThrot2 = (float)3.152617E38F;
            p207.uAilL = (float)2.6056998E38F;
            p207.uAilR = (float)1.2273423E38F;
            p207.uRud = (float) -2.1243883E37F;
            p207.obctrl_status = (byte)(byte)122;
            LoopBackDemoChannel.instance.send(p207); //===============================
            SENS_ATMOS p208 = LoopBackDemoChannel.new_SENS_ATMOS();
            PH.setPack(p208);
            p208.TempAmbient = (float) -2.392669E38F;
            p208.Humidity = (float) -1.5147241E38F;
            LoopBackDemoChannel.instance.send(p208); //===============================
            SENS_BATMON p209 = LoopBackDemoChannel.new_SENS_BATMON();
            PH.setPack(p209);
            p209.temperature = (float) -3.398042E38F;
            p209.voltage = (ushort)(ushort)50850;
            p209.current = (short)(short) -13416;
            p209.SoC = (byte)(byte)134;
            p209.batterystatus = (ushort)(ushort)56313;
            p209.serialnumber = (ushort)(ushort)29062;
            p209.hostfetcontrol = (ushort)(ushort)56112;
            p209.cellvoltage1 = (ushort)(ushort)42272;
            p209.cellvoltage2 = (ushort)(ushort)52404;
            p209.cellvoltage3 = (ushort)(ushort)22243;
            p209.cellvoltage4 = (ushort)(ushort)60069;
            p209.cellvoltage5 = (ushort)(ushort)38694;
            p209.cellvoltage6 = (ushort)(ushort)7926;
            LoopBackDemoChannel.instance.send(p209); //===============================
            FW_SOARING_DATA p210 = LoopBackDemoChannel.new_FW_SOARING_DATA();
            PH.setPack(p210);
            p210.timestamp = (ulong)1384970158496899663L;
            p210.timestampModeChanged = (ulong)3897866801316277789L;
            p210.xW = (float) -3.2591705E38F;
            p210.xR = (float)1.370844E38F;
            p210.xLat = (float)1.7747849E38F;
            p210.xLon = (float)3.6507027E37F;
            p210.VarW = (float) -3.1530772E38F;
            p210.VarR = (float)1.0956742E38F;
            p210.VarLat = (float)2.6438472E38F;
            p210.VarLon = (float)2.976366E38F;
            p210.LoiterRadius = (float)5.607021E37F;
            p210.LoiterDirection = (float) -2.7750712E37F;
            p210.DistToSoarPoint = (float)1.0042317E38F;
            p210.vSinkExp = (float) -1.8165608E38F;
            p210.z1_LocalUpdraftSpeed = (float)9.709006E37F;
            p210.z2_DeltaRoll = (float)1.4515335E38F;
            p210.z1_exp = (float)1.5502151E37F;
            p210.z2_exp = (float)3.1249117E38F;
            p210.ThermalGSNorth = (float)2.8230557E38F;
            p210.ThermalGSEast = (float) -2.210674E38F;
            p210.TSE_dot = (float)6.155047E37F;
            p210.DebugVar1 = (float)2.767259E38F;
            p210.DebugVar2 = (float)2.964649E38F;
            p210.ControlMode = (byte)(byte)248;
            p210.valid = (byte)(byte)66;
            LoopBackDemoChannel.instance.send(p210); //===============================
            SENSORPOD_STATUS p211 = LoopBackDemoChannel.new_SENSORPOD_STATUS();
            PH.setPack(p211);
            p211.timestamp = (ulong)4151703432890331306L;
            p211.visensor_rate_1 = (byte)(byte)120;
            p211.visensor_rate_2 = (byte)(byte)166;
            p211.visensor_rate_3 = (byte)(byte)186;
            p211.visensor_rate_4 = (byte)(byte)121;
            p211.recording_nodes_count = (byte)(byte)192;
            p211.cpu_temp = (byte)(byte)48;
            p211.free_space = (ushort)(ushort)31054;
            LoopBackDemoChannel.instance.send(p211); //===============================
            SENS_POWER_BOARD p212 = LoopBackDemoChannel.new_SENS_POWER_BOARD();
            PH.setPack(p212);
            p212.timestamp = (ulong)8350280965457624853L;
            p212.pwr_brd_status = (byte)(byte)17;
            p212.pwr_brd_led_status = (byte)(byte)38;
            p212.pwr_brd_system_volt = (float) -1.2012809E38F;
            p212.pwr_brd_servo_volt = (float) -1.5457098E38F;
            p212.pwr_brd_mot_l_amp = (float) -2.1155056E38F;
            p212.pwr_brd_mot_r_amp = (float) -4.9773723E37F;
            p212.pwr_brd_servo_1_amp = (float)2.7525524E38F;
            p212.pwr_brd_servo_2_amp = (float) -2.8140523E38F;
            p212.pwr_brd_servo_3_amp = (float) -1.069259E38F;
            p212.pwr_brd_servo_4_amp = (float) -3.146747E37F;
            p212.pwr_brd_aux_amp = (float) -1.0721204E37F;
            LoopBackDemoChannel.instance.send(p212); //===============================
            ESTIMATOR_STATUS p230 = LoopBackDemoChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.time_usec = (ulong)6670024891441694118L;
            p230.flags = (ESTIMATOR_STATUS_FLAGS)ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS;
            p230.vel_ratio = (float)2.7764883E38F;
            p230.pos_horiz_ratio = (float)8.1378255E37F;
            p230.pos_vert_ratio = (float)7.112408E37F;
            p230.mag_ratio = (float)9.640189E37F;
            p230.hagl_ratio = (float)8.415805E37F;
            p230.tas_ratio = (float)1.4610867E38F;
            p230.pos_horiz_accuracy = (float)2.5583998E38F;
            p230.pos_vert_accuracy = (float)1.3352366E38F;
            LoopBackDemoChannel.instance.send(p230); //===============================
            WIND_COV p231 = LoopBackDemoChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.time_usec = (ulong)2193112104362373319L;
            p231.wind_x = (float) -2.2050022E38F;
            p231.wind_y = (float) -2.205852E38F;
            p231.wind_z = (float) -2.219037E38F;
            p231.var_horiz = (float) -1.4082368E38F;
            p231.var_vert = (float)7.8346304E37F;
            p231.wind_alt = (float) -7.256756E37F;
            p231.horiz_accuracy = (float) -3.334511E38F;
            p231.vert_accuracy = (float)1.7785454E38F;
            LoopBackDemoChannel.instance.send(p231); //===============================
            GPS_INPUT p232 = LoopBackDemoChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.time_usec = (ulong)4170399514860353759L;
            p232.gps_id = (byte)(byte)184;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS)GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY;
            p232.time_week_ms = (uint)789408139U;
            p232.time_week = (ushort)(ushort)26316;
            p232.fix_type = (byte)(byte)216;
            p232.lat = (int) -47906683;
            p232.lon = (int)1266301397;
            p232.alt = (float)1.0619117E38F;
            p232.hdop = (float)2.260275E38F;
            p232.vdop = (float) -6.0995655E37F;
            p232.vn = (float)2.3857697E38F;
            p232.ve = (float) -1.5898185E38F;
            p232.vd = (float)1.7509977E38F;
            p232.speed_accuracy = (float)9.094517E37F;
            p232.horiz_accuracy = (float)3.1706807E38F;
            p232.vert_accuracy = (float) -3.2463394E38F;
            p232.satellites_visible = (byte)(byte)78;
            LoopBackDemoChannel.instance.send(p232); //===============================
            GPS_RTCM_DATA p233 = LoopBackDemoChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.flags = (byte)(byte)232;
            p233.len = (byte)(byte)176;
            p233.data__SET(new byte[180], 0);
            LoopBackDemoChannel.instance.send(p233); //===============================
            HIGH_LATENCY p234 = LoopBackDemoChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED;
            p234.custom_mode = (uint)1617852757U;
            p234.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF;
            p234.roll = (short)(short)4894;
            p234.pitch = (short)(short)6054;
            p234.heading = (ushort)(ushort)24033;
            p234.throttle = (sbyte)(sbyte)32;
            p234.heading_sp = (short)(short)13757;
            p234.latitude = (int)516880572;
            p234.longitude = (int)86033643;
            p234.altitude_amsl = (short)(short)14986;
            p234.altitude_sp = (short)(short)15271;
            p234.airspeed = (byte)(byte)180;
            p234.airspeed_sp = (byte)(byte)117;
            p234.groundspeed = (byte)(byte)90;
            p234.climb_rate = (sbyte)(sbyte) - 61;
            p234.gps_nsat = (byte)(byte)178;
            p234.gps_fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX;
            p234.battery_remaining = (byte)(byte)45;
            p234.temperature = (sbyte)(sbyte) - 101;
            p234.temperature_air = (sbyte)(sbyte) - 98;
            p234.failsafe = (byte)(byte)28;
            p234.wp_num = (byte)(byte)254;
            p234.wp_distance = (ushort)(ushort)40342;
            LoopBackDemoChannel.instance.send(p234); //===============================
            VIBRATION p241 = LoopBackDemoChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.time_usec = (ulong)5177780914586276009L;
            p241.vibration_x = (float)5.4618486E37F;
            p241.vibration_y = (float)1.6466434E38F;
            p241.vibration_z = (float)3.3597722E38F;
            p241.clipping_0 = (uint)2049486446U;
            p241.clipping_1 = (uint)362352497U;
            p241.clipping_2 = (uint)23911430U;
            LoopBackDemoChannel.instance.send(p241); //===============================
            HOME_POSITION p242 = LoopBackDemoChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.latitude = (int)1223031872;
            p242.longitude = (int)459881522;
            p242.altitude = (int)857299221;
            p242.x = (float)2.436086E38F;
            p242.y = (float)4.935933E37F;
            p242.z = (float) -2.800466E38F;
            p242.q_SET(new float[4], 0);
            p242.approach_x = (float)2.0398173E38F;
            p242.approach_y = (float) -3.716744E37F;
            p242.approach_z = (float)3.2070402E38F;
            p242.time_usec_SET((ulong)3395846842802910138L, PH);
            LoopBackDemoChannel.instance.send(p242); //===============================
            SET_HOME_POSITION p243 = LoopBackDemoChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.target_system = (byte)(byte)81;
            p243.latitude = (int)32714843;
            p243.longitude = (int) -2137880427;
            p243.altitude = (int) -1639385728;
            p243.x = (float)1.5393027E38F;
            p243.y = (float) -1.5944119E38F;
            p243.z = (float)2.1595572E38F;
            p243.q_SET(new float[4], 0);
            p243.approach_x = (float)3.9078263E37F;
            p243.approach_y = (float)3.3649964E38F;
            p243.approach_z = (float) -1.6743066E38F;
            p243.time_usec_SET((ulong)8278561142818808244L, PH);
            LoopBackDemoChannel.instance.send(p243); //===============================
            MESSAGE_INTERVAL p244 = LoopBackDemoChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.message_id = (ushort)(ushort)52299;
            p244.interval_us = (int) -346015566;
            LoopBackDemoChannel.instance.send(p244); //===============================
            EXTENDED_SYS_STATE p245 = LoopBackDemoChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.vtol_state = (MAV_VTOL_STATE)MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_FW;
            p245.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND;
            LoopBackDemoChannel.instance.send(p245); //===============================
            ADSB_VEHICLE p246 = LoopBackDemoChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.ICAO_address = (uint)4282768976U;
            p246.lat = (int) -1744926936;
            p246.lon = (int) -1508871792;
            p246.altitude_type = (ADSB_ALTITUDE_TYPE)ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC;
            p246.altitude = (int) -1820207823;
            p246.heading = (ushort)(ushort)9574;
            p246.hor_velocity = (ushort)(ushort)60137;
            p246.ver_velocity = (short)(short)12846;
            p246.callsign_SET("DEMO", PH);
            p246.emitter_type = (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_SMALL;
            p246.tslc = (byte)(byte)169;
            p246.flags = (ADSB_FLAGS)ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK;
            p246.squawk = (ushort)(ushort)63234;
            LoopBackDemoChannel.instance.send(p246); //===============================
            COLLISION p247 = LoopBackDemoChannel.new_COLLISION();
            PH.setPack(p247);
            p247.src_ = (MAV_COLLISION_SRC)MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB;
            p247.id = (uint)3949240164U;
            p247.action = (MAV_COLLISION_ACTION)MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_HOVER;
            p247.threat_level = (MAV_COLLISION_THREAT_LEVEL)MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE;
            p247.time_to_minimum_delta = (float)1.668669E38F;
            p247.altitude_minimum_delta = (float)2.2800408E38F;
            p247.horizontal_minimum_delta = (float)2.9335802E38F;
            LoopBackDemoChannel.instance.send(p247); //===============================
            V2_EXTENSION p248 = LoopBackDemoChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.target_network = (byte)(byte)127;
            p248.target_system = (byte)(byte)3;
            p248.target_component = (byte)(byte)24;
            p248.message_type = (ushort)(ushort)19203;
            p248.payload_SET(new byte[249], 0);
            LoopBackDemoChannel.instance.send(p248); //===============================
            MEMORY_VECT p249 = LoopBackDemoChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.address = (ushort)(ushort)20639;
            p249.ver = (byte)(byte)119;
            p249.type = (byte)(byte)49;
            p249.value_SET(new sbyte[32], 0);
            LoopBackDemoChannel.instance.send(p249); //===============================
            DEBUG_VECT p250 = LoopBackDemoChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.name_SET("DEMO", PH);
            p250.time_usec = (ulong)1942993846806466928L;
            p250.x = (float)6.765318E37F;
            p250.y = (float) -3.2295498E38F;
            p250.z = (float)2.445188E37F;
            LoopBackDemoChannel.instance.send(p250); //===============================
            NAMED_VALUE_FLOAT p251 = LoopBackDemoChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.time_boot_ms = (uint)2716416475U;
            p251.name_SET("DEMO", PH);
            p251.value = (float) -1.3850044E38F;
            LoopBackDemoChannel.instance.send(p251); //===============================
            NAMED_VALUE_INT p252 = LoopBackDemoChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.time_boot_ms = (uint)3666434353U;
            p252.name_SET("DEMO", PH);
            p252.value = (int)484825610;
            LoopBackDemoChannel.instance.send(p252); //===============================
            STATUSTEXT p253 = LoopBackDemoChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.severity = (MAV_SEVERITY)MAV_SEVERITY.MAV_SEVERITY_NOTICE;
            p253.text_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p253); //===============================
            DEBUG p254 = LoopBackDemoChannel.new_DEBUG();
            PH.setPack(p254);
            p254.time_boot_ms = (uint)2338656866U;
            p254.ind = (byte)(byte)207;
            p254.value = (float)2.5626916E38F;
            LoopBackDemoChannel.instance.send(p254); //===============================
            SETUP_SIGNING p256 = LoopBackDemoChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.target_system = (byte)(byte)47;
            p256.target_component = (byte)(byte)3;
            p256.secret_key_SET(new byte[32], 0);
            p256.initial_timestamp = (ulong)1854447729857135042L;
            LoopBackDemoChannel.instance.send(p256); //===============================
            BUTTON_CHANGE p257 = LoopBackDemoChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.time_boot_ms = (uint)3723431624U;
            p257.last_change_ms = (uint)2821722729U;
            p257.state = (byte)(byte)191;
            LoopBackDemoChannel.instance.send(p257); //===============================
            PLAY_TUNE p258 = LoopBackDemoChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_system = (byte)(byte)48;
            p258.target_component = (byte)(byte)59;
            p258.tune_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p258); //===============================
            CAMERA_INFORMATION p259 = LoopBackDemoChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.time_boot_ms = (uint)4108767029U;
            p259.vendor_name_SET(new byte[32], 0);
            p259.model_name_SET(new byte[32], 0);
            p259.firmware_version = (uint)2616851195U;
            p259.focal_length = (float)2.881211E38F;
            p259.sensor_size_h = (float) -2.7887666E38F;
            p259.sensor_size_v = (float) -4.942633E37F;
            p259.resolution_h = (ushort)(ushort)19719;
            p259.resolution_v = (ushort)(ushort)16359;
            p259.lens_id = (byte)(byte)124;
            p259.flags = (CAMERA_CAP_FLAGS)CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE;
            p259.cam_definition_version = (ushort)(ushort)15956;
            p259.cam_definition_uri_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p259); //===============================
            CAMERA_SETTINGS p260 = LoopBackDemoChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.time_boot_ms = (uint)4021867983U;
            p260.mode_id = (CAMERA_MODE)CAMERA_MODE.CAMERA_MODE_IMAGE;
            LoopBackDemoChannel.instance.send(p260); //===============================
            STORAGE_INFORMATION p261 = LoopBackDemoChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.time_boot_ms = (uint)3648319953U;
            p261.storage_id = (byte)(byte)249;
            p261.storage_count = (byte)(byte)103;
            p261.status = (byte)(byte)164;
            p261.total_capacity = (float)7.1896244E37F;
            p261.used_capacity = (float)3.1730818E38F;
            p261.available_capacity = (float)8.553134E37F;
            p261.read_speed = (float)3.0113816E37F;
            p261.write_speed = (float) -2.4416282E37F;
            LoopBackDemoChannel.instance.send(p261); //===============================
            CAMERA_CAPTURE_STATUS p262 = LoopBackDemoChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.time_boot_ms = (uint)1207693261U;
            p262.image_status = (byte)(byte)174;
            p262.video_status = (byte)(byte)255;
            p262.image_interval = (float)2.4402544E38F;
            p262.recording_time_ms = (uint)3807261550U;
            p262.available_capacity = (float) -1.4268435E38F;
            LoopBackDemoChannel.instance.send(p262); //===============================
            CAMERA_IMAGE_CAPTURED p263 = LoopBackDemoChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.time_boot_ms = (uint)884024608U;
            p263.time_utc = (ulong)2526558226976661063L;
            p263.camera_id = (byte)(byte)41;
            p263.lat = (int)644102995;
            p263.lon = (int) -1068640999;
            p263.alt = (int)570776974;
            p263.relative_alt = (int)568929367;
            p263.q_SET(new float[4], 0);
            p263.image_index = (int)1736822802;
            p263.capture_result = (sbyte)(sbyte)105;
            p263.file_url_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p263); //===============================
            FLIGHT_INFORMATION p264 = LoopBackDemoChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.time_boot_ms = (uint)2191383023U;
            p264.arming_time_utc = (ulong)4812448995503996748L;
            p264.takeoff_time_utc = (ulong)5839225802781523619L;
            p264.flight_uuid = (ulong)3889997323087279574L;
            LoopBackDemoChannel.instance.send(p264); //===============================
            MOUNT_ORIENTATION p265 = LoopBackDemoChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.time_boot_ms = (uint)1627776141U;
            p265.roll = (float) -1.320495E38F;
            p265.pitch = (float) -1.5812789E38F;
            p265.yaw = (float)2.3027451E38F;
            LoopBackDemoChannel.instance.send(p265); //===============================
            LOGGING_DATA p266 = LoopBackDemoChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.target_system = (byte)(byte)205;
            p266.target_component = (byte)(byte)249;
            p266.sequence = (ushort)(ushort)20370;
            p266.length = (byte)(byte)158;
            p266.first_message_offset = (byte)(byte)91;
            p266.data__SET(new byte[249], 0);
            LoopBackDemoChannel.instance.send(p266); //===============================
            LOGGING_DATA_ACKED p267 = LoopBackDemoChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.target_system = (byte)(byte)254;
            p267.target_component = (byte)(byte)19;
            p267.sequence = (ushort)(ushort)33658;
            p267.length = (byte)(byte)4;
            p267.first_message_offset = (byte)(byte)212;
            p267.data__SET(new byte[249], 0);
            LoopBackDemoChannel.instance.send(p267); //===============================
            LOGGING_ACK p268 = LoopBackDemoChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_system = (byte)(byte)254;
            p268.target_component = (byte)(byte)224;
            p268.sequence = (ushort)(ushort)46272;
            LoopBackDemoChannel.instance.send(p268); //===============================
            VIDEO_STREAM_INFORMATION p269 = LoopBackDemoChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.camera_id = (byte)(byte)49;
            p269.status = (byte)(byte)95;
            p269.framerate = (float)3.0781735E38F;
            p269.resolution_h = (ushort)(ushort)29956;
            p269.resolution_v = (ushort)(ushort)15417;
            p269.bitrate = (uint)3144999136U;
            p269.rotation = (ushort)(ushort)57090;
            p269.uri_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p269); //===============================
            SET_VIDEO_STREAM_SETTINGS p270 = LoopBackDemoChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.target_system = (byte)(byte)182;
            p270.target_component = (byte)(byte)58;
            p270.camera_id = (byte)(byte)179;
            p270.framerate = (float)9.738389E37F;
            p270.resolution_h = (ushort)(ushort)49477;
            p270.resolution_v = (ushort)(ushort)30918;
            p270.bitrate = (uint)1909396263U;
            p270.rotation = (ushort)(ushort)723;
            p270.uri_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p270); //===============================
            WIFI_CONFIG_AP p299 = LoopBackDemoChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.ssid_SET("DEMO", PH);
            p299.password_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p299); //===============================
            PROTOCOL_VERSION p300 = LoopBackDemoChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.version = (ushort)(ushort)64469;
            p300.min_version = (ushort)(ushort)7865;
            p300.max_version = (ushort)(ushort)44562;
            p300.spec_version_hash_SET(new byte[8], 0);
            p300.library_version_hash_SET(new byte[8], 0);
            LoopBackDemoChannel.instance.send(p300); //===============================
            UAVCAN_NODE_STATUS p310 = LoopBackDemoChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.time_usec = (ulong)3973317629146161898L;
            p310.uptime_sec = (uint)3756051801U;
            p310.health = (UAVCAN_NODE_HEALTH)UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL;
            p310.mode = (UAVCAN_NODE_MODE)UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION;
            p310.sub_mode = (byte)(byte)228;
            p310.vendor_specific_status_code = (ushort)(ushort)34537;
            LoopBackDemoChannel.instance.send(p310); //===============================
            UAVCAN_NODE_INFO p311 = LoopBackDemoChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.time_usec = (ulong)4847735502812842356L;
            p311.uptime_sec = (uint)1356383097U;
            p311.name_SET("DEMO", PH);
            p311.hw_version_major = (byte)(byte)161;
            p311.hw_version_minor = (byte)(byte)141;
            p311.hw_unique_id_SET(new byte[16], 0);
            p311.sw_version_major = (byte)(byte)200;
            p311.sw_version_minor = (byte)(byte)61;
            p311.sw_vcs_commit = (uint)3302010422U;
            LoopBackDemoChannel.instance.send(p311); //===============================
            PARAM_EXT_REQUEST_READ p320 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.target_system = (byte)(byte)148;
            p320.target_component = (byte)(byte)63;
            p320.param_id_SET("DEMO", PH);
            p320.param_index = (short)(short) -9400;
            LoopBackDemoChannel.instance.send(p320); //===============================
            PARAM_EXT_REQUEST_LIST p321 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_system = (byte)(byte)63;
            p321.target_component = (byte)(byte)199;
            LoopBackDemoChannel.instance.send(p321); //===============================
            PARAM_EXT_VALUE p322 = LoopBackDemoChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_id_SET("DEMO", PH);
            p322.param_value_SET("DEMO", PH);
            p322.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT8;
            p322.param_count = (ushort)(ushort)3535;
            p322.param_index = (ushort)(ushort)11956;
            LoopBackDemoChannel.instance.send(p322); //===============================
            PARAM_EXT_SET p323 = LoopBackDemoChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.target_system = (byte)(byte)215;
            p323.target_component = (byte)(byte)252;
            p323.param_id_SET("DEMO", PH);
            p323.param_value_SET("DEMO", PH);
            p323.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16;
            LoopBackDemoChannel.instance.send(p323); //===============================
            PARAM_EXT_ACK p324 = LoopBackDemoChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_id_SET("DEMO", PH);
            p324.param_value_SET("DEMO", PH);
            p324.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64;
            p324.param_result = (PARAM_ACK)PARAM_ACK.PARAM_ACK_ACCEPTED;
            LoopBackDemoChannel.instance.send(p324); //===============================
            OBSTACLE_DISTANCE p330 = LoopBackDemoChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.time_usec = (ulong)3727065816809840608L;
            p330.sensor_type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR;
            p330.distances_SET(new ushort[72], 0);
            p330.increment = (byte)(byte)47;
            p330.min_distance = (ushort)(ushort)26998;
            p330.max_distance = (ushort)(ushort)43909;
            LoopBackDemoChannel.instance.send(p330); //===============================
        }
    }
}

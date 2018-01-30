
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
            LoopBackDemoChannel.instance.OnSENSOR_OFFSETSReceive += (src, ph, pack) =>
            {
                short mag_ofs_x = pack.mag_ofs_x;
                short mag_ofs_y = pack.mag_ofs_y;
                short mag_ofs_z = pack.mag_ofs_z;
                float mag_declination = pack.mag_declination;
                int raw_press = pack.raw_press;
                int raw_temp = pack.raw_temp;
                float gyro_cal_x = pack.gyro_cal_x;
                float gyro_cal_y = pack.gyro_cal_y;
                float gyro_cal_z = pack.gyro_cal_z;
                float accel_cal_x = pack.accel_cal_x;
                float accel_cal_y = pack.accel_cal_y;
                float accel_cal_z = pack.accel_cal_z;
            };
            LoopBackDemoChannel.instance.OnSET_MAG_OFFSETSReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                short mag_ofs_x = pack.mag_ofs_x;
                short mag_ofs_y = pack.mag_ofs_y;
                short mag_ofs_z = pack.mag_ofs_z;
            };
            LoopBackDemoChannel.instance.OnMEMINFOReceive += (src, ph, pack) =>
            {
                ushort brkval = pack.brkval;
                ushort freemem = pack.freemem;
                uint freemem32 = pack.freemem32_TRY(ph);
            };
            LoopBackDemoChannel.instance.OnAP_ADCReceive += (src, ph, pack) =>
            {
                ushort adc1 = pack.adc1;
                ushort adc2 = pack.adc2;
                ushort adc3 = pack.adc3;
                ushort adc4 = pack.adc4;
                ushort adc5 = pack.adc5;
                ushort adc6 = pack.adc6;
            };
            LoopBackDemoChannel.instance.OnDIGICAM_CONFIGUREReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                byte mode = pack.mode;
                ushort shutter_speed = pack.shutter_speed;
                byte aperture = pack.aperture;
                byte iso = pack.iso;
                byte exposure_type = pack.exposure_type;
                byte command_id = pack.command_id;
                byte engine_cut_off = pack.engine_cut_off;
                byte extra_param = pack.extra_param;
                float extra_value = pack.extra_value;
            };
            LoopBackDemoChannel.instance.OnDIGICAM_CONTROLReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                byte session = pack.session;
                byte zoom_pos = pack.zoom_pos;
                sbyte zoom_step = pack.zoom_step;
                byte focus_lock = pack.focus_lock;
                byte shot = pack.shot;
                byte command_id = pack.command_id;
                byte extra_param = pack.extra_param;
                float extra_value = pack.extra_value;
            };
            LoopBackDemoChannel.instance.OnMOUNT_CONFIGUREReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                MAV_MOUNT_MODE mount_mode = pack.mount_mode;
                byte stab_roll = pack.stab_roll;
                byte stab_pitch = pack.stab_pitch;
                byte stab_yaw = pack.stab_yaw;
            };
            LoopBackDemoChannel.instance.OnMOUNT_CONTROLReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                int input_a = pack.input_a;
                int input_b = pack.input_b;
                int input_c = pack.input_c;
                byte save_position = pack.save_position;
            };
            LoopBackDemoChannel.instance.OnMOUNT_STATUSReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                int pointing_a = pack.pointing_a;
                int pointing_b = pack.pointing_b;
                int pointing_c = pack.pointing_c;
            };
            LoopBackDemoChannel.instance.OnFENCE_POINTReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                byte idx = pack.idx;
                byte count = pack.count;
                float lat = pack.lat;
                float lng = pack.lng;
            };
            LoopBackDemoChannel.instance.OnFENCE_FETCH_POINTReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                byte idx = pack.idx;
            };
            LoopBackDemoChannel.instance.OnFENCE_STATUSReceive += (src, ph, pack) =>
            {
                byte breach_status = pack.breach_status;
                ushort breach_count = pack.breach_count;
                FENCE_BREACH breach_type = pack.breach_type;
                uint breach_time = pack.breach_time;
            };
            LoopBackDemoChannel.instance.OnAHRSReceive += (src, ph, pack) =>
            {
                float omegaIx = pack.omegaIx;
                float omegaIy = pack.omegaIy;
                float omegaIz = pack.omegaIz;
                float accel_weight = pack.accel_weight;
                float renorm_val = pack.renorm_val;
                float error_rp = pack.error_rp;
                float error_yaw = pack.error_yaw;
            };
            LoopBackDemoChannel.instance.OnSIMSTATEReceive += (src, ph, pack) =>
            {
                float roll = pack.roll;
                float pitch = pack.pitch;
                float yaw = pack.yaw;
                float xacc = pack.xacc;
                float yacc = pack.yacc;
                float zacc = pack.zacc;
                float xgyro = pack.xgyro;
                float ygyro = pack.ygyro;
                float zgyro = pack.zgyro;
                int lat = pack.lat;
                int lng = pack.lng;
            };
            LoopBackDemoChannel.instance.OnHWSTATUSReceive += (src, ph, pack) =>
            {
                ushort Vcc = pack.Vcc;
                byte I2Cerr = pack.I2Cerr;
            };
            LoopBackDemoChannel.instance.OnRADIOReceive += (src, ph, pack) =>
            {
                byte rssi = pack.rssi;
                byte remrssi = pack.remrssi;
                byte txbuf = pack.txbuf;
                byte noise = pack.noise;
                byte remnoise = pack.remnoise;
                ushort rxerrors = pack.rxerrors;
                ushort fixed_ = pack.fixed_;
            };
            LoopBackDemoChannel.instance.OnLIMITS_STATUSReceive += (src, ph, pack) =>
            {
                LIMITS_STATE limits_state = pack.limits_state;
                uint last_trigger = pack.last_trigger;
                uint last_action = pack.last_action;
                uint last_recovery = pack.last_recovery;
                uint last_clear = pack.last_clear;
                ushort breach_count = pack.breach_count;
                LIMIT_MODULE mods_enabled = pack.mods_enabled;
                LIMIT_MODULE mods_required = pack.mods_required;
                LIMIT_MODULE mods_triggered = pack.mods_triggered;
            };
            LoopBackDemoChannel.instance.OnWINDReceive += (src, ph, pack) =>
            {
                float direction = pack.direction;
                float speed = pack.speed;
                float speed_z = pack.speed_z;
            };
            LoopBackDemoChannel.instance.OnDATA16Receive += (src, ph, pack) =>
            {
                byte type = pack.type;
                byte len = pack.len;
                byte[] data_ = pack.data_;
            };
            LoopBackDemoChannel.instance.OnDATA32Receive += (src, ph, pack) =>
            {
                byte type = pack.type;
                byte len = pack.len;
                byte[] data_ = pack.data_;
            };
            LoopBackDemoChannel.instance.OnDATA64Receive += (src, ph, pack) =>
            {
                byte type = pack.type;
                byte len = pack.len;
                byte[] data_ = pack.data_;
            };
            LoopBackDemoChannel.instance.OnDATA96Receive += (src, ph, pack) =>
            {
                byte type = pack.type;
                byte len = pack.len;
                byte[] data_ = pack.data_;
            };
            LoopBackDemoChannel.instance.OnRANGEFINDERReceive += (src, ph, pack) =>
            {
                float distance = pack.distance;
                float voltage = pack.voltage;
            };
            LoopBackDemoChannel.instance.OnAIRSPEED_AUTOCALReceive += (src, ph, pack) =>
            {
                float vx = pack.vx;
                float vy = pack.vy;
                float vz = pack.vz;
                float diff_pressure = pack.diff_pressure;
                float EAS2TAS = pack.EAS2TAS;
                float ratio = pack.ratio;
                float state_x = pack.state_x;
                float state_y = pack.state_y;
                float state_z = pack.state_z;
                float Pax = pack.Pax;
                float Pby = pack.Pby;
                float Pcz = pack.Pcz;
            };
            LoopBackDemoChannel.instance.OnRALLY_POINTReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                byte idx = pack.idx;
                byte count = pack.count;
                int lat = pack.lat;
                int lng = pack.lng;
                short alt = pack.alt;
                short break_alt = pack.break_alt;
                ushort land_dir = pack.land_dir;
                RALLY_FLAGS flags = pack.flags;
            };
            LoopBackDemoChannel.instance.OnRALLY_FETCH_POINTReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                byte idx = pack.idx;
            };
            LoopBackDemoChannel.instance.OnCOMPASSMOT_STATUSReceive += (src, ph, pack) =>
            {
                ushort throttle = pack.throttle;
                float current = pack.current;
                ushort interference = pack.interference;
                float CompensationX = pack.CompensationX;
                float CompensationY = pack.CompensationY;
                float CompensationZ = pack.CompensationZ;
            };
            LoopBackDemoChannel.instance.OnAHRS2Receive += (src, ph, pack) =>
            {
                float roll = pack.roll;
                float pitch = pack.pitch;
                float yaw = pack.yaw;
                float altitude = pack.altitude;
                int lat = pack.lat;
                int lng = pack.lng;
            };
            LoopBackDemoChannel.instance.OnCAMERA_STATUSReceive += (src, ph, pack) =>
            {
                ulong time_usec = pack.time_usec;
                byte target_system = pack.target_system;
                byte cam_idx = pack.cam_idx;
                ushort img_idx = pack.img_idx;
                CAMERA_STATUS_TYPES event_id = pack.event_id;
                float p1 = pack.p1;
                float p2 = pack.p2;
                float p3 = pack.p3;
                float p4 = pack.p4;
            };
            LoopBackDemoChannel.instance.OnCAMERA_FEEDBACKReceive += (src, ph, pack) =>
            {
                ulong time_usec = pack.time_usec;
                byte target_system = pack.target_system;
                byte cam_idx = pack.cam_idx;
                ushort img_idx = pack.img_idx;
                int lat = pack.lat;
                int lng = pack.lng;
                float alt_msl = pack.alt_msl;
                float alt_rel = pack.alt_rel;
                float roll = pack.roll;
                float pitch = pack.pitch;
                float yaw = pack.yaw;
                float foc_len = pack.foc_len;
                CAMERA_FEEDBACK_FLAGS flags = pack.flags;
            };
            LoopBackDemoChannel.instance.OnBATTERY2Receive += (src, ph, pack) =>
            {
                ushort voltage = pack.voltage;
                short current_battery = pack.current_battery;
            };
            LoopBackDemoChannel.instance.OnAHRS3Receive += (src, ph, pack) =>
            {
                float roll = pack.roll;
                float pitch = pack.pitch;
                float yaw = pack.yaw;
                float altitude = pack.altitude;
                int lat = pack.lat;
                int lng = pack.lng;
                float v1 = pack.v1;
                float v2 = pack.v2;
                float v3 = pack.v3;
                float v4 = pack.v4;
            };
            LoopBackDemoChannel.instance.OnAUTOPILOT_VERSION_REQUESTReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
            };
            LoopBackDemoChannel.instance.OnREMOTE_LOG_DATA_BLOCKReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS seqno = pack.seqno;
                byte[] data_ = pack.data_;
            };
            LoopBackDemoChannel.instance.OnREMOTE_LOG_BLOCK_STATUSReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                uint seqno = pack.seqno;
                MAV_REMOTE_LOG_DATA_BLOCK_STATUSES status = pack.status;
            };
            LoopBackDemoChannel.instance.OnLED_CONTROLReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                byte instance = pack.instance;
                byte pattern = pack.pattern;
                byte custom_len = pack.custom_len;
                byte[] custom_bytes = pack.custom_bytes;
            };
            LoopBackDemoChannel.instance.OnMAG_CAL_PROGRESSReceive += (src, ph, pack) =>
            {
                byte compass_id = pack.compass_id;
                byte cal_mask = pack.cal_mask;
                MAG_CAL_STATUS cal_status = pack.cal_status;
                byte attempt = pack.attempt;
                byte completion_pct = pack.completion_pct;
                byte[] completion_mask = pack.completion_mask;
                float direction_x = pack.direction_x;
                float direction_y = pack.direction_y;
                float direction_z = pack.direction_z;
            };
            LoopBackDemoChannel.instance.OnMAG_CAL_REPORTReceive += (src, ph, pack) =>
            {
                byte compass_id = pack.compass_id;
                byte cal_mask = pack.cal_mask;
                MAG_CAL_STATUS cal_status = pack.cal_status;
                byte autosaved = pack.autosaved;
                float fitness = pack.fitness;
                float ofs_x = pack.ofs_x;
                float ofs_y = pack.ofs_y;
                float ofs_z = pack.ofs_z;
                float diag_x = pack.diag_x;
                float diag_y = pack.diag_y;
                float diag_z = pack.diag_z;
                float offdiag_x = pack.offdiag_x;
                float offdiag_y = pack.offdiag_y;
                float offdiag_z = pack.offdiag_z;
            };
            LoopBackDemoChannel.instance.OnEKF_STATUS_REPORTReceive += (src, ph, pack) =>
            {
                EKF_STATUS_FLAGS flags = pack.flags;
                float velocity_variance = pack.velocity_variance;
                float pos_horiz_variance = pack.pos_horiz_variance;
                float pos_vert_variance = pack.pos_vert_variance;
                float compass_variance = pack.compass_variance;
                float terrain_alt_variance = pack.terrain_alt_variance;
            };
            LoopBackDemoChannel.instance.OnPID_TUNINGReceive += (src, ph, pack) =>
            {
                PID_TUNING_AXIS axis = pack.axis;
                float desired = pack.desired;
                float achieved = pack.achieved;
                float FF = pack.FF;
                float P = pack.P;
                float I = pack.I;
                float D = pack.D;
            };
            LoopBackDemoChannel.instance.OnGIMBAL_REPORTReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                float delta_time = pack.delta_time;
                float delta_angle_x = pack.delta_angle_x;
                float delta_angle_y = pack.delta_angle_y;
                float delta_angle_z = pack.delta_angle_z;
                float delta_velocity_x = pack.delta_velocity_x;
                float delta_velocity_y = pack.delta_velocity_y;
                float delta_velocity_z = pack.delta_velocity_z;
                float joint_roll = pack.joint_roll;
                float joint_el = pack.joint_el;
                float joint_az = pack.joint_az;
            };
            LoopBackDemoChannel.instance.OnGIMBAL_CONTROLReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                float demanded_rate_x = pack.demanded_rate_x;
                float demanded_rate_y = pack.demanded_rate_y;
                float demanded_rate_z = pack.demanded_rate_z;
            };
            LoopBackDemoChannel.instance.OnGIMBAL_TORQUE_CMD_REPORTReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                short rl_torque_cmd = pack.rl_torque_cmd;
                short el_torque_cmd = pack.el_torque_cmd;
                short az_torque_cmd = pack.az_torque_cmd;
            };
            LoopBackDemoChannel.instance.OnGOPRO_HEARTBEATReceive += (src, ph, pack) =>
            {
                GOPRO_HEARTBEAT_STATUS status = pack.status;
                GOPRO_CAPTURE_MODE capture_mode = pack.capture_mode;
                GOPRO_HEARTBEAT_FLAGS flags = pack.flags;
            };
            LoopBackDemoChannel.instance.OnGOPRO_GET_REQUESTReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                GOPRO_COMMAND cmd_id = pack.cmd_id;
            };
            LoopBackDemoChannel.instance.OnGOPRO_GET_RESPONSEReceive += (src, ph, pack) =>
            {
                GOPRO_COMMAND cmd_id = pack.cmd_id;
                GOPRO_REQUEST_STATUS status = pack.status;
                byte[] value = pack.value;
            };
            LoopBackDemoChannel.instance.OnGOPRO_SET_REQUESTReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                GOPRO_COMMAND cmd_id = pack.cmd_id;
                byte[] value = pack.value;
            };
            LoopBackDemoChannel.instance.OnGOPRO_SET_RESPONSEReceive += (src, ph, pack) =>
            {
                GOPRO_COMMAND cmd_id = pack.cmd_id;
                GOPRO_REQUEST_STATUS status = pack.status;
            };
            LoopBackDemoChannel.instance.OnRPMReceive += (src, ph, pack) =>
            {
                float rpm1 = pack.rpm1;
                float rpm2 = pack.rpm2;
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
            LoopBackDemoChannel.instance.OnUAVIONIX_ADSB_OUT_CFGReceive += (src, ph, pack) =>
            {
                uint ICAO = pack.ICAO;
                string callsign = pack.callsign_TRY(ph);
                ADSB_EMITTER_TYPE emitterType = pack.emitterType;
                UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE aircraftSize = pack.aircraftSize;
                UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT gpsOffsetLat = pack.gpsOffsetLat;
                UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON gpsOffsetLon = pack.gpsOffsetLon;
                ushort stallSpeed = pack.stallSpeed;
                UAVIONIX_ADSB_OUT_RF_SELECT rfSelect = pack.rfSelect;
            };
            LoopBackDemoChannel.instance.OnUAVIONIX_ADSB_OUT_DYNAMICReceive += (src, ph, pack) =>
            {
                uint utcTime = pack.utcTime;
                int gpsLat = pack.gpsLat;
                int gpsLon = pack.gpsLon;
                int gpsAlt = pack.gpsAlt;
                UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX gpsFix = pack.gpsFix;
                byte numSats = pack.numSats;
                int baroAltMSL = pack.baroAltMSL;
                uint accuracyHor = pack.accuracyHor;
                ushort accuracyVert = pack.accuracyVert;
                ushort accuracyVel = pack.accuracyVel;
                short velVert = pack.velVert;
                short velNS = pack.velNS;
                short VelEW = pack.VelEW;
                UAVIONIX_ADSB_EMERGENCY_STATUS emergencyStatus = pack.emergencyStatus;
                UAVIONIX_ADSB_OUT_DYNAMIC_STATE state = pack.state;
                ushort squawk = pack.squawk;
            };
            LoopBackDemoChannel.instance.OnUAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORTReceive += (src, ph, pack) =>
            {
                UAVIONIX_ADSB_RF_HEALTH rfHealth = pack.rfHealth;
            };
            LoopBackDemoChannel.instance.OnDEVICE_OP_READReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                uint request_id = pack.request_id;
                DEVICE_OP_BUSTYPE bustype = pack.bustype;
                byte bus = pack.bus;
                byte address = pack.address;
                string busname = pack.busname_TRY(ph);
                byte regstart = pack.regstart;
                byte count = pack.count;
            };
            LoopBackDemoChannel.instance.OnDEVICE_OP_READ_REPLYReceive += (src, ph, pack) =>
            {
                uint request_id = pack.request_id;
                byte result = pack.result;
                byte regstart = pack.regstart;
                byte count = pack.count;
                byte[] data_ = pack.data_;
            };
            LoopBackDemoChannel.instance.OnDEVICE_OP_WRITEReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                uint request_id = pack.request_id;
                DEVICE_OP_BUSTYPE bustype = pack.bustype;
                byte bus = pack.bus;
                byte address = pack.address;
                string busname = pack.busname_TRY(ph);
                byte regstart = pack.regstart;
                byte count = pack.count;
                byte[] data_ = pack.data_;
            };
            LoopBackDemoChannel.instance.OnDEVICE_OP_WRITE_REPLYReceive += (src, ph, pack) =>
            {
                uint request_id = pack.request_id;
                byte result = pack.result;
            };
            LoopBackDemoChannel.instance.OnADAP_TUNINGReceive += (src, ph, pack) =>
            {
                PID_TUNING_AXIS axis = pack.axis;
                float desired = pack.desired;
                float achieved = pack.achieved;
                float error = pack.error;
                float theta = pack.theta;
                float omega = pack.omega;
                float sigma = pack.sigma;
                float theta_dot = pack.theta_dot;
                float omega_dot = pack.omega_dot;
                float sigma_dot = pack.sigma_dot;
                float f = pack.f;
                float f_dot = pack.f_dot;
                float u = pack.u;
            };
            LoopBackDemoChannel.instance.OnVISION_POSITION_DELTAReceive += (src, ph, pack) =>
            {
                ulong time_usec = pack.time_usec;
                ulong time_delta_usec = pack.time_delta_usec;
                float[] angle_delta = pack.angle_delta;
                float[] position_delta = pack.position_delta;
                float confidence = pack.confidence;
            };
            HEARTBEAT p0 = LoopBackDemoChannel.new_HEARTBEAT();
            PH.setPack(p0);
            p0.type = (MAV_TYPE)MAV_TYPE.MAV_TYPE_ANTENNA_TRACKER;
            p0.autopilot = (MAV_AUTOPILOT)MAV_AUTOPILOT.MAV_AUTOPILOT_SMARTAP;
            p0.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED;
            p0.custom_mode = (uint)2905745151U;
            p0.system_status = (MAV_STATE)MAV_STATE.MAV_STATE_POWEROFF;
            p0.mavlink_version = (byte)(byte)6;
            LoopBackDemoChannel.instance.send(p0); //===============================
            SYS_STATUS p1 = LoopBackDemoChannel.new_SYS_STATUS();
            PH.setPack(p1);
            p1.onboard_control_sensors_present = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
            p1.onboard_control_sensors_enabled = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
            p1.onboard_control_sensors_health = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
            p1.load = (ushort)(ushort)3757;
            p1.voltage_battery = (ushort)(ushort)30322;
            p1.current_battery = (short)(short)9544;
            p1.battery_remaining = (sbyte)(sbyte) - 57;
            p1.drop_rate_comm = (ushort)(ushort)11202;
            p1.errors_comm = (ushort)(ushort)30232;
            p1.errors_count1 = (ushort)(ushort)25479;
            p1.errors_count2 = (ushort)(ushort)53855;
            p1.errors_count3 = (ushort)(ushort)65479;
            p1.errors_count4 = (ushort)(ushort)41704;
            LoopBackDemoChannel.instance.send(p1); //===============================
            SYSTEM_TIME p2 = LoopBackDemoChannel.new_SYSTEM_TIME();
            PH.setPack(p2);
            p2.time_unix_usec = (ulong)6897609587986606506L;
            p2.time_boot_ms = (uint)4020233627U;
            LoopBackDemoChannel.instance.send(p2); //===============================
            POSITION_TARGET_LOCAL_NED p3 = LoopBackDemoChannel.new_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.time_boot_ms = (uint)2847417881U;
            p3.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT;
            p3.type_mask = (ushort)(ushort)50705;
            p3.x = (float)1.0810877E38F;
            p3.y = (float) -1.7942882E38F;
            p3.z = (float) -1.7669617E38F;
            p3.vx = (float)1.764715E38F;
            p3.vy = (float)1.8222129E38F;
            p3.vz = (float)2.6046995E38F;
            p3.afx = (float) -2.9063555E38F;
            p3.afy = (float)3.0630935E38F;
            p3.afz = (float)2.2717347E38F;
            p3.yaw = (float) -8.4996835E37F;
            p3.yaw_rate = (float) -2.3605506E38F;
            LoopBackDemoChannel.instance.send(p3); //===============================
            PING p4 = LoopBackDemoChannel.new_PING();
            PH.setPack(p4);
            p4.time_usec = (ulong)6868753413758471816L;
            p4.seq = (uint)814114256U;
            p4.target_system = (byte)(byte)24;
            p4.target_component = (byte)(byte)250;
            LoopBackDemoChannel.instance.send(p4); //===============================
            CHANGE_OPERATOR_CONTROL p5 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL();
            PH.setPack(p5);
            p5.target_system = (byte)(byte)248;
            p5.control_request = (byte)(byte)29;
            p5.version = (byte)(byte)142;
            p5.passkey_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p5); //===============================
            CHANGE_OPERATOR_CONTROL_ACK p6 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL_ACK();
            PH.setPack(p6);
            p6.gcs_system_id = (byte)(byte)247;
            p6.control_request = (byte)(byte)19;
            p6.ack = (byte)(byte)165;
            LoopBackDemoChannel.instance.send(p6); //===============================
            AUTH_KEY p7 = LoopBackDemoChannel.new_AUTH_KEY();
            PH.setPack(p7);
            p7.key_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p7); //===============================
            SET_MODE p11 = LoopBackDemoChannel.new_SET_MODE();
            PH.setPack(p11);
            p11.target_system = (byte)(byte)149;
            p11.base_mode = (MAV_MODE)MAV_MODE.MAV_MODE_TEST_DISARMED;
            p11.custom_mode = (uint)1144508983U;
            LoopBackDemoChannel.instance.send(p11); //===============================
            PARAM_REQUEST_READ p20 = LoopBackDemoChannel.new_PARAM_REQUEST_READ();
            PH.setPack(p20);
            p20.target_system = (byte)(byte)79;
            p20.target_component = (byte)(byte)151;
            p20.param_id_SET("DEMO", PH);
            p20.param_index = (short)(short)19313;
            LoopBackDemoChannel.instance.send(p20); //===============================
            PARAM_REQUEST_LIST p21 = LoopBackDemoChannel.new_PARAM_REQUEST_LIST();
            PH.setPack(p21);
            p21.target_system = (byte)(byte)220;
            p21.target_component = (byte)(byte)133;
            LoopBackDemoChannel.instance.send(p21); //===============================
            PARAM_VALUE p22 = LoopBackDemoChannel.new_PARAM_VALUE();
            PH.setPack(p22);
            p22.param_id_SET("DEMO", PH);
            p22.param_value = (float) -2.1486605E38F;
            p22.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32;
            p22.param_count = (ushort)(ushort)64199;
            p22.param_index = (ushort)(ushort)15961;
            LoopBackDemoChannel.instance.send(p22); //===============================
            PARAM_SET p23 = LoopBackDemoChannel.new_PARAM_SET();
            PH.setPack(p23);
            p23.target_system = (byte)(byte)63;
            p23.target_component = (byte)(byte)59;
            p23.param_id_SET("DEMO", PH);
            p23.param_value = (float)2.8037763E36F;
            p23.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32;
            LoopBackDemoChannel.instance.send(p23); //===============================
            GPS_RAW_INT p24 = LoopBackDemoChannel.new_GPS_RAW_INT();
            PH.setPack(p24);
            p24.time_usec = (ulong)4146920176659174034L;
            p24.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_PPP;
            p24.lat = (int)1198890246;
            p24.lon = (int) -327542125;
            p24.alt = (int)1488417102;
            p24.eph = (ushort)(ushort)52551;
            p24.epv = (ushort)(ushort)61641;
            p24.vel = (ushort)(ushort)55362;
            p24.cog = (ushort)(ushort)33950;
            p24.satellites_visible = (byte)(byte)188;
            p24.alt_ellipsoid_SET((int) -1507802202, PH);
            p24.h_acc_SET((uint)1601306589U, PH);
            p24.v_acc_SET((uint)1740287421U, PH);
            p24.vel_acc_SET((uint)193284340U, PH);
            p24.hdg_acc_SET((uint)1021963900U, PH);
            LoopBackDemoChannel.instance.send(p24); //===============================
            GPS_STATUS p25 = LoopBackDemoChannel.new_GPS_STATUS();
            PH.setPack(p25);
            p25.satellites_visible = (byte)(byte)237;
            p25.satellite_prn_SET(new byte[20], 0);
            p25.satellite_used_SET(new byte[20], 0);
            p25.satellite_elevation_SET(new byte[20], 0);
            p25.satellite_azimuth_SET(new byte[20], 0);
            p25.satellite_snr_SET(new byte[20], 0);
            LoopBackDemoChannel.instance.send(p25); //===============================
            SCALED_IMU p26 = LoopBackDemoChannel.new_SCALED_IMU();
            PH.setPack(p26);
            p26.time_boot_ms = (uint)2064519259U;
            p26.xacc = (short)(short) -3270;
            p26.yacc = (short)(short)11581;
            p26.zacc = (short)(short) -11149;
            p26.xgyro = (short)(short) -29839;
            p26.ygyro = (short)(short) -2879;
            p26.zgyro = (short)(short) -17021;
            p26.xmag = (short)(short) -13037;
            p26.ymag = (short)(short) -22882;
            p26.zmag = (short)(short) -15677;
            LoopBackDemoChannel.instance.send(p26); //===============================
            RAW_IMU p27 = LoopBackDemoChannel.new_RAW_IMU();
            PH.setPack(p27);
            p27.time_usec = (ulong)3659122777672860335L;
            p27.xacc = (short)(short) -4767;
            p27.yacc = (short)(short)25775;
            p27.zacc = (short)(short) -26307;
            p27.xgyro = (short)(short)2657;
            p27.ygyro = (short)(short)1863;
            p27.zgyro = (short)(short)18314;
            p27.xmag = (short)(short) -5514;
            p27.ymag = (short)(short)12651;
            p27.zmag = (short)(short) -27049;
            LoopBackDemoChannel.instance.send(p27); //===============================
            RAW_PRESSURE p28 = LoopBackDemoChannel.new_RAW_PRESSURE();
            PH.setPack(p28);
            p28.time_usec = (ulong)5372785918067587032L;
            p28.press_abs = (short)(short)12210;
            p28.press_diff1 = (short)(short) -13028;
            p28.press_diff2 = (short)(short) -21556;
            p28.temperature = (short)(short)7791;
            LoopBackDemoChannel.instance.send(p28); //===============================
            SCALED_PRESSURE p29 = LoopBackDemoChannel.new_SCALED_PRESSURE();
            PH.setPack(p29);
            p29.time_boot_ms = (uint)980186891U;
            p29.press_abs = (float) -1.0237585E37F;
            p29.press_diff = (float) -2.0690932E38F;
            p29.temperature = (short)(short)31972;
            LoopBackDemoChannel.instance.send(p29); //===============================
            ATTITUDE p30 = LoopBackDemoChannel.new_ATTITUDE();
            PH.setPack(p30);
            p30.time_boot_ms = (uint)1847441799U;
            p30.roll = (float)7.7968103E37F;
            p30.pitch = (float) -7.07197E37F;
            p30.yaw = (float)2.3175612E38F;
            p30.rollspeed = (float)1.1340451E38F;
            p30.pitchspeed = (float)2.2848197E38F;
            p30.yawspeed = (float) -1.216885E37F;
            LoopBackDemoChannel.instance.send(p30); //===============================
            ATTITUDE_QUATERNION p31 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION();
            PH.setPack(p31);
            p31.time_boot_ms = (uint)4085360257U;
            p31.q1 = (float)1.5470606E38F;
            p31.q2 = (float)1.204109E37F;
            p31.q3 = (float)1.8144674E38F;
            p31.q4 = (float) -1.742858E38F;
            p31.rollspeed = (float) -3.0920625E38F;
            p31.pitchspeed = (float) -1.8388933E38F;
            p31.yawspeed = (float) -1.9917596E38F;
            LoopBackDemoChannel.instance.send(p31); //===============================
            LOCAL_POSITION_NED p32 = LoopBackDemoChannel.new_LOCAL_POSITION_NED();
            PH.setPack(p32);
            p32.time_boot_ms = (uint)1406996918U;
            p32.x = (float)1.4820958E38F;
            p32.y = (float) -3.044884E38F;
            p32.z = (float) -2.0799712E38F;
            p32.vx = (float) -3.3171901E38F;
            p32.vy = (float) -1.2050478E38F;
            p32.vz = (float) -1.1050477E38F;
            LoopBackDemoChannel.instance.send(p32); //===============================
            GLOBAL_POSITION_INT p33 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT();
            PH.setPack(p33);
            p33.time_boot_ms = (uint)3880535381U;
            p33.lat = (int)447162347;
            p33.lon = (int)781940188;
            p33.alt = (int)1183084004;
            p33.relative_alt = (int) -422204154;
            p33.vx = (short)(short) -574;
            p33.vy = (short)(short)13365;
            p33.vz = (short)(short)17517;
            p33.hdg = (ushort)(ushort)15195;
            LoopBackDemoChannel.instance.send(p33); //===============================
            RC_CHANNELS_SCALED p34 = LoopBackDemoChannel.new_RC_CHANNELS_SCALED();
            PH.setPack(p34);
            p34.time_boot_ms = (uint)2665791742U;
            p34.port = (byte)(byte)227;
            p34.chan1_scaled = (short)(short) -10235;
            p34.chan2_scaled = (short)(short) -26526;
            p34.chan3_scaled = (short)(short)14071;
            p34.chan4_scaled = (short)(short)20558;
            p34.chan5_scaled = (short)(short)14410;
            p34.chan6_scaled = (short)(short)3371;
            p34.chan7_scaled = (short)(short)22032;
            p34.chan8_scaled = (short)(short) -19611;
            p34.rssi = (byte)(byte)37;
            LoopBackDemoChannel.instance.send(p34); //===============================
            RC_CHANNELS_RAW p35 = LoopBackDemoChannel.new_RC_CHANNELS_RAW();
            PH.setPack(p35);
            p35.time_boot_ms = (uint)56167598U;
            p35.port = (byte)(byte)135;
            p35.chan1_raw = (ushort)(ushort)53091;
            p35.chan2_raw = (ushort)(ushort)6313;
            p35.chan3_raw = (ushort)(ushort)9804;
            p35.chan4_raw = (ushort)(ushort)29383;
            p35.chan5_raw = (ushort)(ushort)52833;
            p35.chan6_raw = (ushort)(ushort)61852;
            p35.chan7_raw = (ushort)(ushort)947;
            p35.chan8_raw = (ushort)(ushort)55466;
            p35.rssi = (byte)(byte)165;
            LoopBackDemoChannel.instance.send(p35); //===============================
            SERVO_OUTPUT_RAW p36 = LoopBackDemoChannel.new_SERVO_OUTPUT_RAW();
            PH.setPack(p36);
            p36.time_usec = (uint)1513255556U;
            p36.port = (byte)(byte)88;
            p36.servo1_raw = (ushort)(ushort)39047;
            p36.servo2_raw = (ushort)(ushort)43621;
            p36.servo3_raw = (ushort)(ushort)62718;
            p36.servo4_raw = (ushort)(ushort)11984;
            p36.servo5_raw = (ushort)(ushort)51477;
            p36.servo6_raw = (ushort)(ushort)1679;
            p36.servo7_raw = (ushort)(ushort)16003;
            p36.servo8_raw = (ushort)(ushort)17536;
            p36.servo9_raw_SET((ushort)(ushort)45060, PH);
            p36.servo10_raw_SET((ushort)(ushort)23378, PH);
            p36.servo11_raw_SET((ushort)(ushort)23027, PH);
            p36.servo12_raw_SET((ushort)(ushort)7232, PH);
            p36.servo13_raw_SET((ushort)(ushort)45248, PH);
            p36.servo14_raw_SET((ushort)(ushort)38751, PH);
            p36.servo15_raw_SET((ushort)(ushort)21417, PH);
            p36.servo16_raw_SET((ushort)(ushort)50123, PH);
            LoopBackDemoChannel.instance.send(p36); //===============================
            MISSION_REQUEST_PARTIAL_LIST p37 = LoopBackDemoChannel.new_MISSION_REQUEST_PARTIAL_LIST();
            PH.setPack(p37);
            p37.target_system = (byte)(byte)122;
            p37.target_component = (byte)(byte)179;
            p37.start_index = (short)(short) -7111;
            p37.end_index = (short)(short)14655;
            p37.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            LoopBackDemoChannel.instance.send(p37); //===============================
            MISSION_WRITE_PARTIAL_LIST p38 = LoopBackDemoChannel.new_MISSION_WRITE_PARTIAL_LIST();
            PH.setPack(p38);
            p38.target_system = (byte)(byte)187;
            p38.target_component = (byte)(byte)110;
            p38.start_index = (short)(short)12281;
            p38.end_index = (short)(short) -27291;
            p38.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            LoopBackDemoChannel.instance.send(p38); //===============================
            MISSION_ITEM p39 = LoopBackDemoChannel.new_MISSION_ITEM();
            PH.setPack(p39);
            p39.target_system = (byte)(byte)194;
            p39.target_component = (byte)(byte)240;
            p39.seq = (ushort)(ushort)53806;
            p39.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_NED;
            p39.command = (MAV_CMD)MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE;
            p39.current = (byte)(byte)94;
            p39.autocontinue = (byte)(byte)41;
            p39.param1 = (float) -1.6480187E38F;
            p39.param2 = (float)1.5256258E38F;
            p39.param3 = (float) -3.2358167E38F;
            p39.param4 = (float)1.4640118E38F;
            p39.x = (float) -1.9982885E38F;
            p39.y = (float)1.6183514E38F;
            p39.z = (float) -1.331183E38F;
            p39.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            LoopBackDemoChannel.instance.send(p39); //===============================
            MISSION_REQUEST p40 = LoopBackDemoChannel.new_MISSION_REQUEST();
            PH.setPack(p40);
            p40.target_system = (byte)(byte)43;
            p40.target_component = (byte)(byte)251;
            p40.seq = (ushort)(ushort)59638;
            p40.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            LoopBackDemoChannel.instance.send(p40); //===============================
            MISSION_SET_CURRENT p41 = LoopBackDemoChannel.new_MISSION_SET_CURRENT();
            PH.setPack(p41);
            p41.target_system = (byte)(byte)100;
            p41.target_component = (byte)(byte)35;
            p41.seq = (ushort)(ushort)51975;
            LoopBackDemoChannel.instance.send(p41); //===============================
            MISSION_CURRENT p42 = LoopBackDemoChannel.new_MISSION_CURRENT();
            PH.setPack(p42);
            p42.seq = (ushort)(ushort)4728;
            LoopBackDemoChannel.instance.send(p42); //===============================
            MISSION_REQUEST_LIST p43 = LoopBackDemoChannel.new_MISSION_REQUEST_LIST();
            PH.setPack(p43);
            p43.target_system = (byte)(byte)22;
            p43.target_component = (byte)(byte)66;
            p43.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            LoopBackDemoChannel.instance.send(p43); //===============================
            MISSION_COUNT p44 = LoopBackDemoChannel.new_MISSION_COUNT();
            PH.setPack(p44);
            p44.target_system = (byte)(byte)195;
            p44.target_component = (byte)(byte)198;
            p44.count = (ushort)(ushort)59011;
            p44.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            LoopBackDemoChannel.instance.send(p44); //===============================
            MISSION_CLEAR_ALL p45 = LoopBackDemoChannel.new_MISSION_CLEAR_ALL();
            PH.setPack(p45);
            p45.target_system = (byte)(byte)210;
            p45.target_component = (byte)(byte)11;
            p45.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            LoopBackDemoChannel.instance.send(p45); //===============================
            MISSION_ITEM_REACHED p46 = LoopBackDemoChannel.new_MISSION_ITEM_REACHED();
            PH.setPack(p46);
            p46.seq = (ushort)(ushort)18635;
            LoopBackDemoChannel.instance.send(p46); //===============================
            MISSION_ACK p47 = LoopBackDemoChannel.new_MISSION_ACK();
            PH.setPack(p47);
            p47.target_system = (byte)(byte)34;
            p47.target_component = (byte)(byte)166;
            p47.type = (MAV_MISSION_RESULT)MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM5_X;
            p47.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            LoopBackDemoChannel.instance.send(p47); //===============================
            SET_GPS_GLOBAL_ORIGIN p48 = LoopBackDemoChannel.new_SET_GPS_GLOBAL_ORIGIN();
            PH.setPack(p48);
            p48.target_system = (byte)(byte)226;
            p48.latitude = (int) -1057015197;
            p48.longitude = (int)2042226966;
            p48.altitude = (int) -1845964084;
            p48.time_usec_SET((ulong)7074207845839717850L, PH);
            LoopBackDemoChannel.instance.send(p48); //===============================
            GPS_GLOBAL_ORIGIN p49 = LoopBackDemoChannel.new_GPS_GLOBAL_ORIGIN();
            PH.setPack(p49);
            p49.latitude = (int)492268567;
            p49.longitude = (int) -277232171;
            p49.altitude = (int) -644610978;
            p49.time_usec_SET((ulong)6986357173239518156L, PH);
            LoopBackDemoChannel.instance.send(p49); //===============================
            PARAM_MAP_RC p50 = LoopBackDemoChannel.new_PARAM_MAP_RC();
            PH.setPack(p50);
            p50.target_system = (byte)(byte)84;
            p50.target_component = (byte)(byte)247;
            p50.param_id_SET("DEMO", PH);
            p50.param_index = (short)(short) -27070;
            p50.parameter_rc_channel_index = (byte)(byte)37;
            p50.param_value0 = (float)8.271185E37F;
            p50.scale = (float) -2.4377473E38F;
            p50.param_value_min = (float)1.1445387E38F;
            p50.param_value_max = (float)6.5209727E37F;
            LoopBackDemoChannel.instance.send(p50); //===============================
            MISSION_REQUEST_INT p51 = LoopBackDemoChannel.new_MISSION_REQUEST_INT();
            PH.setPack(p51);
            p51.target_system = (byte)(byte)172;
            p51.target_component = (byte)(byte)171;
            p51.seq = (ushort)(ushort)45824;
            p51.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            LoopBackDemoChannel.instance.send(p51); //===============================
            SAFETY_SET_ALLOWED_AREA p54 = LoopBackDemoChannel.new_SAFETY_SET_ALLOWED_AREA();
            PH.setPack(p54);
            p54.target_system = (byte)(byte)77;
            p54.target_component = (byte)(byte)126;
            p54.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p54.p1x = (float) -1.5288506E38F;
            p54.p1y = (float)1.3942973E37F;
            p54.p1z = (float) -3.1523377E38F;
            p54.p2x = (float)1.2252156E38F;
            p54.p2y = (float)2.5477481E38F;
            p54.p2z = (float) -1.1396002E38F;
            LoopBackDemoChannel.instance.send(p54); //===============================
            SAFETY_ALLOWED_AREA p55 = LoopBackDemoChannel.new_SAFETY_ALLOWED_AREA();
            PH.setPack(p55);
            p55.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_NED;
            p55.p1x = (float)2.7763848E37F;
            p55.p1y = (float) -2.2703032E38F;
            p55.p1z = (float) -2.7660162E38F;
            p55.p2x = (float) -1.8315079E38F;
            p55.p2y = (float)3.3505963E38F;
            p55.p2z = (float) -4.624624E37F;
            LoopBackDemoChannel.instance.send(p55); //===============================
            ATTITUDE_QUATERNION_COV p61 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION_COV();
            PH.setPack(p61);
            p61.time_usec = (ulong)558680687426010750L;
            p61.q_SET(new float[4], 0);
            p61.rollspeed = (float)1.6112284E38F;
            p61.pitchspeed = (float)1.5277727E38F;
            p61.yawspeed = (float)2.97143E38F;
            p61.covariance_SET(new float[9], 0);
            LoopBackDemoChannel.instance.send(p61); //===============================
            NAV_CONTROLLER_OUTPUT p62 = LoopBackDemoChannel.new_NAV_CONTROLLER_OUTPUT();
            PH.setPack(p62);
            p62.nav_roll = (float) -3.35859E38F;
            p62.nav_pitch = (float)3.7674753E37F;
            p62.nav_bearing = (short)(short) -14793;
            p62.target_bearing = (short)(short)190;
            p62.wp_dist = (ushort)(ushort)10868;
            p62.alt_error = (float)1.8035125E38F;
            p62.aspd_error = (float) -1.9473178E38F;
            p62.xtrack_error = (float) -1.3288544E37F;
            LoopBackDemoChannel.instance.send(p62); //===============================
            GLOBAL_POSITION_INT_COV p63 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT_COV();
            PH.setPack(p63);
            p63.time_usec = (ulong)5765423608327350768L;
            p63.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE;
            p63.lat = (int)1244761084;
            p63.lon = (int) -1751253811;
            p63.alt = (int) -1396172219;
            p63.relative_alt = (int)952584397;
            p63.vx = (float)2.0417537E38F;
            p63.vy = (float) -2.0334756E38F;
            p63.vz = (float)7.7141113E37F;
            p63.covariance_SET(new float[36], 0);
            LoopBackDemoChannel.instance.send(p63); //===============================
            LOCAL_POSITION_NED_COV p64 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_COV();
            PH.setPack(p64);
            p64.time_usec = (ulong)4360540431270182920L;
            p64.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS;
            p64.x = (float) -2.5766737E36F;
            p64.y = (float)2.8511782E38F;
            p64.z = (float) -2.3053766E38F;
            p64.vx = (float) -1.4070028E38F;
            p64.vy = (float)2.1156448E38F;
            p64.vz = (float) -5.459488E37F;
            p64.ax = (float) -1.0861545E38F;
            p64.ay = (float)3.116628E38F;
            p64.az = (float)1.4449341E38F;
            p64.covariance_SET(new float[45], 0);
            LoopBackDemoChannel.instance.send(p64); //===============================
            RC_CHANNELS p65 = LoopBackDemoChannel.new_RC_CHANNELS();
            PH.setPack(p65);
            p65.time_boot_ms = (uint)2717594587U;
            p65.chancount = (byte)(byte)106;
            p65.chan1_raw = (ushort)(ushort)38104;
            p65.chan2_raw = (ushort)(ushort)56403;
            p65.chan3_raw = (ushort)(ushort)14205;
            p65.chan4_raw = (ushort)(ushort)21209;
            p65.chan5_raw = (ushort)(ushort)54136;
            p65.chan6_raw = (ushort)(ushort)6580;
            p65.chan7_raw = (ushort)(ushort)13721;
            p65.chan8_raw = (ushort)(ushort)25900;
            p65.chan9_raw = (ushort)(ushort)12744;
            p65.chan10_raw = (ushort)(ushort)32237;
            p65.chan11_raw = (ushort)(ushort)22361;
            p65.chan12_raw = (ushort)(ushort)30057;
            p65.chan13_raw = (ushort)(ushort)7698;
            p65.chan14_raw = (ushort)(ushort)32492;
            p65.chan15_raw = (ushort)(ushort)2424;
            p65.chan16_raw = (ushort)(ushort)29410;
            p65.chan17_raw = (ushort)(ushort)45225;
            p65.chan18_raw = (ushort)(ushort)45460;
            p65.rssi = (byte)(byte)122;
            LoopBackDemoChannel.instance.send(p65); //===============================
            REQUEST_DATA_STREAM p66 = LoopBackDemoChannel.new_REQUEST_DATA_STREAM();
            PH.setPack(p66);
            p66.target_system = (byte)(byte)49;
            p66.target_component = (byte)(byte)195;
            p66.req_stream_id = (byte)(byte)133;
            p66.req_message_rate = (ushort)(ushort)36306;
            p66.start_stop = (byte)(byte)67;
            LoopBackDemoChannel.instance.send(p66); //===============================
            DATA_STREAM p67 = LoopBackDemoChannel.new_DATA_STREAM();
            PH.setPack(p67);
            p67.stream_id = (byte)(byte)192;
            p67.message_rate = (ushort)(ushort)20253;
            p67.on_off = (byte)(byte)27;
            LoopBackDemoChannel.instance.send(p67); //===============================
            MANUAL_CONTROL p69 = LoopBackDemoChannel.new_MANUAL_CONTROL();
            PH.setPack(p69);
            p69.target = (byte)(byte)41;
            p69.x = (short)(short) -5851;
            p69.y = (short)(short)6253;
            p69.z = (short)(short) -5242;
            p69.r = (short)(short)8712;
            p69.buttons = (ushort)(ushort)31071;
            LoopBackDemoChannel.instance.send(p69); //===============================
            RC_CHANNELS_OVERRIDE p70 = LoopBackDemoChannel.new_RC_CHANNELS_OVERRIDE();
            PH.setPack(p70);
            p70.target_system = (byte)(byte)29;
            p70.target_component = (byte)(byte)230;
            p70.chan1_raw = (ushort)(ushort)12832;
            p70.chan2_raw = (ushort)(ushort)11771;
            p70.chan3_raw = (ushort)(ushort)64149;
            p70.chan4_raw = (ushort)(ushort)57490;
            p70.chan5_raw = (ushort)(ushort)26882;
            p70.chan6_raw = (ushort)(ushort)15056;
            p70.chan7_raw = (ushort)(ushort)50935;
            p70.chan8_raw = (ushort)(ushort)34326;
            LoopBackDemoChannel.instance.send(p70); //===============================
            MISSION_ITEM_INT p73 = LoopBackDemoChannel.new_MISSION_ITEM_INT();
            PH.setPack(p73);
            p73.target_system = (byte)(byte)141;
            p73.target_component = (byte)(byte)28;
            p73.seq = (ushort)(ushort)57894;
            p73.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_NED;
            p73.command = (MAV_CMD)MAV_CMD.MAV_CMD_AIRFRAME_CONFIGURATION;
            p73.current = (byte)(byte)59;
            p73.autocontinue = (byte)(byte)52;
            p73.param1 = (float)1.93141E37F;
            p73.param2 = (float) -1.634905E38F;
            p73.param3 = (float) -2.784095E38F;
            p73.param4 = (float)1.7187162E38F;
            p73.x = (int) -1018355930;
            p73.y = (int)898805090;
            p73.z = (float) -3.1960772E38F;
            p73.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            LoopBackDemoChannel.instance.send(p73); //===============================
            VFR_HUD p74 = LoopBackDemoChannel.new_VFR_HUD();
            PH.setPack(p74);
            p74.airspeed = (float) -1.5433053E37F;
            p74.groundspeed = (float) -1.8411098E38F;
            p74.heading = (short)(short) -32233;
            p74.throttle = (ushort)(ushort)8506;
            p74.alt = (float)5.8868665E37F;
            p74.climb = (float) -1.6468377E38F;
            LoopBackDemoChannel.instance.send(p74); //===============================
            COMMAND_INT p75 = LoopBackDemoChannel.new_COMMAND_INT();
            PH.setPack(p75);
            p75.target_system = (byte)(byte)0;
            p75.target_component = (byte)(byte)91;
            p75.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT;
            p75.command = (MAV_CMD)MAV_CMD.MAV_CMD_CONDITION_CHANGE_ALT;
            p75.current = (byte)(byte)43;
            p75.autocontinue = (byte)(byte)25;
            p75.param1 = (float) -3.6033077E37F;
            p75.param2 = (float)2.335172E38F;
            p75.param3 = (float)3.0535537E38F;
            p75.param4 = (float)2.140201E37F;
            p75.x = (int) -696862900;
            p75.y = (int) -1185790653;
            p75.z = (float) -2.0183835E38F;
            LoopBackDemoChannel.instance.send(p75); //===============================
            COMMAND_LONG p76 = LoopBackDemoChannel.new_COMMAND_LONG();
            PH.setPack(p76);
            p76.target_system = (byte)(byte)33;
            p76.target_component = (byte)(byte)144;
            p76.command = (MAV_CMD)MAV_CMD.MAV_CMD_SPATIAL_USER_5;
            p76.confirmation = (byte)(byte)96;
            p76.param1 = (float)1.4969812E38F;
            p76.param2 = (float) -1.5221045E38F;
            p76.param3 = (float) -1.9607024E38F;
            p76.param4 = (float) -9.353271E37F;
            p76.param5 = (float)8.136105E37F;
            p76.param6 = (float)3.3079985E38F;
            p76.param7 = (float)2.0947013E38F;
            LoopBackDemoChannel.instance.send(p76); //===============================
            COMMAND_ACK p77 = LoopBackDemoChannel.new_COMMAND_ACK();
            PH.setPack(p77);
            p77.command = (MAV_CMD)MAV_CMD.MAV_CMD_PREFLIGHT_UAVCAN;
            p77.result = (MAV_RESULT)MAV_RESULT.MAV_RESULT_UNSUPPORTED;
            p77.progress_SET((byte)(byte)194, PH);
            p77.result_param2_SET((int)1674927052, PH);
            p77.target_system_SET((byte)(byte)134, PH);
            p77.target_component_SET((byte)(byte)138, PH);
            LoopBackDemoChannel.instance.send(p77); //===============================
            MANUAL_SETPOINT p81 = LoopBackDemoChannel.new_MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.time_boot_ms = (uint)769892498U;
            p81.roll = (float) -1.4640719E38F;
            p81.pitch = (float) -1.4289913E38F;
            p81.yaw = (float)2.7602147E38F;
            p81.thrust = (float) -1.64847E38F;
            p81.mode_switch = (byte)(byte)77;
            p81.manual_override_switch = (byte)(byte)7;
            LoopBackDemoChannel.instance.send(p81); //===============================
            SET_ATTITUDE_TARGET p82 = LoopBackDemoChannel.new_SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.time_boot_ms = (uint)3171262404U;
            p82.target_system = (byte)(byte)184;
            p82.target_component = (byte)(byte)13;
            p82.type_mask = (byte)(byte)18;
            p82.q_SET(new float[4], 0);
            p82.body_roll_rate = (float) -1.7157612E38F;
            p82.body_pitch_rate = (float) -1.0958746E38F;
            p82.body_yaw_rate = (float) -9.575139E37F;
            p82.thrust = (float)2.6022155E38F;
            LoopBackDemoChannel.instance.send(p82); //===============================
            ATTITUDE_TARGET p83 = LoopBackDemoChannel.new_ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.time_boot_ms = (uint)150077347U;
            p83.type_mask = (byte)(byte)67;
            p83.q_SET(new float[4], 0);
            p83.body_roll_rate = (float)2.767051E38F;
            p83.body_pitch_rate = (float)2.7147022E38F;
            p83.body_yaw_rate = (float) -4.51621E35F;
            p83.thrust = (float)1.5915881E38F;
            LoopBackDemoChannel.instance.send(p83); //===============================
            SET_POSITION_TARGET_LOCAL_NED p84 = LoopBackDemoChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.time_boot_ms = (uint)1941615041U;
            p84.target_system = (byte)(byte)158;
            p84.target_component = (byte)(byte)230;
            p84.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p84.type_mask = (ushort)(ushort)13999;
            p84.x = (float) -1.3138412E38F;
            p84.y = (float)2.8571376E38F;
            p84.z = (float) -3.106502E38F;
            p84.vx = (float)3.1411504E38F;
            p84.vy = (float)1.912402E38F;
            p84.vz = (float)1.894997E38F;
            p84.afx = (float)8.707573E37F;
            p84.afy = (float)2.8475915E38F;
            p84.afz = (float) -2.0847226E38F;
            p84.yaw = (float)3.3766032E38F;
            p84.yaw_rate = (float)3.0329935E37F;
            LoopBackDemoChannel.instance.send(p84); //===============================
            SET_POSITION_TARGET_GLOBAL_INT p86 = LoopBackDemoChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.time_boot_ms = (uint)1691640488U;
            p86.target_system = (byte)(byte)150;
            p86.target_component = (byte)(byte)183;
            p86.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT;
            p86.type_mask = (ushort)(ushort)41366;
            p86.lat_int = (int) -1299080071;
            p86.lon_int = (int)773655831;
            p86.alt = (float) -6.989276E37F;
            p86.vx = (float)3.2979857E36F;
            p86.vy = (float)2.3647026E38F;
            p86.vz = (float)1.6130741E36F;
            p86.afx = (float) -5.732215E37F;
            p86.afy = (float)1.8823465E37F;
            p86.afz = (float) -2.8834808E38F;
            p86.yaw = (float) -3.2005379E38F;
            p86.yaw_rate = (float)1.8109938E38F;
            LoopBackDemoChannel.instance.send(p86); //===============================
            POSITION_TARGET_GLOBAL_INT p87 = LoopBackDemoChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.time_boot_ms = (uint)1601423760U;
            p87.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p87.type_mask = (ushort)(ushort)5686;
            p87.lat_int = (int) -694022784;
            p87.lon_int = (int) -1888303934;
            p87.alt = (float)1.603074E37F;
            p87.vx = (float) -1.9998198E38F;
            p87.vy = (float)1.876373E38F;
            p87.vz = (float)3.3211912E38F;
            p87.afx = (float)2.237635E38F;
            p87.afy = (float)2.8901935E38F;
            p87.afz = (float) -1.4532364E38F;
            p87.yaw = (float) -5.6263866E37F;
            p87.yaw_rate = (float) -2.1003279E38F;
            LoopBackDemoChannel.instance.send(p87); //===============================
            LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.time_boot_ms = (uint)3714977891U;
            p89.x = (float)9.398515E37F;
            p89.y = (float) -2.9085896E38F;
            p89.z = (float)7.069772E37F;
            p89.roll = (float)3.0830747E38F;
            p89.pitch = (float)2.3909833E38F;
            p89.yaw = (float) -2.1047856E38F;
            LoopBackDemoChannel.instance.send(p89); //===============================
            HIL_STATE p90 = LoopBackDemoChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.time_usec = (ulong)1921063025858436217L;
            p90.roll = (float) -1.088883E38F;
            p90.pitch = (float)1.1101408E38F;
            p90.yaw = (float)3.3426421E38F;
            p90.rollspeed = (float) -1.159104E38F;
            p90.pitchspeed = (float)5.6540525E36F;
            p90.yawspeed = (float)3.038226E38F;
            p90.lat = (int) -545458972;
            p90.lon = (int)1093742782;
            p90.alt = (int) -1108565070;
            p90.vx = (short)(short)30360;
            p90.vy = (short)(short)787;
            p90.vz = (short)(short) -9008;
            p90.xacc = (short)(short) -25712;
            p90.yacc = (short)(short) -12505;
            p90.zacc = (short)(short)22593;
            LoopBackDemoChannel.instance.send(p90); //===============================
            HIL_CONTROLS p91 = LoopBackDemoChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.time_usec = (ulong)6487139528862818203L;
            p91.roll_ailerons = (float)3.0649413E37F;
            p91.pitch_elevator = (float)3.26363E38F;
            p91.yaw_rudder = (float)1.7612662E38F;
            p91.throttle = (float)1.9183874E38F;
            p91.aux1 = (float) -2.7094632E38F;
            p91.aux2 = (float) -8.58718E37F;
            p91.aux3 = (float) -2.70548E38F;
            p91.aux4 = (float)1.3631663E38F;
            p91.mode = (MAV_MODE)MAV_MODE.MAV_MODE_MANUAL_ARMED;
            p91.nav_mode = (byte)(byte)95;
            LoopBackDemoChannel.instance.send(p91); //===============================
            HIL_RC_INPUTS_RAW p92 = LoopBackDemoChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.time_usec = (ulong)6444855988549062395L;
            p92.chan1_raw = (ushort)(ushort)37639;
            p92.chan2_raw = (ushort)(ushort)58136;
            p92.chan3_raw = (ushort)(ushort)82;
            p92.chan4_raw = (ushort)(ushort)7820;
            p92.chan5_raw = (ushort)(ushort)60695;
            p92.chan6_raw = (ushort)(ushort)29158;
            p92.chan7_raw = (ushort)(ushort)6842;
            p92.chan8_raw = (ushort)(ushort)46046;
            p92.chan9_raw = (ushort)(ushort)5563;
            p92.chan10_raw = (ushort)(ushort)57913;
            p92.chan11_raw = (ushort)(ushort)57023;
            p92.chan12_raw = (ushort)(ushort)57260;
            p92.rssi = (byte)(byte)222;
            LoopBackDemoChannel.instance.send(p92); //===============================
            HIL_ACTUATOR_CONTROLS p93 = LoopBackDemoChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.time_usec = (ulong)7376932863667676522L;
            p93.controls_SET(new float[16], 0);
            p93.mode = (MAV_MODE)MAV_MODE.MAV_MODE_TEST_ARMED;
            p93.flags = (ulong)2642741588046206110L;
            LoopBackDemoChannel.instance.send(p93); //===============================
            OPTICAL_FLOW p100 = LoopBackDemoChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.time_usec = (ulong)7840616225046051170L;
            p100.sensor_id = (byte)(byte)32;
            p100.flow_x = (short)(short) -22111;
            p100.flow_y = (short)(short) -15171;
            p100.flow_comp_m_x = (float) -2.942482E38F;
            p100.flow_comp_m_y = (float) -2.0092183E37F;
            p100.quality = (byte)(byte)123;
            p100.ground_distance = (float)2.0886792E36F;
            p100.flow_rate_x_SET((float)2.3425027E38F, PH);
            p100.flow_rate_y_SET((float) -9.374468E37F, PH);
            LoopBackDemoChannel.instance.send(p100); //===============================
            GLOBAL_VISION_POSITION_ESTIMATE p101 = LoopBackDemoChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.usec = (ulong)1526600338005156236L;
            p101.x = (float)2.8542861E38F;
            p101.y = (float)3.2888278E38F;
            p101.z = (float)6.917864E37F;
            p101.roll = (float)3.9171542E37F;
            p101.pitch = (float)3.2036287E38F;
            p101.yaw = (float)3.2852893E38F;
            LoopBackDemoChannel.instance.send(p101); //===============================
            VISION_POSITION_ESTIMATE p102 = LoopBackDemoChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.usec = (ulong)2631795508430469851L;
            p102.x = (float) -3.2409567E38F;
            p102.y = (float)2.6481702E38F;
            p102.z = (float) -1.3434862E38F;
            p102.roll = (float)2.5792536E38F;
            p102.pitch = (float)2.8838378E38F;
            p102.yaw = (float)1.7255036E37F;
            LoopBackDemoChannel.instance.send(p102); //===============================
            VISION_SPEED_ESTIMATE p103 = LoopBackDemoChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.usec = (ulong)460762535308713073L;
            p103.x = (float)1.9548576E38F;
            p103.y = (float)6.00165E37F;
            p103.z = (float) -2.9257974E38F;
            LoopBackDemoChannel.instance.send(p103); //===============================
            VICON_POSITION_ESTIMATE p104 = LoopBackDemoChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.usec = (ulong)8842981430737280448L;
            p104.x = (float) -1.6742319E38F;
            p104.y = (float)1.5522418E38F;
            p104.z = (float)2.2842723E38F;
            p104.roll = (float)2.447221E38F;
            p104.pitch = (float) -1.3406962E38F;
            p104.yaw = (float)1.9256876E38F;
            LoopBackDemoChannel.instance.send(p104); //===============================
            HIGHRES_IMU p105 = LoopBackDemoChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.time_usec = (ulong)9007482115006341683L;
            p105.xacc = (float) -3.2048006E38F;
            p105.yacc = (float) -1.2824713E38F;
            p105.zacc = (float) -9.785962E37F;
            p105.xgyro = (float) -1.9998393E38F;
            p105.ygyro = (float)2.5944548E38F;
            p105.zgyro = (float) -9.68806E37F;
            p105.xmag = (float) -1.7236446E38F;
            p105.ymag = (float) -2.2816153E38F;
            p105.zmag = (float)2.57457E38F;
            p105.abs_pressure = (float)2.3952009E38F;
            p105.diff_pressure = (float)1.6644885E38F;
            p105.pressure_alt = (float) -1.4910711E38F;
            p105.temperature = (float)4.0817203E37F;
            p105.fields_updated = (ushort)(ushort)47301;
            LoopBackDemoChannel.instance.send(p105); //===============================
            OPTICAL_FLOW_RAD p106 = LoopBackDemoChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.time_usec = (ulong)7880217624444908686L;
            p106.sensor_id = (byte)(byte)196;
            p106.integration_time_us = (uint)4248036847U;
            p106.integrated_x = (float) -2.7090257E38F;
            p106.integrated_y = (float) -2.9009298E38F;
            p106.integrated_xgyro = (float) -1.3872076E38F;
            p106.integrated_ygyro = (float) -3.0881871E38F;
            p106.integrated_zgyro = (float)2.6409977E38F;
            p106.temperature = (short)(short)29145;
            p106.quality = (byte)(byte)35;
            p106.time_delta_distance_us = (uint)2805319444U;
            p106.distance = (float) -4.0792056E37F;
            LoopBackDemoChannel.instance.send(p106); //===============================
            HIL_SENSOR p107 = LoopBackDemoChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.time_usec = (ulong)7421553069060286546L;
            p107.xacc = (float) -6.902079E37F;
            p107.yacc = (float)2.848542E38F;
            p107.zacc = (float)1.7113247E38F;
            p107.xgyro = (float) -3.607483E36F;
            p107.ygyro = (float) -3.3687176E38F;
            p107.zgyro = (float) -2.350081E38F;
            p107.xmag = (float)2.9081393E38F;
            p107.ymag = (float)9.509147E37F;
            p107.zmag = (float)5.9267934E37F;
            p107.abs_pressure = (float)1.6106909E38F;
            p107.diff_pressure = (float)1.6445749E38F;
            p107.pressure_alt = (float) -1.407364E38F;
            p107.temperature = (float)1.4636563E38F;
            p107.fields_updated = (uint)2107414520U;
            LoopBackDemoChannel.instance.send(p107); //===============================
            SIM_STATE p108 = LoopBackDemoChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.q1 = (float) -1.5119534E38F;
            p108.q2 = (float)4.6050684E37F;
            p108.q3 = (float) -1.6213208E38F;
            p108.q4 = (float) -3.2580557E38F;
            p108.roll = (float) -6.999519E37F;
            p108.pitch = (float) -3.2074033E38F;
            p108.yaw = (float)8.540493E37F;
            p108.xacc = (float)1.589267E38F;
            p108.yacc = (float)1.3882475E38F;
            p108.zacc = (float) -1.8839787E38F;
            p108.xgyro = (float)3.0904624E38F;
            p108.ygyro = (float)3.1020883E38F;
            p108.zgyro = (float)2.1658703E38F;
            p108.lat = (float)1.0611856E38F;
            p108.lon = (float) -8.196831E37F;
            p108.alt = (float) -1.4055108E38F;
            p108.std_dev_horz = (float) -2.6666028E38F;
            p108.std_dev_vert = (float) -6.9878824E37F;
            p108.vn = (float) -3.020474E38F;
            p108.ve = (float)2.4526996E37F;
            p108.vd = (float)1.4209163E37F;
            LoopBackDemoChannel.instance.send(p108); //===============================
            RADIO_STATUS p109 = LoopBackDemoChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.rssi = (byte)(byte)51;
            p109.remrssi = (byte)(byte)101;
            p109.txbuf = (byte)(byte)203;
            p109.noise = (byte)(byte)166;
            p109.remnoise = (byte)(byte)19;
            p109.rxerrors = (ushort)(ushort)807;
            p109.fixed_ = (ushort)(ushort)43922;
            LoopBackDemoChannel.instance.send(p109); //===============================
            FILE_TRANSFER_PROTOCOL p110 = LoopBackDemoChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_network = (byte)(byte)30;
            p110.target_system = (byte)(byte)49;
            p110.target_component = (byte)(byte)246;
            p110.payload_SET(new byte[251], 0);
            LoopBackDemoChannel.instance.send(p110); //===============================
            TIMESYNC p111 = LoopBackDemoChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.tc1 = (long) -3565718615815492824L;
            p111.ts1 = (long) -7499815173443077767L;
            LoopBackDemoChannel.instance.send(p111); //===============================
            CAMERA_TRIGGER p112 = LoopBackDemoChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.time_usec = (ulong)2059382650511181881L;
            p112.seq = (uint)3283910820U;
            LoopBackDemoChannel.instance.send(p112); //===============================
            HIL_GPS p113 = LoopBackDemoChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.time_usec = (ulong)1565697794414833506L;
            p113.fix_type = (byte)(byte)46;
            p113.lat = (int) -2135464514;
            p113.lon = (int)1953731205;
            p113.alt = (int) -363005589;
            p113.eph = (ushort)(ushort)26344;
            p113.epv = (ushort)(ushort)49181;
            p113.vel = (ushort)(ushort)36512;
            p113.vn = (short)(short) -32381;
            p113.ve = (short)(short) -21302;
            p113.vd = (short)(short)24192;
            p113.cog = (ushort)(ushort)22316;
            p113.satellites_visible = (byte)(byte)25;
            LoopBackDemoChannel.instance.send(p113); //===============================
            HIL_OPTICAL_FLOW p114 = LoopBackDemoChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.time_usec = (ulong)5722857231689513158L;
            p114.sensor_id = (byte)(byte)143;
            p114.integration_time_us = (uint)3686873503U;
            p114.integrated_x = (float) -8.662813E37F;
            p114.integrated_y = (float)7.360609E37F;
            p114.integrated_xgyro = (float)2.295975E38F;
            p114.integrated_ygyro = (float) -2.8198506E38F;
            p114.integrated_zgyro = (float) -1.7248035E38F;
            p114.temperature = (short)(short)921;
            p114.quality = (byte)(byte)94;
            p114.time_delta_distance_us = (uint)2458383533U;
            p114.distance = (float)1.1199992E38F;
            LoopBackDemoChannel.instance.send(p114); //===============================
            HIL_STATE_QUATERNION p115 = LoopBackDemoChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.time_usec = (ulong)326427225872312111L;
            p115.attitude_quaternion_SET(new float[4], 0);
            p115.rollspeed = (float)6.5990057E37F;
            p115.pitchspeed = (float) -2.7573548E37F;
            p115.yawspeed = (float)1.1942887E38F;
            p115.lat = (int) -739073624;
            p115.lon = (int) -622577305;
            p115.alt = (int) -2080245681;
            p115.vx = (short)(short)23847;
            p115.vy = (short)(short)22906;
            p115.vz = (short)(short) -10552;
            p115.ind_airspeed = (ushort)(ushort)49102;
            p115.true_airspeed = (ushort)(ushort)51961;
            p115.xacc = (short)(short) -10127;
            p115.yacc = (short)(short)10276;
            p115.zacc = (short)(short) -1440;
            LoopBackDemoChannel.instance.send(p115); //===============================
            SCALED_IMU2 p116 = LoopBackDemoChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.time_boot_ms = (uint)4054919709U;
            p116.xacc = (short)(short) -24490;
            p116.yacc = (short)(short)8114;
            p116.zacc = (short)(short)10286;
            p116.xgyro = (short)(short) -12452;
            p116.ygyro = (short)(short) -30964;
            p116.zgyro = (short)(short)22947;
            p116.xmag = (short)(short) -18961;
            p116.ymag = (short)(short)867;
            p116.zmag = (short)(short)16078;
            LoopBackDemoChannel.instance.send(p116); //===============================
            LOG_REQUEST_LIST p117 = LoopBackDemoChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.target_system = (byte)(byte)127;
            p117.target_component = (byte)(byte)89;
            p117.start = (ushort)(ushort)11094;
            p117.end = (ushort)(ushort)5331;
            LoopBackDemoChannel.instance.send(p117); //===============================
            LOG_ENTRY p118 = LoopBackDemoChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.id = (ushort)(ushort)53333;
            p118.num_logs = (ushort)(ushort)38043;
            p118.last_log_num = (ushort)(ushort)39575;
            p118.time_utc = (uint)4058414252U;
            p118.size = (uint)515802653U;
            LoopBackDemoChannel.instance.send(p118); //===============================
            LOG_REQUEST_DATA p119 = LoopBackDemoChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.target_system = (byte)(byte)3;
            p119.target_component = (byte)(byte)177;
            p119.id = (ushort)(ushort)22423;
            p119.ofs = (uint)2659590629U;
            p119.count = (uint)2227894389U;
            LoopBackDemoChannel.instance.send(p119); //===============================
            LOG_DATA p120 = LoopBackDemoChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.id = (ushort)(ushort)48094;
            p120.ofs = (uint)566881572U;
            p120.count = (byte)(byte)171;
            p120.data__SET(new byte[90], 0);
            LoopBackDemoChannel.instance.send(p120); //===============================
            LOG_ERASE p121 = LoopBackDemoChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_system = (byte)(byte)18;
            p121.target_component = (byte)(byte)49;
            LoopBackDemoChannel.instance.send(p121); //===============================
            LOG_REQUEST_END p122 = LoopBackDemoChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_system = (byte)(byte)230;
            p122.target_component = (byte)(byte)142;
            LoopBackDemoChannel.instance.send(p122); //===============================
            GPS_INJECT_DATA p123 = LoopBackDemoChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.target_system = (byte)(byte)203;
            p123.target_component = (byte)(byte)155;
            p123.len = (byte)(byte)191;
            p123.data__SET(new byte[110], 0);
            LoopBackDemoChannel.instance.send(p123); //===============================
            GPS2_RAW p124 = LoopBackDemoChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.time_usec = (ulong)6221832022075586963L;
            p124.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX;
            p124.lat = (int) -10659784;
            p124.lon = (int) -1959427357;
            p124.alt = (int)1307226222;
            p124.eph = (ushort)(ushort)9673;
            p124.epv = (ushort)(ushort)21201;
            p124.vel = (ushort)(ushort)40734;
            p124.cog = (ushort)(ushort)19676;
            p124.satellites_visible = (byte)(byte)233;
            p124.dgps_numch = (byte)(byte)3;
            p124.dgps_age = (uint)3115973255U;
            LoopBackDemoChannel.instance.send(p124); //===============================
            POWER_STATUS p125 = LoopBackDemoChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vcc = (ushort)(ushort)63792;
            p125.Vservo = (ushort)(ushort)57269;
            p125.flags = (MAV_POWER_STATUS)MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED;
            LoopBackDemoChannel.instance.send(p125); //===============================
            SERIAL_CONTROL p126 = LoopBackDemoChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.device = (SERIAL_CONTROL_DEV)SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1;
            p126.flags = (SERIAL_CONTROL_FLAG)SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY;
            p126.timeout = (ushort)(ushort)5159;
            p126.baudrate = (uint)3601717613U;
            p126.count = (byte)(byte)35;
            p126.data__SET(new byte[70], 0);
            LoopBackDemoChannel.instance.send(p126); //===============================
            GPS_RTK p127 = LoopBackDemoChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.time_last_baseline_ms = (uint)925068938U;
            p127.rtk_receiver_id = (byte)(byte)150;
            p127.wn = (ushort)(ushort)32748;
            p127.tow = (uint)2457973956U;
            p127.rtk_health = (byte)(byte)204;
            p127.rtk_rate = (byte)(byte)139;
            p127.nsats = (byte)(byte)230;
            p127.baseline_coords_type = (byte)(byte)244;
            p127.baseline_a_mm = (int)342128686;
            p127.baseline_b_mm = (int)1096815760;
            p127.baseline_c_mm = (int)850125655;
            p127.accuracy = (uint)1510339253U;
            p127.iar_num_hypotheses = (int)65487570;
            LoopBackDemoChannel.instance.send(p127); //===============================
            GPS2_RTK p128 = LoopBackDemoChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.time_last_baseline_ms = (uint)2861858719U;
            p128.rtk_receiver_id = (byte)(byte)188;
            p128.wn = (ushort)(ushort)17942;
            p128.tow = (uint)2066758205U;
            p128.rtk_health = (byte)(byte)56;
            p128.rtk_rate = (byte)(byte)5;
            p128.nsats = (byte)(byte)128;
            p128.baseline_coords_type = (byte)(byte)95;
            p128.baseline_a_mm = (int)1559889542;
            p128.baseline_b_mm = (int)1227803708;
            p128.baseline_c_mm = (int)716245429;
            p128.accuracy = (uint)1638481478U;
            p128.iar_num_hypotheses = (int)80162309;
            LoopBackDemoChannel.instance.send(p128); //===============================
            SCALED_IMU3 p129 = LoopBackDemoChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.time_boot_ms = (uint)529714045U;
            p129.xacc = (short)(short)11467;
            p129.yacc = (short)(short) -5797;
            p129.zacc = (short)(short) -21247;
            p129.xgyro = (short)(short) -25656;
            p129.ygyro = (short)(short) -23076;
            p129.zgyro = (short)(short)30287;
            p129.xmag = (short)(short) -7516;
            p129.ymag = (short)(short) -32517;
            p129.zmag = (short)(short) -13860;
            LoopBackDemoChannel.instance.send(p129); //===============================
            DATA_TRANSMISSION_HANDSHAKE p130 = LoopBackDemoChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.type = (byte)(byte)227;
            p130.size = (uint)903309992U;
            p130.width = (ushort)(ushort)31684;
            p130.height = (ushort)(ushort)48473;
            p130.packets = (ushort)(ushort)33069;
            p130.payload = (byte)(byte)191;
            p130.jpg_quality = (byte)(byte)187;
            LoopBackDemoChannel.instance.send(p130); //===============================
            ENCAPSULATED_DATA p131 = LoopBackDemoChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.seqnr = (ushort)(ushort)7183;
            p131.data__SET(new byte[253], 0);
            LoopBackDemoChannel.instance.send(p131); //===============================
            DISTANCE_SENSOR p132 = LoopBackDemoChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.time_boot_ms = (uint)487244389U;
            p132.min_distance = (ushort)(ushort)53582;
            p132.max_distance = (ushort)(ushort)49335;
            p132.current_distance = (ushort)(ushort)26118;
            p132.type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN;
            p132.id = (byte)(byte)180;
            p132.orientation = (MAV_SENSOR_ORIENTATION)MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_270_YAW_45;
            p132.covariance = (byte)(byte)218;
            LoopBackDemoChannel.instance.send(p132); //===============================
            TERRAIN_REQUEST p133 = LoopBackDemoChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.lat = (int) -2138626282;
            p133.lon = (int)1345051171;
            p133.grid_spacing = (ushort)(ushort)62209;
            p133.mask = (ulong)1289034786033518506L;
            LoopBackDemoChannel.instance.send(p133); //===============================
            TERRAIN_DATA p134 = LoopBackDemoChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.lat = (int) -1254937794;
            p134.lon = (int)381294735;
            p134.grid_spacing = (ushort)(ushort)49096;
            p134.gridbit = (byte)(byte)44;
            p134.data__SET(new short[16], 0);
            LoopBackDemoChannel.instance.send(p134); //===============================
            TERRAIN_CHECK p135 = LoopBackDemoChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lat = (int) -2034688449;
            p135.lon = (int) -1208963139;
            LoopBackDemoChannel.instance.send(p135); //===============================
            TERRAIN_REPORT p136 = LoopBackDemoChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.lat = (int)745217964;
            p136.lon = (int) -89863813;
            p136.spacing = (ushort)(ushort)31708;
            p136.terrain_height = (float)8.659075E37F;
            p136.current_height = (float) -9.597505E37F;
            p136.pending = (ushort)(ushort)25801;
            p136.loaded = (ushort)(ushort)41547;
            LoopBackDemoChannel.instance.send(p136); //===============================
            SCALED_PRESSURE2 p137 = LoopBackDemoChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.time_boot_ms = (uint)2443373399U;
            p137.press_abs = (float) -1.7428774E38F;
            p137.press_diff = (float) -1.4495148E38F;
            p137.temperature = (short)(short) -2926;
            LoopBackDemoChannel.instance.send(p137); //===============================
            ATT_POS_MOCAP p138 = LoopBackDemoChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.time_usec = (ulong)8471984744506320909L;
            p138.q_SET(new float[4], 0);
            p138.x = (float) -2.7399554E38F;
            p138.y = (float) -1.2397579E38F;
            p138.z = (float) -2.981427E38F;
            LoopBackDemoChannel.instance.send(p138); //===============================
            SET_ACTUATOR_CONTROL_TARGET p139 = LoopBackDemoChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.time_usec = (ulong)7648206226117857967L;
            p139.group_mlx = (byte)(byte)23;
            p139.target_system = (byte)(byte)53;
            p139.target_component = (byte)(byte)48;
            p139.controls_SET(new float[8], 0);
            LoopBackDemoChannel.instance.send(p139); //===============================
            ACTUATOR_CONTROL_TARGET p140 = LoopBackDemoChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.time_usec = (ulong)307478492770507507L;
            p140.group_mlx = (byte)(byte)136;
            p140.controls_SET(new float[8], 0);
            LoopBackDemoChannel.instance.send(p140); //===============================
            ALTITUDE p141 = LoopBackDemoChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.time_usec = (ulong)3037452417482979364L;
            p141.altitude_monotonic = (float) -1.7992862E38F;
            p141.altitude_amsl = (float) -2.6493934E38F;
            p141.altitude_local = (float) -1.8443726E38F;
            p141.altitude_relative = (float)1.6686078E38F;
            p141.altitude_terrain = (float)2.313707E38F;
            p141.bottom_clearance = (float)2.1061844E38F;
            LoopBackDemoChannel.instance.send(p141); //===============================
            RESOURCE_REQUEST p142 = LoopBackDemoChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.request_id = (byte)(byte)191;
            p142.uri_type = (byte)(byte)188;
            p142.uri_SET(new byte[120], 0);
            p142.transfer_type = (byte)(byte)117;
            p142.storage_SET(new byte[120], 0);
            LoopBackDemoChannel.instance.send(p142); //===============================
            SCALED_PRESSURE3 p143 = LoopBackDemoChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.time_boot_ms = (uint)3104751528U;
            p143.press_abs = (float) -2.7561898E38F;
            p143.press_diff = (float)1.9255722E38F;
            p143.temperature = (short)(short) -25882;
            LoopBackDemoChannel.instance.send(p143); //===============================
            FOLLOW_TARGET p144 = LoopBackDemoChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.timestamp = (ulong)7005531372813514057L;
            p144.est_capabilities = (byte)(byte)53;
            p144.lat = (int) -345711258;
            p144.lon = (int)1473532546;
            p144.alt = (float)2.478207E38F;
            p144.vel_SET(new float[3], 0);
            p144.acc_SET(new float[3], 0);
            p144.attitude_q_SET(new float[4], 0);
            p144.rates_SET(new float[3], 0);
            p144.position_cov_SET(new float[3], 0);
            p144.custom_state = (ulong)1772363961672251230L;
            LoopBackDemoChannel.instance.send(p144); //===============================
            CONTROL_SYSTEM_STATE p146 = LoopBackDemoChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.time_usec = (ulong)1671867676702850784L;
            p146.x_acc = (float)1.987948E38F;
            p146.y_acc = (float) -3.3294796E38F;
            p146.z_acc = (float)3.2328733E38F;
            p146.x_vel = (float) -1.0387696E38F;
            p146.y_vel = (float) -9.379944E37F;
            p146.z_vel = (float) -1.1371691E38F;
            p146.x_pos = (float) -2.6841781E38F;
            p146.y_pos = (float)2.7586176E38F;
            p146.z_pos = (float)1.337232E38F;
            p146.airspeed = (float)1.8761233E38F;
            p146.vel_variance_SET(new float[3], 0);
            p146.pos_variance_SET(new float[3], 0);
            p146.q_SET(new float[4], 0);
            p146.roll_rate = (float)2.9821138E38F;
            p146.pitch_rate = (float) -1.2615429E38F;
            p146.yaw_rate = (float) -2.9318032E38F;
            LoopBackDemoChannel.instance.send(p146); //===============================
            BATTERY_STATUS p147 = LoopBackDemoChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.id = (byte)(byte)79;
            p147.battery_function = (MAV_BATTERY_FUNCTION)MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_AVIONICS;
            p147.type = (MAV_BATTERY_TYPE)MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LION;
            p147.temperature = (short)(short) -4437;
            p147.voltages_SET(new ushort[10], 0);
            p147.current_battery = (short)(short) -28896;
            p147.current_consumed = (int) -23525314;
            p147.energy_consumed = (int)450208305;
            p147.battery_remaining = (sbyte)(sbyte) - 37;
            LoopBackDemoChannel.instance.send(p147); //===============================
            AUTOPILOT_VERSION p148 = LoopBackDemoChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY)MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY;
            p148.flight_sw_version = (uint)1536277083U;
            p148.middleware_sw_version = (uint)2488228281U;
            p148.os_sw_version = (uint)2344695251U;
            p148.board_version = (uint)2740695272U;
            p148.flight_custom_version_SET(new byte[8], 0);
            p148.middleware_custom_version_SET(new byte[8], 0);
            p148.os_custom_version_SET(new byte[8], 0);
            p148.vendor_id = (ushort)(ushort)16286;
            p148.product_id = (ushort)(ushort)5497;
            p148.uid = (ulong)2897451614436681092L;
            p148.uid2_SET(new byte[18], 0, PH);
            LoopBackDemoChannel.instance.send(p148); //===============================
            LANDING_TARGET p149 = LoopBackDemoChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.time_usec = (ulong)2352151650143667218L;
            p149.target_num = (byte)(byte)39;
            p149.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
            p149.angle_x = (float) -3.0941751E37F;
            p149.angle_y = (float) -3.0630803E38F;
            p149.distance = (float) -3.3693202E38F;
            p149.size_x = (float) -5.272755E37F;
            p149.size_y = (float) -2.541705E38F;
            p149.x_SET((float)1.2777788E38F, PH);
            p149.y_SET((float)8.952412E37F, PH);
            p149.z_SET((float) -1.1191353E38F, PH);
            p149.q_SET(new float[4], 0, PH);
            p149.type = (LANDING_TARGET_TYPE)LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER;
            p149.position_valid_SET((byte)(byte)164, PH);
            LoopBackDemoChannel.instance.send(p149); //===============================
            SENSOR_OFFSETS p150 = LoopBackDemoChannel.new_SENSOR_OFFSETS();
            PH.setPack(p150);
            p150.mag_ofs_x = (short)(short) -25567;
            p150.mag_ofs_y = (short)(short) -15948;
            p150.mag_ofs_z = (short)(short) -12486;
            p150.mag_declination = (float) -1.7563968E38F;
            p150.raw_press = (int) -961549737;
            p150.raw_temp = (int) -1437845632;
            p150.gyro_cal_x = (float)2.196445E38F;
            p150.gyro_cal_y = (float)2.4314776E38F;
            p150.gyro_cal_z = (float)1.8657484E38F;
            p150.accel_cal_x = (float)4.1848883E37F;
            p150.accel_cal_y = (float) -2.65363E38F;
            p150.accel_cal_z = (float)4.5329725E37F;
            LoopBackDemoChannel.instance.send(p150); //===============================
            SET_MAG_OFFSETS p151 = LoopBackDemoChannel.new_SET_MAG_OFFSETS();
            PH.setPack(p151);
            p151.target_system = (byte)(byte)17;
            p151.target_component = (byte)(byte)225;
            p151.mag_ofs_x = (short)(short) -7148;
            p151.mag_ofs_y = (short)(short) -27118;
            p151.mag_ofs_z = (short)(short) -20718;
            LoopBackDemoChannel.instance.send(p151); //===============================
            MEMINFO p152 = LoopBackDemoChannel.new_MEMINFO();
            PH.setPack(p152);
            p152.brkval = (ushort)(ushort)1630;
            p152.freemem = (ushort)(ushort)55132;
            p152.freemem32_SET((uint)3060481305U, PH);
            LoopBackDemoChannel.instance.send(p152); //===============================
            AP_ADC p153 = LoopBackDemoChannel.new_AP_ADC();
            PH.setPack(p153);
            p153.adc1 = (ushort)(ushort)62487;
            p153.adc2 = (ushort)(ushort)51495;
            p153.adc3 = (ushort)(ushort)22730;
            p153.adc4 = (ushort)(ushort)8927;
            p153.adc5 = (ushort)(ushort)10591;
            p153.adc6 = (ushort)(ushort)23929;
            LoopBackDemoChannel.instance.send(p153); //===============================
            DIGICAM_CONFIGURE p154 = LoopBackDemoChannel.new_DIGICAM_CONFIGURE();
            PH.setPack(p154);
            p154.target_system = (byte)(byte)19;
            p154.target_component = (byte)(byte)112;
            p154.mode = (byte)(byte)56;
            p154.shutter_speed = (ushort)(ushort)15479;
            p154.aperture = (byte)(byte)125;
            p154.iso = (byte)(byte)165;
            p154.exposure_type = (byte)(byte)109;
            p154.command_id = (byte)(byte)79;
            p154.engine_cut_off = (byte)(byte)55;
            p154.extra_param = (byte)(byte)185;
            p154.extra_value = (float) -1.0787468E38F;
            LoopBackDemoChannel.instance.send(p154); //===============================
            DIGICAM_CONTROL p155 = LoopBackDemoChannel.new_DIGICAM_CONTROL();
            PH.setPack(p155);
            p155.target_system = (byte)(byte)216;
            p155.target_component = (byte)(byte)117;
            p155.session = (byte)(byte)25;
            p155.zoom_pos = (byte)(byte)94;
            p155.zoom_step = (sbyte)(sbyte)36;
            p155.focus_lock = (byte)(byte)163;
            p155.shot = (byte)(byte)125;
            p155.command_id = (byte)(byte)55;
            p155.extra_param = (byte)(byte)18;
            p155.extra_value = (float)6.2492335E37F;
            LoopBackDemoChannel.instance.send(p155); //===============================
            MOUNT_CONFIGURE p156 = LoopBackDemoChannel.new_MOUNT_CONFIGURE();
            PH.setPack(p156);
            p156.target_system = (byte)(byte)36;
            p156.target_component = (byte)(byte)248;
            p156.mount_mode = (MAV_MOUNT_MODE)MAV_MOUNT_MODE.MAV_MOUNT_MODE_GPS_POINT;
            p156.stab_roll = (byte)(byte)196;
            p156.stab_pitch = (byte)(byte)84;
            p156.stab_yaw = (byte)(byte)179;
            LoopBackDemoChannel.instance.send(p156); //===============================
            MOUNT_CONTROL p157 = LoopBackDemoChannel.new_MOUNT_CONTROL();
            PH.setPack(p157);
            p157.target_system = (byte)(byte)130;
            p157.target_component = (byte)(byte)26;
            p157.input_a = (int)2117133137;
            p157.input_b = (int) -1417690439;
            p157.input_c = (int) -1984630581;
            p157.save_position = (byte)(byte)155;
            LoopBackDemoChannel.instance.send(p157); //===============================
            MOUNT_STATUS p158 = LoopBackDemoChannel.new_MOUNT_STATUS();
            PH.setPack(p158);
            p158.target_system = (byte)(byte)6;
            p158.target_component = (byte)(byte)140;
            p158.pointing_a = (int) -677522062;
            p158.pointing_b = (int)830691494;
            p158.pointing_c = (int) -965143269;
            LoopBackDemoChannel.instance.send(p158); //===============================
            FENCE_POINT p160 = LoopBackDemoChannel.new_FENCE_POINT();
            PH.setPack(p160);
            p160.target_system = (byte)(byte)0;
            p160.target_component = (byte)(byte)165;
            p160.idx = (byte)(byte)80;
            p160.count = (byte)(byte)184;
            p160.lat = (float) -1.0746984E38F;
            p160.lng = (float)2.081992E38F;
            LoopBackDemoChannel.instance.send(p160); //===============================
            FENCE_FETCH_POINT p161 = LoopBackDemoChannel.new_FENCE_FETCH_POINT();
            PH.setPack(p161);
            p161.target_system = (byte)(byte)236;
            p161.target_component = (byte)(byte)84;
            p161.idx = (byte)(byte)139;
            LoopBackDemoChannel.instance.send(p161); //===============================
            FENCE_STATUS p162 = LoopBackDemoChannel.new_FENCE_STATUS();
            PH.setPack(p162);
            p162.breach_status = (byte)(byte)173;
            p162.breach_count = (ushort)(ushort)5917;
            p162.breach_type = (FENCE_BREACH)FENCE_BREACH.FENCE_BREACH_BOUNDARY;
            p162.breach_time = (uint)1799216038U;
            LoopBackDemoChannel.instance.send(p162); //===============================
            AHRS p163 = LoopBackDemoChannel.new_AHRS();
            PH.setPack(p163);
            p163.omegaIx = (float) -4.272622E37F;
            p163.omegaIy = (float) -8.873114E37F;
            p163.omegaIz = (float) -5.4879926E37F;
            p163.accel_weight = (float)8.941107E37F;
            p163.renorm_val = (float)2.729969E38F;
            p163.error_rp = (float)2.8721348E38F;
            p163.error_yaw = (float)9.413972E37F;
            LoopBackDemoChannel.instance.send(p163); //===============================
            SIMSTATE p164 = LoopBackDemoChannel.new_SIMSTATE();
            PH.setPack(p164);
            p164.roll = (float) -2.2025068E38F;
            p164.pitch = (float) -3.3099897E38F;
            p164.yaw = (float)2.5583294E38F;
            p164.xacc = (float) -2.0613832E38F;
            p164.yacc = (float)1.8887233E38F;
            p164.zacc = (float)3.1199021E38F;
            p164.xgyro = (float)2.8016021E38F;
            p164.ygyro = (float) -1.1404278E37F;
            p164.zgyro = (float)2.9457152E38F;
            p164.lat = (int) -296602699;
            p164.lng = (int) -2016405018;
            LoopBackDemoChannel.instance.send(p164); //===============================
            HWSTATUS p165 = LoopBackDemoChannel.new_HWSTATUS();
            PH.setPack(p165);
            p165.Vcc = (ushort)(ushort)32646;
            p165.I2Cerr = (byte)(byte)155;
            LoopBackDemoChannel.instance.send(p165); //===============================
            RADIO p166 = LoopBackDemoChannel.new_RADIO();
            PH.setPack(p166);
            p166.rssi = (byte)(byte)59;
            p166.remrssi = (byte)(byte)35;
            p166.txbuf = (byte)(byte)156;
            p166.noise = (byte)(byte)63;
            p166.remnoise = (byte)(byte)87;
            p166.rxerrors = (ushort)(ushort)22712;
            p166.fixed_ = (ushort)(ushort)30926;
            LoopBackDemoChannel.instance.send(p166); //===============================
            LIMITS_STATUS p167 = LoopBackDemoChannel.new_LIMITS_STATUS();
            PH.setPack(p167);
            p167.limits_state = (LIMITS_STATE)LIMITS_STATE.LIMITS_RECOVERED;
            p167.last_trigger = (uint)4066690871U;
            p167.last_action = (uint)3783093768U;
            p167.last_recovery = (uint)842507325U;
            p167.last_clear = (uint)2285943036U;
            p167.breach_count = (ushort)(ushort)62141;
            p167.mods_enabled = (LIMIT_MODULE)LIMIT_MODULE.LIMIT_GPSLOCK;
            p167.mods_required = (LIMIT_MODULE)LIMIT_MODULE.LIMIT_GPSLOCK;
            p167.mods_triggered = (LIMIT_MODULE)LIMIT_MODULE.LIMIT_ALTITUDE;
            LoopBackDemoChannel.instance.send(p167); //===============================
            WIND p168 = LoopBackDemoChannel.new_WIND();
            PH.setPack(p168);
            p168.direction = (float)2.4678212E38F;
            p168.speed = (float)4.0799373E37F;
            p168.speed_z = (float)2.720389E38F;
            LoopBackDemoChannel.instance.send(p168); //===============================
            DATA16 p169 = LoopBackDemoChannel.new_DATA16();
            PH.setPack(p169);
            p169.type = (byte)(byte)209;
            p169.len = (byte)(byte)171;
            p169.data__SET(new byte[16], 0);
            LoopBackDemoChannel.instance.send(p169); //===============================
            DATA32 p170 = LoopBackDemoChannel.new_DATA32();
            PH.setPack(p170);
            p170.type = (byte)(byte)241;
            p170.len = (byte)(byte)45;
            p170.data__SET(new byte[32], 0);
            LoopBackDemoChannel.instance.send(p170); //===============================
            DATA64 p171 = LoopBackDemoChannel.new_DATA64();
            PH.setPack(p171);
            p171.type = (byte)(byte)92;
            p171.len = (byte)(byte)119;
            p171.data__SET(new byte[64], 0);
            LoopBackDemoChannel.instance.send(p171); //===============================
            DATA96 p172 = LoopBackDemoChannel.new_DATA96();
            PH.setPack(p172);
            p172.type = (byte)(byte)126;
            p172.len = (byte)(byte)154;
            p172.data__SET(new byte[96], 0);
            LoopBackDemoChannel.instance.send(p172); //===============================
            RANGEFINDER p173 = LoopBackDemoChannel.new_RANGEFINDER();
            PH.setPack(p173);
            p173.distance = (float)2.5705143E38F;
            p173.voltage = (float) -2.9514109E38F;
            LoopBackDemoChannel.instance.send(p173); //===============================
            AIRSPEED_AUTOCAL p174 = LoopBackDemoChannel.new_AIRSPEED_AUTOCAL();
            PH.setPack(p174);
            p174.vx = (float)2.703762E38F;
            p174.vy = (float)2.6562302E38F;
            p174.vz = (float)2.7460697E36F;
            p174.diff_pressure = (float)7.221098E37F;
            p174.EAS2TAS = (float)2.3769895E38F;
            p174.ratio = (float) -1.3337682E38F;
            p174.state_x = (float)2.3733178E38F;
            p174.state_y = (float)1.532264E38F;
            p174.state_z = (float)5.626599E37F;
            p174.Pax = (float) -2.069673E38F;
            p174.Pby = (float) -8.255336E37F;
            p174.Pcz = (float)1.427776E38F;
            LoopBackDemoChannel.instance.send(p174); //===============================
            RALLY_POINT p175 = LoopBackDemoChannel.new_RALLY_POINT();
            PH.setPack(p175);
            p175.target_system = (byte)(byte)167;
            p175.target_component = (byte)(byte)220;
            p175.idx = (byte)(byte)11;
            p175.count = (byte)(byte)199;
            p175.lat = (int)33498729;
            p175.lng = (int) -1425843552;
            p175.alt = (short)(short)12665;
            p175.break_alt = (short)(short) -3998;
            p175.land_dir = (ushort)(ushort)29459;
            p175.flags = (RALLY_FLAGS)RALLY_FLAGS.FAVORABLE_WIND;
            LoopBackDemoChannel.instance.send(p175); //===============================
            RALLY_FETCH_POINT p176 = LoopBackDemoChannel.new_RALLY_FETCH_POINT();
            PH.setPack(p176);
            p176.target_system = (byte)(byte)202;
            p176.target_component = (byte)(byte)204;
            p176.idx = (byte)(byte)205;
            LoopBackDemoChannel.instance.send(p176); //===============================
            COMPASSMOT_STATUS p177 = LoopBackDemoChannel.new_COMPASSMOT_STATUS();
            PH.setPack(p177);
            p177.throttle = (ushort)(ushort)35570;
            p177.current = (float)3.7033112E37F;
            p177.interference = (ushort)(ushort)36895;
            p177.CompensationX = (float)5.969707E37F;
            p177.CompensationY = (float)6.991473E37F;
            p177.CompensationZ = (float) -1.19677665E36F;
            LoopBackDemoChannel.instance.send(p177); //===============================
            AHRS2 p178 = LoopBackDemoChannel.new_AHRS2();
            PH.setPack(p178);
            p178.roll = (float)2.8580864E38F;
            p178.pitch = (float)2.3024741E38F;
            p178.yaw = (float)2.528646E38F;
            p178.altitude = (float) -8.764103E37F;
            p178.lat = (int)900080447;
            p178.lng = (int) -1448396498;
            LoopBackDemoChannel.instance.send(p178); //===============================
            CAMERA_STATUS p179 = LoopBackDemoChannel.new_CAMERA_STATUS();
            PH.setPack(p179);
            p179.time_usec = (ulong)5603326465541385596L;
            p179.target_system = (byte)(byte)126;
            p179.cam_idx = (byte)(byte)190;
            p179.img_idx = (ushort)(ushort)18093;
            p179.event_id = (CAMERA_STATUS_TYPES)CAMERA_STATUS_TYPES.CAMERA_STATUS_TYPE_DISCONNECT;
            p179.p1 = (float) -1.6569653E38F;
            p179.p2 = (float)5.5058325E37F;
            p179.p3 = (float)2.684026E38F;
            p179.p4 = (float) -2.6239412E38F;
            LoopBackDemoChannel.instance.send(p179); //===============================
            CAMERA_FEEDBACK p180 = LoopBackDemoChannel.new_CAMERA_FEEDBACK();
            PH.setPack(p180);
            p180.time_usec = (ulong)3079532376674863079L;
            p180.target_system = (byte)(byte)220;
            p180.cam_idx = (byte)(byte)163;
            p180.img_idx = (ushort)(ushort)41049;
            p180.lat = (int)1806968310;
            p180.lng = (int)1641101586;
            p180.alt_msl = (float)7.267995E37F;
            p180.alt_rel = (float) -2.1591019E38F;
            p180.roll = (float)2.6434285E37F;
            p180.pitch = (float) -1.9869407E38F;
            p180.yaw = (float) -2.1683633E37F;
            p180.foc_len = (float)8.802033E37F;
            p180.flags = (CAMERA_FEEDBACK_FLAGS)CAMERA_FEEDBACK_FLAGS.CAMERA_FEEDBACK_OPENLOOP;
            LoopBackDemoChannel.instance.send(p180); //===============================
            BATTERY2 p181 = LoopBackDemoChannel.new_BATTERY2();
            PH.setPack(p181);
            p181.voltage = (ushort)(ushort)55876;
            p181.current_battery = (short)(short) -17420;
            LoopBackDemoChannel.instance.send(p181); //===============================
            AHRS3 p182 = LoopBackDemoChannel.new_AHRS3();
            PH.setPack(p182);
            p182.roll = (float)2.3539444E38F;
            p182.pitch = (float)6.957547E37F;
            p182.yaw = (float)7.662418E37F;
            p182.altitude = (float) -1.3239789E38F;
            p182.lat = (int)1520413348;
            p182.lng = (int)1876178115;
            p182.v1 = (float) -1.4407807E38F;
            p182.v2 = (float)2.5114314E38F;
            p182.v3 = (float) -2.2223475E38F;
            p182.v4 = (float) -1.2072857E37F;
            LoopBackDemoChannel.instance.send(p182); //===============================
            AUTOPILOT_VERSION_REQUEST p183 = LoopBackDemoChannel.new_AUTOPILOT_VERSION_REQUEST();
            PH.setPack(p183);
            p183.target_system = (byte)(byte)185;
            p183.target_component = (byte)(byte)223;
            LoopBackDemoChannel.instance.send(p183); //===============================
            REMOTE_LOG_DATA_BLOCK p184 = LoopBackDemoChannel.new_REMOTE_LOG_DATA_BLOCK();
            PH.setPack(p184);
            p184.target_system = (byte)(byte)100;
            p184.target_component = (byte)(byte)125;
            p184.seqno = (MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS)MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS.MAV_REMOTE_LOG_DATA_BLOCK_STOP;
            p184.data__SET(new byte[200], 0);
            LoopBackDemoChannel.instance.send(p184); //===============================
            REMOTE_LOG_BLOCK_STATUS p185 = LoopBackDemoChannel.new_REMOTE_LOG_BLOCK_STATUS();
            PH.setPack(p185);
            p185.target_system = (byte)(byte)119;
            p185.target_component = (byte)(byte)179;
            p185.seqno = (uint)1936167985U;
            p185.status = (MAV_REMOTE_LOG_DATA_BLOCK_STATUSES)MAV_REMOTE_LOG_DATA_BLOCK_STATUSES.MAV_REMOTE_LOG_DATA_BLOCK_ACK;
            LoopBackDemoChannel.instance.send(p185); //===============================
            LED_CONTROL p186 = LoopBackDemoChannel.new_LED_CONTROL();
            PH.setPack(p186);
            p186.target_system = (byte)(byte)191;
            p186.target_component = (byte)(byte)117;
            p186.instance = (byte)(byte)159;
            p186.pattern = (byte)(byte)16;
            p186.custom_len = (byte)(byte)213;
            p186.custom_bytes_SET(new byte[24], 0);
            LoopBackDemoChannel.instance.send(p186); //===============================
            MAG_CAL_PROGRESS p191 = LoopBackDemoChannel.new_MAG_CAL_PROGRESS();
            PH.setPack(p191);
            p191.compass_id = (byte)(byte)178;
            p191.cal_mask = (byte)(byte)196;
            p191.cal_status = (MAG_CAL_STATUS)MAG_CAL_STATUS.MAG_CAL_RUNNING_STEP_TWO;
            p191.attempt = (byte)(byte)242;
            p191.completion_pct = (byte)(byte)207;
            p191.completion_mask_SET(new byte[10], 0);
            p191.direction_x = (float) -1.0817613E38F;
            p191.direction_y = (float) -2.8006278E38F;
            p191.direction_z = (float)1.0065432E38F;
            LoopBackDemoChannel.instance.send(p191); //===============================
            MAG_CAL_REPORT p192 = LoopBackDemoChannel.new_MAG_CAL_REPORT();
            PH.setPack(p192);
            p192.compass_id = (byte)(byte)132;
            p192.cal_mask = (byte)(byte)98;
            p192.cal_status = (MAG_CAL_STATUS)MAG_CAL_STATUS.MAG_CAL_RUNNING_STEP_ONE;
            p192.autosaved = (byte)(byte)182;
            p192.fitness = (float) -2.0465444E38F;
            p192.ofs_x = (float) -2.2175065E38F;
            p192.ofs_y = (float)2.590845E38F;
            p192.ofs_z = (float)2.3880698E38F;
            p192.diag_x = (float)1.6902132E38F;
            p192.diag_y = (float) -1.9771932E38F;
            p192.diag_z = (float)2.5827754E38F;
            p192.offdiag_x = (float) -1.1033368E38F;
            p192.offdiag_y = (float)1.7690656E38F;
            p192.offdiag_z = (float) -2.7849872E38F;
            LoopBackDemoChannel.instance.send(p192); //===============================
            EKF_STATUS_REPORT p193 = LoopBackDemoChannel.new_EKF_STATUS_REPORT();
            PH.setPack(p193);
            p193.flags = (EKF_STATUS_FLAGS)EKF_STATUS_FLAGS.EKF_POS_HORIZ_ABS;
            p193.velocity_variance = (float)3.010018E38F;
            p193.pos_horiz_variance = (float)2.1751533E38F;
            p193.pos_vert_variance = (float)1.869817E38F;
            p193.compass_variance = (float)1.1386623E38F;
            p193.terrain_alt_variance = (float) -2.394407E37F;
            LoopBackDemoChannel.instance.send(p193); //===============================
            PID_TUNING p194 = LoopBackDemoChannel.new_PID_TUNING();
            PH.setPack(p194);
            p194.axis = (PID_TUNING_AXIS)PID_TUNING_AXIS.PID_TUNING_YAW;
            p194.desired = (float)2.3851927E38F;
            p194.achieved = (float) -1.6030604E37F;
            p194.FF = (float) -2.7536225E38F;
            p194.P = (float)2.9570987E38F;
            p194.I = (float)5.5068606E36F;
            p194.D = (float)8.526762E37F;
            LoopBackDemoChannel.instance.send(p194); //===============================
            GIMBAL_REPORT p200 = LoopBackDemoChannel.new_GIMBAL_REPORT();
            PH.setPack(p200);
            p200.target_system = (byte)(byte)178;
            p200.target_component = (byte)(byte)151;
            p200.delta_time = (float) -8.793452E37F;
            p200.delta_angle_x = (float)1.0880318E38F;
            p200.delta_angle_y = (float) -5.277794E37F;
            p200.delta_angle_z = (float) -8.5847485E36F;
            p200.delta_velocity_x = (float) -7.6701547E37F;
            p200.delta_velocity_y = (float) -4.385293E37F;
            p200.delta_velocity_z = (float) -2.5897331E38F;
            p200.joint_roll = (float) -1.1985004E38F;
            p200.joint_el = (float)1.5248105E38F;
            p200.joint_az = (float) -2.092186E38F;
            LoopBackDemoChannel.instance.send(p200); //===============================
            GIMBAL_CONTROL p201 = LoopBackDemoChannel.new_GIMBAL_CONTROL();
            PH.setPack(p201);
            p201.target_system = (byte)(byte)191;
            p201.target_component = (byte)(byte)123;
            p201.demanded_rate_x = (float) -1.6316869E38F;
            p201.demanded_rate_y = (float) -2.4296008E38F;
            p201.demanded_rate_z = (float) -1.3770239E38F;
            LoopBackDemoChannel.instance.send(p201); //===============================
            GIMBAL_TORQUE_CMD_REPORT p214 = LoopBackDemoChannel.new_GIMBAL_TORQUE_CMD_REPORT();
            PH.setPack(p214);
            p214.target_system = (byte)(byte)132;
            p214.target_component = (byte)(byte)96;
            p214.rl_torque_cmd = (short)(short)32569;
            p214.el_torque_cmd = (short)(short)24157;
            p214.az_torque_cmd = (short)(short) -17031;
            LoopBackDemoChannel.instance.send(p214); //===============================
            GOPRO_HEARTBEAT p215 = LoopBackDemoChannel.new_GOPRO_HEARTBEAT();
            PH.setPack(p215);
            p215.status = (GOPRO_HEARTBEAT_STATUS)GOPRO_HEARTBEAT_STATUS.GOPRO_HEARTBEAT_STATUS_INCOMPATIBLE;
            p215.capture_mode = (GOPRO_CAPTURE_MODE)GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_BURST;
            p215.flags = (GOPRO_HEARTBEAT_FLAGS)GOPRO_HEARTBEAT_FLAGS.GOPRO_FLAG_RECORDING;
            LoopBackDemoChannel.instance.send(p215); //===============================
            GOPRO_GET_REQUEST p216 = LoopBackDemoChannel.new_GOPRO_GET_REQUEST();
            PH.setPack(p216);
            p216.target_system = (byte)(byte)156;
            p216.target_component = (byte)(byte)56;
            p216.cmd_id = (GOPRO_COMMAND)GOPRO_COMMAND.GOPRO_COMMAND_PROTUNE_EXPOSURE;
            LoopBackDemoChannel.instance.send(p216); //===============================
            GOPRO_GET_RESPONSE p217 = LoopBackDemoChannel.new_GOPRO_GET_RESPONSE();
            PH.setPack(p217);
            p217.cmd_id = (GOPRO_COMMAND)GOPRO_COMMAND.GOPRO_COMMAND_PHOTO_RESOLUTION;
            p217.status = (GOPRO_REQUEST_STATUS)GOPRO_REQUEST_STATUS.GOPRO_REQUEST_FAILED;
            p217.value_SET(new byte[4], 0);
            LoopBackDemoChannel.instance.send(p217); //===============================
            GOPRO_SET_REQUEST p218 = LoopBackDemoChannel.new_GOPRO_SET_REQUEST();
            PH.setPack(p218);
            p218.target_system = (byte)(byte)111;
            p218.target_component = (byte)(byte)201;
            p218.cmd_id = (GOPRO_COMMAND)GOPRO_COMMAND.GOPRO_COMMAND_PHOTO_BURST_RATE;
            p218.value_SET(new byte[4], 0);
            LoopBackDemoChannel.instance.send(p218); //===============================
            GOPRO_SET_RESPONSE p219 = LoopBackDemoChannel.new_GOPRO_SET_RESPONSE();
            PH.setPack(p219);
            p219.cmd_id = (GOPRO_COMMAND)GOPRO_COMMAND.GOPRO_COMMAND_MODEL;
            p219.status = (GOPRO_REQUEST_STATUS)GOPRO_REQUEST_STATUS.GOPRO_REQUEST_FAILED;
            LoopBackDemoChannel.instance.send(p219); //===============================
            RPM p226 = LoopBackDemoChannel.new_RPM();
            PH.setPack(p226);
            p226.rpm1 = (float)1.5344487E38F;
            p226.rpm2 = (float) -3.3106551E38F;
            LoopBackDemoChannel.instance.send(p226); //===============================
            ESTIMATOR_STATUS p230 = LoopBackDemoChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.time_usec = (ulong)6075034036071592918L;
            p230.flags = (ESTIMATOR_STATUS_FLAGS)ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ;
            p230.vel_ratio = (float)2.7151027E38F;
            p230.pos_horiz_ratio = (float)7.178414E37F;
            p230.pos_vert_ratio = (float)2.137574E38F;
            p230.mag_ratio = (float) -5.139758E37F;
            p230.hagl_ratio = (float)1.0182147E38F;
            p230.tas_ratio = (float) -2.1177448E38F;
            p230.pos_horiz_accuracy = (float)1.6263051E38F;
            p230.pos_vert_accuracy = (float)1.3324926E38F;
            LoopBackDemoChannel.instance.send(p230); //===============================
            WIND_COV p231 = LoopBackDemoChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.time_usec = (ulong)2291011255909792763L;
            p231.wind_x = (float) -1.1893964E38F;
            p231.wind_y = (float) -1.4846205E38F;
            p231.wind_z = (float)2.7557688E38F;
            p231.var_horiz = (float) -1.4921161E38F;
            p231.var_vert = (float) -1.1310917E37F;
            p231.wind_alt = (float)1.421996E38F;
            p231.horiz_accuracy = (float)4.362574E37F;
            p231.vert_accuracy = (float)1.7939307E38F;
            LoopBackDemoChannel.instance.send(p231); //===============================
            GPS_INPUT p232 = LoopBackDemoChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.time_usec = (ulong)8246299793567494029L;
            p232.gps_id = (byte)(byte)200;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS)GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY;
            p232.time_week_ms = (uint)4089966584U;
            p232.time_week = (ushort)(ushort)52788;
            p232.fix_type = (byte)(byte)30;
            p232.lat = (int) -1120222796;
            p232.lon = (int) -814270104;
            p232.alt = (float)3.2537242E37F;
            p232.hdop = (float)4.0035005E37F;
            p232.vdop = (float) -3.0546536E38F;
            p232.vn = (float)1.3300542E38F;
            p232.ve = (float)1.0585195E38F;
            p232.vd = (float) -1.6321884E38F;
            p232.speed_accuracy = (float)1.4510172E38F;
            p232.horiz_accuracy = (float) -3.1137746E38F;
            p232.vert_accuracy = (float) -1.0024201E38F;
            p232.satellites_visible = (byte)(byte)124;
            LoopBackDemoChannel.instance.send(p232); //===============================
            GPS_RTCM_DATA p233 = LoopBackDemoChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.flags = (byte)(byte)174;
            p233.len = (byte)(byte)199;
            p233.data__SET(new byte[180], 0);
            LoopBackDemoChannel.instance.send(p233); //===============================
            HIGH_LATENCY p234 = LoopBackDemoChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED;
            p234.custom_mode = (uint)4192519596U;
            p234.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND;
            p234.roll = (short)(short) -18902;
            p234.pitch = (short)(short) -16777;
            p234.heading = (ushort)(ushort)40385;
            p234.throttle = (sbyte)(sbyte) - 29;
            p234.heading_sp = (short)(short)19094;
            p234.latitude = (int)1923038339;
            p234.longitude = (int)399610637;
            p234.altitude_amsl = (short)(short)12629;
            p234.altitude_sp = (short)(short)31942;
            p234.airspeed = (byte)(byte)60;
            p234.airspeed_sp = (byte)(byte)48;
            p234.groundspeed = (byte)(byte)169;
            p234.climb_rate = (sbyte)(sbyte) - 112;
            p234.gps_nsat = (byte)(byte)197;
            p234.gps_fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX;
            p234.battery_remaining = (byte)(byte)211;
            p234.temperature = (sbyte)(sbyte) - 69;
            p234.temperature_air = (sbyte)(sbyte)125;
            p234.failsafe = (byte)(byte)171;
            p234.wp_num = (byte)(byte)60;
            p234.wp_distance = (ushort)(ushort)54328;
            LoopBackDemoChannel.instance.send(p234); //===============================
            VIBRATION p241 = LoopBackDemoChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.time_usec = (ulong)7989354879349935805L;
            p241.vibration_x = (float) -1.631284E38F;
            p241.vibration_y = (float)1.9329579E38F;
            p241.vibration_z = (float)1.0231135E38F;
            p241.clipping_0 = (uint)2146614340U;
            p241.clipping_1 = (uint)551513186U;
            p241.clipping_2 = (uint)4239511362U;
            LoopBackDemoChannel.instance.send(p241); //===============================
            HOME_POSITION p242 = LoopBackDemoChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.latitude = (int)729274053;
            p242.longitude = (int) -276524426;
            p242.altitude = (int)1797846160;
            p242.x = (float) -1.4775413E38F;
            p242.y = (float)1.4952006E38F;
            p242.z = (float) -2.6536073E38F;
            p242.q_SET(new float[4], 0);
            p242.approach_x = (float) -3.1464889E38F;
            p242.approach_y = (float)7.4964465E37F;
            p242.approach_z = (float) -1.4424004E38F;
            p242.time_usec_SET((ulong)5850779655884381474L, PH);
            LoopBackDemoChannel.instance.send(p242); //===============================
            SET_HOME_POSITION p243 = LoopBackDemoChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.target_system = (byte)(byte)96;
            p243.latitude = (int)535040502;
            p243.longitude = (int)1041428327;
            p243.altitude = (int)1134634094;
            p243.x = (float)3.2750066E38F;
            p243.y = (float) -2.2411973E38F;
            p243.z = (float) -3.0082842E38F;
            p243.q_SET(new float[4], 0);
            p243.approach_x = (float) -2.1134712E37F;
            p243.approach_y = (float)7.7193923E37F;
            p243.approach_z = (float) -1.0247936E38F;
            p243.time_usec_SET((ulong)2298915671605496076L, PH);
            LoopBackDemoChannel.instance.send(p243); //===============================
            MESSAGE_INTERVAL p244 = LoopBackDemoChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.message_id = (ushort)(ushort)2950;
            p244.interval_us = (int)1654012709;
            LoopBackDemoChannel.instance.send(p244); //===============================
            EXTENDED_SYS_STATE p245 = LoopBackDemoChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.vtol_state = (MAV_VTOL_STATE)MAV_VTOL_STATE.MAV_VTOL_STATE_FW;
            p245.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING;
            LoopBackDemoChannel.instance.send(p245); //===============================
            ADSB_VEHICLE p246 = LoopBackDemoChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.ICAO_address = (uint)1269598416U;
            p246.lat = (int)1656880076;
            p246.lon = (int)197659474;
            p246.altitude_type = (ADSB_ALTITUDE_TYPE)ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
            p246.altitude = (int)194843666;
            p246.heading = (ushort)(ushort)44219;
            p246.hor_velocity = (ushort)(ushort)54281;
            p246.ver_velocity = (short)(short) -28827;
            p246.callsign_SET("DEMO", PH);
            p246.emitter_type = (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_ULTRA_LIGHT;
            p246.tslc = (byte)(byte)25;
            p246.flags = (ADSB_FLAGS)ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING;
            p246.squawk = (ushort)(ushort)15044;
            LoopBackDemoChannel.instance.send(p246); //===============================
            COLLISION p247 = LoopBackDemoChannel.new_COLLISION();
            PH.setPack(p247);
            p247.src_ = (MAV_COLLISION_SRC)MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT;
            p247.id = (uint)451105424U;
            p247.action = (MAV_COLLISION_ACTION)MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_HORIZONTALLY;
            p247.threat_level = (MAV_COLLISION_THREAT_LEVEL)MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW;
            p247.time_to_minimum_delta = (float) -1.0786525E38F;
            p247.altitude_minimum_delta = (float) -2.7933566E37F;
            p247.horizontal_minimum_delta = (float) -1.5656027E38F;
            LoopBackDemoChannel.instance.send(p247); //===============================
            V2_EXTENSION p248 = LoopBackDemoChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.target_network = (byte)(byte)255;
            p248.target_system = (byte)(byte)241;
            p248.target_component = (byte)(byte)21;
            p248.message_type = (ushort)(ushort)23543;
            p248.payload_SET(new byte[249], 0);
            LoopBackDemoChannel.instance.send(p248); //===============================
            MEMORY_VECT p249 = LoopBackDemoChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.address = (ushort)(ushort)47411;
            p249.ver = (byte)(byte)44;
            p249.type = (byte)(byte)225;
            p249.value_SET(new sbyte[32], 0);
            LoopBackDemoChannel.instance.send(p249); //===============================
            DEBUG_VECT p250 = LoopBackDemoChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.name_SET("DEMO", PH);
            p250.time_usec = (ulong)1471359960267754392L;
            p250.x = (float)1.338361E38F;
            p250.y = (float) -2.2051604E38F;
            p250.z = (float)3.360205E38F;
            LoopBackDemoChannel.instance.send(p250); //===============================
            NAMED_VALUE_FLOAT p251 = LoopBackDemoChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.time_boot_ms = (uint)4014511423U;
            p251.name_SET("DEMO", PH);
            p251.value = (float) -2.1290212E38F;
            LoopBackDemoChannel.instance.send(p251); //===============================
            NAMED_VALUE_INT p252 = LoopBackDemoChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.time_boot_ms = (uint)2118087266U;
            p252.name_SET("DEMO", PH);
            p252.value = (int)1106526349;
            LoopBackDemoChannel.instance.send(p252); //===============================
            STATUSTEXT p253 = LoopBackDemoChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.severity = (MAV_SEVERITY)MAV_SEVERITY.MAV_SEVERITY_ERROR;
            p253.text_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p253); //===============================
            DEBUG p254 = LoopBackDemoChannel.new_DEBUG();
            PH.setPack(p254);
            p254.time_boot_ms = (uint)2858234669U;
            p254.ind = (byte)(byte)199;
            p254.value = (float) -4.0620155E37F;
            LoopBackDemoChannel.instance.send(p254); //===============================
            SETUP_SIGNING p256 = LoopBackDemoChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.target_system = (byte)(byte)103;
            p256.target_component = (byte)(byte)23;
            p256.secret_key_SET(new byte[32], 0);
            p256.initial_timestamp = (ulong)6142159417767685213L;
            LoopBackDemoChannel.instance.send(p256); //===============================
            BUTTON_CHANGE p257 = LoopBackDemoChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.time_boot_ms = (uint)3891040790U;
            p257.last_change_ms = (uint)1320793979U;
            p257.state = (byte)(byte)196;
            LoopBackDemoChannel.instance.send(p257); //===============================
            PLAY_TUNE p258 = LoopBackDemoChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_system = (byte)(byte)180;
            p258.target_component = (byte)(byte)150;
            p258.tune_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p258); //===============================
            CAMERA_INFORMATION p259 = LoopBackDemoChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.time_boot_ms = (uint)2230904350U;
            p259.vendor_name_SET(new byte[32], 0);
            p259.model_name_SET(new byte[32], 0);
            p259.firmware_version = (uint)1696469407U;
            p259.focal_length = (float)1.2360596E38F;
            p259.sensor_size_h = (float)1.1664586E37F;
            p259.sensor_size_v = (float) -3.4590326E37F;
            p259.resolution_h = (ushort)(ushort)48632;
            p259.resolution_v = (ushort)(ushort)53894;
            p259.lens_id = (byte)(byte)146;
            p259.flags = (CAMERA_CAP_FLAGS)CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE;
            p259.cam_definition_version = (ushort)(ushort)64159;
            p259.cam_definition_uri_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p259); //===============================
            CAMERA_SETTINGS p260 = LoopBackDemoChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.time_boot_ms = (uint)4211760041U;
            p260.mode_id = (CAMERA_MODE)CAMERA_MODE.CAMERA_MODE_IMAGE;
            LoopBackDemoChannel.instance.send(p260); //===============================
            STORAGE_INFORMATION p261 = LoopBackDemoChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.time_boot_ms = (uint)2970684147U;
            p261.storage_id = (byte)(byte)238;
            p261.storage_count = (byte)(byte)194;
            p261.status = (byte)(byte)7;
            p261.total_capacity = (float)1.7754619E38F;
            p261.used_capacity = (float)1.1635013E38F;
            p261.available_capacity = (float) -3.3873985E38F;
            p261.read_speed = (float)2.888182E38F;
            p261.write_speed = (float) -2.0713125E38F;
            LoopBackDemoChannel.instance.send(p261); //===============================
            CAMERA_CAPTURE_STATUS p262 = LoopBackDemoChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.time_boot_ms = (uint)1036074079U;
            p262.image_status = (byte)(byte)62;
            p262.video_status = (byte)(byte)127;
            p262.image_interval = (float) -2.2506483E38F;
            p262.recording_time_ms = (uint)400023689U;
            p262.available_capacity = (float) -2.980463E38F;
            LoopBackDemoChannel.instance.send(p262); //===============================
            CAMERA_IMAGE_CAPTURED p263 = LoopBackDemoChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.time_boot_ms = (uint)4077390656U;
            p263.time_utc = (ulong)6412196829138338050L;
            p263.camera_id = (byte)(byte)171;
            p263.lat = (int) -143539182;
            p263.lon = (int) -778919865;
            p263.alt = (int)83910702;
            p263.relative_alt = (int)1644901691;
            p263.q_SET(new float[4], 0);
            p263.image_index = (int) -2116200505;
            p263.capture_result = (sbyte)(sbyte) - 12;
            p263.file_url_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p263); //===============================
            FLIGHT_INFORMATION p264 = LoopBackDemoChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.time_boot_ms = (uint)2177175U;
            p264.arming_time_utc = (ulong)8147554444910929038L;
            p264.takeoff_time_utc = (ulong)4550474717592794919L;
            p264.flight_uuid = (ulong)1856029307477431527L;
            LoopBackDemoChannel.instance.send(p264); //===============================
            MOUNT_ORIENTATION p265 = LoopBackDemoChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.time_boot_ms = (uint)610388019U;
            p265.roll = (float) -2.2333513E38F;
            p265.pitch = (float) -1.4285984E38F;
            p265.yaw = (float) -1.8747755E38F;
            LoopBackDemoChannel.instance.send(p265); //===============================
            LOGGING_DATA p266 = LoopBackDemoChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.target_system = (byte)(byte)53;
            p266.target_component = (byte)(byte)219;
            p266.sequence = (ushort)(ushort)24682;
            p266.length = (byte)(byte)67;
            p266.first_message_offset = (byte)(byte)125;
            p266.data__SET(new byte[249], 0);
            LoopBackDemoChannel.instance.send(p266); //===============================
            LOGGING_DATA_ACKED p267 = LoopBackDemoChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.target_system = (byte)(byte)70;
            p267.target_component = (byte)(byte)78;
            p267.sequence = (ushort)(ushort)29229;
            p267.length = (byte)(byte)72;
            p267.first_message_offset = (byte)(byte)170;
            p267.data__SET(new byte[249], 0);
            LoopBackDemoChannel.instance.send(p267); //===============================
            LOGGING_ACK p268 = LoopBackDemoChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_system = (byte)(byte)224;
            p268.target_component = (byte)(byte)52;
            p268.sequence = (ushort)(ushort)58373;
            LoopBackDemoChannel.instance.send(p268); //===============================
            VIDEO_STREAM_INFORMATION p269 = LoopBackDemoChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.camera_id = (byte)(byte)83;
            p269.status = (byte)(byte)99;
            p269.framerate = (float)3.10856E38F;
            p269.resolution_h = (ushort)(ushort)13622;
            p269.resolution_v = (ushort)(ushort)39492;
            p269.bitrate = (uint)3757715147U;
            p269.rotation = (ushort)(ushort)43586;
            p269.uri_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p269); //===============================
            SET_VIDEO_STREAM_SETTINGS p270 = LoopBackDemoChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.target_system = (byte)(byte)178;
            p270.target_component = (byte)(byte)161;
            p270.camera_id = (byte)(byte)52;
            p270.framerate = (float)9.29683E37F;
            p270.resolution_h = (ushort)(ushort)18610;
            p270.resolution_v = (ushort)(ushort)22795;
            p270.bitrate = (uint)4099695223U;
            p270.rotation = (ushort)(ushort)58415;
            p270.uri_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p270); //===============================
            WIFI_CONFIG_AP p299 = LoopBackDemoChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.ssid_SET("DEMO", PH);
            p299.password_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p299); //===============================
            PROTOCOL_VERSION p300 = LoopBackDemoChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.version = (ushort)(ushort)2123;
            p300.min_version = (ushort)(ushort)59958;
            p300.max_version = (ushort)(ushort)58867;
            p300.spec_version_hash_SET(new byte[8], 0);
            p300.library_version_hash_SET(new byte[8], 0);
            LoopBackDemoChannel.instance.send(p300); //===============================
            UAVCAN_NODE_STATUS p310 = LoopBackDemoChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.time_usec = (ulong)4787505046083947034L;
            p310.uptime_sec = (uint)2270499062U;
            p310.health = (UAVCAN_NODE_HEALTH)UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING;
            p310.mode = (UAVCAN_NODE_MODE)UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE;
            p310.sub_mode = (byte)(byte)156;
            p310.vendor_specific_status_code = (ushort)(ushort)8090;
            LoopBackDemoChannel.instance.send(p310); //===============================
            UAVCAN_NODE_INFO p311 = LoopBackDemoChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.time_usec = (ulong)4030498603779447531L;
            p311.uptime_sec = (uint)2366320656U;
            p311.name_SET("DEMO", PH);
            p311.hw_version_major = (byte)(byte)3;
            p311.hw_version_minor = (byte)(byte)183;
            p311.hw_unique_id_SET(new byte[16], 0);
            p311.sw_version_major = (byte)(byte)250;
            p311.sw_version_minor = (byte)(byte)249;
            p311.sw_vcs_commit = (uint)1600955441U;
            LoopBackDemoChannel.instance.send(p311); //===============================
            PARAM_EXT_REQUEST_READ p320 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.target_system = (byte)(byte)218;
            p320.target_component = (byte)(byte)185;
            p320.param_id_SET("DEMO", PH);
            p320.param_index = (short)(short) -28027;
            LoopBackDemoChannel.instance.send(p320); //===============================
            PARAM_EXT_REQUEST_LIST p321 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_system = (byte)(byte)117;
            p321.target_component = (byte)(byte)113;
            LoopBackDemoChannel.instance.send(p321); //===============================
            PARAM_EXT_VALUE p322 = LoopBackDemoChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_id_SET("DEMO", PH);
            p322.param_value_SET("DEMO", PH);
            p322.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16;
            p322.param_count = (ushort)(ushort)3196;
            p322.param_index = (ushort)(ushort)28981;
            LoopBackDemoChannel.instance.send(p322); //===============================
            PARAM_EXT_SET p323 = LoopBackDemoChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.target_system = (byte)(byte)62;
            p323.target_component = (byte)(byte)40;
            p323.param_id_SET("DEMO", PH);
            p323.param_value_SET("DEMO", PH);
            p323.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT64;
            LoopBackDemoChannel.instance.send(p323); //===============================
            PARAM_EXT_ACK p324 = LoopBackDemoChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_id_SET("DEMO", PH);
            p324.param_value_SET("DEMO", PH);
            p324.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM;
            p324.param_result = (PARAM_ACK)PARAM_ACK.PARAM_ACK_ACCEPTED;
            LoopBackDemoChannel.instance.send(p324); //===============================
            OBSTACLE_DISTANCE p330 = LoopBackDemoChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.time_usec = (ulong)3053846303319291102L;
            p330.sensor_type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED;
            p330.distances_SET(new ushort[72], 0);
            p330.increment = (byte)(byte)7;
            p330.min_distance = (ushort)(ushort)34713;
            p330.max_distance = (ushort)(ushort)52778;
            LoopBackDemoChannel.instance.send(p330); //===============================
            UAVIONIX_ADSB_OUT_CFG p10001 = LoopBackDemoChannel.new_UAVIONIX_ADSB_OUT_CFG();
            PH.setPack(p10001);
            p10001.ICAO = (uint)2014158949U;
            p10001.callsign_SET("DEMO", PH);
            p10001.emitterType = (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_SPACE;
            p10001.aircraftSize = (UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE)UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE.UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L25M_W28P5M;
            p10001.gpsOffsetLat = (UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT)UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_LEFT_6M;
            p10001.gpsOffsetLon = (UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON)UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_APPLIED_BY_SENSOR;
            p10001.stallSpeed = (ushort)(ushort)57076;
            p10001.rfSelect = (UAVIONIX_ADSB_OUT_RF_SELECT)UAVIONIX_ADSB_OUT_RF_SELECT.UAVIONIX_ADSB_OUT_RF_SELECT_TX_ENABLED;
            LoopBackDemoChannel.instance.send(p10001); //===============================
            UAVIONIX_ADSB_OUT_DYNAMIC p10002 = LoopBackDemoChannel.new_UAVIONIX_ADSB_OUT_DYNAMIC();
            PH.setPack(p10002);
            p10002.utcTime = (uint)1725295800U;
            p10002.gpsLat = (int) -323087636;
            p10002.gpsLon = (int) -2105685173;
            p10002.gpsAlt = (int)238899064;
            p10002.gpsFix = (UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX)UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX.UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_DGPS;
            p10002.numSats = (byte)(byte)60;
            p10002.baroAltMSL = (int)1199601674;
            p10002.accuracyHor = (uint)1838033906U;
            p10002.accuracyVert = (ushort)(ushort)40351;
            p10002.accuracyVel = (ushort)(ushort)4180;
            p10002.velVert = (short)(short) -14944;
            p10002.velNS = (short)(short)11988;
            p10002.VelEW = (short)(short) -7004;
            p10002.emergencyStatus = (UAVIONIX_ADSB_EMERGENCY_STATUS)UAVIONIX_ADSB_EMERGENCY_STATUS.UAVIONIX_ADSB_OUT_NO_EMERGENCY;
            p10002.state = (UAVIONIX_ADSB_OUT_DYNAMIC_STATE)UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_INTENT_CHANGE;
            p10002.squawk = (ushort)(ushort)1528;
            LoopBackDemoChannel.instance.send(p10002); //===============================
            UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT p10003 = LoopBackDemoChannel.new_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT();
            PH.setPack(p10003);
            p10003.rfHealth = (UAVIONIX_ADSB_RF_HEALTH)UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_INITIALIZING;
            LoopBackDemoChannel.instance.send(p10003); //===============================
            DEVICE_OP_READ p11000 = LoopBackDemoChannel.new_DEVICE_OP_READ();
            PH.setPack(p11000);
            p11000.target_system = (byte)(byte)235;
            p11000.target_component = (byte)(byte)127;
            p11000.request_id = (uint)1471388501U;
            p11000.bustype = (DEVICE_OP_BUSTYPE)DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_SPI;
            p11000.bus = (byte)(byte)248;
            p11000.address = (byte)(byte)180;
            p11000.busname_SET("DEMO", PH);
            p11000.regstart = (byte)(byte)82;
            p11000.count = (byte)(byte)254;
            LoopBackDemoChannel.instance.send(p11000); //===============================
            DEVICE_OP_READ_REPLY p11001 = LoopBackDemoChannel.new_DEVICE_OP_READ_REPLY();
            PH.setPack(p11001);
            p11001.request_id = (uint)761538568U;
            p11001.result = (byte)(byte)217;
            p11001.regstart = (byte)(byte)192;
            p11001.count = (byte)(byte)244;
            p11001.data__SET(new byte[128], 0);
            LoopBackDemoChannel.instance.send(p11001); //===============================
            DEVICE_OP_WRITE p11002 = LoopBackDemoChannel.new_DEVICE_OP_WRITE();
            PH.setPack(p11002);
            p11002.target_system = (byte)(byte)48;
            p11002.target_component = (byte)(byte)188;
            p11002.request_id = (uint)3575514019U;
            p11002.bustype = (DEVICE_OP_BUSTYPE)DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_SPI;
            p11002.bus = (byte)(byte)204;
            p11002.address = (byte)(byte)97;
            p11002.busname_SET("DEMO", PH);
            p11002.regstart = (byte)(byte)200;
            p11002.count = (byte)(byte)122;
            p11002.data__SET(new byte[128], 0);
            LoopBackDemoChannel.instance.send(p11002); //===============================
            DEVICE_OP_WRITE_REPLY p11003 = LoopBackDemoChannel.new_DEVICE_OP_WRITE_REPLY();
            PH.setPack(p11003);
            p11003.request_id = (uint)1652320241U;
            p11003.result = (byte)(byte)166;
            LoopBackDemoChannel.instance.send(p11003); //===============================
            ADAP_TUNING p11010 = LoopBackDemoChannel.new_ADAP_TUNING();
            PH.setPack(p11010);
            p11010.axis = (PID_TUNING_AXIS)PID_TUNING_AXIS.PID_TUNING_ACCZ;
            p11010.desired = (float) -3.7015504E37F;
            p11010.achieved = (float) -1.9610962E38F;
            p11010.error = (float) -1.1483992E37F;
            p11010.theta = (float)1.859086E38F;
            p11010.omega = (float)2.1344997E37F;
            p11010.sigma = (float)2.0617424E38F;
            p11010.theta_dot = (float) -9.039935E37F;
            p11010.omega_dot = (float)2.8986204E38F;
            p11010.sigma_dot = (float) -1.8167998E37F;
            p11010.f = (float)3.1701755E36F;
            p11010.f_dot = (float) -1.3556083E38F;
            p11010.u = (float) -2.8798623E37F;
            LoopBackDemoChannel.instance.send(p11010); //===============================
            VISION_POSITION_DELTA p11011 = LoopBackDemoChannel.new_VISION_POSITION_DELTA();
            PH.setPack(p11011);
            p11011.time_usec = (ulong)7720297360801151207L;
            p11011.time_delta_usec = (ulong)8532049757944740095L;
            p11011.angle_delta_SET(new float[3], 0);
            p11011.position_delta_SET(new float[3], 0);
            p11011.confidence = (float)8.606099E37F;
            LoopBackDemoChannel.instance.send(p11011); //===============================
        }
    }
}

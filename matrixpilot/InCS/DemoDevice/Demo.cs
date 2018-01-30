
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
            LoopBackDemoChannel.instance.OnFLEXIFUNCTION_SETReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
            };
            LoopBackDemoChannel.instance.OnFLEXIFUNCTION_READ_REQReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                short read_req_type = pack.read_req_type;
                short data_index = pack.data_index;
            };
            LoopBackDemoChannel.instance.OnFLEXIFUNCTION_BUFFER_FUNCTIONReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                ushort func_index = pack.func_index;
                ushort func_count = pack.func_count;
                ushort data_address = pack.data_address;
                ushort data_size = pack.data_size;
                sbyte[] data_ = pack.data_;
            };
            LoopBackDemoChannel.instance.OnFLEXIFUNCTION_BUFFER_FUNCTION_ACKReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                ushort func_index = pack.func_index;
                ushort result = pack.result;
            };
            LoopBackDemoChannel.instance.OnFLEXIFUNCTION_DIRECTORYReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                byte directory_type = pack.directory_type;
                byte start_index = pack.start_index;
                byte count = pack.count;
                sbyte[] directory_data = pack.directory_data;
            };
            LoopBackDemoChannel.instance.OnFLEXIFUNCTION_DIRECTORY_ACKReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                byte directory_type = pack.directory_type;
                byte start_index = pack.start_index;
                byte count = pack.count;
                ushort result = pack.result;
            };
            LoopBackDemoChannel.instance.OnFLEXIFUNCTION_COMMANDReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                byte command_type = pack.command_type;
            };
            LoopBackDemoChannel.instance.OnFLEXIFUNCTION_COMMAND_ACKReceive += (src, ph, pack) =>
            {
                ushort command_type = pack.command_type;
                ushort result = pack.result;
            };
            LoopBackDemoChannel.instance.OnSERIAL_UDB_EXTRA_F2_AReceive += (src, ph, pack) =>
            {
                uint sue_time = pack.sue_time;
                byte sue_status = pack.sue_status;
                int sue_latitude = pack.sue_latitude;
                int sue_longitude = pack.sue_longitude;
                int sue_altitude = pack.sue_altitude;
                ushort sue_waypoint_index = pack.sue_waypoint_index;
                short sue_rmat0 = pack.sue_rmat0;
                short sue_rmat1 = pack.sue_rmat1;
                short sue_rmat2 = pack.sue_rmat2;
                short sue_rmat3 = pack.sue_rmat3;
                short sue_rmat4 = pack.sue_rmat4;
                short sue_rmat5 = pack.sue_rmat5;
                short sue_rmat6 = pack.sue_rmat6;
                short sue_rmat7 = pack.sue_rmat7;
                short sue_rmat8 = pack.sue_rmat8;
                ushort sue_cog = pack.sue_cog;
                short sue_sog = pack.sue_sog;
                ushort sue_cpu_load = pack.sue_cpu_load;
                ushort sue_air_speed_3DIMU = pack.sue_air_speed_3DIMU;
                short sue_estimated_wind_0 = pack.sue_estimated_wind_0;
                short sue_estimated_wind_1 = pack.sue_estimated_wind_1;
                short sue_estimated_wind_2 = pack.sue_estimated_wind_2;
                short sue_magFieldEarth0 = pack.sue_magFieldEarth0;
                short sue_magFieldEarth1 = pack.sue_magFieldEarth1;
                short sue_magFieldEarth2 = pack.sue_magFieldEarth2;
                short sue_svs = pack.sue_svs;
                short sue_hdop = pack.sue_hdop;
            };
            LoopBackDemoChannel.instance.OnSERIAL_UDB_EXTRA_F2_BReceive += (src, ph, pack) =>
            {
                uint sue_time = pack.sue_time;
                short sue_pwm_input_1 = pack.sue_pwm_input_1;
                short sue_pwm_input_2 = pack.sue_pwm_input_2;
                short sue_pwm_input_3 = pack.sue_pwm_input_3;
                short sue_pwm_input_4 = pack.sue_pwm_input_4;
                short sue_pwm_input_5 = pack.sue_pwm_input_5;
                short sue_pwm_input_6 = pack.sue_pwm_input_6;
                short sue_pwm_input_7 = pack.sue_pwm_input_7;
                short sue_pwm_input_8 = pack.sue_pwm_input_8;
                short sue_pwm_input_9 = pack.sue_pwm_input_9;
                short sue_pwm_input_10 = pack.sue_pwm_input_10;
                short sue_pwm_input_11 = pack.sue_pwm_input_11;
                short sue_pwm_input_12 = pack.sue_pwm_input_12;
                short sue_pwm_output_1 = pack.sue_pwm_output_1;
                short sue_pwm_output_2 = pack.sue_pwm_output_2;
                short sue_pwm_output_3 = pack.sue_pwm_output_3;
                short sue_pwm_output_4 = pack.sue_pwm_output_4;
                short sue_pwm_output_5 = pack.sue_pwm_output_5;
                short sue_pwm_output_6 = pack.sue_pwm_output_6;
                short sue_pwm_output_7 = pack.sue_pwm_output_7;
                short sue_pwm_output_8 = pack.sue_pwm_output_8;
                short sue_pwm_output_9 = pack.sue_pwm_output_9;
                short sue_pwm_output_10 = pack.sue_pwm_output_10;
                short sue_pwm_output_11 = pack.sue_pwm_output_11;
                short sue_pwm_output_12 = pack.sue_pwm_output_12;
                short sue_imu_location_x = pack.sue_imu_location_x;
                short sue_imu_location_y = pack.sue_imu_location_y;
                short sue_imu_location_z = pack.sue_imu_location_z;
                short sue_location_error_earth_x = pack.sue_location_error_earth_x;
                short sue_location_error_earth_y = pack.sue_location_error_earth_y;
                short sue_location_error_earth_z = pack.sue_location_error_earth_z;
                uint sue_flags = pack.sue_flags;
                short sue_osc_fails = pack.sue_osc_fails;
                short sue_imu_velocity_x = pack.sue_imu_velocity_x;
                short sue_imu_velocity_y = pack.sue_imu_velocity_y;
                short sue_imu_velocity_z = pack.sue_imu_velocity_z;
                short sue_waypoint_goal_x = pack.sue_waypoint_goal_x;
                short sue_waypoint_goal_y = pack.sue_waypoint_goal_y;
                short sue_waypoint_goal_z = pack.sue_waypoint_goal_z;
                short sue_aero_x = pack.sue_aero_x;
                short sue_aero_y = pack.sue_aero_y;
                short sue_aero_z = pack.sue_aero_z;
                short sue_barom_temp = pack.sue_barom_temp;
                int sue_barom_press = pack.sue_barom_press;
                int sue_barom_alt = pack.sue_barom_alt;
                short sue_bat_volt = pack.sue_bat_volt;
                short sue_bat_amp = pack.sue_bat_amp;
                short sue_bat_amp_hours = pack.sue_bat_amp_hours;
                short sue_desired_height = pack.sue_desired_height;
                short sue_memory_stack_free = pack.sue_memory_stack_free;
            };
            LoopBackDemoChannel.instance.OnSERIAL_UDB_EXTRA_F4Receive += (src, ph, pack) =>
            {
                byte sue_ROLL_STABILIZATION_AILERONS = pack.sue_ROLL_STABILIZATION_AILERONS;
                byte sue_ROLL_STABILIZATION_RUDDER = pack.sue_ROLL_STABILIZATION_RUDDER;
                byte sue_PITCH_STABILIZATION = pack.sue_PITCH_STABILIZATION;
                byte sue_YAW_STABILIZATION_RUDDER = pack.sue_YAW_STABILIZATION_RUDDER;
                byte sue_YAW_STABILIZATION_AILERON = pack.sue_YAW_STABILIZATION_AILERON;
                byte sue_AILERON_NAVIGATION = pack.sue_AILERON_NAVIGATION;
                byte sue_RUDDER_NAVIGATION = pack.sue_RUDDER_NAVIGATION;
                byte sue_ALTITUDEHOLD_STABILIZED = pack.sue_ALTITUDEHOLD_STABILIZED;
                byte sue_ALTITUDEHOLD_WAYPOINT = pack.sue_ALTITUDEHOLD_WAYPOINT;
                byte sue_RACING_MODE = pack.sue_RACING_MODE;
            };
            LoopBackDemoChannel.instance.OnSERIAL_UDB_EXTRA_F5Receive += (src, ph, pack) =>
            {
                float sue_YAWKP_AILERON = pack.sue_YAWKP_AILERON;
                float sue_YAWKD_AILERON = pack.sue_YAWKD_AILERON;
                float sue_ROLLKP = pack.sue_ROLLKP;
                float sue_ROLLKD = pack.sue_ROLLKD;
            };
            LoopBackDemoChannel.instance.OnSERIAL_UDB_EXTRA_F6Receive += (src, ph, pack) =>
            {
                float sue_PITCHGAIN = pack.sue_PITCHGAIN;
                float sue_PITCHKD = pack.sue_PITCHKD;
                float sue_RUDDER_ELEV_MIX = pack.sue_RUDDER_ELEV_MIX;
                float sue_ROLL_ELEV_MIX = pack.sue_ROLL_ELEV_MIX;
                float sue_ELEVATOR_BOOST = pack.sue_ELEVATOR_BOOST;
            };
            LoopBackDemoChannel.instance.OnSERIAL_UDB_EXTRA_F7Receive += (src, ph, pack) =>
            {
                float sue_YAWKP_RUDDER = pack.sue_YAWKP_RUDDER;
                float sue_YAWKD_RUDDER = pack.sue_YAWKD_RUDDER;
                float sue_ROLLKP_RUDDER = pack.sue_ROLLKP_RUDDER;
                float sue_ROLLKD_RUDDER = pack.sue_ROLLKD_RUDDER;
                float sue_RUDDER_BOOST = pack.sue_RUDDER_BOOST;
                float sue_RTL_PITCH_DOWN = pack.sue_RTL_PITCH_DOWN;
            };
            LoopBackDemoChannel.instance.OnSERIAL_UDB_EXTRA_F8Receive += (src, ph, pack) =>
            {
                float sue_HEIGHT_TARGET_MAX = pack.sue_HEIGHT_TARGET_MAX;
                float sue_HEIGHT_TARGET_MIN = pack.sue_HEIGHT_TARGET_MIN;
                float sue_ALT_HOLD_THROTTLE_MIN = pack.sue_ALT_HOLD_THROTTLE_MIN;
                float sue_ALT_HOLD_THROTTLE_MAX = pack.sue_ALT_HOLD_THROTTLE_MAX;
                float sue_ALT_HOLD_PITCH_MIN = pack.sue_ALT_HOLD_PITCH_MIN;
                float sue_ALT_HOLD_PITCH_MAX = pack.sue_ALT_HOLD_PITCH_MAX;
                float sue_ALT_HOLD_PITCH_HIGH = pack.sue_ALT_HOLD_PITCH_HIGH;
            };
            LoopBackDemoChannel.instance.OnSERIAL_UDB_EXTRA_F13Receive += (src, ph, pack) =>
            {
                short sue_week_no = pack.sue_week_no;
                int sue_lat_origin = pack.sue_lat_origin;
                int sue_lon_origin = pack.sue_lon_origin;
                int sue_alt_origin = pack.sue_alt_origin;
            };
            LoopBackDemoChannel.instance.OnSERIAL_UDB_EXTRA_F14Receive += (src, ph, pack) =>
            {
                byte sue_WIND_ESTIMATION = pack.sue_WIND_ESTIMATION;
                byte sue_GPS_TYPE = pack.sue_GPS_TYPE;
                byte sue_DR = pack.sue_DR;
                byte sue_BOARD_TYPE = pack.sue_BOARD_TYPE;
                byte sue_AIRFRAME = pack.sue_AIRFRAME;
                short sue_RCON = pack.sue_RCON;
                short sue_TRAP_FLAGS = pack.sue_TRAP_FLAGS;
                uint sue_TRAP_SOURCE = pack.sue_TRAP_SOURCE;
                short sue_osc_fail_count = pack.sue_osc_fail_count;
                byte sue_CLOCK_CONFIG = pack.sue_CLOCK_CONFIG;
                byte sue_FLIGHT_PLAN_TYPE = pack.sue_FLIGHT_PLAN_TYPE;
            };
            LoopBackDemoChannel.instance.OnSERIAL_UDB_EXTRA_F15Receive += (src, ph, pack) =>
            {
                byte[] sue_ID_VEHICLE_MODEL_NAME = pack.sue_ID_VEHICLE_MODEL_NAME;
                byte[] sue_ID_VEHICLE_REGISTRATION = pack.sue_ID_VEHICLE_REGISTRATION;
            };
            LoopBackDemoChannel.instance.OnSERIAL_UDB_EXTRA_F16Receive += (src, ph, pack) =>
            {
                byte[] sue_ID_LEAD_PILOT = pack.sue_ID_LEAD_PILOT;
                byte[] sue_ID_DIY_DRONES_URL = pack.sue_ID_DIY_DRONES_URL;
            };
            LoopBackDemoChannel.instance.OnALTITUDESReceive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                int alt_gps = pack.alt_gps;
                int alt_imu = pack.alt_imu;
                int alt_barometric = pack.alt_barometric;
                int alt_optical_flow = pack.alt_optical_flow;
                int alt_range_finder = pack.alt_range_finder;
                int alt_extra = pack.alt_extra;
            };
            LoopBackDemoChannel.instance.OnAIRSPEEDSReceive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                short airspeed_imu = pack.airspeed_imu;
                short airspeed_pitot = pack.airspeed_pitot;
                short airspeed_hot_wire = pack.airspeed_hot_wire;
                short airspeed_ultrasonic = pack.airspeed_ultrasonic;
                short aoa = pack.aoa;
                short aoy = pack.aoy;
            };
            LoopBackDemoChannel.instance.OnSERIAL_UDB_EXTRA_F17Receive += (src, ph, pack) =>
            {
                float sue_feed_forward = pack.sue_feed_forward;
                float sue_turn_rate_nav = pack.sue_turn_rate_nav;
                float sue_turn_rate_fbw = pack.sue_turn_rate_fbw;
            };
            LoopBackDemoChannel.instance.OnSERIAL_UDB_EXTRA_F18Receive += (src, ph, pack) =>
            {
                float angle_of_attack_normal = pack.angle_of_attack_normal;
                float angle_of_attack_inverted = pack.angle_of_attack_inverted;
                float elevator_trim_normal = pack.elevator_trim_normal;
                float elevator_trim_inverted = pack.elevator_trim_inverted;
                float reference_speed = pack.reference_speed;
            };
            LoopBackDemoChannel.instance.OnSERIAL_UDB_EXTRA_F19Receive += (src, ph, pack) =>
            {
                byte sue_aileron_output_channel = pack.sue_aileron_output_channel;
                byte sue_aileron_reversed = pack.sue_aileron_reversed;
                byte sue_elevator_output_channel = pack.sue_elevator_output_channel;
                byte sue_elevator_reversed = pack.sue_elevator_reversed;
                byte sue_throttle_output_channel = pack.sue_throttle_output_channel;
                byte sue_throttle_reversed = pack.sue_throttle_reversed;
                byte sue_rudder_output_channel = pack.sue_rudder_output_channel;
                byte sue_rudder_reversed = pack.sue_rudder_reversed;
            };
            LoopBackDemoChannel.instance.OnSERIAL_UDB_EXTRA_F20Receive += (src, ph, pack) =>
            {
                byte sue_number_of_inputs = pack.sue_number_of_inputs;
                short sue_trim_value_input_1 = pack.sue_trim_value_input_1;
                short sue_trim_value_input_2 = pack.sue_trim_value_input_2;
                short sue_trim_value_input_3 = pack.sue_trim_value_input_3;
                short sue_trim_value_input_4 = pack.sue_trim_value_input_4;
                short sue_trim_value_input_5 = pack.sue_trim_value_input_5;
                short sue_trim_value_input_6 = pack.sue_trim_value_input_6;
                short sue_trim_value_input_7 = pack.sue_trim_value_input_7;
                short sue_trim_value_input_8 = pack.sue_trim_value_input_8;
                short sue_trim_value_input_9 = pack.sue_trim_value_input_9;
                short sue_trim_value_input_10 = pack.sue_trim_value_input_10;
                short sue_trim_value_input_11 = pack.sue_trim_value_input_11;
                short sue_trim_value_input_12 = pack.sue_trim_value_input_12;
            };
            LoopBackDemoChannel.instance.OnSERIAL_UDB_EXTRA_F21Receive += (src, ph, pack) =>
            {
                short sue_accel_x_offset = pack.sue_accel_x_offset;
                short sue_accel_y_offset = pack.sue_accel_y_offset;
                short sue_accel_z_offset = pack.sue_accel_z_offset;
                short sue_gyro_x_offset = pack.sue_gyro_x_offset;
                short sue_gyro_y_offset = pack.sue_gyro_y_offset;
                short sue_gyro_z_offset = pack.sue_gyro_z_offset;
            };
            LoopBackDemoChannel.instance.OnSERIAL_UDB_EXTRA_F22Receive += (src, ph, pack) =>
            {
                short sue_accel_x_at_calibration = pack.sue_accel_x_at_calibration;
                short sue_accel_y_at_calibration = pack.sue_accel_y_at_calibration;
                short sue_accel_z_at_calibration = pack.sue_accel_z_at_calibration;
                short sue_gyro_x_at_calibration = pack.sue_gyro_x_at_calibration;
                short sue_gyro_y_at_calibration = pack.sue_gyro_y_at_calibration;
                short sue_gyro_z_at_calibration = pack.sue_gyro_z_at_calibration;
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
            p0.type = (MAV_TYPE)MAV_TYPE.MAV_TYPE_COAXIAL;
            p0.autopilot = (MAV_AUTOPILOT)MAV_AUTOPILOT.MAV_AUTOPILOT_RESERVED;
            p0.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED;
            p0.custom_mode = (uint)2724145656U;
            p0.system_status = (MAV_STATE)MAV_STATE.MAV_STATE_ACTIVE;
            p0.mavlink_version = (byte)(byte)201;
            LoopBackDemoChannel.instance.send(p0); //===============================
            SYS_STATUS p1 = LoopBackDemoChannel.new_SYS_STATUS();
            PH.setPack(p1);
            p1.onboard_control_sensors_present = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING;
            p1.onboard_control_sensors_enabled = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR;
            p1.onboard_control_sensors_health = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN;
            p1.load = (ushort)(ushort)44680;
            p1.voltage_battery = (ushort)(ushort)12774;
            p1.current_battery = (short)(short)12030;
            p1.battery_remaining = (sbyte)(sbyte)53;
            p1.drop_rate_comm = (ushort)(ushort)36213;
            p1.errors_comm = (ushort)(ushort)33039;
            p1.errors_count1 = (ushort)(ushort)45813;
            p1.errors_count2 = (ushort)(ushort)40130;
            p1.errors_count3 = (ushort)(ushort)35000;
            p1.errors_count4 = (ushort)(ushort)14445;
            LoopBackDemoChannel.instance.send(p1); //===============================
            SYSTEM_TIME p2 = LoopBackDemoChannel.new_SYSTEM_TIME();
            PH.setPack(p2);
            p2.time_unix_usec = (ulong)991727609054407541L;
            p2.time_boot_ms = (uint)3091030045U;
            LoopBackDemoChannel.instance.send(p2); //===============================
            POSITION_TARGET_LOCAL_NED p3 = LoopBackDemoChannel.new_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.time_boot_ms = (uint)4010274065U;
            p3.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_MISSION;
            p3.type_mask = (ushort)(ushort)42556;
            p3.x = (float) -5.3473513E37F;
            p3.y = (float)1.9903577E38F;
            p3.z = (float)2.9213043E38F;
            p3.vx = (float)2.1941312E38F;
            p3.vy = (float) -3.261862E36F;
            p3.vz = (float) -3.279113E38F;
            p3.afx = (float) -3.3561956E38F;
            p3.afy = (float) -2.4848657E38F;
            p3.afz = (float)6.279901E37F;
            p3.yaw = (float) -1.0525637E38F;
            p3.yaw_rate = (float) -1.7960109E37F;
            LoopBackDemoChannel.instance.send(p3); //===============================
            PING p4 = LoopBackDemoChannel.new_PING();
            PH.setPack(p4);
            p4.time_usec = (ulong)1013028185467648412L;
            p4.seq = (uint)4142546296U;
            p4.target_system = (byte)(byte)213;
            p4.target_component = (byte)(byte)161;
            LoopBackDemoChannel.instance.send(p4); //===============================
            CHANGE_OPERATOR_CONTROL p5 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL();
            PH.setPack(p5);
            p5.target_system = (byte)(byte)253;
            p5.control_request = (byte)(byte)218;
            p5.version = (byte)(byte)198;
            p5.passkey_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p5); //===============================
            CHANGE_OPERATOR_CONTROL_ACK p6 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL_ACK();
            PH.setPack(p6);
            p6.gcs_system_id = (byte)(byte)254;
            p6.control_request = (byte)(byte)133;
            p6.ack = (byte)(byte)70;
            LoopBackDemoChannel.instance.send(p6); //===============================
            AUTH_KEY p7 = LoopBackDemoChannel.new_AUTH_KEY();
            PH.setPack(p7);
            p7.key_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p7); //===============================
            SET_MODE p11 = LoopBackDemoChannel.new_SET_MODE();
            PH.setPack(p11);
            p11.target_system = (byte)(byte)183;
            p11.base_mode = (MAV_MODE)MAV_MODE.MAV_MODE_AUTO_ARMED;
            p11.custom_mode = (uint)186496519U;
            LoopBackDemoChannel.instance.send(p11); //===============================
            PARAM_REQUEST_READ p20 = LoopBackDemoChannel.new_PARAM_REQUEST_READ();
            PH.setPack(p20);
            p20.target_system = (byte)(byte)13;
            p20.target_component = (byte)(byte)121;
            p20.param_id_SET("DEMO", PH);
            p20.param_index = (short)(short)6721;
            LoopBackDemoChannel.instance.send(p20); //===============================
            PARAM_REQUEST_LIST p21 = LoopBackDemoChannel.new_PARAM_REQUEST_LIST();
            PH.setPack(p21);
            p21.target_system = (byte)(byte)146;
            p21.target_component = (byte)(byte)9;
            LoopBackDemoChannel.instance.send(p21); //===============================
            PARAM_VALUE p22 = LoopBackDemoChannel.new_PARAM_VALUE();
            PH.setPack(p22);
            p22.param_id_SET("DEMO", PH);
            p22.param_value = (float)2.4074214E38F;
            p22.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16;
            p22.param_count = (ushort)(ushort)46784;
            p22.param_index = (ushort)(ushort)40501;
            LoopBackDemoChannel.instance.send(p22); //===============================
            PARAM_SET p23 = LoopBackDemoChannel.new_PARAM_SET();
            PH.setPack(p23);
            p23.target_system = (byte)(byte)163;
            p23.target_component = (byte)(byte)246;
            p23.param_id_SET("DEMO", PH);
            p23.param_value = (float) -1.1637358E38F;
            p23.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT64;
            LoopBackDemoChannel.instance.send(p23); //===============================
            GPS_RAW_INT p24 = LoopBackDemoChannel.new_GPS_RAW_INT();
            PH.setPack(p24);
            p24.time_usec = (ulong)7757421560632195982L;
            p24.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC;
            p24.lat = (int) -1763420158;
            p24.lon = (int)1155799711;
            p24.alt = (int)31213735;
            p24.eph = (ushort)(ushort)37993;
            p24.epv = (ushort)(ushort)24172;
            p24.vel = (ushort)(ushort)38133;
            p24.cog = (ushort)(ushort)19264;
            p24.satellites_visible = (byte)(byte)197;
            p24.alt_ellipsoid_SET((int) -1845183689, PH);
            p24.h_acc_SET((uint)483962124U, PH);
            p24.v_acc_SET((uint)903031480U, PH);
            p24.vel_acc_SET((uint)1133531739U, PH);
            p24.hdg_acc_SET((uint)3680967785U, PH);
            LoopBackDemoChannel.instance.send(p24); //===============================
            GPS_STATUS p25 = LoopBackDemoChannel.new_GPS_STATUS();
            PH.setPack(p25);
            p25.satellites_visible = (byte)(byte)174;
            p25.satellite_prn_SET(new byte[20], 0);
            p25.satellite_used_SET(new byte[20], 0);
            p25.satellite_elevation_SET(new byte[20], 0);
            p25.satellite_azimuth_SET(new byte[20], 0);
            p25.satellite_snr_SET(new byte[20], 0);
            LoopBackDemoChannel.instance.send(p25); //===============================
            SCALED_IMU p26 = LoopBackDemoChannel.new_SCALED_IMU();
            PH.setPack(p26);
            p26.time_boot_ms = (uint)4163331401U;
            p26.xacc = (short)(short)10504;
            p26.yacc = (short)(short) -28766;
            p26.zacc = (short)(short)15407;
            p26.xgyro = (short)(short)11148;
            p26.ygyro = (short)(short) -10579;
            p26.zgyro = (short)(short) -6764;
            p26.xmag = (short)(short) -12983;
            p26.ymag = (short)(short)20997;
            p26.zmag = (short)(short)31222;
            LoopBackDemoChannel.instance.send(p26); //===============================
            RAW_IMU p27 = LoopBackDemoChannel.new_RAW_IMU();
            PH.setPack(p27);
            p27.time_usec = (ulong)9151654620178561549L;
            p27.xacc = (short)(short) -31248;
            p27.yacc = (short)(short)13432;
            p27.zacc = (short)(short) -11097;
            p27.xgyro = (short)(short)5;
            p27.ygyro = (short)(short) -5702;
            p27.zgyro = (short)(short)20863;
            p27.xmag = (short)(short)20964;
            p27.ymag = (short)(short)4027;
            p27.zmag = (short)(short)12140;
            LoopBackDemoChannel.instance.send(p27); //===============================
            RAW_PRESSURE p28 = LoopBackDemoChannel.new_RAW_PRESSURE();
            PH.setPack(p28);
            p28.time_usec = (ulong)1643787761930991101L;
            p28.press_abs = (short)(short) -21593;
            p28.press_diff1 = (short)(short)3690;
            p28.press_diff2 = (short)(short)11412;
            p28.temperature = (short)(short)9370;
            LoopBackDemoChannel.instance.send(p28); //===============================
            SCALED_PRESSURE p29 = LoopBackDemoChannel.new_SCALED_PRESSURE();
            PH.setPack(p29);
            p29.time_boot_ms = (uint)1107722075U;
            p29.press_abs = (float) -8.4777227E37F;
            p29.press_diff = (float) -3.3838787E38F;
            p29.temperature = (short)(short)16917;
            LoopBackDemoChannel.instance.send(p29); //===============================
            ATTITUDE p30 = LoopBackDemoChannel.new_ATTITUDE();
            PH.setPack(p30);
            p30.time_boot_ms = (uint)2884816654U;
            p30.roll = (float)1.6981964E38F;
            p30.pitch = (float) -1.9196317E38F;
            p30.yaw = (float) -1.7833144E38F;
            p30.rollspeed = (float)2.2404416E38F;
            p30.pitchspeed = (float)2.3975502E38F;
            p30.yawspeed = (float)6.5369603E37F;
            LoopBackDemoChannel.instance.send(p30); //===============================
            ATTITUDE_QUATERNION p31 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION();
            PH.setPack(p31);
            p31.time_boot_ms = (uint)468815709U;
            p31.q1 = (float) -3.0390307E38F;
            p31.q2 = (float) -2.0269924E38F;
            p31.q3 = (float) -4.0235314E37F;
            p31.q4 = (float) -2.7146389E38F;
            p31.rollspeed = (float)3.1370121E38F;
            p31.pitchspeed = (float)4.8406577E37F;
            p31.yawspeed = (float)2.1518793E38F;
            LoopBackDemoChannel.instance.send(p31); //===============================
            LOCAL_POSITION_NED p32 = LoopBackDemoChannel.new_LOCAL_POSITION_NED();
            PH.setPack(p32);
            p32.time_boot_ms = (uint)315242119U;
            p32.x = (float)4.76104E37F;
            p32.y = (float) -3.7443663E37F;
            p32.z = (float) -2.8421056E37F;
            p32.vx = (float)1.1955783E38F;
            p32.vy = (float)3.0298255E38F;
            p32.vz = (float) -4.838609E36F;
            LoopBackDemoChannel.instance.send(p32); //===============================
            GLOBAL_POSITION_INT p33 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT();
            PH.setPack(p33);
            p33.time_boot_ms = (uint)1671141502U;
            p33.lat = (int)651640982;
            p33.lon = (int) -12044177;
            p33.alt = (int) -11665929;
            p33.relative_alt = (int) -1566555490;
            p33.vx = (short)(short)31693;
            p33.vy = (short)(short) -26964;
            p33.vz = (short)(short)22732;
            p33.hdg = (ushort)(ushort)38694;
            LoopBackDemoChannel.instance.send(p33); //===============================
            RC_CHANNELS_SCALED p34 = LoopBackDemoChannel.new_RC_CHANNELS_SCALED();
            PH.setPack(p34);
            p34.time_boot_ms = (uint)772828177U;
            p34.port = (byte)(byte)164;
            p34.chan1_scaled = (short)(short) -18360;
            p34.chan2_scaled = (short)(short) -8788;
            p34.chan3_scaled = (short)(short)23794;
            p34.chan4_scaled = (short)(short)11915;
            p34.chan5_scaled = (short)(short)14592;
            p34.chan6_scaled = (short)(short) -15224;
            p34.chan7_scaled = (short)(short)24578;
            p34.chan8_scaled = (short)(short) -8275;
            p34.rssi = (byte)(byte)112;
            LoopBackDemoChannel.instance.send(p34); //===============================
            RC_CHANNELS_RAW p35 = LoopBackDemoChannel.new_RC_CHANNELS_RAW();
            PH.setPack(p35);
            p35.time_boot_ms = (uint)2759038207U;
            p35.port = (byte)(byte)206;
            p35.chan1_raw = (ushort)(ushort)39647;
            p35.chan2_raw = (ushort)(ushort)41927;
            p35.chan3_raw = (ushort)(ushort)56019;
            p35.chan4_raw = (ushort)(ushort)46704;
            p35.chan5_raw = (ushort)(ushort)34176;
            p35.chan6_raw = (ushort)(ushort)36632;
            p35.chan7_raw = (ushort)(ushort)12508;
            p35.chan8_raw = (ushort)(ushort)57592;
            p35.rssi = (byte)(byte)227;
            LoopBackDemoChannel.instance.send(p35); //===============================
            SERVO_OUTPUT_RAW p36 = LoopBackDemoChannel.new_SERVO_OUTPUT_RAW();
            PH.setPack(p36);
            p36.time_usec = (uint)31056739U;
            p36.port = (byte)(byte)215;
            p36.servo1_raw = (ushort)(ushort)47557;
            p36.servo2_raw = (ushort)(ushort)48001;
            p36.servo3_raw = (ushort)(ushort)48265;
            p36.servo4_raw = (ushort)(ushort)3905;
            p36.servo5_raw = (ushort)(ushort)61654;
            p36.servo6_raw = (ushort)(ushort)58980;
            p36.servo7_raw = (ushort)(ushort)26891;
            p36.servo8_raw = (ushort)(ushort)57019;
            p36.servo9_raw_SET((ushort)(ushort)55564, PH);
            p36.servo10_raw_SET((ushort)(ushort)18092, PH);
            p36.servo11_raw_SET((ushort)(ushort)11903, PH);
            p36.servo12_raw_SET((ushort)(ushort)7461, PH);
            p36.servo13_raw_SET((ushort)(ushort)28456, PH);
            p36.servo14_raw_SET((ushort)(ushort)64289, PH);
            p36.servo15_raw_SET((ushort)(ushort)15784, PH);
            p36.servo16_raw_SET((ushort)(ushort)39466, PH);
            LoopBackDemoChannel.instance.send(p36); //===============================
            MISSION_REQUEST_PARTIAL_LIST p37 = LoopBackDemoChannel.new_MISSION_REQUEST_PARTIAL_LIST();
            PH.setPack(p37);
            p37.target_system = (byte)(byte)118;
            p37.target_component = (byte)(byte)214;
            p37.start_index = (short)(short)27441;
            p37.end_index = (short)(short) -23;
            p37.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            LoopBackDemoChannel.instance.send(p37); //===============================
            MISSION_WRITE_PARTIAL_LIST p38 = LoopBackDemoChannel.new_MISSION_WRITE_PARTIAL_LIST();
            PH.setPack(p38);
            p38.target_system = (byte)(byte)156;
            p38.target_component = (byte)(byte)254;
            p38.start_index = (short)(short) -29864;
            p38.end_index = (short)(short) -23819;
            p38.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            LoopBackDemoChannel.instance.send(p38); //===============================
            MISSION_ITEM p39 = LoopBackDemoChannel.new_MISSION_ITEM();
            PH.setPack(p39);
            p39.target_system = (byte)(byte)37;
            p39.target_component = (byte)(byte)236;
            p39.seq = (ushort)(ushort)20626;
            p39.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
            p39.command = (MAV_CMD)MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_STANDARD;
            p39.current = (byte)(byte)205;
            p39.autocontinue = (byte)(byte)253;
            p39.param1 = (float)1.6290544E38F;
            p39.param2 = (float)2.310982E38F;
            p39.param3 = (float) -1.3429912E38F;
            p39.param4 = (float) -2.393187E38F;
            p39.x = (float)2.7104816E37F;
            p39.y = (float) -3.183651E38F;
            p39.z = (float)1.819339E38F;
            p39.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            LoopBackDemoChannel.instance.send(p39); //===============================
            MISSION_REQUEST p40 = LoopBackDemoChannel.new_MISSION_REQUEST();
            PH.setPack(p40);
            p40.target_system = (byte)(byte)43;
            p40.target_component = (byte)(byte)241;
            p40.seq = (ushort)(ushort)24878;
            p40.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            LoopBackDemoChannel.instance.send(p40); //===============================
            MISSION_SET_CURRENT p41 = LoopBackDemoChannel.new_MISSION_SET_CURRENT();
            PH.setPack(p41);
            p41.target_system = (byte)(byte)76;
            p41.target_component = (byte)(byte)26;
            p41.seq = (ushort)(ushort)38381;
            LoopBackDemoChannel.instance.send(p41); //===============================
            MISSION_CURRENT p42 = LoopBackDemoChannel.new_MISSION_CURRENT();
            PH.setPack(p42);
            p42.seq = (ushort)(ushort)58387;
            LoopBackDemoChannel.instance.send(p42); //===============================
            MISSION_REQUEST_LIST p43 = LoopBackDemoChannel.new_MISSION_REQUEST_LIST();
            PH.setPack(p43);
            p43.target_system = (byte)(byte)11;
            p43.target_component = (byte)(byte)171;
            p43.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            LoopBackDemoChannel.instance.send(p43); //===============================
            MISSION_COUNT p44 = LoopBackDemoChannel.new_MISSION_COUNT();
            PH.setPack(p44);
            p44.target_system = (byte)(byte)46;
            p44.target_component = (byte)(byte)82;
            p44.count = (ushort)(ushort)47359;
            p44.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            LoopBackDemoChannel.instance.send(p44); //===============================
            MISSION_CLEAR_ALL p45 = LoopBackDemoChannel.new_MISSION_CLEAR_ALL();
            PH.setPack(p45);
            p45.target_system = (byte)(byte)217;
            p45.target_component = (byte)(byte)206;
            p45.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            LoopBackDemoChannel.instance.send(p45); //===============================
            MISSION_ITEM_REACHED p46 = LoopBackDemoChannel.new_MISSION_ITEM_REACHED();
            PH.setPack(p46);
            p46.seq = (ushort)(ushort)59778;
            LoopBackDemoChannel.instance.send(p46); //===============================
            MISSION_ACK p47 = LoopBackDemoChannel.new_MISSION_ACK();
            PH.setPack(p47);
            p47.target_system = (byte)(byte)12;
            p47.target_component = (byte)(byte)30;
            p47.type = (MAV_MISSION_RESULT)MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM4;
            p47.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            LoopBackDemoChannel.instance.send(p47); //===============================
            SET_GPS_GLOBAL_ORIGIN p48 = LoopBackDemoChannel.new_SET_GPS_GLOBAL_ORIGIN();
            PH.setPack(p48);
            p48.target_system = (byte)(byte)201;
            p48.latitude = (int) -1860168059;
            p48.longitude = (int) -633757412;
            p48.altitude = (int)1994681925;
            p48.time_usec_SET((ulong)3338743376867013139L, PH);
            LoopBackDemoChannel.instance.send(p48); //===============================
            GPS_GLOBAL_ORIGIN p49 = LoopBackDemoChannel.new_GPS_GLOBAL_ORIGIN();
            PH.setPack(p49);
            p49.latitude = (int)1393633133;
            p49.longitude = (int) -1511693076;
            p49.altitude = (int) -11341018;
            p49.time_usec_SET((ulong)2623898319958756538L, PH);
            LoopBackDemoChannel.instance.send(p49); //===============================
            PARAM_MAP_RC p50 = LoopBackDemoChannel.new_PARAM_MAP_RC();
            PH.setPack(p50);
            p50.target_system = (byte)(byte)13;
            p50.target_component = (byte)(byte)182;
            p50.param_id_SET("DEMO", PH);
            p50.param_index = (short)(short) -6550;
            p50.parameter_rc_channel_index = (byte)(byte)231;
            p50.param_value0 = (float) -3.5131152E37F;
            p50.scale = (float) -2.4633964E38F;
            p50.param_value_min = (float)3.3287322E38F;
            p50.param_value_max = (float)6.1299045E37F;
            LoopBackDemoChannel.instance.send(p50); //===============================
            MISSION_REQUEST_INT p51 = LoopBackDemoChannel.new_MISSION_REQUEST_INT();
            PH.setPack(p51);
            p51.target_system = (byte)(byte)218;
            p51.target_component = (byte)(byte)42;
            p51.seq = (ushort)(ushort)59856;
            p51.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            LoopBackDemoChannel.instance.send(p51); //===============================
            SAFETY_SET_ALLOWED_AREA p54 = LoopBackDemoChannel.new_SAFETY_SET_ALLOWED_AREA();
            PH.setPack(p54);
            p54.target_system = (byte)(byte)103;
            p54.target_component = (byte)(byte)251;
            p54.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_ENU;
            p54.p1x = (float) -6.720467E37F;
            p54.p1y = (float)2.366638E38F;
            p54.p1z = (float)5.2839287E37F;
            p54.p2x = (float)1.375982E38F;
            p54.p2y = (float) -9.548462E37F;
            p54.p2z = (float)4.955589E37F;
            LoopBackDemoChannel.instance.send(p54); //===============================
            SAFETY_ALLOWED_AREA p55 = LoopBackDemoChannel.new_SAFETY_ALLOWED_AREA();
            PH.setPack(p55);
            p55.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL;
            p55.p1x = (float)2.192768E38F;
            p55.p1y = (float) -1.9262423E38F;
            p55.p1z = (float) -1.5481939E38F;
            p55.p2x = (float)1.4200191E38F;
            p55.p2y = (float)4.1569032E37F;
            p55.p2z = (float) -6.8414044E36F;
            LoopBackDemoChannel.instance.send(p55); //===============================
            ATTITUDE_QUATERNION_COV p61 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION_COV();
            PH.setPack(p61);
            p61.time_usec = (ulong)4189205058795096746L;
            p61.q_SET(new float[4], 0);
            p61.rollspeed = (float) -3.1474886E38F;
            p61.pitchspeed = (float) -2.4616187E38F;
            p61.yawspeed = (float)1.6440049E38F;
            p61.covariance_SET(new float[9], 0);
            LoopBackDemoChannel.instance.send(p61); //===============================
            NAV_CONTROLLER_OUTPUT p62 = LoopBackDemoChannel.new_NAV_CONTROLLER_OUTPUT();
            PH.setPack(p62);
            p62.nav_roll = (float)7.5711233E37F;
            p62.nav_pitch = (float) -4.449117E37F;
            p62.nav_bearing = (short)(short)16420;
            p62.target_bearing = (short)(short) -26593;
            p62.wp_dist = (ushort)(ushort)50176;
            p62.alt_error = (float) -2.5379639E38F;
            p62.aspd_error = (float)2.286329E38F;
            p62.xtrack_error = (float)1.933297E38F;
            LoopBackDemoChannel.instance.send(p62); //===============================
            GLOBAL_POSITION_INT_COV p63 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT_COV();
            PH.setPack(p63);
            p63.time_usec = (ulong)7947121612020339445L;
            p63.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS;
            p63.lat = (int) -964335435;
            p63.lon = (int)227045350;
            p63.alt = (int) -890004721;
            p63.relative_alt = (int) -428171593;
            p63.vx = (float)1.4152578E38F;
            p63.vy = (float) -2.5640978E38F;
            p63.vz = (float)2.1272641E38F;
            p63.covariance_SET(new float[36], 0);
            LoopBackDemoChannel.instance.send(p63); //===============================
            LOCAL_POSITION_NED_COV p64 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_COV();
            PH.setPack(p64);
            p64.time_usec = (ulong)6468505517712013236L;
            p64.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO;
            p64.x = (float)2.9413577E38F;
            p64.y = (float)1.1148871E38F;
            p64.z = (float)3.2952969E38F;
            p64.vx = (float) -2.8444964E38F;
            p64.vy = (float) -2.016383E38F;
            p64.vz = (float) -2.7184763E38F;
            p64.ax = (float) -1.2700308E38F;
            p64.ay = (float)1.8434944E38F;
            p64.az = (float)1.2282995E38F;
            p64.covariance_SET(new float[45], 0);
            LoopBackDemoChannel.instance.send(p64); //===============================
            RC_CHANNELS p65 = LoopBackDemoChannel.new_RC_CHANNELS();
            PH.setPack(p65);
            p65.time_boot_ms = (uint)923780137U;
            p65.chancount = (byte)(byte)146;
            p65.chan1_raw = (ushort)(ushort)1953;
            p65.chan2_raw = (ushort)(ushort)34474;
            p65.chan3_raw = (ushort)(ushort)39240;
            p65.chan4_raw = (ushort)(ushort)29568;
            p65.chan5_raw = (ushort)(ushort)18535;
            p65.chan6_raw = (ushort)(ushort)10420;
            p65.chan7_raw = (ushort)(ushort)64532;
            p65.chan8_raw = (ushort)(ushort)61397;
            p65.chan9_raw = (ushort)(ushort)23619;
            p65.chan10_raw = (ushort)(ushort)29929;
            p65.chan11_raw = (ushort)(ushort)58908;
            p65.chan12_raw = (ushort)(ushort)42752;
            p65.chan13_raw = (ushort)(ushort)13639;
            p65.chan14_raw = (ushort)(ushort)441;
            p65.chan15_raw = (ushort)(ushort)44660;
            p65.chan16_raw = (ushort)(ushort)25413;
            p65.chan17_raw = (ushort)(ushort)21894;
            p65.chan18_raw = (ushort)(ushort)60810;
            p65.rssi = (byte)(byte)221;
            LoopBackDemoChannel.instance.send(p65); //===============================
            REQUEST_DATA_STREAM p66 = LoopBackDemoChannel.new_REQUEST_DATA_STREAM();
            PH.setPack(p66);
            p66.target_system = (byte)(byte)107;
            p66.target_component = (byte)(byte)240;
            p66.req_stream_id = (byte)(byte)69;
            p66.req_message_rate = (ushort)(ushort)58521;
            p66.start_stop = (byte)(byte)54;
            LoopBackDemoChannel.instance.send(p66); //===============================
            DATA_STREAM p67 = LoopBackDemoChannel.new_DATA_STREAM();
            PH.setPack(p67);
            p67.stream_id = (byte)(byte)70;
            p67.message_rate = (ushort)(ushort)60102;
            p67.on_off = (byte)(byte)74;
            LoopBackDemoChannel.instance.send(p67); //===============================
            MANUAL_CONTROL p69 = LoopBackDemoChannel.new_MANUAL_CONTROL();
            PH.setPack(p69);
            p69.target = (byte)(byte)126;
            p69.x = (short)(short)32027;
            p69.y = (short)(short)28169;
            p69.z = (short)(short) -2121;
            p69.r = (short)(short) -30813;
            p69.buttons = (ushort)(ushort)53915;
            LoopBackDemoChannel.instance.send(p69); //===============================
            RC_CHANNELS_OVERRIDE p70 = LoopBackDemoChannel.new_RC_CHANNELS_OVERRIDE();
            PH.setPack(p70);
            p70.target_system = (byte)(byte)13;
            p70.target_component = (byte)(byte)251;
            p70.chan1_raw = (ushort)(ushort)49710;
            p70.chan2_raw = (ushort)(ushort)31320;
            p70.chan3_raw = (ushort)(ushort)14255;
            p70.chan4_raw = (ushort)(ushort)61371;
            p70.chan5_raw = (ushort)(ushort)18671;
            p70.chan6_raw = (ushort)(ushort)47980;
            p70.chan7_raw = (ushort)(ushort)27682;
            p70.chan8_raw = (ushort)(ushort)34220;
            LoopBackDemoChannel.instance.send(p70); //===============================
            MISSION_ITEM_INT p73 = LoopBackDemoChannel.new_MISSION_ITEM_INT();
            PH.setPack(p73);
            p73.target_system = (byte)(byte)207;
            p73.target_component = (byte)(byte)195;
            p73.seq = (ushort)(ushort)1000;
            p73.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p73.command = (MAV_CMD)MAV_CMD.MAV_CMD_GET_HOME_POSITION;
            p73.current = (byte)(byte)103;
            p73.autocontinue = (byte)(byte)26;
            p73.param1 = (float)1.9218686E38F;
            p73.param2 = (float) -2.3040168E37F;
            p73.param3 = (float) -3.2094932E38F;
            p73.param4 = (float)8.925098E37F;
            p73.x = (int)1555695747;
            p73.y = (int)666792832;
            p73.z = (float) -3.0259086E38F;
            p73.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            LoopBackDemoChannel.instance.send(p73); //===============================
            VFR_HUD p74 = LoopBackDemoChannel.new_VFR_HUD();
            PH.setPack(p74);
            p74.airspeed = (float)1.4672259E38F;
            p74.groundspeed = (float)1.8856374E38F;
            p74.heading = (short)(short)13589;
            p74.throttle = (ushort)(ushort)14721;
            p74.alt = (float) -1.9487992E38F;
            p74.climb = (float)6.7356686E37F;
            LoopBackDemoChannel.instance.send(p74); //===============================
            COMMAND_INT p75 = LoopBackDemoChannel.new_COMMAND_INT();
            PH.setPack(p75);
            p75.target_system = (byte)(byte)192;
            p75.target_component = (byte)(byte)32;
            p75.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
            p75.command = (MAV_CMD)MAV_CMD.MAV_CMD_STORAGE_FORMAT;
            p75.current = (byte)(byte)174;
            p75.autocontinue = (byte)(byte)235;
            p75.param1 = (float)2.2369343E37F;
            p75.param2 = (float) -3.3850129E38F;
            p75.param3 = (float)1.3447854E38F;
            p75.param4 = (float)2.994087E38F;
            p75.x = (int)492539011;
            p75.y = (int) -1972125906;
            p75.z = (float) -7.578321E37F;
            LoopBackDemoChannel.instance.send(p75); //===============================
            COMMAND_LONG p76 = LoopBackDemoChannel.new_COMMAND_LONG();
            PH.setPack(p76);
            p76.target_system = (byte)(byte)24;
            p76.target_component = (byte)(byte)125;
            p76.command = (MAV_CMD)MAV_CMD.MAV_CMD_PANORAMA_CREATE;
            p76.confirmation = (byte)(byte)227;
            p76.param1 = (float)2.8550911E38F;
            p76.param2 = (float) -1.4867852E38F;
            p76.param3 = (float)1.5668721E38F;
            p76.param4 = (float) -1.7007369E38F;
            p76.param5 = (float) -1.443553E38F;
            p76.param6 = (float) -6.2392627E37F;
            p76.param7 = (float)2.2990336E38F;
            LoopBackDemoChannel.instance.send(p76); //===============================
            COMMAND_ACK p77 = LoopBackDemoChannel.new_COMMAND_ACK();
            PH.setPack(p77);
            p77.command = (MAV_CMD)MAV_CMD.MAV_CMD_SPATIAL_USER_1;
            p77.result = (MAV_RESULT)MAV_RESULT.MAV_RESULT_IN_PROGRESS;
            p77.progress_SET((byte)(byte)219, PH);
            p77.result_param2_SET((int)1505099729, PH);
            p77.target_system_SET((byte)(byte)202, PH);
            p77.target_component_SET((byte)(byte)242, PH);
            LoopBackDemoChannel.instance.send(p77); //===============================
            MANUAL_SETPOINT p81 = LoopBackDemoChannel.new_MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.time_boot_ms = (uint)2355634851U;
            p81.roll = (float) -2.2045744E38F;
            p81.pitch = (float) -2.3706778E38F;
            p81.yaw = (float)1.5946652E38F;
            p81.thrust = (float)1.3311251E38F;
            p81.mode_switch = (byte)(byte)220;
            p81.manual_override_switch = (byte)(byte)7;
            LoopBackDemoChannel.instance.send(p81); //===============================
            SET_ATTITUDE_TARGET p82 = LoopBackDemoChannel.new_SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.time_boot_ms = (uint)987753101U;
            p82.target_system = (byte)(byte)95;
            p82.target_component = (byte)(byte)101;
            p82.type_mask = (byte)(byte)57;
            p82.q_SET(new float[4], 0);
            p82.body_roll_rate = (float) -6.757316E37F;
            p82.body_pitch_rate = (float) -9.919269E37F;
            p82.body_yaw_rate = (float) -2.812487E38F;
            p82.thrust = (float) -1.703822E38F;
            LoopBackDemoChannel.instance.send(p82); //===============================
            ATTITUDE_TARGET p83 = LoopBackDemoChannel.new_ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.time_boot_ms = (uint)4179933466U;
            p83.type_mask = (byte)(byte)135;
            p83.q_SET(new float[4], 0);
            p83.body_roll_rate = (float)1.7757674E38F;
            p83.body_pitch_rate = (float)2.0788717E38F;
            p83.body_yaw_rate = (float) -2.62139E38F;
            p83.thrust = (float) -2.6407803E38F;
            LoopBackDemoChannel.instance.send(p83); //===============================
            SET_POSITION_TARGET_LOCAL_NED p84 = LoopBackDemoChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.time_boot_ms = (uint)1736224353U;
            p84.target_system = (byte)(byte)25;
            p84.target_component = (byte)(byte)106;
            p84.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
            p84.type_mask = (ushort)(ushort)21486;
            p84.x = (float) -9.798563E37F;
            p84.y = (float)2.7771306E38F;
            p84.z = (float) -5.3391993E37F;
            p84.vx = (float) -1.3936857E38F;
            p84.vy = (float)6.876998E37F;
            p84.vz = (float)1.4509573E37F;
            p84.afx = (float)3.1681144E38F;
            p84.afy = (float) -1.812548E38F;
            p84.afz = (float)1.3502122E38F;
            p84.yaw = (float) -2.737065E38F;
            p84.yaw_rate = (float) -3.2383806E38F;
            LoopBackDemoChannel.instance.send(p84); //===============================
            SET_POSITION_TARGET_GLOBAL_INT p86 = LoopBackDemoChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.time_boot_ms = (uint)3636058743U;
            p86.target_system = (byte)(byte)88;
            p86.target_component = (byte)(byte)91;
            p86.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_ENU;
            p86.type_mask = (ushort)(ushort)34831;
            p86.lat_int = (int)1915450433;
            p86.lon_int = (int) -642110802;
            p86.alt = (float)3.3993821E38F;
            p86.vx = (float)3.0442617E38F;
            p86.vy = (float)2.6808638E38F;
            p86.vz = (float) -1.1229168E38F;
            p86.afx = (float)2.8911364E38F;
            p86.afy = (float) -3.3797277E38F;
            p86.afz = (float)1.606124E38F;
            p86.yaw = (float)1.760338E37F;
            p86.yaw_rate = (float)2.6950597E38F;
            LoopBackDemoChannel.instance.send(p86); //===============================
            POSITION_TARGET_GLOBAL_INT p87 = LoopBackDemoChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.time_boot_ms = (uint)3941929741U;
            p87.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_ENU;
            p87.type_mask = (ushort)(ushort)35555;
            p87.lat_int = (int) -109178822;
            p87.lon_int = (int)652752760;
            p87.alt = (float)2.9300423E38F;
            p87.vx = (float) -1.8259412E38F;
            p87.vy = (float) -1.6500292E38F;
            p87.vz = (float)2.7691026E38F;
            p87.afx = (float)2.7098813E37F;
            p87.afy = (float)1.3633989E38F;
            p87.afz = (float) -1.6305592E38F;
            p87.yaw = (float)4.5279055E37F;
            p87.yaw_rate = (float)1.7783607E38F;
            LoopBackDemoChannel.instance.send(p87); //===============================
            LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.time_boot_ms = (uint)204361636U;
            p89.x = (float) -2.7735053E38F;
            p89.y = (float)1.0161E38F;
            p89.z = (float) -5.436717E37F;
            p89.roll = (float) -1.3828408E38F;
            p89.pitch = (float) -1.2135039E38F;
            p89.yaw = (float) -2.3967166E38F;
            LoopBackDemoChannel.instance.send(p89); //===============================
            HIL_STATE p90 = LoopBackDemoChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.time_usec = (ulong)4996130438295702495L;
            p90.roll = (float)6.925243E37F;
            p90.pitch = (float) -9.800084E37F;
            p90.yaw = (float) -3.128364E38F;
            p90.rollspeed = (float)2.0200185E38F;
            p90.pitchspeed = (float)2.47271E38F;
            p90.yawspeed = (float) -6.5944006E37F;
            p90.lat = (int)2003615079;
            p90.lon = (int) -870164253;
            p90.alt = (int)1274056316;
            p90.vx = (short)(short) -7745;
            p90.vy = (short)(short)32325;
            p90.vz = (short)(short) -25005;
            p90.xacc = (short)(short) -7658;
            p90.yacc = (short)(short) -5815;
            p90.zacc = (short)(short) -31742;
            LoopBackDemoChannel.instance.send(p90); //===============================
            HIL_CONTROLS p91 = LoopBackDemoChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.time_usec = (ulong)5344524356287357105L;
            p91.roll_ailerons = (float)8.712442E37F;
            p91.pitch_elevator = (float)2.738264E38F;
            p91.yaw_rudder = (float)2.8844106E38F;
            p91.throttle = (float) -4.2781585E37F;
            p91.aux1 = (float) -2.7463525E38F;
            p91.aux2 = (float) -1.8121706E38F;
            p91.aux3 = (float) -1.7943894E38F;
            p91.aux4 = (float) -2.9104205E38F;
            p91.mode = (MAV_MODE)MAV_MODE.MAV_MODE_STABILIZE_ARMED;
            p91.nav_mode = (byte)(byte)109;
            LoopBackDemoChannel.instance.send(p91); //===============================
            HIL_RC_INPUTS_RAW p92 = LoopBackDemoChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.time_usec = (ulong)8172623180672855896L;
            p92.chan1_raw = (ushort)(ushort)20006;
            p92.chan2_raw = (ushort)(ushort)43738;
            p92.chan3_raw = (ushort)(ushort)35006;
            p92.chan4_raw = (ushort)(ushort)39814;
            p92.chan5_raw = (ushort)(ushort)39232;
            p92.chan6_raw = (ushort)(ushort)29138;
            p92.chan7_raw = (ushort)(ushort)40390;
            p92.chan8_raw = (ushort)(ushort)50742;
            p92.chan9_raw = (ushort)(ushort)60040;
            p92.chan10_raw = (ushort)(ushort)38676;
            p92.chan11_raw = (ushort)(ushort)9303;
            p92.chan12_raw = (ushort)(ushort)58396;
            p92.rssi = (byte)(byte)230;
            LoopBackDemoChannel.instance.send(p92); //===============================
            HIL_ACTUATOR_CONTROLS p93 = LoopBackDemoChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.time_usec = (ulong)307449569459379299L;
            p93.controls_SET(new float[16], 0);
            p93.mode = (MAV_MODE)MAV_MODE.MAV_MODE_MANUAL_DISARMED;
            p93.flags = (ulong)3210055931863411350L;
            LoopBackDemoChannel.instance.send(p93); //===============================
            OPTICAL_FLOW p100 = LoopBackDemoChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.time_usec = (ulong)6543057403498743962L;
            p100.sensor_id = (byte)(byte)243;
            p100.flow_x = (short)(short)996;
            p100.flow_y = (short)(short)2850;
            p100.flow_comp_m_x = (float) -1.2416177E38F;
            p100.flow_comp_m_y = (float) -1.1331725E38F;
            p100.quality = (byte)(byte)179;
            p100.ground_distance = (float) -2.712583E37F;
            p100.flow_rate_x_SET((float)5.4051065E37F, PH);
            p100.flow_rate_y_SET((float)1.3770182E38F, PH);
            LoopBackDemoChannel.instance.send(p100); //===============================
            GLOBAL_VISION_POSITION_ESTIMATE p101 = LoopBackDemoChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.usec = (ulong)5800295216823827795L;
            p101.x = (float) -6.6595436E37F;
            p101.y = (float) -3.767432E37F;
            p101.z = (float) -9.301555E37F;
            p101.roll = (float) -1.324348E38F;
            p101.pitch = (float) -2.0749864E38F;
            p101.yaw = (float)3.1493475E38F;
            LoopBackDemoChannel.instance.send(p101); //===============================
            VISION_POSITION_ESTIMATE p102 = LoopBackDemoChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.usec = (ulong)1163235346464403638L;
            p102.x = (float) -1.1330983E38F;
            p102.y = (float) -5.931058E37F;
            p102.z = (float) -2.3556566E37F;
            p102.roll = (float) -1.7011677E38F;
            p102.pitch = (float)1.977671E38F;
            p102.yaw = (float) -4.106443E37F;
            LoopBackDemoChannel.instance.send(p102); //===============================
            VISION_SPEED_ESTIMATE p103 = LoopBackDemoChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.usec = (ulong)7138750969883828409L;
            p103.x = (float) -8.307503E37F;
            p103.y = (float) -1.4410087E38F;
            p103.z = (float)1.9185099E38F;
            LoopBackDemoChannel.instance.send(p103); //===============================
            VICON_POSITION_ESTIMATE p104 = LoopBackDemoChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.usec = (ulong)1967752201788349437L;
            p104.x = (float)1.5423759E38F;
            p104.y = (float)2.3275872E38F;
            p104.z = (float)3.3958124E38F;
            p104.roll = (float) -1.0591078E38F;
            p104.pitch = (float) -6.062858E37F;
            p104.yaw = (float) -1.5939022E38F;
            LoopBackDemoChannel.instance.send(p104); //===============================
            HIGHRES_IMU p105 = LoopBackDemoChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.time_usec = (ulong)1032020810488970225L;
            p105.xacc = (float)2.0294778E38F;
            p105.yacc = (float) -9.135861E37F;
            p105.zacc = (float)3.3595319E38F;
            p105.xgyro = (float) -1.91976E38F;
            p105.ygyro = (float) -3.3926932E37F;
            p105.zgyro = (float)9.637024E37F;
            p105.xmag = (float) -3.354592E38F;
            p105.ymag = (float) -1.250039E38F;
            p105.zmag = (float) -2.9552702E38F;
            p105.abs_pressure = (float) -2.8952436E38F;
            p105.diff_pressure = (float) -3.061465E38F;
            p105.pressure_alt = (float)1.0844976E38F;
            p105.temperature = (float) -5.5836875E37F;
            p105.fields_updated = (ushort)(ushort)54003;
            LoopBackDemoChannel.instance.send(p105); //===============================
            OPTICAL_FLOW_RAD p106 = LoopBackDemoChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.time_usec = (ulong)332645812157744792L;
            p106.sensor_id = (byte)(byte)217;
            p106.integration_time_us = (uint)870265943U;
            p106.integrated_x = (float) -2.7304235E38F;
            p106.integrated_y = (float)4.9074903E36F;
            p106.integrated_xgyro = (float) -8.781903E37F;
            p106.integrated_ygyro = (float) -2.6395165E38F;
            p106.integrated_zgyro = (float) -1.1778084E38F;
            p106.temperature = (short)(short)23945;
            p106.quality = (byte)(byte)100;
            p106.time_delta_distance_us = (uint)433614558U;
            p106.distance = (float) -1.5597407E38F;
            LoopBackDemoChannel.instance.send(p106); //===============================
            HIL_SENSOR p107 = LoopBackDemoChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.time_usec = (ulong)7445029065851748371L;
            p107.xacc = (float) -8.967645E37F;
            p107.yacc = (float)1.3551506E38F;
            p107.zacc = (float) -3.3990121E37F;
            p107.xgyro = (float) -3.0273983E38F;
            p107.ygyro = (float)2.7933076E38F;
            p107.zgyro = (float) -1.9555573E38F;
            p107.xmag = (float)1.4361453E38F;
            p107.ymag = (float)1.1470551E37F;
            p107.zmag = (float)1.8854134E37F;
            p107.abs_pressure = (float)2.2069625E37F;
            p107.diff_pressure = (float) -2.2111009E38F;
            p107.pressure_alt = (float) -2.3614035E38F;
            p107.temperature = (float)1.9679322E37F;
            p107.fields_updated = (uint)1909635465U;
            LoopBackDemoChannel.instance.send(p107); //===============================
            SIM_STATE p108 = LoopBackDemoChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.q1 = (float)2.949027E38F;
            p108.q2 = (float) -1.4622255E38F;
            p108.q3 = (float) -1.718277E38F;
            p108.q4 = (float)4.0603622E37F;
            p108.roll = (float)3.2702524E38F;
            p108.pitch = (float) -3.0348306E37F;
            p108.yaw = (float)1.7819076E38F;
            p108.xacc = (float)1.7947128E37F;
            p108.yacc = (float)1.9389497E38F;
            p108.zacc = (float) -6.49147E36F;
            p108.xgyro = (float)4.741439E37F;
            p108.ygyro = (float)1.178946E38F;
            p108.zgyro = (float)2.5536683E38F;
            p108.lat = (float) -3.1101262E38F;
            p108.lon = (float)2.535552E38F;
            p108.alt = (float)2.155508E38F;
            p108.std_dev_horz = (float)1.0136734E38F;
            p108.std_dev_vert = (float)3.3810325E38F;
            p108.vn = (float) -7.643557E37F;
            p108.ve = (float)1.5915814E38F;
            p108.vd = (float)2.0652961E38F;
            LoopBackDemoChannel.instance.send(p108); //===============================
            RADIO_STATUS p109 = LoopBackDemoChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.rssi = (byte)(byte)52;
            p109.remrssi = (byte)(byte)17;
            p109.txbuf = (byte)(byte)22;
            p109.noise = (byte)(byte)124;
            p109.remnoise = (byte)(byte)131;
            p109.rxerrors = (ushort)(ushort)24087;
            p109.fixed_ = (ushort)(ushort)64216;
            LoopBackDemoChannel.instance.send(p109); //===============================
            FILE_TRANSFER_PROTOCOL p110 = LoopBackDemoChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_network = (byte)(byte)227;
            p110.target_system = (byte)(byte)118;
            p110.target_component = (byte)(byte)12;
            p110.payload_SET(new byte[251], 0);
            LoopBackDemoChannel.instance.send(p110); //===============================
            TIMESYNC p111 = LoopBackDemoChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.tc1 = (long) -5226981166155583487L;
            p111.ts1 = (long)1764971092597530550L;
            LoopBackDemoChannel.instance.send(p111); //===============================
            CAMERA_TRIGGER p112 = LoopBackDemoChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.time_usec = (ulong)8743110503133821525L;
            p112.seq = (uint)1479472069U;
            LoopBackDemoChannel.instance.send(p112); //===============================
            HIL_GPS p113 = LoopBackDemoChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.time_usec = (ulong)6201621727956422927L;
            p113.fix_type = (byte)(byte)60;
            p113.lat = (int) -2083830323;
            p113.lon = (int) -679858247;
            p113.alt = (int) -1123325983;
            p113.eph = (ushort)(ushort)33414;
            p113.epv = (ushort)(ushort)41198;
            p113.vel = (ushort)(ushort)9578;
            p113.vn = (short)(short) -21139;
            p113.ve = (short)(short)13091;
            p113.vd = (short)(short)6643;
            p113.cog = (ushort)(ushort)577;
            p113.satellites_visible = (byte)(byte)223;
            LoopBackDemoChannel.instance.send(p113); //===============================
            HIL_OPTICAL_FLOW p114 = LoopBackDemoChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.time_usec = (ulong)1070103511109494588L;
            p114.sensor_id = (byte)(byte)7;
            p114.integration_time_us = (uint)3514310893U;
            p114.integrated_x = (float) -2.808921E38F;
            p114.integrated_y = (float) -1.6134398E38F;
            p114.integrated_xgyro = (float) -2.6565002E38F;
            p114.integrated_ygyro = (float) -1.3801334E38F;
            p114.integrated_zgyro = (float)8.3005584E37F;
            p114.temperature = (short)(short)6695;
            p114.quality = (byte)(byte)11;
            p114.time_delta_distance_us = (uint)3683561942U;
            p114.distance = (float)2.6091357E38F;
            LoopBackDemoChannel.instance.send(p114); //===============================
            HIL_STATE_QUATERNION p115 = LoopBackDemoChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.time_usec = (ulong)350234999466306129L;
            p115.attitude_quaternion_SET(new float[4], 0);
            p115.rollspeed = (float)3.0566755E38F;
            p115.pitchspeed = (float) -2.4057777E38F;
            p115.yawspeed = (float)2.2902613E38F;
            p115.lat = (int)216178442;
            p115.lon = (int)304396649;
            p115.alt = (int) -1468503274;
            p115.vx = (short)(short)21398;
            p115.vy = (short)(short) -28758;
            p115.vz = (short)(short)32494;
            p115.ind_airspeed = (ushort)(ushort)26462;
            p115.true_airspeed = (ushort)(ushort)4063;
            p115.xacc = (short)(short)13984;
            p115.yacc = (short)(short) -18377;
            p115.zacc = (short)(short) -26569;
            LoopBackDemoChannel.instance.send(p115); //===============================
            SCALED_IMU2 p116 = LoopBackDemoChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.time_boot_ms = (uint)2623587258U;
            p116.xacc = (short)(short)26643;
            p116.yacc = (short)(short) -15336;
            p116.zacc = (short)(short) -21530;
            p116.xgyro = (short)(short)7600;
            p116.ygyro = (short)(short)12204;
            p116.zgyro = (short)(short) -21946;
            p116.xmag = (short)(short) -16094;
            p116.ymag = (short)(short)3227;
            p116.zmag = (short)(short) -25516;
            LoopBackDemoChannel.instance.send(p116); //===============================
            LOG_REQUEST_LIST p117 = LoopBackDemoChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.target_system = (byte)(byte)194;
            p117.target_component = (byte)(byte)72;
            p117.start = (ushort)(ushort)39873;
            p117.end = (ushort)(ushort)49777;
            LoopBackDemoChannel.instance.send(p117); //===============================
            LOG_ENTRY p118 = LoopBackDemoChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.id = (ushort)(ushort)54845;
            p118.num_logs = (ushort)(ushort)56969;
            p118.last_log_num = (ushort)(ushort)51695;
            p118.time_utc = (uint)329776651U;
            p118.size = (uint)1042239023U;
            LoopBackDemoChannel.instance.send(p118); //===============================
            LOG_REQUEST_DATA p119 = LoopBackDemoChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.target_system = (byte)(byte)228;
            p119.target_component = (byte)(byte)5;
            p119.id = (ushort)(ushort)40975;
            p119.ofs = (uint)1961620032U;
            p119.count = (uint)3078994994U;
            LoopBackDemoChannel.instance.send(p119); //===============================
            LOG_DATA p120 = LoopBackDemoChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.id = (ushort)(ushort)57159;
            p120.ofs = (uint)3373491156U;
            p120.count = (byte)(byte)111;
            p120.data__SET(new byte[90], 0);
            LoopBackDemoChannel.instance.send(p120); //===============================
            LOG_ERASE p121 = LoopBackDemoChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_system = (byte)(byte)139;
            p121.target_component = (byte)(byte)189;
            LoopBackDemoChannel.instance.send(p121); //===============================
            LOG_REQUEST_END p122 = LoopBackDemoChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_system = (byte)(byte)110;
            p122.target_component = (byte)(byte)144;
            LoopBackDemoChannel.instance.send(p122); //===============================
            GPS_INJECT_DATA p123 = LoopBackDemoChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.target_system = (byte)(byte)135;
            p123.target_component = (byte)(byte)29;
            p123.len = (byte)(byte)246;
            p123.data__SET(new byte[110], 0);
            LoopBackDemoChannel.instance.send(p123); //===============================
            GPS2_RAW p124 = LoopBackDemoChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.time_usec = (ulong)2573027724814858300L;
            p124.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX;
            p124.lat = (int)1086605343;
            p124.lon = (int) -1846027379;
            p124.alt = (int) -550139388;
            p124.eph = (ushort)(ushort)34790;
            p124.epv = (ushort)(ushort)45539;
            p124.vel = (ushort)(ushort)4358;
            p124.cog = (ushort)(ushort)62345;
            p124.satellites_visible = (byte)(byte)242;
            p124.dgps_numch = (byte)(byte)247;
            p124.dgps_age = (uint)1612547935U;
            LoopBackDemoChannel.instance.send(p124); //===============================
            POWER_STATUS p125 = LoopBackDemoChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vcc = (ushort)(ushort)61387;
            p125.Vservo = (ushort)(ushort)33466;
            p125.flags = (MAV_POWER_STATUS)MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT;
            LoopBackDemoChannel.instance.send(p125); //===============================
            SERIAL_CONTROL p126 = LoopBackDemoChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.device = (SERIAL_CONTROL_DEV)SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1;
            p126.flags = (SERIAL_CONTROL_FLAG)SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI;
            p126.timeout = (ushort)(ushort)11304;
            p126.baudrate = (uint)378351625U;
            p126.count = (byte)(byte)183;
            p126.data__SET(new byte[70], 0);
            LoopBackDemoChannel.instance.send(p126); //===============================
            GPS_RTK p127 = LoopBackDemoChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.time_last_baseline_ms = (uint)1876224352U;
            p127.rtk_receiver_id = (byte)(byte)228;
            p127.wn = (ushort)(ushort)80;
            p127.tow = (uint)2086794646U;
            p127.rtk_health = (byte)(byte)215;
            p127.rtk_rate = (byte)(byte)234;
            p127.nsats = (byte)(byte)92;
            p127.baseline_coords_type = (byte)(byte)65;
            p127.baseline_a_mm = (int)1890900691;
            p127.baseline_b_mm = (int)549138774;
            p127.baseline_c_mm = (int)1422370576;
            p127.accuracy = (uint)2177156184U;
            p127.iar_num_hypotheses = (int) -1685491852;
            LoopBackDemoChannel.instance.send(p127); //===============================
            GPS2_RTK p128 = LoopBackDemoChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.time_last_baseline_ms = (uint)2013098809U;
            p128.rtk_receiver_id = (byte)(byte)201;
            p128.wn = (ushort)(ushort)16629;
            p128.tow = (uint)1276941351U;
            p128.rtk_health = (byte)(byte)46;
            p128.rtk_rate = (byte)(byte)235;
            p128.nsats = (byte)(byte)203;
            p128.baseline_coords_type = (byte)(byte)111;
            p128.baseline_a_mm = (int)1349568820;
            p128.baseline_b_mm = (int)1078221950;
            p128.baseline_c_mm = (int)1677677259;
            p128.accuracy = (uint)376024960U;
            p128.iar_num_hypotheses = (int)1308133130;
            LoopBackDemoChannel.instance.send(p128); //===============================
            SCALED_IMU3 p129 = LoopBackDemoChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.time_boot_ms = (uint)855548133U;
            p129.xacc = (short)(short)20411;
            p129.yacc = (short)(short)19171;
            p129.zacc = (short)(short)29637;
            p129.xgyro = (short)(short) -17209;
            p129.ygyro = (short)(short) -5893;
            p129.zgyro = (short)(short) -17440;
            p129.xmag = (short)(short)17134;
            p129.ymag = (short)(short)577;
            p129.zmag = (short)(short)12071;
            LoopBackDemoChannel.instance.send(p129); //===============================
            DATA_TRANSMISSION_HANDSHAKE p130 = LoopBackDemoChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.type = (byte)(byte)25;
            p130.size = (uint)3671280312U;
            p130.width = (ushort)(ushort)20483;
            p130.height = (ushort)(ushort)64697;
            p130.packets = (ushort)(ushort)26643;
            p130.payload = (byte)(byte)23;
            p130.jpg_quality = (byte)(byte)32;
            LoopBackDemoChannel.instance.send(p130); //===============================
            ENCAPSULATED_DATA p131 = LoopBackDemoChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.seqnr = (ushort)(ushort)36047;
            p131.data__SET(new byte[253], 0);
            LoopBackDemoChannel.instance.send(p131); //===============================
            DISTANCE_SENSOR p132 = LoopBackDemoChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.time_boot_ms = (uint)2826569120U;
            p132.min_distance = (ushort)(ushort)17997;
            p132.max_distance = (ushort)(ushort)8296;
            p132.current_distance = (ushort)(ushort)13410;
            p132.type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER;
            p132.id = (byte)(byte)223;
            p132.orientation = (MAV_SENSOR_ORIENTATION)MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_270_PITCH_180;
            p132.covariance = (byte)(byte)78;
            LoopBackDemoChannel.instance.send(p132); //===============================
            TERRAIN_REQUEST p133 = LoopBackDemoChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.lat = (int) -1674513778;
            p133.lon = (int) -1188146353;
            p133.grid_spacing = (ushort)(ushort)384;
            p133.mask = (ulong)1206958194027835324L;
            LoopBackDemoChannel.instance.send(p133); //===============================
            TERRAIN_DATA p134 = LoopBackDemoChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.lat = (int)1627417607;
            p134.lon = (int) -656523918;
            p134.grid_spacing = (ushort)(ushort)53772;
            p134.gridbit = (byte)(byte)147;
            p134.data__SET(new short[16], 0);
            LoopBackDemoChannel.instance.send(p134); //===============================
            TERRAIN_CHECK p135 = LoopBackDemoChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lat = (int)2048976884;
            p135.lon = (int) -238593997;
            LoopBackDemoChannel.instance.send(p135); //===============================
            TERRAIN_REPORT p136 = LoopBackDemoChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.lat = (int) -1538446794;
            p136.lon = (int) -1474820161;
            p136.spacing = (ushort)(ushort)276;
            p136.terrain_height = (float)1.3552106E38F;
            p136.current_height = (float)1.4961673E38F;
            p136.pending = (ushort)(ushort)30907;
            p136.loaded = (ushort)(ushort)1377;
            LoopBackDemoChannel.instance.send(p136); //===============================
            SCALED_PRESSURE2 p137 = LoopBackDemoChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.time_boot_ms = (uint)3756537643U;
            p137.press_abs = (float) -1.3762206E38F;
            p137.press_diff = (float)6.337722E37F;
            p137.temperature = (short)(short)32131;
            LoopBackDemoChannel.instance.send(p137); //===============================
            ATT_POS_MOCAP p138 = LoopBackDemoChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.time_usec = (ulong)5064681572837329576L;
            p138.q_SET(new float[4], 0);
            p138.x = (float) -2.0815539E38F;
            p138.y = (float) -2.2821599E38F;
            p138.z = (float)1.6629482E38F;
            LoopBackDemoChannel.instance.send(p138); //===============================
            SET_ACTUATOR_CONTROL_TARGET p139 = LoopBackDemoChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.time_usec = (ulong)2287966988922821873L;
            p139.group_mlx = (byte)(byte)215;
            p139.target_system = (byte)(byte)207;
            p139.target_component = (byte)(byte)186;
            p139.controls_SET(new float[8], 0);
            LoopBackDemoChannel.instance.send(p139); //===============================
            ACTUATOR_CONTROL_TARGET p140 = LoopBackDemoChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.time_usec = (ulong)1864256291093792267L;
            p140.group_mlx = (byte)(byte)22;
            p140.controls_SET(new float[8], 0);
            LoopBackDemoChannel.instance.send(p140); //===============================
            ALTITUDE p141 = LoopBackDemoChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.time_usec = (ulong)8833009978082879443L;
            p141.altitude_monotonic = (float) -1.5827875E38F;
            p141.altitude_amsl = (float) -6.4764046E37F;
            p141.altitude_local = (float)2.4001816E37F;
            p141.altitude_relative = (float) -1.6976494E38F;
            p141.altitude_terrain = (float)3.3907999E38F;
            p141.bottom_clearance = (float)2.1341125E38F;
            LoopBackDemoChannel.instance.send(p141); //===============================
            RESOURCE_REQUEST p142 = LoopBackDemoChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.request_id = (byte)(byte)245;
            p142.uri_type = (byte)(byte)234;
            p142.uri_SET(new byte[120], 0);
            p142.transfer_type = (byte)(byte)202;
            p142.storage_SET(new byte[120], 0);
            LoopBackDemoChannel.instance.send(p142); //===============================
            SCALED_PRESSURE3 p143 = LoopBackDemoChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.time_boot_ms = (uint)3496322102U;
            p143.press_abs = (float) -2.3008323E38F;
            p143.press_diff = (float) -3.1493465E38F;
            p143.temperature = (short)(short)23880;
            LoopBackDemoChannel.instance.send(p143); //===============================
            FOLLOW_TARGET p144 = LoopBackDemoChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.timestamp = (ulong)2209614407008434856L;
            p144.est_capabilities = (byte)(byte)91;
            p144.lat = (int) -1022976938;
            p144.lon = (int)830122032;
            p144.alt = (float) -2.184336E38F;
            p144.vel_SET(new float[3], 0);
            p144.acc_SET(new float[3], 0);
            p144.attitude_q_SET(new float[4], 0);
            p144.rates_SET(new float[3], 0);
            p144.position_cov_SET(new float[3], 0);
            p144.custom_state = (ulong)8367284632503420743L;
            LoopBackDemoChannel.instance.send(p144); //===============================
            CONTROL_SYSTEM_STATE p146 = LoopBackDemoChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.time_usec = (ulong)299162535820499176L;
            p146.x_acc = (float) -1.753988E38F;
            p146.y_acc = (float) -9.244103E36F;
            p146.z_acc = (float) -1.8109267E38F;
            p146.x_vel = (float)1.828265E38F;
            p146.y_vel = (float) -3.2133628E38F;
            p146.z_vel = (float)1.0458934E38F;
            p146.x_pos = (float)1.4869484E38F;
            p146.y_pos = (float) -1.3333389E38F;
            p146.z_pos = (float)6.652002E37F;
            p146.airspeed = (float)9.914806E37F;
            p146.vel_variance_SET(new float[3], 0);
            p146.pos_variance_SET(new float[3], 0);
            p146.q_SET(new float[4], 0);
            p146.roll_rate = (float) -5.557582E37F;
            p146.pitch_rate = (float) -2.225683E38F;
            p146.yaw_rate = (float)2.4237494E38F;
            LoopBackDemoChannel.instance.send(p146); //===============================
            BATTERY_STATUS p147 = LoopBackDemoChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.id = (byte)(byte)71;
            p147.battery_function = (MAV_BATTERY_FUNCTION)MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_UNKNOWN;
            p147.type = (MAV_BATTERY_TYPE)MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_UNKNOWN;
            p147.temperature = (short)(short)2238;
            p147.voltages_SET(new ushort[10], 0);
            p147.current_battery = (short)(short) -17847;
            p147.current_consumed = (int) -1548511102;
            p147.energy_consumed = (int) -1884885992;
            p147.battery_remaining = (sbyte)(sbyte)52;
            LoopBackDemoChannel.instance.send(p147); //===============================
            AUTOPILOT_VERSION p148 = LoopBackDemoChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY)MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MAVLINK2;
            p148.flight_sw_version = (uint)1645144215U;
            p148.middleware_sw_version = (uint)3359136388U;
            p148.os_sw_version = (uint)291706217U;
            p148.board_version = (uint)4045710770U;
            p148.flight_custom_version_SET(new byte[8], 0);
            p148.middleware_custom_version_SET(new byte[8], 0);
            p148.os_custom_version_SET(new byte[8], 0);
            p148.vendor_id = (ushort)(ushort)40609;
            p148.product_id = (ushort)(ushort)24019;
            p148.uid = (ulong)4119606247626112727L;
            p148.uid2_SET(new byte[18], 0, PH);
            LoopBackDemoChannel.instance.send(p148); //===============================
            LANDING_TARGET p149 = LoopBackDemoChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.time_usec = (ulong)6533137787109634965L;
            p149.target_num = (byte)(byte)22;
            p149.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL;
            p149.angle_x = (float)2.0107482E38F;
            p149.angle_y = (float)1.6167763E38F;
            p149.distance = (float)3.1461611E38F;
            p149.size_x = (float) -2.0853236E38F;
            p149.size_y = (float)1.4354413E38F;
            p149.x_SET((float) -4.9417974E37F, PH);
            p149.y_SET((float) -2.7664923E38F, PH);
            p149.z_SET((float)2.3190874E37F, PH);
            p149.q_SET(new float[4], 0, PH);
            p149.type = (LANDING_TARGET_TYPE)LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER;
            p149.position_valid_SET((byte)(byte)65, PH);
            LoopBackDemoChannel.instance.send(p149); //===============================
            FLEXIFUNCTION_SET p150 = LoopBackDemoChannel.new_FLEXIFUNCTION_SET();
            PH.setPack(p150);
            p150.target_system = (byte)(byte)253;
            p150.target_component = (byte)(byte)194;
            LoopBackDemoChannel.instance.send(p150); //===============================
            FLEXIFUNCTION_READ_REQ p151 = LoopBackDemoChannel.new_FLEXIFUNCTION_READ_REQ();
            PH.setPack(p151);
            p151.target_system = (byte)(byte)219;
            p151.target_component = (byte)(byte)224;
            p151.read_req_type = (short)(short) -20879;
            p151.data_index = (short)(short)27487;
            LoopBackDemoChannel.instance.send(p151); //===============================
            FLEXIFUNCTION_BUFFER_FUNCTION p152 = LoopBackDemoChannel.new_FLEXIFUNCTION_BUFFER_FUNCTION();
            PH.setPack(p152);
            p152.target_system = (byte)(byte)11;
            p152.target_component = (byte)(byte)116;
            p152.func_index = (ushort)(ushort)12667;
            p152.func_count = (ushort)(ushort)11211;
            p152.data_address = (ushort)(ushort)30190;
            p152.data_size = (ushort)(ushort)23018;
            p152.data__SET(new sbyte[48], 0);
            LoopBackDemoChannel.instance.send(p152); //===============================
            FLEXIFUNCTION_BUFFER_FUNCTION_ACK p153 = LoopBackDemoChannel.new_FLEXIFUNCTION_BUFFER_FUNCTION_ACK();
            PH.setPack(p153);
            p153.target_system = (byte)(byte)157;
            p153.target_component = (byte)(byte)145;
            p153.func_index = (ushort)(ushort)22386;
            p153.result = (ushort)(ushort)4541;
            LoopBackDemoChannel.instance.send(p153); //===============================
            FLEXIFUNCTION_DIRECTORY p155 = LoopBackDemoChannel.new_FLEXIFUNCTION_DIRECTORY();
            PH.setPack(p155);
            p155.target_system = (byte)(byte)236;
            p155.target_component = (byte)(byte)3;
            p155.directory_type = (byte)(byte)60;
            p155.start_index = (byte)(byte)197;
            p155.count = (byte)(byte)142;
            p155.directory_data_SET(new sbyte[48], 0);
            LoopBackDemoChannel.instance.send(p155); //===============================
            FLEXIFUNCTION_DIRECTORY_ACK p156 = LoopBackDemoChannel.new_FLEXIFUNCTION_DIRECTORY_ACK();
            PH.setPack(p156);
            p156.target_system = (byte)(byte)66;
            p156.target_component = (byte)(byte)7;
            p156.directory_type = (byte)(byte)139;
            p156.start_index = (byte)(byte)250;
            p156.count = (byte)(byte)162;
            p156.result = (ushort)(ushort)63899;
            LoopBackDemoChannel.instance.send(p156); //===============================
            FLEXIFUNCTION_COMMAND p157 = LoopBackDemoChannel.new_FLEXIFUNCTION_COMMAND();
            PH.setPack(p157);
            p157.target_system = (byte)(byte)70;
            p157.target_component = (byte)(byte)76;
            p157.command_type = (byte)(byte)122;
            LoopBackDemoChannel.instance.send(p157); //===============================
            FLEXIFUNCTION_COMMAND_ACK p158 = LoopBackDemoChannel.new_FLEXIFUNCTION_COMMAND_ACK();
            PH.setPack(p158);
            p158.command_type = (ushort)(ushort)38458;
            p158.result = (ushort)(ushort)55283;
            LoopBackDemoChannel.instance.send(p158); //===============================
            SERIAL_UDB_EXTRA_F2_A p170 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F2_A();
            PH.setPack(p170);
            p170.sue_time = (uint)2603385262U;
            p170.sue_status = (byte)(byte)20;
            p170.sue_latitude = (int)1023601622;
            p170.sue_longitude = (int)285373066;
            p170.sue_altitude = (int)1704825118;
            p170.sue_waypoint_index = (ushort)(ushort)29774;
            p170.sue_rmat0 = (short)(short)25482;
            p170.sue_rmat1 = (short)(short)24673;
            p170.sue_rmat2 = (short)(short)31775;
            p170.sue_rmat3 = (short)(short) -15982;
            p170.sue_rmat4 = (short)(short) -376;
            p170.sue_rmat5 = (short)(short) -22315;
            p170.sue_rmat6 = (short)(short)24785;
            p170.sue_rmat7 = (short)(short)13356;
            p170.sue_rmat8 = (short)(short)11367;
            p170.sue_cog = (ushort)(ushort)43582;
            p170.sue_sog = (short)(short)15468;
            p170.sue_cpu_load = (ushort)(ushort)40074;
            p170.sue_air_speed_3DIMU = (ushort)(ushort)50515;
            p170.sue_estimated_wind_0 = (short)(short)2393;
            p170.sue_estimated_wind_1 = (short)(short)18653;
            p170.sue_estimated_wind_2 = (short)(short) -5893;
            p170.sue_magFieldEarth0 = (short)(short)15127;
            p170.sue_magFieldEarth1 = (short)(short) -8204;
            p170.sue_magFieldEarth2 = (short)(short)13561;
            p170.sue_svs = (short)(short)17427;
            p170.sue_hdop = (short)(short)23466;
            LoopBackDemoChannel.instance.send(p170); //===============================
            SERIAL_UDB_EXTRA_F2_B p171 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F2_B();
            PH.setPack(p171);
            p171.sue_time = (uint)509368174U;
            p171.sue_pwm_input_1 = (short)(short)28954;
            p171.sue_pwm_input_2 = (short)(short) -17338;
            p171.sue_pwm_input_3 = (short)(short)2193;
            p171.sue_pwm_input_4 = (short)(short)24950;
            p171.sue_pwm_input_5 = (short)(short)22832;
            p171.sue_pwm_input_6 = (short)(short)24327;
            p171.sue_pwm_input_7 = (short)(short)6802;
            p171.sue_pwm_input_8 = (short)(short) -9321;
            p171.sue_pwm_input_9 = (short)(short) -29827;
            p171.sue_pwm_input_10 = (short)(short) -30314;
            p171.sue_pwm_input_11 = (short)(short)2826;
            p171.sue_pwm_input_12 = (short)(short) -107;
            p171.sue_pwm_output_1 = (short)(short) -32386;
            p171.sue_pwm_output_2 = (short)(short)25020;
            p171.sue_pwm_output_3 = (short)(short) -9038;
            p171.sue_pwm_output_4 = (short)(short)23486;
            p171.sue_pwm_output_5 = (short)(short)25756;
            p171.sue_pwm_output_6 = (short)(short) -18229;
            p171.sue_pwm_output_7 = (short)(short) -28955;
            p171.sue_pwm_output_8 = (short)(short) -32136;
            p171.sue_pwm_output_9 = (short)(short) -14785;
            p171.sue_pwm_output_10 = (short)(short) -19330;
            p171.sue_pwm_output_11 = (short)(short)15846;
            p171.sue_pwm_output_12 = (short)(short)23632;
            p171.sue_imu_location_x = (short)(short) -17381;
            p171.sue_imu_location_y = (short)(short) -14833;
            p171.sue_imu_location_z = (short)(short)343;
            p171.sue_location_error_earth_x = (short)(short) -31155;
            p171.sue_location_error_earth_y = (short)(short)18044;
            p171.sue_location_error_earth_z = (short)(short)5164;
            p171.sue_flags = (uint)2543848778U;
            p171.sue_osc_fails = (short)(short) -32482;
            p171.sue_imu_velocity_x = (short)(short)8869;
            p171.sue_imu_velocity_y = (short)(short) -7564;
            p171.sue_imu_velocity_z = (short)(short) -31400;
            p171.sue_waypoint_goal_x = (short)(short)6760;
            p171.sue_waypoint_goal_y = (short)(short)24271;
            p171.sue_waypoint_goal_z = (short)(short) -25382;
            p171.sue_aero_x = (short)(short)19927;
            p171.sue_aero_y = (short)(short)2446;
            p171.sue_aero_z = (short)(short)24177;
            p171.sue_barom_temp = (short)(short) -8454;
            p171.sue_barom_press = (int)646846997;
            p171.sue_barom_alt = (int) -1992062602;
            p171.sue_bat_volt = (short)(short) -6185;
            p171.sue_bat_amp = (short)(short) -790;
            p171.sue_bat_amp_hours = (short)(short)20365;
            p171.sue_desired_height = (short)(short)23019;
            p171.sue_memory_stack_free = (short)(short) -11915;
            LoopBackDemoChannel.instance.send(p171); //===============================
            SERIAL_UDB_EXTRA_F4 p172 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F4();
            PH.setPack(p172);
            p172.sue_ROLL_STABILIZATION_AILERONS = (byte)(byte)9;
            p172.sue_ROLL_STABILIZATION_RUDDER = (byte)(byte)198;
            p172.sue_PITCH_STABILIZATION = (byte)(byte)91;
            p172.sue_YAW_STABILIZATION_RUDDER = (byte)(byte)255;
            p172.sue_YAW_STABILIZATION_AILERON = (byte)(byte)44;
            p172.sue_AILERON_NAVIGATION = (byte)(byte)210;
            p172.sue_RUDDER_NAVIGATION = (byte)(byte)129;
            p172.sue_ALTITUDEHOLD_STABILIZED = (byte)(byte)129;
            p172.sue_ALTITUDEHOLD_WAYPOINT = (byte)(byte)134;
            p172.sue_RACING_MODE = (byte)(byte)185;
            LoopBackDemoChannel.instance.send(p172); //===============================
            SERIAL_UDB_EXTRA_F5 p173 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F5();
            PH.setPack(p173);
            p173.sue_YAWKP_AILERON = (float) -2.4711346E38F;
            p173.sue_YAWKD_AILERON = (float) -1.1877571E38F;
            p173.sue_ROLLKP = (float) -1.8744849E38F;
            p173.sue_ROLLKD = (float)2.237702E38F;
            LoopBackDemoChannel.instance.send(p173); //===============================
            SERIAL_UDB_EXTRA_F6 p174 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F6();
            PH.setPack(p174);
            p174.sue_PITCHGAIN = (float)1.5678136E38F;
            p174.sue_PITCHKD = (float)3.2589163E38F;
            p174.sue_RUDDER_ELEV_MIX = (float) -1.4990702E38F;
            p174.sue_ROLL_ELEV_MIX = (float)2.7780135E38F;
            p174.sue_ELEVATOR_BOOST = (float) -2.1442056E38F;
            LoopBackDemoChannel.instance.send(p174); //===============================
            SERIAL_UDB_EXTRA_F7 p175 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F7();
            PH.setPack(p175);
            p175.sue_YAWKP_RUDDER = (float)1.9746976E38F;
            p175.sue_YAWKD_RUDDER = (float)2.9538249E38F;
            p175.sue_ROLLKP_RUDDER = (float) -1.6858181E38F;
            p175.sue_ROLLKD_RUDDER = (float)3.0217263E38F;
            p175.sue_RUDDER_BOOST = (float) -1.537808E38F;
            p175.sue_RTL_PITCH_DOWN = (float)2.3109398E38F;
            LoopBackDemoChannel.instance.send(p175); //===============================
            SERIAL_UDB_EXTRA_F8 p176 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F8();
            PH.setPack(p176);
            p176.sue_HEIGHT_TARGET_MAX = (float)9.326524E37F;
            p176.sue_HEIGHT_TARGET_MIN = (float)1.8820979E38F;
            p176.sue_ALT_HOLD_THROTTLE_MIN = (float)2.1926922E38F;
            p176.sue_ALT_HOLD_THROTTLE_MAX = (float)2.361057E37F;
            p176.sue_ALT_HOLD_PITCH_MIN = (float)1.9613336E38F;
            p176.sue_ALT_HOLD_PITCH_MAX = (float)3.2952865E38F;
            p176.sue_ALT_HOLD_PITCH_HIGH = (float)6.937929E37F;
            LoopBackDemoChannel.instance.send(p176); //===============================
            SERIAL_UDB_EXTRA_F13 p177 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F13();
            PH.setPack(p177);
            p177.sue_week_no = (short)(short)29842;
            p177.sue_lat_origin = (int)845114089;
            p177.sue_lon_origin = (int) -286246169;
            p177.sue_alt_origin = (int)26746551;
            LoopBackDemoChannel.instance.send(p177); //===============================
            SERIAL_UDB_EXTRA_F14 p178 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F14();
            PH.setPack(p178);
            p178.sue_WIND_ESTIMATION = (byte)(byte)215;
            p178.sue_GPS_TYPE = (byte)(byte)163;
            p178.sue_DR = (byte)(byte)255;
            p178.sue_BOARD_TYPE = (byte)(byte)141;
            p178.sue_AIRFRAME = (byte)(byte)82;
            p178.sue_RCON = (short)(short)32356;
            p178.sue_TRAP_FLAGS = (short)(short) -21565;
            p178.sue_TRAP_SOURCE = (uint)303109565U;
            p178.sue_osc_fail_count = (short)(short) -6312;
            p178.sue_CLOCK_CONFIG = (byte)(byte)102;
            p178.sue_FLIGHT_PLAN_TYPE = (byte)(byte)109;
            LoopBackDemoChannel.instance.send(p178); //===============================
            SERIAL_UDB_EXTRA_F15 p179 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F15();
            PH.setPack(p179);
            p179.sue_ID_VEHICLE_MODEL_NAME_SET(new byte[40], 0);
            p179.sue_ID_VEHICLE_REGISTRATION_SET(new byte[20], 0);
            LoopBackDemoChannel.instance.send(p179); //===============================
            SERIAL_UDB_EXTRA_F16 p180 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F16();
            PH.setPack(p180);
            p180.sue_ID_LEAD_PILOT_SET(new byte[40], 0);
            p180.sue_ID_DIY_DRONES_URL_SET(new byte[70], 0);
            LoopBackDemoChannel.instance.send(p180); //===============================
            ALTITUDES p181 = LoopBackDemoChannel.new_ALTITUDES();
            PH.setPack(p181);
            p181.time_boot_ms = (uint)2066722107U;
            p181.alt_gps = (int) -629087410;
            p181.alt_imu = (int)119333992;
            p181.alt_barometric = (int)458910623;
            p181.alt_optical_flow = (int) -1721621969;
            p181.alt_range_finder = (int) -1085834519;
            p181.alt_extra = (int)80711887;
            LoopBackDemoChannel.instance.send(p181); //===============================
            AIRSPEEDS p182 = LoopBackDemoChannel.new_AIRSPEEDS();
            PH.setPack(p182);
            p182.time_boot_ms = (uint)3473734321U;
            p182.airspeed_imu = (short)(short) -6226;
            p182.airspeed_pitot = (short)(short)9843;
            p182.airspeed_hot_wire = (short)(short)6359;
            p182.airspeed_ultrasonic = (short)(short) -15232;
            p182.aoa = (short)(short)5106;
            p182.aoy = (short)(short) -11356;
            LoopBackDemoChannel.instance.send(p182); //===============================
            SERIAL_UDB_EXTRA_F17 p183 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F17();
            PH.setPack(p183);
            p183.sue_feed_forward = (float)2.2878309E38F;
            p183.sue_turn_rate_nav = (float)2.0244248E38F;
            p183.sue_turn_rate_fbw = (float)1.748445E38F;
            LoopBackDemoChannel.instance.send(p183); //===============================
            SERIAL_UDB_EXTRA_F18 p184 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F18();
            PH.setPack(p184);
            p184.angle_of_attack_normal = (float) -1.8872174E38F;
            p184.angle_of_attack_inverted = (float) -2.191813E37F;
            p184.elevator_trim_normal = (float) -2.876746E38F;
            p184.elevator_trim_inverted = (float)3.2819249E38F;
            p184.reference_speed = (float)6.164672E37F;
            LoopBackDemoChannel.instance.send(p184); //===============================
            SERIAL_UDB_EXTRA_F19 p185 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F19();
            PH.setPack(p185);
            p185.sue_aileron_output_channel = (byte)(byte)82;
            p185.sue_aileron_reversed = (byte)(byte)57;
            p185.sue_elevator_output_channel = (byte)(byte)149;
            p185.sue_elevator_reversed = (byte)(byte)195;
            p185.sue_throttle_output_channel = (byte)(byte)138;
            p185.sue_throttle_reversed = (byte)(byte)140;
            p185.sue_rudder_output_channel = (byte)(byte)0;
            p185.sue_rudder_reversed = (byte)(byte)85;
            LoopBackDemoChannel.instance.send(p185); //===============================
            SERIAL_UDB_EXTRA_F20 p186 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F20();
            PH.setPack(p186);
            p186.sue_number_of_inputs = (byte)(byte)122;
            p186.sue_trim_value_input_1 = (short)(short) -24773;
            p186.sue_trim_value_input_2 = (short)(short)19880;
            p186.sue_trim_value_input_3 = (short)(short)5311;
            p186.sue_trim_value_input_4 = (short)(short)28131;
            p186.sue_trim_value_input_5 = (short)(short) -16457;
            p186.sue_trim_value_input_6 = (short)(short)5235;
            p186.sue_trim_value_input_7 = (short)(short)281;
            p186.sue_trim_value_input_8 = (short)(short) -12148;
            p186.sue_trim_value_input_9 = (short)(short) -6201;
            p186.sue_trim_value_input_10 = (short)(short) -5662;
            p186.sue_trim_value_input_11 = (short)(short)24129;
            p186.sue_trim_value_input_12 = (short)(short)31498;
            LoopBackDemoChannel.instance.send(p186); //===============================
            SERIAL_UDB_EXTRA_F21 p187 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F21();
            PH.setPack(p187);
            p187.sue_accel_x_offset = (short)(short)12418;
            p187.sue_accel_y_offset = (short)(short) -121;
            p187.sue_accel_z_offset = (short)(short) -28493;
            p187.sue_gyro_x_offset = (short)(short) -11849;
            p187.sue_gyro_y_offset = (short)(short) -25797;
            p187.sue_gyro_z_offset = (short)(short) -26845;
            LoopBackDemoChannel.instance.send(p187); //===============================
            SERIAL_UDB_EXTRA_F22 p188 = LoopBackDemoChannel.new_SERIAL_UDB_EXTRA_F22();
            PH.setPack(p188);
            p188.sue_accel_x_at_calibration = (short)(short) -28573;
            p188.sue_accel_y_at_calibration = (short)(short)11411;
            p188.sue_accel_z_at_calibration = (short)(short) -14930;
            p188.sue_gyro_x_at_calibration = (short)(short)2122;
            p188.sue_gyro_y_at_calibration = (short)(short)1533;
            p188.sue_gyro_z_at_calibration = (short)(short)6008;
            LoopBackDemoChannel.instance.send(p188); //===============================
            ESTIMATOR_STATUS p230 = LoopBackDemoChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.time_usec = (ulong)7873734238194762448L;
            p230.flags = (ESTIMATOR_STATUS_FLAGS)ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL;
            p230.vel_ratio = (float) -1.9435495E37F;
            p230.pos_horiz_ratio = (float) -2.619203E38F;
            p230.pos_vert_ratio = (float) -8.866305E37F;
            p230.mag_ratio = (float) -2.0901013E38F;
            p230.hagl_ratio = (float) -2.6513774E38F;
            p230.tas_ratio = (float)2.1748843E37F;
            p230.pos_horiz_accuracy = (float)2.569749E38F;
            p230.pos_vert_accuracy = (float)3.7342167E37F;
            LoopBackDemoChannel.instance.send(p230); //===============================
            WIND_COV p231 = LoopBackDemoChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.time_usec = (ulong)1009760141139088675L;
            p231.wind_x = (float) -2.0622258E38F;
            p231.wind_y = (float)2.5327588E38F;
            p231.wind_z = (float) -3.2537192E38F;
            p231.var_horiz = (float) -2.8010217E38F;
            p231.var_vert = (float)2.6383518E37F;
            p231.wind_alt = (float) -2.2331466E38F;
            p231.horiz_accuracy = (float)7.13E37F;
            p231.vert_accuracy = (float) -1.4423917E38F;
            LoopBackDemoChannel.instance.send(p231); //===============================
            GPS_INPUT p232 = LoopBackDemoChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.time_usec = (ulong)5308030960955567557L;
            p232.gps_id = (byte)(byte)51;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS)GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY;
            p232.time_week_ms = (uint)547804473U;
            p232.time_week = (ushort)(ushort)27390;
            p232.fix_type = (byte)(byte)32;
            p232.lat = (int) -929112424;
            p232.lon = (int) -1622011899;
            p232.alt = (float) -1.9113897E38F;
            p232.hdop = (float)2.6321733E37F;
            p232.vdop = (float) -3.246465E38F;
            p232.vn = (float)1.0365234E38F;
            p232.ve = (float)3.154951E38F;
            p232.vd = (float)3.0046814E38F;
            p232.speed_accuracy = (float) -1.6005285E38F;
            p232.horiz_accuracy = (float) -2.400058E38F;
            p232.vert_accuracy = (float) -7.3522173E37F;
            p232.satellites_visible = (byte)(byte)235;
            LoopBackDemoChannel.instance.send(p232); //===============================
            GPS_RTCM_DATA p233 = LoopBackDemoChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.flags = (byte)(byte)56;
            p233.len = (byte)(byte)131;
            p233.data__SET(new byte[180], 0);
            LoopBackDemoChannel.instance.send(p233); //===============================
            HIGH_LATENCY p234 = LoopBackDemoChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
            p234.custom_mode = (uint)2594936032U;
            p234.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING;
            p234.roll = (short)(short)28449;
            p234.pitch = (short)(short)12939;
            p234.heading = (ushort)(ushort)30828;
            p234.throttle = (sbyte)(sbyte) - 3;
            p234.heading_sp = (short)(short) -11680;
            p234.latitude = (int)2025068902;
            p234.longitude = (int) -1999331220;
            p234.altitude_amsl = (short)(short) -9965;
            p234.altitude_sp = (short)(short) -9064;
            p234.airspeed = (byte)(byte)222;
            p234.airspeed_sp = (byte)(byte)99;
            p234.groundspeed = (byte)(byte)242;
            p234.climb_rate = (sbyte)(sbyte)95;
            p234.gps_nsat = (byte)(byte)78;
            p234.gps_fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS;
            p234.battery_remaining = (byte)(byte)6;
            p234.temperature = (sbyte)(sbyte)109;
            p234.temperature_air = (sbyte)(sbyte)120;
            p234.failsafe = (byte)(byte)196;
            p234.wp_num = (byte)(byte)21;
            p234.wp_distance = (ushort)(ushort)8594;
            LoopBackDemoChannel.instance.send(p234); //===============================
            VIBRATION p241 = LoopBackDemoChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.time_usec = (ulong)904793436290142567L;
            p241.vibration_x = (float)1.2313049E38F;
            p241.vibration_y = (float) -1.9178917E38F;
            p241.vibration_z = (float)2.1381683E37F;
            p241.clipping_0 = (uint)4095023347U;
            p241.clipping_1 = (uint)2606718391U;
            p241.clipping_2 = (uint)2789150662U;
            LoopBackDemoChannel.instance.send(p241); //===============================
            HOME_POSITION p242 = LoopBackDemoChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.latitude = (int)1998345082;
            p242.longitude = (int) -902685870;
            p242.altitude = (int) -471461573;
            p242.x = (float) -2.4189057E38F;
            p242.y = (float) -8.847083E37F;
            p242.z = (float) -1.756563E38F;
            p242.q_SET(new float[4], 0);
            p242.approach_x = (float)2.4430923E38F;
            p242.approach_y = (float) -2.374128E38F;
            p242.approach_z = (float) -8.4972364E37F;
            p242.time_usec_SET((ulong)1306604411124695495L, PH);
            LoopBackDemoChannel.instance.send(p242); //===============================
            SET_HOME_POSITION p243 = LoopBackDemoChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.target_system = (byte)(byte)135;
            p243.latitude = (int)1416141560;
            p243.longitude = (int)1032068558;
            p243.altitude = (int) -1733684602;
            p243.x = (float)3.1740533E38F;
            p243.y = (float) -2.9075913E38F;
            p243.z = (float)9.331649E37F;
            p243.q_SET(new float[4], 0);
            p243.approach_x = (float) -3.362465E38F;
            p243.approach_y = (float)2.1796844E37F;
            p243.approach_z = (float) -7.5524615E37F;
            p243.time_usec_SET((ulong)4548507061571638597L, PH);
            LoopBackDemoChannel.instance.send(p243); //===============================
            MESSAGE_INTERVAL p244 = LoopBackDemoChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.message_id = (ushort)(ushort)3311;
            p244.interval_us = (int) -1845212940;
            LoopBackDemoChannel.instance.send(p244); //===============================
            EXTENDED_SYS_STATE p245 = LoopBackDemoChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.vtol_state = (MAV_VTOL_STATE)MAV_VTOL_STATE.MAV_VTOL_STATE_MC;
            p245.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING;
            LoopBackDemoChannel.instance.send(p245); //===============================
            ADSB_VEHICLE p246 = LoopBackDemoChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.ICAO_address = (uint)2783233210U;
            p246.lat = (int)1065653648;
            p246.lon = (int)995730165;
            p246.altitude_type = (ADSB_ALTITUDE_TYPE)ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
            p246.altitude = (int) -1995080205;
            p246.heading = (ushort)(ushort)51167;
            p246.hor_velocity = (ushort)(ushort)44346;
            p246.ver_velocity = (short)(short) -7629;
            p246.callsign_SET("DEMO", PH);
            p246.emitter_type = (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_LIGHT;
            p246.tslc = (byte)(byte)137;
            p246.flags = (ADSB_FLAGS)ADSB_FLAGS.ADSB_FLAGS_SIMULATED;
            p246.squawk = (ushort)(ushort)47190;
            LoopBackDemoChannel.instance.send(p246); //===============================
            COLLISION p247 = LoopBackDemoChannel.new_COLLISION();
            PH.setPack(p247);
            p247.src_ = (MAV_COLLISION_SRC)MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT;
            p247.id = (uint)2406276525U;
            p247.action = (MAV_COLLISION_ACTION)MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_ASCEND_OR_DESCEND;
            p247.threat_level = (MAV_COLLISION_THREAT_LEVEL)MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE;
            p247.time_to_minimum_delta = (float) -9.509616E37F;
            p247.altitude_minimum_delta = (float)2.6085446E38F;
            p247.horizontal_minimum_delta = (float) -3.3053637E37F;
            LoopBackDemoChannel.instance.send(p247); //===============================
            V2_EXTENSION p248 = LoopBackDemoChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.target_network = (byte)(byte)152;
            p248.target_system = (byte)(byte)29;
            p248.target_component = (byte)(byte)82;
            p248.message_type = (ushort)(ushort)24144;
            p248.payload_SET(new byte[249], 0);
            LoopBackDemoChannel.instance.send(p248); //===============================
            MEMORY_VECT p249 = LoopBackDemoChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.address = (ushort)(ushort)13844;
            p249.ver = (byte)(byte)31;
            p249.type = (byte)(byte)241;
            p249.value_SET(new sbyte[32], 0);
            LoopBackDemoChannel.instance.send(p249); //===============================
            DEBUG_VECT p250 = LoopBackDemoChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.name_SET("DEMO", PH);
            p250.time_usec = (ulong)4724733470497664432L;
            p250.x = (float)1.7175086E38F;
            p250.y = (float) -7.414707E37F;
            p250.z = (float)1.01456276E37F;
            LoopBackDemoChannel.instance.send(p250); //===============================
            NAMED_VALUE_FLOAT p251 = LoopBackDemoChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.time_boot_ms = (uint)2796928764U;
            p251.name_SET("DEMO", PH);
            p251.value = (float) -3.4172656E37F;
            LoopBackDemoChannel.instance.send(p251); //===============================
            NAMED_VALUE_INT p252 = LoopBackDemoChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.time_boot_ms = (uint)3627860702U;
            p252.name_SET("DEMO", PH);
            p252.value = (int) -1070678476;
            LoopBackDemoChannel.instance.send(p252); //===============================
            STATUSTEXT p253 = LoopBackDemoChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.severity = (MAV_SEVERITY)MAV_SEVERITY.MAV_SEVERITY_WARNING;
            p253.text_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p253); //===============================
            DEBUG p254 = LoopBackDemoChannel.new_DEBUG();
            PH.setPack(p254);
            p254.time_boot_ms = (uint)2591594301U;
            p254.ind = (byte)(byte)34;
            p254.value = (float)2.0830444E38F;
            LoopBackDemoChannel.instance.send(p254); //===============================
            SETUP_SIGNING p256 = LoopBackDemoChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.target_system = (byte)(byte)79;
            p256.target_component = (byte)(byte)242;
            p256.secret_key_SET(new byte[32], 0);
            p256.initial_timestamp = (ulong)7237417504080216753L;
            LoopBackDemoChannel.instance.send(p256); //===============================
            BUTTON_CHANGE p257 = LoopBackDemoChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.time_boot_ms = (uint)4254695626U;
            p257.last_change_ms = (uint)425781310U;
            p257.state = (byte)(byte)240;
            LoopBackDemoChannel.instance.send(p257); //===============================
            PLAY_TUNE p258 = LoopBackDemoChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_system = (byte)(byte)72;
            p258.target_component = (byte)(byte)71;
            p258.tune_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p258); //===============================
            CAMERA_INFORMATION p259 = LoopBackDemoChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.time_boot_ms = (uint)1204986672U;
            p259.vendor_name_SET(new byte[32], 0);
            p259.model_name_SET(new byte[32], 0);
            p259.firmware_version = (uint)4164122657U;
            p259.focal_length = (float) -1.7446973E38F;
            p259.sensor_size_h = (float) -3.350231E38F;
            p259.sensor_size_v = (float) -1.3356187E38F;
            p259.resolution_h = (ushort)(ushort)46675;
            p259.resolution_v = (ushort)(ushort)17071;
            p259.lens_id = (byte)(byte)244;
            p259.flags = (CAMERA_CAP_FLAGS)CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE;
            p259.cam_definition_version = (ushort)(ushort)29669;
            p259.cam_definition_uri_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p259); //===============================
            CAMERA_SETTINGS p260 = LoopBackDemoChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.time_boot_ms = (uint)264376000U;
            p260.mode_id = (CAMERA_MODE)CAMERA_MODE.CAMERA_MODE_IMAGE;
            LoopBackDemoChannel.instance.send(p260); //===============================
            STORAGE_INFORMATION p261 = LoopBackDemoChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.time_boot_ms = (uint)1385430647U;
            p261.storage_id = (byte)(byte)207;
            p261.storage_count = (byte)(byte)236;
            p261.status = (byte)(byte)91;
            p261.total_capacity = (float) -2.3260103E37F;
            p261.used_capacity = (float) -9.758323E37F;
            p261.available_capacity = (float) -3.358903E38F;
            p261.read_speed = (float)1.6237585E37F;
            p261.write_speed = (float) -1.7354289E38F;
            LoopBackDemoChannel.instance.send(p261); //===============================
            CAMERA_CAPTURE_STATUS p262 = LoopBackDemoChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.time_boot_ms = (uint)4057456317U;
            p262.image_status = (byte)(byte)218;
            p262.video_status = (byte)(byte)180;
            p262.image_interval = (float) -3.2982453E38F;
            p262.recording_time_ms = (uint)2852967994U;
            p262.available_capacity = (float)6.364763E36F;
            LoopBackDemoChannel.instance.send(p262); //===============================
            CAMERA_IMAGE_CAPTURED p263 = LoopBackDemoChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.time_boot_ms = (uint)3039067648U;
            p263.time_utc = (ulong)1029203039762870155L;
            p263.camera_id = (byte)(byte)29;
            p263.lat = (int)301625146;
            p263.lon = (int)2117963860;
            p263.alt = (int)1874504818;
            p263.relative_alt = (int) -181698056;
            p263.q_SET(new float[4], 0);
            p263.image_index = (int) -772957618;
            p263.capture_result = (sbyte)(sbyte) - 36;
            p263.file_url_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p263); //===============================
            FLIGHT_INFORMATION p264 = LoopBackDemoChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.time_boot_ms = (uint)1900399711U;
            p264.arming_time_utc = (ulong)5996184999990877299L;
            p264.takeoff_time_utc = (ulong)6655923142856865761L;
            p264.flight_uuid = (ulong)8758689867467463286L;
            LoopBackDemoChannel.instance.send(p264); //===============================
            MOUNT_ORIENTATION p265 = LoopBackDemoChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.time_boot_ms = (uint)3816278518U;
            p265.roll = (float) -3.3514384E38F;
            p265.pitch = (float) -1.9947287E38F;
            p265.yaw = (float)2.9908697E38F;
            LoopBackDemoChannel.instance.send(p265); //===============================
            LOGGING_DATA p266 = LoopBackDemoChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.target_system = (byte)(byte)169;
            p266.target_component = (byte)(byte)163;
            p266.sequence = (ushort)(ushort)6707;
            p266.length = (byte)(byte)145;
            p266.first_message_offset = (byte)(byte)131;
            p266.data__SET(new byte[249], 0);
            LoopBackDemoChannel.instance.send(p266); //===============================
            LOGGING_DATA_ACKED p267 = LoopBackDemoChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.target_system = (byte)(byte)245;
            p267.target_component = (byte)(byte)53;
            p267.sequence = (ushort)(ushort)24490;
            p267.length = (byte)(byte)128;
            p267.first_message_offset = (byte)(byte)90;
            p267.data__SET(new byte[249], 0);
            LoopBackDemoChannel.instance.send(p267); //===============================
            LOGGING_ACK p268 = LoopBackDemoChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_system = (byte)(byte)142;
            p268.target_component = (byte)(byte)231;
            p268.sequence = (ushort)(ushort)50851;
            LoopBackDemoChannel.instance.send(p268); //===============================
            VIDEO_STREAM_INFORMATION p269 = LoopBackDemoChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.camera_id = (byte)(byte)50;
            p269.status = (byte)(byte)17;
            p269.framerate = (float)3.0224378E38F;
            p269.resolution_h = (ushort)(ushort)61236;
            p269.resolution_v = (ushort)(ushort)8334;
            p269.bitrate = (uint)717879113U;
            p269.rotation = (ushort)(ushort)45098;
            p269.uri_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p269); //===============================
            SET_VIDEO_STREAM_SETTINGS p270 = LoopBackDemoChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.target_system = (byte)(byte)20;
            p270.target_component = (byte)(byte)246;
            p270.camera_id = (byte)(byte)228;
            p270.framerate = (float)1.461772E38F;
            p270.resolution_h = (ushort)(ushort)27361;
            p270.resolution_v = (ushort)(ushort)62662;
            p270.bitrate = (uint)2796624022U;
            p270.rotation = (ushort)(ushort)12283;
            p270.uri_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p270); //===============================
            WIFI_CONFIG_AP p299 = LoopBackDemoChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.ssid_SET("DEMO", PH);
            p299.password_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p299); //===============================
            PROTOCOL_VERSION p300 = LoopBackDemoChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.version = (ushort)(ushort)31077;
            p300.min_version = (ushort)(ushort)15145;
            p300.max_version = (ushort)(ushort)61998;
            p300.spec_version_hash_SET(new byte[8], 0);
            p300.library_version_hash_SET(new byte[8], 0);
            LoopBackDemoChannel.instance.send(p300); //===============================
            UAVCAN_NODE_STATUS p310 = LoopBackDemoChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.time_usec = (ulong)9051784213516709344L;
            p310.uptime_sec = (uint)1552845707U;
            p310.health = (UAVCAN_NODE_HEALTH)UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING;
            p310.mode = (UAVCAN_NODE_MODE)UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION;
            p310.sub_mode = (byte)(byte)82;
            p310.vendor_specific_status_code = (ushort)(ushort)26565;
            LoopBackDemoChannel.instance.send(p310); //===============================
            UAVCAN_NODE_INFO p311 = LoopBackDemoChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.time_usec = (ulong)6981990687477843493L;
            p311.uptime_sec = (uint)365919406U;
            p311.name_SET("DEMO", PH);
            p311.hw_version_major = (byte)(byte)254;
            p311.hw_version_minor = (byte)(byte)253;
            p311.hw_unique_id_SET(new byte[16], 0);
            p311.sw_version_major = (byte)(byte)237;
            p311.sw_version_minor = (byte)(byte)167;
            p311.sw_vcs_commit = (uint)2735922117U;
            LoopBackDemoChannel.instance.send(p311); //===============================
            PARAM_EXT_REQUEST_READ p320 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.target_system = (byte)(byte)253;
            p320.target_component = (byte)(byte)189;
            p320.param_id_SET("DEMO", PH);
            p320.param_index = (short)(short) -686;
            LoopBackDemoChannel.instance.send(p320); //===============================
            PARAM_EXT_REQUEST_LIST p321 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_system = (byte)(byte)11;
            p321.target_component = (byte)(byte)107;
            LoopBackDemoChannel.instance.send(p321); //===============================
            PARAM_EXT_VALUE p322 = LoopBackDemoChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_id_SET("DEMO", PH);
            p322.param_value_SET("DEMO", PH);
            p322.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32;
            p322.param_count = (ushort)(ushort)21347;
            p322.param_index = (ushort)(ushort)23696;
            LoopBackDemoChannel.instance.send(p322); //===============================
            PARAM_EXT_SET p323 = LoopBackDemoChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.target_system = (byte)(byte)154;
            p323.target_component = (byte)(byte)156;
            p323.param_id_SET("DEMO", PH);
            p323.param_value_SET("DEMO", PH);
            p323.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT64;
            LoopBackDemoChannel.instance.send(p323); //===============================
            PARAM_EXT_ACK p324 = LoopBackDemoChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_id_SET("DEMO", PH);
            p324.param_value_SET("DEMO", PH);
            p324.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32;
            p324.param_result = (PARAM_ACK)PARAM_ACK.PARAM_ACK_FAILED;
            LoopBackDemoChannel.instance.send(p324); //===============================
            OBSTACLE_DISTANCE p330 = LoopBackDemoChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.time_usec = (ulong)8569682937644194810L;
            p330.sensor_type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED;
            p330.distances_SET(new ushort[72], 0);
            p330.increment = (byte)(byte)203;
            p330.min_distance = (ushort)(ushort)46975;
            p330.max_distance = (ushort)(ushort)20905;
            LoopBackDemoChannel.instance.send(p330); //===============================
        }
    }
}

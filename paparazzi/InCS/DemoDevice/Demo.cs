
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
            LoopBackDemoChannel.instance.OnSCRIPT_ITEMReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                ushort seq = pack.seq;
                string name = pack.name_TRY(ph);
            };
            LoopBackDemoChannel.instance.OnSCRIPT_REQUESTReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                ushort seq = pack.seq;
            };
            LoopBackDemoChannel.instance.OnSCRIPT_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
            };
            LoopBackDemoChannel.instance.OnSCRIPT_COUNTReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                ushort count = pack.count;
            };
            LoopBackDemoChannel.instance.OnSCRIPT_CURRENTReceive += (src, ph, pack) =>
            {
                ushort seq = pack.seq;
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
            p0.type = (MAV_TYPE)MAV_TYPE.MAV_TYPE_TRICOPTER;
            p0.autopilot = (MAV_AUTOPILOT)MAV_AUTOPILOT.MAV_AUTOPILOT_ARMAZILA;
            p0.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED;
            p0.custom_mode = (uint)3194768282U;
            p0.system_status = (MAV_STATE)MAV_STATE.MAV_STATE_FLIGHT_TERMINATION;
            p0.mavlink_version = (byte)(byte)39;
            LoopBackDemoChannel.instance.send(p0); //===============================
            SYS_STATUS p1 = LoopBackDemoChannel.new_SYS_STATUS();
            PH.setPack(p1);
            p1.onboard_control_sensors_present = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL;
            p1.onboard_control_sensors_enabled = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION;
            p1.onboard_control_sensors_health = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL;
            p1.load = (ushort)(ushort)50461;
            p1.voltage_battery = (ushort)(ushort)53607;
            p1.current_battery = (short)(short)10488;
            p1.battery_remaining = (sbyte)(sbyte)48;
            p1.drop_rate_comm = (ushort)(ushort)44325;
            p1.errors_comm = (ushort)(ushort)38053;
            p1.errors_count1 = (ushort)(ushort)63734;
            p1.errors_count2 = (ushort)(ushort)14247;
            p1.errors_count3 = (ushort)(ushort)29885;
            p1.errors_count4 = (ushort)(ushort)46697;
            LoopBackDemoChannel.instance.send(p1); //===============================
            SYSTEM_TIME p2 = LoopBackDemoChannel.new_SYSTEM_TIME();
            PH.setPack(p2);
            p2.time_unix_usec = (ulong)4543779418494487242L;
            p2.time_boot_ms = (uint)1616931955U;
            LoopBackDemoChannel.instance.send(p2); //===============================
            POSITION_TARGET_LOCAL_NED p3 = LoopBackDemoChannel.new_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.time_boot_ms = (uint)964636103U;
            p3.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_ENU;
            p3.type_mask = (ushort)(ushort)59466;
            p3.x = (float) -2.1413544E36F;
            p3.y = (float) -2.9353902E38F;
            p3.z = (float) -1.5344922E38F;
            p3.vx = (float)1.804371E38F;
            p3.vy = (float)2.7579838E38F;
            p3.vz = (float)3.1523542E38F;
            p3.afx = (float) -1.8133148E38F;
            p3.afy = (float)4.9459087E37F;
            p3.afz = (float) -1.6027345E38F;
            p3.yaw = (float) -3.3484299E38F;
            p3.yaw_rate = (float) -1.7128318E38F;
            LoopBackDemoChannel.instance.send(p3); //===============================
            PING p4 = LoopBackDemoChannel.new_PING();
            PH.setPack(p4);
            p4.time_usec = (ulong)1097905806896221118L;
            p4.seq = (uint)1473847790U;
            p4.target_system = (byte)(byte)227;
            p4.target_component = (byte)(byte)56;
            LoopBackDemoChannel.instance.send(p4); //===============================
            CHANGE_OPERATOR_CONTROL p5 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL();
            PH.setPack(p5);
            p5.target_system = (byte)(byte)164;
            p5.control_request = (byte)(byte)167;
            p5.version = (byte)(byte)75;
            p5.passkey_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p5); //===============================
            CHANGE_OPERATOR_CONTROL_ACK p6 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL_ACK();
            PH.setPack(p6);
            p6.gcs_system_id = (byte)(byte)185;
            p6.control_request = (byte)(byte)38;
            p6.ack = (byte)(byte)145;
            LoopBackDemoChannel.instance.send(p6); //===============================
            AUTH_KEY p7 = LoopBackDemoChannel.new_AUTH_KEY();
            PH.setPack(p7);
            p7.key_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p7); //===============================
            SET_MODE p11 = LoopBackDemoChannel.new_SET_MODE();
            PH.setPack(p11);
            p11.target_system = (byte)(byte)235;
            p11.base_mode = (MAV_MODE)MAV_MODE.MAV_MODE_GUIDED_DISARMED;
            p11.custom_mode = (uint)1733689380U;
            LoopBackDemoChannel.instance.send(p11); //===============================
            PARAM_REQUEST_READ p20 = LoopBackDemoChannel.new_PARAM_REQUEST_READ();
            PH.setPack(p20);
            p20.target_system = (byte)(byte)218;
            p20.target_component = (byte)(byte)97;
            p20.param_id_SET("DEMO", PH);
            p20.param_index = (short)(short)23169;
            LoopBackDemoChannel.instance.send(p20); //===============================
            PARAM_REQUEST_LIST p21 = LoopBackDemoChannel.new_PARAM_REQUEST_LIST();
            PH.setPack(p21);
            p21.target_system = (byte)(byte)79;
            p21.target_component = (byte)(byte)27;
            LoopBackDemoChannel.instance.send(p21); //===============================
            PARAM_VALUE p22 = LoopBackDemoChannel.new_PARAM_VALUE();
            PH.setPack(p22);
            p22.param_id_SET("DEMO", PH);
            p22.param_value = (float) -2.1482573E38F;
            p22.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT64;
            p22.param_count = (ushort)(ushort)18642;
            p22.param_index = (ushort)(ushort)14002;
            LoopBackDemoChannel.instance.send(p22); //===============================
            PARAM_SET p23 = LoopBackDemoChannel.new_PARAM_SET();
            PH.setPack(p23);
            p23.target_system = (byte)(byte)209;
            p23.target_component = (byte)(byte)179;
            p23.param_id_SET("DEMO", PH);
            p23.param_value = (float) -3.0279437E38F;
            p23.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT16;
            LoopBackDemoChannel.instance.send(p23); //===============================
            GPS_RAW_INT p24 = LoopBackDemoChannel.new_GPS_RAW_INT();
            PH.setPack(p24);
            p24.time_usec = (ulong)6503561595705344632L;
            p24.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS;
            p24.lat = (int) -1408619281;
            p24.lon = (int) -1483379696;
            p24.alt = (int) -435346930;
            p24.eph = (ushort)(ushort)51804;
            p24.epv = (ushort)(ushort)49480;
            p24.vel = (ushort)(ushort)61309;
            p24.cog = (ushort)(ushort)1248;
            p24.satellites_visible = (byte)(byte)137;
            p24.alt_ellipsoid_SET((int) -583558285, PH);
            p24.h_acc_SET((uint)116981722U, PH);
            p24.v_acc_SET((uint)70567314U, PH);
            p24.vel_acc_SET((uint)1265622953U, PH);
            p24.hdg_acc_SET((uint)837582659U, PH);
            LoopBackDemoChannel.instance.send(p24); //===============================
            GPS_STATUS p25 = LoopBackDemoChannel.new_GPS_STATUS();
            PH.setPack(p25);
            p25.satellites_visible = (byte)(byte)222;
            p25.satellite_prn_SET(new byte[20], 0);
            p25.satellite_used_SET(new byte[20], 0);
            p25.satellite_elevation_SET(new byte[20], 0);
            p25.satellite_azimuth_SET(new byte[20], 0);
            p25.satellite_snr_SET(new byte[20], 0);
            LoopBackDemoChannel.instance.send(p25); //===============================
            SCALED_IMU p26 = LoopBackDemoChannel.new_SCALED_IMU();
            PH.setPack(p26);
            p26.time_boot_ms = (uint)1434391208U;
            p26.xacc = (short)(short) -28302;
            p26.yacc = (short)(short)19174;
            p26.zacc = (short)(short) -18316;
            p26.xgyro = (short)(short) -32052;
            p26.ygyro = (short)(short) -29353;
            p26.zgyro = (short)(short) -4893;
            p26.xmag = (short)(short)19929;
            p26.ymag = (short)(short)12737;
            p26.zmag = (short)(short)18258;
            LoopBackDemoChannel.instance.send(p26); //===============================
            RAW_IMU p27 = LoopBackDemoChannel.new_RAW_IMU();
            PH.setPack(p27);
            p27.time_usec = (ulong)7785026466670517413L;
            p27.xacc = (short)(short) -32706;
            p27.yacc = (short)(short) -19710;
            p27.zacc = (short)(short)28494;
            p27.xgyro = (short)(short) -21651;
            p27.ygyro = (short)(short)13028;
            p27.zgyro = (short)(short) -2814;
            p27.xmag = (short)(short)21699;
            p27.ymag = (short)(short)10890;
            p27.zmag = (short)(short) -6102;
            LoopBackDemoChannel.instance.send(p27); //===============================
            RAW_PRESSURE p28 = LoopBackDemoChannel.new_RAW_PRESSURE();
            PH.setPack(p28);
            p28.time_usec = (ulong)1000958187917882850L;
            p28.press_abs = (short)(short) -10193;
            p28.press_diff1 = (short)(short)18262;
            p28.press_diff2 = (short)(short) -12655;
            p28.temperature = (short)(short) -29216;
            LoopBackDemoChannel.instance.send(p28); //===============================
            SCALED_PRESSURE p29 = LoopBackDemoChannel.new_SCALED_PRESSURE();
            PH.setPack(p29);
            p29.time_boot_ms = (uint)1420320190U;
            p29.press_abs = (float) -3.3187476E38F;
            p29.press_diff = (float) -2.8045228E38F;
            p29.temperature = (short)(short)7126;
            LoopBackDemoChannel.instance.send(p29); //===============================
            ATTITUDE p30 = LoopBackDemoChannel.new_ATTITUDE();
            PH.setPack(p30);
            p30.time_boot_ms = (uint)2585629988U;
            p30.roll = (float)2.9068936E38F;
            p30.pitch = (float)1.0727404E38F;
            p30.yaw = (float)1.4543208E38F;
            p30.rollspeed = (float) -1.975478E38F;
            p30.pitchspeed = (float) -2.7368771E38F;
            p30.yawspeed = (float) -2.5786853E38F;
            LoopBackDemoChannel.instance.send(p30); //===============================
            ATTITUDE_QUATERNION p31 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION();
            PH.setPack(p31);
            p31.time_boot_ms = (uint)1034488514U;
            p31.q1 = (float)2.0961473E38F;
            p31.q2 = (float)5.2596405E37F;
            p31.q3 = (float) -6.1895804E37F;
            p31.q4 = (float)1.958742E38F;
            p31.rollspeed = (float)1.9788782E38F;
            p31.pitchspeed = (float) -2.6982458E38F;
            p31.yawspeed = (float)1.1349924E38F;
            LoopBackDemoChannel.instance.send(p31); //===============================
            LOCAL_POSITION_NED p32 = LoopBackDemoChannel.new_LOCAL_POSITION_NED();
            PH.setPack(p32);
            p32.time_boot_ms = (uint)72319420U;
            p32.x = (float) -2.7374639E38F;
            p32.y = (float) -3.0680844E38F;
            p32.z = (float)2.977842E37F;
            p32.vx = (float)2.2888411E38F;
            p32.vy = (float) -1.316841E38F;
            p32.vz = (float)1.9356211E38F;
            LoopBackDemoChannel.instance.send(p32); //===============================
            GLOBAL_POSITION_INT p33 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT();
            PH.setPack(p33);
            p33.time_boot_ms = (uint)3826690933U;
            p33.lat = (int)1309926310;
            p33.lon = (int) -820649042;
            p33.alt = (int) -1608537571;
            p33.relative_alt = (int) -130824105;
            p33.vx = (short)(short)11140;
            p33.vy = (short)(short) -24333;
            p33.vz = (short)(short) -4839;
            p33.hdg = (ushort)(ushort)3555;
            LoopBackDemoChannel.instance.send(p33); //===============================
            RC_CHANNELS_SCALED p34 = LoopBackDemoChannel.new_RC_CHANNELS_SCALED();
            PH.setPack(p34);
            p34.time_boot_ms = (uint)265550644U;
            p34.port = (byte)(byte)185;
            p34.chan1_scaled = (short)(short)3958;
            p34.chan2_scaled = (short)(short)8586;
            p34.chan3_scaled = (short)(short) -20315;
            p34.chan4_scaled = (short)(short)5301;
            p34.chan5_scaled = (short)(short) -22302;
            p34.chan6_scaled = (short)(short) -25082;
            p34.chan7_scaled = (short)(short)11981;
            p34.chan8_scaled = (short)(short) -21523;
            p34.rssi = (byte)(byte)136;
            LoopBackDemoChannel.instance.send(p34); //===============================
            RC_CHANNELS_RAW p35 = LoopBackDemoChannel.new_RC_CHANNELS_RAW();
            PH.setPack(p35);
            p35.time_boot_ms = (uint)2142577598U;
            p35.port = (byte)(byte)224;
            p35.chan1_raw = (ushort)(ushort)23876;
            p35.chan2_raw = (ushort)(ushort)59146;
            p35.chan3_raw = (ushort)(ushort)27703;
            p35.chan4_raw = (ushort)(ushort)18193;
            p35.chan5_raw = (ushort)(ushort)36619;
            p35.chan6_raw = (ushort)(ushort)8385;
            p35.chan7_raw = (ushort)(ushort)25836;
            p35.chan8_raw = (ushort)(ushort)62625;
            p35.rssi = (byte)(byte)94;
            LoopBackDemoChannel.instance.send(p35); //===============================
            SERVO_OUTPUT_RAW p36 = LoopBackDemoChannel.new_SERVO_OUTPUT_RAW();
            PH.setPack(p36);
            p36.time_usec = (uint)2259915837U;
            p36.port = (byte)(byte)74;
            p36.servo1_raw = (ushort)(ushort)46614;
            p36.servo2_raw = (ushort)(ushort)45403;
            p36.servo3_raw = (ushort)(ushort)38073;
            p36.servo4_raw = (ushort)(ushort)57577;
            p36.servo5_raw = (ushort)(ushort)19297;
            p36.servo6_raw = (ushort)(ushort)23378;
            p36.servo7_raw = (ushort)(ushort)41891;
            p36.servo8_raw = (ushort)(ushort)39568;
            p36.servo9_raw_SET((ushort)(ushort)53151, PH);
            p36.servo10_raw_SET((ushort)(ushort)22540, PH);
            p36.servo11_raw_SET((ushort)(ushort)31248, PH);
            p36.servo12_raw_SET((ushort)(ushort)16785, PH);
            p36.servo13_raw_SET((ushort)(ushort)49605, PH);
            p36.servo14_raw_SET((ushort)(ushort)25935, PH);
            p36.servo15_raw_SET((ushort)(ushort)6332, PH);
            p36.servo16_raw_SET((ushort)(ushort)589, PH);
            LoopBackDemoChannel.instance.send(p36); //===============================
            MISSION_REQUEST_PARTIAL_LIST p37 = LoopBackDemoChannel.new_MISSION_REQUEST_PARTIAL_LIST();
            PH.setPack(p37);
            p37.target_system = (byte)(byte)113;
            p37.target_component = (byte)(byte)179;
            p37.start_index = (short)(short)28075;
            p37.end_index = (short)(short) -4844;
            p37.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            LoopBackDemoChannel.instance.send(p37); //===============================
            MISSION_WRITE_PARTIAL_LIST p38 = LoopBackDemoChannel.new_MISSION_WRITE_PARTIAL_LIST();
            PH.setPack(p38);
            p38.target_system = (byte)(byte)52;
            p38.target_component = (byte)(byte)56;
            p38.start_index = (short)(short)11515;
            p38.end_index = (short)(short) -23881;
            p38.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            LoopBackDemoChannel.instance.send(p38); //===============================
            MISSION_ITEM p39 = LoopBackDemoChannel.new_MISSION_ITEM();
            PH.setPack(p39);
            p39.target_system = (byte)(byte)171;
            p39.target_component = (byte)(byte)227;
            p39.seq = (ushort)(ushort)64635;
            p39.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_NED;
            p39.command = (MAV_CMD)MAV_CMD.MAV_CMD_LOGGING_STOP;
            p39.current = (byte)(byte)233;
            p39.autocontinue = (byte)(byte)77;
            p39.param1 = (float) -1.5892202E38F;
            p39.param2 = (float) -4.549976E37F;
            p39.param3 = (float) -3.1733487E38F;
            p39.param4 = (float) -2.4304688E37F;
            p39.x = (float) -8.864408E37F;
            p39.y = (float) -1.5602177E38F;
            p39.z = (float)2.492415E38F;
            p39.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            LoopBackDemoChannel.instance.send(p39); //===============================
            MISSION_REQUEST p40 = LoopBackDemoChannel.new_MISSION_REQUEST();
            PH.setPack(p40);
            p40.target_system = (byte)(byte)57;
            p40.target_component = (byte)(byte)245;
            p40.seq = (ushort)(ushort)7074;
            p40.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            LoopBackDemoChannel.instance.send(p40); //===============================
            MISSION_SET_CURRENT p41 = LoopBackDemoChannel.new_MISSION_SET_CURRENT();
            PH.setPack(p41);
            p41.target_system = (byte)(byte)67;
            p41.target_component = (byte)(byte)114;
            p41.seq = (ushort)(ushort)63886;
            LoopBackDemoChannel.instance.send(p41); //===============================
            MISSION_CURRENT p42 = LoopBackDemoChannel.new_MISSION_CURRENT();
            PH.setPack(p42);
            p42.seq = (ushort)(ushort)41955;
            LoopBackDemoChannel.instance.send(p42); //===============================
            MISSION_REQUEST_LIST p43 = LoopBackDemoChannel.new_MISSION_REQUEST_LIST();
            PH.setPack(p43);
            p43.target_system = (byte)(byte)167;
            p43.target_component = (byte)(byte)16;
            p43.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            LoopBackDemoChannel.instance.send(p43); //===============================
            MISSION_COUNT p44 = LoopBackDemoChannel.new_MISSION_COUNT();
            PH.setPack(p44);
            p44.target_system = (byte)(byte)16;
            p44.target_component = (byte)(byte)244;
            p44.count = (ushort)(ushort)23170;
            p44.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            LoopBackDemoChannel.instance.send(p44); //===============================
            MISSION_CLEAR_ALL p45 = LoopBackDemoChannel.new_MISSION_CLEAR_ALL();
            PH.setPack(p45);
            p45.target_system = (byte)(byte)77;
            p45.target_component = (byte)(byte)61;
            p45.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            LoopBackDemoChannel.instance.send(p45); //===============================
            MISSION_ITEM_REACHED p46 = LoopBackDemoChannel.new_MISSION_ITEM_REACHED();
            PH.setPack(p46);
            p46.seq = (ushort)(ushort)58274;
            LoopBackDemoChannel.instance.send(p46); //===============================
            MISSION_ACK p47 = LoopBackDemoChannel.new_MISSION_ACK();
            PH.setPack(p47);
            p47.target_system = (byte)(byte)102;
            p47.target_component = (byte)(byte)21;
            p47.type = (MAV_MISSION_RESULT)MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM2;
            p47.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            LoopBackDemoChannel.instance.send(p47); //===============================
            SET_GPS_GLOBAL_ORIGIN p48 = LoopBackDemoChannel.new_SET_GPS_GLOBAL_ORIGIN();
            PH.setPack(p48);
            p48.target_system = (byte)(byte)177;
            p48.latitude = (int) -132834568;
            p48.longitude = (int)2120481914;
            p48.altitude = (int)653394564;
            p48.time_usec_SET((ulong)3534051987186849833L, PH);
            LoopBackDemoChannel.instance.send(p48); //===============================
            GPS_GLOBAL_ORIGIN p49 = LoopBackDemoChannel.new_GPS_GLOBAL_ORIGIN();
            PH.setPack(p49);
            p49.latitude = (int)1131347804;
            p49.longitude = (int)1452635611;
            p49.altitude = (int) -1267186105;
            p49.time_usec_SET((ulong)3135368139826631429L, PH);
            LoopBackDemoChannel.instance.send(p49); //===============================
            PARAM_MAP_RC p50 = LoopBackDemoChannel.new_PARAM_MAP_RC();
            PH.setPack(p50);
            p50.target_system = (byte)(byte)76;
            p50.target_component = (byte)(byte)227;
            p50.param_id_SET("DEMO", PH);
            p50.param_index = (short)(short)9524;
            p50.parameter_rc_channel_index = (byte)(byte)146;
            p50.param_value0 = (float) -8.875523E36F;
            p50.scale = (float)2.7671675E38F;
            p50.param_value_min = (float)2.31288E38F;
            p50.param_value_max = (float)2.9050542E38F;
            LoopBackDemoChannel.instance.send(p50); //===============================
            MISSION_REQUEST_INT p51 = LoopBackDemoChannel.new_MISSION_REQUEST_INT();
            PH.setPack(p51);
            p51.target_system = (byte)(byte)138;
            p51.target_component = (byte)(byte)251;
            p51.seq = (ushort)(ushort)11838;
            p51.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            LoopBackDemoChannel.instance.send(p51); //===============================
            SAFETY_SET_ALLOWED_AREA p54 = LoopBackDemoChannel.new_SAFETY_SET_ALLOWED_AREA();
            PH.setPack(p54);
            p54.target_system = (byte)(byte)154;
            p54.target_component = (byte)(byte)86;
            p54.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_NED;
            p54.p1x = (float) -4.679613E37F;
            p54.p1y = (float)2.3068472E37F;
            p54.p1z = (float)3.6767448E37F;
            p54.p2x = (float)1.3513842E38F;
            p54.p2y = (float)2.43014E37F;
            p54.p2z = (float)1.301126E38F;
            LoopBackDemoChannel.instance.send(p54); //===============================
            SAFETY_ALLOWED_AREA p55 = LoopBackDemoChannel.new_SAFETY_ALLOWED_AREA();
            PH.setPack(p55);
            p55.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_NED;
            p55.p1x = (float) -1.2997115E38F;
            p55.p1y = (float) -8.714185E37F;
            p55.p1z = (float)5.655749E37F;
            p55.p2x = (float)1.184585E38F;
            p55.p2y = (float) -3.2602038E38F;
            p55.p2z = (float)3.8285472E37F;
            LoopBackDemoChannel.instance.send(p55); //===============================
            ATTITUDE_QUATERNION_COV p61 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION_COV();
            PH.setPack(p61);
            p61.time_usec = (ulong)511240337439255338L;
            p61.q_SET(new float[4], 0);
            p61.rollspeed = (float)4.5359936E37F;
            p61.pitchspeed = (float) -1.1000585E38F;
            p61.yawspeed = (float)3.2713647E38F;
            p61.covariance_SET(new float[9], 0);
            LoopBackDemoChannel.instance.send(p61); //===============================
            NAV_CONTROLLER_OUTPUT p62 = LoopBackDemoChannel.new_NAV_CONTROLLER_OUTPUT();
            PH.setPack(p62);
            p62.nav_roll = (float)1.189831E38F;
            p62.nav_pitch = (float) -3.3346253E38F;
            p62.nav_bearing = (short)(short) -16748;
            p62.target_bearing = (short)(short) -10607;
            p62.wp_dist = (ushort)(ushort)50583;
            p62.alt_error = (float)2.248023E38F;
            p62.aspd_error = (float) -2.4621722E38F;
            p62.xtrack_error = (float) -9.0068996E36F;
            LoopBackDemoChannel.instance.send(p62); //===============================
            GLOBAL_POSITION_INT_COV p63 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT_COV();
            PH.setPack(p63);
            p63.time_usec = (ulong)6938992803327513895L;
            p63.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE;
            p63.lat = (int) -1381537845;
            p63.lon = (int)1661149138;
            p63.alt = (int)1187872396;
            p63.relative_alt = (int) -480425901;
            p63.vx = (float) -7.5929604E37F;
            p63.vy = (float)1.8720613E38F;
            p63.vz = (float) -1.1057086E38F;
            p63.covariance_SET(new float[36], 0);
            LoopBackDemoChannel.instance.send(p63); //===============================
            LOCAL_POSITION_NED_COV p64 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_COV();
            PH.setPack(p64);
            p64.time_usec = (ulong)1673109094511997860L;
            p64.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION;
            p64.x = (float) -2.6823752E38F;
            p64.y = (float) -1.6951734E38F;
            p64.z = (float) -3.0329812E38F;
            p64.vx = (float)3.2070546E38F;
            p64.vy = (float)2.5958775E38F;
            p64.vz = (float)1.0908488E38F;
            p64.ax = (float)2.3876722E38F;
            p64.ay = (float) -3.327417E38F;
            p64.az = (float)1.3645184E37F;
            p64.covariance_SET(new float[45], 0);
            LoopBackDemoChannel.instance.send(p64); //===============================
            RC_CHANNELS p65 = LoopBackDemoChannel.new_RC_CHANNELS();
            PH.setPack(p65);
            p65.time_boot_ms = (uint)883539196U;
            p65.chancount = (byte)(byte)77;
            p65.chan1_raw = (ushort)(ushort)28240;
            p65.chan2_raw = (ushort)(ushort)44968;
            p65.chan3_raw = (ushort)(ushort)22495;
            p65.chan4_raw = (ushort)(ushort)37200;
            p65.chan5_raw = (ushort)(ushort)1742;
            p65.chan6_raw = (ushort)(ushort)40991;
            p65.chan7_raw = (ushort)(ushort)57765;
            p65.chan8_raw = (ushort)(ushort)50455;
            p65.chan9_raw = (ushort)(ushort)8311;
            p65.chan10_raw = (ushort)(ushort)2096;
            p65.chan11_raw = (ushort)(ushort)1063;
            p65.chan12_raw = (ushort)(ushort)49950;
            p65.chan13_raw = (ushort)(ushort)30515;
            p65.chan14_raw = (ushort)(ushort)40860;
            p65.chan15_raw = (ushort)(ushort)57492;
            p65.chan16_raw = (ushort)(ushort)63208;
            p65.chan17_raw = (ushort)(ushort)40142;
            p65.chan18_raw = (ushort)(ushort)1641;
            p65.rssi = (byte)(byte)43;
            LoopBackDemoChannel.instance.send(p65); //===============================
            REQUEST_DATA_STREAM p66 = LoopBackDemoChannel.new_REQUEST_DATA_STREAM();
            PH.setPack(p66);
            p66.target_system = (byte)(byte)220;
            p66.target_component = (byte)(byte)189;
            p66.req_stream_id = (byte)(byte)126;
            p66.req_message_rate = (ushort)(ushort)14081;
            p66.start_stop = (byte)(byte)183;
            LoopBackDemoChannel.instance.send(p66); //===============================
            DATA_STREAM p67 = LoopBackDemoChannel.new_DATA_STREAM();
            PH.setPack(p67);
            p67.stream_id = (byte)(byte)187;
            p67.message_rate = (ushort)(ushort)7955;
            p67.on_off = (byte)(byte)227;
            LoopBackDemoChannel.instance.send(p67); //===============================
            MANUAL_CONTROL p69 = LoopBackDemoChannel.new_MANUAL_CONTROL();
            PH.setPack(p69);
            p69.target = (byte)(byte)115;
            p69.x = (short)(short)9699;
            p69.y = (short)(short)10637;
            p69.z = (short)(short)23616;
            p69.r = (short)(short)13976;
            p69.buttons = (ushort)(ushort)2555;
            LoopBackDemoChannel.instance.send(p69); //===============================
            RC_CHANNELS_OVERRIDE p70 = LoopBackDemoChannel.new_RC_CHANNELS_OVERRIDE();
            PH.setPack(p70);
            p70.target_system = (byte)(byte)53;
            p70.target_component = (byte)(byte)61;
            p70.chan1_raw = (ushort)(ushort)48306;
            p70.chan2_raw = (ushort)(ushort)9852;
            p70.chan3_raw = (ushort)(ushort)4744;
            p70.chan4_raw = (ushort)(ushort)50333;
            p70.chan5_raw = (ushort)(ushort)35213;
            p70.chan6_raw = (ushort)(ushort)1577;
            p70.chan7_raw = (ushort)(ushort)54015;
            p70.chan8_raw = (ushort)(ushort)4920;
            LoopBackDemoChannel.instance.send(p70); //===============================
            MISSION_ITEM_INT p73 = LoopBackDemoChannel.new_MISSION_ITEM_INT();
            PH.setPack(p73);
            p73.target_system = (byte)(byte)108;
            p73.target_component = (byte)(byte)27;
            p73.seq = (ushort)(ushort)32991;
            p73.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT;
            p73.command = (MAV_CMD)MAV_CMD.MAV_CMD_CONDITION_DELAY;
            p73.current = (byte)(byte)230;
            p73.autocontinue = (byte)(byte)249;
            p73.param1 = (float) -4.439477E36F;
            p73.param2 = (float)1.8963986E38F;
            p73.param3 = (float) -1.4001499E38F;
            p73.param4 = (float)2.454777E38F;
            p73.x = (int) -1568001201;
            p73.y = (int) -1902452715;
            p73.z = (float)1.910335E38F;
            p73.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            LoopBackDemoChannel.instance.send(p73); //===============================
            VFR_HUD p74 = LoopBackDemoChannel.new_VFR_HUD();
            PH.setPack(p74);
            p74.airspeed = (float) -2.6475124E38F;
            p74.groundspeed = (float)1.4583273E38F;
            p74.heading = (short)(short) -6770;
            p74.throttle = (ushort)(ushort)4658;
            p74.alt = (float) -1.9476026E38F;
            p74.climb = (float) -1.4812034E38F;
            LoopBackDemoChannel.instance.send(p74); //===============================
            COMMAND_INT p75 = LoopBackDemoChannel.new_COMMAND_INT();
            PH.setPack(p75);
            p75.target_system = (byte)(byte)192;
            p75.target_component = (byte)(byte)184;
            p75.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_NED;
            p75.command = (MAV_CMD)MAV_CMD.MAV_CMD_SPATIAL_USER_4;
            p75.current = (byte)(byte)42;
            p75.autocontinue = (byte)(byte)237;
            p75.param1 = (float) -1.6917398E38F;
            p75.param2 = (float) -1.4143372E38F;
            p75.param3 = (float)2.4367849E38F;
            p75.param4 = (float)8.3922374E37F;
            p75.x = (int) -1995084451;
            p75.y = (int) -305949237;
            p75.z = (float)1.711078E38F;
            LoopBackDemoChannel.instance.send(p75); //===============================
            COMMAND_LONG p76 = LoopBackDemoChannel.new_COMMAND_LONG();
            PH.setPack(p76);
            p76.target_system = (byte)(byte)218;
            p76.target_component = (byte)(byte)201;
            p76.command = (MAV_CMD)MAV_CMD.MAV_CMD_DO_DIGICAM_CONFIGURE;
            p76.confirmation = (byte)(byte)0;
            p76.param1 = (float)3.3554632E38F;
            p76.param2 = (float)1.598799E37F;
            p76.param3 = (float)2.232751E38F;
            p76.param4 = (float) -4.382276E35F;
            p76.param5 = (float) -2.0822853E38F;
            p76.param6 = (float) -8.3530163E37F;
            p76.param7 = (float) -3.1237838E38F;
            LoopBackDemoChannel.instance.send(p76); //===============================
            COMMAND_ACK p77 = LoopBackDemoChannel.new_COMMAND_ACK();
            PH.setPack(p77);
            p77.command = (MAV_CMD)MAV_CMD.MAV_CMD_VIDEO_START_STREAMING;
            p77.result = (MAV_RESULT)MAV_RESULT.MAV_RESULT_FAILED;
            p77.progress_SET((byte)(byte)120, PH);
            p77.result_param2_SET((int) -241399780, PH);
            p77.target_system_SET((byte)(byte)115, PH);
            p77.target_component_SET((byte)(byte)234, PH);
            LoopBackDemoChannel.instance.send(p77); //===============================
            MANUAL_SETPOINT p81 = LoopBackDemoChannel.new_MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.time_boot_ms = (uint)2786884819U;
            p81.roll = (float) -2.0561015E38F;
            p81.pitch = (float)2.9828624E38F;
            p81.yaw = (float) -3.3822488E38F;
            p81.thrust = (float) -2.5046679E38F;
            p81.mode_switch = (byte)(byte)248;
            p81.manual_override_switch = (byte)(byte)136;
            LoopBackDemoChannel.instance.send(p81); //===============================
            SET_ATTITUDE_TARGET p82 = LoopBackDemoChannel.new_SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.time_boot_ms = (uint)306469202U;
            p82.target_system = (byte)(byte)192;
            p82.target_component = (byte)(byte)22;
            p82.type_mask = (byte)(byte)10;
            p82.q_SET(new float[4], 0);
            p82.body_roll_rate = (float)5.0041724E37F;
            p82.body_pitch_rate = (float)1.1484581E38F;
            p82.body_yaw_rate = (float)1.8191622E38F;
            p82.thrust = (float)2.2083928E38F;
            LoopBackDemoChannel.instance.send(p82); //===============================
            ATTITUDE_TARGET p83 = LoopBackDemoChannel.new_ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.time_boot_ms = (uint)3027755591U;
            p83.type_mask = (byte)(byte)137;
            p83.q_SET(new float[4], 0);
            p83.body_roll_rate = (float)3.114124E38F;
            p83.body_pitch_rate = (float)4.5209085E37F;
            p83.body_yaw_rate = (float)6.5369283E37F;
            p83.thrust = (float) -2.8135136E38F;
            LoopBackDemoChannel.instance.send(p83); //===============================
            SET_POSITION_TARGET_LOCAL_NED p84 = LoopBackDemoChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.time_boot_ms = (uint)4108076909U;
            p84.target_system = (byte)(byte)168;
            p84.target_component = (byte)(byte)44;
            p84.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_ENU;
            p84.type_mask = (ushort)(ushort)15634;
            p84.x = (float)2.6137594E38F;
            p84.y = (float)4.9282853E37F;
            p84.z = (float) -2.515682E38F;
            p84.vx = (float) -2.4197505E37F;
            p84.vy = (float)2.7917676E38F;
            p84.vz = (float)2.5489077E38F;
            p84.afx = (float)3.1692646E38F;
            p84.afy = (float)2.784856E38F;
            p84.afz = (float)1.1986681E38F;
            p84.yaw = (float) -2.66306E38F;
            p84.yaw_rate = (float) -2.3206095E38F;
            LoopBackDemoChannel.instance.send(p84); //===============================
            SET_POSITION_TARGET_GLOBAL_INT p86 = LoopBackDemoChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.time_boot_ms = (uint)2870811682U;
            p86.target_system = (byte)(byte)172;
            p86.target_component = (byte)(byte)178;
            p86.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
            p86.type_mask = (ushort)(ushort)59829;
            p86.lat_int = (int)348004135;
            p86.lon_int = (int)1049324153;
            p86.alt = (float) -7.6616224E37F;
            p86.vx = (float)4.190452E37F;
            p86.vy = (float) -2.3505925E38F;
            p86.vz = (float) -1.4489147E38F;
            p86.afx = (float)3.2128609E38F;
            p86.afy = (float) -2.2094797E38F;
            p86.afz = (float) -9.128111E37F;
            p86.yaw = (float)9.313139E37F;
            p86.yaw_rate = (float)1.2431914E38F;
            LoopBackDemoChannel.instance.send(p86); //===============================
            POSITION_TARGET_GLOBAL_INT p87 = LoopBackDemoChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.time_boot_ms = (uint)1015109374U;
            p87.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_NED;
            p87.type_mask = (ushort)(ushort)46973;
            p87.lat_int = (int) -1079570547;
            p87.lon_int = (int) -1500722498;
            p87.alt = (float) -2.4373459E38F;
            p87.vx = (float)1.4040964E38F;
            p87.vy = (float) -8.1554535E37F;
            p87.vz = (float) -2.8062164E37F;
            p87.afx = (float) -2.0328394E38F;
            p87.afy = (float)3.2692954E38F;
            p87.afz = (float)2.8117476E38F;
            p87.yaw = (float) -6.688146E37F;
            p87.yaw_rate = (float) -3.1143504E38F;
            LoopBackDemoChannel.instance.send(p87); //===============================
            LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.time_boot_ms = (uint)2402271422U;
            p89.x = (float) -6.173674E37F;
            p89.y = (float)1.6017821E38F;
            p89.z = (float)1.8256306E38F;
            p89.roll = (float)2.8717971E38F;
            p89.pitch = (float) -4.3805025E37F;
            p89.yaw = (float)1.6289306E37F;
            LoopBackDemoChannel.instance.send(p89); //===============================
            HIL_STATE p90 = LoopBackDemoChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.time_usec = (ulong)6588617485934465368L;
            p90.roll = (float)1.1111414E38F;
            p90.pitch = (float) -2.9089947E38F;
            p90.yaw = (float)2.7078072E38F;
            p90.rollspeed = (float)2.6834036E38F;
            p90.pitchspeed = (float) -6.629366E37F;
            p90.yawspeed = (float) -2.6492147E38F;
            p90.lat = (int) -1246519914;
            p90.lon = (int) -1614580183;
            p90.alt = (int) -1428488452;
            p90.vx = (short)(short)4232;
            p90.vy = (short)(short) -27230;
            p90.vz = (short)(short)19333;
            p90.xacc = (short)(short)32385;
            p90.yacc = (short)(short)10614;
            p90.zacc = (short)(short) -30650;
            LoopBackDemoChannel.instance.send(p90); //===============================
            HIL_CONTROLS p91 = LoopBackDemoChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.time_usec = (ulong)730156911727528549L;
            p91.roll_ailerons = (float) -2.0967517E38F;
            p91.pitch_elevator = (float) -2.745969E38F;
            p91.yaw_rudder = (float)8.57193E37F;
            p91.throttle = (float)5.773621E37F;
            p91.aux1 = (float)1.8222794E37F;
            p91.aux2 = (float) -1.9486291E37F;
            p91.aux3 = (float) -5.975389E37F;
            p91.aux4 = (float) -3.3607663E38F;
            p91.mode = (MAV_MODE)MAV_MODE.MAV_MODE_AUTO_DISARMED;
            p91.nav_mode = (byte)(byte)248;
            LoopBackDemoChannel.instance.send(p91); //===============================
            HIL_RC_INPUTS_RAW p92 = LoopBackDemoChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.time_usec = (ulong)8199155581794079459L;
            p92.chan1_raw = (ushort)(ushort)34797;
            p92.chan2_raw = (ushort)(ushort)18708;
            p92.chan3_raw = (ushort)(ushort)52251;
            p92.chan4_raw = (ushort)(ushort)42176;
            p92.chan5_raw = (ushort)(ushort)9995;
            p92.chan6_raw = (ushort)(ushort)25345;
            p92.chan7_raw = (ushort)(ushort)63589;
            p92.chan8_raw = (ushort)(ushort)62592;
            p92.chan9_raw = (ushort)(ushort)22311;
            p92.chan10_raw = (ushort)(ushort)48176;
            p92.chan11_raw = (ushort)(ushort)62358;
            p92.chan12_raw = (ushort)(ushort)32178;
            p92.rssi = (byte)(byte)104;
            LoopBackDemoChannel.instance.send(p92); //===============================
            HIL_ACTUATOR_CONTROLS p93 = LoopBackDemoChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.time_usec = (ulong)7996251081224396317L;
            p93.controls_SET(new float[16], 0);
            p93.mode = (MAV_MODE)MAV_MODE.MAV_MODE_GUIDED_ARMED;
            p93.flags = (ulong)5420669329639663342L;
            LoopBackDemoChannel.instance.send(p93); //===============================
            OPTICAL_FLOW p100 = LoopBackDemoChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.time_usec = (ulong)2479731394618309346L;
            p100.sensor_id = (byte)(byte)1;
            p100.flow_x = (short)(short)6667;
            p100.flow_y = (short)(short) -17110;
            p100.flow_comp_m_x = (float) -3.3899742E38F;
            p100.flow_comp_m_y = (float) -2.664956E38F;
            p100.quality = (byte)(byte)46;
            p100.ground_distance = (float)2.8166184E38F;
            p100.flow_rate_x_SET((float)3.35933E38F, PH);
            p100.flow_rate_y_SET((float)2.2393764E38F, PH);
            LoopBackDemoChannel.instance.send(p100); //===============================
            GLOBAL_VISION_POSITION_ESTIMATE p101 = LoopBackDemoChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.usec = (ulong)3230151503023588655L;
            p101.x = (float)2.998315E38F;
            p101.y = (float)3.196017E38F;
            p101.z = (float) -1.7059168E38F;
            p101.roll = (float) -3.199972E38F;
            p101.pitch = (float)1.7916223E38F;
            p101.yaw = (float) -1.90682E38F;
            LoopBackDemoChannel.instance.send(p101); //===============================
            VISION_POSITION_ESTIMATE p102 = LoopBackDemoChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.usec = (ulong)3419387151089399278L;
            p102.x = (float)1.920362E38F;
            p102.y = (float)1.2714128E38F;
            p102.z = (float) -3.2362903E38F;
            p102.roll = (float)2.5690998E38F;
            p102.pitch = (float)2.1230912E38F;
            p102.yaw = (float)2.2233168E38F;
            LoopBackDemoChannel.instance.send(p102); //===============================
            VISION_SPEED_ESTIMATE p103 = LoopBackDemoChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.usec = (ulong)6778631463023518096L;
            p103.x = (float) -3.2205848E38F;
            p103.y = (float)1.8234665E38F;
            p103.z = (float)6.8821953E37F;
            LoopBackDemoChannel.instance.send(p103); //===============================
            VICON_POSITION_ESTIMATE p104 = LoopBackDemoChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.usec = (ulong)230243525654563149L;
            p104.x = (float)2.6219404E38F;
            p104.y = (float)2.6779178E38F;
            p104.z = (float)5.64991E37F;
            p104.roll = (float) -3.2258807E38F;
            p104.pitch = (float) -1.0825389E38F;
            p104.yaw = (float)4.741244E37F;
            LoopBackDemoChannel.instance.send(p104); //===============================
            HIGHRES_IMU p105 = LoopBackDemoChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.time_usec = (ulong)783598651799090905L;
            p105.xacc = (float)2.0244346E38F;
            p105.yacc = (float) -5.063975E37F;
            p105.zacc = (float)1.5957215E38F;
            p105.xgyro = (float) -3.0716845E37F;
            p105.ygyro = (float)2.2713035E38F;
            p105.zgyro = (float)3.1535095E38F;
            p105.xmag = (float) -4.5477584E37F;
            p105.ymag = (float)1.0300964E38F;
            p105.zmag = (float) -3.9500698E37F;
            p105.abs_pressure = (float) -2.6520139E38F;
            p105.diff_pressure = (float) -1.2294859E38F;
            p105.pressure_alt = (float) -1.6277748E38F;
            p105.temperature = (float) -3.1723915E37F;
            p105.fields_updated = (ushort)(ushort)1141;
            LoopBackDemoChannel.instance.send(p105); //===============================
            OPTICAL_FLOW_RAD p106 = LoopBackDemoChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.time_usec = (ulong)7928272244138513604L;
            p106.sensor_id = (byte)(byte)176;
            p106.integration_time_us = (uint)2683827190U;
            p106.integrated_x = (float) -1.3437181E37F;
            p106.integrated_y = (float)1.2411223E38F;
            p106.integrated_xgyro = (float) -1.3961285E38F;
            p106.integrated_ygyro = (float)1.9790612E38F;
            p106.integrated_zgyro = (float)2.2913568E38F;
            p106.temperature = (short)(short)11076;
            p106.quality = (byte)(byte)216;
            p106.time_delta_distance_us = (uint)1003804135U;
            p106.distance = (float)1.8970235E38F;
            LoopBackDemoChannel.instance.send(p106); //===============================
            HIL_SENSOR p107 = LoopBackDemoChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.time_usec = (ulong)9199870238509018502L;
            p107.xacc = (float) -3.2740821E37F;
            p107.yacc = (float)1.9985907E38F;
            p107.zacc = (float) -1.2567399E38F;
            p107.xgyro = (float) -2.628676E38F;
            p107.ygyro = (float)3.3876983E38F;
            p107.zgyro = (float) -2.5640284E38F;
            p107.xmag = (float)3.037575E37F;
            p107.ymag = (float)2.4157636E38F;
            p107.zmag = (float)1.0895148E38F;
            p107.abs_pressure = (float) -2.0779079E37F;
            p107.diff_pressure = (float)5.806093E37F;
            p107.pressure_alt = (float) -1.1771462E38F;
            p107.temperature = (float) -1.7465309E38F;
            p107.fields_updated = (uint)1844850000U;
            LoopBackDemoChannel.instance.send(p107); //===============================
            SIM_STATE p108 = LoopBackDemoChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.q1 = (float)2.5380083E38F;
            p108.q2 = (float) -2.874567E37F;
            p108.q3 = (float) -2.4387117E38F;
            p108.q4 = (float)6.766405E37F;
            p108.roll = (float)5.812448E37F;
            p108.pitch = (float) -1.5270485E38F;
            p108.yaw = (float)4.3058957E37F;
            p108.xacc = (float)3.0817595E37F;
            p108.yacc = (float) -1.4451762E38F;
            p108.zacc = (float)3.2688014E38F;
            p108.xgyro = (float)1.3468494E38F;
            p108.ygyro = (float)2.9233222E38F;
            p108.zgyro = (float)3.185625E38F;
            p108.lat = (float) -9.459123E37F;
            p108.lon = (float)2.2897836E38F;
            p108.alt = (float) -2.4730516E38F;
            p108.std_dev_horz = (float) -8.348206E37F;
            p108.std_dev_vert = (float)5.6976245E37F;
            p108.vn = (float)3.86542E37F;
            p108.ve = (float) -1.6642881E38F;
            p108.vd = (float) -1.9774854E38F;
            LoopBackDemoChannel.instance.send(p108); //===============================
            RADIO_STATUS p109 = LoopBackDemoChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.rssi = (byte)(byte)241;
            p109.remrssi = (byte)(byte)238;
            p109.txbuf = (byte)(byte)213;
            p109.noise = (byte)(byte)42;
            p109.remnoise = (byte)(byte)25;
            p109.rxerrors = (ushort)(ushort)52591;
            p109.fixed_ = (ushort)(ushort)27760;
            LoopBackDemoChannel.instance.send(p109); //===============================
            FILE_TRANSFER_PROTOCOL p110 = LoopBackDemoChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_network = (byte)(byte)31;
            p110.target_system = (byte)(byte)161;
            p110.target_component = (byte)(byte)49;
            p110.payload_SET(new byte[251], 0);
            LoopBackDemoChannel.instance.send(p110); //===============================
            TIMESYNC p111 = LoopBackDemoChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.tc1 = (long) -9152413316434455988L;
            p111.ts1 = (long) -3553802751858594445L;
            LoopBackDemoChannel.instance.send(p111); //===============================
            CAMERA_TRIGGER p112 = LoopBackDemoChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.time_usec = (ulong)1482415474719886794L;
            p112.seq = (uint)4027759862U;
            LoopBackDemoChannel.instance.send(p112); //===============================
            HIL_GPS p113 = LoopBackDemoChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.time_usec = (ulong)8270750121406622613L;
            p113.fix_type = (byte)(byte)174;
            p113.lat = (int)1682743773;
            p113.lon = (int) -48309646;
            p113.alt = (int)544292904;
            p113.eph = (ushort)(ushort)6064;
            p113.epv = (ushort)(ushort)56521;
            p113.vel = (ushort)(ushort)5934;
            p113.vn = (short)(short) -19227;
            p113.ve = (short)(short)7969;
            p113.vd = (short)(short) -10017;
            p113.cog = (ushort)(ushort)33348;
            p113.satellites_visible = (byte)(byte)215;
            LoopBackDemoChannel.instance.send(p113); //===============================
            HIL_OPTICAL_FLOW p114 = LoopBackDemoChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.time_usec = (ulong)2042452808166132590L;
            p114.sensor_id = (byte)(byte)87;
            p114.integration_time_us = (uint)50978586U;
            p114.integrated_x = (float) -1.5929306E37F;
            p114.integrated_y = (float) -1.6076132E38F;
            p114.integrated_xgyro = (float) -2.4524336E38F;
            p114.integrated_ygyro = (float)1.3988659E38F;
            p114.integrated_zgyro = (float) -8.709975E37F;
            p114.temperature = (short)(short) -1920;
            p114.quality = (byte)(byte)159;
            p114.time_delta_distance_us = (uint)1733889783U;
            p114.distance = (float) -4.7550553E37F;
            LoopBackDemoChannel.instance.send(p114); //===============================
            HIL_STATE_QUATERNION p115 = LoopBackDemoChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.time_usec = (ulong)5247380210099355023L;
            p115.attitude_quaternion_SET(new float[4], 0);
            p115.rollspeed = (float) -1.8670086E38F;
            p115.pitchspeed = (float) -6.9760293E37F;
            p115.yawspeed = (float)1.5339242E38F;
            p115.lat = (int) -1873384;
            p115.lon = (int)1799831296;
            p115.alt = (int) -1381850142;
            p115.vx = (short)(short)1865;
            p115.vy = (short)(short)30564;
            p115.vz = (short)(short) -30773;
            p115.ind_airspeed = (ushort)(ushort)41032;
            p115.true_airspeed = (ushort)(ushort)10905;
            p115.xacc = (short)(short)29917;
            p115.yacc = (short)(short)20044;
            p115.zacc = (short)(short) -10691;
            LoopBackDemoChannel.instance.send(p115); //===============================
            SCALED_IMU2 p116 = LoopBackDemoChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.time_boot_ms = (uint)536414531U;
            p116.xacc = (short)(short)1412;
            p116.yacc = (short)(short) -20127;
            p116.zacc = (short)(short) -14420;
            p116.xgyro = (short)(short)21260;
            p116.ygyro = (short)(short) -17374;
            p116.zgyro = (short)(short)3310;
            p116.xmag = (short)(short) -3154;
            p116.ymag = (short)(short) -20999;
            p116.zmag = (short)(short) -19115;
            LoopBackDemoChannel.instance.send(p116); //===============================
            LOG_REQUEST_LIST p117 = LoopBackDemoChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.target_system = (byte)(byte)113;
            p117.target_component = (byte)(byte)232;
            p117.start = (ushort)(ushort)20133;
            p117.end = (ushort)(ushort)20258;
            LoopBackDemoChannel.instance.send(p117); //===============================
            LOG_ENTRY p118 = LoopBackDemoChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.id = (ushort)(ushort)10261;
            p118.num_logs = (ushort)(ushort)42657;
            p118.last_log_num = (ushort)(ushort)50523;
            p118.time_utc = (uint)1436162321U;
            p118.size = (uint)1560719922U;
            LoopBackDemoChannel.instance.send(p118); //===============================
            LOG_REQUEST_DATA p119 = LoopBackDemoChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.target_system = (byte)(byte)89;
            p119.target_component = (byte)(byte)133;
            p119.id = (ushort)(ushort)8646;
            p119.ofs = (uint)702606567U;
            p119.count = (uint)3351964457U;
            LoopBackDemoChannel.instance.send(p119); //===============================
            LOG_DATA p120 = LoopBackDemoChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.id = (ushort)(ushort)46017;
            p120.ofs = (uint)1546519294U;
            p120.count = (byte)(byte)181;
            p120.data__SET(new byte[90], 0);
            LoopBackDemoChannel.instance.send(p120); //===============================
            LOG_ERASE p121 = LoopBackDemoChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_system = (byte)(byte)74;
            p121.target_component = (byte)(byte)203;
            LoopBackDemoChannel.instance.send(p121); //===============================
            LOG_REQUEST_END p122 = LoopBackDemoChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_system = (byte)(byte)214;
            p122.target_component = (byte)(byte)193;
            LoopBackDemoChannel.instance.send(p122); //===============================
            GPS_INJECT_DATA p123 = LoopBackDemoChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.target_system = (byte)(byte)77;
            p123.target_component = (byte)(byte)211;
            p123.len = (byte)(byte)25;
            p123.data__SET(new byte[110], 0);
            LoopBackDemoChannel.instance.send(p123); //===============================
            GPS2_RAW p124 = LoopBackDemoChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.time_usec = (ulong)2597462422390968364L;
            p124.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT;
            p124.lat = (int)1709909385;
            p124.lon = (int) -923013933;
            p124.alt = (int)139518168;
            p124.eph = (ushort)(ushort)55440;
            p124.epv = (ushort)(ushort)30926;
            p124.vel = (ushort)(ushort)59264;
            p124.cog = (ushort)(ushort)53057;
            p124.satellites_visible = (byte)(byte)74;
            p124.dgps_numch = (byte)(byte)137;
            p124.dgps_age = (uint)3831146759U;
            LoopBackDemoChannel.instance.send(p124); //===============================
            POWER_STATUS p125 = LoopBackDemoChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vcc = (ushort)(ushort)54850;
            p125.Vservo = (ushort)(ushort)40832;
            p125.flags = (MAV_POWER_STATUS)MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT;
            LoopBackDemoChannel.instance.send(p125); //===============================
            SERIAL_CONTROL p126 = LoopBackDemoChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.device = (SERIAL_CONTROL_DEV)SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2;
            p126.flags = (SERIAL_CONTROL_FLAG)SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND;
            p126.timeout = (ushort)(ushort)1744;
            p126.baudrate = (uint)3455141465U;
            p126.count = (byte)(byte)25;
            p126.data__SET(new byte[70], 0);
            LoopBackDemoChannel.instance.send(p126); //===============================
            GPS_RTK p127 = LoopBackDemoChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.time_last_baseline_ms = (uint)1170972574U;
            p127.rtk_receiver_id = (byte)(byte)203;
            p127.wn = (ushort)(ushort)25939;
            p127.tow = (uint)2154769123U;
            p127.rtk_health = (byte)(byte)181;
            p127.rtk_rate = (byte)(byte)182;
            p127.nsats = (byte)(byte)133;
            p127.baseline_coords_type = (byte)(byte)73;
            p127.baseline_a_mm = (int)167840338;
            p127.baseline_b_mm = (int)336864213;
            p127.baseline_c_mm = (int) -617294983;
            p127.accuracy = (uint)1185745945U;
            p127.iar_num_hypotheses = (int)94606114;
            LoopBackDemoChannel.instance.send(p127); //===============================
            GPS2_RTK p128 = LoopBackDemoChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.time_last_baseline_ms = (uint)3785933947U;
            p128.rtk_receiver_id = (byte)(byte)14;
            p128.wn = (ushort)(ushort)25862;
            p128.tow = (uint)917527492U;
            p128.rtk_health = (byte)(byte)110;
            p128.rtk_rate = (byte)(byte)24;
            p128.nsats = (byte)(byte)133;
            p128.baseline_coords_type = (byte)(byte)190;
            p128.baseline_a_mm = (int)627363413;
            p128.baseline_b_mm = (int) -1303363568;
            p128.baseline_c_mm = (int)1326009729;
            p128.accuracy = (uint)3661152670U;
            p128.iar_num_hypotheses = (int) -2139523374;
            LoopBackDemoChannel.instance.send(p128); //===============================
            SCALED_IMU3 p129 = LoopBackDemoChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.time_boot_ms = (uint)113101817U;
            p129.xacc = (short)(short)30705;
            p129.yacc = (short)(short)22165;
            p129.zacc = (short)(short) -22401;
            p129.xgyro = (short)(short)27944;
            p129.ygyro = (short)(short) -23778;
            p129.zgyro = (short)(short) -29074;
            p129.xmag = (short)(short) -9309;
            p129.ymag = (short)(short)5785;
            p129.zmag = (short)(short)22519;
            LoopBackDemoChannel.instance.send(p129); //===============================
            DATA_TRANSMISSION_HANDSHAKE p130 = LoopBackDemoChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.type = (byte)(byte)71;
            p130.size = (uint)1990197548U;
            p130.width = (ushort)(ushort)31536;
            p130.height = (ushort)(ushort)21673;
            p130.packets = (ushort)(ushort)30835;
            p130.payload = (byte)(byte)245;
            p130.jpg_quality = (byte)(byte)127;
            LoopBackDemoChannel.instance.send(p130); //===============================
            ENCAPSULATED_DATA p131 = LoopBackDemoChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.seqnr = (ushort)(ushort)20836;
            p131.data__SET(new byte[253], 0);
            LoopBackDemoChannel.instance.send(p131); //===============================
            DISTANCE_SENSOR p132 = LoopBackDemoChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.time_boot_ms = (uint)352531369U;
            p132.min_distance = (ushort)(ushort)57831;
            p132.max_distance = (ushort)(ushort)63482;
            p132.current_distance = (ushort)(ushort)28086;
            p132.type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER;
            p132.id = (byte)(byte)198;
            p132.orientation = (MAV_SENSOR_ORIENTATION)MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_270_YAW_45;
            p132.covariance = (byte)(byte)151;
            LoopBackDemoChannel.instance.send(p132); //===============================
            TERRAIN_REQUEST p133 = LoopBackDemoChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.lat = (int) -394722838;
            p133.lon = (int)821877918;
            p133.grid_spacing = (ushort)(ushort)40477;
            p133.mask = (ulong)7269386274982452611L;
            LoopBackDemoChannel.instance.send(p133); //===============================
            TERRAIN_DATA p134 = LoopBackDemoChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.lat = (int) -1542165117;
            p134.lon = (int) -1022310650;
            p134.grid_spacing = (ushort)(ushort)7357;
            p134.gridbit = (byte)(byte)152;
            p134.data__SET(new short[16], 0);
            LoopBackDemoChannel.instance.send(p134); //===============================
            TERRAIN_CHECK p135 = LoopBackDemoChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lat = (int)96096166;
            p135.lon = (int)774424135;
            LoopBackDemoChannel.instance.send(p135); //===============================
            TERRAIN_REPORT p136 = LoopBackDemoChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.lat = (int)562219753;
            p136.lon = (int) -1261025351;
            p136.spacing = (ushort)(ushort)65143;
            p136.terrain_height = (float)2.12161E38F;
            p136.current_height = (float) -3.0580523E38F;
            p136.pending = (ushort)(ushort)19388;
            p136.loaded = (ushort)(ushort)55599;
            LoopBackDemoChannel.instance.send(p136); //===============================
            SCALED_PRESSURE2 p137 = LoopBackDemoChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.time_boot_ms = (uint)2252395622U;
            p137.press_abs = (float) -7.765456E37F;
            p137.press_diff = (float)6.650256E36F;
            p137.temperature = (short)(short)1049;
            LoopBackDemoChannel.instance.send(p137); //===============================
            ATT_POS_MOCAP p138 = LoopBackDemoChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.time_usec = (ulong)2144686427877289701L;
            p138.q_SET(new float[4], 0);
            p138.x = (float) -1.3249948E38F;
            p138.y = (float)2.7127386E38F;
            p138.z = (float)3.3578462E38F;
            LoopBackDemoChannel.instance.send(p138); //===============================
            SET_ACTUATOR_CONTROL_TARGET p139 = LoopBackDemoChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.time_usec = (ulong)7915535323214624572L;
            p139.group_mlx = (byte)(byte)32;
            p139.target_system = (byte)(byte)34;
            p139.target_component = (byte)(byte)185;
            p139.controls_SET(new float[8], 0);
            LoopBackDemoChannel.instance.send(p139); //===============================
            ACTUATOR_CONTROL_TARGET p140 = LoopBackDemoChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.time_usec = (ulong)1409246522097998943L;
            p140.group_mlx = (byte)(byte)124;
            p140.controls_SET(new float[8], 0);
            LoopBackDemoChannel.instance.send(p140); //===============================
            ALTITUDE p141 = LoopBackDemoChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.time_usec = (ulong)5567107745739047961L;
            p141.altitude_monotonic = (float) -8.303594E37F;
            p141.altitude_amsl = (float) -2.5206944E38F;
            p141.altitude_local = (float) -4.8139965E37F;
            p141.altitude_relative = (float) -3.0779254E38F;
            p141.altitude_terrain = (float)1.8153722E38F;
            p141.bottom_clearance = (float)2.0703156E38F;
            LoopBackDemoChannel.instance.send(p141); //===============================
            RESOURCE_REQUEST p142 = LoopBackDemoChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.request_id = (byte)(byte)153;
            p142.uri_type = (byte)(byte)51;
            p142.uri_SET(new byte[120], 0);
            p142.transfer_type = (byte)(byte)182;
            p142.storage_SET(new byte[120], 0);
            LoopBackDemoChannel.instance.send(p142); //===============================
            SCALED_PRESSURE3 p143 = LoopBackDemoChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.time_boot_ms = (uint)3996874231U;
            p143.press_abs = (float)1.4116882E38F;
            p143.press_diff = (float)1.9042856E37F;
            p143.temperature = (short)(short)22883;
            LoopBackDemoChannel.instance.send(p143); //===============================
            FOLLOW_TARGET p144 = LoopBackDemoChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.timestamp = (ulong)2159660813587159358L;
            p144.est_capabilities = (byte)(byte)26;
            p144.lat = (int) -1582528427;
            p144.lon = (int) -828119383;
            p144.alt = (float)2.4377651E38F;
            p144.vel_SET(new float[3], 0);
            p144.acc_SET(new float[3], 0);
            p144.attitude_q_SET(new float[4], 0);
            p144.rates_SET(new float[3], 0);
            p144.position_cov_SET(new float[3], 0);
            p144.custom_state = (ulong)2273625433561775208L;
            LoopBackDemoChannel.instance.send(p144); //===============================
            CONTROL_SYSTEM_STATE p146 = LoopBackDemoChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.time_usec = (ulong)6416607430153552367L;
            p146.x_acc = (float)2.1034536E38F;
            p146.y_acc = (float) -2.881107E37F;
            p146.z_acc = (float)1.3540464E38F;
            p146.x_vel = (float)3.4029586E37F;
            p146.y_vel = (float) -2.2653809E38F;
            p146.z_vel = (float)1.1845596E38F;
            p146.x_pos = (float)3.3581399E38F;
            p146.y_pos = (float) -2.4232644E38F;
            p146.z_pos = (float) -8.3222677E37F;
            p146.airspeed = (float)1.9153626E37F;
            p146.vel_variance_SET(new float[3], 0);
            p146.pos_variance_SET(new float[3], 0);
            p146.q_SET(new float[4], 0);
            p146.roll_rate = (float)2.8394102E38F;
            p146.pitch_rate = (float)4.303574E37F;
            p146.yaw_rate = (float)2.9571518E38F;
            LoopBackDemoChannel.instance.send(p146); //===============================
            BATTERY_STATUS p147 = LoopBackDemoChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.id = (byte)(byte)182;
            p147.battery_function = (MAV_BATTERY_FUNCTION)MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_ALL;
            p147.type = (MAV_BATTERY_TYPE)MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_NIMH;
            p147.temperature = (short)(short) -20931;
            p147.voltages_SET(new ushort[10], 0);
            p147.current_battery = (short)(short)1875;
            p147.current_consumed = (int)713002880;
            p147.energy_consumed = (int)1841781964;
            p147.battery_remaining = (sbyte)(sbyte) - 83;
            LoopBackDemoChannel.instance.send(p147); //===============================
            AUTOPILOT_VERSION p148 = LoopBackDemoChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY)MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT;
            p148.flight_sw_version = (uint)1297152407U;
            p148.middleware_sw_version = (uint)47330491U;
            p148.os_sw_version = (uint)1325778604U;
            p148.board_version = (uint)947456352U;
            p148.flight_custom_version_SET(new byte[8], 0);
            p148.middleware_custom_version_SET(new byte[8], 0);
            p148.os_custom_version_SET(new byte[8], 0);
            p148.vendor_id = (ushort)(ushort)63331;
            p148.product_id = (ushort)(ushort)42823;
            p148.uid = (ulong)8625905002598759807L;
            p148.uid2_SET(new byte[18], 0, PH);
            LoopBackDemoChannel.instance.send(p148); //===============================
            LANDING_TARGET p149 = LoopBackDemoChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.time_usec = (ulong)677040562171828221L;
            p149.target_num = (byte)(byte)1;
            p149.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT;
            p149.angle_x = (float) -2.6989827E38F;
            p149.angle_y = (float) -2.516507E38F;
            p149.distance = (float) -7.713779E37F;
            p149.size_x = (float) -1.2622696E38F;
            p149.size_y = (float) -3.117933E38F;
            p149.x_SET((float)1.3999533E38F, PH);
            p149.y_SET((float)3.0582659E38F, PH);
            p149.z_SET((float)1.8685117E38F, PH);
            p149.q_SET(new float[4], 0, PH);
            p149.type = (LANDING_TARGET_TYPE)LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON;
            p149.position_valid_SET((byte)(byte)27, PH);
            LoopBackDemoChannel.instance.send(p149); //===============================
            SCRIPT_ITEM p180 = LoopBackDemoChannel.new_SCRIPT_ITEM();
            PH.setPack(p180);
            p180.target_system = (byte)(byte)138;
            p180.target_component = (byte)(byte)128;
            p180.seq = (ushort)(ushort)63100;
            p180.name_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p180); //===============================
            SCRIPT_REQUEST p181 = LoopBackDemoChannel.new_SCRIPT_REQUEST();
            PH.setPack(p181);
            p181.target_system = (byte)(byte)37;
            p181.target_component = (byte)(byte)136;
            p181.seq = (ushort)(ushort)34531;
            LoopBackDemoChannel.instance.send(p181); //===============================
            SCRIPT_REQUEST_LIST p182 = LoopBackDemoChannel.new_SCRIPT_REQUEST_LIST();
            PH.setPack(p182);
            p182.target_system = (byte)(byte)243;
            p182.target_component = (byte)(byte)28;
            LoopBackDemoChannel.instance.send(p182); //===============================
            SCRIPT_COUNT p183 = LoopBackDemoChannel.new_SCRIPT_COUNT();
            PH.setPack(p183);
            p183.target_system = (byte)(byte)210;
            p183.target_component = (byte)(byte)55;
            p183.count = (ushort)(ushort)44908;
            LoopBackDemoChannel.instance.send(p183); //===============================
            SCRIPT_CURRENT p184 = LoopBackDemoChannel.new_SCRIPT_CURRENT();
            PH.setPack(p184);
            p184.seq = (ushort)(ushort)34218;
            LoopBackDemoChannel.instance.send(p184); //===============================
            ESTIMATOR_STATUS p230 = LoopBackDemoChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.time_usec = (ulong)1429983495051966775L;
            p230.flags = (ESTIMATOR_STATUS_FLAGS)ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS;
            p230.vel_ratio = (float) -1.563993E37F;
            p230.pos_horiz_ratio = (float) -2.1612664E38F;
            p230.pos_vert_ratio = (float) -1.0221906E38F;
            p230.mag_ratio = (float) -1.907219E38F;
            p230.hagl_ratio = (float)2.6028771E38F;
            p230.tas_ratio = (float) -6.2150607E37F;
            p230.pos_horiz_accuracy = (float) -2.4522307E38F;
            p230.pos_vert_accuracy = (float)3.2602395E38F;
            LoopBackDemoChannel.instance.send(p230); //===============================
            WIND_COV p231 = LoopBackDemoChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.time_usec = (ulong)4925580729582627213L;
            p231.wind_x = (float)2.9151573E38F;
            p231.wind_y = (float) -1.354587E38F;
            p231.wind_z = (float)1.4865862E38F;
            p231.var_horiz = (float) -1.6929066E38F;
            p231.var_vert = (float)2.4251675E38F;
            p231.wind_alt = (float)1.4682943E38F;
            p231.horiz_accuracy = (float) -1.3369348E38F;
            p231.vert_accuracy = (float) -4.966766E37F;
            LoopBackDemoChannel.instance.send(p231); //===============================
            GPS_INPUT p232 = LoopBackDemoChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.time_usec = (ulong)6804279508672431346L;
            p232.gps_id = (byte)(byte)125;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS)GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP;
            p232.time_week_ms = (uint)1358574047U;
            p232.time_week = (ushort)(ushort)43241;
            p232.fix_type = (byte)(byte)22;
            p232.lat = (int) -1152668362;
            p232.lon = (int) -153457830;
            p232.alt = (float)6.7820646E37F;
            p232.hdop = (float) -1.0149886E38F;
            p232.vdop = (float)3.828406E37F;
            p232.vn = (float) -2.963177E38F;
            p232.ve = (float) -2.8986281E38F;
            p232.vd = (float)2.2128083E38F;
            p232.speed_accuracy = (float)1.498187E38F;
            p232.horiz_accuracy = (float)1.3738653E38F;
            p232.vert_accuracy = (float)3.2535413E38F;
            p232.satellites_visible = (byte)(byte)101;
            LoopBackDemoChannel.instance.send(p232); //===============================
            GPS_RTCM_DATA p233 = LoopBackDemoChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.flags = (byte)(byte)5;
            p233.len = (byte)(byte)182;
            p233.data__SET(new byte[180], 0);
            LoopBackDemoChannel.instance.send(p233); //===============================
            HIGH_LATENCY p234 = LoopBackDemoChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
            p234.custom_mode = (uint)1131288373U;
            p234.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND;
            p234.roll = (short)(short) -6150;
            p234.pitch = (short)(short)11218;
            p234.heading = (ushort)(ushort)34289;
            p234.throttle = (sbyte)(sbyte)26;
            p234.heading_sp = (short)(short)28818;
            p234.latitude = (int)1560875827;
            p234.longitude = (int) -1932345231;
            p234.altitude_amsl = (short)(short)13798;
            p234.altitude_sp = (short)(short)27135;
            p234.airspeed = (byte)(byte)30;
            p234.airspeed_sp = (byte)(byte)194;
            p234.groundspeed = (byte)(byte)126;
            p234.climb_rate = (sbyte)(sbyte)86;
            p234.gps_nsat = (byte)(byte)148;
            p234.gps_fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX;
            p234.battery_remaining = (byte)(byte)90;
            p234.temperature = (sbyte)(sbyte)33;
            p234.temperature_air = (sbyte)(sbyte) - 96;
            p234.failsafe = (byte)(byte)25;
            p234.wp_num = (byte)(byte)86;
            p234.wp_distance = (ushort)(ushort)4934;
            LoopBackDemoChannel.instance.send(p234); //===============================
            VIBRATION p241 = LoopBackDemoChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.time_usec = (ulong)7346501212883167802L;
            p241.vibration_x = (float)4.874104E37F;
            p241.vibration_y = (float)2.9452318E37F;
            p241.vibration_z = (float)3.142584E38F;
            p241.clipping_0 = (uint)1836590168U;
            p241.clipping_1 = (uint)542238046U;
            p241.clipping_2 = (uint)1034210131U;
            LoopBackDemoChannel.instance.send(p241); //===============================
            HOME_POSITION p242 = LoopBackDemoChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.latitude = (int) -1822940282;
            p242.longitude = (int) -1738179157;
            p242.altitude = (int)1742162122;
            p242.x = (float) -1.241049E38F;
            p242.y = (float)1.4885655E38F;
            p242.z = (float)2.7858699E38F;
            p242.q_SET(new float[4], 0);
            p242.approach_x = (float) -1.0001222E38F;
            p242.approach_y = (float)1.0140367E38F;
            p242.approach_z = (float) -3.3528792E38F;
            p242.time_usec_SET((ulong)5976522365718679178L, PH);
            LoopBackDemoChannel.instance.send(p242); //===============================
            SET_HOME_POSITION p243 = LoopBackDemoChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.target_system = (byte)(byte)92;
            p243.latitude = (int) -303979844;
            p243.longitude = (int)1722685488;
            p243.altitude = (int)2042358873;
            p243.x = (float) -3.3328225E37F;
            p243.y = (float)5.964511E37F;
            p243.z = (float)3.538495E37F;
            p243.q_SET(new float[4], 0);
            p243.approach_x = (float)2.0244315E38F;
            p243.approach_y = (float) -2.1580575E38F;
            p243.approach_z = (float)1.4108018E35F;
            p243.time_usec_SET((ulong)4539965107728017541L, PH);
            LoopBackDemoChannel.instance.send(p243); //===============================
            MESSAGE_INTERVAL p244 = LoopBackDemoChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.message_id = (ushort)(ushort)24755;
            p244.interval_us = (int)1948154615;
            LoopBackDemoChannel.instance.send(p244); //===============================
            EXTENDED_SYS_STATE p245 = LoopBackDemoChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.vtol_state = (MAV_VTOL_STATE)MAV_VTOL_STATE.MAV_VTOL_STATE_UNDEFINED;
            p245.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF;
            LoopBackDemoChannel.instance.send(p245); //===============================
            ADSB_VEHICLE p246 = LoopBackDemoChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.ICAO_address = (uint)1174034873U;
            p246.lat = (int)336427653;
            p246.lon = (int)1864934021;
            p246.altitude_type = (ADSB_ALTITUDE_TYPE)ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC;
            p246.altitude = (int) -1851310262;
            p246.heading = (ushort)(ushort)18832;
            p246.hor_velocity = (ushort)(ushort)19782;
            p246.ver_velocity = (short)(short)28911;
            p246.callsign_SET("DEMO", PH);
            p246.emitter_type = (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_LIGHT;
            p246.tslc = (byte)(byte)214;
            p246.flags = (ADSB_FLAGS)ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN;
            p246.squawk = (ushort)(ushort)64525;
            LoopBackDemoChannel.instance.send(p246); //===============================
            COLLISION p247 = LoopBackDemoChannel.new_COLLISION();
            PH.setPack(p247);
            p247.src_ = (MAV_COLLISION_SRC)MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB;
            p247.id = (uint)2218483307U;
            p247.action = (MAV_COLLISION_ACTION)MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_PERPENDICULAR;
            p247.threat_level = (MAV_COLLISION_THREAT_LEVEL)MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH;
            p247.time_to_minimum_delta = (float)1.8562875E37F;
            p247.altitude_minimum_delta = (float)3.1851447E38F;
            p247.horizontal_minimum_delta = (float)9.483675E36F;
            LoopBackDemoChannel.instance.send(p247); //===============================
            V2_EXTENSION p248 = LoopBackDemoChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.target_network = (byte)(byte)106;
            p248.target_system = (byte)(byte)62;
            p248.target_component = (byte)(byte)80;
            p248.message_type = (ushort)(ushort)36145;
            p248.payload_SET(new byte[249], 0);
            LoopBackDemoChannel.instance.send(p248); //===============================
            MEMORY_VECT p249 = LoopBackDemoChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.address = (ushort)(ushort)31321;
            p249.ver = (byte)(byte)74;
            p249.type = (byte)(byte)78;
            p249.value_SET(new sbyte[32], 0);
            LoopBackDemoChannel.instance.send(p249); //===============================
            DEBUG_VECT p250 = LoopBackDemoChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.name_SET("DEMO", PH);
            p250.time_usec = (ulong)7376474087208314278L;
            p250.x = (float)9.216989E37F;
            p250.y = (float) -6.950414E36F;
            p250.z = (float) -1.3928674E37F;
            LoopBackDemoChannel.instance.send(p250); //===============================
            NAMED_VALUE_FLOAT p251 = LoopBackDemoChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.time_boot_ms = (uint)4034744938U;
            p251.name_SET("DEMO", PH);
            p251.value = (float) -1.5071991E38F;
            LoopBackDemoChannel.instance.send(p251); //===============================
            NAMED_VALUE_INT p252 = LoopBackDemoChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.time_boot_ms = (uint)2817034738U;
            p252.name_SET("DEMO", PH);
            p252.value = (int)117535952;
            LoopBackDemoChannel.instance.send(p252); //===============================
            STATUSTEXT p253 = LoopBackDemoChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.severity = (MAV_SEVERITY)MAV_SEVERITY.MAV_SEVERITY_CRITICAL;
            p253.text_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p253); //===============================
            DEBUG p254 = LoopBackDemoChannel.new_DEBUG();
            PH.setPack(p254);
            p254.time_boot_ms = (uint)1339174761U;
            p254.ind = (byte)(byte)45;
            p254.value = (float)4.6437743E37F;
            LoopBackDemoChannel.instance.send(p254); //===============================
            SETUP_SIGNING p256 = LoopBackDemoChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.target_system = (byte)(byte)63;
            p256.target_component = (byte)(byte)150;
            p256.secret_key_SET(new byte[32], 0);
            p256.initial_timestamp = (ulong)5076128684122692350L;
            LoopBackDemoChannel.instance.send(p256); //===============================
            BUTTON_CHANGE p257 = LoopBackDemoChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.time_boot_ms = (uint)977068401U;
            p257.last_change_ms = (uint)3021547886U;
            p257.state = (byte)(byte)255;
            LoopBackDemoChannel.instance.send(p257); //===============================
            PLAY_TUNE p258 = LoopBackDemoChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_system = (byte)(byte)86;
            p258.target_component = (byte)(byte)171;
            p258.tune_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p258); //===============================
            CAMERA_INFORMATION p259 = LoopBackDemoChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.time_boot_ms = (uint)2584447274U;
            p259.vendor_name_SET(new byte[32], 0);
            p259.model_name_SET(new byte[32], 0);
            p259.firmware_version = (uint)115245182U;
            p259.focal_length = (float)2.4165033E38F;
            p259.sensor_size_h = (float) -2.1303986E38F;
            p259.sensor_size_v = (float) -6.658031E37F;
            p259.resolution_h = (ushort)(ushort)53686;
            p259.resolution_v = (ushort)(ushort)9176;
            p259.lens_id = (byte)(byte)173;
            p259.flags = (CAMERA_CAP_FLAGS)CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_MODES;
            p259.cam_definition_version = (ushort)(ushort)55367;
            p259.cam_definition_uri_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p259); //===============================
            CAMERA_SETTINGS p260 = LoopBackDemoChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.time_boot_ms = (uint)3087904682U;
            p260.mode_id = (CAMERA_MODE)CAMERA_MODE.CAMERA_MODE_VIDEO;
            LoopBackDemoChannel.instance.send(p260); //===============================
            STORAGE_INFORMATION p261 = LoopBackDemoChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.time_boot_ms = (uint)2683904016U;
            p261.storage_id = (byte)(byte)45;
            p261.storage_count = (byte)(byte)56;
            p261.status = (byte)(byte)63;
            p261.total_capacity = (float) -1.4770645E38F;
            p261.used_capacity = (float) -1.1999247E38F;
            p261.available_capacity = (float)1.5948753E38F;
            p261.read_speed = (float) -1.4693746E38F;
            p261.write_speed = (float) -1.0264704E38F;
            LoopBackDemoChannel.instance.send(p261); //===============================
            CAMERA_CAPTURE_STATUS p262 = LoopBackDemoChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.time_boot_ms = (uint)456946195U;
            p262.image_status = (byte)(byte)92;
            p262.video_status = (byte)(byte)20;
            p262.image_interval = (float)1.5354691E38F;
            p262.recording_time_ms = (uint)2274939747U;
            p262.available_capacity = (float) -2.927044E38F;
            LoopBackDemoChannel.instance.send(p262); //===============================
            CAMERA_IMAGE_CAPTURED p263 = LoopBackDemoChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.time_boot_ms = (uint)3972449831U;
            p263.time_utc = (ulong)4838478705026977913L;
            p263.camera_id = (byte)(byte)193;
            p263.lat = (int) -636446474;
            p263.lon = (int) -1495816251;
            p263.alt = (int) -121455970;
            p263.relative_alt = (int) -725639319;
            p263.q_SET(new float[4], 0);
            p263.image_index = (int) -559534843;
            p263.capture_result = (sbyte)(sbyte)127;
            p263.file_url_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p263); //===============================
            FLIGHT_INFORMATION p264 = LoopBackDemoChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.time_boot_ms = (uint)1501472526U;
            p264.arming_time_utc = (ulong)9206794046926628485L;
            p264.takeoff_time_utc = (ulong)2188728460932928323L;
            p264.flight_uuid = (ulong)5266914745600472725L;
            LoopBackDemoChannel.instance.send(p264); //===============================
            MOUNT_ORIENTATION p265 = LoopBackDemoChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.time_boot_ms = (uint)3389815459U;
            p265.roll = (float) -2.6797084E37F;
            p265.pitch = (float)1.4544052E38F;
            p265.yaw = (float)1.1738538E38F;
            LoopBackDemoChannel.instance.send(p265); //===============================
            LOGGING_DATA p266 = LoopBackDemoChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.target_system = (byte)(byte)133;
            p266.target_component = (byte)(byte)89;
            p266.sequence = (ushort)(ushort)40709;
            p266.length = (byte)(byte)18;
            p266.first_message_offset = (byte)(byte)40;
            p266.data__SET(new byte[249], 0);
            LoopBackDemoChannel.instance.send(p266); //===============================
            LOGGING_DATA_ACKED p267 = LoopBackDemoChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.target_system = (byte)(byte)37;
            p267.target_component = (byte)(byte)117;
            p267.sequence = (ushort)(ushort)11680;
            p267.length = (byte)(byte)118;
            p267.first_message_offset = (byte)(byte)195;
            p267.data__SET(new byte[249], 0);
            LoopBackDemoChannel.instance.send(p267); //===============================
            LOGGING_ACK p268 = LoopBackDemoChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_system = (byte)(byte)209;
            p268.target_component = (byte)(byte)50;
            p268.sequence = (ushort)(ushort)57505;
            LoopBackDemoChannel.instance.send(p268); //===============================
            VIDEO_STREAM_INFORMATION p269 = LoopBackDemoChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.camera_id = (byte)(byte)148;
            p269.status = (byte)(byte)250;
            p269.framerate = (float) -9.014185E37F;
            p269.resolution_h = (ushort)(ushort)61654;
            p269.resolution_v = (ushort)(ushort)15110;
            p269.bitrate = (uint)304071867U;
            p269.rotation = (ushort)(ushort)7320;
            p269.uri_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p269); //===============================
            SET_VIDEO_STREAM_SETTINGS p270 = LoopBackDemoChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.target_system = (byte)(byte)66;
            p270.target_component = (byte)(byte)25;
            p270.camera_id = (byte)(byte)80;
            p270.framerate = (float)3.2394255E38F;
            p270.resolution_h = (ushort)(ushort)12857;
            p270.resolution_v = (ushort)(ushort)63031;
            p270.bitrate = (uint)551585626U;
            p270.rotation = (ushort)(ushort)44230;
            p270.uri_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p270); //===============================
            WIFI_CONFIG_AP p299 = LoopBackDemoChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.ssid_SET("DEMO", PH);
            p299.password_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p299); //===============================
            PROTOCOL_VERSION p300 = LoopBackDemoChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.version = (ushort)(ushort)15166;
            p300.min_version = (ushort)(ushort)49151;
            p300.max_version = (ushort)(ushort)60567;
            p300.spec_version_hash_SET(new byte[8], 0);
            p300.library_version_hash_SET(new byte[8], 0);
            LoopBackDemoChannel.instance.send(p300); //===============================
            UAVCAN_NODE_STATUS p310 = LoopBackDemoChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.time_usec = (ulong)8714480302180750076L;
            p310.uptime_sec = (uint)2890752127U;
            p310.health = (UAVCAN_NODE_HEALTH)UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK;
            p310.mode = (UAVCAN_NODE_MODE)UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE;
            p310.sub_mode = (byte)(byte)137;
            p310.vendor_specific_status_code = (ushort)(ushort)48009;
            LoopBackDemoChannel.instance.send(p310); //===============================
            UAVCAN_NODE_INFO p311 = LoopBackDemoChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.time_usec = (ulong)4524822153138538985L;
            p311.uptime_sec = (uint)1046622150U;
            p311.name_SET("DEMO", PH);
            p311.hw_version_major = (byte)(byte)89;
            p311.hw_version_minor = (byte)(byte)189;
            p311.hw_unique_id_SET(new byte[16], 0);
            p311.sw_version_major = (byte)(byte)98;
            p311.sw_version_minor = (byte)(byte)14;
            p311.sw_vcs_commit = (uint)949390578U;
            LoopBackDemoChannel.instance.send(p311); //===============================
            PARAM_EXT_REQUEST_READ p320 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.target_system = (byte)(byte)28;
            p320.target_component = (byte)(byte)211;
            p320.param_id_SET("DEMO", PH);
            p320.param_index = (short)(short)26983;
            LoopBackDemoChannel.instance.send(p320); //===============================
            PARAM_EXT_REQUEST_LIST p321 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_system = (byte)(byte)23;
            p321.target_component = (byte)(byte)198;
            LoopBackDemoChannel.instance.send(p321); //===============================
            PARAM_EXT_VALUE p322 = LoopBackDemoChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_id_SET("DEMO", PH);
            p322.param_value_SET("DEMO", PH);
            p322.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM;
            p322.param_count = (ushort)(ushort)31789;
            p322.param_index = (ushort)(ushort)28497;
            LoopBackDemoChannel.instance.send(p322); //===============================
            PARAM_EXT_SET p323 = LoopBackDemoChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.target_system = (byte)(byte)1;
            p323.target_component = (byte)(byte)68;
            p323.param_id_SET("DEMO", PH);
            p323.param_value_SET("DEMO", PH);
            p323.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64;
            LoopBackDemoChannel.instance.send(p323); //===============================
            PARAM_EXT_ACK p324 = LoopBackDemoChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_id_SET("DEMO", PH);
            p324.param_value_SET("DEMO", PH);
            p324.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64;
            p324.param_result = (PARAM_ACK)PARAM_ACK.PARAM_ACK_IN_PROGRESS;
            LoopBackDemoChannel.instance.send(p324); //===============================
            OBSTACLE_DISTANCE p330 = LoopBackDemoChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.time_usec = (ulong)3877737012785316692L;
            p330.sensor_type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND;
            p330.distances_SET(new ushort[72], 0);
            p330.increment = (byte)(byte)181;
            p330.min_distance = (ushort)(ushort)21406;
            p330.max_distance = (ushort)(ushort)55156;
            LoopBackDemoChannel.instance.send(p330); //===============================
        }
    }
}

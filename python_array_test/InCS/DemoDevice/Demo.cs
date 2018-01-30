
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
            LoopBackDemoChannel.instance.OnARRAY_TEST_0Receive += (src, ph, pack) =>
            {
                byte v1 = pack.v1;
                sbyte[] ar_i8 = pack.ar_i8;
                byte[] ar_u8 = pack.ar_u8;
                ushort[] ar_u16 = pack.ar_u16;
                uint[] ar_u32 = pack.ar_u32;
            };
            LoopBackDemoChannel.instance.OnARRAY_TEST_1Receive += (src, ph, pack) =>
            {
                uint[] ar_u32 = pack.ar_u32;
            };
            LoopBackDemoChannel.instance.OnARRAY_TEST_3Receive += (src, ph, pack) =>
            {
                byte v = pack.v;
                uint[] ar_u32 = pack.ar_u32;
            };
            LoopBackDemoChannel.instance.OnARRAY_TEST_4Receive += (src, ph, pack) =>
            {
                uint[] ar_u32 = pack.ar_u32;
                byte v = pack.v;
            };
            LoopBackDemoChannel.instance.OnARRAY_TEST_5Receive += (src, ph, pack) =>
            {
                string c1 = pack.c1_TRY(ph);
                string c2 = pack.c2_TRY(ph);
            };
            LoopBackDemoChannel.instance.OnARRAY_TEST_6Receive += (src, ph, pack) =>
            {
                byte v1 = pack.v1;
                ushort v2 = pack.v2;
                uint v3 = pack.v3;
                uint[] ar_u32 = pack.ar_u32;
                int[] ar_i32 = pack.ar_i32;
                ushort[] ar_u16 = pack.ar_u16;
                short[] ar_i16 = pack.ar_i16;
                byte[] ar_u8 = pack.ar_u8;
                sbyte[] ar_i8 = pack.ar_i8;
                string ar_c = pack.ar_c_TRY(ph);
                double[] ar_d = pack.ar_d;
                float[] ar_f = pack.ar_f;
            };
            LoopBackDemoChannel.instance.OnARRAY_TEST_7Receive += (src, ph, pack) =>
            {
                double[] ar_d = pack.ar_d;
                float[] ar_f = pack.ar_f;
                uint[] ar_u32 = pack.ar_u32;
                int[] ar_i32 = pack.ar_i32;
                ushort[] ar_u16 = pack.ar_u16;
                short[] ar_i16 = pack.ar_i16;
                byte[] ar_u8 = pack.ar_u8;
                sbyte[] ar_i8 = pack.ar_i8;
                string ar_c = pack.ar_c_TRY(ph);
            };
            LoopBackDemoChannel.instance.OnARRAY_TEST_8Receive += (src, ph, pack) =>
            {
                uint v3 = pack.v3;
                double[] ar_d = pack.ar_d;
                ushort[] ar_u16 = pack.ar_u16;
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
            p0.type = (MAV_TYPE)MAV_TYPE.MAV_TYPE_VTOL_TILTROTOR;
            p0.autopilot = (MAV_AUTOPILOT)MAV_AUTOPILOT.MAV_AUTOPILOT_AUTOQUAD;
            p0.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
            p0.custom_mode = (uint)2137898007U;
            p0.system_status = (MAV_STATE)MAV_STATE.MAV_STATE_STANDBY;
            p0.mavlink_version = (byte)(byte)53;
            LoopBackDemoChannel.instance.send(p0); //===============================
            SYS_STATUS p1 = LoopBackDemoChannel.new_SYS_STATUS();
            PH.setPack(p1);
            p1.onboard_control_sensors_present = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION;
            p1.onboard_control_sensors_enabled = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION;
            p1.onboard_control_sensors_health = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
            p1.load = (ushort)(ushort)37667;
            p1.voltage_battery = (ushort)(ushort)34540;
            p1.current_battery = (short)(short) -6138;
            p1.battery_remaining = (sbyte)(sbyte) - 24;
            p1.drop_rate_comm = (ushort)(ushort)40826;
            p1.errors_comm = (ushort)(ushort)43663;
            p1.errors_count1 = (ushort)(ushort)16773;
            p1.errors_count2 = (ushort)(ushort)43013;
            p1.errors_count3 = (ushort)(ushort)16392;
            p1.errors_count4 = (ushort)(ushort)8905;
            LoopBackDemoChannel.instance.send(p1); //===============================
            SYSTEM_TIME p2 = LoopBackDemoChannel.new_SYSTEM_TIME();
            PH.setPack(p2);
            p2.time_unix_usec = (ulong)8833981039093093622L;
            p2.time_boot_ms = (uint)2294890588U;
            LoopBackDemoChannel.instance.send(p2); //===============================
            POSITION_TARGET_LOCAL_NED p3 = LoopBackDemoChannel.new_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.time_boot_ms = (uint)3485081716U;
            p3.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p3.type_mask = (ushort)(ushort)59455;
            p3.x = (float)2.1238437E38F;
            p3.y = (float)2.6879752E38F;
            p3.z = (float) -2.7438027E38F;
            p3.vx = (float) -9.274309E36F;
            p3.vy = (float) -2.0632573E38F;
            p3.vz = (float) -1.6399835E38F;
            p3.afx = (float) -3.3932471E38F;
            p3.afy = (float)1.211508E38F;
            p3.afz = (float)2.4811484E38F;
            p3.yaw = (float) -1.6671258E38F;
            p3.yaw_rate = (float) -3.3044396E37F;
            LoopBackDemoChannel.instance.send(p3); //===============================
            PING p4 = LoopBackDemoChannel.new_PING();
            PH.setPack(p4);
            p4.time_usec = (ulong)3261980944899793386L;
            p4.seq = (uint)909296445U;
            p4.target_system = (byte)(byte)66;
            p4.target_component = (byte)(byte)102;
            LoopBackDemoChannel.instance.send(p4); //===============================
            CHANGE_OPERATOR_CONTROL p5 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL();
            PH.setPack(p5);
            p5.target_system = (byte)(byte)23;
            p5.control_request = (byte)(byte)22;
            p5.version = (byte)(byte)46;
            p5.passkey_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p5); //===============================
            CHANGE_OPERATOR_CONTROL_ACK p6 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL_ACK();
            PH.setPack(p6);
            p6.gcs_system_id = (byte)(byte)177;
            p6.control_request = (byte)(byte)46;
            p6.ack = (byte)(byte)24;
            LoopBackDemoChannel.instance.send(p6); //===============================
            AUTH_KEY p7 = LoopBackDemoChannel.new_AUTH_KEY();
            PH.setPack(p7);
            p7.key_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p7); //===============================
            SET_MODE p11 = LoopBackDemoChannel.new_SET_MODE();
            PH.setPack(p11);
            p11.target_system = (byte)(byte)51;
            p11.base_mode = (MAV_MODE)MAV_MODE.MAV_MODE_TEST_ARMED;
            p11.custom_mode = (uint)1952708532U;
            LoopBackDemoChannel.instance.send(p11); //===============================
            PARAM_REQUEST_READ p20 = LoopBackDemoChannel.new_PARAM_REQUEST_READ();
            PH.setPack(p20);
            p20.target_system = (byte)(byte)193;
            p20.target_component = (byte)(byte)174;
            p20.param_id_SET("DEMO", PH);
            p20.param_index = (short)(short)4506;
            LoopBackDemoChannel.instance.send(p20); //===============================
            PARAM_REQUEST_LIST p21 = LoopBackDemoChannel.new_PARAM_REQUEST_LIST();
            PH.setPack(p21);
            p21.target_system = (byte)(byte)59;
            p21.target_component = (byte)(byte)64;
            LoopBackDemoChannel.instance.send(p21); //===============================
            PARAM_VALUE p22 = LoopBackDemoChannel.new_PARAM_VALUE();
            PH.setPack(p22);
            p22.param_id_SET("DEMO", PH);
            p22.param_value = (float)4.3872753E37F;
            p22.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT64;
            p22.param_count = (ushort)(ushort)7991;
            p22.param_index = (ushort)(ushort)1808;
            LoopBackDemoChannel.instance.send(p22); //===============================
            PARAM_SET p23 = LoopBackDemoChannel.new_PARAM_SET();
            PH.setPack(p23);
            p23.target_system = (byte)(byte)23;
            p23.target_component = (byte)(byte)44;
            p23.param_id_SET("DEMO", PH);
            p23.param_value = (float)1.8723599E38F;
            p23.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT16;
            LoopBackDemoChannel.instance.send(p23); //===============================
            GPS_RAW_INT p24 = LoopBackDemoChannel.new_GPS_RAW_INT();
            PH.setPack(p24);
            p24.time_usec = (ulong)2793918747809470430L;
            p24.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS;
            p24.lat = (int) -1028076530;
            p24.lon = (int)183020633;
            p24.alt = (int) -377612301;
            p24.eph = (ushort)(ushort)44405;
            p24.epv = (ushort)(ushort)40517;
            p24.vel = (ushort)(ushort)63429;
            p24.cog = (ushort)(ushort)32414;
            p24.satellites_visible = (byte)(byte)253;
            p24.alt_ellipsoid_SET((int)2103477892, PH);
            p24.h_acc_SET((uint)3732237798U, PH);
            p24.v_acc_SET((uint)72051629U, PH);
            p24.vel_acc_SET((uint)3272520379U, PH);
            p24.hdg_acc_SET((uint)1698620692U, PH);
            LoopBackDemoChannel.instance.send(p24); //===============================
            GPS_STATUS p25 = LoopBackDemoChannel.new_GPS_STATUS();
            PH.setPack(p25);
            p25.satellites_visible = (byte)(byte)1;
            p25.satellite_prn_SET(new byte[20], 0);
            p25.satellite_used_SET(new byte[20], 0);
            p25.satellite_elevation_SET(new byte[20], 0);
            p25.satellite_azimuth_SET(new byte[20], 0);
            p25.satellite_snr_SET(new byte[20], 0);
            LoopBackDemoChannel.instance.send(p25); //===============================
            SCALED_IMU p26 = LoopBackDemoChannel.new_SCALED_IMU();
            PH.setPack(p26);
            p26.time_boot_ms = (uint)785693381U;
            p26.xacc = (short)(short) -13570;
            p26.yacc = (short)(short) -4035;
            p26.zacc = (short)(short)20205;
            p26.xgyro = (short)(short) -2185;
            p26.ygyro = (short)(short) -9062;
            p26.zgyro = (short)(short) -16959;
            p26.xmag = (short)(short)18555;
            p26.ymag = (short)(short)29837;
            p26.zmag = (short)(short)14524;
            LoopBackDemoChannel.instance.send(p26); //===============================
            RAW_IMU p27 = LoopBackDemoChannel.new_RAW_IMU();
            PH.setPack(p27);
            p27.time_usec = (ulong)1826765027799910998L;
            p27.xacc = (short)(short)12979;
            p27.yacc = (short)(short)6573;
            p27.zacc = (short)(short) -32284;
            p27.xgyro = (short)(short)218;
            p27.ygyro = (short)(short) -18226;
            p27.zgyro = (short)(short) -15374;
            p27.xmag = (short)(short)2383;
            p27.ymag = (short)(short)785;
            p27.zmag = (short)(short) -14670;
            LoopBackDemoChannel.instance.send(p27); //===============================
            RAW_PRESSURE p28 = LoopBackDemoChannel.new_RAW_PRESSURE();
            PH.setPack(p28);
            p28.time_usec = (ulong)5707570264854048654L;
            p28.press_abs = (short)(short)28583;
            p28.press_diff1 = (short)(short)27677;
            p28.press_diff2 = (short)(short)13930;
            p28.temperature = (short)(short) -9266;
            LoopBackDemoChannel.instance.send(p28); //===============================
            SCALED_PRESSURE p29 = LoopBackDemoChannel.new_SCALED_PRESSURE();
            PH.setPack(p29);
            p29.time_boot_ms = (uint)3137368841U;
            p29.press_abs = (float) -2.7385573E38F;
            p29.press_diff = (float)3.204127E38F;
            p29.temperature = (short)(short) -30660;
            LoopBackDemoChannel.instance.send(p29); //===============================
            ATTITUDE p30 = LoopBackDemoChannel.new_ATTITUDE();
            PH.setPack(p30);
            p30.time_boot_ms = (uint)3420581128U;
            p30.roll = (float) -1.5491047E38F;
            p30.pitch = (float)3.1647784E38F;
            p30.yaw = (float) -1.3171756E38F;
            p30.rollspeed = (float)1.7396237E38F;
            p30.pitchspeed = (float) -1.8679834E38F;
            p30.yawspeed = (float) -1.8574752E38F;
            LoopBackDemoChannel.instance.send(p30); //===============================
            ATTITUDE_QUATERNION p31 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION();
            PH.setPack(p31);
            p31.time_boot_ms = (uint)2415410833U;
            p31.q1 = (float)2.2662735E38F;
            p31.q2 = (float) -3.085111E38F;
            p31.q3 = (float)4.6705122E36F;
            p31.q4 = (float)6.3114417E37F;
            p31.rollspeed = (float)1.2575399E38F;
            p31.pitchspeed = (float) -1.7032855E38F;
            p31.yawspeed = (float) -2.0438213E38F;
            LoopBackDemoChannel.instance.send(p31); //===============================
            LOCAL_POSITION_NED p32 = LoopBackDemoChannel.new_LOCAL_POSITION_NED();
            PH.setPack(p32);
            p32.time_boot_ms = (uint)4185684547U;
            p32.x = (float) -2.5052818E38F;
            p32.y = (float) -2.8624025E38F;
            p32.z = (float)2.8252658E38F;
            p32.vx = (float) -1.0958091E38F;
            p32.vy = (float)1.712999E38F;
            p32.vz = (float)1.8424926E37F;
            LoopBackDemoChannel.instance.send(p32); //===============================
            GLOBAL_POSITION_INT p33 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT();
            PH.setPack(p33);
            p33.time_boot_ms = (uint)2789374176U;
            p33.lat = (int)1316917205;
            p33.lon = (int) -143991165;
            p33.alt = (int) -2082218156;
            p33.relative_alt = (int)541540041;
            p33.vx = (short)(short) -13316;
            p33.vy = (short)(short)4475;
            p33.vz = (short)(short) -3338;
            p33.hdg = (ushort)(ushort)16484;
            LoopBackDemoChannel.instance.send(p33); //===============================
            RC_CHANNELS_SCALED p34 = LoopBackDemoChannel.new_RC_CHANNELS_SCALED();
            PH.setPack(p34);
            p34.time_boot_ms = (uint)13782723U;
            p34.port = (byte)(byte)82;
            p34.chan1_scaled = (short)(short) -13587;
            p34.chan2_scaled = (short)(short) -19163;
            p34.chan3_scaled = (short)(short) -13341;
            p34.chan4_scaled = (short)(short) -31700;
            p34.chan5_scaled = (short)(short) -19978;
            p34.chan6_scaled = (short)(short)4706;
            p34.chan7_scaled = (short)(short) -30955;
            p34.chan8_scaled = (short)(short)12960;
            p34.rssi = (byte)(byte)103;
            LoopBackDemoChannel.instance.send(p34); //===============================
            RC_CHANNELS_RAW p35 = LoopBackDemoChannel.new_RC_CHANNELS_RAW();
            PH.setPack(p35);
            p35.time_boot_ms = (uint)3288862226U;
            p35.port = (byte)(byte)69;
            p35.chan1_raw = (ushort)(ushort)47866;
            p35.chan2_raw = (ushort)(ushort)52617;
            p35.chan3_raw = (ushort)(ushort)27618;
            p35.chan4_raw = (ushort)(ushort)47146;
            p35.chan5_raw = (ushort)(ushort)28899;
            p35.chan6_raw = (ushort)(ushort)43143;
            p35.chan7_raw = (ushort)(ushort)42659;
            p35.chan8_raw = (ushort)(ushort)43126;
            p35.rssi = (byte)(byte)71;
            LoopBackDemoChannel.instance.send(p35); //===============================
            SERVO_OUTPUT_RAW p36 = LoopBackDemoChannel.new_SERVO_OUTPUT_RAW();
            PH.setPack(p36);
            p36.time_usec = (uint)2407865851U;
            p36.port = (byte)(byte)251;
            p36.servo1_raw = (ushort)(ushort)40999;
            p36.servo2_raw = (ushort)(ushort)6843;
            p36.servo3_raw = (ushort)(ushort)60899;
            p36.servo4_raw = (ushort)(ushort)20542;
            p36.servo5_raw = (ushort)(ushort)14616;
            p36.servo6_raw = (ushort)(ushort)53725;
            p36.servo7_raw = (ushort)(ushort)4327;
            p36.servo8_raw = (ushort)(ushort)60692;
            p36.servo9_raw_SET((ushort)(ushort)32275, PH);
            p36.servo10_raw_SET((ushort)(ushort)56637, PH);
            p36.servo11_raw_SET((ushort)(ushort)42742, PH);
            p36.servo12_raw_SET((ushort)(ushort)27705, PH);
            p36.servo13_raw_SET((ushort)(ushort)28897, PH);
            p36.servo14_raw_SET((ushort)(ushort)19031, PH);
            p36.servo15_raw_SET((ushort)(ushort)32059, PH);
            p36.servo16_raw_SET((ushort)(ushort)52076, PH);
            LoopBackDemoChannel.instance.send(p36); //===============================
            MISSION_REQUEST_PARTIAL_LIST p37 = LoopBackDemoChannel.new_MISSION_REQUEST_PARTIAL_LIST();
            PH.setPack(p37);
            p37.target_system = (byte)(byte)190;
            p37.target_component = (byte)(byte)252;
            p37.start_index = (short)(short)1305;
            p37.end_index = (short)(short) -8047;
            p37.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            LoopBackDemoChannel.instance.send(p37); //===============================
            MISSION_WRITE_PARTIAL_LIST p38 = LoopBackDemoChannel.new_MISSION_WRITE_PARTIAL_LIST();
            PH.setPack(p38);
            p38.target_system = (byte)(byte)100;
            p38.target_component = (byte)(byte)36;
            p38.start_index = (short)(short)19834;
            p38.end_index = (short)(short)24762;
            p38.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            LoopBackDemoChannel.instance.send(p38); //===============================
            MISSION_ITEM p39 = LoopBackDemoChannel.new_MISSION_ITEM();
            PH.setPack(p39);
            p39.target_system = (byte)(byte)197;
            p39.target_component = (byte)(byte)242;
            p39.seq = (ushort)(ushort)2760;
            p39.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_NED;
            p39.command = (MAV_CMD)MAV_CMD.MAV_CMD_DO_GO_AROUND;
            p39.current = (byte)(byte)163;
            p39.autocontinue = (byte)(byte)87;
            p39.param1 = (float) -2.5064858E38F;
            p39.param2 = (float) -1.189387E38F;
            p39.param3 = (float) -4.1812999E37F;
            p39.param4 = (float)2.9064214E38F;
            p39.x = (float) -1.3737119E38F;
            p39.y = (float)2.4177087E38F;
            p39.z = (float) -1.3391118E38F;
            p39.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            LoopBackDemoChannel.instance.send(p39); //===============================
            MISSION_REQUEST p40 = LoopBackDemoChannel.new_MISSION_REQUEST();
            PH.setPack(p40);
            p40.target_system = (byte)(byte)115;
            p40.target_component = (byte)(byte)6;
            p40.seq = (ushort)(ushort)58420;
            p40.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            LoopBackDemoChannel.instance.send(p40); //===============================
            MISSION_SET_CURRENT p41 = LoopBackDemoChannel.new_MISSION_SET_CURRENT();
            PH.setPack(p41);
            p41.target_system = (byte)(byte)64;
            p41.target_component = (byte)(byte)68;
            p41.seq = (ushort)(ushort)59471;
            LoopBackDemoChannel.instance.send(p41); //===============================
            MISSION_CURRENT p42 = LoopBackDemoChannel.new_MISSION_CURRENT();
            PH.setPack(p42);
            p42.seq = (ushort)(ushort)49710;
            LoopBackDemoChannel.instance.send(p42); //===============================
            MISSION_REQUEST_LIST p43 = LoopBackDemoChannel.new_MISSION_REQUEST_LIST();
            PH.setPack(p43);
            p43.target_system = (byte)(byte)235;
            p43.target_component = (byte)(byte)10;
            p43.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            LoopBackDemoChannel.instance.send(p43); //===============================
            MISSION_COUNT p44 = LoopBackDemoChannel.new_MISSION_COUNT();
            PH.setPack(p44);
            p44.target_system = (byte)(byte)117;
            p44.target_component = (byte)(byte)71;
            p44.count = (ushort)(ushort)9625;
            p44.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            LoopBackDemoChannel.instance.send(p44); //===============================
            MISSION_CLEAR_ALL p45 = LoopBackDemoChannel.new_MISSION_CLEAR_ALL();
            PH.setPack(p45);
            p45.target_system = (byte)(byte)188;
            p45.target_component = (byte)(byte)134;
            p45.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            LoopBackDemoChannel.instance.send(p45); //===============================
            MISSION_ITEM_REACHED p46 = LoopBackDemoChannel.new_MISSION_ITEM_REACHED();
            PH.setPack(p46);
            p46.seq = (ushort)(ushort)58702;
            LoopBackDemoChannel.instance.send(p46); //===============================
            MISSION_ACK p47 = LoopBackDemoChannel.new_MISSION_ACK();
            PH.setPack(p47);
            p47.target_system = (byte)(byte)146;
            p47.target_component = (byte)(byte)206;
            p47.type = (MAV_MISSION_RESULT)MAV_MISSION_RESULT.MAV_MISSION_UNSUPPORTED_FRAME;
            p47.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            LoopBackDemoChannel.instance.send(p47); //===============================
            SET_GPS_GLOBAL_ORIGIN p48 = LoopBackDemoChannel.new_SET_GPS_GLOBAL_ORIGIN();
            PH.setPack(p48);
            p48.target_system = (byte)(byte)128;
            p48.latitude = (int)954788297;
            p48.longitude = (int) -368719486;
            p48.altitude = (int)584813229;
            p48.time_usec_SET((ulong)7429967600658468577L, PH);
            LoopBackDemoChannel.instance.send(p48); //===============================
            GPS_GLOBAL_ORIGIN p49 = LoopBackDemoChannel.new_GPS_GLOBAL_ORIGIN();
            PH.setPack(p49);
            p49.latitude = (int)2029636903;
            p49.longitude = (int)424331156;
            p49.altitude = (int) -533342940;
            p49.time_usec_SET((ulong)6256217046286512932L, PH);
            LoopBackDemoChannel.instance.send(p49); //===============================
            PARAM_MAP_RC p50 = LoopBackDemoChannel.new_PARAM_MAP_RC();
            PH.setPack(p50);
            p50.target_system = (byte)(byte)48;
            p50.target_component = (byte)(byte)106;
            p50.param_id_SET("DEMO", PH);
            p50.param_index = (short)(short) -23625;
            p50.parameter_rc_channel_index = (byte)(byte)158;
            p50.param_value0 = (float) -2.8476128E38F;
            p50.scale = (float) -1.3119095E38F;
            p50.param_value_min = (float)2.8736651E38F;
            p50.param_value_max = (float)7.899815E36F;
            LoopBackDemoChannel.instance.send(p50); //===============================
            MISSION_REQUEST_INT p51 = LoopBackDemoChannel.new_MISSION_REQUEST_INT();
            PH.setPack(p51);
            p51.target_system = (byte)(byte)59;
            p51.target_component = (byte)(byte)204;
            p51.seq = (ushort)(ushort)5667;
            p51.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            LoopBackDemoChannel.instance.send(p51); //===============================
            SAFETY_SET_ALLOWED_AREA p54 = LoopBackDemoChannel.new_SAFETY_SET_ALLOWED_AREA();
            PH.setPack(p54);
            p54.target_system = (byte)(byte)14;
            p54.target_component = (byte)(byte)210;
            p54.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL;
            p54.p1x = (float)3.9269825E37F;
            p54.p1y = (float)8.0715203E37F;
            p54.p1z = (float)1.501743E36F;
            p54.p2x = (float) -3.3147836E38F;
            p54.p2y = (float)3.276265E38F;
            p54.p2z = (float)3.2989865E38F;
            LoopBackDemoChannel.instance.send(p54); //===============================
            SAFETY_ALLOWED_AREA p55 = LoopBackDemoChannel.new_SAFETY_ALLOWED_AREA();
            PH.setPack(p55);
            p55.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
            p55.p1x = (float)4.628138E37F;
            p55.p1y = (float) -1.7961118E38F;
            p55.p1z = (float) -1.3307768E38F;
            p55.p2x = (float) -2.8178122E38F;
            p55.p2y = (float) -1.351857E38F;
            p55.p2z = (float) -1.3809684E38F;
            LoopBackDemoChannel.instance.send(p55); //===============================
            ATTITUDE_QUATERNION_COV p61 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION_COV();
            PH.setPack(p61);
            p61.time_usec = (ulong)6431107681249442060L;
            p61.q_SET(new float[4], 0);
            p61.rollspeed = (float)2.5041115E38F;
            p61.pitchspeed = (float) -1.5418612E38F;
            p61.yawspeed = (float) -9.084416E37F;
            p61.covariance_SET(new float[9], 0);
            LoopBackDemoChannel.instance.send(p61); //===============================
            NAV_CONTROLLER_OUTPUT p62 = LoopBackDemoChannel.new_NAV_CONTROLLER_OUTPUT();
            PH.setPack(p62);
            p62.nav_roll = (float) -5.809596E37F;
            p62.nav_pitch = (float)2.5877172E38F;
            p62.nav_bearing = (short)(short)14324;
            p62.target_bearing = (short)(short) -1594;
            p62.wp_dist = (ushort)(ushort)20244;
            p62.alt_error = (float)1.6822532E38F;
            p62.aspd_error = (float) -2.0453924E38F;
            p62.xtrack_error = (float)2.8723768E38F;
            LoopBackDemoChannel.instance.send(p62); //===============================
            GLOBAL_POSITION_INT_COV p63 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT_COV();
            PH.setPack(p63);
            p63.time_usec = (ulong)3247509435612276652L;
            p63.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE;
            p63.lat = (int)842647298;
            p63.lon = (int) -1721130944;
            p63.alt = (int) -83903235;
            p63.relative_alt = (int) -194650393;
            p63.vx = (float) -1.0765572E38F;
            p63.vy = (float)2.3338494E38F;
            p63.vz = (float) -2.7317315E38F;
            p63.covariance_SET(new float[36], 0);
            LoopBackDemoChannel.instance.send(p63); //===============================
            LOCAL_POSITION_NED_COV p64 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_COV();
            PH.setPack(p64);
            p64.time_usec = (ulong)3709123618634876887L;
            p64.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO;
            p64.x = (float) -3.1145934E38F;
            p64.y = (float)3.1075085E38F;
            p64.z = (float) -7.757382E37F;
            p64.vx = (float) -1.1711806E38F;
            p64.vy = (float)1.0745743E38F;
            p64.vz = (float) -3.3171051E38F;
            p64.ax = (float) -2.0649714E38F;
            p64.ay = (float) -4.266946E37F;
            p64.az = (float)1.7885925E38F;
            p64.covariance_SET(new float[45], 0);
            LoopBackDemoChannel.instance.send(p64); //===============================
            RC_CHANNELS p65 = LoopBackDemoChannel.new_RC_CHANNELS();
            PH.setPack(p65);
            p65.time_boot_ms = (uint)2543637361U;
            p65.chancount = (byte)(byte)126;
            p65.chan1_raw = (ushort)(ushort)58613;
            p65.chan2_raw = (ushort)(ushort)23896;
            p65.chan3_raw = (ushort)(ushort)39102;
            p65.chan4_raw = (ushort)(ushort)8706;
            p65.chan5_raw = (ushort)(ushort)48288;
            p65.chan6_raw = (ushort)(ushort)13889;
            p65.chan7_raw = (ushort)(ushort)963;
            p65.chan8_raw = (ushort)(ushort)55770;
            p65.chan9_raw = (ushort)(ushort)910;
            p65.chan10_raw = (ushort)(ushort)37929;
            p65.chan11_raw = (ushort)(ushort)36057;
            p65.chan12_raw = (ushort)(ushort)2107;
            p65.chan13_raw = (ushort)(ushort)49408;
            p65.chan14_raw = (ushort)(ushort)14111;
            p65.chan15_raw = (ushort)(ushort)48628;
            p65.chan16_raw = (ushort)(ushort)43136;
            p65.chan17_raw = (ushort)(ushort)3349;
            p65.chan18_raw = (ushort)(ushort)62302;
            p65.rssi = (byte)(byte)165;
            LoopBackDemoChannel.instance.send(p65); //===============================
            REQUEST_DATA_STREAM p66 = LoopBackDemoChannel.new_REQUEST_DATA_STREAM();
            PH.setPack(p66);
            p66.target_system = (byte)(byte)145;
            p66.target_component = (byte)(byte)38;
            p66.req_stream_id = (byte)(byte)112;
            p66.req_message_rate = (ushort)(ushort)38274;
            p66.start_stop = (byte)(byte)181;
            LoopBackDemoChannel.instance.send(p66); //===============================
            DATA_STREAM p67 = LoopBackDemoChannel.new_DATA_STREAM();
            PH.setPack(p67);
            p67.stream_id = (byte)(byte)202;
            p67.message_rate = (ushort)(ushort)40928;
            p67.on_off = (byte)(byte)159;
            LoopBackDemoChannel.instance.send(p67); //===============================
            MANUAL_CONTROL p69 = LoopBackDemoChannel.new_MANUAL_CONTROL();
            PH.setPack(p69);
            p69.target = (byte)(byte)221;
            p69.x = (short)(short)6768;
            p69.y = (short)(short) -32762;
            p69.z = (short)(short) -23433;
            p69.r = (short)(short) -31607;
            p69.buttons = (ushort)(ushort)26691;
            LoopBackDemoChannel.instance.send(p69); //===============================
            RC_CHANNELS_OVERRIDE p70 = LoopBackDemoChannel.new_RC_CHANNELS_OVERRIDE();
            PH.setPack(p70);
            p70.target_system = (byte)(byte)79;
            p70.target_component = (byte)(byte)77;
            p70.chan1_raw = (ushort)(ushort)54079;
            p70.chan2_raw = (ushort)(ushort)61864;
            p70.chan3_raw = (ushort)(ushort)40458;
            p70.chan4_raw = (ushort)(ushort)4300;
            p70.chan5_raw = (ushort)(ushort)63288;
            p70.chan6_raw = (ushort)(ushort)47448;
            p70.chan7_raw = (ushort)(ushort)28033;
            p70.chan8_raw = (ushort)(ushort)17658;
            LoopBackDemoChannel.instance.send(p70); //===============================
            MISSION_ITEM_INT p73 = LoopBackDemoChannel.new_MISSION_ITEM_INT();
            PH.setPack(p73);
            p73.target_system = (byte)(byte)134;
            p73.target_component = (byte)(byte)184;
            p73.seq = (ushort)(ushort)10647;
            p73.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_MISSION;
            p73.command = (MAV_CMD)MAV_CMD.MAV_CMD_NAV_LAND;
            p73.current = (byte)(byte)171;
            p73.autocontinue = (byte)(byte)225;
            p73.param1 = (float)1.9016288E38F;
            p73.param2 = (float)2.8170523E38F;
            p73.param3 = (float) -6.694039E37F;
            p73.param4 = (float) -3.3270273E37F;
            p73.x = (int)249744262;
            p73.y = (int) -1531820255;
            p73.z = (float)2.2756874E38F;
            p73.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            LoopBackDemoChannel.instance.send(p73); //===============================
            VFR_HUD p74 = LoopBackDemoChannel.new_VFR_HUD();
            PH.setPack(p74);
            p74.airspeed = (float)3.460649E37F;
            p74.groundspeed = (float) -2.1631374E38F;
            p74.heading = (short)(short) -20112;
            p74.throttle = (ushort)(ushort)23508;
            p74.alt = (float) -1.5169375E37F;
            p74.climb = (float) -1.0574994E38F;
            LoopBackDemoChannel.instance.send(p74); //===============================
            COMMAND_INT p75 = LoopBackDemoChannel.new_COMMAND_INT();
            PH.setPack(p75);
            p75.target_system = (byte)(byte)85;
            p75.target_component = (byte)(byte)49;
            p75.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_NED;
            p75.command = (MAV_CMD)MAV_CMD.MAV_CMD_USER_3;
            p75.current = (byte)(byte)200;
            p75.autocontinue = (byte)(byte)177;
            p75.param1 = (float) -1.0113815E38F;
            p75.param2 = (float) -1.5215263E38F;
            p75.param3 = (float) -3.0853E38F;
            p75.param4 = (float)3.1252603E38F;
            p75.x = (int) -1652322729;
            p75.y = (int)753398164;
            p75.z = (float)6.964551E37F;
            LoopBackDemoChannel.instance.send(p75); //===============================
            COMMAND_LONG p76 = LoopBackDemoChannel.new_COMMAND_LONG();
            PH.setPack(p76);
            p76.target_system = (byte)(byte)14;
            p76.target_component = (byte)(byte)41;
            p76.command = (MAV_CMD)MAV_CMD.MAV_CMD_NAV_GUIDED_ENABLE;
            p76.confirmation = (byte)(byte)96;
            p76.param1 = (float)2.9193075E38F;
            p76.param2 = (float) -2.352059E38F;
            p76.param3 = (float) -4.0791665E37F;
            p76.param4 = (float) -9.530082E37F;
            p76.param5 = (float)1.8557338E36F;
            p76.param6 = (float)1.7413619E38F;
            p76.param7 = (float) -3.9811153E37F;
            LoopBackDemoChannel.instance.send(p76); //===============================
            COMMAND_ACK p77 = LoopBackDemoChannel.new_COMMAND_ACK();
            PH.setPack(p77);
            p77.command = (MAV_CMD)MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE;
            p77.result = (MAV_RESULT)MAV_RESULT.MAV_RESULT_UNSUPPORTED;
            p77.progress_SET((byte)(byte)169, PH);
            p77.result_param2_SET((int)2035835218, PH);
            p77.target_system_SET((byte)(byte)101, PH);
            p77.target_component_SET((byte)(byte)240, PH);
            LoopBackDemoChannel.instance.send(p77); //===============================
            MANUAL_SETPOINT p81 = LoopBackDemoChannel.new_MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.time_boot_ms = (uint)1842594320U;
            p81.roll = (float)7.841526E37F;
            p81.pitch = (float)3.322159E38F;
            p81.yaw = (float) -2.7330312E38F;
            p81.thrust = (float)8.651667E37F;
            p81.mode_switch = (byte)(byte)86;
            p81.manual_override_switch = (byte)(byte)177;
            LoopBackDemoChannel.instance.send(p81); //===============================
            SET_ATTITUDE_TARGET p82 = LoopBackDemoChannel.new_SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.time_boot_ms = (uint)1308256613U;
            p82.target_system = (byte)(byte)76;
            p82.target_component = (byte)(byte)159;
            p82.type_mask = (byte)(byte)65;
            p82.q_SET(new float[4], 0);
            p82.body_roll_rate = (float)3.1149792E38F;
            p82.body_pitch_rate = (float)4.698672E37F;
            p82.body_yaw_rate = (float)1.4601817E38F;
            p82.thrust = (float) -2.6367775E38F;
            LoopBackDemoChannel.instance.send(p82); //===============================
            ATTITUDE_TARGET p83 = LoopBackDemoChannel.new_ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.time_boot_ms = (uint)4217550637U;
            p83.type_mask = (byte)(byte)206;
            p83.q_SET(new float[4], 0);
            p83.body_roll_rate = (float) -1.6543697E38F;
            p83.body_pitch_rate = (float)2.8755214E38F;
            p83.body_yaw_rate = (float) -3.3290168E38F;
            p83.thrust = (float)1.5605084E38F;
            LoopBackDemoChannel.instance.send(p83); //===============================
            SET_POSITION_TARGET_LOCAL_NED p84 = LoopBackDemoChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.time_boot_ms = (uint)2338139053U;
            p84.target_system = (byte)(byte)144;
            p84.target_component = (byte)(byte)20;
            p84.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
            p84.type_mask = (ushort)(ushort)33915;
            p84.x = (float) -2.961105E37F;
            p84.y = (float) -2.6501437E38F;
            p84.z = (float) -1.1224211E38F;
            p84.vx = (float) -2.6876844E38F;
            p84.vy = (float) -8.796315E37F;
            p84.vz = (float) -8.988292E37F;
            p84.afx = (float)2.3622038E38F;
            p84.afy = (float) -1.1515938E37F;
            p84.afz = (float) -2.5606335E38F;
            p84.yaw = (float) -3.3954011E38F;
            p84.yaw_rate = (float) -2.4257488E38F;
            LoopBackDemoChannel.instance.send(p84); //===============================
            SET_POSITION_TARGET_GLOBAL_INT p86 = LoopBackDemoChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.time_boot_ms = (uint)3023425981U;
            p86.target_system = (byte)(byte)88;
            p86.target_component = (byte)(byte)213;
            p86.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p86.type_mask = (ushort)(ushort)26106;
            p86.lat_int = (int)1870563430;
            p86.lon_int = (int)1591396674;
            p86.alt = (float)5.0167744E37F;
            p86.vx = (float) -1.6588894E36F;
            p86.vy = (float)1.5545756E38F;
            p86.vz = (float) -1.4713135E38F;
            p86.afx = (float) -1.6404841E38F;
            p86.afy = (float)3.3824565E38F;
            p86.afz = (float) -1.7537021E38F;
            p86.yaw = (float) -7.2370523E37F;
            p86.yaw_rate = (float)2.9552517E38F;
            LoopBackDemoChannel.instance.send(p86); //===============================
            POSITION_TARGET_GLOBAL_INT p87 = LoopBackDemoChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.time_boot_ms = (uint)708991135U;
            p87.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL;
            p87.type_mask = (ushort)(ushort)16272;
            p87.lat_int = (int)2011264959;
            p87.lon_int = (int) -1427009580;
            p87.alt = (float)2.165825E38F;
            p87.vx = (float)3.0560267E38F;
            p87.vy = (float) -5.2976883E37F;
            p87.vz = (float)1.2204277E38F;
            p87.afx = (float) -3.0057933E37F;
            p87.afy = (float) -1.2189022E38F;
            p87.afz = (float)6.9370653E37F;
            p87.yaw = (float)2.3840348E38F;
            p87.yaw_rate = (float) -5.9860434E37F;
            LoopBackDemoChannel.instance.send(p87); //===============================
            LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.time_boot_ms = (uint)3698541468U;
            p89.x = (float) -2.478483E38F;
            p89.y = (float)7.54538E37F;
            p89.z = (float)3.3321099E38F;
            p89.roll = (float)2.6538237E38F;
            p89.pitch = (float)3.0529775E38F;
            p89.yaw = (float) -1.3405038E38F;
            LoopBackDemoChannel.instance.send(p89); //===============================
            HIL_STATE p90 = LoopBackDemoChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.time_usec = (ulong)9106464625116597417L;
            p90.roll = (float)7.1808923E37F;
            p90.pitch = (float) -1.9508694E38F;
            p90.yaw = (float) -3.9020248E37F;
            p90.rollspeed = (float)3.6903218E37F;
            p90.pitchspeed = (float)3.3475813E38F;
            p90.yawspeed = (float) -8.2181165E37F;
            p90.lat = (int) -787664936;
            p90.lon = (int) -256725813;
            p90.alt = (int)1150335051;
            p90.vx = (short)(short) -5067;
            p90.vy = (short)(short) -21063;
            p90.vz = (short)(short) -21252;
            p90.xacc = (short)(short)12373;
            p90.yacc = (short)(short) -15252;
            p90.zacc = (short)(short)26131;
            LoopBackDemoChannel.instance.send(p90); //===============================
            HIL_CONTROLS p91 = LoopBackDemoChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.time_usec = (ulong)501401982928136387L;
            p91.roll_ailerons = (float) -1.5810879E38F;
            p91.pitch_elevator = (float) -2.6899345E38F;
            p91.yaw_rudder = (float) -3.2927581E38F;
            p91.throttle = (float) -1.7765527E38F;
            p91.aux1 = (float) -1.8289247E38F;
            p91.aux2 = (float) -2.249853E38F;
            p91.aux3 = (float) -3.2338185E38F;
            p91.aux4 = (float) -2.9337605E38F;
            p91.mode = (MAV_MODE)MAV_MODE.MAV_MODE_TEST_DISARMED;
            p91.nav_mode = (byte)(byte)62;
            LoopBackDemoChannel.instance.send(p91); //===============================
            HIL_RC_INPUTS_RAW p92 = LoopBackDemoChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.time_usec = (ulong)8603561637963113160L;
            p92.chan1_raw = (ushort)(ushort)29508;
            p92.chan2_raw = (ushort)(ushort)32664;
            p92.chan3_raw = (ushort)(ushort)32694;
            p92.chan4_raw = (ushort)(ushort)50579;
            p92.chan5_raw = (ushort)(ushort)34379;
            p92.chan6_raw = (ushort)(ushort)63329;
            p92.chan7_raw = (ushort)(ushort)43863;
            p92.chan8_raw = (ushort)(ushort)39098;
            p92.chan9_raw = (ushort)(ushort)43300;
            p92.chan10_raw = (ushort)(ushort)37852;
            p92.chan11_raw = (ushort)(ushort)25489;
            p92.chan12_raw = (ushort)(ushort)53765;
            p92.rssi = (byte)(byte)167;
            LoopBackDemoChannel.instance.send(p92); //===============================
            HIL_ACTUATOR_CONTROLS p93 = LoopBackDemoChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.time_usec = (ulong)2478703015354976816L;
            p93.controls_SET(new float[16], 0);
            p93.mode = (MAV_MODE)MAV_MODE.MAV_MODE_STABILIZE_ARMED;
            p93.flags = (ulong)7998952571584048830L;
            LoopBackDemoChannel.instance.send(p93); //===============================
            OPTICAL_FLOW p100 = LoopBackDemoChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.time_usec = (ulong)5029465486506168876L;
            p100.sensor_id = (byte)(byte)75;
            p100.flow_x = (short)(short)5110;
            p100.flow_y = (short)(short) -9155;
            p100.flow_comp_m_x = (float) -3.3375506E38F;
            p100.flow_comp_m_y = (float)9.861052E37F;
            p100.quality = (byte)(byte)168;
            p100.ground_distance = (float) -4.7962605E37F;
            p100.flow_rate_x_SET((float) -1.4334656E38F, PH);
            p100.flow_rate_y_SET((float) -7.3278323E37F, PH);
            LoopBackDemoChannel.instance.send(p100); //===============================
            GLOBAL_VISION_POSITION_ESTIMATE p101 = LoopBackDemoChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.usec = (ulong)3924168300641513250L;
            p101.x = (float) -6.1522187E37F;
            p101.y = (float)1.293924E38F;
            p101.z = (float)1.3209763E38F;
            p101.roll = (float) -2.7359902E38F;
            p101.pitch = (float) -1.3218955E38F;
            p101.yaw = (float) -3.0457288E35F;
            LoopBackDemoChannel.instance.send(p101); //===============================
            VISION_POSITION_ESTIMATE p102 = LoopBackDemoChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.usec = (ulong)3392831312900711030L;
            p102.x = (float)2.1704888E38F;
            p102.y = (float) -2.350201E38F;
            p102.z = (float) -2.3275448E38F;
            p102.roll = (float)8.4438896E37F;
            p102.pitch = (float) -1.44761E38F;
            p102.yaw = (float)1.0267556E38F;
            LoopBackDemoChannel.instance.send(p102); //===============================
            VISION_SPEED_ESTIMATE p103 = LoopBackDemoChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.usec = (ulong)7534824507676085540L;
            p103.x = (float)1.9384213E38F;
            p103.y = (float) -2.9784234E38F;
            p103.z = (float)1.1367851E38F;
            LoopBackDemoChannel.instance.send(p103); //===============================
            VICON_POSITION_ESTIMATE p104 = LoopBackDemoChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.usec = (ulong)7696284614547302138L;
            p104.x = (float)1.0173255E38F;
            p104.y = (float) -2.1011094E37F;
            p104.z = (float)4.747625E37F;
            p104.roll = (float) -2.8604388E38F;
            p104.pitch = (float) -2.0428143E38F;
            p104.yaw = (float)1.8335986E38F;
            LoopBackDemoChannel.instance.send(p104); //===============================
            HIGHRES_IMU p105 = LoopBackDemoChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.time_usec = (ulong)5549477017858766380L;
            p105.xacc = (float) -1.3911169E38F;
            p105.yacc = (float) -8.0478974E37F;
            p105.zacc = (float) -1.2619563E38F;
            p105.xgyro = (float)1.9021706E38F;
            p105.ygyro = (float)1.806548E38F;
            p105.zgyro = (float)7.2377616E37F;
            p105.xmag = (float) -8.647185E37F;
            p105.ymag = (float) -3.4540386E37F;
            p105.zmag = (float) -3.3425488E38F;
            p105.abs_pressure = (float)2.2782028E38F;
            p105.diff_pressure = (float) -1.639277E36F;
            p105.pressure_alt = (float)2.4355602E38F;
            p105.temperature = (float)3.1626637E38F;
            p105.fields_updated = (ushort)(ushort)52997;
            LoopBackDemoChannel.instance.send(p105); //===============================
            OPTICAL_FLOW_RAD p106 = LoopBackDemoChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.time_usec = (ulong)3313623375882671501L;
            p106.sensor_id = (byte)(byte)89;
            p106.integration_time_us = (uint)3303795935U;
            p106.integrated_x = (float)2.0228886E38F;
            p106.integrated_y = (float)3.1277952E38F;
            p106.integrated_xgyro = (float)2.2658188E38F;
            p106.integrated_ygyro = (float) -1.7035214E38F;
            p106.integrated_zgyro = (float) -1.955413E38F;
            p106.temperature = (short)(short) -25758;
            p106.quality = (byte)(byte)173;
            p106.time_delta_distance_us = (uint)3846046835U;
            p106.distance = (float) -2.2403343E38F;
            LoopBackDemoChannel.instance.send(p106); //===============================
            HIL_SENSOR p107 = LoopBackDemoChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.time_usec = (ulong)1415871291316916267L;
            p107.xacc = (float)1.5017782E37F;
            p107.yacc = (float)5.465698E37F;
            p107.zacc = (float) -2.4436424E38F;
            p107.xgyro = (float)2.5795747E38F;
            p107.ygyro = (float)2.165999E38F;
            p107.zgyro = (float) -3.2236687E38F;
            p107.xmag = (float)8.662736E37F;
            p107.ymag = (float)1.4356474E38F;
            p107.zmag = (float) -3.1246827E38F;
            p107.abs_pressure = (float)1.0710593E38F;
            p107.diff_pressure = (float) -2.1987124E38F;
            p107.pressure_alt = (float)1.340467E37F;
            p107.temperature = (float) -1.1992645E38F;
            p107.fields_updated = (uint)3207955168U;
            LoopBackDemoChannel.instance.send(p107); //===============================
            SIM_STATE p108 = LoopBackDemoChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.q1 = (float) -4.0527208E37F;
            p108.q2 = (float)6.4004586E37F;
            p108.q3 = (float)9.146275E37F;
            p108.q4 = (float)1.7012274E37F;
            p108.roll = (float)1.8226656E38F;
            p108.pitch = (float) -1.298646E38F;
            p108.yaw = (float) -1.1738374E38F;
            p108.xacc = (float)2.2541982E38F;
            p108.yacc = (float)1.1137618E38F;
            p108.zacc = (float)2.0712595E38F;
            p108.xgyro = (float) -1.0125994E38F;
            p108.ygyro = (float)1.161918E38F;
            p108.zgyro = (float)1.0030431E38F;
            p108.lat = (float)1.7915317E38F;
            p108.lon = (float) -5.6112686E37F;
            p108.alt = (float)3.3668516E38F;
            p108.std_dev_horz = (float) -3.0715142E38F;
            p108.std_dev_vert = (float)5.2062887E37F;
            p108.vn = (float)9.048189E37F;
            p108.ve = (float)2.7273505E37F;
            p108.vd = (float)2.5092687E38F;
            LoopBackDemoChannel.instance.send(p108); //===============================
            RADIO_STATUS p109 = LoopBackDemoChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.rssi = (byte)(byte)87;
            p109.remrssi = (byte)(byte)10;
            p109.txbuf = (byte)(byte)249;
            p109.noise = (byte)(byte)211;
            p109.remnoise = (byte)(byte)152;
            p109.rxerrors = (ushort)(ushort)62321;
            p109.fixed_ = (ushort)(ushort)46267;
            LoopBackDemoChannel.instance.send(p109); //===============================
            FILE_TRANSFER_PROTOCOL p110 = LoopBackDemoChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_network = (byte)(byte)63;
            p110.target_system = (byte)(byte)235;
            p110.target_component = (byte)(byte)46;
            p110.payload_SET(new byte[251], 0);
            LoopBackDemoChannel.instance.send(p110); //===============================
            TIMESYNC p111 = LoopBackDemoChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.tc1 = (long) -8696158507481707502L;
            p111.ts1 = (long)6591523253757916005L;
            LoopBackDemoChannel.instance.send(p111); //===============================
            CAMERA_TRIGGER p112 = LoopBackDemoChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.time_usec = (ulong)8328342383611741016L;
            p112.seq = (uint)2722900023U;
            LoopBackDemoChannel.instance.send(p112); //===============================
            HIL_GPS p113 = LoopBackDemoChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.time_usec = (ulong)1960811403727915034L;
            p113.fix_type = (byte)(byte)235;
            p113.lat = (int) -1927863019;
            p113.lon = (int) -196828658;
            p113.alt = (int)1670690217;
            p113.eph = (ushort)(ushort)39845;
            p113.epv = (ushort)(ushort)30677;
            p113.vel = (ushort)(ushort)11342;
            p113.vn = (short)(short) -28754;
            p113.ve = (short)(short) -3906;
            p113.vd = (short)(short) -3344;
            p113.cog = (ushort)(ushort)49450;
            p113.satellites_visible = (byte)(byte)82;
            LoopBackDemoChannel.instance.send(p113); //===============================
            HIL_OPTICAL_FLOW p114 = LoopBackDemoChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.time_usec = (ulong)5603424699889282044L;
            p114.sensor_id = (byte)(byte)95;
            p114.integration_time_us = (uint)2608156816U;
            p114.integrated_x = (float)2.1398873E38F;
            p114.integrated_y = (float)2.398879E38F;
            p114.integrated_xgyro = (float)1.9058295E38F;
            p114.integrated_ygyro = (float) -3.322826E38F;
            p114.integrated_zgyro = (float)1.7869354E36F;
            p114.temperature = (short)(short) -26183;
            p114.quality = (byte)(byte)12;
            p114.time_delta_distance_us = (uint)3848756828U;
            p114.distance = (float) -2.0557236E38F;
            LoopBackDemoChannel.instance.send(p114); //===============================
            HIL_STATE_QUATERNION p115 = LoopBackDemoChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.time_usec = (ulong)818536360420533242L;
            p115.attitude_quaternion_SET(new float[4], 0);
            p115.rollspeed = (float) -2.0457015E38F;
            p115.pitchspeed = (float) -1.0130702E38F;
            p115.yawspeed = (float) -3.1889298E38F;
            p115.lat = (int) -166021978;
            p115.lon = (int)114154735;
            p115.alt = (int) -21231281;
            p115.vx = (short)(short)4690;
            p115.vy = (short)(short)4975;
            p115.vz = (short)(short)26761;
            p115.ind_airspeed = (ushort)(ushort)227;
            p115.true_airspeed = (ushort)(ushort)29430;
            p115.xacc = (short)(short)29495;
            p115.yacc = (short)(short)10872;
            p115.zacc = (short)(short)3164;
            LoopBackDemoChannel.instance.send(p115); //===============================
            SCALED_IMU2 p116 = LoopBackDemoChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.time_boot_ms = (uint)3914770814U;
            p116.xacc = (short)(short) -27390;
            p116.yacc = (short)(short) -12738;
            p116.zacc = (short)(short)32084;
            p116.xgyro = (short)(short) -18706;
            p116.ygyro = (short)(short) -25050;
            p116.zgyro = (short)(short) -5882;
            p116.xmag = (short)(short) -6130;
            p116.ymag = (short)(short) -19394;
            p116.zmag = (short)(short)4138;
            LoopBackDemoChannel.instance.send(p116); //===============================
            LOG_REQUEST_LIST p117 = LoopBackDemoChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.target_system = (byte)(byte)131;
            p117.target_component = (byte)(byte)209;
            p117.start = (ushort)(ushort)60836;
            p117.end = (ushort)(ushort)29572;
            LoopBackDemoChannel.instance.send(p117); //===============================
            LOG_ENTRY p118 = LoopBackDemoChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.id = (ushort)(ushort)27008;
            p118.num_logs = (ushort)(ushort)65030;
            p118.last_log_num = (ushort)(ushort)47389;
            p118.time_utc = (uint)2161684618U;
            p118.size = (uint)3453036030U;
            LoopBackDemoChannel.instance.send(p118); //===============================
            LOG_REQUEST_DATA p119 = LoopBackDemoChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.target_system = (byte)(byte)127;
            p119.target_component = (byte)(byte)7;
            p119.id = (ushort)(ushort)47016;
            p119.ofs = (uint)3768878380U;
            p119.count = (uint)2844246227U;
            LoopBackDemoChannel.instance.send(p119); //===============================
            LOG_DATA p120 = LoopBackDemoChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.id = (ushort)(ushort)14304;
            p120.ofs = (uint)3034218636U;
            p120.count = (byte)(byte)176;
            p120.data__SET(new byte[90], 0);
            LoopBackDemoChannel.instance.send(p120); //===============================
            LOG_ERASE p121 = LoopBackDemoChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_system = (byte)(byte)91;
            p121.target_component = (byte)(byte)235;
            LoopBackDemoChannel.instance.send(p121); //===============================
            LOG_REQUEST_END p122 = LoopBackDemoChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_system = (byte)(byte)153;
            p122.target_component = (byte)(byte)48;
            LoopBackDemoChannel.instance.send(p122); //===============================
            GPS_INJECT_DATA p123 = LoopBackDemoChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.target_system = (byte)(byte)190;
            p123.target_component = (byte)(byte)144;
            p123.len = (byte)(byte)102;
            p123.data__SET(new byte[110], 0);
            LoopBackDemoChannel.instance.send(p123); //===============================
            GPS2_RAW p124 = LoopBackDemoChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.time_usec = (ulong)7132846286888676599L;
            p124.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC;
            p124.lat = (int)343935108;
            p124.lon = (int) -903549514;
            p124.alt = (int) -104619748;
            p124.eph = (ushort)(ushort)7850;
            p124.epv = (ushort)(ushort)42662;
            p124.vel = (ushort)(ushort)55234;
            p124.cog = (ushort)(ushort)44960;
            p124.satellites_visible = (byte)(byte)90;
            p124.dgps_numch = (byte)(byte)123;
            p124.dgps_age = (uint)346795490U;
            LoopBackDemoChannel.instance.send(p124); //===============================
            POWER_STATUS p125 = LoopBackDemoChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vcc = (ushort)(ushort)12193;
            p125.Vservo = (ushort)(ushort)45915;
            p125.flags = (MAV_POWER_STATUS)MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED;
            LoopBackDemoChannel.instance.send(p125); //===============================
            SERIAL_CONTROL p126 = LoopBackDemoChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.device = (SERIAL_CONTROL_DEV)SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2;
            p126.flags = (SERIAL_CONTROL_FLAG)SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI;
            p126.timeout = (ushort)(ushort)45585;
            p126.baudrate = (uint)2892639497U;
            p126.count = (byte)(byte)107;
            p126.data__SET(new byte[70], 0);
            LoopBackDemoChannel.instance.send(p126); //===============================
            GPS_RTK p127 = LoopBackDemoChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.time_last_baseline_ms = (uint)828239733U;
            p127.rtk_receiver_id = (byte)(byte)136;
            p127.wn = (ushort)(ushort)29082;
            p127.tow = (uint)1737066086U;
            p127.rtk_health = (byte)(byte)180;
            p127.rtk_rate = (byte)(byte)123;
            p127.nsats = (byte)(byte)224;
            p127.baseline_coords_type = (byte)(byte)225;
            p127.baseline_a_mm = (int)112707629;
            p127.baseline_b_mm = (int) -2019521657;
            p127.baseline_c_mm = (int) -2069586512;
            p127.accuracy = (uint)2692636791U;
            p127.iar_num_hypotheses = (int) -1364979414;
            LoopBackDemoChannel.instance.send(p127); //===============================
            GPS2_RTK p128 = LoopBackDemoChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.time_last_baseline_ms = (uint)967066101U;
            p128.rtk_receiver_id = (byte)(byte)55;
            p128.wn = (ushort)(ushort)52274;
            p128.tow = (uint)644090845U;
            p128.rtk_health = (byte)(byte)50;
            p128.rtk_rate = (byte)(byte)170;
            p128.nsats = (byte)(byte)112;
            p128.baseline_coords_type = (byte)(byte)63;
            p128.baseline_a_mm = (int) -291760188;
            p128.baseline_b_mm = (int)772440457;
            p128.baseline_c_mm = (int) -1664813903;
            p128.accuracy = (uint)139076065U;
            p128.iar_num_hypotheses = (int)1160807933;
            LoopBackDemoChannel.instance.send(p128); //===============================
            SCALED_IMU3 p129 = LoopBackDemoChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.time_boot_ms = (uint)1276786399U;
            p129.xacc = (short)(short)13618;
            p129.yacc = (short)(short)30889;
            p129.zacc = (short)(short)14180;
            p129.xgyro = (short)(short) -10656;
            p129.ygyro = (short)(short) -21096;
            p129.zgyro = (short)(short) -28572;
            p129.xmag = (short)(short) -17738;
            p129.ymag = (short)(short) -20360;
            p129.zmag = (short)(short)12626;
            LoopBackDemoChannel.instance.send(p129); //===============================
            DATA_TRANSMISSION_HANDSHAKE p130 = LoopBackDemoChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.type = (byte)(byte)59;
            p130.size = (uint)3245554769U;
            p130.width = (ushort)(ushort)31585;
            p130.height = (ushort)(ushort)52945;
            p130.packets = (ushort)(ushort)8225;
            p130.payload = (byte)(byte)188;
            p130.jpg_quality = (byte)(byte)5;
            LoopBackDemoChannel.instance.send(p130); //===============================
            ENCAPSULATED_DATA p131 = LoopBackDemoChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.seqnr = (ushort)(ushort)41639;
            p131.data__SET(new byte[253], 0);
            LoopBackDemoChannel.instance.send(p131); //===============================
            DISTANCE_SENSOR p132 = LoopBackDemoChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.time_boot_ms = (uint)1431108695U;
            p132.min_distance = (ushort)(ushort)65523;
            p132.max_distance = (ushort)(ushort)22317;
            p132.current_distance = (ushort)(ushort)59697;
            p132.type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER;
            p132.id = (byte)(byte)77;
            p132.orientation = (MAV_SENSOR_ORIENTATION)MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_90;
            p132.covariance = (byte)(byte)48;
            LoopBackDemoChannel.instance.send(p132); //===============================
            TERRAIN_REQUEST p133 = LoopBackDemoChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.lat = (int)978827324;
            p133.lon = (int) -141152617;
            p133.grid_spacing = (ushort)(ushort)17974;
            p133.mask = (ulong)8371773575584025158L;
            LoopBackDemoChannel.instance.send(p133); //===============================
            TERRAIN_DATA p134 = LoopBackDemoChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.lat = (int)296829108;
            p134.lon = (int)600809855;
            p134.grid_spacing = (ushort)(ushort)26250;
            p134.gridbit = (byte)(byte)194;
            p134.data__SET(new short[16], 0);
            LoopBackDemoChannel.instance.send(p134); //===============================
            TERRAIN_CHECK p135 = LoopBackDemoChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lat = (int) -1323366003;
            p135.lon = (int) -1854173398;
            LoopBackDemoChannel.instance.send(p135); //===============================
            TERRAIN_REPORT p136 = LoopBackDemoChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.lat = (int)507515493;
            p136.lon = (int)533461556;
            p136.spacing = (ushort)(ushort)41660;
            p136.terrain_height = (float)1.3813369E38F;
            p136.current_height = (float) -2.3533086E38F;
            p136.pending = (ushort)(ushort)2023;
            p136.loaded = (ushort)(ushort)49928;
            LoopBackDemoChannel.instance.send(p136); //===============================
            SCALED_PRESSURE2 p137 = LoopBackDemoChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.time_boot_ms = (uint)4027764230U;
            p137.press_abs = (float) -2.0433864E38F;
            p137.press_diff = (float)2.6118673E38F;
            p137.temperature = (short)(short) -25068;
            LoopBackDemoChannel.instance.send(p137); //===============================
            ATT_POS_MOCAP p138 = LoopBackDemoChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.time_usec = (ulong)6688248507797990945L;
            p138.q_SET(new float[4], 0);
            p138.x = (float) -3.1865994E38F;
            p138.y = (float) -1.2909108E38F;
            p138.z = (float)2.4663576E38F;
            LoopBackDemoChannel.instance.send(p138); //===============================
            SET_ACTUATOR_CONTROL_TARGET p139 = LoopBackDemoChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.time_usec = (ulong)8069622651712982494L;
            p139.group_mlx = (byte)(byte)250;
            p139.target_system = (byte)(byte)254;
            p139.target_component = (byte)(byte)54;
            p139.controls_SET(new float[8], 0);
            LoopBackDemoChannel.instance.send(p139); //===============================
            ACTUATOR_CONTROL_TARGET p140 = LoopBackDemoChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.time_usec = (ulong)6285367992779457292L;
            p140.group_mlx = (byte)(byte)34;
            p140.controls_SET(new float[8], 0);
            LoopBackDemoChannel.instance.send(p140); //===============================
            ALTITUDE p141 = LoopBackDemoChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.time_usec = (ulong)8354853005290922649L;
            p141.altitude_monotonic = (float)5.093605E37F;
            p141.altitude_amsl = (float) -5.6506083E37F;
            p141.altitude_local = (float) -2.5643472E38F;
            p141.altitude_relative = (float)7.5951833E37F;
            p141.altitude_terrain = (float)1.5561739E38F;
            p141.bottom_clearance = (float) -1.2622218E38F;
            LoopBackDemoChannel.instance.send(p141); //===============================
            RESOURCE_REQUEST p142 = LoopBackDemoChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.request_id = (byte)(byte)97;
            p142.uri_type = (byte)(byte)253;
            p142.uri_SET(new byte[120], 0);
            p142.transfer_type = (byte)(byte)68;
            p142.storage_SET(new byte[120], 0);
            LoopBackDemoChannel.instance.send(p142); //===============================
            SCALED_PRESSURE3 p143 = LoopBackDemoChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.time_boot_ms = (uint)2245534781U;
            p143.press_abs = (float)2.7398578E38F;
            p143.press_diff = (float)2.2130316E38F;
            p143.temperature = (short)(short)15820;
            LoopBackDemoChannel.instance.send(p143); //===============================
            FOLLOW_TARGET p144 = LoopBackDemoChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.timestamp = (ulong)1818292391947041708L;
            p144.est_capabilities = (byte)(byte)141;
            p144.lat = (int) -730537219;
            p144.lon = (int) -902721962;
            p144.alt = (float)2.5195503E38F;
            p144.vel_SET(new float[3], 0);
            p144.acc_SET(new float[3], 0);
            p144.attitude_q_SET(new float[4], 0);
            p144.rates_SET(new float[3], 0);
            p144.position_cov_SET(new float[3], 0);
            p144.custom_state = (ulong)6463065673189012056L;
            LoopBackDemoChannel.instance.send(p144); //===============================
            CONTROL_SYSTEM_STATE p146 = LoopBackDemoChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.time_usec = (ulong)3147128532422175749L;
            p146.x_acc = (float)1.1979183E38F;
            p146.y_acc = (float) -1.1627588E38F;
            p146.z_acc = (float) -2.3851049E38F;
            p146.x_vel = (float) -2.0342358E38F;
            p146.y_vel = (float)2.1816419E38F;
            p146.z_vel = (float)7.060238E37F;
            p146.x_pos = (float)1.326288E38F;
            p146.y_pos = (float) -8.410005E37F;
            p146.z_pos = (float)3.3720453E38F;
            p146.airspeed = (float)2.272017E38F;
            p146.vel_variance_SET(new float[3], 0);
            p146.pos_variance_SET(new float[3], 0);
            p146.q_SET(new float[4], 0);
            p146.roll_rate = (float)2.2667211E38F;
            p146.pitch_rate = (float)2.3434412E38F;
            p146.yaw_rate = (float)2.3576496E38F;
            LoopBackDemoChannel.instance.send(p146); //===============================
            BATTERY_STATUS p147 = LoopBackDemoChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.id = (byte)(byte)217;
            p147.battery_function = (MAV_BATTERY_FUNCTION)MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD;
            p147.type = (MAV_BATTERY_TYPE)MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIPO;
            p147.temperature = (short)(short) -29183;
            p147.voltages_SET(new ushort[10], 0);
            p147.current_battery = (short)(short)20928;
            p147.current_consumed = (int) -1981481718;
            p147.energy_consumed = (int) -533700259;
            p147.battery_remaining = (sbyte)(sbyte) - 7;
            LoopBackDemoChannel.instance.send(p147); //===============================
            AUTOPILOT_VERSION p148 = LoopBackDemoChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY)MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION;
            p148.flight_sw_version = (uint)3193403314U;
            p148.middleware_sw_version = (uint)723807723U;
            p148.os_sw_version = (uint)1016394389U;
            p148.board_version = (uint)3549381617U;
            p148.flight_custom_version_SET(new byte[8], 0);
            p148.middleware_custom_version_SET(new byte[8], 0);
            p148.os_custom_version_SET(new byte[8], 0);
            p148.vendor_id = (ushort)(ushort)39613;
            p148.product_id = (ushort)(ushort)48992;
            p148.uid = (ulong)2257407650473426468L;
            p148.uid2_SET(new byte[18], 0, PH);
            LoopBackDemoChannel.instance.send(p148); //===============================
            LANDING_TARGET p149 = LoopBackDemoChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.time_usec = (ulong)4077079318302022945L;
            p149.target_num = (byte)(byte)236;
            p149.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_MISSION;
            p149.angle_x = (float)2.8360021E38F;
            p149.angle_y = (float) -1.3307163E38F;
            p149.distance = (float)9.052936E37F;
            p149.size_x = (float)3.3731022E38F;
            p149.size_y = (float) -7.2878034E37F;
            p149.x_SET((float)3.1807323E38F, PH);
            p149.y_SET((float) -2.8297943E38F, PH);
            p149.z_SET((float) -5.0109696E37F, PH);
            p149.q_SET(new float[4], 0, PH);
            p149.type = (LANDING_TARGET_TYPE)LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_RADIO_BEACON;
            p149.position_valid_SET((byte)(byte)216, PH);
            LoopBackDemoChannel.instance.send(p149); //===============================
            ARRAY_TEST_0 p150 = LoopBackDemoChannel.new_ARRAY_TEST_0();
            PH.setPack(p150);
            p150.v1 = (byte)(byte)226;
            p150.ar_i8_SET(new sbyte[4], 0);
            p150.ar_u8_SET(new byte[4], 0);
            p150.ar_u16_SET(new ushort[4], 0);
            p150.ar_u32_SET(new uint[4], 0);
            LoopBackDemoChannel.instance.send(p150); //===============================
            ARRAY_TEST_1 p151 = LoopBackDemoChannel.new_ARRAY_TEST_1();
            PH.setPack(p151);
            p151.ar_u32_SET(new uint[4], 0);
            LoopBackDemoChannel.instance.send(p151); //===============================
            ARRAY_TEST_3 p153 = LoopBackDemoChannel.new_ARRAY_TEST_3();
            PH.setPack(p153);
            p153.v = (byte)(byte)211;
            p153.ar_u32_SET(new uint[4], 0);
            LoopBackDemoChannel.instance.send(p153); //===============================
            ARRAY_TEST_4 p154 = LoopBackDemoChannel.new_ARRAY_TEST_4();
            PH.setPack(p154);
            p154.ar_u32_SET(new uint[4], 0);
            p154.v = (byte)(byte)135;
            LoopBackDemoChannel.instance.send(p154); //===============================
            ARRAY_TEST_5 p155 = LoopBackDemoChannel.new_ARRAY_TEST_5();
            PH.setPack(p155);
            p155.c1_SET("DEMO", PH);
            p155.c2_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p155); //===============================
            ARRAY_TEST_6 p156 = LoopBackDemoChannel.new_ARRAY_TEST_6();
            PH.setPack(p156);
            p156.v1 = (byte)(byte)242;
            p156.v2 = (ushort)(ushort)63558;
            p156.v3 = (uint)4099660638U;
            p156.ar_u32_SET(new uint[2], 0);
            p156.ar_i32_SET(new int[2], 0);
            p156.ar_u16_SET(new ushort[2], 0);
            p156.ar_i16_SET(new short[2], 0);
            p156.ar_u8_SET(new byte[2], 0);
            p156.ar_i8_SET(new sbyte[2], 0);
            p156.ar_c_SET("DEMO", PH);
            p156.ar_d_SET(new double[2], 0);
            p156.ar_f_SET(new float[2], 0);
            LoopBackDemoChannel.instance.send(p156); //===============================
            ARRAY_TEST_7 p157 = LoopBackDemoChannel.new_ARRAY_TEST_7();
            PH.setPack(p157);
            p157.ar_d_SET(new double[2], 0);
            p157.ar_f_SET(new float[2], 0);
            p157.ar_u32_SET(new uint[2], 0);
            p157.ar_i32_SET(new int[2], 0);
            p157.ar_u16_SET(new ushort[2], 0);
            p157.ar_i16_SET(new short[2], 0);
            p157.ar_u8_SET(new byte[2], 0);
            p157.ar_i8_SET(new sbyte[2], 0);
            p157.ar_c_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p157); //===============================
            ARRAY_TEST_8 p158 = LoopBackDemoChannel.new_ARRAY_TEST_8();
            PH.setPack(p158);
            p158.v3 = (uint)370040120U;
            p158.ar_d_SET(new double[2], 0);
            p158.ar_u16_SET(new ushort[2], 0);
            LoopBackDemoChannel.instance.send(p158); //===============================
            ESTIMATOR_STATUS p230 = LoopBackDemoChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.time_usec = (ulong)239414135361116599L;
            p230.flags = (ESTIMATOR_STATUS_FLAGS)ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS;
            p230.vel_ratio = (float)1.2150905E37F;
            p230.pos_horiz_ratio = (float) -2.9798695E38F;
            p230.pos_vert_ratio = (float) -1.4508763E38F;
            p230.mag_ratio = (float) -2.9020064E38F;
            p230.hagl_ratio = (float) -1.8068568E38F;
            p230.tas_ratio = (float)1.1184806E38F;
            p230.pos_horiz_accuracy = (float) -1.5739887E38F;
            p230.pos_vert_accuracy = (float)7.8487976E37F;
            LoopBackDemoChannel.instance.send(p230); //===============================
            WIND_COV p231 = LoopBackDemoChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.time_usec = (ulong)810876318351861591L;
            p231.wind_x = (float) -1.7326445E38F;
            p231.wind_y = (float)5.19924E37F;
            p231.wind_z = (float) -3.40177E38F;
            p231.var_horiz = (float)6.4647584E37F;
            p231.var_vert = (float) -2.7140523E38F;
            p231.wind_alt = (float)9.626624E37F;
            p231.horiz_accuracy = (float) -1.545078E38F;
            p231.vert_accuracy = (float)1.6702579E38F;
            LoopBackDemoChannel.instance.send(p231); //===============================
            GPS_INPUT p232 = LoopBackDemoChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.time_usec = (ulong)899087746080964139L;
            p232.gps_id = (byte)(byte)210;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS)GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP;
            p232.time_week_ms = (uint)3157165962U;
            p232.time_week = (ushort)(ushort)59602;
            p232.fix_type = (byte)(byte)24;
            p232.lat = (int) -1029054721;
            p232.lon = (int)1971642873;
            p232.alt = (float)9.529295E37F;
            p232.hdop = (float) -2.8456557E38F;
            p232.vdop = (float) -1.1152082E38F;
            p232.vn = (float) -1.1500003E38F;
            p232.ve = (float)1.0911065E38F;
            p232.vd = (float)2.0823492E38F;
            p232.speed_accuracy = (float)2.2365652E38F;
            p232.horiz_accuracy = (float) -9.995883E37F;
            p232.vert_accuracy = (float)3.3961017E38F;
            p232.satellites_visible = (byte)(byte)206;
            LoopBackDemoChannel.instance.send(p232); //===============================
            GPS_RTCM_DATA p233 = LoopBackDemoChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.flags = (byte)(byte)30;
            p233.len = (byte)(byte)234;
            p233.data__SET(new byte[180], 0);
            LoopBackDemoChannel.instance.send(p233); //===============================
            HIGH_LATENCY p234 = LoopBackDemoChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED;
            p234.custom_mode = (uint)1290988834U;
            p234.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING;
            p234.roll = (short)(short)14489;
            p234.pitch = (short)(short)22802;
            p234.heading = (ushort)(ushort)3782;
            p234.throttle = (sbyte)(sbyte) - 103;
            p234.heading_sp = (short)(short)25747;
            p234.latitude = (int) -2101757468;
            p234.longitude = (int) -1037828819;
            p234.altitude_amsl = (short)(short) -4820;
            p234.altitude_sp = (short)(short)2314;
            p234.airspeed = (byte)(byte)8;
            p234.airspeed_sp = (byte)(byte)55;
            p234.groundspeed = (byte)(byte)157;
            p234.climb_rate = (sbyte)(sbyte)17;
            p234.gps_nsat = (byte)(byte)225;
            p234.gps_fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS;
            p234.battery_remaining = (byte)(byte)166;
            p234.temperature = (sbyte)(sbyte)11;
            p234.temperature_air = (sbyte)(sbyte) - 114;
            p234.failsafe = (byte)(byte)29;
            p234.wp_num = (byte)(byte)209;
            p234.wp_distance = (ushort)(ushort)64576;
            LoopBackDemoChannel.instance.send(p234); //===============================
            VIBRATION p241 = LoopBackDemoChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.time_usec = (ulong)4040721860987966685L;
            p241.vibration_x = (float) -2.7957602E38F;
            p241.vibration_y = (float) -2.234825E38F;
            p241.vibration_z = (float)7.523236E37F;
            p241.clipping_0 = (uint)2361624955U;
            p241.clipping_1 = (uint)3356698706U;
            p241.clipping_2 = (uint)232169368U;
            LoopBackDemoChannel.instance.send(p241); //===============================
            HOME_POSITION p242 = LoopBackDemoChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.latitude = (int)1516122569;
            p242.longitude = (int) -415088272;
            p242.altitude = (int) -1435471815;
            p242.x = (float)2.3261504E38F;
            p242.y = (float) -1.5385461E38F;
            p242.z = (float)1.7285682E38F;
            p242.q_SET(new float[4], 0);
            p242.approach_x = (float)3.0212069E38F;
            p242.approach_y = (float)3.3076618E38F;
            p242.approach_z = (float) -2.8583019E37F;
            p242.time_usec_SET((ulong)3915013525052617432L, PH);
            LoopBackDemoChannel.instance.send(p242); //===============================
            SET_HOME_POSITION p243 = LoopBackDemoChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.target_system = (byte)(byte)228;
            p243.latitude = (int) -441100252;
            p243.longitude = (int)241882647;
            p243.altitude = (int) -622862570;
            p243.x = (float)2.2599523E38F;
            p243.y = (float) -4.3533616E37F;
            p243.z = (float) -1.1112306E38F;
            p243.q_SET(new float[4], 0);
            p243.approach_x = (float) -2.72085E38F;
            p243.approach_y = (float)2.48397E38F;
            p243.approach_z = (float)2.8461892E38F;
            p243.time_usec_SET((ulong)8688859128076000978L, PH);
            LoopBackDemoChannel.instance.send(p243); //===============================
            MESSAGE_INTERVAL p244 = LoopBackDemoChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.message_id = (ushort)(ushort)28747;
            p244.interval_us = (int) -1429145434;
            LoopBackDemoChannel.instance.send(p244); //===============================
            EXTENDED_SYS_STATE p245 = LoopBackDemoChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.vtol_state = (MAV_VTOL_STATE)MAV_VTOL_STATE.MAV_VTOL_STATE_MC;
            p245.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR;
            LoopBackDemoChannel.instance.send(p245); //===============================
            ADSB_VEHICLE p246 = LoopBackDemoChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.ICAO_address = (uint)1401832244U;
            p246.lat = (int)1340983723;
            p246.lon = (int) -328962138;
            p246.altitude_type = (ADSB_ALTITUDE_TYPE)ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC;
            p246.altitude = (int) -516902477;
            p246.heading = (ushort)(ushort)17667;
            p246.hor_velocity = (ushort)(ushort)10417;
            p246.ver_velocity = (short)(short) -27199;
            p246.callsign_SET("DEMO", PH);
            p246.emitter_type = (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_LIGHTER_AIR;
            p246.tslc = (byte)(byte)103;
            p246.flags = (ADSB_FLAGS)ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS;
            p246.squawk = (ushort)(ushort)225;
            LoopBackDemoChannel.instance.send(p246); //===============================
            COLLISION p247 = LoopBackDemoChannel.new_COLLISION();
            PH.setPack(p247);
            p247.src_ = (MAV_COLLISION_SRC)MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB;
            p247.id = (uint)3398464065U;
            p247.action = (MAV_COLLISION_ACTION)MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_REPORT;
            p247.threat_level = (MAV_COLLISION_THREAT_LEVEL)MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW;
            p247.time_to_minimum_delta = (float)1.5555375E37F;
            p247.altitude_minimum_delta = (float) -2.2608654E38F;
            p247.horizontal_minimum_delta = (float) -1.0389239E38F;
            LoopBackDemoChannel.instance.send(p247); //===============================
            V2_EXTENSION p248 = LoopBackDemoChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.target_network = (byte)(byte)159;
            p248.target_system = (byte)(byte)120;
            p248.target_component = (byte)(byte)110;
            p248.message_type = (ushort)(ushort)34218;
            p248.payload_SET(new byte[249], 0);
            LoopBackDemoChannel.instance.send(p248); //===============================
            MEMORY_VECT p249 = LoopBackDemoChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.address = (ushort)(ushort)13890;
            p249.ver = (byte)(byte)163;
            p249.type = (byte)(byte)125;
            p249.value_SET(new sbyte[32], 0);
            LoopBackDemoChannel.instance.send(p249); //===============================
            DEBUG_VECT p250 = LoopBackDemoChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.name_SET("DEMO", PH);
            p250.time_usec = (ulong)6291407016746956717L;
            p250.x = (float)2.854832E38F;
            p250.y = (float) -3.3565425E38F;
            p250.z = (float)1.176126E38F;
            LoopBackDemoChannel.instance.send(p250); //===============================
            NAMED_VALUE_FLOAT p251 = LoopBackDemoChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.time_boot_ms = (uint)2363400724U;
            p251.name_SET("DEMO", PH);
            p251.value = (float) -1.7534549E38F;
            LoopBackDemoChannel.instance.send(p251); //===============================
            NAMED_VALUE_INT p252 = LoopBackDemoChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.time_boot_ms = (uint)864467899U;
            p252.name_SET("DEMO", PH);
            p252.value = (int)1764645297;
            LoopBackDemoChannel.instance.send(p252); //===============================
            STATUSTEXT p253 = LoopBackDemoChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.severity = (MAV_SEVERITY)MAV_SEVERITY.MAV_SEVERITY_INFO;
            p253.text_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p253); //===============================
            DEBUG p254 = LoopBackDemoChannel.new_DEBUG();
            PH.setPack(p254);
            p254.time_boot_ms = (uint)1395727146U;
            p254.ind = (byte)(byte)203;
            p254.value = (float) -1.6599138E38F;
            LoopBackDemoChannel.instance.send(p254); //===============================
            SETUP_SIGNING p256 = LoopBackDemoChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.target_system = (byte)(byte)152;
            p256.target_component = (byte)(byte)230;
            p256.secret_key_SET(new byte[32], 0);
            p256.initial_timestamp = (ulong)3547513169746493241L;
            LoopBackDemoChannel.instance.send(p256); //===============================
            BUTTON_CHANGE p257 = LoopBackDemoChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.time_boot_ms = (uint)4223421499U;
            p257.last_change_ms = (uint)390712208U;
            p257.state = (byte)(byte)73;
            LoopBackDemoChannel.instance.send(p257); //===============================
            PLAY_TUNE p258 = LoopBackDemoChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_system = (byte)(byte)181;
            p258.target_component = (byte)(byte)240;
            p258.tune_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p258); //===============================
            CAMERA_INFORMATION p259 = LoopBackDemoChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.time_boot_ms = (uint)2788406996U;
            p259.vendor_name_SET(new byte[32], 0);
            p259.model_name_SET(new byte[32], 0);
            p259.firmware_version = (uint)1655138607U;
            p259.focal_length = (float) -3.2953628E38F;
            p259.sensor_size_h = (float) -5.6707564E37F;
            p259.sensor_size_v = (float) -2.9151437E38F;
            p259.resolution_h = (ushort)(ushort)61260;
            p259.resolution_v = (ushort)(ushort)22866;
            p259.lens_id = (byte)(byte)207;
            p259.flags = (CAMERA_CAP_FLAGS)CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO;
            p259.cam_definition_version = (ushort)(ushort)40323;
            p259.cam_definition_uri_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p259); //===============================
            CAMERA_SETTINGS p260 = LoopBackDemoChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.time_boot_ms = (uint)1345246815U;
            p260.mode_id = (CAMERA_MODE)CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY;
            LoopBackDemoChannel.instance.send(p260); //===============================
            STORAGE_INFORMATION p261 = LoopBackDemoChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.time_boot_ms = (uint)3644274882U;
            p261.storage_id = (byte)(byte)173;
            p261.storage_count = (byte)(byte)211;
            p261.status = (byte)(byte)232;
            p261.total_capacity = (float) -9.115061E37F;
            p261.used_capacity = (float) -1.5349413E38F;
            p261.available_capacity = (float)9.542591E37F;
            p261.read_speed = (float) -2.9648914E38F;
            p261.write_speed = (float)2.8400172E38F;
            LoopBackDemoChannel.instance.send(p261); //===============================
            CAMERA_CAPTURE_STATUS p262 = LoopBackDemoChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.time_boot_ms = (uint)2162897808U;
            p262.image_status = (byte)(byte)22;
            p262.video_status = (byte)(byte)226;
            p262.image_interval = (float) -3.2796565E38F;
            p262.recording_time_ms = (uint)493649108U;
            p262.available_capacity = (float)2.8249945E38F;
            LoopBackDemoChannel.instance.send(p262); //===============================
            CAMERA_IMAGE_CAPTURED p263 = LoopBackDemoChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.time_boot_ms = (uint)2114628386U;
            p263.time_utc = (ulong)5114816150530190759L;
            p263.camera_id = (byte)(byte)111;
            p263.lat = (int) -79232422;
            p263.lon = (int)74037701;
            p263.alt = (int)1795953766;
            p263.relative_alt = (int)1190694561;
            p263.q_SET(new float[4], 0);
            p263.image_index = (int) -861381618;
            p263.capture_result = (sbyte)(sbyte) - 42;
            p263.file_url_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p263); //===============================
            FLIGHT_INFORMATION p264 = LoopBackDemoChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.time_boot_ms = (uint)4241746032U;
            p264.arming_time_utc = (ulong)3664642967108170689L;
            p264.takeoff_time_utc = (ulong)6243942409411421416L;
            p264.flight_uuid = (ulong)5095083555215976588L;
            LoopBackDemoChannel.instance.send(p264); //===============================
            MOUNT_ORIENTATION p265 = LoopBackDemoChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.time_boot_ms = (uint)3850699838U;
            p265.roll = (float) -3.49017E37F;
            p265.pitch = (float) -1.6989834E38F;
            p265.yaw = (float) -2.4821372E38F;
            LoopBackDemoChannel.instance.send(p265); //===============================
            LOGGING_DATA p266 = LoopBackDemoChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.target_system = (byte)(byte)109;
            p266.target_component = (byte)(byte)213;
            p266.sequence = (ushort)(ushort)5590;
            p266.length = (byte)(byte)210;
            p266.first_message_offset = (byte)(byte)186;
            p266.data__SET(new byte[249], 0);
            LoopBackDemoChannel.instance.send(p266); //===============================
            LOGGING_DATA_ACKED p267 = LoopBackDemoChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.target_system = (byte)(byte)19;
            p267.target_component = (byte)(byte)48;
            p267.sequence = (ushort)(ushort)44110;
            p267.length = (byte)(byte)138;
            p267.first_message_offset = (byte)(byte)159;
            p267.data__SET(new byte[249], 0);
            LoopBackDemoChannel.instance.send(p267); //===============================
            LOGGING_ACK p268 = LoopBackDemoChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_system = (byte)(byte)211;
            p268.target_component = (byte)(byte)167;
            p268.sequence = (ushort)(ushort)24311;
            LoopBackDemoChannel.instance.send(p268); //===============================
            VIDEO_STREAM_INFORMATION p269 = LoopBackDemoChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.camera_id = (byte)(byte)95;
            p269.status = (byte)(byte)191;
            p269.framerate = (float)2.6793682E37F;
            p269.resolution_h = (ushort)(ushort)41649;
            p269.resolution_v = (ushort)(ushort)31319;
            p269.bitrate = (uint)3408149616U;
            p269.rotation = (ushort)(ushort)53673;
            p269.uri_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p269); //===============================
            SET_VIDEO_STREAM_SETTINGS p270 = LoopBackDemoChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.target_system = (byte)(byte)166;
            p270.target_component = (byte)(byte)126;
            p270.camera_id = (byte)(byte)110;
            p270.framerate = (float) -1.6045552E38F;
            p270.resolution_h = (ushort)(ushort)24466;
            p270.resolution_v = (ushort)(ushort)13949;
            p270.bitrate = (uint)667073968U;
            p270.rotation = (ushort)(ushort)53677;
            p270.uri_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p270); //===============================
            WIFI_CONFIG_AP p299 = LoopBackDemoChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.ssid_SET("DEMO", PH);
            p299.password_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p299); //===============================
            PROTOCOL_VERSION p300 = LoopBackDemoChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.version = (ushort)(ushort)42085;
            p300.min_version = (ushort)(ushort)46143;
            p300.max_version = (ushort)(ushort)3030;
            p300.spec_version_hash_SET(new byte[8], 0);
            p300.library_version_hash_SET(new byte[8], 0);
            LoopBackDemoChannel.instance.send(p300); //===============================
            UAVCAN_NODE_STATUS p310 = LoopBackDemoChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.time_usec = (ulong)7280625486582299593L;
            p310.uptime_sec = (uint)552564349U;
            p310.health = (UAVCAN_NODE_HEALTH)UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING;
            p310.mode = (UAVCAN_NODE_MODE)UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE;
            p310.sub_mode = (byte)(byte)34;
            p310.vendor_specific_status_code = (ushort)(ushort)63042;
            LoopBackDemoChannel.instance.send(p310); //===============================
            UAVCAN_NODE_INFO p311 = LoopBackDemoChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.time_usec = (ulong)1788176112389723604L;
            p311.uptime_sec = (uint)3603507809U;
            p311.name_SET("DEMO", PH);
            p311.hw_version_major = (byte)(byte)199;
            p311.hw_version_minor = (byte)(byte)183;
            p311.hw_unique_id_SET(new byte[16], 0);
            p311.sw_version_major = (byte)(byte)7;
            p311.sw_version_minor = (byte)(byte)237;
            p311.sw_vcs_commit = (uint)372035373U;
            LoopBackDemoChannel.instance.send(p311); //===============================
            PARAM_EXT_REQUEST_READ p320 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.target_system = (byte)(byte)39;
            p320.target_component = (byte)(byte)138;
            p320.param_id_SET("DEMO", PH);
            p320.param_index = (short)(short) -17763;
            LoopBackDemoChannel.instance.send(p320); //===============================
            PARAM_EXT_REQUEST_LIST p321 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_system = (byte)(byte)12;
            p321.target_component = (byte)(byte)183;
            LoopBackDemoChannel.instance.send(p321); //===============================
            PARAM_EXT_VALUE p322 = LoopBackDemoChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_id_SET("DEMO", PH);
            p322.param_value_SET("DEMO", PH);
            p322.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64;
            p322.param_count = (ushort)(ushort)45135;
            p322.param_index = (ushort)(ushort)63670;
            LoopBackDemoChannel.instance.send(p322); //===============================
            PARAM_EXT_SET p323 = LoopBackDemoChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.target_system = (byte)(byte)153;
            p323.target_component = (byte)(byte)141;
            p323.param_id_SET("DEMO", PH);
            p323.param_value_SET("DEMO", PH);
            p323.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16;
            LoopBackDemoChannel.instance.send(p323); //===============================
            PARAM_EXT_ACK p324 = LoopBackDemoChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_id_SET("DEMO", PH);
            p324.param_value_SET("DEMO", PH);
            p324.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32;
            p324.param_result = (PARAM_ACK)PARAM_ACK.PARAM_ACK_IN_PROGRESS;
            LoopBackDemoChannel.instance.send(p324); //===============================
            OBSTACLE_DISTANCE p330 = LoopBackDemoChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.time_usec = (ulong)2823347885209233803L;
            p330.sensor_type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER;
            p330.distances_SET(new ushort[72], 0);
            p330.increment = (byte)(byte)221;
            p330.min_distance = (ushort)(ushort)62510;
            p330.max_distance = (ushort)(ushort)50153;
            LoopBackDemoChannel.instance.send(p330); //===============================
        }
    }
}

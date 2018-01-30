
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
            LoopBackDemoChannel.instance.OnCPU_LOADReceive += (src, ph, pack) =>
            {
                byte sensLoad = pack.sensLoad;
                byte ctrlLoad = pack.ctrlLoad;
                ushort batVolt = pack.batVolt;
            };
            LoopBackDemoChannel.instance.OnSENSOR_BIASReceive += (src, ph, pack) =>
            {
                float axBias = pack.axBias;
                float ayBias = pack.ayBias;
                float azBias = pack.azBias;
                float gxBias = pack.gxBias;
                float gyBias = pack.gyBias;
                float gzBias = pack.gzBias;
            };
            LoopBackDemoChannel.instance.OnDIAGNOSTICReceive += (src, ph, pack) =>
            {
                float diagFl1 = pack.diagFl1;
                float diagFl2 = pack.diagFl2;
                float diagFl3 = pack.diagFl3;
                short diagSh1 = pack.diagSh1;
                short diagSh2 = pack.diagSh2;
                short diagSh3 = pack.diagSh3;
            };
            LoopBackDemoChannel.instance.OnSLUGS_NAVIGATIONReceive += (src, ph, pack) =>
            {
                float u_m = pack.u_m;
                float phi_c = pack.phi_c;
                float theta_c = pack.theta_c;
                float psiDot_c = pack.psiDot_c;
                float ay_body = pack.ay_body;
                float totalDist = pack.totalDist;
                float dist2Go = pack.dist2Go;
                byte fromWP = pack.fromWP;
                byte toWP = pack.toWP;
                ushort h_c = pack.h_c;
            };
            LoopBackDemoChannel.instance.OnDATA_LOGReceive += (src, ph, pack) =>
            {
                float fl_1 = pack.fl_1;
                float fl_2 = pack.fl_2;
                float fl_3 = pack.fl_3;
                float fl_4 = pack.fl_4;
                float fl_5 = pack.fl_5;
                float fl_6 = pack.fl_6;
            };
            LoopBackDemoChannel.instance.OnGPS_DATE_TIMEReceive += (src, ph, pack) =>
            {
                byte year = pack.year;
                byte month = pack.month;
                byte day = pack.day;
                byte hour = pack.hour;
                byte min = pack.min;
                byte sec = pack.sec;
                byte clockStat = pack.clockStat;
                byte visSat = pack.visSat;
                byte useSat = pack.useSat;
                byte GppGl = pack.GppGl;
                byte sigUsedMask = pack.sigUsedMask;
                byte percentUsed = pack.percentUsed;
            };
            LoopBackDemoChannel.instance.OnMID_LVL_CMDSReceive += (src, ph, pack) =>
            {
                byte target = pack.target;
                float hCommand = pack.hCommand;
                float uCommand = pack.uCommand;
                float rCommand = pack.rCommand;
            };
            LoopBackDemoChannel.instance.OnCTRL_SRFC_PTReceive += (src, ph, pack) =>
            {
                byte target = pack.target;
                ushort bitfieldPt = pack.bitfieldPt;
            };
            LoopBackDemoChannel.instance.OnSLUGS_CAMERA_ORDERReceive += (src, ph, pack) =>
            {
                byte target = pack.target;
                sbyte pan = pack.pan;
                sbyte tilt = pack.tilt;
                sbyte zoom = pack.zoom;
                sbyte moveHome = pack.moveHome;
            };
            LoopBackDemoChannel.instance.OnCONTROL_SURFACEReceive += (src, ph, pack) =>
            {
                byte target = pack.target;
                byte idSurface = pack.idSurface;
                float mControl = pack.mControl;
                float bControl = pack.bControl;
            };
            LoopBackDemoChannel.instance.OnSLUGS_MOBILE_LOCATIONReceive += (src, ph, pack) =>
            {
                byte target = pack.target;
                float latitude = pack.latitude;
                float longitude = pack.longitude;
            };
            LoopBackDemoChannel.instance.OnSLUGS_CONFIGURATION_CAMERAReceive += (src, ph, pack) =>
            {
                byte target = pack.target;
                byte idOrder = pack.idOrder;
                byte order = pack.order;
            };
            LoopBackDemoChannel.instance.OnISR_LOCATIONReceive += (src, ph, pack) =>
            {
                byte target = pack.target;
                float latitude = pack.latitude;
                float longitude = pack.longitude;
                float height = pack.height;
                byte option1 = pack.option1;
                byte option2 = pack.option2;
                byte option3 = pack.option3;
            };
            LoopBackDemoChannel.instance.OnVOLT_SENSORReceive += (src, ph, pack) =>
            {
                byte r2Type = pack.r2Type;
                ushort voltage = pack.voltage;
                ushort reading2 = pack.reading2;
            };
            LoopBackDemoChannel.instance.OnPTZ_STATUSReceive += (src, ph, pack) =>
            {
                byte zoom = pack.zoom;
                short pan = pack.pan;
                short tilt = pack.tilt;
            };
            LoopBackDemoChannel.instance.OnUAV_STATUSReceive += (src, ph, pack) =>
            {
                byte target = pack.target;
                float latitude = pack.latitude;
                float longitude = pack.longitude;
                float altitude = pack.altitude;
                float speed = pack.speed;
                float course = pack.course;
            };
            LoopBackDemoChannel.instance.OnSTATUS_GPSReceive += (src, ph, pack) =>
            {
                ushort csFails = pack.csFails;
                byte gpsQuality = pack.gpsQuality;
                byte msgsType = pack.msgsType;
                byte posStatus = pack.posStatus;
                float magVar = pack.magVar;
                sbyte magDir = pack.magDir;
                byte modeInd = pack.modeInd;
            };
            LoopBackDemoChannel.instance.OnNOVATEL_DIAGReceive += (src, ph, pack) =>
            {
                byte timeStatus = pack.timeStatus;
                uint receiverStatus = pack.receiverStatus;
                byte solStatus = pack.solStatus;
                byte posType = pack.posType;
                byte velType = pack.velType;
                float posSolAge = pack.posSolAge;
                ushort csFails = pack.csFails;
            };
            LoopBackDemoChannel.instance.OnSENSOR_DIAGReceive += (src, ph, pack) =>
            {
                float float1 = pack.float1;
                float float2 = pack.float2;
                short int1 = pack.int1;
                sbyte char1 = pack.char1;
            };
            LoopBackDemoChannel.instance.OnBOOTReceive += (src, ph, pack) =>
            {
                uint version = pack.version;
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
            p0.type = (MAV_TYPE)MAV_TYPE.MAV_TYPE_VTOL_RESERVED2;
            p0.autopilot = (MAV_AUTOPILOT)MAV_AUTOPILOT.MAV_AUTOPILOT_GENERIC;
            p0.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED;
            p0.custom_mode = (uint)1935895103U;
            p0.system_status = (MAV_STATE)MAV_STATE.MAV_STATE_POWEROFF;
            p0.mavlink_version = (byte)(byte)126;
            LoopBackDemoChannel.instance.send(p0); //===============================
            SYS_STATUS p1 = LoopBackDemoChannel.new_SYS_STATUS();
            PH.setPack(p1);
            p1.onboard_control_sensors_present = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION;
            p1.onboard_control_sensors_enabled = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING;
            p1.onboard_control_sensors_health = (MAV_SYS_STATUS_SENSOR)MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
            p1.load = (ushort)(ushort)39511;
            p1.voltage_battery = (ushort)(ushort)55771;
            p1.current_battery = (short)(short) -23789;
            p1.battery_remaining = (sbyte)(sbyte)50;
            p1.drop_rate_comm = (ushort)(ushort)25844;
            p1.errors_comm = (ushort)(ushort)43871;
            p1.errors_count1 = (ushort)(ushort)17487;
            p1.errors_count2 = (ushort)(ushort)58116;
            p1.errors_count3 = (ushort)(ushort)26400;
            p1.errors_count4 = (ushort)(ushort)57634;
            LoopBackDemoChannel.instance.send(p1); //===============================
            SYSTEM_TIME p2 = LoopBackDemoChannel.new_SYSTEM_TIME();
            PH.setPack(p2);
            p2.time_unix_usec = (ulong)2149427872630264349L;
            p2.time_boot_ms = (uint)1184936585U;
            LoopBackDemoChannel.instance.send(p2); //===============================
            POSITION_TARGET_LOCAL_NED p3 = LoopBackDemoChannel.new_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.time_boot_ms = (uint)3667156862U;
            p3.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p3.type_mask = (ushort)(ushort)6632;
            p3.x = (float)1.4413235E38F;
            p3.y = (float)1.2929777E38F;
            p3.z = (float)2.724074E38F;
            p3.vx = (float)1.2389788E38F;
            p3.vy = (float)2.7821777E37F;
            p3.vz = (float) -8.205539E37F;
            p3.afx = (float) -1.1871656E38F;
            p3.afy = (float)7.747288E37F;
            p3.afz = (float) -1.476352E38F;
            p3.yaw = (float)2.2370482E38F;
            p3.yaw_rate = (float) -2.1570008E38F;
            LoopBackDemoChannel.instance.send(p3); //===============================
            PING p4 = LoopBackDemoChannel.new_PING();
            PH.setPack(p4);
            p4.time_usec = (ulong)2883798515106232025L;
            p4.seq = (uint)841312941U;
            p4.target_system = (byte)(byte)64;
            p4.target_component = (byte)(byte)74;
            LoopBackDemoChannel.instance.send(p4); //===============================
            CHANGE_OPERATOR_CONTROL p5 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL();
            PH.setPack(p5);
            p5.target_system = (byte)(byte)25;
            p5.control_request = (byte)(byte)36;
            p5.version = (byte)(byte)153;
            p5.passkey_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p5); //===============================
            CHANGE_OPERATOR_CONTROL_ACK p6 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL_ACK();
            PH.setPack(p6);
            p6.gcs_system_id = (byte)(byte)27;
            p6.control_request = (byte)(byte)205;
            p6.ack = (byte)(byte)165;
            LoopBackDemoChannel.instance.send(p6); //===============================
            AUTH_KEY p7 = LoopBackDemoChannel.new_AUTH_KEY();
            PH.setPack(p7);
            p7.key_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p7); //===============================
            SET_MODE p11 = LoopBackDemoChannel.new_SET_MODE();
            PH.setPack(p11);
            p11.target_system = (byte)(byte)142;
            p11.base_mode = (MAV_MODE)MAV_MODE.MAV_MODE_GUIDED_DISARMED;
            p11.custom_mode = (uint)3284031925U;
            LoopBackDemoChannel.instance.send(p11); //===============================
            PARAM_REQUEST_READ p20 = LoopBackDemoChannel.new_PARAM_REQUEST_READ();
            PH.setPack(p20);
            p20.target_system = (byte)(byte)195;
            p20.target_component = (byte)(byte)210;
            p20.param_id_SET("DEMO", PH);
            p20.param_index = (short)(short) -4543;
            LoopBackDemoChannel.instance.send(p20); //===============================
            PARAM_REQUEST_LIST p21 = LoopBackDemoChannel.new_PARAM_REQUEST_LIST();
            PH.setPack(p21);
            p21.target_system = (byte)(byte)119;
            p21.target_component = (byte)(byte)129;
            LoopBackDemoChannel.instance.send(p21); //===============================
            PARAM_VALUE p22 = LoopBackDemoChannel.new_PARAM_VALUE();
            PH.setPack(p22);
            p22.param_id_SET("DEMO", PH);
            p22.param_value = (float)5.5495036E37F;
            p22.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL64;
            p22.param_count = (ushort)(ushort)18657;
            p22.param_index = (ushort)(ushort)51584;
            LoopBackDemoChannel.instance.send(p22); //===============================
            PARAM_SET p23 = LoopBackDemoChannel.new_PARAM_SET();
            PH.setPack(p23);
            p23.target_system = (byte)(byte)66;
            p23.target_component = (byte)(byte)184;
            p23.param_id_SET("DEMO", PH);
            p23.param_value = (float)2.995175E38F;
            p23.param_type = (MAV_PARAM_TYPE)MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL64;
            LoopBackDemoChannel.instance.send(p23); //===============================
            GPS_RAW_INT p24 = LoopBackDemoChannel.new_GPS_RAW_INT();
            PH.setPack(p24);
            p24.time_usec = (ulong)1642459439427855088L;
            p24.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS;
            p24.lat = (int) -1866103306;
            p24.lon = (int) -167672274;
            p24.alt = (int)1826057662;
            p24.eph = (ushort)(ushort)32467;
            p24.epv = (ushort)(ushort)7365;
            p24.vel = (ushort)(ushort)60535;
            p24.cog = (ushort)(ushort)35071;
            p24.satellites_visible = (byte)(byte)242;
            p24.alt_ellipsoid_SET((int)201595094, PH);
            p24.h_acc_SET((uint)2754586420U, PH);
            p24.v_acc_SET((uint)2082620178U, PH);
            p24.vel_acc_SET((uint)1030481715U, PH);
            p24.hdg_acc_SET((uint)2984715081U, PH);
            LoopBackDemoChannel.instance.send(p24); //===============================
            GPS_STATUS p25 = LoopBackDemoChannel.new_GPS_STATUS();
            PH.setPack(p25);
            p25.satellites_visible = (byte)(byte)124;
            p25.satellite_prn_SET(new byte[20], 0);
            p25.satellite_used_SET(new byte[20], 0);
            p25.satellite_elevation_SET(new byte[20], 0);
            p25.satellite_azimuth_SET(new byte[20], 0);
            p25.satellite_snr_SET(new byte[20], 0);
            LoopBackDemoChannel.instance.send(p25); //===============================
            SCALED_IMU p26 = LoopBackDemoChannel.new_SCALED_IMU();
            PH.setPack(p26);
            p26.time_boot_ms = (uint)2541188511U;
            p26.xacc = (short)(short) -12204;
            p26.yacc = (short)(short) -27409;
            p26.zacc = (short)(short)22510;
            p26.xgyro = (short)(short) -10481;
            p26.ygyro = (short)(short) -19449;
            p26.zgyro = (short)(short)23534;
            p26.xmag = (short)(short)2156;
            p26.ymag = (short)(short)520;
            p26.zmag = (short)(short)4906;
            LoopBackDemoChannel.instance.send(p26); //===============================
            RAW_IMU p27 = LoopBackDemoChannel.new_RAW_IMU();
            PH.setPack(p27);
            p27.time_usec = (ulong)8492066875805953327L;
            p27.xacc = (short)(short)19328;
            p27.yacc = (short)(short)5243;
            p27.zacc = (short)(short) -11610;
            p27.xgyro = (short)(short) -20718;
            p27.ygyro = (short)(short)22603;
            p27.zgyro = (short)(short)22391;
            p27.xmag = (short)(short)25958;
            p27.ymag = (short)(short) -24758;
            p27.zmag = (short)(short) -28057;
            LoopBackDemoChannel.instance.send(p27); //===============================
            RAW_PRESSURE p28 = LoopBackDemoChannel.new_RAW_PRESSURE();
            PH.setPack(p28);
            p28.time_usec = (ulong)68662036242949077L;
            p28.press_abs = (short)(short) -30426;
            p28.press_diff1 = (short)(short)9785;
            p28.press_diff2 = (short)(short) -28371;
            p28.temperature = (short)(short) -2744;
            LoopBackDemoChannel.instance.send(p28); //===============================
            SCALED_PRESSURE p29 = LoopBackDemoChannel.new_SCALED_PRESSURE();
            PH.setPack(p29);
            p29.time_boot_ms = (uint)4203370654U;
            p29.press_abs = (float) -1.9350114E38F;
            p29.press_diff = (float) -1.4632114E38F;
            p29.temperature = (short)(short)26166;
            LoopBackDemoChannel.instance.send(p29); //===============================
            ATTITUDE p30 = LoopBackDemoChannel.new_ATTITUDE();
            PH.setPack(p30);
            p30.time_boot_ms = (uint)350258339U;
            p30.roll = (float)3.0165156E37F;
            p30.pitch = (float)2.6959638E38F;
            p30.yaw = (float)1.3644903E38F;
            p30.rollspeed = (float)3.760145E37F;
            p30.pitchspeed = (float)4.4959054E37F;
            p30.yawspeed = (float) -3.1905837E38F;
            LoopBackDemoChannel.instance.send(p30); //===============================
            ATTITUDE_QUATERNION p31 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION();
            PH.setPack(p31);
            p31.time_boot_ms = (uint)3031307574U;
            p31.q1 = (float)6.395836E37F;
            p31.q2 = (float)3.009733E38F;
            p31.q3 = (float)1.0942435E38F;
            p31.q4 = (float)3.1092455E38F;
            p31.rollspeed = (float)1.3986037E38F;
            p31.pitchspeed = (float)1.01975576E37F;
            p31.yawspeed = (float) -1.8119556E38F;
            LoopBackDemoChannel.instance.send(p31); //===============================
            LOCAL_POSITION_NED p32 = LoopBackDemoChannel.new_LOCAL_POSITION_NED();
            PH.setPack(p32);
            p32.time_boot_ms = (uint)722198065U;
            p32.x = (float)2.5217416E38F;
            p32.y = (float)2.6587821E38F;
            p32.z = (float) -2.664008E38F;
            p32.vx = (float)5.883252E37F;
            p32.vy = (float) -2.4355412E38F;
            p32.vz = (float) -2.0912999E37F;
            LoopBackDemoChannel.instance.send(p32); //===============================
            GLOBAL_POSITION_INT p33 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT();
            PH.setPack(p33);
            p33.time_boot_ms = (uint)2620421976U;
            p33.lat = (int)1843753520;
            p33.lon = (int)1972701138;
            p33.alt = (int) -1315757840;
            p33.relative_alt = (int)475152601;
            p33.vx = (short)(short) -16931;
            p33.vy = (short)(short) -96;
            p33.vz = (short)(short)15656;
            p33.hdg = (ushort)(ushort)56821;
            LoopBackDemoChannel.instance.send(p33); //===============================
            RC_CHANNELS_SCALED p34 = LoopBackDemoChannel.new_RC_CHANNELS_SCALED();
            PH.setPack(p34);
            p34.time_boot_ms = (uint)75373186U;
            p34.port = (byte)(byte)230;
            p34.chan1_scaled = (short)(short)17164;
            p34.chan2_scaled = (short)(short)23309;
            p34.chan3_scaled = (short)(short) -1512;
            p34.chan4_scaled = (short)(short)11712;
            p34.chan5_scaled = (short)(short)12957;
            p34.chan6_scaled = (short)(short)31910;
            p34.chan7_scaled = (short)(short)1764;
            p34.chan8_scaled = (short)(short)1781;
            p34.rssi = (byte)(byte)17;
            LoopBackDemoChannel.instance.send(p34); //===============================
            RC_CHANNELS_RAW p35 = LoopBackDemoChannel.new_RC_CHANNELS_RAW();
            PH.setPack(p35);
            p35.time_boot_ms = (uint)1342609785U;
            p35.port = (byte)(byte)160;
            p35.chan1_raw = (ushort)(ushort)13298;
            p35.chan2_raw = (ushort)(ushort)29065;
            p35.chan3_raw = (ushort)(ushort)52314;
            p35.chan4_raw = (ushort)(ushort)55796;
            p35.chan5_raw = (ushort)(ushort)15712;
            p35.chan6_raw = (ushort)(ushort)23213;
            p35.chan7_raw = (ushort)(ushort)64341;
            p35.chan8_raw = (ushort)(ushort)33717;
            p35.rssi = (byte)(byte)209;
            LoopBackDemoChannel.instance.send(p35); //===============================
            SERVO_OUTPUT_RAW p36 = LoopBackDemoChannel.new_SERVO_OUTPUT_RAW();
            PH.setPack(p36);
            p36.time_usec = (uint)2622370709U;
            p36.port = (byte)(byte)254;
            p36.servo1_raw = (ushort)(ushort)41808;
            p36.servo2_raw = (ushort)(ushort)17485;
            p36.servo3_raw = (ushort)(ushort)7940;
            p36.servo4_raw = (ushort)(ushort)32211;
            p36.servo5_raw = (ushort)(ushort)60014;
            p36.servo6_raw = (ushort)(ushort)2730;
            p36.servo7_raw = (ushort)(ushort)46645;
            p36.servo8_raw = (ushort)(ushort)23370;
            p36.servo9_raw_SET((ushort)(ushort)11877, PH);
            p36.servo10_raw_SET((ushort)(ushort)63742, PH);
            p36.servo11_raw_SET((ushort)(ushort)28172, PH);
            p36.servo12_raw_SET((ushort)(ushort)63961, PH);
            p36.servo13_raw_SET((ushort)(ushort)42566, PH);
            p36.servo14_raw_SET((ushort)(ushort)16407, PH);
            p36.servo15_raw_SET((ushort)(ushort)20567, PH);
            p36.servo16_raw_SET((ushort)(ushort)2495, PH);
            LoopBackDemoChannel.instance.send(p36); //===============================
            MISSION_REQUEST_PARTIAL_LIST p37 = LoopBackDemoChannel.new_MISSION_REQUEST_PARTIAL_LIST();
            PH.setPack(p37);
            p37.target_system = (byte)(byte)236;
            p37.target_component = (byte)(byte)2;
            p37.start_index = (short)(short) -29213;
            p37.end_index = (short)(short) -32711;
            p37.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            LoopBackDemoChannel.instance.send(p37); //===============================
            MISSION_WRITE_PARTIAL_LIST p38 = LoopBackDemoChannel.new_MISSION_WRITE_PARTIAL_LIST();
            PH.setPack(p38);
            p38.target_system = (byte)(byte)152;
            p38.target_component = (byte)(byte)76;
            p38.start_index = (short)(short)8746;
            p38.end_index = (short)(short) -96;
            p38.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            LoopBackDemoChannel.instance.send(p38); //===============================
            MISSION_ITEM p39 = LoopBackDemoChannel.new_MISSION_ITEM();
            PH.setPack(p39);
            p39.target_system = (byte)(byte)82;
            p39.target_component = (byte)(byte)213;
            p39.seq = (ushort)(ushort)8371;
            p39.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_INT;
            p39.command = (MAV_CMD)MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL;
            p39.current = (byte)(byte)147;
            p39.autocontinue = (byte)(byte)76;
            p39.param1 = (float)4.2694153E37F;
            p39.param2 = (float)8.5576164E36F;
            p39.param3 = (float)1.6123072E38F;
            p39.param4 = (float) -2.7406673E38F;
            p39.x = (float) -1.8272922E38F;
            p39.y = (float)3.3161504E38F;
            p39.z = (float)9.941935E37F;
            p39.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            LoopBackDemoChannel.instance.send(p39); //===============================
            MISSION_REQUEST p40 = LoopBackDemoChannel.new_MISSION_REQUEST();
            PH.setPack(p40);
            p40.target_system = (byte)(byte)130;
            p40.target_component = (byte)(byte)163;
            p40.seq = (ushort)(ushort)25208;
            p40.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            LoopBackDemoChannel.instance.send(p40); //===============================
            MISSION_SET_CURRENT p41 = LoopBackDemoChannel.new_MISSION_SET_CURRENT();
            PH.setPack(p41);
            p41.target_system = (byte)(byte)75;
            p41.target_component = (byte)(byte)165;
            p41.seq = (ushort)(ushort)31765;
            LoopBackDemoChannel.instance.send(p41); //===============================
            MISSION_CURRENT p42 = LoopBackDemoChannel.new_MISSION_CURRENT();
            PH.setPack(p42);
            p42.seq = (ushort)(ushort)9431;
            LoopBackDemoChannel.instance.send(p42); //===============================
            MISSION_REQUEST_LIST p43 = LoopBackDemoChannel.new_MISSION_REQUEST_LIST();
            PH.setPack(p43);
            p43.target_system = (byte)(byte)173;
            p43.target_component = (byte)(byte)142;
            p43.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            LoopBackDemoChannel.instance.send(p43); //===============================
            MISSION_COUNT p44 = LoopBackDemoChannel.new_MISSION_COUNT();
            PH.setPack(p44);
            p44.target_system = (byte)(byte)91;
            p44.target_component = (byte)(byte)179;
            p44.count = (ushort)(ushort)12627;
            p44.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            LoopBackDemoChannel.instance.send(p44); //===============================
            MISSION_CLEAR_ALL p45 = LoopBackDemoChannel.new_MISSION_CLEAR_ALL();
            PH.setPack(p45);
            p45.target_system = (byte)(byte)132;
            p45.target_component = (byte)(byte)252;
            p45.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            LoopBackDemoChannel.instance.send(p45); //===============================
            MISSION_ITEM_REACHED p46 = LoopBackDemoChannel.new_MISSION_ITEM_REACHED();
            PH.setPack(p46);
            p46.seq = (ushort)(ushort)27214;
            LoopBackDemoChannel.instance.send(p46); //===============================
            MISSION_ACK p47 = LoopBackDemoChannel.new_MISSION_ACK();
            PH.setPack(p47);
            p47.target_system = (byte)(byte)9;
            p47.target_component = (byte)(byte)98;
            p47.type = (MAV_MISSION_RESULT)MAV_MISSION_RESULT.MAV_MISSION_ACCEPTED;
            p47.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            LoopBackDemoChannel.instance.send(p47); //===============================
            SET_GPS_GLOBAL_ORIGIN p48 = LoopBackDemoChannel.new_SET_GPS_GLOBAL_ORIGIN();
            PH.setPack(p48);
            p48.target_system = (byte)(byte)231;
            p48.latitude = (int)1078448244;
            p48.longitude = (int) -1821114849;
            p48.altitude = (int) -454868;
            p48.time_usec_SET((ulong)3158887797452133392L, PH);
            LoopBackDemoChannel.instance.send(p48); //===============================
            GPS_GLOBAL_ORIGIN p49 = LoopBackDemoChannel.new_GPS_GLOBAL_ORIGIN();
            PH.setPack(p49);
            p49.latitude = (int)1153582831;
            p49.longitude = (int) -1492136359;
            p49.altitude = (int) -1248463915;
            p49.time_usec_SET((ulong)109277806871183238L, PH);
            LoopBackDemoChannel.instance.send(p49); //===============================
            PARAM_MAP_RC p50 = LoopBackDemoChannel.new_PARAM_MAP_RC();
            PH.setPack(p50);
            p50.target_system = (byte)(byte)189;
            p50.target_component = (byte)(byte)232;
            p50.param_id_SET("DEMO", PH);
            p50.param_index = (short)(short)31109;
            p50.parameter_rc_channel_index = (byte)(byte)127;
            p50.param_value0 = (float) -2.9031917E37F;
            p50.scale = (float) -1.4099675E38F;
            p50.param_value_min = (float) -2.5315082E38F;
            p50.param_value_max = (float)1.1321578E38F;
            LoopBackDemoChannel.instance.send(p50); //===============================
            MISSION_REQUEST_INT p51 = LoopBackDemoChannel.new_MISSION_REQUEST_INT();
            PH.setPack(p51);
            p51.target_system = (byte)(byte)20;
            p51.target_component = (byte)(byte)239;
            p51.seq = (ushort)(ushort)65107;
            p51.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            LoopBackDemoChannel.instance.send(p51); //===============================
            SAFETY_SET_ALLOWED_AREA p54 = LoopBackDemoChannel.new_SAFETY_SET_ALLOWED_AREA();
            PH.setPack(p54);
            p54.target_system = (byte)(byte)54;
            p54.target_component = (byte)(byte)60;
            p54.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL;
            p54.p1x = (float)1.1083228E38F;
            p54.p1y = (float)2.6636317E38F;
            p54.p1z = (float)1.4472635E38F;
            p54.p2x = (float)2.653524E38F;
            p54.p2y = (float) -1.4333463E38F;
            p54.p2z = (float)2.7903667E38F;
            LoopBackDemoChannel.instance.send(p54); //===============================
            SAFETY_ALLOWED_AREA p55 = LoopBackDemoChannel.new_SAFETY_ALLOWED_AREA();
            PH.setPack(p55);
            p55.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL;
            p55.p1x = (float) -1.7790348E38F;
            p55.p1y = (float)1.9867484E38F;
            p55.p1z = (float) -1.583148E38F;
            p55.p2x = (float)1.5605681E37F;
            p55.p2y = (float) -3.0143038E38F;
            p55.p2z = (float)2.9643547E38F;
            LoopBackDemoChannel.instance.send(p55); //===============================
            ATTITUDE_QUATERNION_COV p61 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION_COV();
            PH.setPack(p61);
            p61.time_usec = (ulong)2702008556675351313L;
            p61.q_SET(new float[4], 0);
            p61.rollspeed = (float) -2.3586708E38F;
            p61.pitchspeed = (float)6.25857E36F;
            p61.yawspeed = (float) -3.283247E38F;
            p61.covariance_SET(new float[9], 0);
            LoopBackDemoChannel.instance.send(p61); //===============================
            NAV_CONTROLLER_OUTPUT p62 = LoopBackDemoChannel.new_NAV_CONTROLLER_OUTPUT();
            PH.setPack(p62);
            p62.nav_roll = (float)6.608401E36F;
            p62.nav_pitch = (float) -1.5658434E38F;
            p62.nav_bearing = (short)(short)11013;
            p62.target_bearing = (short)(short)21387;
            p62.wp_dist = (ushort)(ushort)29424;
            p62.alt_error = (float)3.377389E38F;
            p62.aspd_error = (float) -9.766986E37F;
            p62.xtrack_error = (float) -3.0038E38F;
            LoopBackDemoChannel.instance.send(p62); //===============================
            GLOBAL_POSITION_INT_COV p63 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT_COV();
            PH.setPack(p63);
            p63.time_usec = (ulong)4078941210388412161L;
            p63.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE;
            p63.lat = (int)1007997389;
            p63.lon = (int)37356347;
            p63.alt = (int)595909669;
            p63.relative_alt = (int)1451965645;
            p63.vx = (float) -3.0027138E38F;
            p63.vy = (float)3.363838E38F;
            p63.vz = (float) -1.2650442E38F;
            p63.covariance_SET(new float[36], 0);
            LoopBackDemoChannel.instance.send(p63); //===============================
            LOCAL_POSITION_NED_COV p64 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_COV();
            PH.setPack(p64);
            p64.time_usec = (ulong)2767716163210396781L;
            p64.estimator_type = (MAV_ESTIMATOR_TYPE)MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS;
            p64.x = (float) -2.953276E38F;
            p64.y = (float) -3.0511687E38F;
            p64.z = (float)6.7099064E37F;
            p64.vx = (float) -1.5990273E38F;
            p64.vy = (float)1.2847985E38F;
            p64.vz = (float) -1.1587389E38F;
            p64.ax = (float) -3.2971815E38F;
            p64.ay = (float) -2.4687518E38F;
            p64.az = (float)2.6617176E38F;
            p64.covariance_SET(new float[45], 0);
            LoopBackDemoChannel.instance.send(p64); //===============================
            RC_CHANNELS p65 = LoopBackDemoChannel.new_RC_CHANNELS();
            PH.setPack(p65);
            p65.time_boot_ms = (uint)3443030833U;
            p65.chancount = (byte)(byte)96;
            p65.chan1_raw = (ushort)(ushort)34174;
            p65.chan2_raw = (ushort)(ushort)11448;
            p65.chan3_raw = (ushort)(ushort)60987;
            p65.chan4_raw = (ushort)(ushort)56362;
            p65.chan5_raw = (ushort)(ushort)32150;
            p65.chan6_raw = (ushort)(ushort)11288;
            p65.chan7_raw = (ushort)(ushort)46497;
            p65.chan8_raw = (ushort)(ushort)20751;
            p65.chan9_raw = (ushort)(ushort)11281;
            p65.chan10_raw = (ushort)(ushort)4160;
            p65.chan11_raw = (ushort)(ushort)60946;
            p65.chan12_raw = (ushort)(ushort)61252;
            p65.chan13_raw = (ushort)(ushort)62155;
            p65.chan14_raw = (ushort)(ushort)57863;
            p65.chan15_raw = (ushort)(ushort)25783;
            p65.chan16_raw = (ushort)(ushort)19233;
            p65.chan17_raw = (ushort)(ushort)14668;
            p65.chan18_raw = (ushort)(ushort)62432;
            p65.rssi = (byte)(byte)98;
            LoopBackDemoChannel.instance.send(p65); //===============================
            REQUEST_DATA_STREAM p66 = LoopBackDemoChannel.new_REQUEST_DATA_STREAM();
            PH.setPack(p66);
            p66.target_system = (byte)(byte)203;
            p66.target_component = (byte)(byte)55;
            p66.req_stream_id = (byte)(byte)108;
            p66.req_message_rate = (ushort)(ushort)12140;
            p66.start_stop = (byte)(byte)25;
            LoopBackDemoChannel.instance.send(p66); //===============================
            DATA_STREAM p67 = LoopBackDemoChannel.new_DATA_STREAM();
            PH.setPack(p67);
            p67.stream_id = (byte)(byte)121;
            p67.message_rate = (ushort)(ushort)10321;
            p67.on_off = (byte)(byte)213;
            LoopBackDemoChannel.instance.send(p67); //===============================
            MANUAL_CONTROL p69 = LoopBackDemoChannel.new_MANUAL_CONTROL();
            PH.setPack(p69);
            p69.target = (byte)(byte)129;
            p69.x = (short)(short) -29221;
            p69.y = (short)(short)3832;
            p69.z = (short)(short) -7552;
            p69.r = (short)(short) -5129;
            p69.buttons = (ushort)(ushort)16133;
            LoopBackDemoChannel.instance.send(p69); //===============================
            RC_CHANNELS_OVERRIDE p70 = LoopBackDemoChannel.new_RC_CHANNELS_OVERRIDE();
            PH.setPack(p70);
            p70.target_system = (byte)(byte)37;
            p70.target_component = (byte)(byte)152;
            p70.chan1_raw = (ushort)(ushort)60896;
            p70.chan2_raw = (ushort)(ushort)16472;
            p70.chan3_raw = (ushort)(ushort)45717;
            p70.chan4_raw = (ushort)(ushort)50686;
            p70.chan5_raw = (ushort)(ushort)15875;
            p70.chan6_raw = (ushort)(ushort)49154;
            p70.chan7_raw = (ushort)(ushort)1572;
            p70.chan8_raw = (ushort)(ushort)4051;
            LoopBackDemoChannel.instance.send(p70); //===============================
            MISSION_ITEM_INT p73 = LoopBackDemoChannel.new_MISSION_ITEM_INT();
            PH.setPack(p73);
            p73.target_system = (byte)(byte)40;
            p73.target_component = (byte)(byte)46;
            p73.seq = (ushort)(ushort)54526;
            p73.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p73.command = (MAV_CMD)MAV_CMD.MAV_CMD_NAV_VTOL_LAND;
            p73.current = (byte)(byte)165;
            p73.autocontinue = (byte)(byte)17;
            p73.param1 = (float)2.6824065E37F;
            p73.param2 = (float)2.5041006E38F;
            p73.param3 = (float) -2.8868067E38F;
            p73.param4 = (float)2.975676E38F;
            p73.x = (int) -1357397105;
            p73.y = (int)601926918;
            p73.z = (float)1.8942957E38F;
            p73.mission_type = (MAV_MISSION_TYPE)MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            LoopBackDemoChannel.instance.send(p73); //===============================
            VFR_HUD p74 = LoopBackDemoChannel.new_VFR_HUD();
            PH.setPack(p74);
            p74.airspeed = (float)2.4271852E38F;
            p74.groundspeed = (float) -8.943688E37F;
            p74.heading = (short)(short)24889;
            p74.throttle = (ushort)(ushort)54089;
            p74.alt = (float)8.3629035E37F;
            p74.climb = (float)1.2279102E38F;
            LoopBackDemoChannel.instance.send(p74); //===============================
            COMMAND_INT p75 = LoopBackDemoChannel.new_COMMAND_INT();
            PH.setPack(p75);
            p75.target_system = (byte)(byte)0;
            p75.target_component = (byte)(byte)170;
            p75.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_INT;
            p75.command = (MAV_CMD)MAV_CMD.MAV_CMD_USER_2;
            p75.current = (byte)(byte)216;
            p75.autocontinue = (byte)(byte)191;
            p75.param1 = (float)1.8246429E38F;
            p75.param2 = (float)1.0190735E38F;
            p75.param3 = (float)2.3694564E38F;
            p75.param4 = (float)2.2150413E38F;
            p75.x = (int)1866946044;
            p75.y = (int) -519866922;
            p75.z = (float)9.22481E37F;
            LoopBackDemoChannel.instance.send(p75); //===============================
            COMMAND_LONG p76 = LoopBackDemoChannel.new_COMMAND_LONG();
            PH.setPack(p76);
            p76.target_system = (byte)(byte)88;
            p76.target_component = (byte)(byte)125;
            p76.command = (MAV_CMD)MAV_CMD.MAV_CMD_DO_GUIDED_MASTER;
            p76.confirmation = (byte)(byte)159;
            p76.param1 = (float) -2.56049E38F;
            p76.param2 = (float) -4.811354E37F;
            p76.param3 = (float) -2.0846569E38F;
            p76.param4 = (float)2.6890672E38F;
            p76.param5 = (float)1.7182898E38F;
            p76.param6 = (float)2.7196103E38F;
            p76.param7 = (float) -1.05905695E36F;
            LoopBackDemoChannel.instance.send(p76); //===============================
            COMMAND_ACK p77 = LoopBackDemoChannel.new_COMMAND_ACK();
            PH.setPack(p77);
            p77.command = (MAV_CMD)MAV_CMD.MAV_CMD_NAV_LAST;
            p77.result = (MAV_RESULT)MAV_RESULT.MAV_RESULT_UNSUPPORTED;
            p77.progress_SET((byte)(byte)36, PH);
            p77.result_param2_SET((int)533646797, PH);
            p77.target_system_SET((byte)(byte)210, PH);
            p77.target_component_SET((byte)(byte)101, PH);
            LoopBackDemoChannel.instance.send(p77); //===============================
            MANUAL_SETPOINT p81 = LoopBackDemoChannel.new_MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.time_boot_ms = (uint)3356675096U;
            p81.roll = (float)1.4837955E38F;
            p81.pitch = (float)7.8960906E37F;
            p81.yaw = (float) -2.7749233E38F;
            p81.thrust = (float)1.4830399E38F;
            p81.mode_switch = (byte)(byte)55;
            p81.manual_override_switch = (byte)(byte)149;
            LoopBackDemoChannel.instance.send(p81); //===============================
            SET_ATTITUDE_TARGET p82 = LoopBackDemoChannel.new_SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.time_boot_ms = (uint)1876223775U;
            p82.target_system = (byte)(byte)146;
            p82.target_component = (byte)(byte)132;
            p82.type_mask = (byte)(byte)99;
            p82.q_SET(new float[4], 0);
            p82.body_roll_rate = (float)1.071658E38F;
            p82.body_pitch_rate = (float) -7.237023E36F;
            p82.body_yaw_rate = (float) -2.6939395E38F;
            p82.thrust = (float) -2.3184847E38F;
            LoopBackDemoChannel.instance.send(p82); //===============================
            ATTITUDE_TARGET p83 = LoopBackDemoChannel.new_ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.time_boot_ms = (uint)2137101659U;
            p83.type_mask = (byte)(byte)185;
            p83.q_SET(new float[4], 0);
            p83.body_roll_rate = (float) -2.0102215E38F;
            p83.body_pitch_rate = (float) -2.790771E38F;
            p83.body_yaw_rate = (float)3.1874805E38F;
            p83.thrust = (float) -3.4999696E37F;
            LoopBackDemoChannel.instance.send(p83); //===============================
            SET_POSITION_TARGET_LOCAL_NED p84 = LoopBackDemoChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.time_boot_ms = (uint)594619269U;
            p84.target_system = (byte)(byte)233;
            p84.target_component = (byte)(byte)214;
            p84.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_INT;
            p84.type_mask = (ushort)(ushort)55429;
            p84.x = (float)2.9076798E38F;
            p84.y = (float)1.533965E38F;
            p84.z = (float) -2.0705426E37F;
            p84.vx = (float) -1.2079377E38F;
            p84.vy = (float)1.8992647E37F;
            p84.vz = (float)2.243746E38F;
            p84.afx = (float)2.57991E38F;
            p84.afy = (float) -2.317094E38F;
            p84.afz = (float) -1.7279621E38F;
            p84.yaw = (float) -3.364436E38F;
            p84.yaw_rate = (float) -9.649019E37F;
            LoopBackDemoChannel.instance.send(p84); //===============================
            SET_POSITION_TARGET_GLOBAL_INT p86 = LoopBackDemoChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.time_boot_ms = (uint)1573260603U;
            p86.target_system = (byte)(byte)103;
            p86.target_component = (byte)(byte)172;
            p86.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
            p86.type_mask = (ushort)(ushort)49215;
            p86.lat_int = (int)1141051579;
            p86.lon_int = (int)2002541612;
            p86.alt = (float)1.2765317E38F;
            p86.vx = (float)6.1783556E37F;
            p86.vy = (float) -2.6985247E38F;
            p86.vz = (float)2.7109126E38F;
            p86.afx = (float)1.2281352E38F;
            p86.afy = (float) -7.6206E37F;
            p86.afz = (float)1.7354348E38F;
            p86.yaw = (float) -1.0027894E38F;
            p86.yaw_rate = (float)5.4217477E37F;
            LoopBackDemoChannel.instance.send(p86); //===============================
            POSITION_TARGET_GLOBAL_INT p87 = LoopBackDemoChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.time_boot_ms = (uint)1401098550U;
            p87.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_MISSION;
            p87.type_mask = (ushort)(ushort)2569;
            p87.lat_int = (int) -1671319521;
            p87.lon_int = (int)2086762167;
            p87.alt = (float) -2.273439E38F;
            p87.vx = (float)1.750832E38F;
            p87.vy = (float) -2.9597084E38F;
            p87.vz = (float) -1.7206166E38F;
            p87.afx = (float)2.5696444E37F;
            p87.afy = (float) -2.1530212E38F;
            p87.afz = (float) -1.7638107E38F;
            p87.yaw = (float) -3.2448884E38F;
            p87.yaw_rate = (float) -1.6763722E38F;
            LoopBackDemoChannel.instance.send(p87); //===============================
            LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.time_boot_ms = (uint)1501623068U;
            p89.x = (float)2.6318353E38F;
            p89.y = (float)2.6990669E38F;
            p89.z = (float) -2.4006062E37F;
            p89.roll = (float)7.8233985E37F;
            p89.pitch = (float) -1.9933996E38F;
            p89.yaw = (float) -1.196141E38F;
            LoopBackDemoChannel.instance.send(p89); //===============================
            HIL_STATE p90 = LoopBackDemoChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.time_usec = (ulong)2219106206961586318L;
            p90.roll = (float) -3.1562896E38F;
            p90.pitch = (float)1.5154617E38F;
            p90.yaw = (float) -1.4504563E38F;
            p90.rollspeed = (float)2.543105E38F;
            p90.pitchspeed = (float) -9.0136087E36F;
            p90.yawspeed = (float) -4.6903944E37F;
            p90.lat = (int)1709191397;
            p90.lon = (int)897518522;
            p90.alt = (int)546954121;
            p90.vx = (short)(short)31634;
            p90.vy = (short)(short) -8436;
            p90.vz = (short)(short) -17600;
            p90.xacc = (short)(short) -11717;
            p90.yacc = (short)(short)27831;
            p90.zacc = (short)(short) -10813;
            LoopBackDemoChannel.instance.send(p90); //===============================
            HIL_CONTROLS p91 = LoopBackDemoChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.time_usec = (ulong)3763327228034430490L;
            p91.roll_ailerons = (float)2.3293475E38F;
            p91.pitch_elevator = (float) -2.0066785E38F;
            p91.yaw_rudder = (float)6.2394604E37F;
            p91.throttle = (float) -9.583067E37F;
            p91.aux1 = (float) -3.1158138E38F;
            p91.aux2 = (float) -1.883926E38F;
            p91.aux3 = (float) -2.6461922E37F;
            p91.aux4 = (float)9.238421E37F;
            p91.mode = (MAV_MODE)MAV_MODE.MAV_MODE_PREFLIGHT;
            p91.nav_mode = (byte)(byte)148;
            LoopBackDemoChannel.instance.send(p91); //===============================
            HIL_RC_INPUTS_RAW p92 = LoopBackDemoChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.time_usec = (ulong)9155183187343908349L;
            p92.chan1_raw = (ushort)(ushort)30944;
            p92.chan2_raw = (ushort)(ushort)39807;
            p92.chan3_raw = (ushort)(ushort)39471;
            p92.chan4_raw = (ushort)(ushort)29825;
            p92.chan5_raw = (ushort)(ushort)20831;
            p92.chan6_raw = (ushort)(ushort)14275;
            p92.chan7_raw = (ushort)(ushort)57687;
            p92.chan8_raw = (ushort)(ushort)7336;
            p92.chan9_raw = (ushort)(ushort)25902;
            p92.chan10_raw = (ushort)(ushort)36862;
            p92.chan11_raw = (ushort)(ushort)61057;
            p92.chan12_raw = (ushort)(ushort)62914;
            p92.rssi = (byte)(byte)25;
            LoopBackDemoChannel.instance.send(p92); //===============================
            HIL_ACTUATOR_CONTROLS p93 = LoopBackDemoChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.time_usec = (ulong)6998172820270650903L;
            p93.controls_SET(new float[16], 0);
            p93.mode = (MAV_MODE)MAV_MODE.MAV_MODE_TEST_ARMED;
            p93.flags = (ulong)3769242333974402810L;
            LoopBackDemoChannel.instance.send(p93); //===============================
            OPTICAL_FLOW p100 = LoopBackDemoChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.time_usec = (ulong)5416527450337743580L;
            p100.sensor_id = (byte)(byte)37;
            p100.flow_x = (short)(short)26188;
            p100.flow_y = (short)(short)27403;
            p100.flow_comp_m_x = (float) -2.7026374E38F;
            p100.flow_comp_m_y = (float) -1.2339228E38F;
            p100.quality = (byte)(byte)91;
            p100.ground_distance = (float) -2.7439762E38F;
            p100.flow_rate_x_SET((float)3.3687235E38F, PH);
            p100.flow_rate_y_SET((float) -8.2917857E37F, PH);
            LoopBackDemoChannel.instance.send(p100); //===============================
            GLOBAL_VISION_POSITION_ESTIMATE p101 = LoopBackDemoChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.usec = (ulong)2613766335856416896L;
            p101.x = (float)1.1136294E38F;
            p101.y = (float)2.110102E37F;
            p101.z = (float) -2.576896E38F;
            p101.roll = (float)4.8643404E37F;
            p101.pitch = (float)3.4975872E37F;
            p101.yaw = (float) -2.2770092E38F;
            LoopBackDemoChannel.instance.send(p101); //===============================
            VISION_POSITION_ESTIMATE p102 = LoopBackDemoChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.usec = (ulong)5558328090365184864L;
            p102.x = (float)1.0575937E38F;
            p102.y = (float)9.833491E37F;
            p102.z = (float)1.1113192E38F;
            p102.roll = (float) -4.6605484E37F;
            p102.pitch = (float) -2.6455197E38F;
            p102.yaw = (float)2.609968E37F;
            LoopBackDemoChannel.instance.send(p102); //===============================
            VISION_SPEED_ESTIMATE p103 = LoopBackDemoChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.usec = (ulong)273074739452809538L;
            p103.x = (float)6.090223E37F;
            p103.y = (float) -8.150993E37F;
            p103.z = (float) -3.3022002E38F;
            LoopBackDemoChannel.instance.send(p103); //===============================
            VICON_POSITION_ESTIMATE p104 = LoopBackDemoChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.usec = (ulong)8221238278971024221L;
            p104.x = (float)1.7382964E37F;
            p104.y = (float) -1.8026823E38F;
            p104.z = (float) -2.1039749E38F;
            p104.roll = (float) -1.1505346E38F;
            p104.pitch = (float)7.631712E37F;
            p104.yaw = (float) -1.5621045E38F;
            LoopBackDemoChannel.instance.send(p104); //===============================
            HIGHRES_IMU p105 = LoopBackDemoChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.time_usec = (ulong)7686437222104993828L;
            p105.xacc = (float)2.541543E38F;
            p105.yacc = (float) -9.604121E37F;
            p105.zacc = (float) -1.6229611E38F;
            p105.xgyro = (float) -5.4562673E37F;
            p105.ygyro = (float) -1.6604205E37F;
            p105.zgyro = (float) -2.1425349E36F;
            p105.xmag = (float) -2.5278089E38F;
            p105.ymag = (float)3.282356E38F;
            p105.zmag = (float)3.813931E37F;
            p105.abs_pressure = (float) -2.0502436E37F;
            p105.diff_pressure = (float)2.1303996E38F;
            p105.pressure_alt = (float) -1.9654697E38F;
            p105.temperature = (float)3.743278E36F;
            p105.fields_updated = (ushort)(ushort)62427;
            LoopBackDemoChannel.instance.send(p105); //===============================
            OPTICAL_FLOW_RAD p106 = LoopBackDemoChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.time_usec = (ulong)7058379777532185980L;
            p106.sensor_id = (byte)(byte)67;
            p106.integration_time_us = (uint)2600120552U;
            p106.integrated_x = (float) -5.5600504E37F;
            p106.integrated_y = (float) -5.437133E37F;
            p106.integrated_xgyro = (float)1.1992382E38F;
            p106.integrated_ygyro = (float) -4.141793E37F;
            p106.integrated_zgyro = (float)9.44877E37F;
            p106.temperature = (short)(short) -9838;
            p106.quality = (byte)(byte)86;
            p106.time_delta_distance_us = (uint)1853048450U;
            p106.distance = (float) -2.0985844E38F;
            LoopBackDemoChannel.instance.send(p106); //===============================
            HIL_SENSOR p107 = LoopBackDemoChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.time_usec = (ulong)8148328357313889052L;
            p107.xacc = (float) -5.981022E37F;
            p107.yacc = (float)9.42594E37F;
            p107.zacc = (float) -2.2553147E38F;
            p107.xgyro = (float)2.9032428E38F;
            p107.ygyro = (float)3.1651793E38F;
            p107.zgyro = (float) -2.2543101E38F;
            p107.xmag = (float)2.1919557E38F;
            p107.ymag = (float) -1.9798287E38F;
            p107.zmag = (float)2.0708626E38F;
            p107.abs_pressure = (float)3.3715632E37F;
            p107.diff_pressure = (float)3.4470305E37F;
            p107.pressure_alt = (float)1.1681518E38F;
            p107.temperature = (float) -2.3514821E38F;
            p107.fields_updated = (uint)747219631U;
            LoopBackDemoChannel.instance.send(p107); //===============================
            SIM_STATE p108 = LoopBackDemoChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.q1 = (float)3.1236943E38F;
            p108.q2 = (float) -1.9210332E38F;
            p108.q3 = (float)1.04822034E37F;
            p108.q4 = (float) -1.810809E38F;
            p108.roll = (float) -1.0073774E38F;
            p108.pitch = (float) -2.7999376E38F;
            p108.yaw = (float) -2.511747E38F;
            p108.xacc = (float)1.7842701E38F;
            p108.yacc = (float) -2.3823351E38F;
            p108.zacc = (float) -4.460905E37F;
            p108.xgyro = (float)1.9887312E38F;
            p108.ygyro = (float)1.939632E38F;
            p108.zgyro = (float)3.244911E38F;
            p108.lat = (float) -3.2847135E38F;
            p108.lon = (float) -2.4033197E38F;
            p108.alt = (float) -1.3859936E38F;
            p108.std_dev_horz = (float) -6.6671226E37F;
            p108.std_dev_vert = (float) -1.1425741E38F;
            p108.vn = (float) -1.2372447E38F;
            p108.ve = (float)2.012381E38F;
            p108.vd = (float)2.4289118E37F;
            LoopBackDemoChannel.instance.send(p108); //===============================
            RADIO_STATUS p109 = LoopBackDemoChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.rssi = (byte)(byte)73;
            p109.remrssi = (byte)(byte)108;
            p109.txbuf = (byte)(byte)59;
            p109.noise = (byte)(byte)77;
            p109.remnoise = (byte)(byte)10;
            p109.rxerrors = (ushort)(ushort)29791;
            p109.fixed_ = (ushort)(ushort)2927;
            LoopBackDemoChannel.instance.send(p109); //===============================
            FILE_TRANSFER_PROTOCOL p110 = LoopBackDemoChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_network = (byte)(byte)89;
            p110.target_system = (byte)(byte)194;
            p110.target_component = (byte)(byte)157;
            p110.payload_SET(new byte[251], 0);
            LoopBackDemoChannel.instance.send(p110); //===============================
            TIMESYNC p111 = LoopBackDemoChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.tc1 = (long)574142186498131994L;
            p111.ts1 = (long)4840815650048173583L;
            LoopBackDemoChannel.instance.send(p111); //===============================
            CAMERA_TRIGGER p112 = LoopBackDemoChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.time_usec = (ulong)154554138381529469L;
            p112.seq = (uint)2781777378U;
            LoopBackDemoChannel.instance.send(p112); //===============================
            HIL_GPS p113 = LoopBackDemoChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.time_usec = (ulong)2797414763474310283L;
            p113.fix_type = (byte)(byte)103;
            p113.lat = (int)1264679790;
            p113.lon = (int) -573359059;
            p113.alt = (int) -943149459;
            p113.eph = (ushort)(ushort)61744;
            p113.epv = (ushort)(ushort)52925;
            p113.vel = (ushort)(ushort)52886;
            p113.vn = (short)(short)24998;
            p113.ve = (short)(short)21512;
            p113.vd = (short)(short) -18157;
            p113.cog = (ushort)(ushort)40898;
            p113.satellites_visible = (byte)(byte)89;
            LoopBackDemoChannel.instance.send(p113); //===============================
            HIL_OPTICAL_FLOW p114 = LoopBackDemoChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.time_usec = (ulong)883428899839783821L;
            p114.sensor_id = (byte)(byte)69;
            p114.integration_time_us = (uint)372880628U;
            p114.integrated_x = (float)2.6476828E38F;
            p114.integrated_y = (float)5.011248E37F;
            p114.integrated_xgyro = (float)1.6923216E38F;
            p114.integrated_ygyro = (float) -4.5629235E37F;
            p114.integrated_zgyro = (float) -1.0532787E38F;
            p114.temperature = (short)(short)23432;
            p114.quality = (byte)(byte)248;
            p114.time_delta_distance_us = (uint)2184378312U;
            p114.distance = (float) -1.2855951E38F;
            LoopBackDemoChannel.instance.send(p114); //===============================
            HIL_STATE_QUATERNION p115 = LoopBackDemoChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.time_usec = (ulong)7677500611255612320L;
            p115.attitude_quaternion_SET(new float[4], 0);
            p115.rollspeed = (float) -1.7238776E38F;
            p115.pitchspeed = (float)1.6621511E37F;
            p115.yawspeed = (float) -9.433156E37F;
            p115.lat = (int)884642139;
            p115.lon = (int) -1167269819;
            p115.alt = (int) -800410761;
            p115.vx = (short)(short)5242;
            p115.vy = (short)(short)27059;
            p115.vz = (short)(short)31606;
            p115.ind_airspeed = (ushort)(ushort)1107;
            p115.true_airspeed = (ushort)(ushort)9594;
            p115.xacc = (short)(short) -31816;
            p115.yacc = (short)(short)13971;
            p115.zacc = (short)(short) -19406;
            LoopBackDemoChannel.instance.send(p115); //===============================
            SCALED_IMU2 p116 = LoopBackDemoChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.time_boot_ms = (uint)1156647698U;
            p116.xacc = (short)(short)29012;
            p116.yacc = (short)(short)20646;
            p116.zacc = (short)(short)22557;
            p116.xgyro = (short)(short)4552;
            p116.ygyro = (short)(short)10564;
            p116.zgyro = (short)(short)13074;
            p116.xmag = (short)(short)8558;
            p116.ymag = (short)(short) -29270;
            p116.zmag = (short)(short) -22469;
            LoopBackDemoChannel.instance.send(p116); //===============================
            LOG_REQUEST_LIST p117 = LoopBackDemoChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.target_system = (byte)(byte)162;
            p117.target_component = (byte)(byte)15;
            p117.start = (ushort)(ushort)9470;
            p117.end = (ushort)(ushort)721;
            LoopBackDemoChannel.instance.send(p117); //===============================
            LOG_ENTRY p118 = LoopBackDemoChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.id = (ushort)(ushort)52142;
            p118.num_logs = (ushort)(ushort)57310;
            p118.last_log_num = (ushort)(ushort)29740;
            p118.time_utc = (uint)4188476505U;
            p118.size = (uint)1521924430U;
            LoopBackDemoChannel.instance.send(p118); //===============================
            LOG_REQUEST_DATA p119 = LoopBackDemoChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.target_system = (byte)(byte)215;
            p119.target_component = (byte)(byte)103;
            p119.id = (ushort)(ushort)10581;
            p119.ofs = (uint)2343322116U;
            p119.count = (uint)949949798U;
            LoopBackDemoChannel.instance.send(p119); //===============================
            LOG_DATA p120 = LoopBackDemoChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.id = (ushort)(ushort)13781;
            p120.ofs = (uint)3922337767U;
            p120.count = (byte)(byte)89;
            p120.data__SET(new byte[90], 0);
            LoopBackDemoChannel.instance.send(p120); //===============================
            LOG_ERASE p121 = LoopBackDemoChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_system = (byte)(byte)22;
            p121.target_component = (byte)(byte)255;
            LoopBackDemoChannel.instance.send(p121); //===============================
            LOG_REQUEST_END p122 = LoopBackDemoChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_system = (byte)(byte)82;
            p122.target_component = (byte)(byte)143;
            LoopBackDemoChannel.instance.send(p122); //===============================
            GPS_INJECT_DATA p123 = LoopBackDemoChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.target_system = (byte)(byte)153;
            p123.target_component = (byte)(byte)111;
            p123.len = (byte)(byte)87;
            p123.data__SET(new byte[110], 0);
            LoopBackDemoChannel.instance.send(p123); //===============================
            GPS2_RAW p124 = LoopBackDemoChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.time_usec = (ulong)6168540940983926775L;
            p124.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS;
            p124.lat = (int) -2107641223;
            p124.lon = (int) -930484275;
            p124.alt = (int) -1410838437;
            p124.eph = (ushort)(ushort)37780;
            p124.epv = (ushort)(ushort)44127;
            p124.vel = (ushort)(ushort)9001;
            p124.cog = (ushort)(ushort)21286;
            p124.satellites_visible = (byte)(byte)54;
            p124.dgps_numch = (byte)(byte)185;
            p124.dgps_age = (uint)2833855686U;
            LoopBackDemoChannel.instance.send(p124); //===============================
            POWER_STATUS p125 = LoopBackDemoChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vcc = (ushort)(ushort)18290;
            p125.Vservo = (ushort)(ushort)42641;
            p125.flags = (MAV_POWER_STATUS)MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT;
            LoopBackDemoChannel.instance.send(p125); //===============================
            SERIAL_CONTROL p126 = LoopBackDemoChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.device = (SERIAL_CONTROL_DEV)SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1;
            p126.flags = (SERIAL_CONTROL_FLAG)SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE;
            p126.timeout = (ushort)(ushort)16000;
            p126.baudrate = (uint)3493333987U;
            p126.count = (byte)(byte)37;
            p126.data__SET(new byte[70], 0);
            LoopBackDemoChannel.instance.send(p126); //===============================
            GPS_RTK p127 = LoopBackDemoChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.time_last_baseline_ms = (uint)2146361170U;
            p127.rtk_receiver_id = (byte)(byte)146;
            p127.wn = (ushort)(ushort)59542;
            p127.tow = (uint)3776023125U;
            p127.rtk_health = (byte)(byte)169;
            p127.rtk_rate = (byte)(byte)132;
            p127.nsats = (byte)(byte)225;
            p127.baseline_coords_type = (byte)(byte)226;
            p127.baseline_a_mm = (int)1622220129;
            p127.baseline_b_mm = (int)83640261;
            p127.baseline_c_mm = (int) -466409114;
            p127.accuracy = (uint)1946271602U;
            p127.iar_num_hypotheses = (int) -1835495321;
            LoopBackDemoChannel.instance.send(p127); //===============================
            GPS2_RTK p128 = LoopBackDemoChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.time_last_baseline_ms = (uint)3486785739U;
            p128.rtk_receiver_id = (byte)(byte)26;
            p128.wn = (ushort)(ushort)30108;
            p128.tow = (uint)4072249316U;
            p128.rtk_health = (byte)(byte)180;
            p128.rtk_rate = (byte)(byte)101;
            p128.nsats = (byte)(byte)72;
            p128.baseline_coords_type = (byte)(byte)119;
            p128.baseline_a_mm = (int)1087026848;
            p128.baseline_b_mm = (int) -190315175;
            p128.baseline_c_mm = (int)1424056430;
            p128.accuracy = (uint)4217681110U;
            p128.iar_num_hypotheses = (int) -97737430;
            LoopBackDemoChannel.instance.send(p128); //===============================
            SCALED_IMU3 p129 = LoopBackDemoChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.time_boot_ms = (uint)3213841747U;
            p129.xacc = (short)(short) -14926;
            p129.yacc = (short)(short) -32532;
            p129.zacc = (short)(short)20695;
            p129.xgyro = (short)(short) -223;
            p129.ygyro = (short)(short) -12336;
            p129.zgyro = (short)(short)23137;
            p129.xmag = (short)(short)17001;
            p129.ymag = (short)(short)22020;
            p129.zmag = (short)(short)5867;
            LoopBackDemoChannel.instance.send(p129); //===============================
            DATA_TRANSMISSION_HANDSHAKE p130 = LoopBackDemoChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.type = (byte)(byte)76;
            p130.size = (uint)1974183500U;
            p130.width = (ushort)(ushort)43951;
            p130.height = (ushort)(ushort)46461;
            p130.packets = (ushort)(ushort)47332;
            p130.payload = (byte)(byte)71;
            p130.jpg_quality = (byte)(byte)161;
            LoopBackDemoChannel.instance.send(p130); //===============================
            ENCAPSULATED_DATA p131 = LoopBackDemoChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.seqnr = (ushort)(ushort)57103;
            p131.data__SET(new byte[253], 0);
            LoopBackDemoChannel.instance.send(p131); //===============================
            DISTANCE_SENSOR p132 = LoopBackDemoChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.time_boot_ms = (uint)1161114642U;
            p132.min_distance = (ushort)(ushort)17160;
            p132.max_distance = (ushort)(ushort)24967;
            p132.current_distance = (ushort)(ushort)2395;
            p132.type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND;
            p132.id = (byte)(byte)249;
            p132.orientation = (MAV_SENSOR_ORIENTATION)MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_PITCH_180_YAW_270;
            p132.covariance = (byte)(byte)5;
            LoopBackDemoChannel.instance.send(p132); //===============================
            TERRAIN_REQUEST p133 = LoopBackDemoChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.lat = (int) -516813503;
            p133.lon = (int)445078209;
            p133.grid_spacing = (ushort)(ushort)50729;
            p133.mask = (ulong)6327474844782720932L;
            LoopBackDemoChannel.instance.send(p133); //===============================
            TERRAIN_DATA p134 = LoopBackDemoChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.lat = (int)374192599;
            p134.lon = (int)1880811118;
            p134.grid_spacing = (ushort)(ushort)5547;
            p134.gridbit = (byte)(byte)121;
            p134.data__SET(new short[16], 0);
            LoopBackDemoChannel.instance.send(p134); //===============================
            TERRAIN_CHECK p135 = LoopBackDemoChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lat = (int)543486491;
            p135.lon = (int) -454510864;
            LoopBackDemoChannel.instance.send(p135); //===============================
            TERRAIN_REPORT p136 = LoopBackDemoChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.lat = (int) -1618541775;
            p136.lon = (int) -422676071;
            p136.spacing = (ushort)(ushort)42678;
            p136.terrain_height = (float) -9.388229E37F;
            p136.current_height = (float) -6.118438E37F;
            p136.pending = (ushort)(ushort)58104;
            p136.loaded = (ushort)(ushort)14106;
            LoopBackDemoChannel.instance.send(p136); //===============================
            SCALED_PRESSURE2 p137 = LoopBackDemoChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.time_boot_ms = (uint)1818074738U;
            p137.press_abs = (float)3.103681E38F;
            p137.press_diff = (float)1.8194938E38F;
            p137.temperature = (short)(short) -16396;
            LoopBackDemoChannel.instance.send(p137); //===============================
            ATT_POS_MOCAP p138 = LoopBackDemoChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.time_usec = (ulong)6517426084660687203L;
            p138.q_SET(new float[4], 0);
            p138.x = (float)2.6792643E38F;
            p138.y = (float)2.128891E38F;
            p138.z = (float) -1.3585089E38F;
            LoopBackDemoChannel.instance.send(p138); //===============================
            SET_ACTUATOR_CONTROL_TARGET p139 = LoopBackDemoChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.time_usec = (ulong)4867937686229186957L;
            p139.group_mlx = (byte)(byte)222;
            p139.target_system = (byte)(byte)102;
            p139.target_component = (byte)(byte)228;
            p139.controls_SET(new float[8], 0);
            LoopBackDemoChannel.instance.send(p139); //===============================
            ACTUATOR_CONTROL_TARGET p140 = LoopBackDemoChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.time_usec = (ulong)630184312564666264L;
            p140.group_mlx = (byte)(byte)218;
            p140.controls_SET(new float[8], 0);
            LoopBackDemoChannel.instance.send(p140); //===============================
            ALTITUDE p141 = LoopBackDemoChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.time_usec = (ulong)6519624439775202417L;
            p141.altitude_monotonic = (float) -8.497153E37F;
            p141.altitude_amsl = (float)6.7780375E37F;
            p141.altitude_local = (float)1.0251861E38F;
            p141.altitude_relative = (float)4.7703553E37F;
            p141.altitude_terrain = (float) -2.4933397E38F;
            p141.bottom_clearance = (float)6.628971E37F;
            LoopBackDemoChannel.instance.send(p141); //===============================
            RESOURCE_REQUEST p142 = LoopBackDemoChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.request_id = (byte)(byte)178;
            p142.uri_type = (byte)(byte)120;
            p142.uri_SET(new byte[120], 0);
            p142.transfer_type = (byte)(byte)84;
            p142.storage_SET(new byte[120], 0);
            LoopBackDemoChannel.instance.send(p142); //===============================
            SCALED_PRESSURE3 p143 = LoopBackDemoChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.time_boot_ms = (uint)247729262U;
            p143.press_abs = (float) -1.2468263E38F;
            p143.press_diff = (float) -2.9292391E38F;
            p143.temperature = (short)(short)24336;
            LoopBackDemoChannel.instance.send(p143); //===============================
            FOLLOW_TARGET p144 = LoopBackDemoChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.timestamp = (ulong)871878176773813568L;
            p144.est_capabilities = (byte)(byte)170;
            p144.lat = (int) -1431354944;
            p144.lon = (int) -1703322676;
            p144.alt = (float) -3.1537995E38F;
            p144.vel_SET(new float[3], 0);
            p144.acc_SET(new float[3], 0);
            p144.attitude_q_SET(new float[4], 0);
            p144.rates_SET(new float[3], 0);
            p144.position_cov_SET(new float[3], 0);
            p144.custom_state = (ulong)4163266030827963548L;
            LoopBackDemoChannel.instance.send(p144); //===============================
            CONTROL_SYSTEM_STATE p146 = LoopBackDemoChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.time_usec = (ulong)7144839952714113064L;
            p146.x_acc = (float) -3.023291E37F;
            p146.y_acc = (float) -1.4733752E38F;
            p146.z_acc = (float)1.579429E38F;
            p146.x_vel = (float) -1.133943E38F;
            p146.y_vel = (float)2.6272898E38F;
            p146.z_vel = (float)5.4295443E37F;
            p146.x_pos = (float)3.328677E37F;
            p146.y_pos = (float) -2.7799036E38F;
            p146.z_pos = (float)2.2868883E38F;
            p146.airspeed = (float) -2.184975E38F;
            p146.vel_variance_SET(new float[3], 0);
            p146.pos_variance_SET(new float[3], 0);
            p146.q_SET(new float[4], 0);
            p146.roll_rate = (float) -3.2359084E38F;
            p146.pitch_rate = (float) -2.5220511E38F;
            p146.yaw_rate = (float) -4.694994E37F;
            LoopBackDemoChannel.instance.send(p146); //===============================
            BATTERY_STATUS p147 = LoopBackDemoChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.id = (byte)(byte)124;
            p147.battery_function = (MAV_BATTERY_FUNCTION)MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_PROPULSION;
            p147.type = (MAV_BATTERY_TYPE)MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_UNKNOWN;
            p147.temperature = (short)(short)9781;
            p147.voltages_SET(new ushort[10], 0);
            p147.current_battery = (short)(short) -16199;
            p147.current_consumed = (int) -1107689683;
            p147.energy_consumed = (int) -223060772;
            p147.battery_remaining = (sbyte)(sbyte) - 86;
            LoopBackDemoChannel.instance.send(p147); //===============================
            AUTOPILOT_VERSION p148 = LoopBackDemoChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY)MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_INT;
            p148.flight_sw_version = (uint)104072407U;
            p148.middleware_sw_version = (uint)87823588U;
            p148.os_sw_version = (uint)1472271240U;
            p148.board_version = (uint)3284618747U;
            p148.flight_custom_version_SET(new byte[8], 0);
            p148.middleware_custom_version_SET(new byte[8], 0);
            p148.os_custom_version_SET(new byte[8], 0);
            p148.vendor_id = (ushort)(ushort)21696;
            p148.product_id = (ushort)(ushort)53007;
            p148.uid = (ulong)572991979019307160L;
            p148.uid2_SET(new byte[18], 0, PH);
            LoopBackDemoChannel.instance.send(p148); //===============================
            LANDING_TARGET p149 = LoopBackDemoChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.time_usec = (ulong)4204475584280843191L;
            p149.target_num = (byte)(byte)71;
            p149.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_ENU;
            p149.angle_x = (float) -1.0195932E38F;
            p149.angle_y = (float) -8.688375E37F;
            p149.distance = (float) -2.3526263E38F;
            p149.size_x = (float) -1.8352265E38F;
            p149.size_y = (float) -7.7984116E37F;
            p149.x_SET((float) -5.266744E37F, PH);
            p149.y_SET((float)2.6385612E37F, PH);
            p149.z_SET((float) -4.8324053E37F, PH);
            p149.q_SET(new float[4], 0, PH);
            p149.type = (LANDING_TARGET_TYPE)LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON;
            p149.position_valid_SET((byte)(byte)183, PH);
            LoopBackDemoChannel.instance.send(p149); //===============================
            CPU_LOAD p170 = LoopBackDemoChannel.new_CPU_LOAD();
            PH.setPack(p170);
            p170.sensLoad = (byte)(byte)44;
            p170.ctrlLoad = (byte)(byte)39;
            p170.batVolt = (ushort)(ushort)7783;
            LoopBackDemoChannel.instance.send(p170); //===============================
            SENSOR_BIAS p172 = LoopBackDemoChannel.new_SENSOR_BIAS();
            PH.setPack(p172);
            p172.axBias = (float)1.691149E38F;
            p172.ayBias = (float)7.993518E36F;
            p172.azBias = (float)2.8106562E38F;
            p172.gxBias = (float)2.1502094E38F;
            p172.gyBias = (float)1.5381655E37F;
            p172.gzBias = (float)2.9635257E38F;
            LoopBackDemoChannel.instance.send(p172); //===============================
            DIAGNOSTIC p173 = LoopBackDemoChannel.new_DIAGNOSTIC();
            PH.setPack(p173);
            p173.diagFl1 = (float)1.8580738E36F;
            p173.diagFl2 = (float) -3.1689784E38F;
            p173.diagFl3 = (float)2.8240363E38F;
            p173.diagSh1 = (short)(short)17621;
            p173.diagSh2 = (short)(short)470;
            p173.diagSh3 = (short)(short)7495;
            LoopBackDemoChannel.instance.send(p173); //===============================
            SLUGS_NAVIGATION p176 = LoopBackDemoChannel.new_SLUGS_NAVIGATION();
            PH.setPack(p176);
            p176.u_m = (float) -2.1698483E38F;
            p176.phi_c = (float) -2.3717412E38F;
            p176.theta_c = (float)2.3285867E38F;
            p176.psiDot_c = (float)2.2150377E38F;
            p176.ay_body = (float)2.984427E38F;
            p176.totalDist = (float)9.197609E37F;
            p176.dist2Go = (float) -3.1987473E38F;
            p176.fromWP = (byte)(byte)144;
            p176.toWP = (byte)(byte)19;
            p176.h_c = (ushort)(ushort)22658;
            LoopBackDemoChannel.instance.send(p176); //===============================
            DATA_LOG p177 = LoopBackDemoChannel.new_DATA_LOG();
            PH.setPack(p177);
            p177.fl_1 = (float)2.4829817E38F;
            p177.fl_2 = (float)4.5853336E37F;
            p177.fl_3 = (float) -1.1488359E38F;
            p177.fl_4 = (float) -5.3622487E37F;
            p177.fl_5 = (float)2.0974107E38F;
            p177.fl_6 = (float)8.916455E37F;
            LoopBackDemoChannel.instance.send(p177); //===============================
            GPS_DATE_TIME p179 = LoopBackDemoChannel.new_GPS_DATE_TIME();
            PH.setPack(p179);
            p179.year = (byte)(byte)37;
            p179.month = (byte)(byte)69;
            p179.day = (byte)(byte)182;
            p179.hour = (byte)(byte)128;
            p179.min = (byte)(byte)155;
            p179.sec = (byte)(byte)147;
            p179.clockStat = (byte)(byte)243;
            p179.visSat = (byte)(byte)142;
            p179.useSat = (byte)(byte)155;
            p179.GppGl = (byte)(byte)229;
            p179.sigUsedMask = (byte)(byte)108;
            p179.percentUsed = (byte)(byte)186;
            LoopBackDemoChannel.instance.send(p179); //===============================
            MID_LVL_CMDS p180 = LoopBackDemoChannel.new_MID_LVL_CMDS();
            PH.setPack(p180);
            p180.target = (byte)(byte)18;
            p180.hCommand = (float) -2.2939273E38F;
            p180.uCommand = (float) -2.4620168E37F;
            p180.rCommand = (float) -1.793898E38F;
            LoopBackDemoChannel.instance.send(p180); //===============================
            CTRL_SRFC_PT p181 = LoopBackDemoChannel.new_CTRL_SRFC_PT();
            PH.setPack(p181);
            p181.target = (byte)(byte)172;
            p181.bitfieldPt = (ushort)(ushort)49983;
            LoopBackDemoChannel.instance.send(p181); //===============================
            SLUGS_CAMERA_ORDER p184 = LoopBackDemoChannel.new_SLUGS_CAMERA_ORDER();
            PH.setPack(p184);
            p184.target = (byte)(byte)91;
            p184.pan = (sbyte)(sbyte)74;
            p184.tilt = (sbyte)(sbyte) - 8;
            p184.zoom = (sbyte)(sbyte) - 59;
            p184.moveHome = (sbyte)(sbyte) - 107;
            LoopBackDemoChannel.instance.send(p184); //===============================
            CONTROL_SURFACE p185 = LoopBackDemoChannel.new_CONTROL_SURFACE();
            PH.setPack(p185);
            p185.target = (byte)(byte)214;
            p185.idSurface = (byte)(byte)203;
            p185.mControl = (float) -1.6019095E38F;
            p185.bControl = (float)1.4888332E38F;
            LoopBackDemoChannel.instance.send(p185); //===============================
            SLUGS_MOBILE_LOCATION p186 = LoopBackDemoChannel.new_SLUGS_MOBILE_LOCATION();
            PH.setPack(p186);
            p186.target = (byte)(byte)116;
            p186.latitude = (float) -2.9206921E38F;
            p186.longitude = (float)5.583411E37F;
            LoopBackDemoChannel.instance.send(p186); //===============================
            SLUGS_CONFIGURATION_CAMERA p188 = LoopBackDemoChannel.new_SLUGS_CONFIGURATION_CAMERA();
            PH.setPack(p188);
            p188.target = (byte)(byte)176;
            p188.idOrder = (byte)(byte)223;
            p188.order = (byte)(byte)129;
            LoopBackDemoChannel.instance.send(p188); //===============================
            ISR_LOCATION p189 = LoopBackDemoChannel.new_ISR_LOCATION();
            PH.setPack(p189);
            p189.target = (byte)(byte)244;
            p189.latitude = (float)7.3361724E37F;
            p189.longitude = (float) -2.3740064E37F;
            p189.height = (float)2.9756031E38F;
            p189.option1 = (byte)(byte)45;
            p189.option2 = (byte)(byte)34;
            p189.option3 = (byte)(byte)60;
            LoopBackDemoChannel.instance.send(p189); //===============================
            VOLT_SENSOR p191 = LoopBackDemoChannel.new_VOLT_SENSOR();
            PH.setPack(p191);
            p191.r2Type = (byte)(byte)171;
            p191.voltage = (ushort)(ushort)27852;
            p191.reading2 = (ushort)(ushort)22097;
            LoopBackDemoChannel.instance.send(p191); //===============================
            PTZ_STATUS p192 = LoopBackDemoChannel.new_PTZ_STATUS();
            PH.setPack(p192);
            p192.zoom = (byte)(byte)76;
            p192.pan = (short)(short) -5159;
            p192.tilt = (short)(short)28407;
            LoopBackDemoChannel.instance.send(p192); //===============================
            UAV_STATUS p193 = LoopBackDemoChannel.new_UAV_STATUS();
            PH.setPack(p193);
            p193.target = (byte)(byte)144;
            p193.latitude = (float)1.715995E38F;
            p193.longitude = (float)3.0753915E38F;
            p193.altitude = (float) -1.9373796E38F;
            p193.speed = (float)2.1259058E38F;
            p193.course = (float) -2.2856961E38F;
            LoopBackDemoChannel.instance.send(p193); //===============================
            STATUS_GPS p194 = LoopBackDemoChannel.new_STATUS_GPS();
            PH.setPack(p194);
            p194.csFails = (ushort)(ushort)5205;
            p194.gpsQuality = (byte)(byte)44;
            p194.msgsType = (byte)(byte)7;
            p194.posStatus = (byte)(byte)103;
            p194.magVar = (float) -1.8718307E38F;
            p194.magDir = (sbyte)(sbyte)20;
            p194.modeInd = (byte)(byte)40;
            LoopBackDemoChannel.instance.send(p194); //===============================
            NOVATEL_DIAG p195 = LoopBackDemoChannel.new_NOVATEL_DIAG();
            PH.setPack(p195);
            p195.timeStatus = (byte)(byte)20;
            p195.receiverStatus = (uint)518695203U;
            p195.solStatus = (byte)(byte)56;
            p195.posType = (byte)(byte)93;
            p195.velType = (byte)(byte)147;
            p195.posSolAge = (float) -1.3112446E38F;
            p195.csFails = (ushort)(ushort)19852;
            LoopBackDemoChannel.instance.send(p195); //===============================
            SENSOR_DIAG p196 = LoopBackDemoChannel.new_SENSOR_DIAG();
            PH.setPack(p196);
            p196.float1 = (float)2.1014988E38F;
            p196.float2 = (float) -4.0078046E37F;
            p196.int1 = (short)(short)2777;
            p196.char1 = (sbyte)(sbyte) - 56;
            LoopBackDemoChannel.instance.send(p196); //===============================
            BOOT p197 = LoopBackDemoChannel.new_BOOT();
            PH.setPack(p197);
            p197.version = (uint)3646973059U;
            LoopBackDemoChannel.instance.send(p197); //===============================
            ESTIMATOR_STATUS p230 = LoopBackDemoChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.time_usec = (ulong)1171294111650426236L;
            p230.flags = (ESTIMATOR_STATUS_FLAGS)ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE;
            p230.vel_ratio = (float)3.5729942E37F;
            p230.pos_horiz_ratio = (float)2.4461347E38F;
            p230.pos_vert_ratio = (float)4.0442377E37F;
            p230.mag_ratio = (float) -1.0093089E38F;
            p230.hagl_ratio = (float) -3.2887371E38F;
            p230.tas_ratio = (float)9.27413E37F;
            p230.pos_horiz_accuracy = (float) -2.3895818E38F;
            p230.pos_vert_accuracy = (float)1.6875844E38F;
            LoopBackDemoChannel.instance.send(p230); //===============================
            WIND_COV p231 = LoopBackDemoChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.time_usec = (ulong)3615022809740425738L;
            p231.wind_x = (float)2.936163E38F;
            p231.wind_y = (float) -7.1448236E37F;
            p231.wind_z = (float) -1.8371819E38F;
            p231.var_horiz = (float) -2.9113004E38F;
            p231.var_vert = (float) -1.5591816E38F;
            p231.wind_alt = (float)1.8152365E38F;
            p231.horiz_accuracy = (float)1.6670991E38F;
            p231.vert_accuracy = (float) -1.5107172E38F;
            LoopBackDemoChannel.instance.send(p231); //===============================
            GPS_INPUT p232 = LoopBackDemoChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.time_usec = (ulong)6601282476921025741L;
            p232.gps_id = (byte)(byte)72;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS)GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT;
            p232.time_week_ms = (uint)3573115724U;
            p232.time_week = (ushort)(ushort)52650;
            p232.fix_type = (byte)(byte)104;
            p232.lat = (int)713313256;
            p232.lon = (int)1660682438;
            p232.alt = (float) -1.0010231E37F;
            p232.hdop = (float)2.4342198E38F;
            p232.vdop = (float) -2.3490515E38F;
            p232.vn = (float) -1.2853254E38F;
            p232.ve = (float) -1.0389539E38F;
            p232.vd = (float)6.0189455E37F;
            p232.speed_accuracy = (float) -1.998521E38F;
            p232.horiz_accuracy = (float)3.0453042E38F;
            p232.vert_accuracy = (float)8.333959E37F;
            p232.satellites_visible = (byte)(byte)48;
            LoopBackDemoChannel.instance.send(p232); //===============================
            GPS_RTCM_DATA p233 = LoopBackDemoChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.flags = (byte)(byte)196;
            p233.len = (byte)(byte)152;
            p233.data__SET(new byte[180], 0);
            LoopBackDemoChannel.instance.send(p233); //===============================
            HIGH_LATENCY p234 = LoopBackDemoChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED;
            p234.custom_mode = (uint)256521345U;
            p234.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING;
            p234.roll = (short)(short) -27322;
            p234.pitch = (short)(short)3881;
            p234.heading = (ushort)(ushort)62456;
            p234.throttle = (sbyte)(sbyte)80;
            p234.heading_sp = (short)(short) -26000;
            p234.latitude = (int)31249780;
            p234.longitude = (int)1540113836;
            p234.altitude_amsl = (short)(short) -26508;
            p234.altitude_sp = (short)(short) -25559;
            p234.airspeed = (byte)(byte)211;
            p234.airspeed_sp = (byte)(byte)27;
            p234.groundspeed = (byte)(byte)193;
            p234.climb_rate = (sbyte)(sbyte) - 72;
            p234.gps_nsat = (byte)(byte)17;
            p234.gps_fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX;
            p234.battery_remaining = (byte)(byte)204;
            p234.temperature = (sbyte)(sbyte) - 46;
            p234.temperature_air = (sbyte)(sbyte)24;
            p234.failsafe = (byte)(byte)52;
            p234.wp_num = (byte)(byte)2;
            p234.wp_distance = (ushort)(ushort)26477;
            LoopBackDemoChannel.instance.send(p234); //===============================
            VIBRATION p241 = LoopBackDemoChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.time_usec = (ulong)7816194063191750286L;
            p241.vibration_x = (float) -2.5810058E38F;
            p241.vibration_y = (float) -2.6988794E38F;
            p241.vibration_z = (float) -2.2928183E38F;
            p241.clipping_0 = (uint)1826531946U;
            p241.clipping_1 = (uint)2646473082U;
            p241.clipping_2 = (uint)1475442075U;
            LoopBackDemoChannel.instance.send(p241); //===============================
            HOME_POSITION p242 = LoopBackDemoChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.latitude = (int) -2008173219;
            p242.longitude = (int) -161695673;
            p242.altitude = (int)1407301406;
            p242.x = (float) -2.3312415E38F;
            p242.y = (float) -1.0115911E38F;
            p242.z = (float)3.3291939E38F;
            p242.q_SET(new float[4], 0);
            p242.approach_x = (float)3.3101337E38F;
            p242.approach_y = (float) -1.1385929E38F;
            p242.approach_z = (float) -2.8959423E38F;
            p242.time_usec_SET((ulong)1143074622741076980L, PH);
            LoopBackDemoChannel.instance.send(p242); //===============================
            SET_HOME_POSITION p243 = LoopBackDemoChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.target_system = (byte)(byte)0;
            p243.latitude = (int)408944185;
            p243.longitude = (int)1919738144;
            p243.altitude = (int)961039676;
            p243.x = (float)1.6644028E38F;
            p243.y = (float) -1.2280698E38F;
            p243.z = (float) -2.9463772E38F;
            p243.q_SET(new float[4], 0);
            p243.approach_x = (float) -1.7935656E38F;
            p243.approach_y = (float) -2.0282312E38F;
            p243.approach_z = (float) -1.1238089E38F;
            p243.time_usec_SET((ulong)1857651430732642805L, PH);
            LoopBackDemoChannel.instance.send(p243); //===============================
            MESSAGE_INTERVAL p244 = LoopBackDemoChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.message_id = (ushort)(ushort)17435;
            p244.interval_us = (int) -490751672;
            LoopBackDemoChannel.instance.send(p244); //===============================
            EXTENDED_SYS_STATE p245 = LoopBackDemoChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.vtol_state = (MAV_VTOL_STATE)MAV_VTOL_STATE.MAV_VTOL_STATE_UNDEFINED;
            p245.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING;
            LoopBackDemoChannel.instance.send(p245); //===============================
            ADSB_VEHICLE p246 = LoopBackDemoChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.ICAO_address = (uint)1158931772U;
            p246.lat = (int) -1842803509;
            p246.lon = (int) -2025636437;
            p246.altitude_type = (ADSB_ALTITUDE_TYPE)ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
            p246.altitude = (int) -806984531;
            p246.heading = (ushort)(ushort)24905;
            p246.hor_velocity = (ushort)(ushort)29374;
            p246.ver_velocity = (short)(short) -7124;
            p246.callsign_SET("DEMO", PH);
            p246.emitter_type = (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_HEAVY;
            p246.tslc = (byte)(byte)60;
            p246.flags = (ADSB_FLAGS)ADSB_FLAGS.ADSB_FLAGS_SIMULATED;
            p246.squawk = (ushort)(ushort)52897;
            LoopBackDemoChannel.instance.send(p246); //===============================
            COLLISION p247 = LoopBackDemoChannel.new_COLLISION();
            PH.setPack(p247);
            p247.src_ = (MAV_COLLISION_SRC)MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT;
            p247.id = (uint)2017042782U;
            p247.action = (MAV_COLLISION_ACTION)MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_REPORT;
            p247.threat_level = (MAV_COLLISION_THREAT_LEVEL)MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW;
            p247.time_to_minimum_delta = (float)7.454457E37F;
            p247.altitude_minimum_delta = (float) -6.7831177E37F;
            p247.horizontal_minimum_delta = (float) -3.2723232E38F;
            LoopBackDemoChannel.instance.send(p247); //===============================
            V2_EXTENSION p248 = LoopBackDemoChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.target_network = (byte)(byte)127;
            p248.target_system = (byte)(byte)217;
            p248.target_component = (byte)(byte)239;
            p248.message_type = (ushort)(ushort)17237;
            p248.payload_SET(new byte[249], 0);
            LoopBackDemoChannel.instance.send(p248); //===============================
            MEMORY_VECT p249 = LoopBackDemoChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.address = (ushort)(ushort)46534;
            p249.ver = (byte)(byte)180;
            p249.type = (byte)(byte)159;
            p249.value_SET(new sbyte[32], 0);
            LoopBackDemoChannel.instance.send(p249); //===============================
            DEBUG_VECT p250 = LoopBackDemoChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.name_SET("DEMO", PH);
            p250.time_usec = (ulong)1029400501924310614L;
            p250.x = (float) -1.8374137E38F;
            p250.y = (float) -2.7119016E38F;
            p250.z = (float)5.1909547E37F;
            LoopBackDemoChannel.instance.send(p250); //===============================
            NAMED_VALUE_FLOAT p251 = LoopBackDemoChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.time_boot_ms = (uint)733853820U;
            p251.name_SET("DEMO", PH);
            p251.value = (float)1.0955602E38F;
            LoopBackDemoChannel.instance.send(p251); //===============================
            NAMED_VALUE_INT p252 = LoopBackDemoChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.time_boot_ms = (uint)1029742580U;
            p252.name_SET("DEMO", PH);
            p252.value = (int) -1759606054;
            LoopBackDemoChannel.instance.send(p252); //===============================
            STATUSTEXT p253 = LoopBackDemoChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.severity = (MAV_SEVERITY)MAV_SEVERITY.MAV_SEVERITY_WARNING;
            p253.text_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p253); //===============================
            DEBUG p254 = LoopBackDemoChannel.new_DEBUG();
            PH.setPack(p254);
            p254.time_boot_ms = (uint)3589240200U;
            p254.ind = (byte)(byte)104;
            p254.value = (float) -3.027257E38F;
            LoopBackDemoChannel.instance.send(p254); //===============================
            SETUP_SIGNING p256 = LoopBackDemoChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.target_system = (byte)(byte)158;
            p256.target_component = (byte)(byte)220;
            p256.secret_key_SET(new byte[32], 0);
            p256.initial_timestamp = (ulong)5047637658327471373L;
            LoopBackDemoChannel.instance.send(p256); //===============================
            BUTTON_CHANGE p257 = LoopBackDemoChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.time_boot_ms = (uint)4218122954U;
            p257.last_change_ms = (uint)1066282174U;
            p257.state = (byte)(byte)240;
            LoopBackDemoChannel.instance.send(p257); //===============================
            PLAY_TUNE p258 = LoopBackDemoChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_system = (byte)(byte)49;
            p258.target_component = (byte)(byte)224;
            p258.tune_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p258); //===============================
            CAMERA_INFORMATION p259 = LoopBackDemoChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.time_boot_ms = (uint)330988455U;
            p259.vendor_name_SET(new byte[32], 0);
            p259.model_name_SET(new byte[32], 0);
            p259.firmware_version = (uint)3480881031U;
            p259.focal_length = (float)2.639674E38F;
            p259.sensor_size_h = (float)1.1130578E38F;
            p259.sensor_size_v = (float)2.667585E38F;
            p259.resolution_h = (ushort)(ushort)59438;
            p259.resolution_v = (ushort)(ushort)729;
            p259.lens_id = (byte)(byte)28;
            p259.flags = (CAMERA_CAP_FLAGS)CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO;
            p259.cam_definition_version = (ushort)(ushort)4851;
            p259.cam_definition_uri_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p259); //===============================
            CAMERA_SETTINGS p260 = LoopBackDemoChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.time_boot_ms = (uint)685757711U;
            p260.mode_id = (CAMERA_MODE)CAMERA_MODE.CAMERA_MODE_IMAGE;
            LoopBackDemoChannel.instance.send(p260); //===============================
            STORAGE_INFORMATION p261 = LoopBackDemoChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.time_boot_ms = (uint)3372509437U;
            p261.storage_id = (byte)(byte)158;
            p261.storage_count = (byte)(byte)232;
            p261.status = (byte)(byte)22;
            p261.total_capacity = (float)9.74138E37F;
            p261.used_capacity = (float) -2.4411688E37F;
            p261.available_capacity = (float) -2.2218844E38F;
            p261.read_speed = (float)1.9807757E37F;
            p261.write_speed = (float)1.6639927E38F;
            LoopBackDemoChannel.instance.send(p261); //===============================
            CAMERA_CAPTURE_STATUS p262 = LoopBackDemoChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.time_boot_ms = (uint)1482448510U;
            p262.image_status = (byte)(byte)98;
            p262.video_status = (byte)(byte)57;
            p262.image_interval = (float)1.9631464E38F;
            p262.recording_time_ms = (uint)1187262547U;
            p262.available_capacity = (float) -1.1706719E38F;
            LoopBackDemoChannel.instance.send(p262); //===============================
            CAMERA_IMAGE_CAPTURED p263 = LoopBackDemoChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.time_boot_ms = (uint)2659211787U;
            p263.time_utc = (ulong)8399753487390422549L;
            p263.camera_id = (byte)(byte)92;
            p263.lat = (int) -1416898904;
            p263.lon = (int)1644628865;
            p263.alt = (int) -1603650884;
            p263.relative_alt = (int) -168908502;
            p263.q_SET(new float[4], 0);
            p263.image_index = (int)506274447;
            p263.capture_result = (sbyte)(sbyte) - 119;
            p263.file_url_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p263); //===============================
            FLIGHT_INFORMATION p264 = LoopBackDemoChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.time_boot_ms = (uint)3601296527U;
            p264.arming_time_utc = (ulong)1934763759280281068L;
            p264.takeoff_time_utc = (ulong)7160899244906543066L;
            p264.flight_uuid = (ulong)4977916991366225682L;
            LoopBackDemoChannel.instance.send(p264); //===============================
            MOUNT_ORIENTATION p265 = LoopBackDemoChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.time_boot_ms = (uint)3574861952U;
            p265.roll = (float)4.109888E37F;
            p265.pitch = (float) -1.9375865E38F;
            p265.yaw = (float)2.3041014E38F;
            LoopBackDemoChannel.instance.send(p265); //===============================
            LOGGING_DATA p266 = LoopBackDemoChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.target_system = (byte)(byte)172;
            p266.target_component = (byte)(byte)59;
            p266.sequence = (ushort)(ushort)59774;
            p266.length = (byte)(byte)19;
            p266.first_message_offset = (byte)(byte)217;
            p266.data__SET(new byte[249], 0);
            LoopBackDemoChannel.instance.send(p266); //===============================
            LOGGING_DATA_ACKED p267 = LoopBackDemoChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.target_system = (byte)(byte)97;
            p267.target_component = (byte)(byte)158;
            p267.sequence = (ushort)(ushort)25563;
            p267.length = (byte)(byte)99;
            p267.first_message_offset = (byte)(byte)28;
            p267.data__SET(new byte[249], 0);
            LoopBackDemoChannel.instance.send(p267); //===============================
            LOGGING_ACK p268 = LoopBackDemoChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_system = (byte)(byte)232;
            p268.target_component = (byte)(byte)197;
            p268.sequence = (ushort)(ushort)43959;
            LoopBackDemoChannel.instance.send(p268); //===============================
            VIDEO_STREAM_INFORMATION p269 = LoopBackDemoChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.camera_id = (byte)(byte)148;
            p269.status = (byte)(byte)118;
            p269.framerate = (float)7.8560567E37F;
            p269.resolution_h = (ushort)(ushort)53596;
            p269.resolution_v = (ushort)(ushort)27789;
            p269.bitrate = (uint)1600698776U;
            p269.rotation = (ushort)(ushort)6668;
            p269.uri_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p269); //===============================
            SET_VIDEO_STREAM_SETTINGS p270 = LoopBackDemoChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.target_system = (byte)(byte)137;
            p270.target_component = (byte)(byte)208;
            p270.camera_id = (byte)(byte)219;
            p270.framerate = (float)3.2254708E38F;
            p270.resolution_h = (ushort)(ushort)28314;
            p270.resolution_v = (ushort)(ushort)45716;
            p270.bitrate = (uint)3994218660U;
            p270.rotation = (ushort)(ushort)54906;
            p270.uri_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p270); //===============================
            WIFI_CONFIG_AP p299 = LoopBackDemoChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.ssid_SET("DEMO", PH);
            p299.password_SET("DEMO", PH);
            LoopBackDemoChannel.instance.send(p299); //===============================
            PROTOCOL_VERSION p300 = LoopBackDemoChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.version = (ushort)(ushort)43098;
            p300.min_version = (ushort)(ushort)45871;
            p300.max_version = (ushort)(ushort)12075;
            p300.spec_version_hash_SET(new byte[8], 0);
            p300.library_version_hash_SET(new byte[8], 0);
            LoopBackDemoChannel.instance.send(p300); //===============================
            UAVCAN_NODE_STATUS p310 = LoopBackDemoChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.time_usec = (ulong)2478014133285303984L;
            p310.uptime_sec = (uint)566718128U;
            p310.health = (UAVCAN_NODE_HEALTH)UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL;
            p310.mode = (UAVCAN_NODE_MODE)UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE;
            p310.sub_mode = (byte)(byte)143;
            p310.vendor_specific_status_code = (ushort)(ushort)31869;
            LoopBackDemoChannel.instance.send(p310); //===============================
            UAVCAN_NODE_INFO p311 = LoopBackDemoChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.time_usec = (ulong)7100996972378987894L;
            p311.uptime_sec = (uint)3004910063U;
            p311.name_SET("DEMO", PH);
            p311.hw_version_major = (byte)(byte)235;
            p311.hw_version_minor = (byte)(byte)248;
            p311.hw_unique_id_SET(new byte[16], 0);
            p311.sw_version_major = (byte)(byte)119;
            p311.sw_version_minor = (byte)(byte)145;
            p311.sw_vcs_commit = (uint)2794075434U;
            LoopBackDemoChannel.instance.send(p311); //===============================
            PARAM_EXT_REQUEST_READ p320 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.target_system = (byte)(byte)221;
            p320.target_component = (byte)(byte)237;
            p320.param_id_SET("DEMO", PH);
            p320.param_index = (short)(short)23360;
            LoopBackDemoChannel.instance.send(p320); //===============================
            PARAM_EXT_REQUEST_LIST p321 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_system = (byte)(byte)156;
            p321.target_component = (byte)(byte)5;
            LoopBackDemoChannel.instance.send(p321); //===============================
            PARAM_EXT_VALUE p322 = LoopBackDemoChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_id_SET("DEMO", PH);
            p322.param_value_SET("DEMO", PH);
            p322.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32;
            p322.param_count = (ushort)(ushort)51925;
            p322.param_index = (ushort)(ushort)24008;
            LoopBackDemoChannel.instance.send(p322); //===============================
            PARAM_EXT_SET p323 = LoopBackDemoChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.target_system = (byte)(byte)167;
            p323.target_component = (byte)(byte)136;
            p323.param_id_SET("DEMO", PH);
            p323.param_value_SET("DEMO", PH);
            p323.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64;
            LoopBackDemoChannel.instance.send(p323); //===============================
            PARAM_EXT_ACK p324 = LoopBackDemoChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_id_SET("DEMO", PH);
            p324.param_value_SET("DEMO", PH);
            p324.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16;
            p324.param_result = (PARAM_ACK)PARAM_ACK.PARAM_ACK_IN_PROGRESS;
            LoopBackDemoChannel.instance.send(p324); //===============================
            OBSTACLE_DISTANCE p330 = LoopBackDemoChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.time_usec = (ulong)8260348676067557756L;
            p330.sensor_type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED;
            p330.distances_SET(new ushort[72], 0);
            p330.increment = (byte)(byte)96;
            p330.min_distance = (ushort)(ushort)20058;
            p330.max_distance = (ushort)(ushort)36724;
            LoopBackDemoChannel.instance.send(p330); //===============================
        }
    }
}

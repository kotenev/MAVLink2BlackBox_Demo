
using System.Diagnostics;
using System.Threading;
using Inside = org.unirail.BlackBox.Host.Pack.Meta.Field.Bounds.Inside;
namespace org.noname
{
    public class Demo :  GroundControl
    {
        static void Main_(string[] args)
        {
            Inside PH = new Inside();
            CommunicationChannel.instance.OnHEARTBEATReceive += (src, ph, pack) =>
            {
                MAV_TYPE type = pack.type;
                MAV_AUTOPILOT autopilot = pack.autopilot;
                MAV_MODE_FLAG base_mode = pack.base_mode;
                uint custom_mode = pack.custom_mode;
                MAV_STATE system_status = pack.system_status;
                byte mavlink_version = pack.mavlink_version;
            };
            CommunicationChannel.instance.OnSYS_STATUSReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnSYSTEM_TIMEReceive += (src, ph, pack) =>
            {
                ulong time_unix_usec = pack.time_unix_usec;
                uint time_boot_ms = pack.time_boot_ms;
            };
            CommunicationChannel.instance.OnPOSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnPINGReceive += (src, ph, pack) =>
            {
                ulong time_usec = pack.time_usec;
                uint seq = pack.seq;
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
            };
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROLReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte control_request = pack.control_request;
                byte version = pack.version;
                string passkey = pack.passkey_TRY(ph);
            };
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROL_ACKReceive += (src, ph, pack) =>
            {
                byte gcs_system_id = pack.gcs_system_id;
                byte control_request = pack.control_request;
                byte ack = pack.ack;
            };
            CommunicationChannel.instance.OnAUTH_KEYReceive += (src, ph, pack) =>
            {
                string key = pack.key_TRY(ph);
            };
            CommunicationChannel.instance.OnSET_MODEReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                MAV_MODE base_mode = pack.base_mode;
                uint custom_mode = pack.custom_mode;
            };
            CommunicationChannel.instance.OnPARAM_REQUEST_READReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                string param_id = pack.param_id_TRY(ph);
                short param_index = pack.param_index;
            };
            CommunicationChannel.instance.OnPARAM_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
            };
            CommunicationChannel.instance.OnPARAM_VALUEReceive += (src, ph, pack) =>
            {
                string param_id = pack.param_id_TRY(ph);
                float param_value = pack.param_value;
                MAV_PARAM_TYPE param_type = pack.param_type;
                ushort param_count = pack.param_count;
                ushort param_index = pack.param_index;
            };
            CommunicationChannel.instance.OnPARAM_SETReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                string param_id = pack.param_id_TRY(ph);
                float param_value = pack.param_value;
                MAV_PARAM_TYPE param_type = pack.param_type;
            };
            CommunicationChannel.instance.OnGPS_RAW_INTReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnGPS_STATUSReceive += (src, ph, pack) =>
            {
                byte satellites_visible = pack.satellites_visible;
                byte[] satellite_prn = pack.satellite_prn;
                byte[] satellite_used = pack.satellite_used;
                byte[] satellite_elevation = pack.satellite_elevation;
                byte[] satellite_azimuth = pack.satellite_azimuth;
                byte[] satellite_snr = pack.satellite_snr;
            };
            CommunicationChannel.instance.OnSCALED_IMUReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnRAW_IMUReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnRAW_PRESSUREReceive += (src, ph, pack) =>
            {
                ulong time_usec = pack.time_usec;
                short press_abs = pack.press_abs;
                short press_diff1 = pack.press_diff1;
                short press_diff2 = pack.press_diff2;
                short temperature = pack.temperature;
            };
            CommunicationChannel.instance.OnSCALED_PRESSUREReceive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                float press_abs = pack.press_abs;
                float press_diff = pack.press_diff;
                short temperature = pack.temperature;
            };
            CommunicationChannel.instance.OnATTITUDEReceive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                float roll = pack.roll;
                float pitch = pack.pitch;
                float yaw = pack.yaw;
                float rollspeed = pack.rollspeed;
                float pitchspeed = pack.pitchspeed;
                float yawspeed = pack.yawspeed;
            };
            CommunicationChannel.instance.OnATTITUDE_QUATERNIONReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnLOCAL_POSITION_NEDReceive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                float x = pack.x;
                float y = pack.y;
                float z = pack.z;
                float vx = pack.vx;
                float vy = pack.vy;
                float vz = pack.vz;
            };
            CommunicationChannel.instance.OnGLOBAL_POSITION_INTReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnRC_CHANNELS_SCALEDReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnRC_CHANNELS_RAWReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnSERVO_OUTPUT_RAWReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnMISSION_REQUEST_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                short start_index = pack.start_index;
                short end_index = pack.end_index;
                MAV_MISSION_TYPE mission_type = pack.mission_type;
            };
            CommunicationChannel.instance.OnMISSION_WRITE_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                short start_index = pack.start_index;
                short end_index = pack.end_index;
                MAV_MISSION_TYPE mission_type = pack.mission_type;
            };
            CommunicationChannel.instance.OnMISSION_ITEMReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnMISSION_REQUESTReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                ushort seq = pack.seq;
                MAV_MISSION_TYPE mission_type = pack.mission_type;
            };
            CommunicationChannel.instance.OnMISSION_SET_CURRENTReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                ushort seq = pack.seq;
            };
            CommunicationChannel.instance.OnMISSION_CURRENTReceive += (src, ph, pack) =>
            {
                ushort seq = pack.seq;
            };
            CommunicationChannel.instance.OnMISSION_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                MAV_MISSION_TYPE mission_type = pack.mission_type;
            };
            CommunicationChannel.instance.OnMISSION_COUNTReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                ushort count = pack.count;
                MAV_MISSION_TYPE mission_type = pack.mission_type;
            };
            CommunicationChannel.instance.OnMISSION_CLEAR_ALLReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                MAV_MISSION_TYPE mission_type = pack.mission_type;
            };
            CommunicationChannel.instance.OnMISSION_ITEM_REACHEDReceive += (src, ph, pack) =>
            {
                ushort seq = pack.seq;
            };
            CommunicationChannel.instance.OnMISSION_ACKReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                MAV_MISSION_RESULT type = pack.type;
                MAV_MISSION_TYPE mission_type = pack.mission_type;
            };
            CommunicationChannel.instance.OnSET_GPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                int latitude = pack.latitude;
                int longitude = pack.longitude;
                int altitude = pack.altitude;
                ulong time_usec = pack.time_usec_TRY(ph);
            };
            CommunicationChannel.instance.OnGPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                int latitude = pack.latitude;
                int longitude = pack.longitude;
                int altitude = pack.altitude;
                ulong time_usec = pack.time_usec_TRY(ph);
            };
            CommunicationChannel.instance.OnPARAM_MAP_RCReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnMISSION_REQUEST_INTReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                ushort seq = pack.seq;
                MAV_MISSION_TYPE mission_type = pack.mission_type;
            };
            CommunicationChannel.instance.OnSAFETY_SET_ALLOWED_AREAReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnSAFETY_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                MAV_FRAME frame = pack.frame;
                float p1x = pack.p1x;
                float p1y = pack.p1y;
                float p1z = pack.p1z;
                float p2x = pack.p2x;
                float p2y = pack.p2y;
                float p2z = pack.p2z;
            };
            CommunicationChannel.instance.OnATTITUDE_QUATERNION_COVReceive += (src, ph, pack) =>
            {
                ulong time_usec = pack.time_usec;
                float[] q = pack.q;
                float rollspeed = pack.rollspeed;
                float pitchspeed = pack.pitchspeed;
                float yawspeed = pack.yawspeed;
                float[] covariance = pack.covariance;
            };
            CommunicationChannel.instance.OnNAV_CONTROLLER_OUTPUTReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnGLOBAL_POSITION_INT_COVReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_COVReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnRC_CHANNELSReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnREQUEST_DATA_STREAMReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                byte req_stream_id = pack.req_stream_id;
                ushort req_message_rate = pack.req_message_rate;
                byte start_stop = pack.start_stop;
            };
            CommunicationChannel.instance.OnDATA_STREAMReceive += (src, ph, pack) =>
            {
                byte stream_id = pack.stream_id;
                ushort message_rate = pack.message_rate;
                byte on_off = pack.on_off;
            };
            CommunicationChannel.instance.OnMANUAL_CONTROLReceive += (src, ph, pack) =>
            {
                byte target = pack.target;
                short x = pack.x;
                short y = pack.y;
                short z = pack.z;
                short r = pack.r;
                ushort buttons = pack.buttons;
            };
            CommunicationChannel.instance.OnRC_CHANNELS_OVERRIDEReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnMISSION_ITEM_INTReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnVFR_HUDReceive += (src, ph, pack) =>
            {
                float airspeed = pack.airspeed;
                float groundspeed = pack.groundspeed;
                short heading = pack.heading;
                ushort throttle = pack.throttle;
                float alt = pack.alt;
                float climb = pack.climb;
            };
            CommunicationChannel.instance.OnCOMMAND_INTReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnCOMMAND_LONGReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnCOMMAND_ACKReceive += (src, ph, pack) =>
            {
                MAV_CMD command = pack.command;
                MAV_RESULT result = pack.result;
                byte progress = pack.progress_TRY(ph);
                int result_param2 = pack.result_param2_TRY(ph);
                byte target_system = pack.target_system_TRY(ph);
                byte target_component = pack.target_component_TRY(ph);
            };
            CommunicationChannel.instance.OnMANUAL_SETPOINTReceive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                float roll = pack.roll;
                float pitch = pack.pitch;
                float yaw = pack.yaw;
                float thrust = pack.thrust;
                byte mode_switch = pack.mode_switch;
                byte manual_override_switch = pack.manual_override_switch;
            };
            CommunicationChannel.instance.OnSET_ATTITUDE_TARGETReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                byte type_mask = pack.type_mask;
                float[] q = pack.q;
                float body_roll_rate = pack.body_roll_rate;
                float body_pitch_rate = pack.body_pitch_rate;
                float body_yaw_rate = pack.body_yaw_rate;
                float thrust = pack.thrust;
            };
            CommunicationChannel.instance.OnSET_POSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnSET_POSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnPOSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                float x = pack.x;
                float y = pack.y;
                float z = pack.z;
                float roll = pack.roll;
                float pitch = pack.pitch;
                float yaw = pack.yaw;
            };
            CommunicationChannel.instance.OnHIL_STATEReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnHIL_CONTROLSReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnHIL_RC_INPUTS_RAWReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnHIL_ACTUATOR_CONTROLSReceive += (src, ph, pack) =>
            {
                ulong time_usec = pack.time_usec;
                float[] controls = pack.controls;
                MAV_MODE mode = pack.mode;
                ulong flags = pack.flags;
            };
            CommunicationChannel.instance.OnOPTICAL_FLOWReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnGLOBAL_VISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                ulong usec = pack.usec;
                float x = pack.x;
                float y = pack.y;
                float z = pack.z;
                float roll = pack.roll;
                float pitch = pack.pitch;
                float yaw = pack.yaw;
            };
            CommunicationChannel.instance.OnVISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                ulong usec = pack.usec;
                float x = pack.x;
                float y = pack.y;
                float z = pack.z;
                float roll = pack.roll;
                float pitch = pack.pitch;
                float yaw = pack.yaw;
            };
            CommunicationChannel.instance.OnVISION_SPEED_ESTIMATEReceive += (src, ph, pack) =>
            {
                ulong usec = pack.usec;
                float x = pack.x;
                float y = pack.y;
                float z = pack.z;
            };
            CommunicationChannel.instance.OnVICON_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                ulong usec = pack.usec;
                float x = pack.x;
                float y = pack.y;
                float z = pack.z;
                float roll = pack.roll;
                float pitch = pack.pitch;
                float yaw = pack.yaw;
            };
            CommunicationChannel.instance.OnHIGHRES_IMUReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnOPTICAL_FLOW_RADReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnHIL_SENSORReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnSIM_STATEReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnRADIO_STATUSReceive += (src, ph, pack) =>
            {
                byte rssi = pack.rssi;
                byte remrssi = pack.remrssi;
                byte txbuf = pack.txbuf;
                byte noise = pack.noise;
                byte remnoise = pack.remnoise;
                ushort rxerrors = pack.rxerrors;
                ushort fixed_ = pack.fixed_;
            };
            CommunicationChannel.instance.OnFILE_TRANSFER_PROTOCOLReceive += (src, ph, pack) =>
            {
                byte target_network = pack.target_network;
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                byte[] payload = pack.payload;
            };
            CommunicationChannel.instance.OnTIMESYNCReceive += (src, ph, pack) =>
            {
                long tc1 = pack.tc1;
                long ts1 = pack.ts1;
            };
            CommunicationChannel.instance.OnCAMERA_TRIGGERReceive += (src, ph, pack) =>
            {
                ulong time_usec = pack.time_usec;
                uint seq = pack.seq;
            };
            CommunicationChannel.instance.OnHIL_GPSReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnHIL_OPTICAL_FLOWReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnHIL_STATE_QUATERNIONReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnSCALED_IMU2Receive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnLOG_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                ushort start = pack.start;
                ushort end = pack.end;
            };
            CommunicationChannel.instance.OnLOG_ENTRYReceive += (src, ph, pack) =>
            {
                ushort id = pack.id;
                ushort num_logs = pack.num_logs;
                ushort last_log_num = pack.last_log_num;
                uint time_utc = pack.time_utc;
                uint size = pack.size;
            };
            CommunicationChannel.instance.OnLOG_REQUEST_DATAReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                ushort id = pack.id;
                uint ofs = pack.ofs;
                uint count = pack.count;
            };
            CommunicationChannel.instance.OnLOG_DATAReceive += (src, ph, pack) =>
            {
                ushort id = pack.id;
                uint ofs = pack.ofs;
                byte count = pack.count;
                byte[] data_ = pack.data_;
            };
            CommunicationChannel.instance.OnLOG_ERASEReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
            };
            CommunicationChannel.instance.OnLOG_REQUEST_ENDReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
            };
            CommunicationChannel.instance.OnGPS_INJECT_DATAReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                byte len = pack.len;
                byte[] data_ = pack.data_;
            };
            CommunicationChannel.instance.OnGPS2_RAWReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnPOWER_STATUSReceive += (src, ph, pack) =>
            {
                ushort Vcc = pack.Vcc;
                ushort Vservo = pack.Vservo;
                MAV_POWER_STATUS flags = pack.flags;
            };
            CommunicationChannel.instance.OnSERIAL_CONTROLReceive += (src, ph, pack) =>
            {
                SERIAL_CONTROL_DEV device = pack.device;
                SERIAL_CONTROL_FLAG flags = pack.flags;
                ushort timeout = pack.timeout;
                uint baudrate = pack.baudrate;
                byte count = pack.count;
                byte[] data_ = pack.data_;
            };
            CommunicationChannel.instance.OnGPS_RTKReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnGPS2_RTKReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnSCALED_IMU3Receive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnDATA_TRANSMISSION_HANDSHAKEReceive += (src, ph, pack) =>
            {
                byte type = pack.type;
                uint size = pack.size;
                ushort width = pack.width;
                ushort height = pack.height;
                ushort packets = pack.packets;
                byte payload = pack.payload;
                byte jpg_quality = pack.jpg_quality;
            };
            CommunicationChannel.instance.OnENCAPSULATED_DATAReceive += (src, ph, pack) =>
            {
                ushort seqnr = pack.seqnr;
                byte[] data_ = pack.data_;
            };
            CommunicationChannel.instance.OnDISTANCE_SENSORReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnTERRAIN_REQUESTReceive += (src, ph, pack) =>
            {
                int lat = pack.lat;
                int lon = pack.lon;
                ushort grid_spacing = pack.grid_spacing;
                ulong mask = pack.mask;
            };
            CommunicationChannel.instance.OnTERRAIN_DATAReceive += (src, ph, pack) =>
            {
                int lat = pack.lat;
                int lon = pack.lon;
                ushort grid_spacing = pack.grid_spacing;
                byte gridbit = pack.gridbit;
                short[] data_ = pack.data_;
            };
            CommunicationChannel.instance.OnTERRAIN_CHECKReceive += (src, ph, pack) =>
            {
                int lat = pack.lat;
                int lon = pack.lon;
            };
            CommunicationChannel.instance.OnTERRAIN_REPORTReceive += (src, ph, pack) =>
            {
                int lat = pack.lat;
                int lon = pack.lon;
                ushort spacing = pack.spacing;
                float terrain_height = pack.terrain_height;
                float current_height = pack.current_height;
                ushort pending = pack.pending;
                ushort loaded = pack.loaded;
            };
            CommunicationChannel.instance.OnSCALED_PRESSURE2Receive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                float press_abs = pack.press_abs;
                float press_diff = pack.press_diff;
                short temperature = pack.temperature;
            };
            CommunicationChannel.instance.OnATT_POS_MOCAPReceive += (src, ph, pack) =>
            {
                ulong time_usec = pack.time_usec;
                float[] q = pack.q;
                float x = pack.x;
                float y = pack.y;
                float z = pack.z;
            };
            CommunicationChannel.instance.OnSET_ACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                ulong time_usec = pack.time_usec;
                byte group_mlx = pack.group_mlx;
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                float[] controls = pack.controls;
            };
            CommunicationChannel.instance.OnACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                ulong time_usec = pack.time_usec;
                byte group_mlx = pack.group_mlx;
                float[] controls = pack.controls;
            };
            CommunicationChannel.instance.OnALTITUDEReceive += (src, ph, pack) =>
            {
                ulong time_usec = pack.time_usec;
                float altitude_monotonic = pack.altitude_monotonic;
                float altitude_amsl = pack.altitude_amsl;
                float altitude_local = pack.altitude_local;
                float altitude_relative = pack.altitude_relative;
                float altitude_terrain = pack.altitude_terrain;
                float bottom_clearance = pack.bottom_clearance;
            };
            CommunicationChannel.instance.OnRESOURCE_REQUESTReceive += (src, ph, pack) =>
            {
                byte request_id = pack.request_id;
                byte uri_type = pack.uri_type;
                byte[] uri = pack.uri;
                byte transfer_type = pack.transfer_type;
                byte[] storage = pack.storage;
            };
            CommunicationChannel.instance.OnSCALED_PRESSURE3Receive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                float press_abs = pack.press_abs;
                float press_diff = pack.press_diff;
                short temperature = pack.temperature;
            };
            CommunicationChannel.instance.OnFOLLOW_TARGETReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnCONTROL_SYSTEM_STATEReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnBATTERY_STATUSReceive += (src, ph, pack) =>
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
            POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.time_boot_ms = (uint)4020317697U;
            p3.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_NED;
            p3.type_mask = (ushort)(ushort)11903;
            p3.x = (float)5.9275073E37F;
            p3.y = (float) -2.4703942E38F;
            p3.z = (float)1.7030212E38F;
            p3.vx = (float) -9.457459E37F;
            p3.vy = (float)1.0566369E38F;
            p3.vz = (float) -1.6399233E38F;
            p3.afx = (float)1.1833874E38F;
            p3.afy = (float)1.5412032E38F;
            p3.afz = (float)1.3952139E38F;
            p3.yaw = (float)1.183535E38F;
            p3.yaw_rate = (float) -1.0115422E38F;
            CommunicationChannel.instance.send(p3); //===============================
            SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.time_boot_ms = (uint)4278347027U;
            p82.target_system = (byte)(byte)148;
            p82.target_component = (byte)(byte)246;
            p82.type_mask = (byte)(byte)45;
            p82.q_SET(new float[4], 0);
            p82.body_roll_rate = (float)1.7279454E37F;
            p82.body_pitch_rate = (float) -2.503284E38F;
            p82.body_yaw_rate = (float) -1.8694443E38F;
            p82.thrust = (float)3.3641258E38F;
            CommunicationChannel.instance.send(p82); //===============================
            ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.time_boot_ms = (uint)3046754139U;
            p83.type_mask = (byte)(byte)145;
            p83.q_SET(new float[4], 0);
            p83.body_roll_rate = (float) -2.0053616E38F;
            p83.body_pitch_rate = (float)3.1525395E38F;
            p83.body_yaw_rate = (float)1.1917808E38F;
            p83.thrust = (float)6.111114E37F;
            CommunicationChannel.instance.send(p83); //===============================
            SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.time_boot_ms = (uint)3559170457U;
            p84.target_system = (byte)(byte)86;
            p84.target_component = (byte)(byte)202;
            p84.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_ENU;
            p84.type_mask = (ushort)(ushort)56934;
            p84.x = (float) -1.8078681E37F;
            p84.y = (float)3.2321614E38F;
            p84.z = (float)2.331795E38F;
            p84.vx = (float)9.185482E37F;
            p84.vy = (float) -2.1935619E38F;
            p84.vz = (float)2.1944529E38F;
            p84.afx = (float)2.6193318E38F;
            p84.afy = (float)2.0155064E38F;
            p84.afz = (float)2.1189658E38F;
            p84.yaw = (float)2.2645917E38F;
            p84.yaw_rate = (float) -8.5444385E36F;
            CommunicationChannel.instance.send(p84); //===============================
            SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.time_boot_ms = (uint)585760037U;
            p86.target_system = (byte)(byte)156;
            p86.target_component = (byte)(byte)79;
            p86.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL;
            p86.type_mask = (ushort)(ushort)16182;
            p86.lat_int = (int)1868527530;
            p86.lon_int = (int)1728839327;
            p86.alt = (float) -2.168796E37F;
            p86.vx = (float)1.558391E38F;
            p86.vy = (float) -3.2011674E38F;
            p86.vz = (float)1.1180087E38F;
            p86.afx = (float) -3.1866712E38F;
            p86.afy = (float) -8.324796E37F;
            p86.afz = (float)7.735416E37F;
            p86.yaw = (float)1.6961938E38F;
            p86.yaw_rate = (float) -1.7986025E38F;
            CommunicationChannel.instance.send(p86); //===============================
            POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.time_boot_ms = (uint)3930184427U;
            p87.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_ENU;
            p87.type_mask = (ushort)(ushort)25927;
            p87.lat_int = (int) -1781752268;
            p87.lon_int = (int)1118990996;
            p87.alt = (float) -2.1999208E38F;
            p87.vx = (float)2.0841886E38F;
            p87.vy = (float)3.2912426E38F;
            p87.vz = (float) -3.2810523E38F;
            p87.afx = (float)2.6616742E38F;
            p87.afy = (float)2.4895644E38F;
            p87.afz = (float)3.0196254E37F;
            p87.yaw = (float) -1.6548599E38F;
            p87.yaw_rate = (float)2.1152963E38F;
            CommunicationChannel.instance.send(p87); //===============================
            LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.time_boot_ms = (uint)2446062515U;
            p89.x = (float) -4.2676766E36F;
            p89.y = (float)1.3097251E38F;
            p89.z = (float) -7.992225E36F;
            p89.roll = (float)1.1396919E37F;
            p89.pitch = (float)1.0379681E38F;
            p89.yaw = (float)1.5039262E38F;
            CommunicationChannel.instance.send(p89); //===============================
            HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.time_usec = (ulong)1021606847194242580L;
            p90.roll = (float)3.127156E38F;
            p90.pitch = (float) -1.4198844E38F;
            p90.yaw = (float)1.7409429E38F;
            p90.rollspeed = (float) -2.4384121E37F;
            p90.pitchspeed = (float) -1.7301609E37F;
            p90.yawspeed = (float)3.1284102E38F;
            p90.lat = (int)630423997;
            p90.lon = (int) -608506276;
            p90.alt = (int)554562758;
            p90.vx = (short)(short)10398;
            p90.vy = (short)(short) -23754;
            p90.vz = (short)(short) -16586;
            p90.xacc = (short)(short) -27548;
            p90.yacc = (short)(short)1420;
            p90.zacc = (short)(short)31598;
            CommunicationChannel.instance.send(p90); //===============================
            HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.time_usec = (ulong)4347652055607102675L;
            p91.roll_ailerons = (float) -9.449294E37F;
            p91.pitch_elevator = (float) -1.5420332E38F;
            p91.yaw_rudder = (float) -2.0928147E38F;
            p91.throttle = (float) -1.4662873E38F;
            p91.aux1 = (float) -7.217215E37F;
            p91.aux2 = (float)1.9197386E38F;
            p91.aux3 = (float) -4.3095227E37F;
            p91.aux4 = (float) -9.83761E37F;
            p91.mode = MAV_MODE.MAV_MODE_MANUAL_ARMED;
            p91.nav_mode = (byte)(byte)185;
            CommunicationChannel.instance.send(p91); //===============================
            HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.time_usec = (ulong)7624465966013182204L;
            p92.chan1_raw = (ushort)(ushort)31438;
            p92.chan2_raw = (ushort)(ushort)48399;
            p92.chan3_raw = (ushort)(ushort)51888;
            p92.chan4_raw = (ushort)(ushort)33085;
            p92.chan5_raw = (ushort)(ushort)45269;
            p92.chan6_raw = (ushort)(ushort)14960;
            p92.chan7_raw = (ushort)(ushort)28456;
            p92.chan8_raw = (ushort)(ushort)11348;
            p92.chan9_raw = (ushort)(ushort)21569;
            p92.chan10_raw = (ushort)(ushort)29759;
            p92.chan11_raw = (ushort)(ushort)26555;
            p92.chan12_raw = (ushort)(ushort)56001;
            p92.rssi = (byte)(byte)16;
            CommunicationChannel.instance.send(p92); //===============================
            HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.time_usec = (ulong)6583903588268359288L;
            p93.controls_SET(new float[16], 0);
            p93.mode = MAV_MODE.MAV_MODE_TEST_ARMED;
            p93.flags = (ulong)1175653613499155104L;
            CommunicationChannel.instance.send(p93); //===============================
            OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.time_usec = (ulong)6175125268956678517L;
            p100.sensor_id = (byte)(byte)199;
            p100.flow_x = (short)(short) -9401;
            p100.flow_y = (short)(short) -16504;
            p100.flow_comp_m_x = (float)3.0055992E38F;
            p100.flow_comp_m_y = (float)7.8021445E37F;
            p100.quality = (byte)(byte)52;
            p100.ground_distance = (float) -6.9706814E37F;
            p100.flow_rate_x_SET((float)3.036257E38F, PH);
            p100.flow_rate_y_SET((float) -1.7380514E38F, PH);
            CommunicationChannel.instance.send(p100); //===============================
            GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.usec = (ulong)8494053768140176687L;
            p101.x = (float)1.6728248E37F;
            p101.y = (float)2.2126359E38F;
            p101.z = (float) -5.0380157E37F;
            p101.roll = (float)2.8620457E38F;
            p101.pitch = (float)3.3054758E38F;
            p101.yaw = (float) -3.7200353E37F;
            CommunicationChannel.instance.send(p101); //===============================
            VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.usec = (ulong)2078091534518649693L;
            p102.x = (float) -3.3712786E38F;
            p102.y = (float) -7.7933683E37F;
            p102.z = (float) -3.1731169E38F;
            p102.roll = (float) -9.237345E37F;
            p102.pitch = (float) -1.1342073E38F;
            p102.yaw = (float)1.8153558E38F;
            CommunicationChannel.instance.send(p102); //===============================
            VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.usec = (ulong)8800161693600254019L;
            p103.x = (float)2.917059E38F;
            p103.y = (float) -2.2559694E38F;
            p103.z = (float) -1.4067904E38F;
            CommunicationChannel.instance.send(p103); //===============================
            VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.usec = (ulong)7332564355980303575L;
            p104.x = (float) -2.9565733E38F;
            p104.y = (float) -3.4523574E37F;
            p104.z = (float)2.8192089E38F;
            p104.roll = (float)1.9396906E38F;
            p104.pitch = (float)3.3135604E38F;
            p104.yaw = (float)3.3414698E38F;
            CommunicationChannel.instance.send(p104); //===============================
            HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.time_usec = (ulong)2572732263630363336L;
            p105.xacc = (float)2.6309979E38F;
            p105.yacc = (float) -3.2884631E38F;
            p105.zacc = (float) -1.3985135E38F;
            p105.xgyro = (float)3.3776278E38F;
            p105.ygyro = (float)2.5655956E38F;
            p105.zgyro = (float)1.1097168E38F;
            p105.xmag = (float)2.2389397E38F;
            p105.ymag = (float) -2.2122355E38F;
            p105.zmag = (float)1.2568624E38F;
            p105.abs_pressure = (float)1.990565E38F;
            p105.diff_pressure = (float)2.895359E38F;
            p105.pressure_alt = (float) -1.2448547E38F;
            p105.temperature = (float)9.665946E37F;
            p105.fields_updated = (ushort)(ushort)12029;
            CommunicationChannel.instance.send(p105); //===============================
            OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.time_usec = (ulong)7157851515969873196L;
            p106.sensor_id = (byte)(byte)164;
            p106.integration_time_us = (uint)206231834U;
            p106.integrated_x = (float)3.1874482E38F;
            p106.integrated_y = (float) -3.3539416E38F;
            p106.integrated_xgyro = (float)4.87899E37F;
            p106.integrated_ygyro = (float)2.2828183E38F;
            p106.integrated_zgyro = (float)2.6463314E38F;
            p106.temperature = (short)(short)27533;
            p106.quality = (byte)(byte)2;
            p106.time_delta_distance_us = (uint)3756577839U;
            p106.distance = (float)2.4533672E38F;
            CommunicationChannel.instance.send(p106); //===============================
            HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.time_usec = (ulong)8104859081263078958L;
            p107.xacc = (float) -1.9812746E38F;
            p107.yacc = (float)9.64508E37F;
            p107.zacc = (float)2.7132702E38F;
            p107.xgyro = (float) -4.742899E37F;
            p107.ygyro = (float)8.140579E37F;
            p107.zgyro = (float)1.8501387E38F;
            p107.xmag = (float)3.2984987E38F;
            p107.ymag = (float)3.6289358E37F;
            p107.zmag = (float) -2.2401974E38F;
            p107.abs_pressure = (float) -1.1506952E38F;
            p107.diff_pressure = (float) -2.8585939E38F;
            p107.pressure_alt = (float)9.042491E37F;
            p107.temperature = (float) -1.9232454E38F;
            p107.fields_updated = (uint)2049969556U;
            CommunicationChannel.instance.send(p107); //===============================
            SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.q1 = (float) -1.6730436E38F;
            p108.q2 = (float) -1.3539046E38F;
            p108.q3 = (float)2.1921088E38F;
            p108.q4 = (float)1.7991718E38F;
            p108.roll = (float) -1.4965701E38F;
            p108.pitch = (float) -4.1004475E36F;
            p108.yaw = (float) -8.733101E37F;
            p108.xacc = (float) -3.394232E38F;
            p108.yacc = (float) -1.9311888E38F;
            p108.zacc = (float) -1.645393E38F;
            p108.xgyro = (float) -1.6799256E37F;
            p108.ygyro = (float) -3.3368083E38F;
            p108.zgyro = (float) -5.629851E37F;
            p108.lat = (float)1.917653E38F;
            p108.lon = (float)2.681793E38F;
            p108.alt = (float)7.659789E37F;
            p108.std_dev_horz = (float)5.038768E37F;
            p108.std_dev_vert = (float)3.2235913E38F;
            p108.vn = (float)3.9701285E37F;
            p108.ve = (float) -1.930338E38F;
            p108.vd = (float) -1.1289382E38F;
            CommunicationChannel.instance.send(p108); //===============================
            RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.rssi = (byte)(byte)78;
            p109.remrssi = (byte)(byte)181;
            p109.txbuf = (byte)(byte)159;
            p109.noise = (byte)(byte)26;
            p109.remnoise = (byte)(byte)222;
            p109.rxerrors = (ushort)(ushort)33053;
            p109.fixed_ = (ushort)(ushort)1804;
            CommunicationChannel.instance.send(p109); //===============================
            FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_network = (byte)(byte)253;
            p110.target_system = (byte)(byte)17;
            p110.target_component = (byte)(byte)71;
            p110.payload_SET(new byte[251], 0);
            CommunicationChannel.instance.send(p110); //===============================
            TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.tc1 = (long)468981727756086488L;
            p111.ts1 = (long)352527073402146826L;
            CommunicationChannel.instance.send(p111); //===============================
            CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.time_usec = (ulong)4438564184650351296L;
            p112.seq = (uint)2458606002U;
            CommunicationChannel.instance.send(p112); //===============================
            HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.time_usec = (ulong)4341659297965658687L;
            p113.fix_type = (byte)(byte)65;
            p113.lat = (int)834509845;
            p113.lon = (int)543611288;
            p113.alt = (int) -921720048;
            p113.eph = (ushort)(ushort)43517;
            p113.epv = (ushort)(ushort)16920;
            p113.vel = (ushort)(ushort)15587;
            p113.vn = (short)(short)23170;
            p113.ve = (short)(short) -12692;
            p113.vd = (short)(short)4551;
            p113.cog = (ushort)(ushort)39581;
            p113.satellites_visible = (byte)(byte)255;
            CommunicationChannel.instance.send(p113); //===============================
            HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.time_usec = (ulong)6211401954369555158L;
            p114.sensor_id = (byte)(byte)134;
            p114.integration_time_us = (uint)2691110054U;
            p114.integrated_x = (float)8.343349E37F;
            p114.integrated_y = (float) -6.360486E37F;
            p114.integrated_xgyro = (float)6.6208864E37F;
            p114.integrated_ygyro = (float) -9.661843E37F;
            p114.integrated_zgyro = (float)2.7474175E38F;
            p114.temperature = (short)(short) -10458;
            p114.quality = (byte)(byte)11;
            p114.time_delta_distance_us = (uint)3846373453U;
            p114.distance = (float)7.888432E36F;
            CommunicationChannel.instance.send(p114); //===============================
            HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.time_usec = (ulong)5177027592009577547L;
            p115.attitude_quaternion_SET(new float[4], 0);
            p115.rollspeed = (float) -8.356804E36F;
            p115.pitchspeed = (float) -1.909405E37F;
            p115.yawspeed = (float) -1.6864582E38F;
            p115.lat = (int)1355038947;
            p115.lon = (int) -927370471;
            p115.alt = (int)565227795;
            p115.vx = (short)(short) -2901;
            p115.vy = (short)(short) -25494;
            p115.vz = (short)(short)24206;
            p115.ind_airspeed = (ushort)(ushort)49706;
            p115.true_airspeed = (ushort)(ushort)61928;
            p115.xacc = (short)(short)24371;
            p115.yacc = (short)(short)31449;
            p115.zacc = (short)(short)28576;
            CommunicationChannel.instance.send(p115); //===============================
            SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.time_boot_ms = (uint)3354507323U;
            p116.xacc = (short)(short)6065;
            p116.yacc = (short)(short)9843;
            p116.zacc = (short)(short) -5759;
            p116.xgyro = (short)(short) -29763;
            p116.ygyro = (short)(short) -12179;
            p116.zgyro = (short)(short) -19550;
            p116.xmag = (short)(short) -21460;
            p116.ymag = (short)(short)23526;
            p116.zmag = (short)(short) -17831;
            CommunicationChannel.instance.send(p116); //===============================
            LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.target_system = (byte)(byte)236;
            p117.target_component = (byte)(byte)86;
            p117.start = (ushort)(ushort)58838;
            p117.end = (ushort)(ushort)61660;
            CommunicationChannel.instance.send(p117); //===============================
            LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.id = (ushort)(ushort)28493;
            p118.num_logs = (ushort)(ushort)45736;
            p118.last_log_num = (ushort)(ushort)1077;
            p118.time_utc = (uint)3754545573U;
            p118.size = (uint)2829983703U;
            CommunicationChannel.instance.send(p118); //===============================
            LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.target_system = (byte)(byte)115;
            p119.target_component = (byte)(byte)43;
            p119.id = (ushort)(ushort)50150;
            p119.ofs = (uint)660562309U;
            p119.count = (uint)1276394276U;
            CommunicationChannel.instance.send(p119); //===============================
            LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.id = (ushort)(ushort)40349;
            p120.ofs = (uint)858403287U;
            p120.count = (byte)(byte)220;
            p120.data__SET(new byte[90], 0);
            CommunicationChannel.instance.send(p120); //===============================
            LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_system = (byte)(byte)38;
            p121.target_component = (byte)(byte)223;
            CommunicationChannel.instance.send(p121); //===============================
            LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_system = (byte)(byte)73;
            p122.target_component = (byte)(byte)187;
            CommunicationChannel.instance.send(p122); //===============================
            GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.target_system = (byte)(byte)42;
            p123.target_component = (byte)(byte)201;
            p123.len = (byte)(byte)110;
            p123.data__SET(new byte[110], 0);
            CommunicationChannel.instance.send(p123); //===============================
            GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.time_usec = (ulong)5214911054917736672L;
            p124.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_PPP;
            p124.lat = (int)477247522;
            p124.lon = (int) -1712795905;
            p124.alt = (int) -2018133368;
            p124.eph = (ushort)(ushort)30486;
            p124.epv = (ushort)(ushort)25521;
            p124.vel = (ushort)(ushort)50077;
            p124.cog = (ushort)(ushort)21960;
            p124.satellites_visible = (byte)(byte)173;
            p124.dgps_numch = (byte)(byte)228;
            p124.dgps_age = (uint)2081686459U;
            CommunicationChannel.instance.send(p124); //===============================
            POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vcc = (ushort)(ushort)44092;
            p125.Vservo = (ushort)(ushort)27638;
            p125.flags = (MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT |
                          MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED |
                          MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID);
            CommunicationChannel.instance.send(p125); //===============================
            SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.device = SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1;
            p126.flags = (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING |
                          SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI |
                          SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY);
            p126.timeout = (ushort)(ushort)64927;
            p126.baudrate = (uint)612296448U;
            p126.count = (byte)(byte)227;
            p126.data__SET(new byte[70], 0);
            CommunicationChannel.instance.send(p126); //===============================
            GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.time_last_baseline_ms = (uint)3254164432U;
            p127.rtk_receiver_id = (byte)(byte)211;
            p127.wn = (ushort)(ushort)20996;
            p127.tow = (uint)3812200204U;
            p127.rtk_health = (byte)(byte)205;
            p127.rtk_rate = (byte)(byte)93;
            p127.nsats = (byte)(byte)174;
            p127.baseline_coords_type = (byte)(byte)179;
            p127.baseline_a_mm = (int) -1673192741;
            p127.baseline_b_mm = (int) -1636455511;
            p127.baseline_c_mm = (int) -1684428747;
            p127.accuracy = (uint)1390493006U;
            p127.iar_num_hypotheses = (int)1905121473;
            CommunicationChannel.instance.send(p127); //===============================
            GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.time_last_baseline_ms = (uint)319835830U;
            p128.rtk_receiver_id = (byte)(byte)91;
            p128.wn = (ushort)(ushort)2816;
            p128.tow = (uint)1248708967U;
            p128.rtk_health = (byte)(byte)141;
            p128.rtk_rate = (byte)(byte)48;
            p128.nsats = (byte)(byte)131;
            p128.baseline_coords_type = (byte)(byte)230;
            p128.baseline_a_mm = (int)682881918;
            p128.baseline_b_mm = (int) -2131074583;
            p128.baseline_c_mm = (int) -1742546961;
            p128.accuracy = (uint)4121237960U;
            p128.iar_num_hypotheses = (int)1610266704;
            CommunicationChannel.instance.send(p128); //===============================
            SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.time_boot_ms = (uint)941768203U;
            p129.xacc = (short)(short)26988;
            p129.yacc = (short)(short) -18414;
            p129.zacc = (short)(short) -16743;
            p129.xgyro = (short)(short) -6343;
            p129.ygyro = (short)(short) -28990;
            p129.zgyro = (short)(short) -20242;
            p129.xmag = (short)(short)9787;
            p129.ymag = (short)(short) -17628;
            p129.zmag = (short)(short)24527;
            CommunicationChannel.instance.send(p129); //===============================
            DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.type = (byte)(byte)4;
            p130.size = (uint)1673646999U;
            p130.width = (ushort)(ushort)1789;
            p130.height = (ushort)(ushort)14338;
            p130.packets = (ushort)(ushort)10000;
            p130.payload = (byte)(byte)176;
            p130.jpg_quality = (byte)(byte)208;
            CommunicationChannel.instance.send(p130); //===============================
            ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.seqnr = (ushort)(ushort)50803;
            p131.data__SET(new byte[253], 0);
            CommunicationChannel.instance.send(p131); //===============================
            DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.time_boot_ms = (uint)2251505553U;
            p132.min_distance = (ushort)(ushort)14104;
            p132.max_distance = (ushort)(ushort)27718;
            p132.current_distance = (ushort)(ushort)39472;
            p132.type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER;
            p132.id = (byte)(byte)167;
            p132.orientation = MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_90_YAW_90;
            p132.covariance = (byte)(byte)127;
            CommunicationChannel.instance.send(p132); //===============================
            TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.lat = (int)51570190;
            p133.lon = (int) -155476565;
            p133.grid_spacing = (ushort)(ushort)40574;
            p133.mask = (ulong)3495881136580767283L;
            CommunicationChannel.instance.send(p133); //===============================
            TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.lat = (int) -1029741807;
            p134.lon = (int) -1494056259;
            p134.grid_spacing = (ushort)(ushort)42927;
            p134.gridbit = (byte)(byte)117;
            p134.data__SET(new short[16], 0);
            CommunicationChannel.instance.send(p134); //===============================
            TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lat = (int)540965825;
            p135.lon = (int)743937330;
            CommunicationChannel.instance.send(p135); //===============================
            TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.lat = (int) -269760072;
            p136.lon = (int) -810896380;
            p136.spacing = (ushort)(ushort)56051;
            p136.terrain_height = (float) -3.2837319E38F;
            p136.current_height = (float) -4.2370948E37F;
            p136.pending = (ushort)(ushort)36220;
            p136.loaded = (ushort)(ushort)6300;
            CommunicationChannel.instance.send(p136); //===============================
            SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.time_boot_ms = (uint)775496700U;
            p137.press_abs = (float)3.1082292E38F;
            p137.press_diff = (float)1.4449661E38F;
            p137.temperature = (short)(short)29928;
            CommunicationChannel.instance.send(p137); //===============================
            ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.time_usec = (ulong)2933958153859060075L;
            p138.q_SET(new float[4], 0);
            p138.x = (float) -2.2024034E38F;
            p138.y = (float) -2.2520127E38F;
            p138.z = (float) -1.4591549E38F;
            CommunicationChannel.instance.send(p138); //===============================
            SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.time_usec = (ulong)9049157908812030202L;
            p139.group_mlx = (byte)(byte)152;
            p139.target_system = (byte)(byte)185;
            p139.target_component = (byte)(byte)126;
            p139.controls_SET(new float[8], 0);
            CommunicationChannel.instance.send(p139); //===============================
            ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.time_usec = (ulong)2972189954225072608L;
            p140.group_mlx = (byte)(byte)217;
            p140.controls_SET(new float[8], 0);
            CommunicationChannel.instance.send(p140); //===============================
            ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.time_usec = (ulong)9051157149027504375L;
            p141.altitude_monotonic = (float) -2.8586365E38F;
            p141.altitude_amsl = (float)7.2472203E37F;
            p141.altitude_local = (float) -9.4259044E36F;
            p141.altitude_relative = (float) -2.2809042E38F;
            p141.altitude_terrain = (float) -8.82573E37F;
            p141.bottom_clearance = (float)1.0536688E38F;
            CommunicationChannel.instance.send(p141); //===============================
            RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.request_id = (byte)(byte)154;
            p142.uri_type = (byte)(byte)125;
            p142.uri_SET(new byte[120], 0);
            p142.transfer_type = (byte)(byte)252;
            p142.storage_SET(new byte[120], 0);
            CommunicationChannel.instance.send(p142); //===============================
            SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.time_boot_ms = (uint)3746931239U;
            p143.press_abs = (float) -1.2088502E38F;
            p143.press_diff = (float) -5.3458996E37F;
            p143.temperature = (short)(short) -4188;
            CommunicationChannel.instance.send(p143); //===============================
            FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.timestamp = (ulong)8251467940142936964L;
            p144.est_capabilities = (byte)(byte)132;
            p144.lat = (int) -1288651166;
            p144.lon = (int) -1866638707;
            p144.alt = (float) -3.2420724E38F;
            p144.vel_SET(new float[3], 0);
            p144.acc_SET(new float[3], 0);
            p144.attitude_q_SET(new float[4], 0);
            p144.rates_SET(new float[3], 0);
            p144.position_cov_SET(new float[3], 0);
            p144.custom_state = (ulong)3111301974804971344L;
            CommunicationChannel.instance.send(p144); //===============================
            CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.time_usec = (ulong)7304924413122650650L;
            p146.x_acc = (float) -1.6547014E38F;
            p146.y_acc = (float) -1.4154834E37F;
            p146.z_acc = (float)2.6196452E38F;
            p146.x_vel = (float)2.1934994E38F;
            p146.y_vel = (float)2.7383198E38F;
            p146.z_vel = (float)1.1180023E38F;
            p146.x_pos = (float) -2.6739477E38F;
            p146.y_pos = (float)4.952098E37F;
            p146.z_pos = (float)2.2670706E38F;
            p146.airspeed = (float)1.3824043E38F;
            p146.vel_variance_SET(new float[3], 0);
            p146.pos_variance_SET(new float[3], 0);
            p146.q_SET(new float[4], 0);
            p146.roll_rate = (float) -3.2280834E38F;
            p146.pitch_rate = (float) -2.3528358E38F;
            p146.yaw_rate = (float)7.099957E37F;
            CommunicationChannel.instance.send(p146); //===============================
            BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.id = (byte)(byte)60;
            p147.battery_function = MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_ALL;
            p147.type = MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIPO;
            p147.temperature = (short)(short)2412;
            p147.voltages_SET(new ushort[10], 0);
            p147.current_battery = (short)(short) -31521;
            p147.current_consumed = (int)1958771817;
            p147.energy_consumed = (int) -253046031;
            p147.battery_remaining = (sbyte)(sbyte)115;
            CommunicationChannel.instance.send(p147); //===============================
            AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMMAND_INT);
            p148.flight_sw_version = (uint)1741048769U;
            p148.middleware_sw_version = (uint)632645404U;
            p148.os_sw_version = (uint)1356000302U;
            p148.board_version = (uint)4150265684U;
            p148.flight_custom_version_SET(new byte[8], 0);
            p148.middleware_custom_version_SET(new byte[8], 0);
            p148.os_custom_version_SET(new byte[8], 0);
            p148.vendor_id = (ushort)(ushort)31979;
            p148.product_id = (ushort)(ushort)21029;
            p148.uid = (ulong)7478340118032578610L;
            p148.uid2_SET(new byte[18], 0, PH);
            CommunicationChannel.instance.send(p148); //===============================
            LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.time_usec = (ulong)4682446349920480557L;
            p149.target_num = (byte)(byte)196;
            p149.frame = MAV_FRAME.MAV_FRAME_MISSION;
            p149.angle_x = (float) -1.3127641E38F;
            p149.angle_y = (float) -8.575668E37F;
            p149.distance = (float) -9.6953E37F;
            p149.size_x = (float)1.7498067E38F;
            p149.size_y = (float) -1.6289903E38F;
            p149.x_SET((float)2.2745992E38F, PH);
            p149.y_SET((float) -3.2150015E38F, PH);
            p149.z_SET((float)3.113289E38F, PH);
            p149.q_SET(new float[4], 0, PH);
            p149.type = LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON;
            p149.position_valid_SET((byte)(byte)143, PH);
            CommunicationChannel.instance.send(p149); //===============================
            SENS_POWER p201 = CommunicationChannel.new_SENS_POWER();
            PH.setPack(p201);
            p201.adc121_vspb_volt = (float)1.6369134E38F;
            p201.adc121_cspb_amp = (float) -1.2915049E38F;
            p201.adc121_cs1_amp = (float)1.2522783E38F;
            p201.adc121_cs2_amp = (float) -1.5472214E38F;
            CommunicationChannel.instance.send(p201); //===============================
            SENS_MPPT p202 = CommunicationChannel.new_SENS_MPPT();
            PH.setPack(p202);
            p202.mppt_timestamp = (ulong)8432658390912101177L;
            p202.mppt1_volt = (float)6.2475627E37F;
            p202.mppt1_amp = (float) -1.9977451E38F;
            p202.mppt1_pwm = (ushort)(ushort)41563;
            p202.mppt1_status = (byte)(byte)61;
            p202.mppt2_volt = (float) -1.2462358E37F;
            p202.mppt2_amp = (float) -3.207203E38F;
            p202.mppt2_pwm = (ushort)(ushort)21440;
            p202.mppt2_status = (byte)(byte)153;
            p202.mppt3_volt = (float) -1.4412795E38F;
            p202.mppt3_amp = (float)5.588643E37F;
            p202.mppt3_pwm = (ushort)(ushort)41775;
            p202.mppt3_status = (byte)(byte)12;
            CommunicationChannel.instance.send(p202); //===============================
            ASLCTRL_DATA p203 = CommunicationChannel.new_ASLCTRL_DATA();
            PH.setPack(p203);
            p203.timestamp = (ulong)884488114800602516L;
            p203.aslctrl_mode = (byte)(byte)197;
            p203.h = (float)8.575795E37F;
            p203.hRef = (float) -8.955742E37F;
            p203.hRef_t = (float) -2.1947291E38F;
            p203.PitchAngle = (float)1.1398516E38F;
            p203.PitchAngleRef = (float) -3.2263949E38F;
            p203.q = (float) -8.0303906E37F;
            p203.qRef = (float)7.8004063E37F;
            p203.uElev = (float)2.5163065E38F;
            p203.uThrot = (float)1.1882201E38F;
            p203.uThrot2 = (float) -1.7970907E38F;
            p203.nZ = (float) -1.3096139E38F;
            p203.AirspeedRef = (float)2.823567E38F;
            p203.SpoilersEngaged = (byte)(byte)125;
            p203.YawAngle = (float) -9.241366E37F;
            p203.YawAngleRef = (float)1.0367117E38F;
            p203.RollAngle = (float) -7.956648E37F;
            p203.RollAngleRef = (float)1.6215314E38F;
            p203.p = (float)1.4760807E38F;
            p203.pRef = (float)3.1364933E38F;
            p203.r = (float)2.350023E38F;
            p203.rRef = (float)2.4798594E38F;
            p203.uAil = (float) -1.9342239E38F;
            p203.uRud = (float)3.283126E37F;
            CommunicationChannel.instance.send(p203); //===============================
            ASLCTRL_DEBUG p204 = CommunicationChannel.new_ASLCTRL_DEBUG();
            PH.setPack(p204);
            p204.i32_1 = (uint)1850815789U;
            p204.i8_1 = (byte)(byte)129;
            p204.i8_2 = (byte)(byte)238;
            p204.f_1 = (float)3.0854135E38F;
            p204.f_2 = (float)2.8026345E37F;
            p204.f_3 = (float) -2.1599476E38F;
            p204.f_4 = (float)1.8825544E38F;
            p204.f_5 = (float) -3.0831149E38F;
            p204.f_6 = (float) -1.6896901E38F;
            p204.f_7 = (float) -3.05119E38F;
            p204.f_8 = (float) -2.7087412E38F;
            CommunicationChannel.instance.send(p204); //===============================
            ASLUAV_STATUS p205 = CommunicationChannel.new_ASLUAV_STATUS();
            PH.setPack(p205);
            p205.LED_status = (byte)(byte)150;
            p205.SATCOM_status = (byte)(byte)4;
            p205.Servo_status_SET(new byte[8], 0);
            p205.Motor_rpm = (float) -2.2391237E38F;
            CommunicationChannel.instance.send(p205); //===============================
            EKF_EXT p206 = CommunicationChannel.new_EKF_EXT();
            PH.setPack(p206);
            p206.timestamp = (ulong)7774673177447428163L;
            p206.Windspeed = (float) -2.486498E38F;
            p206.WindDir = (float)1.9902796E38F;
            p206.WindZ = (float)2.6879036E38F;
            p206.Airspeed = (float) -6.630195E37F;
            p206.beta = (float)3.3974882E38F;
            p206.alpha = (float)2.4872207E38F;
            CommunicationChannel.instance.send(p206); //===============================
            ASL_OBCTRL p207 = CommunicationChannel.new_ASL_OBCTRL();
            PH.setPack(p207);
            p207.timestamp = (ulong)5826661841110181443L;
            p207.uElev = (float)1.1067789E38F;
            p207.uThrot = (float) -8.903331E37F;
            p207.uThrot2 = (float)4.594005E37F;
            p207.uAilL = (float) -2.466616E38F;
            p207.uAilR = (float)6.34091E37F;
            p207.uRud = (float)9.08852E37F;
            p207.obctrl_status = (byte)(byte)197;
            CommunicationChannel.instance.send(p207); //===============================
            SENS_ATMOS p208 = CommunicationChannel.new_SENS_ATMOS();
            PH.setPack(p208);
            p208.TempAmbient = (float)1.707511E38F;
            p208.Humidity = (float) -3.3558942E38F;
            CommunicationChannel.instance.send(p208); //===============================
            SENS_BATMON p209 = CommunicationChannel.new_SENS_BATMON();
            PH.setPack(p209);
            p209.temperature = (float) -7.214369E37F;
            p209.voltage = (ushort)(ushort)38819;
            p209.current = (short)(short) -3144;
            p209.SoC = (byte)(byte)113;
            p209.batterystatus = (ushort)(ushort)19793;
            p209.serialnumber = (ushort)(ushort)36863;
            p209.hostfetcontrol = (ushort)(ushort)55400;
            p209.cellvoltage1 = (ushort)(ushort)20834;
            p209.cellvoltage2 = (ushort)(ushort)63873;
            p209.cellvoltage3 = (ushort)(ushort)33302;
            p209.cellvoltage4 = (ushort)(ushort)51089;
            p209.cellvoltage5 = (ushort)(ushort)27055;
            p209.cellvoltage6 = (ushort)(ushort)39241;
            CommunicationChannel.instance.send(p209); //===============================
            FW_SOARING_DATA p210 = CommunicationChannel.new_FW_SOARING_DATA();
            PH.setPack(p210);
            p210.timestamp = (ulong)6516729874237369955L;
            p210.timestampModeChanged = (ulong)2955865269303024962L;
            p210.xW = (float) -2.7182252E38F;
            p210.xR = (float)5.919611E37F;
            p210.xLat = (float) -1.6783886E38F;
            p210.xLon = (float)2.0251552E38F;
            p210.VarW = (float)2.9368844E38F;
            p210.VarR = (float)3.078371E38F;
            p210.VarLat = (float) -2.4663643E38F;
            p210.VarLon = (float) -1.9567143E37F;
            p210.LoiterRadius = (float)2.913298E38F;
            p210.LoiterDirection = (float)9.657758E37F;
            p210.DistToSoarPoint = (float)6.844298E37F;
            p210.vSinkExp = (float)1.1074327E37F;
            p210.z1_LocalUpdraftSpeed = (float)1.4760602E38F;
            p210.z2_DeltaRoll = (float)8.973975E37F;
            p210.z1_exp = (float)6.824179E37F;
            p210.z2_exp = (float)2.0861016E38F;
            p210.ThermalGSNorth = (float)1.7507443E38F;
            p210.ThermalGSEast = (float) -2.9704698E38F;
            p210.TSE_dot = (float)7.8417677E37F;
            p210.DebugVar1 = (float) -1.1516115E38F;
            p210.DebugVar2 = (float) -7.496522E37F;
            p210.ControlMode = (byte)(byte)91;
            p210.valid = (byte)(byte)43;
            CommunicationChannel.instance.send(p210); //===============================
            SENSORPOD_STATUS p211 = CommunicationChannel.new_SENSORPOD_STATUS();
            PH.setPack(p211);
            p211.timestamp = (ulong)1469972252005418794L;
            p211.visensor_rate_1 = (byte)(byte)13;
            p211.visensor_rate_2 = (byte)(byte)128;
            p211.visensor_rate_3 = (byte)(byte)15;
            p211.visensor_rate_4 = (byte)(byte)253;
            p211.recording_nodes_count = (byte)(byte)170;
            p211.cpu_temp = (byte)(byte)241;
            p211.free_space = (ushort)(ushort)18789;
            CommunicationChannel.instance.send(p211); //===============================
            SENS_POWER_BOARD p212 = CommunicationChannel.new_SENS_POWER_BOARD();
            PH.setPack(p212);
            p212.timestamp = (ulong)4695179922935282555L;
            p212.pwr_brd_status = (byte)(byte)42;
            p212.pwr_brd_led_status = (byte)(byte)237;
            p212.pwr_brd_system_volt = (float)1.3184002E38F;
            p212.pwr_brd_servo_volt = (float)1.9081048E38F;
            p212.pwr_brd_mot_l_amp = (float) -1.4303766E38F;
            p212.pwr_brd_mot_r_amp = (float) -3.0796606E38F;
            p212.pwr_brd_servo_1_amp = (float)2.873662E38F;
            p212.pwr_brd_servo_2_amp = (float) -8.129885E37F;
            p212.pwr_brd_servo_3_amp = (float) -5.8071495E37F;
            p212.pwr_brd_servo_4_amp = (float) -1.3295494E38F;
            p212.pwr_brd_aux_amp = (float) -2.951928E38F;
            CommunicationChannel.instance.send(p212); //===============================
            ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.time_usec = (ulong)3412411723248364164L;
            p230.flags = (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH);
            p230.vel_ratio = (float)6.040155E37F;
            p230.pos_horiz_ratio = (float) -2.7342641E38F;
            p230.pos_vert_ratio = (float) -1.1310189E38F;
            p230.mag_ratio = (float) -4.917021E37F;
            p230.hagl_ratio = (float) -2.9654745E38F;
            p230.tas_ratio = (float)9.016463E37F;
            p230.pos_horiz_accuracy = (float)3.2541398E38F;
            p230.pos_vert_accuracy = (float)2.8382873E38F;
            CommunicationChannel.instance.send(p230); //===============================
            WIND_COV p231 = CommunicationChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.time_usec = (ulong)1165268150641697332L;
            p231.wind_x = (float)3.1275648E38F;
            p231.wind_y = (float) -2.6525094E38F;
            p231.wind_z = (float)2.2182245E38F;
            p231.var_horiz = (float)1.2896659E38F;
            p231.var_vert = (float) -1.7393483E38F;
            p231.wind_alt = (float)1.9888401E38F;
            p231.horiz_accuracy = (float) -1.0474447E38F;
            p231.vert_accuracy = (float)2.5919181E38F;
            CommunicationChannel.instance.send(p231); //===============================
            GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.time_usec = (ulong)1848480598460615887L;
            p232.gps_id = (byte)(byte)61;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT);
            p232.time_week_ms = (uint)253679363U;
            p232.time_week = (ushort)(ushort)55007;
            p232.fix_type = (byte)(byte)86;
            p232.lat = (int)1025336849;
            p232.lon = (int)774064185;
            p232.alt = (float) -1.4510325E38F;
            p232.hdop = (float)1.5141998E38F;
            p232.vdop = (float)4.1648394E37F;
            p232.vn = (float) -2.480489E38F;
            p232.ve = (float) -3.3746703E38F;
            p232.vd = (float) -7.027678E36F;
            p232.speed_accuracy = (float)3.1224587E38F;
            p232.horiz_accuracy = (float) -5.1484656E37F;
            p232.vert_accuracy = (float) -1.6093805E38F;
            p232.satellites_visible = (byte)(byte)8;
            CommunicationChannel.instance.send(p232); //===============================
            GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.flags = (byte)(byte)26;
            p233.len = (byte)(byte)76;
            p233.data__SET(new byte[180], 0);
            CommunicationChannel.instance.send(p233); //===============================
            HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED);
            p234.custom_mode = (uint)3258762827U;
            p234.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED;
            p234.roll = (short)(short) -21079;
            p234.pitch = (short)(short)22350;
            p234.heading = (ushort)(ushort)5422;
            p234.throttle = (sbyte)(sbyte)7;
            p234.heading_sp = (short)(short) -30099;
            p234.latitude = (int) -637787647;
            p234.longitude = (int) -1860215631;
            p234.altitude_amsl = (short)(short) -24567;
            p234.altitude_sp = (short)(short) -8343;
            p234.airspeed = (byte)(byte)182;
            p234.airspeed_sp = (byte)(byte)41;
            p234.groundspeed = (byte)(byte)55;
            p234.climb_rate = (sbyte)(sbyte) - 44;
            p234.gps_nsat = (byte)(byte)144;
            p234.gps_fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS;
            p234.battery_remaining = (byte)(byte)53;
            p234.temperature = (sbyte)(sbyte) - 42;
            p234.temperature_air = (sbyte)(sbyte)110;
            p234.failsafe = (byte)(byte)59;
            p234.wp_num = (byte)(byte)67;
            p234.wp_distance = (ushort)(ushort)48066;
            CommunicationChannel.instance.send(p234); //===============================
            VIBRATION p241 = CommunicationChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.time_usec = (ulong)8199151137888744541L;
            p241.vibration_x = (float)7.3138947E37F;
            p241.vibration_y = (float) -9.75861E37F;
            p241.vibration_z = (float)1.5591716E38F;
            p241.clipping_0 = (uint)1891128574U;
            p241.clipping_1 = (uint)4257908679U;
            p241.clipping_2 = (uint)1625865854U;
            CommunicationChannel.instance.send(p241); //===============================
            HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.latitude = (int) -1355002242;
            p242.longitude = (int) -1811493953;
            p242.altitude = (int) -1171895029;
            p242.x = (float) -3.3216924E38F;
            p242.y = (float)2.1699842E38F;
            p242.z = (float) -2.222459E38F;
            p242.q_SET(new float[4], 0);
            p242.approach_x = (float) -1.8028527E38F;
            p242.approach_y = (float)3.2429232E38F;
            p242.approach_z = (float) -3.0719807E38F;
            p242.time_usec_SET((ulong)1896273293327987231L, PH);
            CommunicationChannel.instance.send(p242); //===============================
            SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.target_system = (byte)(byte)179;
            p243.latitude = (int)952906014;
            p243.longitude = (int) -926462391;
            p243.altitude = (int)79595995;
            p243.x = (float)1.722037E38F;
            p243.y = (float) -1.3851735E37F;
            p243.z = (float) -4.755435E37F;
            p243.q_SET(new float[4], 0);
            p243.approach_x = (float) -1.702682E38F;
            p243.approach_y = (float) -1.7634362E37F;
            p243.approach_z = (float) -6.1052953E37F;
            p243.time_usec_SET((ulong)1562463061268324614L, PH);
            CommunicationChannel.instance.send(p243); //===============================
            MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.message_id = (ushort)(ushort)31779;
            p244.interval_us = (int)741099125;
            CommunicationChannel.instance.send(p244); //===============================
            EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.vtol_state = MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_FW;
            p245.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED;
            CommunicationChannel.instance.send(p245); //===============================
            ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.ICAO_address = (uint)4027626150U;
            p246.lat = (int) -1281828221;
            p246.lon = (int)977682545;
            p246.altitude_type = ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
            p246.altitude = (int) -433081678;
            p246.heading = (ushort)(ushort)43315;
            p246.hor_velocity = (ushort)(ushort)64274;
            p246.ver_velocity = (short)(short) -11917;
            p246.callsign_SET("DEMO", PH);
            p246.emitter_type = ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UNASSGINED3;
            p246.tslc = (byte)(byte)23;
            p246.flags = (ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY |
                          ADSB_FLAGS.ADSB_FLAGS_SIMULATED |
                          ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK);
            p246.squawk = (ushort)(ushort)23405;
            CommunicationChannel.instance.send(p246); //===============================
            COLLISION p247 = CommunicationChannel.new_COLLISION();
            PH.setPack(p247);
            p247.src_ = MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT;
            p247.id = (uint)1797084297U;
            p247.action = MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_NONE;
            p247.threat_level = (MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW);
            p247.time_to_minimum_delta = (float)1.6923965E38F;
            p247.altitude_minimum_delta = (float) -2.8398694E38F;
            p247.horizontal_minimum_delta = (float) -1.6465242E37F;
            CommunicationChannel.instance.send(p247); //===============================
            V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.target_network = (byte)(byte)139;
            p248.target_system = (byte)(byte)95;
            p248.target_component = (byte)(byte)140;
            p248.message_type = (ushort)(ushort)31852;
            p248.payload_SET(new byte[249], 0);
            CommunicationChannel.instance.send(p248); //===============================
            MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.address = (ushort)(ushort)59852;
            p249.ver = (byte)(byte)109;
            p249.type = (byte)(byte)93;
            p249.value_SET(new sbyte[32], 0);
            CommunicationChannel.instance.send(p249); //===============================
            DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.name_SET("DEMO", PH);
            p250.time_usec = (ulong)484591643106935596L;
            p250.x = (float) -2.0509595E38F;
            p250.y = (float) -1.833259E38F;
            p250.z = (float)1.7171516E38F;
            CommunicationChannel.instance.send(p250); //===============================
            NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.time_boot_ms = (uint)4066405338U;
            p251.name_SET("DEMO", PH);
            p251.value = (float) -2.4323236E38F;
            CommunicationChannel.instance.send(p251); //===============================
            NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.time_boot_ms = (uint)316535305U;
            p252.name_SET("DEMO", PH);
            p252.value = (int) -1936604758;
            CommunicationChannel.instance.send(p252); //===============================
            STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.severity = MAV_SEVERITY.MAV_SEVERITY_EMERGENCY;
            p253.text_SET("DEMO", PH);
            CommunicationChannel.instance.send(p253); //===============================
            DEBUG p254 = CommunicationChannel.new_DEBUG();
            PH.setPack(p254);
            p254.time_boot_ms = (uint)2751364915U;
            p254.ind = (byte)(byte)251;
            p254.value = (float) -2.683498E38F;
            CommunicationChannel.instance.send(p254); //===============================
            SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.target_system = (byte)(byte)234;
            p256.target_component = (byte)(byte)198;
            p256.secret_key_SET(new byte[32], 0);
            p256.initial_timestamp = (ulong)3836026501227708327L;
            CommunicationChannel.instance.send(p256); //===============================
            BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.time_boot_ms = (uint)3702537910U;
            p257.last_change_ms = (uint)1329323229U;
            p257.state = (byte)(byte)48;
            CommunicationChannel.instance.send(p257); //===============================
            PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_system = (byte)(byte)142;
            p258.target_component = (byte)(byte)180;
            p258.tune_SET("DEMO", PH);
            CommunicationChannel.instance.send(p258); //===============================
            CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.time_boot_ms = (uint)824833593U;
            p259.vendor_name_SET(new byte[32], 0);
            p259.model_name_SET(new byte[32], 0);
            p259.firmware_version = (uint)2834791U;
            p259.focal_length = (float)1.5438558E38F;
            p259.sensor_size_h = (float)1.7297011E38F;
            p259.sensor_size_v = (float) -2.1692157E38F;
            p259.resolution_h = (ushort)(ushort)6939;
            p259.resolution_v = (ushort)(ushort)33149;
            p259.lens_id = (byte)(byte)172;
            p259.flags = (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
                          CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE |
                          CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE);
            p259.cam_definition_version = (ushort)(ushort)29832;
            p259.cam_definition_uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p259); //===============================
            CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.time_boot_ms = (uint)3710421705U;
            p260.mode_id = (CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY);
            CommunicationChannel.instance.send(p260); //===============================
            STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.time_boot_ms = (uint)27775703U;
            p261.storage_id = (byte)(byte)164;
            p261.storage_count = (byte)(byte)254;
            p261.status = (byte)(byte)171;
            p261.total_capacity = (float)3.0230836E38F;
            p261.used_capacity = (float) -2.516966E36F;
            p261.available_capacity = (float) -2.4756162E37F;
            p261.read_speed = (float) -1.3267654E38F;
            p261.write_speed = (float) -2.5890273E38F;
            CommunicationChannel.instance.send(p261); //===============================
            CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.time_boot_ms = (uint)224876454U;
            p262.image_status = (byte)(byte)137;
            p262.video_status = (byte)(byte)76;
            p262.image_interval = (float)1.2608996E37F;
            p262.recording_time_ms = (uint)3575226545U;
            p262.available_capacity = (float) -6.0567747E37F;
            CommunicationChannel.instance.send(p262); //===============================
            CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.time_boot_ms = (uint)4163468742U;
            p263.time_utc = (ulong)6690921351772611929L;
            p263.camera_id = (byte)(byte)75;
            p263.lat = (int)78330101;
            p263.lon = (int)1166355675;
            p263.alt = (int)656582230;
            p263.relative_alt = (int) -1842836301;
            p263.q_SET(new float[4], 0);
            p263.image_index = (int)752478171;
            p263.capture_result = (sbyte)(sbyte) - 68;
            p263.file_url_SET("DEMO", PH);
            CommunicationChannel.instance.send(p263); //===============================
            FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.time_boot_ms = (uint)1696415993U;
            p264.arming_time_utc = (ulong)8102753711086355731L;
            p264.takeoff_time_utc = (ulong)4124010698301213522L;
            p264.flight_uuid = (ulong)177867065774566342L;
            CommunicationChannel.instance.send(p264); //===============================
            MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.time_boot_ms = (uint)3601078171U;
            p265.roll = (float) -3.1251617E38F;
            p265.pitch = (float)2.5238548E38F;
            p265.yaw = (float) -2.6415601E38F;
            CommunicationChannel.instance.send(p265); //===============================
            LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.target_system = (byte)(byte)174;
            p266.target_component = (byte)(byte)31;
            p266.sequence = (ushort)(ushort)11882;
            p266.length = (byte)(byte)45;
            p266.first_message_offset = (byte)(byte)103;
            p266.data__SET(new byte[249], 0);
            CommunicationChannel.instance.send(p266); //===============================
            LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.target_system = (byte)(byte)18;
            p267.target_component = (byte)(byte)33;
            p267.sequence = (ushort)(ushort)55184;
            p267.length = (byte)(byte)173;
            p267.first_message_offset = (byte)(byte)23;
            p267.data__SET(new byte[249], 0);
            CommunicationChannel.instance.send(p267); //===============================
            LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_system = (byte)(byte)59;
            p268.target_component = (byte)(byte)22;
            p268.sequence = (ushort)(ushort)45234;
            CommunicationChannel.instance.send(p268); //===============================
            VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.camera_id = (byte)(byte)183;
            p269.status = (byte)(byte)241;
            p269.framerate = (float) -6.471624E37F;
            p269.resolution_h = (ushort)(ushort)43857;
            p269.resolution_v = (ushort)(ushort)28159;
            p269.bitrate = (uint)3897778U;
            p269.rotation = (ushort)(ushort)19128;
            p269.uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p269); //===============================
            SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.target_system = (byte)(byte)213;
            p270.target_component = (byte)(byte)170;
            p270.camera_id = (byte)(byte)254;
            p270.framerate = (float)1.958423E38F;
            p270.resolution_h = (ushort)(ushort)22943;
            p270.resolution_v = (ushort)(ushort)22713;
            p270.bitrate = (uint)2019294714U;
            p270.rotation = (ushort)(ushort)3954;
            p270.uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p270); //===============================
            WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.ssid_SET("DEMO", PH);
            p299.password_SET("DEMO", PH);
            CommunicationChannel.instance.send(p299); //===============================
            PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.version = (ushort)(ushort)57092;
            p300.min_version = (ushort)(ushort)21138;
            p300.max_version = (ushort)(ushort)61804;
            p300.spec_version_hash_SET(new byte[8], 0);
            p300.library_version_hash_SET(new byte[8], 0);
            CommunicationChannel.instance.send(p300); //===============================
            UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.time_usec = (ulong)3241748559155640631L;
            p310.uptime_sec = (uint)3994886971U;
            p310.health = UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING;
            p310.mode = UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE;
            p310.sub_mode = (byte)(byte)6;
            p310.vendor_specific_status_code = (ushort)(ushort)35152;
            CommunicationChannel.instance.send(p310); //===============================
            UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.time_usec = (ulong)1113595867528275841L;
            p311.uptime_sec = (uint)2484007037U;
            p311.name_SET("DEMO", PH);
            p311.hw_version_major = (byte)(byte)186;
            p311.hw_version_minor = (byte)(byte)148;
            p311.hw_unique_id_SET(new byte[16], 0);
            p311.sw_version_major = (byte)(byte)174;
            p311.sw_version_minor = (byte)(byte)186;
            p311.sw_vcs_commit = (uint)1613067593U;
            CommunicationChannel.instance.send(p311); //===============================
            PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.target_system = (byte)(byte)88;
            p320.target_component = (byte)(byte)147;
            p320.param_id_SET("DEMO", PH);
            p320.param_index = (short)(short)27874;
            CommunicationChannel.instance.send(p320); //===============================
            PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_system = (byte)(byte)114;
            p321.target_component = (byte)(byte)217;
            CommunicationChannel.instance.send(p321); //===============================
            PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_id_SET("DEMO", PH);
            p322.param_value_SET("DEMO", PH);
            p322.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32;
            p322.param_count = (ushort)(ushort)39513;
            p322.param_index = (ushort)(ushort)4489;
            CommunicationChannel.instance.send(p322); //===============================
            PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.target_system = (byte)(byte)161;
            p323.target_component = (byte)(byte)168;
            p323.param_id_SET("DEMO", PH);
            p323.param_value_SET("DEMO", PH);
            p323.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32;
            CommunicationChannel.instance.send(p323); //===============================
            PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_id_SET("DEMO", PH);
            p324.param_value_SET("DEMO", PH);
            p324.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM;
            p324.param_result = PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED;
            CommunicationChannel.instance.send(p324); //===============================
            OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.time_usec = (ulong)8093512808214071265L;
            p330.sensor_type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN;
            p330.distances_SET(new ushort[72], 0);
            p330.increment = (byte)(byte)26;
            p330.min_distance = (ushort)(ushort)30684;
            p330.max_distance = (ushort)(ushort)49830;
            CommunicationChannel.instance.send(p330); //===============================
        }
    }
}

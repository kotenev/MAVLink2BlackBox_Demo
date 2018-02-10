
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
            p3.time_boot_ms = (uint)505198683U;
            p3.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT;
            p3.type_mask = (ushort)(ushort)44063;
            p3.x = (float) -1.740376E37F;
            p3.y = (float)2.201165E37F;
            p3.z = (float)2.3384267E37F;
            p3.vx = (float)2.115556E38F;
            p3.vy = (float) -3.191447E38F;
            p3.vz = (float)2.5653494E38F;
            p3.afx = (float) -1.1494109E38F;
            p3.afy = (float)3.2293358E37F;
            p3.afz = (float)6.4351116E36F;
            p3.yaw = (float)2.25163E38F;
            p3.yaw_rate = (float)3.3293754E38F;
            CommunicationChannel.instance.send(p3); //===============================
            SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.time_boot_ms = (uint)1160911492U;
            p82.target_system = (byte)(byte)206;
            p82.target_component = (byte)(byte)217;
            p82.type_mask = (byte)(byte)135;
            p82.q_SET(new float[4], 0);
            p82.body_roll_rate = (float)2.0555374E38F;
            p82.body_pitch_rate = (float)1.6144829E38F;
            p82.body_yaw_rate = (float)6.7711907E37F;
            p82.thrust = (float) -1.4680665E38F;
            CommunicationChannel.instance.send(p82); //===============================
            ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.time_boot_ms = (uint)766220510U;
            p83.type_mask = (byte)(byte)69;
            p83.q_SET(new float[4], 0);
            p83.body_roll_rate = (float) -1.563495E38F;
            p83.body_pitch_rate = (float) -7.911677E37F;
            p83.body_yaw_rate = (float) -9.317379E37F;
            p83.thrust = (float) -2.7964387E37F;
            CommunicationChannel.instance.send(p83); //===============================
            SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.time_boot_ms = (uint)3981154981U;
            p84.target_system = (byte)(byte)179;
            p84.target_component = (byte)(byte)58;
            p84.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
            p84.type_mask = (ushort)(ushort)44409;
            p84.x = (float)5.11679E37F;
            p84.y = (float) -2.899998E37F;
            p84.z = (float)8.2977396E37F;
            p84.vx = (float)1.8731728E38F;
            p84.vy = (float) -1.1996686E38F;
            p84.vz = (float) -2.3054214E38F;
            p84.afx = (float) -1.84252E38F;
            p84.afy = (float)2.6739294E38F;
            p84.afz = (float) -3.252719E37F;
            p84.yaw = (float) -1.2137551E38F;
            p84.yaw_rate = (float) -2.0949636E35F;
            CommunicationChannel.instance.send(p84); //===============================
            SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.time_boot_ms = (uint)2931221345U;
            p86.target_system = (byte)(byte)6;
            p86.target_component = (byte)(byte)34;
            p86.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT;
            p86.type_mask = (ushort)(ushort)27032;
            p86.lat_int = (int)291651685;
            p86.lon_int = (int) -1000891334;
            p86.alt = (float)1.3121982E38F;
            p86.vx = (float) -7.095499E37F;
            p86.vy = (float)1.0203067E38F;
            p86.vz = (float) -2.6186982E38F;
            p86.afx = (float) -2.0548793E38F;
            p86.afy = (float) -2.0618747E38F;
            p86.afz = (float) -1.380568E38F;
            p86.yaw = (float) -1.8845433E38F;
            p86.yaw_rate = (float)9.0745896E36F;
            CommunicationChannel.instance.send(p86); //===============================
            POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.time_boot_ms = (uint)1633922415U;
            p87.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p87.type_mask = (ushort)(ushort)51516;
            p87.lat_int = (int) -1805639785;
            p87.lon_int = (int) -1353909575;
            p87.alt = (float) -3.0033665E38F;
            p87.vx = (float)1.5465783E38F;
            p87.vy = (float) -1.224347E38F;
            p87.vz = (float) -1.4918152E38F;
            p87.afx = (float) -2.9036981E38F;
            p87.afy = (float) -3.188853E38F;
            p87.afz = (float)2.3681194E38F;
            p87.yaw = (float) -9.952899E37F;
            p87.yaw_rate = (float) -3.1960374E38F;
            CommunicationChannel.instance.send(p87); //===============================
            LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.time_boot_ms = (uint)1629022263U;
            p89.x = (float) -2.5170981E38F;
            p89.y = (float)2.9775265E38F;
            p89.z = (float)7.619288E36F;
            p89.roll = (float) -2.9973444E37F;
            p89.pitch = (float)2.4678983E38F;
            p89.yaw = (float) -2.5043563E38F;
            CommunicationChannel.instance.send(p89); //===============================
            HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.time_usec = (ulong)8819991660570531062L;
            p90.roll = (float) -1.7452022E38F;
            p90.pitch = (float)2.7211787E38F;
            p90.yaw = (float) -1.4710742E38F;
            p90.rollspeed = (float) -1.7118707E38F;
            p90.pitchspeed = (float)1.9578488E38F;
            p90.yawspeed = (float)3.300672E38F;
            p90.lat = (int)1457712678;
            p90.lon = (int) -582976840;
            p90.alt = (int)815711958;
            p90.vx = (short)(short) -30002;
            p90.vy = (short)(short)12353;
            p90.vz = (short)(short)25544;
            p90.xacc = (short)(short)13089;
            p90.yacc = (short)(short) -24248;
            p90.zacc = (short)(short) -24844;
            CommunicationChannel.instance.send(p90); //===============================
            HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.time_usec = (ulong)2991668013250450241L;
            p91.roll_ailerons = (float) -5.640858E37F;
            p91.pitch_elevator = (float) -9.383946E37F;
            p91.yaw_rudder = (float)2.8418672E38F;
            p91.throttle = (float) -1.9705474E38F;
            p91.aux1 = (float) -3.3419939E38F;
            p91.aux2 = (float)8.893939E37F;
            p91.aux3 = (float)2.833163E38F;
            p91.aux4 = (float) -4.708224E37F;
            p91.mode = MAV_MODE.MAV_MODE_PREFLIGHT;
            p91.nav_mode = (byte)(byte)199;
            CommunicationChannel.instance.send(p91); //===============================
            HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.time_usec = (ulong)2244506609084934378L;
            p92.chan1_raw = (ushort)(ushort)29351;
            p92.chan2_raw = (ushort)(ushort)28118;
            p92.chan3_raw = (ushort)(ushort)55123;
            p92.chan4_raw = (ushort)(ushort)1572;
            p92.chan5_raw = (ushort)(ushort)29232;
            p92.chan6_raw = (ushort)(ushort)32620;
            p92.chan7_raw = (ushort)(ushort)38950;
            p92.chan8_raw = (ushort)(ushort)56999;
            p92.chan9_raw = (ushort)(ushort)1689;
            p92.chan10_raw = (ushort)(ushort)11387;
            p92.chan11_raw = (ushort)(ushort)44495;
            p92.chan12_raw = (ushort)(ushort)23088;
            p92.rssi = (byte)(byte)18;
            CommunicationChannel.instance.send(p92); //===============================
            HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.time_usec = (ulong)7518171045433578653L;
            p93.controls_SET(new float[16], 0);
            p93.mode = MAV_MODE.MAV_MODE_TEST_ARMED;
            p93.flags = (ulong)6849239671840314549L;
            CommunicationChannel.instance.send(p93); //===============================
            OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.time_usec = (ulong)665995575753507853L;
            p100.sensor_id = (byte)(byte)221;
            p100.flow_x = (short)(short) -5248;
            p100.flow_y = (short)(short)30790;
            p100.flow_comp_m_x = (float)4.0561447E37F;
            p100.flow_comp_m_y = (float)1.7331542E38F;
            p100.quality = (byte)(byte)124;
            p100.ground_distance = (float)2.7482207E38F;
            p100.flow_rate_x_SET((float)1.4862422E38F, PH);
            p100.flow_rate_y_SET((float)1.2763257E38F, PH);
            CommunicationChannel.instance.send(p100); //===============================
            GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.usec = (ulong)4482564662781179942L;
            p101.x = (float) -7.5875703E37F;
            p101.y = (float) -2.838468E38F;
            p101.z = (float)3.1895562E38F;
            p101.roll = (float) -2.8216213E38F;
            p101.pitch = (float)3.1490666E38F;
            p101.yaw = (float)1.4535274E38F;
            CommunicationChannel.instance.send(p101); //===============================
            VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.usec = (ulong)6683213475336861466L;
            p102.x = (float) -1.9471095E38F;
            p102.y = (float) -2.2519785E38F;
            p102.z = (float)3.3485692E38F;
            p102.roll = (float) -8.225222E37F;
            p102.pitch = (float)1.7160641E37F;
            p102.yaw = (float)3.3461595E37F;
            CommunicationChannel.instance.send(p102); //===============================
            VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.usec = (ulong)6363813958002554825L;
            p103.x = (float)3.3336572E38F;
            p103.y = (float) -2.2393764E38F;
            p103.z = (float)3.1671632E37F;
            CommunicationChannel.instance.send(p103); //===============================
            VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.usec = (ulong)7795589069156684964L;
            p104.x = (float) -3.3498377E38F;
            p104.y = (float)6.166631E37F;
            p104.z = (float)5.777416E37F;
            p104.roll = (float) -2.033026E38F;
            p104.pitch = (float)7.236667E37F;
            p104.yaw = (float) -1.118569E38F;
            CommunicationChannel.instance.send(p104); //===============================
            HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.time_usec = (ulong)6997263088725208343L;
            p105.xacc = (float)2.3870654E38F;
            p105.yacc = (float) -1.742014E38F;
            p105.zacc = (float) -1.170746E38F;
            p105.xgyro = (float) -2.7869745E38F;
            p105.ygyro = (float) -1.3487401E38F;
            p105.zgyro = (float)2.5849026E38F;
            p105.xmag = (float)3.3866186E38F;
            p105.ymag = (float) -1.4809999E38F;
            p105.zmag = (float)2.0631016E38F;
            p105.abs_pressure = (float) -1.7524556E38F;
            p105.diff_pressure = (float) -2.5048526E38F;
            p105.pressure_alt = (float)3.9570837E37F;
            p105.temperature = (float)1.45238E38F;
            p105.fields_updated = (ushort)(ushort)13306;
            CommunicationChannel.instance.send(p105); //===============================
            OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.time_usec = (ulong)8500479399881583859L;
            p106.sensor_id = (byte)(byte)57;
            p106.integration_time_us = (uint)362735197U;
            p106.integrated_x = (float) -4.6179406E37F;
            p106.integrated_y = (float)1.7850882E37F;
            p106.integrated_xgyro = (float)2.3697844E38F;
            p106.integrated_ygyro = (float)2.331585E36F;
            p106.integrated_zgyro = (float)3.2965686E38F;
            p106.temperature = (short)(short) -27683;
            p106.quality = (byte)(byte)106;
            p106.time_delta_distance_us = (uint)3669576922U;
            p106.distance = (float) -2.570373E38F;
            CommunicationChannel.instance.send(p106); //===============================
            HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.time_usec = (ulong)5194298286524249841L;
            p107.xacc = (float) -2.4157094E38F;
            p107.yacc = (float)2.3882004E38F;
            p107.zacc = (float) -1.1620979E37F;
            p107.xgyro = (float)5.8061607E37F;
            p107.ygyro = (float) -2.0774463E38F;
            p107.zgyro = (float) -3.2587661E37F;
            p107.xmag = (float)1.2357137E38F;
            p107.ymag = (float) -2.2382404E38F;
            p107.zmag = (float) -1.5946449E38F;
            p107.abs_pressure = (float) -2.0397964E37F;
            p107.diff_pressure = (float) -1.1695268E38F;
            p107.pressure_alt = (float) -2.7531231E38F;
            p107.temperature = (float) -3.0820219E38F;
            p107.fields_updated = (uint)1163269329U;
            CommunicationChannel.instance.send(p107); //===============================
            SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.q1 = (float) -2.556271E38F;
            p108.q2 = (float)2.062472E38F;
            p108.q3 = (float)2.3516418E38F;
            p108.q4 = (float)2.6304892E38F;
            p108.roll = (float)2.0055756E38F;
            p108.pitch = (float) -9.780802E36F;
            p108.yaw = (float)1.5657867E36F;
            p108.xacc = (float) -1.8322257E38F;
            p108.yacc = (float) -1.5188786E38F;
            p108.zacc = (float) -7.477331E37F;
            p108.xgyro = (float)3.0298078E38F;
            p108.ygyro = (float) -2.7326284E38F;
            p108.zgyro = (float) -1.2149838E38F;
            p108.lat = (float)5.589103E37F;
            p108.lon = (float) -8.096739E37F;
            p108.alt = (float) -1.1375059E38F;
            p108.std_dev_horz = (float) -2.0437554E38F;
            p108.std_dev_vert = (float)2.7540837E38F;
            p108.vn = (float)2.1508512E38F;
            p108.ve = (float) -2.7899548E38F;
            p108.vd = (float)3.4008918E38F;
            CommunicationChannel.instance.send(p108); //===============================
            RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.rssi = (byte)(byte)224;
            p109.remrssi = (byte)(byte)245;
            p109.txbuf = (byte)(byte)172;
            p109.noise = (byte)(byte)30;
            p109.remnoise = (byte)(byte)124;
            p109.rxerrors = (ushort)(ushort)46568;
            p109.fixed_ = (ushort)(ushort)43990;
            CommunicationChannel.instance.send(p109); //===============================
            FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_network = (byte)(byte)113;
            p110.target_system = (byte)(byte)185;
            p110.target_component = (byte)(byte)145;
            p110.payload_SET(new byte[251], 0);
            CommunicationChannel.instance.send(p110); //===============================
            TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.tc1 = (long) -516784418611442360L;
            p111.ts1 = (long) -9019362642073201978L;
            CommunicationChannel.instance.send(p111); //===============================
            CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.time_usec = (ulong)931850001024351042L;
            p112.seq = (uint)1912935284U;
            CommunicationChannel.instance.send(p112); //===============================
            HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.time_usec = (ulong)1803786207550582946L;
            p113.fix_type = (byte)(byte)110;
            p113.lat = (int)1514203241;
            p113.lon = (int)1804283380;
            p113.alt = (int) -952831777;
            p113.eph = (ushort)(ushort)8764;
            p113.epv = (ushort)(ushort)29034;
            p113.vel = (ushort)(ushort)47056;
            p113.vn = (short)(short) -8265;
            p113.ve = (short)(short) -29065;
            p113.vd = (short)(short)28918;
            p113.cog = (ushort)(ushort)54131;
            p113.satellites_visible = (byte)(byte)233;
            CommunicationChannel.instance.send(p113); //===============================
            HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.time_usec = (ulong)6016130306567350137L;
            p114.sensor_id = (byte)(byte)184;
            p114.integration_time_us = (uint)3410798279U;
            p114.integrated_x = (float) -2.0605543E38F;
            p114.integrated_y = (float)2.0706069E38F;
            p114.integrated_xgyro = (float)1.053303E38F;
            p114.integrated_ygyro = (float) -2.701995E38F;
            p114.integrated_zgyro = (float)1.364763E37F;
            p114.temperature = (short)(short)21035;
            p114.quality = (byte)(byte)213;
            p114.time_delta_distance_us = (uint)1230989707U;
            p114.distance = (float) -5.1623316E37F;
            CommunicationChannel.instance.send(p114); //===============================
            HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.time_usec = (ulong)4869676262057682388L;
            p115.attitude_quaternion_SET(new float[4], 0);
            p115.rollspeed = (float)1.8952455E38F;
            p115.pitchspeed = (float) -1.7420858E38F;
            p115.yawspeed = (float) -2.8919449E38F;
            p115.lat = (int)1265205506;
            p115.lon = (int) -1918022551;
            p115.alt = (int) -2048450322;
            p115.vx = (short)(short) -31930;
            p115.vy = (short)(short)3181;
            p115.vz = (short)(short)13753;
            p115.ind_airspeed = (ushort)(ushort)64303;
            p115.true_airspeed = (ushort)(ushort)34546;
            p115.xacc = (short)(short) -4129;
            p115.yacc = (short)(short) -26063;
            p115.zacc = (short)(short) -787;
            CommunicationChannel.instance.send(p115); //===============================
            SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.time_boot_ms = (uint)1799078884U;
            p116.xacc = (short)(short)7651;
            p116.yacc = (short)(short)19651;
            p116.zacc = (short)(short)19399;
            p116.xgyro = (short)(short) -18909;
            p116.ygyro = (short)(short) -6118;
            p116.zgyro = (short)(short)31622;
            p116.xmag = (short)(short)12510;
            p116.ymag = (short)(short) -1332;
            p116.zmag = (short)(short)28292;
            CommunicationChannel.instance.send(p116); //===============================
            LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.target_system = (byte)(byte)208;
            p117.target_component = (byte)(byte)36;
            p117.start = (ushort)(ushort)38058;
            p117.end = (ushort)(ushort)35441;
            CommunicationChannel.instance.send(p117); //===============================
            LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.id = (ushort)(ushort)50274;
            p118.num_logs = (ushort)(ushort)53671;
            p118.last_log_num = (ushort)(ushort)34244;
            p118.time_utc = (uint)3712085386U;
            p118.size = (uint)398685263U;
            CommunicationChannel.instance.send(p118); //===============================
            LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.target_system = (byte)(byte)48;
            p119.target_component = (byte)(byte)172;
            p119.id = (ushort)(ushort)54023;
            p119.ofs = (uint)4219320329U;
            p119.count = (uint)863154846U;
            CommunicationChannel.instance.send(p119); //===============================
            LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.id = (ushort)(ushort)19971;
            p120.ofs = (uint)2123078893U;
            p120.count = (byte)(byte)17;
            p120.data__SET(new byte[90], 0);
            CommunicationChannel.instance.send(p120); //===============================
            LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_system = (byte)(byte)86;
            p121.target_component = (byte)(byte)4;
            CommunicationChannel.instance.send(p121); //===============================
            LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_system = (byte)(byte)79;
            p122.target_component = (byte)(byte)18;
            CommunicationChannel.instance.send(p122); //===============================
            GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.target_system = (byte)(byte)17;
            p123.target_component = (byte)(byte)204;
            p123.len = (byte)(byte)55;
            p123.data__SET(new byte[110], 0);
            CommunicationChannel.instance.send(p123); //===============================
            GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.time_usec = (ulong)1410159629088829692L;
            p124.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC;
            p124.lat = (int) -1971968833;
            p124.lon = (int) -1649472191;
            p124.alt = (int)124460105;
            p124.eph = (ushort)(ushort)34661;
            p124.epv = (ushort)(ushort)37890;
            p124.vel = (ushort)(ushort)53679;
            p124.cog = (ushort)(ushort)39586;
            p124.satellites_visible = (byte)(byte)109;
            p124.dgps_numch = (byte)(byte)43;
            p124.dgps_age = (uint)1190309632U;
            CommunicationChannel.instance.send(p124); //===============================
            POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vcc = (ushort)(ushort)23051;
            p125.Vservo = (ushort)(ushort)12846;
            p125.flags = (MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED);
            CommunicationChannel.instance.send(p125); //===============================
            SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.device = SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1;
            p126.flags = (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI |
                          SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND |
                          SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING |
                          SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE);
            p126.timeout = (ushort)(ushort)60511;
            p126.baudrate = (uint)61485274U;
            p126.count = (byte)(byte)156;
            p126.data__SET(new byte[70], 0);
            CommunicationChannel.instance.send(p126); //===============================
            GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.time_last_baseline_ms = (uint)1154072719U;
            p127.rtk_receiver_id = (byte)(byte)81;
            p127.wn = (ushort)(ushort)46825;
            p127.tow = (uint)509621073U;
            p127.rtk_health = (byte)(byte)67;
            p127.rtk_rate = (byte)(byte)39;
            p127.nsats = (byte)(byte)72;
            p127.baseline_coords_type = (byte)(byte)215;
            p127.baseline_a_mm = (int) -1157728770;
            p127.baseline_b_mm = (int)160171572;
            p127.baseline_c_mm = (int)1950357017;
            p127.accuracy = (uint)2963315658U;
            p127.iar_num_hypotheses = (int)1346466101;
            CommunicationChannel.instance.send(p127); //===============================
            GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.time_last_baseline_ms = (uint)752883434U;
            p128.rtk_receiver_id = (byte)(byte)163;
            p128.wn = (ushort)(ushort)28053;
            p128.tow = (uint)2068605335U;
            p128.rtk_health = (byte)(byte)179;
            p128.rtk_rate = (byte)(byte)174;
            p128.nsats = (byte)(byte)93;
            p128.baseline_coords_type = (byte)(byte)99;
            p128.baseline_a_mm = (int)948785961;
            p128.baseline_b_mm = (int)2032135772;
            p128.baseline_c_mm = (int) -674096335;
            p128.accuracy = (uint)1954499095U;
            p128.iar_num_hypotheses = (int) -1376033573;
            CommunicationChannel.instance.send(p128); //===============================
            SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.time_boot_ms = (uint)2220492890U;
            p129.xacc = (short)(short) -7206;
            p129.yacc = (short)(short) -28296;
            p129.zacc = (short)(short) -18614;
            p129.xgyro = (short)(short)3670;
            p129.ygyro = (short)(short)25702;
            p129.zgyro = (short)(short) -18554;
            p129.xmag = (short)(short) -4186;
            p129.ymag = (short)(short) -18569;
            p129.zmag = (short)(short)25401;
            CommunicationChannel.instance.send(p129); //===============================
            DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.type = (byte)(byte)224;
            p130.size = (uint)4104672642U;
            p130.width = (ushort)(ushort)20444;
            p130.height = (ushort)(ushort)35349;
            p130.packets = (ushort)(ushort)21679;
            p130.payload = (byte)(byte)29;
            p130.jpg_quality = (byte)(byte)173;
            CommunicationChannel.instance.send(p130); //===============================
            ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.seqnr = (ushort)(ushort)1605;
            p131.data__SET(new byte[253], 0);
            CommunicationChannel.instance.send(p131); //===============================
            DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.time_boot_ms = (uint)1527010430U;
            p132.min_distance = (ushort)(ushort)40633;
            p132.max_distance = (ushort)(ushort)52241;
            p132.current_distance = (ushort)(ushort)48016;
            p132.type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR;
            p132.id = (byte)(byte)241;
            p132.orientation = MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_45;
            p132.covariance = (byte)(byte)228;
            CommunicationChannel.instance.send(p132); //===============================
            TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.lat = (int) -1429889531;
            p133.lon = (int) -2058509306;
            p133.grid_spacing = (ushort)(ushort)21108;
            p133.mask = (ulong)6765112993964508285L;
            CommunicationChannel.instance.send(p133); //===============================
            TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.lat = (int)1163102856;
            p134.lon = (int)982344951;
            p134.grid_spacing = (ushort)(ushort)14666;
            p134.gridbit = (byte)(byte)74;
            p134.data__SET(new short[16], 0);
            CommunicationChannel.instance.send(p134); //===============================
            TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lat = (int)228246619;
            p135.lon = (int) -126631990;
            CommunicationChannel.instance.send(p135); //===============================
            TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.lat = (int) -1216890723;
            p136.lon = (int)665231758;
            p136.spacing = (ushort)(ushort)5036;
            p136.terrain_height = (float)2.8705508E38F;
            p136.current_height = (float) -2.188501E38F;
            p136.pending = (ushort)(ushort)37043;
            p136.loaded = (ushort)(ushort)17168;
            CommunicationChannel.instance.send(p136); //===============================
            SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.time_boot_ms = (uint)1129061303U;
            p137.press_abs = (float)3.0360538E38F;
            p137.press_diff = (float) -1.820867E38F;
            p137.temperature = (short)(short) -3577;
            CommunicationChannel.instance.send(p137); //===============================
            ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.time_usec = (ulong)4507430934661758388L;
            p138.q_SET(new float[4], 0);
            p138.x = (float) -3.0591952E38F;
            p138.y = (float) -2.261244E38F;
            p138.z = (float)3.3187742E38F;
            CommunicationChannel.instance.send(p138); //===============================
            SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.time_usec = (ulong)8477806023657454830L;
            p139.group_mlx = (byte)(byte)67;
            p139.target_system = (byte)(byte)22;
            p139.target_component = (byte)(byte)74;
            p139.controls_SET(new float[8], 0);
            CommunicationChannel.instance.send(p139); //===============================
            ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.time_usec = (ulong)3384696974410129933L;
            p140.group_mlx = (byte)(byte)217;
            p140.controls_SET(new float[8], 0);
            CommunicationChannel.instance.send(p140); //===============================
            ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.time_usec = (ulong)8566862283506741651L;
            p141.altitude_monotonic = (float) -2.3039024E38F;
            p141.altitude_amsl = (float)3.287199E38F;
            p141.altitude_local = (float)2.553924E38F;
            p141.altitude_relative = (float) -8.890361E36F;
            p141.altitude_terrain = (float)2.3320234E38F;
            p141.bottom_clearance = (float)2.8853158E38F;
            CommunicationChannel.instance.send(p141); //===============================
            RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.request_id = (byte)(byte)32;
            p142.uri_type = (byte)(byte)229;
            p142.uri_SET(new byte[120], 0);
            p142.transfer_type = (byte)(byte)0;
            p142.storage_SET(new byte[120], 0);
            CommunicationChannel.instance.send(p142); //===============================
            SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.time_boot_ms = (uint)1207019357U;
            p143.press_abs = (float) -1.6303719E38F;
            p143.press_diff = (float) -1.7244386E37F;
            p143.temperature = (short)(short) -12203;
            CommunicationChannel.instance.send(p143); //===============================
            FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.timestamp = (ulong)348258730458270524L;
            p144.est_capabilities = (byte)(byte)196;
            p144.lat = (int) -2062444798;
            p144.lon = (int)1632316443;
            p144.alt = (float) -1.5754371E38F;
            p144.vel_SET(new float[3], 0);
            p144.acc_SET(new float[3], 0);
            p144.attitude_q_SET(new float[4], 0);
            p144.rates_SET(new float[3], 0);
            p144.position_cov_SET(new float[3], 0);
            p144.custom_state = (ulong)8743845655835789882L;
            CommunicationChannel.instance.send(p144); //===============================
            CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.time_usec = (ulong)7959161855921771879L;
            p146.x_acc = (float)2.8763213E38F;
            p146.y_acc = (float) -2.1149545E38F;
            p146.z_acc = (float)9.8941055E36F;
            p146.x_vel = (float) -1.7815904E38F;
            p146.y_vel = (float)7.4036246E37F;
            p146.z_vel = (float) -2.4279154E38F;
            p146.x_pos = (float)2.213368E38F;
            p146.y_pos = (float)8.465358E37F;
            p146.z_pos = (float) -2.6680443E38F;
            p146.airspeed = (float) -3.1101164E37F;
            p146.vel_variance_SET(new float[3], 0);
            p146.pos_variance_SET(new float[3], 0);
            p146.q_SET(new float[4], 0);
            p146.roll_rate = (float)7.888371E36F;
            p146.pitch_rate = (float) -8.84077E37F;
            p146.yaw_rate = (float) -4.5586455E37F;
            CommunicationChannel.instance.send(p146); //===============================
            BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.id = (byte)(byte)14;
            p147.battery_function = MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_AVIONICS;
            p147.type = MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_NIMH;
            p147.temperature = (short)(short)20184;
            p147.voltages_SET(new ushort[10], 0);
            p147.current_battery = (short)(short) -15057;
            p147.current_consumed = (int) -1625423092;
            p147.energy_consumed = (int) -1721711699;
            p147.battery_remaining = (sbyte)(sbyte)64;
            CommunicationChannel.instance.send(p147); //===============================
            AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMMAND_INT);
            p148.flight_sw_version = (uint)4250067105U;
            p148.middleware_sw_version = (uint)316548101U;
            p148.os_sw_version = (uint)3037294178U;
            p148.board_version = (uint)2083648878U;
            p148.flight_custom_version_SET(new byte[8], 0);
            p148.middleware_custom_version_SET(new byte[8], 0);
            p148.os_custom_version_SET(new byte[8], 0);
            p148.vendor_id = (ushort)(ushort)39033;
            p148.product_id = (ushort)(ushort)23037;
            p148.uid = (ulong)2182222733627330710L;
            p148.uid2_SET(new byte[18], 0, PH);
            CommunicationChannel.instance.send(p148); //===============================
            LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.time_usec = (ulong)161633676575437351L;
            p149.target_num = (byte)(byte)122;
            p149.frame = MAV_FRAME.MAV_FRAME_BODY_NED;
            p149.angle_x = (float)1.2162537E38F;
            p149.angle_y = (float) -1.9516613E38F;
            p149.distance = (float)2.6891202E38F;
            p149.size_x = (float) -2.3329018E38F;
            p149.size_y = (float)6.3777986E36F;
            p149.x_SET((float)9.000752E37F, PH);
            p149.y_SET((float)3.3957688E38F, PH);
            p149.z_SET((float)9.337768E37F, PH);
            p149.q_SET(new float[4], 0, PH);
            p149.type = LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_FIDUCIAL;
            p149.position_valid_SET((byte)(byte)229, PH);
            CommunicationChannel.instance.send(p149); //===============================
            SENS_POWER p201 = CommunicationChannel.new_SENS_POWER();
            PH.setPack(p201);
            p201.adc121_vspb_volt = (float)3.3786558E38F;
            p201.adc121_cspb_amp = (float) -2.8677692E38F;
            p201.adc121_cs1_amp = (float)1.5139204E38F;
            p201.adc121_cs2_amp = (float) -2.4497143E38F;
            CommunicationChannel.instance.send(p201); //===============================
            SENS_MPPT p202 = CommunicationChannel.new_SENS_MPPT();
            PH.setPack(p202);
            p202.mppt_timestamp = (ulong)3976747030460532308L;
            p202.mppt1_volt = (float) -2.0883706E38F;
            p202.mppt1_amp = (float)2.510019E37F;
            p202.mppt1_pwm = (ushort)(ushort)20638;
            p202.mppt1_status = (byte)(byte)122;
            p202.mppt2_volt = (float) -1.9390282E38F;
            p202.mppt2_amp = (float)1.4191924E37F;
            p202.mppt2_pwm = (ushort)(ushort)44735;
            p202.mppt2_status = (byte)(byte)31;
            p202.mppt3_volt = (float) -1.8881122E38F;
            p202.mppt3_amp = (float)1.6053371E38F;
            p202.mppt3_pwm = (ushort)(ushort)39014;
            p202.mppt3_status = (byte)(byte)239;
            CommunicationChannel.instance.send(p202); //===============================
            ASLCTRL_DATA p203 = CommunicationChannel.new_ASLCTRL_DATA();
            PH.setPack(p203);
            p203.timestamp = (ulong)3922376483823724077L;
            p203.aslctrl_mode = (byte)(byte)80;
            p203.h = (float) -3.0939023E38F;
            p203.hRef = (float)2.5687159E38F;
            p203.hRef_t = (float)2.36144E38F;
            p203.PitchAngle = (float)2.0960828E38F;
            p203.PitchAngleRef = (float)1.9762261E38F;
            p203.q = (float) -6.124594E37F;
            p203.qRef = (float)7.8810776E37F;
            p203.uElev = (float)2.6628741E38F;
            p203.uThrot = (float)7.208432E37F;
            p203.uThrot2 = (float)1.4788568E37F;
            p203.nZ = (float) -7.0666395E37F;
            p203.AirspeedRef = (float)6.96757E37F;
            p203.SpoilersEngaged = (byte)(byte)14;
            p203.YawAngle = (float)1.4208186E38F;
            p203.YawAngleRef = (float) -6.5668925E37F;
            p203.RollAngle = (float) -3.3298011E38F;
            p203.RollAngleRef = (float)4.7398526E37F;
            p203.p = (float) -2.8678412E38F;
            p203.pRef = (float) -4.2280795E37F;
            p203.r = (float) -2.519228E38F;
            p203.rRef = (float) -1.5313346E38F;
            p203.uAil = (float)1.7263215E38F;
            p203.uRud = (float) -2.9042348E38F;
            CommunicationChannel.instance.send(p203); //===============================
            ASLCTRL_DEBUG p204 = CommunicationChannel.new_ASLCTRL_DEBUG();
            PH.setPack(p204);
            p204.i32_1 = (uint)3241952107U;
            p204.i8_1 = (byte)(byte)39;
            p204.i8_2 = (byte)(byte)54;
            p204.f_1 = (float)3.328293E38F;
            p204.f_2 = (float) -1.2218239E38F;
            p204.f_3 = (float)8.1384877E37F;
            p204.f_4 = (float) -1.0715548E38F;
            p204.f_5 = (float)1.5603831E38F;
            p204.f_6 = (float)1.4132101E37F;
            p204.f_7 = (float)1.4910856E38F;
            p204.f_8 = (float)2.1924104E38F;
            CommunicationChannel.instance.send(p204); //===============================
            ASLUAV_STATUS p205 = CommunicationChannel.new_ASLUAV_STATUS();
            PH.setPack(p205);
            p205.LED_status = (byte)(byte)115;
            p205.SATCOM_status = (byte)(byte)217;
            p205.Servo_status_SET(new byte[8], 0);
            p205.Motor_rpm = (float) -3.8927152E37F;
            CommunicationChannel.instance.send(p205); //===============================
            EKF_EXT p206 = CommunicationChannel.new_EKF_EXT();
            PH.setPack(p206);
            p206.timestamp = (ulong)6684801578406596526L;
            p206.Windspeed = (float)5.6030364E37F;
            p206.WindDir = (float) -1.1699152E38F;
            p206.WindZ = (float)1.5410435E38F;
            p206.Airspeed = (float)3.373636E38F;
            p206.beta = (float) -6.0633787E37F;
            p206.alpha = (float) -1.5750322E38F;
            CommunicationChannel.instance.send(p206); //===============================
            ASL_OBCTRL p207 = CommunicationChannel.new_ASL_OBCTRL();
            PH.setPack(p207);
            p207.timestamp = (ulong)221491005115810685L;
            p207.uElev = (float) -9.320381E37F;
            p207.uThrot = (float)2.1954208E36F;
            p207.uThrot2 = (float) -1.1898727E38F;
            p207.uAilL = (float) -2.1296948E38F;
            p207.uAilR = (float)1.9947005E38F;
            p207.uRud = (float)1.961636E38F;
            p207.obctrl_status = (byte)(byte)56;
            CommunicationChannel.instance.send(p207); //===============================
            SENS_ATMOS p208 = CommunicationChannel.new_SENS_ATMOS();
            PH.setPack(p208);
            p208.TempAmbient = (float)2.94958E38F;
            p208.Humidity = (float)1.5671066E37F;
            CommunicationChannel.instance.send(p208); //===============================
            SENS_BATMON p209 = CommunicationChannel.new_SENS_BATMON();
            PH.setPack(p209);
            p209.temperature = (float) -1.3536298E38F;
            p209.voltage = (ushort)(ushort)6798;
            p209.current = (short)(short)15773;
            p209.SoC = (byte)(byte)140;
            p209.batterystatus = (ushort)(ushort)45347;
            p209.serialnumber = (ushort)(ushort)15653;
            p209.hostfetcontrol = (ushort)(ushort)19545;
            p209.cellvoltage1 = (ushort)(ushort)51315;
            p209.cellvoltage2 = (ushort)(ushort)46492;
            p209.cellvoltage3 = (ushort)(ushort)30542;
            p209.cellvoltage4 = (ushort)(ushort)42239;
            p209.cellvoltage5 = (ushort)(ushort)63212;
            p209.cellvoltage6 = (ushort)(ushort)30835;
            CommunicationChannel.instance.send(p209); //===============================
            FW_SOARING_DATA p210 = CommunicationChannel.new_FW_SOARING_DATA();
            PH.setPack(p210);
            p210.timestamp = (ulong)292924372159661465L;
            p210.timestampModeChanged = (ulong)7270822584944391980L;
            p210.xW = (float)9.049611E37F;
            p210.xR = (float)8.9752E37F;
            p210.xLat = (float)3.0106491E37F;
            p210.xLon = (float)1.7324468E38F;
            p210.VarW = (float) -3.3868148E38F;
            p210.VarR = (float) -4.6679504E37F;
            p210.VarLat = (float) -4.705704E37F;
            p210.VarLon = (float) -1.6673312E38F;
            p210.LoiterRadius = (float)2.2033033E38F;
            p210.LoiterDirection = (float)1.01268246E37F;
            p210.DistToSoarPoint = (float) -4.141908E37F;
            p210.vSinkExp = (float) -2.764505E38F;
            p210.z1_LocalUpdraftSpeed = (float) -1.4184105E38F;
            p210.z2_DeltaRoll = (float)1.5534514E38F;
            p210.z1_exp = (float) -1.206876E38F;
            p210.z2_exp = (float) -2.8266495E38F;
            p210.ThermalGSNorth = (float) -8.090595E37F;
            p210.ThermalGSEast = (float) -3.3016469E38F;
            p210.TSE_dot = (float) -5.2375256E37F;
            p210.DebugVar1 = (float)1.723226E37F;
            p210.DebugVar2 = (float) -9.66828E37F;
            p210.ControlMode = (byte)(byte)196;
            p210.valid = (byte)(byte)220;
            CommunicationChannel.instance.send(p210); //===============================
            SENSORPOD_STATUS p211 = CommunicationChannel.new_SENSORPOD_STATUS();
            PH.setPack(p211);
            p211.timestamp = (ulong)8537974150640339764L;
            p211.visensor_rate_1 = (byte)(byte)76;
            p211.visensor_rate_2 = (byte)(byte)183;
            p211.visensor_rate_3 = (byte)(byte)170;
            p211.visensor_rate_4 = (byte)(byte)119;
            p211.recording_nodes_count = (byte)(byte)105;
            p211.cpu_temp = (byte)(byte)253;
            p211.free_space = (ushort)(ushort)60143;
            CommunicationChannel.instance.send(p211); //===============================
            SENS_POWER_BOARD p212 = CommunicationChannel.new_SENS_POWER_BOARD();
            PH.setPack(p212);
            p212.timestamp = (ulong)3579460266453502229L;
            p212.pwr_brd_status = (byte)(byte)17;
            p212.pwr_brd_led_status = (byte)(byte)183;
            p212.pwr_brd_system_volt = (float)1.276922E38F;
            p212.pwr_brd_servo_volt = (float)7.6047085E37F;
            p212.pwr_brd_mot_l_amp = (float)2.2240648E38F;
            p212.pwr_brd_mot_r_amp = (float)2.1388576E38F;
            p212.pwr_brd_servo_1_amp = (float) -1.3348748E38F;
            p212.pwr_brd_servo_2_amp = (float) -2.9123017E38F;
            p212.pwr_brd_servo_3_amp = (float) -1.0990849E38F;
            p212.pwr_brd_servo_4_amp = (float)3.158285E38F;
            p212.pwr_brd_aux_amp = (float) -1.9258531E38F;
            CommunicationChannel.instance.send(p212); //===============================
            ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.time_usec = (ulong)2481516043589607258L;
            p230.flags = (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH);
            p230.vel_ratio = (float) -2.363512E38F;
            p230.pos_horiz_ratio = (float)1.7289207E38F;
            p230.pos_vert_ratio = (float) -7.84812E37F;
            p230.mag_ratio = (float) -5.552685E37F;
            p230.hagl_ratio = (float)1.5398606E38F;
            p230.tas_ratio = (float)1.8725822E38F;
            p230.pos_horiz_accuracy = (float) -2.837261E37F;
            p230.pos_vert_accuracy = (float)1.0514612E38F;
            CommunicationChannel.instance.send(p230); //===============================
            WIND_COV p231 = CommunicationChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.time_usec = (ulong)7648131884104778520L;
            p231.wind_x = (float)1.2517359E38F;
            p231.wind_y = (float)9.019707E37F;
            p231.wind_z = (float)1.990225E38F;
            p231.var_horiz = (float)1.8474401E38F;
            p231.var_vert = (float) -2.9423408E38F;
            p231.wind_alt = (float)1.6230109E38F;
            p231.horiz_accuracy = (float)6.571454E37F;
            p231.vert_accuracy = (float) -2.5163631E38F;
            CommunicationChannel.instance.send(p231); //===============================
            GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.time_usec = (ulong)3211304702219375881L;
            p232.gps_id = (byte)(byte)14;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY);
            p232.time_week_ms = (uint)75818126U;
            p232.time_week = (ushort)(ushort)50202;
            p232.fix_type = (byte)(byte)86;
            p232.lat = (int) -494081188;
            p232.lon = (int) -525253486;
            p232.alt = (float)3.59837E37F;
            p232.hdop = (float)4.448196E37F;
            p232.vdop = (float) -1.3065152E38F;
            p232.vn = (float)6.252365E37F;
            p232.ve = (float)2.4487505E38F;
            p232.vd = (float)2.9384307E38F;
            p232.speed_accuracy = (float) -4.6637458E36F;
            p232.horiz_accuracy = (float) -3.3422902E38F;
            p232.vert_accuracy = (float) -1.861771E38F;
            p232.satellites_visible = (byte)(byte)55;
            CommunicationChannel.instance.send(p232); //===============================
            GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.flags = (byte)(byte)58;
            p233.len = (byte)(byte)170;
            p233.data__SET(new byte[180], 0);
            CommunicationChannel.instance.send(p233); //===============================
            HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED);
            p234.custom_mode = (uint)379953097U;
            p234.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND;
            p234.roll = (short)(short) -18809;
            p234.pitch = (short)(short) -29598;
            p234.heading = (ushort)(ushort)3420;
            p234.throttle = (sbyte)(sbyte)125;
            p234.heading_sp = (short)(short) -4184;
            p234.latitude = (int) -2118928321;
            p234.longitude = (int) -1026979930;
            p234.altitude_amsl = (short)(short)29469;
            p234.altitude_sp = (short)(short) -32199;
            p234.airspeed = (byte)(byte)85;
            p234.airspeed_sp = (byte)(byte)31;
            p234.groundspeed = (byte)(byte)75;
            p234.climb_rate = (sbyte)(sbyte) - 80;
            p234.gps_nsat = (byte)(byte)84;
            p234.gps_fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX;
            p234.battery_remaining = (byte)(byte)110;
            p234.temperature = (sbyte)(sbyte) - 121;
            p234.temperature_air = (sbyte)(sbyte) - 92;
            p234.failsafe = (byte)(byte)142;
            p234.wp_num = (byte)(byte)10;
            p234.wp_distance = (ushort)(ushort)64849;
            CommunicationChannel.instance.send(p234); //===============================
            VIBRATION p241 = CommunicationChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.time_usec = (ulong)8560824563385056655L;
            p241.vibration_x = (float)2.6147413E38F;
            p241.vibration_y = (float)1.3911568E38F;
            p241.vibration_z = (float)1.6147199E38F;
            p241.clipping_0 = (uint)2702389691U;
            p241.clipping_1 = (uint)2860959544U;
            p241.clipping_2 = (uint)2782478756U;
            CommunicationChannel.instance.send(p241); //===============================
            HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.latitude = (int)2011038539;
            p242.longitude = (int) -1891483937;
            p242.altitude = (int)1924862537;
            p242.x = (float) -3.0346482E38F;
            p242.y = (float)6.375818E36F;
            p242.z = (float)8.707223E37F;
            p242.q_SET(new float[4], 0);
            p242.approach_x = (float) -2.8483294E38F;
            p242.approach_y = (float) -3.0226323E38F;
            p242.approach_z = (float)3.0074384E38F;
            p242.time_usec_SET((ulong)2708855278440056242L, PH);
            CommunicationChannel.instance.send(p242); //===============================
            SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.target_system = (byte)(byte)123;
            p243.latitude = (int) -485659534;
            p243.longitude = (int) -1513212933;
            p243.altitude = (int) -947834307;
            p243.x = (float) -2.6954126E38F;
            p243.y = (float)2.7608465E38F;
            p243.z = (float)1.6795825E38F;
            p243.q_SET(new float[4], 0);
            p243.approach_x = (float) -8.818017E37F;
            p243.approach_y = (float)2.0729758E38F;
            p243.approach_z = (float) -2.0284695E38F;
            p243.time_usec_SET((ulong)5509282351210470559L, PH);
            CommunicationChannel.instance.send(p243); //===============================
            MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.message_id = (ushort)(ushort)56355;
            p244.interval_us = (int) -1134957812;
            CommunicationChannel.instance.send(p244); //===============================
            EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.vtol_state = MAV_VTOL_STATE.MAV_VTOL_STATE_MC;
            p245.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF;
            CommunicationChannel.instance.send(p245); //===============================
            ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.ICAO_address = (uint)3569132172U;
            p246.lat = (int) -1673838166;
            p246.lon = (int)1864490676;
            p246.altitude_type = ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
            p246.altitude = (int)1252488935;
            p246.heading = (ushort)(ushort)60004;
            p246.hor_velocity = (ushort)(ushort)63646;
            p246.ver_velocity = (short)(short) -10493;
            p246.callsign_SET("DEMO", PH);
            p246.emitter_type = ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_HEAVY;
            p246.tslc = (byte)(byte)198;
            p246.flags = (ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING);
            p246.squawk = (ushort)(ushort)21444;
            CommunicationChannel.instance.send(p246); //===============================
            COLLISION p247 = CommunicationChannel.new_COLLISION();
            PH.setPack(p247);
            p247.src_ = MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB;
            p247.id = (uint)3973875552U;
            p247.action = MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_ASCEND_OR_DESCEND;
            p247.threat_level = MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE;
            p247.time_to_minimum_delta = (float) -2.2625127E38F;
            p247.altitude_minimum_delta = (float) -3.0828311E38F;
            p247.horizontal_minimum_delta = (float) -7.317366E36F;
            CommunicationChannel.instance.send(p247); //===============================
            V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.target_network = (byte)(byte)145;
            p248.target_system = (byte)(byte)238;
            p248.target_component = (byte)(byte)127;
            p248.message_type = (ushort)(ushort)11855;
            p248.payload_SET(new byte[249], 0);
            CommunicationChannel.instance.send(p248); //===============================
            MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.address = (ushort)(ushort)31722;
            p249.ver = (byte)(byte)230;
            p249.type = (byte)(byte)243;
            p249.value_SET(new sbyte[32], 0);
            CommunicationChannel.instance.send(p249); //===============================
            DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.name_SET("DEMO", PH);
            p250.time_usec = (ulong)1583689293334172326L;
            p250.x = (float)3.1214464E38F;
            p250.y = (float) -2.3484818E38F;
            p250.z = (float) -1.1763779E38F;
            CommunicationChannel.instance.send(p250); //===============================
            NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.time_boot_ms = (uint)85057610U;
            p251.name_SET("DEMO", PH);
            p251.value = (float) -3.0770679E38F;
            CommunicationChannel.instance.send(p251); //===============================
            NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.time_boot_ms = (uint)1724586084U;
            p252.name_SET("DEMO", PH);
            p252.value = (int)973058330;
            CommunicationChannel.instance.send(p252); //===============================
            STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.severity = MAV_SEVERITY.MAV_SEVERITY_INFO;
            p253.text_SET("DEMO", PH);
            CommunicationChannel.instance.send(p253); //===============================
            DEBUG p254 = CommunicationChannel.new_DEBUG();
            PH.setPack(p254);
            p254.time_boot_ms = (uint)1953973016U;
            p254.ind = (byte)(byte)27;
            p254.value = (float) -1.5940738E37F;
            CommunicationChannel.instance.send(p254); //===============================
            SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.target_system = (byte)(byte)123;
            p256.target_component = (byte)(byte)18;
            p256.secret_key_SET(new byte[32], 0);
            p256.initial_timestamp = (ulong)2871169922278603779L;
            CommunicationChannel.instance.send(p256); //===============================
            BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.time_boot_ms = (uint)612356945U;
            p257.last_change_ms = (uint)4095691179U;
            p257.state = (byte)(byte)168;
            CommunicationChannel.instance.send(p257); //===============================
            PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_system = (byte)(byte)174;
            p258.target_component = (byte)(byte)149;
            p258.tune_SET("DEMO", PH);
            CommunicationChannel.instance.send(p258); //===============================
            CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.time_boot_ms = (uint)2265991093U;
            p259.vendor_name_SET(new byte[32], 0);
            p259.model_name_SET(new byte[32], 0);
            p259.firmware_version = (uint)1825708046U;
            p259.focal_length = (float) -1.1805777E38F;
            p259.sensor_size_h = (float)8.16168E37F;
            p259.sensor_size_v = (float) -3.6093382E37F;
            p259.resolution_h = (ushort)(ushort)22557;
            p259.resolution_v = (ushort)(ushort)55173;
            p259.lens_id = (byte)(byte)221;
            p259.flags = (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE);
            p259.cam_definition_version = (ushort)(ushort)57041;
            p259.cam_definition_uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p259); //===============================
            CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.time_boot_ms = (uint)529103950U;
            p260.mode_id = CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY;
            CommunicationChannel.instance.send(p260); //===============================
            STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.time_boot_ms = (uint)2725204186U;
            p261.storage_id = (byte)(byte)234;
            p261.storage_count = (byte)(byte)50;
            p261.status = (byte)(byte)172;
            p261.total_capacity = (float) -7.7806036E37F;
            p261.used_capacity = (float)2.5037894E38F;
            p261.available_capacity = (float)7.782173E37F;
            p261.read_speed = (float)3.7007784E37F;
            p261.write_speed = (float)2.112295E38F;
            CommunicationChannel.instance.send(p261); //===============================
            CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.time_boot_ms = (uint)1404811741U;
            p262.image_status = (byte)(byte)251;
            p262.video_status = (byte)(byte)231;
            p262.image_interval = (float) -3.6644726E37F;
            p262.recording_time_ms = (uint)709924568U;
            p262.available_capacity = (float) -2.1927554E38F;
            CommunicationChannel.instance.send(p262); //===============================
            CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.time_boot_ms = (uint)3377145894U;
            p263.time_utc = (ulong)7814433384770066255L;
            p263.camera_id = (byte)(byte)125;
            p263.lat = (int) -1700194077;
            p263.lon = (int) -725594227;
            p263.alt = (int)1512337764;
            p263.relative_alt = (int)1422589603;
            p263.q_SET(new float[4], 0);
            p263.image_index = (int)1665342099;
            p263.capture_result = (sbyte)(sbyte)39;
            p263.file_url_SET("DEMO", PH);
            CommunicationChannel.instance.send(p263); //===============================
            FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.time_boot_ms = (uint)740412487U;
            p264.arming_time_utc = (ulong)5410687415971181803L;
            p264.takeoff_time_utc = (ulong)7981525085763990897L;
            p264.flight_uuid = (ulong)4330133932268283057L;
            CommunicationChannel.instance.send(p264); //===============================
            MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.time_boot_ms = (uint)3397146189U;
            p265.roll = (float)2.3968415E38F;
            p265.pitch = (float) -1.6268519E38F;
            p265.yaw = (float)8.1293733E37F;
            CommunicationChannel.instance.send(p265); //===============================
            LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.target_system = (byte)(byte)53;
            p266.target_component = (byte)(byte)253;
            p266.sequence = (ushort)(ushort)12801;
            p266.length = (byte)(byte)47;
            p266.first_message_offset = (byte)(byte)15;
            p266.data__SET(new byte[249], 0);
            CommunicationChannel.instance.send(p266); //===============================
            LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.target_system = (byte)(byte)73;
            p267.target_component = (byte)(byte)97;
            p267.sequence = (ushort)(ushort)55627;
            p267.length = (byte)(byte)55;
            p267.first_message_offset = (byte)(byte)131;
            p267.data__SET(new byte[249], 0);
            CommunicationChannel.instance.send(p267); //===============================
            LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_system = (byte)(byte)202;
            p268.target_component = (byte)(byte)49;
            p268.sequence = (ushort)(ushort)28244;
            CommunicationChannel.instance.send(p268); //===============================
            VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.camera_id = (byte)(byte)152;
            p269.status = (byte)(byte)172;
            p269.framerate = (float) -3.2502608E38F;
            p269.resolution_h = (ushort)(ushort)40593;
            p269.resolution_v = (ushort)(ushort)55232;
            p269.bitrate = (uint)1869746949U;
            p269.rotation = (ushort)(ushort)5114;
            p269.uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p269); //===============================
            SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.target_system = (byte)(byte)121;
            p270.target_component = (byte)(byte)47;
            p270.camera_id = (byte)(byte)207;
            p270.framerate = (float)3.2082044E37F;
            p270.resolution_h = (ushort)(ushort)45151;
            p270.resolution_v = (ushort)(ushort)2143;
            p270.bitrate = (uint)1021490373U;
            p270.rotation = (ushort)(ushort)30745;
            p270.uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p270); //===============================
            WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.ssid_SET("DEMO", PH);
            p299.password_SET("DEMO", PH);
            CommunicationChannel.instance.send(p299); //===============================
            PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.version = (ushort)(ushort)48217;
            p300.min_version = (ushort)(ushort)6329;
            p300.max_version = (ushort)(ushort)50238;
            p300.spec_version_hash_SET(new byte[8], 0);
            p300.library_version_hash_SET(new byte[8], 0);
            CommunicationChannel.instance.send(p300); //===============================
            UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.time_usec = (ulong)6721452736605781544L;
            p310.uptime_sec = (uint)1425595826U;
            p310.health = UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL;
            p310.mode = UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OPERATIONAL;
            p310.sub_mode = (byte)(byte)191;
            p310.vendor_specific_status_code = (ushort)(ushort)10093;
            CommunicationChannel.instance.send(p310); //===============================
            UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.time_usec = (ulong)2915106295426198897L;
            p311.uptime_sec = (uint)2598213896U;
            p311.name_SET("DEMO", PH);
            p311.hw_version_major = (byte)(byte)223;
            p311.hw_version_minor = (byte)(byte)101;
            p311.hw_unique_id_SET(new byte[16], 0);
            p311.sw_version_major = (byte)(byte)36;
            p311.sw_version_minor = (byte)(byte)194;
            p311.sw_vcs_commit = (uint)3018212827U;
            CommunicationChannel.instance.send(p311); //===============================
            PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.target_system = (byte)(byte)59;
            p320.target_component = (byte)(byte)255;
            p320.param_id_SET("DEMO", PH);
            p320.param_index = (short)(short) -9421;
            CommunicationChannel.instance.send(p320); //===============================
            PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_system = (byte)(byte)227;
            p321.target_component = (byte)(byte)128;
            CommunicationChannel.instance.send(p321); //===============================
            PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_id_SET("DEMO", PH);
            p322.param_value_SET("DEMO", PH);
            p322.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8;
            p322.param_count = (ushort)(ushort)41573;
            p322.param_index = (ushort)(ushort)21224;
            CommunicationChannel.instance.send(p322); //===============================
            PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.target_system = (byte)(byte)188;
            p323.target_component = (byte)(byte)53;
            p323.param_id_SET("DEMO", PH);
            p323.param_value_SET("DEMO", PH);
            p323.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64;
            CommunicationChannel.instance.send(p323); //===============================
            PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_id_SET("DEMO", PH);
            p324.param_value_SET("DEMO", PH);
            p324.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64;
            p324.param_result = PARAM_ACK.PARAM_ACK_ACCEPTED;
            CommunicationChannel.instance.send(p324); //===============================
            OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.time_usec = (ulong)2316099065274726575L;
            p330.sensor_type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR;
            p330.distances_SET(new ushort[72], 0);
            p330.increment = (byte)(byte)183;
            p330.min_distance = (ushort)(ushort)7850;
            p330.max_distance = (ushort)(ushort)6007;
            CommunicationChannel.instance.send(p330); //===============================
        }
    }
}

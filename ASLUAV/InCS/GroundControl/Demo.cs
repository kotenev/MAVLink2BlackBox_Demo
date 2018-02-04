
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
            POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.time_boot_ms = (uint)1561462507U;
            p3.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p3.type_mask = (ushort)(ushort)44659;
            p3.x = (float)2.7459857E38F;
            p3.y = (float)3.1486423E38F;
            p3.z = (float) -1.2339048E38F;
            p3.vx = (float)9.474242E37F;
            p3.vy = (float)1.1692934E38F;
            p3.vz = (float)2.6997674E38F;
            p3.afx = (float)2.2955067E38F;
            p3.afy = (float) -2.7630694E38F;
            p3.afz = (float)4.416111E37F;
            p3.yaw = (float) -1.1675625E38F;
            p3.yaw_rate = (float) -1.2309309E38F;
            CommunicationChannel.instance.send(p3); //===============================
            MANUAL_SETPOINT p81 = CommunicationChannel.new_MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.time_boot_ms = (uint)555329684U;
            p81.roll = (float)2.6260982E38F;
            p81.pitch = (float) -1.6696911E38F;
            p81.yaw = (float) -1.7614379E37F;
            p81.thrust = (float) -1.0784042E38F;
            p81.mode_switch = (byte)(byte)85;
            p81.manual_override_switch = (byte)(byte)91;
            CommunicationChannel.instance.send(p81); //===============================
            SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.time_boot_ms = (uint)3731944308U;
            p82.target_system = (byte)(byte)78;
            p82.target_component = (byte)(byte)246;
            p82.type_mask = (byte)(byte)40;
            p82.q_SET(new float[4], 0);
            p82.body_roll_rate = (float)2.6530363E38F;
            p82.body_pitch_rate = (float)1.3187278E38F;
            p82.body_yaw_rate = (float) -1.2099334E38F;
            p82.thrust = (float) -1.3503659E38F;
            CommunicationChannel.instance.send(p82); //===============================
            ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.time_boot_ms = (uint)2457333397U;
            p83.type_mask = (byte)(byte)86;
            p83.q_SET(new float[4], 0);
            p83.body_roll_rate = (float)4.986665E37F;
            p83.body_pitch_rate = (float) -3.0522006E38F;
            p83.body_yaw_rate = (float) -2.2390121E38F;
            p83.thrust = (float)2.1624584E38F;
            CommunicationChannel.instance.send(p83); //===============================
            SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.time_boot_ms = (uint)3018945266U;
            p84.target_system = (byte)(byte)57;
            p84.target_component = (byte)(byte)93;
            p84.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_MISSION;
            p84.type_mask = (ushort)(ushort)58452;
            p84.x = (float) -6.467513E37F;
            p84.y = (float)6.7259685E37F;
            p84.z = (float) -1.0901958E38F;
            p84.vx = (float) -3.7189056E37F;
            p84.vy = (float)1.1102149E38F;
            p84.vz = (float) -5.4404253E37F;
            p84.afx = (float)9.7019176E36F;
            p84.afy = (float)3.0451772E38F;
            p84.afz = (float) -3.0605302E38F;
            p84.yaw = (float) -1.8918535E38F;
            p84.yaw_rate = (float) -2.24725E38F;
            CommunicationChannel.instance.send(p84); //===============================
            SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.time_boot_ms = (uint)2141816624U;
            p86.target_system = (byte)(byte)47;
            p86.target_component = (byte)(byte)91;
            p86.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
            p86.type_mask = (ushort)(ushort)51385;
            p86.lat_int = (int)1603755791;
            p86.lon_int = (int) -1263321098;
            p86.alt = (float)1.3289918E38F;
            p86.vx = (float) -2.8308883E38F;
            p86.vy = (float) -2.7959812E36F;
            p86.vz = (float)1.6984272E38F;
            p86.afx = (float)2.0088321E38F;
            p86.afy = (float)1.4550085E38F;
            p86.afz = (float)8.872086E37F;
            p86.yaw = (float) -9.105131E37F;
            p86.yaw_rate = (float) -3.1416558E38F;
            CommunicationChannel.instance.send(p86); //===============================
            POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.time_boot_ms = (uint)2013935875U;
            p87.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL;
            p87.type_mask = (ushort)(ushort)3737;
            p87.lat_int = (int) -896841088;
            p87.lon_int = (int) -1630367321;
            p87.alt = (float) -1.3439359E38F;
            p87.vx = (float)2.14196E37F;
            p87.vy = (float) -3.0453113E38F;
            p87.vz = (float)3.4007867E38F;
            p87.afx = (float) -3.1111444E38F;
            p87.afy = (float)1.273085E37F;
            p87.afz = (float) -8.640598E37F;
            p87.yaw = (float)1.8001529E38F;
            p87.yaw_rate = (float) -1.3140728E38F;
            CommunicationChannel.instance.send(p87); //===============================
            LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.time_boot_ms = (uint)1547053927U;
            p89.x = (float)3.353278E38F;
            p89.y = (float) -2.775529E38F;
            p89.z = (float)3.4178484E37F;
            p89.roll = (float) -2.6095375E38F;
            p89.pitch = (float) -7.6697095E37F;
            p89.yaw = (float) -1.8503368E38F;
            CommunicationChannel.instance.send(p89); //===============================
            HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.time_usec = (ulong)1139445188809075680L;
            p90.roll = (float) -3.380565E38F;
            p90.pitch = (float) -1.728132E38F;
            p90.yaw = (float) -1.5569171E38F;
            p90.rollspeed = (float)1.9660535E38F;
            p90.pitchspeed = (float) -1.3206126E38F;
            p90.yawspeed = (float) -3.0735776E37F;
            p90.lat = (int) -1499909894;
            p90.lon = (int) -160802901;
            p90.alt = (int)428064545;
            p90.vx = (short)(short)18430;
            p90.vy = (short)(short) -16761;
            p90.vz = (short)(short) -17555;
            p90.xacc = (short)(short) -18551;
            p90.yacc = (short)(short)25581;
            p90.zacc = (short)(short)3417;
            CommunicationChannel.instance.send(p90); //===============================
            HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.time_usec = (ulong)3104124972020698409L;
            p91.roll_ailerons = (float)1.5392118E38F;
            p91.pitch_elevator = (float)7.1220393E37F;
            p91.yaw_rudder = (float) -3.3357533E37F;
            p91.throttle = (float)2.4173783E38F;
            p91.aux1 = (float) -3.1404334E38F;
            p91.aux2 = (float) -2.514276E38F;
            p91.aux3 = (float)2.9807668E38F;
            p91.aux4 = (float)1.2559981E38F;
            p91.mode = (MAV_MODE)MAV_MODE.MAV_MODE_AUTO_ARMED;
            p91.nav_mode = (byte)(byte)80;
            CommunicationChannel.instance.send(p91); //===============================
            HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.time_usec = (ulong)1921411927136838284L;
            p92.chan1_raw = (ushort)(ushort)56102;
            p92.chan2_raw = (ushort)(ushort)23162;
            p92.chan3_raw = (ushort)(ushort)61333;
            p92.chan4_raw = (ushort)(ushort)5886;
            p92.chan5_raw = (ushort)(ushort)42612;
            p92.chan6_raw = (ushort)(ushort)25419;
            p92.chan7_raw = (ushort)(ushort)17847;
            p92.chan8_raw = (ushort)(ushort)21744;
            p92.chan9_raw = (ushort)(ushort)42380;
            p92.chan10_raw = (ushort)(ushort)8328;
            p92.chan11_raw = (ushort)(ushort)36971;
            p92.chan12_raw = (ushort)(ushort)60490;
            p92.rssi = (byte)(byte)238;
            CommunicationChannel.instance.send(p92); //===============================
            HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.time_usec = (ulong)3037571410778126198L;
            p93.controls_SET(new float[16], 0);
            p93.mode = (MAV_MODE)MAV_MODE.MAV_MODE_MANUAL_ARMED;
            p93.flags = (ulong)3031129882451309545L;
            CommunicationChannel.instance.send(p93); //===============================
            OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.time_usec = (ulong)3600933912328901177L;
            p100.sensor_id = (byte)(byte)6;
            p100.flow_x = (short)(short) -10299;
            p100.flow_y = (short)(short)589;
            p100.flow_comp_m_x = (float)2.3828436E38F;
            p100.flow_comp_m_y = (float)2.4921083E37F;
            p100.quality = (byte)(byte)20;
            p100.ground_distance = (float) -2.9039705E38F;
            p100.flow_rate_x_SET((float) -1.850186E38F, PH);
            p100.flow_rate_y_SET((float)2.1987897E38F, PH);
            CommunicationChannel.instance.send(p100); //===============================
            GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.usec = (ulong)9047816531608875130L;
            p101.x = (float)2.5837104E38F;
            p101.y = (float)1.652994E38F;
            p101.z = (float)2.2276848E38F;
            p101.roll = (float)2.1759296E38F;
            p101.pitch = (float)2.8385776E38F;
            p101.yaw = (float) -1.0450053E38F;
            CommunicationChannel.instance.send(p101); //===============================
            VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.usec = (ulong)4405015298640909937L;
            p102.x = (float)4.4265792E36F;
            p102.y = (float)3.2360175E38F;
            p102.z = (float) -5.66092E36F;
            p102.roll = (float)8.79719E37F;
            p102.pitch = (float)1.9784618E38F;
            p102.yaw = (float) -2.3578336E38F;
            CommunicationChannel.instance.send(p102); //===============================
            VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.usec = (ulong)671396057750507297L;
            p103.x = (float) -5.853253E36F;
            p103.y = (float) -1.0207435E38F;
            p103.z = (float)1.3212236E38F;
            CommunicationChannel.instance.send(p103); //===============================
            VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.usec = (ulong)3499604128026477630L;
            p104.x = (float) -1.0391014E38F;
            p104.y = (float) -4.66032E36F;
            p104.z = (float)3.0522303E38F;
            p104.roll = (float) -3.020584E38F;
            p104.pitch = (float) -2.2907848E38F;
            p104.yaw = (float) -2.3368865E37F;
            CommunicationChannel.instance.send(p104); //===============================
            HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.time_usec = (ulong)435064337192312489L;
            p105.xacc = (float) -2.2714632E38F;
            p105.yacc = (float)3.0020568E38F;
            p105.zacc = (float)2.4462115E38F;
            p105.xgyro = (float)3.3413183E38F;
            p105.ygyro = (float)1.5669032E38F;
            p105.zgyro = (float) -3.2269042E38F;
            p105.xmag = (float) -1.7996866E38F;
            p105.ymag = (float)5.758756E37F;
            p105.zmag = (float)3.0434916E38F;
            p105.abs_pressure = (float) -2.6178265E38F;
            p105.diff_pressure = (float)2.6400682E38F;
            p105.pressure_alt = (float)1.2401928E38F;
            p105.temperature = (float)9.5517764E36F;
            p105.fields_updated = (ushort)(ushort)58327;
            CommunicationChannel.instance.send(p105); //===============================
            OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.time_usec = (ulong)9049338883143620811L;
            p106.sensor_id = (byte)(byte)64;
            p106.integration_time_us = (uint)398984912U;
            p106.integrated_x = (float)1.2833293E38F;
            p106.integrated_y = (float)2.0755517E38F;
            p106.integrated_xgyro = (float) -1.0453116E38F;
            p106.integrated_ygyro = (float) -2.2626626E38F;
            p106.integrated_zgyro = (float)1.986079E38F;
            p106.temperature = (short)(short) -24481;
            p106.quality = (byte)(byte)180;
            p106.time_delta_distance_us = (uint)2040017538U;
            p106.distance = (float)2.4319396E38F;
            CommunicationChannel.instance.send(p106); //===============================
            HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.time_usec = (ulong)2063999609214902351L;
            p107.xacc = (float) -1.9988372E37F;
            p107.yacc = (float)2.0291431E38F;
            p107.zacc = (float)3.0628238E38F;
            p107.xgyro = (float) -4.408153E37F;
            p107.ygyro = (float) -3.1073877E38F;
            p107.zgyro = (float)3.386024E37F;
            p107.xmag = (float) -4.2520038E37F;
            p107.ymag = (float) -3.2284534E38F;
            p107.zmag = (float) -4.0921237E37F;
            p107.abs_pressure = (float) -1.2692679E38F;
            p107.diff_pressure = (float)1.0362686E37F;
            p107.pressure_alt = (float)3.2913672E38F;
            p107.temperature = (float) -7.5987733E37F;
            p107.fields_updated = (uint)1441089088U;
            CommunicationChannel.instance.send(p107); //===============================
            SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.q1 = (float)1.4714354E37F;
            p108.q2 = (float)1.7707296E38F;
            p108.q3 = (float) -8.1684636E37F;
            p108.q4 = (float) -3.2235858E37F;
            p108.roll = (float)2.2940916E37F;
            p108.pitch = (float)1.8995213E38F;
            p108.yaw = (float)8.598841E36F;
            p108.xacc = (float)3.6558397E37F;
            p108.yacc = (float)2.751052E38F;
            p108.zacc = (float)1.487232E37F;
            p108.xgyro = (float) -2.5335567E37F;
            p108.ygyro = (float) -8.1226994E37F;
            p108.zgyro = (float)2.5138665E38F;
            p108.lat = (float)1.331033E38F;
            p108.lon = (float) -1.5695967E38F;
            p108.alt = (float) -2.1351878E37F;
            p108.std_dev_horz = (float)2.9729946E38F;
            p108.std_dev_vert = (float) -5.479919E37F;
            p108.vn = (float) -1.3227166E38F;
            p108.ve = (float)2.2255363E38F;
            p108.vd = (float)1.4206053E38F;
            CommunicationChannel.instance.send(p108); //===============================
            RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.rssi = (byte)(byte)66;
            p109.remrssi = (byte)(byte)154;
            p109.txbuf = (byte)(byte)113;
            p109.noise = (byte)(byte)113;
            p109.remnoise = (byte)(byte)242;
            p109.rxerrors = (ushort)(ushort)2936;
            p109.fixed_ = (ushort)(ushort)53522;
            CommunicationChannel.instance.send(p109); //===============================
            FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_network = (byte)(byte)32;
            p110.target_system = (byte)(byte)136;
            p110.target_component = (byte)(byte)145;
            p110.payload_SET(new byte[251], 0);
            CommunicationChannel.instance.send(p110); //===============================
            TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.tc1 = (long) -1503039700838571402L;
            p111.ts1 = (long) -7399474123693155433L;
            CommunicationChannel.instance.send(p111); //===============================
            CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.time_usec = (ulong)879842974567887100L;
            p112.seq = (uint)4021842444U;
            CommunicationChannel.instance.send(p112); //===============================
            HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.time_usec = (ulong)151890693183708020L;
            p113.fix_type = (byte)(byte)99;
            p113.lat = (int) -402555319;
            p113.lon = (int) -1125390396;
            p113.alt = (int) -273014893;
            p113.eph = (ushort)(ushort)30387;
            p113.epv = (ushort)(ushort)31649;
            p113.vel = (ushort)(ushort)35200;
            p113.vn = (short)(short)25845;
            p113.ve = (short)(short)17419;
            p113.vd = (short)(short)338;
            p113.cog = (ushort)(ushort)37459;
            p113.satellites_visible = (byte)(byte)210;
            CommunicationChannel.instance.send(p113); //===============================
            HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.time_usec = (ulong)1501202751287958357L;
            p114.sensor_id = (byte)(byte)198;
            p114.integration_time_us = (uint)3689863910U;
            p114.integrated_x = (float)8.491584E36F;
            p114.integrated_y = (float) -7.402576E37F;
            p114.integrated_xgyro = (float) -2.6749365E38F;
            p114.integrated_ygyro = (float)1.621513E38F;
            p114.integrated_zgyro = (float) -1.1848993E38F;
            p114.temperature = (short)(short)8877;
            p114.quality = (byte)(byte)116;
            p114.time_delta_distance_us = (uint)3143271972U;
            p114.distance = (float) -1.6588867E38F;
            CommunicationChannel.instance.send(p114); //===============================
            HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.time_usec = (ulong)5553252239711254862L;
            p115.attitude_quaternion_SET(new float[4], 0);
            p115.rollspeed = (float)3.1271655E37F;
            p115.pitchspeed = (float) -1.3192864E38F;
            p115.yawspeed = (float)1.5329772E38F;
            p115.lat = (int) -341284622;
            p115.lon = (int)1715757881;
            p115.alt = (int) -470136390;
            p115.vx = (short)(short) -4694;
            p115.vy = (short)(short) -20406;
            p115.vz = (short)(short) -23147;
            p115.ind_airspeed = (ushort)(ushort)30810;
            p115.true_airspeed = (ushort)(ushort)16278;
            p115.xacc = (short)(short)1208;
            p115.yacc = (short)(short) -25879;
            p115.zacc = (short)(short)18472;
            CommunicationChannel.instance.send(p115); //===============================
            SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.time_boot_ms = (uint)2879779603U;
            p116.xacc = (short)(short)8981;
            p116.yacc = (short)(short)13749;
            p116.zacc = (short)(short) -7163;
            p116.xgyro = (short)(short)7648;
            p116.ygyro = (short)(short)32208;
            p116.zgyro = (short)(short)27832;
            p116.xmag = (short)(short)13927;
            p116.ymag = (short)(short)9917;
            p116.zmag = (short)(short) -7895;
            CommunicationChannel.instance.send(p116); //===============================
            LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.target_system = (byte)(byte)74;
            p117.target_component = (byte)(byte)63;
            p117.start = (ushort)(ushort)60222;
            p117.end = (ushort)(ushort)44178;
            CommunicationChannel.instance.send(p117); //===============================
            LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.id = (ushort)(ushort)36133;
            p118.num_logs = (ushort)(ushort)23473;
            p118.last_log_num = (ushort)(ushort)27515;
            p118.time_utc = (uint)1388932333U;
            p118.size = (uint)1672630482U;
            CommunicationChannel.instance.send(p118); //===============================
            LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.target_system = (byte)(byte)208;
            p119.target_component = (byte)(byte)74;
            p119.id = (ushort)(ushort)11539;
            p119.ofs = (uint)193334022U;
            p119.count = (uint)3428166922U;
            CommunicationChannel.instance.send(p119); //===============================
            LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.id = (ushort)(ushort)62634;
            p120.ofs = (uint)257272052U;
            p120.count = (byte)(byte)228;
            p120.data__SET(new byte[90], 0);
            CommunicationChannel.instance.send(p120); //===============================
            LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_system = (byte)(byte)220;
            p121.target_component = (byte)(byte)136;
            CommunicationChannel.instance.send(p121); //===============================
            LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_system = (byte)(byte)41;
            p122.target_component = (byte)(byte)61;
            CommunicationChannel.instance.send(p122); //===============================
            GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.target_system = (byte)(byte)246;
            p123.target_component = (byte)(byte)127;
            p123.len = (byte)(byte)88;
            p123.data__SET(new byte[110], 0);
            CommunicationChannel.instance.send(p123); //===============================
            GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.time_usec = (ulong)8172500613777617706L;
            p124.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED;
            p124.lat = (int) -807974256;
            p124.lon = (int) -488599584;
            p124.alt = (int) -986904741;
            p124.eph = (ushort)(ushort)8542;
            p124.epv = (ushort)(ushort)54050;
            p124.vel = (ushort)(ushort)2763;
            p124.cog = (ushort)(ushort)22904;
            p124.satellites_visible = (byte)(byte)53;
            p124.dgps_numch = (byte)(byte)226;
            p124.dgps_age = (uint)551419212U;
            CommunicationChannel.instance.send(p124); //===============================
            POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vcc = (ushort)(ushort)3207;
            p125.Vservo = (ushort)(ushort)29896;
            p125.flags = (MAV_POWER_STATUS)MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED;
            CommunicationChannel.instance.send(p125); //===============================
            SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.device = (SERIAL_CONTROL_DEV)SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1;
            p126.flags = (SERIAL_CONTROL_FLAG)SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND;
            p126.timeout = (ushort)(ushort)18733;
            p126.baudrate = (uint)1456854581U;
            p126.count = (byte)(byte)24;
            p126.data__SET(new byte[70], 0);
            CommunicationChannel.instance.send(p126); //===============================
            GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.time_last_baseline_ms = (uint)417462910U;
            p127.rtk_receiver_id = (byte)(byte)71;
            p127.wn = (ushort)(ushort)13987;
            p127.tow = (uint)771672191U;
            p127.rtk_health = (byte)(byte)146;
            p127.rtk_rate = (byte)(byte)20;
            p127.nsats = (byte)(byte)158;
            p127.baseline_coords_type = (byte)(byte)65;
            p127.baseline_a_mm = (int)1991699074;
            p127.baseline_b_mm = (int)1902903215;
            p127.baseline_c_mm = (int)1757027543;
            p127.accuracy = (uint)1149956894U;
            p127.iar_num_hypotheses = (int)1867231554;
            CommunicationChannel.instance.send(p127); //===============================
            GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.time_last_baseline_ms = (uint)4290315051U;
            p128.rtk_receiver_id = (byte)(byte)168;
            p128.wn = (ushort)(ushort)58640;
            p128.tow = (uint)1404444828U;
            p128.rtk_health = (byte)(byte)172;
            p128.rtk_rate = (byte)(byte)155;
            p128.nsats = (byte)(byte)43;
            p128.baseline_coords_type = (byte)(byte)226;
            p128.baseline_a_mm = (int)2001573970;
            p128.baseline_b_mm = (int) -500254413;
            p128.baseline_c_mm = (int) -1149104520;
            p128.accuracy = (uint)2810506420U;
            p128.iar_num_hypotheses = (int) -1818946650;
            CommunicationChannel.instance.send(p128); //===============================
            SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.time_boot_ms = (uint)1544293210U;
            p129.xacc = (short)(short) -2051;
            p129.yacc = (short)(short) -3151;
            p129.zacc = (short)(short)19954;
            p129.xgyro = (short)(short)10361;
            p129.ygyro = (short)(short) -17987;
            p129.zgyro = (short)(short) -24192;
            p129.xmag = (short)(short) -14691;
            p129.ymag = (short)(short) -1255;
            p129.zmag = (short)(short) -32242;
            CommunicationChannel.instance.send(p129); //===============================
            DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.type = (byte)(byte)62;
            p130.size = (uint)2047609084U;
            p130.width = (ushort)(ushort)12064;
            p130.height = (ushort)(ushort)14714;
            p130.packets = (ushort)(ushort)18351;
            p130.payload = (byte)(byte)174;
            p130.jpg_quality = (byte)(byte)139;
            CommunicationChannel.instance.send(p130); //===============================
            ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.seqnr = (ushort)(ushort)3335;
            p131.data__SET(new byte[253], 0);
            CommunicationChannel.instance.send(p131); //===============================
            DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.time_boot_ms = (uint)1341515488U;
            p132.min_distance = (ushort)(ushort)21807;
            p132.max_distance = (ushort)(ushort)51594;
            p132.current_distance = (ushort)(ushort)44058;
            p132.type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND;
            p132.id = (byte)(byte)49;
            p132.orientation = (MAV_SENSOR_ORIENTATION)MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_90_PITCH_270;
            p132.covariance = (byte)(byte)129;
            CommunicationChannel.instance.send(p132); //===============================
            TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.lat = (int) -1847018187;
            p133.lon = (int)1019613964;
            p133.grid_spacing = (ushort)(ushort)4364;
            p133.mask = (ulong)3920671439688878890L;
            CommunicationChannel.instance.send(p133); //===============================
            TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.lat = (int) -932305191;
            p134.lon = (int) -1726563606;
            p134.grid_spacing = (ushort)(ushort)44509;
            p134.gridbit = (byte)(byte)223;
            p134.data__SET(new short[16], 0);
            CommunicationChannel.instance.send(p134); //===============================
            TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lat = (int) -1345964771;
            p135.lon = (int)1972887412;
            CommunicationChannel.instance.send(p135); //===============================
            TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.lat = (int) -1401056611;
            p136.lon = (int)1945617003;
            p136.spacing = (ushort)(ushort)22878;
            p136.terrain_height = (float)2.2629683E38F;
            p136.current_height = (float)3.030444E38F;
            p136.pending = (ushort)(ushort)19417;
            p136.loaded = (ushort)(ushort)63961;
            CommunicationChannel.instance.send(p136); //===============================
            SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.time_boot_ms = (uint)3908596872U;
            p137.press_abs = (float)3.1785258E38F;
            p137.press_diff = (float)5.2670844E36F;
            p137.temperature = (short)(short)20043;
            CommunicationChannel.instance.send(p137); //===============================
            ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.time_usec = (ulong)9111344028340210237L;
            p138.q_SET(new float[4], 0);
            p138.x = (float)2.5542912E38F;
            p138.y = (float)1.3204487E38F;
            p138.z = (float) -2.3967032E38F;
            CommunicationChannel.instance.send(p138); //===============================
            SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.time_usec = (ulong)7468064149202783578L;
            p139.group_mlx = (byte)(byte)202;
            p139.target_system = (byte)(byte)43;
            p139.target_component = (byte)(byte)187;
            p139.controls_SET(new float[8], 0);
            CommunicationChannel.instance.send(p139); //===============================
            ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.time_usec = (ulong)1674235363477509360L;
            p140.group_mlx = (byte)(byte)236;
            p140.controls_SET(new float[8], 0);
            CommunicationChannel.instance.send(p140); //===============================
            ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.time_usec = (ulong)5696671974623740592L;
            p141.altitude_monotonic = (float) -8.766139E37F;
            p141.altitude_amsl = (float) -1.4912137E38F;
            p141.altitude_local = (float) -2.4474342E38F;
            p141.altitude_relative = (float)2.65906E38F;
            p141.altitude_terrain = (float)1.2888466E38F;
            p141.bottom_clearance = (float) -2.8816207E38F;
            CommunicationChannel.instance.send(p141); //===============================
            RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.request_id = (byte)(byte)146;
            p142.uri_type = (byte)(byte)10;
            p142.uri_SET(new byte[120], 0);
            p142.transfer_type = (byte)(byte)101;
            p142.storage_SET(new byte[120], 0);
            CommunicationChannel.instance.send(p142); //===============================
            SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.time_boot_ms = (uint)2549663315U;
            p143.press_abs = (float) -3.2203189E38F;
            p143.press_diff = (float)2.6259443E38F;
            p143.temperature = (short)(short)1262;
            CommunicationChannel.instance.send(p143); //===============================
            FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.timestamp = (ulong)1648834426747005474L;
            p144.est_capabilities = (byte)(byte)183;
            p144.lat = (int)170382667;
            p144.lon = (int) -13014881;
            p144.alt = (float)1.8369288E38F;
            p144.vel_SET(new float[3], 0);
            p144.acc_SET(new float[3], 0);
            p144.attitude_q_SET(new float[4], 0);
            p144.rates_SET(new float[3], 0);
            p144.position_cov_SET(new float[3], 0);
            p144.custom_state = (ulong)2616976112409946899L;
            CommunicationChannel.instance.send(p144); //===============================
            CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.time_usec = (ulong)6666038485157016772L;
            p146.x_acc = (float) -1.7571655E38F;
            p146.y_acc = (float) -1.5536496E38F;
            p146.z_acc = (float)1.2453744E38F;
            p146.x_vel = (float) -2.0511075E37F;
            p146.y_vel = (float) -1.4068839E38F;
            p146.z_vel = (float)6.2249124E37F;
            p146.x_pos = (float)1.3628692E38F;
            p146.y_pos = (float)1.3154448E38F;
            p146.z_pos = (float)7.554366E37F;
            p146.airspeed = (float)2.4907494E37F;
            p146.vel_variance_SET(new float[3], 0);
            p146.pos_variance_SET(new float[3], 0);
            p146.q_SET(new float[4], 0);
            p146.roll_rate = (float) -2.2258853E38F;
            p146.pitch_rate = (float) -4.5283887E37F;
            p146.yaw_rate = (float) -1.710434E38F;
            CommunicationChannel.instance.send(p146); //===============================
            BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.id = (byte)(byte)225;
            p147.battery_function = (MAV_BATTERY_FUNCTION)MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_ALL;
            p147.type = (MAV_BATTERY_TYPE)MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIPO;
            p147.temperature = (short)(short) -14735;
            p147.voltages_SET(new ushort[10], 0);
            p147.current_battery = (short)(short) -28110;
            p147.current_consumed = (int) -2087051182;
            p147.energy_consumed = (int) -1037146434;
            p147.battery_remaining = (sbyte)(sbyte) - 76;
            CommunicationChannel.instance.send(p147); //===============================
            AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY)MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION;
            p148.flight_sw_version = (uint)1104776116U;
            p148.middleware_sw_version = (uint)13539139U;
            p148.os_sw_version = (uint)1735796060U;
            p148.board_version = (uint)736086781U;
            p148.flight_custom_version_SET(new byte[8], 0);
            p148.middleware_custom_version_SET(new byte[8], 0);
            p148.os_custom_version_SET(new byte[8], 0);
            p148.vendor_id = (ushort)(ushort)27641;
            p148.product_id = (ushort)(ushort)30871;
            p148.uid = (ulong)2707453199908554751L;
            p148.uid2_SET(new byte[18], 0, PH);
            CommunicationChannel.instance.send(p148); //===============================
            LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.time_usec = (ulong)7681791725793954915L;
            p149.target_num = (byte)(byte)128;
            p149.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT;
            p149.angle_x = (float)4.251168E37F;
            p149.angle_y = (float)9.844227E37F;
            p149.distance = (float) -3.3754073E38F;
            p149.size_x = (float) -1.8485808E38F;
            p149.size_y = (float)3.2579507E38F;
            p149.x_SET((float)2.2682007E38F, PH);
            p149.y_SET((float)3.3827672E38F, PH);
            p149.z_SET((float)2.236883E38F, PH);
            p149.q_SET(new float[4], 0, PH);
            p149.type = (LANDING_TARGET_TYPE)LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER;
            p149.position_valid_SET((byte)(byte)15, PH);
            CommunicationChannel.instance.send(p149); //===============================
            SENS_POWER p201 = CommunicationChannel.new_SENS_POWER();
            PH.setPack(p201);
            p201.adc121_vspb_volt = (float)3.223705E38F;
            p201.adc121_cspb_amp = (float)1.0759344E38F;
            p201.adc121_cs1_amp = (float) -1.1813239E38F;
            p201.adc121_cs2_amp = (float)2.9125228E38F;
            CommunicationChannel.instance.send(p201); //===============================
            SENS_MPPT p202 = CommunicationChannel.new_SENS_MPPT();
            PH.setPack(p202);
            p202.mppt_timestamp = (ulong)3777370861058295573L;
            p202.mppt1_volt = (float)3.8826437E37F;
            p202.mppt1_amp = (float)8.008144E37F;
            p202.mppt1_pwm = (ushort)(ushort)8597;
            p202.mppt1_status = (byte)(byte)233;
            p202.mppt2_volt = (float) -1.2110641E38F;
            p202.mppt2_amp = (float) -1.194097E38F;
            p202.mppt2_pwm = (ushort)(ushort)53489;
            p202.mppt2_status = (byte)(byte)226;
            p202.mppt3_volt = (float) -3.360779E38F;
            p202.mppt3_amp = (float)3.1698125E38F;
            p202.mppt3_pwm = (ushort)(ushort)43690;
            p202.mppt3_status = (byte)(byte)234;
            CommunicationChannel.instance.send(p202); //===============================
            ASLCTRL_DATA p203 = CommunicationChannel.new_ASLCTRL_DATA();
            PH.setPack(p203);
            p203.timestamp = (ulong)2367067291403219551L;
            p203.aslctrl_mode = (byte)(byte)183;
            p203.h = (float) -1.0040537E38F;
            p203.hRef = (float)7.2549754E36F;
            p203.hRef_t = (float) -3.6323443E37F;
            p203.PitchAngle = (float) -2.827568E38F;
            p203.PitchAngleRef = (float) -2.2264628E38F;
            p203.q = (float) -3.2279923E38F;
            p203.qRef = (float)2.9876691E38F;
            p203.uElev = (float) -3.3883948E37F;
            p203.uThrot = (float) -1.1850911E38F;
            p203.uThrot2 = (float) -1.6203633E38F;
            p203.nZ = (float) -3.25823E38F;
            p203.AirspeedRef = (float)9.271926E37F;
            p203.SpoilersEngaged = (byte)(byte)22;
            p203.YawAngle = (float)2.5468196E38F;
            p203.YawAngleRef = (float)2.016648E38F;
            p203.RollAngle = (float)1.692656E38F;
            p203.RollAngleRef = (float)1.697072E38F;
            p203.p = (float) -3.285536E38F;
            p203.pRef = (float) -2.1730112E37F;
            p203.r = (float)3.1192239E38F;
            p203.rRef = (float)3.3973883E37F;
            p203.uAil = (float) -1.1998052E38F;
            p203.uRud = (float)2.640142E38F;
            CommunicationChannel.instance.send(p203); //===============================
            ASLCTRL_DEBUG p204 = CommunicationChannel.new_ASLCTRL_DEBUG();
            PH.setPack(p204);
            p204.i32_1 = (uint)3320120508U;
            p204.i8_1 = (byte)(byte)23;
            p204.i8_2 = (byte)(byte)130;
            p204.f_1 = (float)7.9746713E37F;
            p204.f_2 = (float)1.246111E38F;
            p204.f_3 = (float)1.398508E38F;
            p204.f_4 = (float)3.6503797E37F;
            p204.f_5 = (float) -2.7513338E38F;
            p204.f_6 = (float) -1.1468325E38F;
            p204.f_7 = (float) -1.5219127E38F;
            p204.f_8 = (float) -2.5675904E38F;
            CommunicationChannel.instance.send(p204); //===============================
            ASLUAV_STATUS p205 = CommunicationChannel.new_ASLUAV_STATUS();
            PH.setPack(p205);
            p205.LED_status = (byte)(byte)253;
            p205.SATCOM_status = (byte)(byte)19;
            p205.Servo_status_SET(new byte[8], 0);
            p205.Motor_rpm = (float) -1.0580933E37F;
            CommunicationChannel.instance.send(p205); //===============================
            EKF_EXT p206 = CommunicationChannel.new_EKF_EXT();
            PH.setPack(p206);
            p206.timestamp = (ulong)6420046796079162046L;
            p206.Windspeed = (float) -1.7735716E38F;
            p206.WindDir = (float)1.7969564E38F;
            p206.WindZ = (float) -3.2297748E38F;
            p206.Airspeed = (float)2.1082857E38F;
            p206.beta = (float) -5.0587707E37F;
            p206.alpha = (float) -3.366339E38F;
            CommunicationChannel.instance.send(p206); //===============================
            ASL_OBCTRL p207 = CommunicationChannel.new_ASL_OBCTRL();
            PH.setPack(p207);
            p207.timestamp = (ulong)4713411640850274272L;
            p207.uElev = (float)3.19438E38F;
            p207.uThrot = (float) -1.2777994E38F;
            p207.uThrot2 = (float) -1.8104407E38F;
            p207.uAilL = (float)1.6536273E38F;
            p207.uAilR = (float)2.9688732E38F;
            p207.uRud = (float)2.8501846E37F;
            p207.obctrl_status = (byte)(byte)175;
            CommunicationChannel.instance.send(p207); //===============================
            SENS_ATMOS p208 = CommunicationChannel.new_SENS_ATMOS();
            PH.setPack(p208);
            p208.TempAmbient = (float)2.3642632E37F;
            p208.Humidity = (float) -1.3956598E38F;
            CommunicationChannel.instance.send(p208); //===============================
            SENS_BATMON p209 = CommunicationChannel.new_SENS_BATMON();
            PH.setPack(p209);
            p209.temperature = (float)4.69945E37F;
            p209.voltage = (ushort)(ushort)56178;
            p209.current = (short)(short)13919;
            p209.SoC = (byte)(byte)53;
            p209.batterystatus = (ushort)(ushort)15429;
            p209.serialnumber = (ushort)(ushort)25869;
            p209.hostfetcontrol = (ushort)(ushort)20698;
            p209.cellvoltage1 = (ushort)(ushort)46980;
            p209.cellvoltage2 = (ushort)(ushort)60702;
            p209.cellvoltage3 = (ushort)(ushort)44020;
            p209.cellvoltage4 = (ushort)(ushort)8691;
            p209.cellvoltage5 = (ushort)(ushort)60704;
            p209.cellvoltage6 = (ushort)(ushort)8575;
            CommunicationChannel.instance.send(p209); //===============================
            FW_SOARING_DATA p210 = CommunicationChannel.new_FW_SOARING_DATA();
            PH.setPack(p210);
            p210.timestamp = (ulong)8362921031695478499L;
            p210.timestampModeChanged = (ulong)3347944737172095343L;
            p210.xW = (float) -2.3979887E38F;
            p210.xR = (float)2.6174861E38F;
            p210.xLat = (float) -2.9646794E38F;
            p210.xLon = (float) -3.2737655E38F;
            p210.VarW = (float) -7.5619425E37F;
            p210.VarR = (float)6.2454927E36F;
            p210.VarLat = (float)2.5819875E38F;
            p210.VarLon = (float) -6.612637E37F;
            p210.LoiterRadius = (float) -2.265813E36F;
            p210.LoiterDirection = (float)3.2541345E38F;
            p210.DistToSoarPoint = (float)3.1008295E38F;
            p210.vSinkExp = (float) -9.064634E37F;
            p210.z1_LocalUpdraftSpeed = (float) -2.8719438E38F;
            p210.z2_DeltaRoll = (float) -4.4192024E37F;
            p210.z1_exp = (float) -9.012679E37F;
            p210.z2_exp = (float)3.0349386E37F;
            p210.ThermalGSNorth = (float)7.1168055E37F;
            p210.ThermalGSEast = (float)2.2876544E38F;
            p210.TSE_dot = (float)1.7150806E38F;
            p210.DebugVar1 = (float)2.667516E38F;
            p210.DebugVar2 = (float) -2.3445419E38F;
            p210.ControlMode = (byte)(byte)33;
            p210.valid = (byte)(byte)76;
            CommunicationChannel.instance.send(p210); //===============================
            SENSORPOD_STATUS p211 = CommunicationChannel.new_SENSORPOD_STATUS();
            PH.setPack(p211);
            p211.timestamp = (ulong)4118954457057901876L;
            p211.visensor_rate_1 = (byte)(byte)218;
            p211.visensor_rate_2 = (byte)(byte)25;
            p211.visensor_rate_3 = (byte)(byte)19;
            p211.visensor_rate_4 = (byte)(byte)225;
            p211.recording_nodes_count = (byte)(byte)55;
            p211.cpu_temp = (byte)(byte)190;
            p211.free_space = (ushort)(ushort)8229;
            CommunicationChannel.instance.send(p211); //===============================
            SENS_POWER_BOARD p212 = CommunicationChannel.new_SENS_POWER_BOARD();
            PH.setPack(p212);
            p212.timestamp = (ulong)3407643827065108330L;
            p212.pwr_brd_status = (byte)(byte)75;
            p212.pwr_brd_led_status = (byte)(byte)16;
            p212.pwr_brd_system_volt = (float)6.4849105E37F;
            p212.pwr_brd_servo_volt = (float) -3.0885796E38F;
            p212.pwr_brd_mot_l_amp = (float) -2.586531E37F;
            p212.pwr_brd_mot_r_amp = (float)4.0965529E37F;
            p212.pwr_brd_servo_1_amp = (float)1.1846019E38F;
            p212.pwr_brd_servo_2_amp = (float)2.209451E38F;
            p212.pwr_brd_servo_3_amp = (float) -3.1989492E37F;
            p212.pwr_brd_servo_4_amp = (float) -1.0542265E38F;
            p212.pwr_brd_aux_amp = (float) -4.4392556E37F;
            CommunicationChannel.instance.send(p212); //===============================
            ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.time_usec = (ulong)535743464139361336L;
            p230.flags = (ESTIMATOR_STATUS_FLAGS)ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE;
            p230.vel_ratio = (float)2.3007388E38F;
            p230.pos_horiz_ratio = (float) -2.0698035E38F;
            p230.pos_vert_ratio = (float)1.2210541E38F;
            p230.mag_ratio = (float)2.3705786E38F;
            p230.hagl_ratio = (float) -1.7010794E38F;
            p230.tas_ratio = (float) -2.255953E38F;
            p230.pos_horiz_accuracy = (float)1.3012105E38F;
            p230.pos_vert_accuracy = (float)7.1926216E37F;
            CommunicationChannel.instance.send(p230); //===============================
            WIND_COV p231 = CommunicationChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.time_usec = (ulong)6239409160732966736L;
            p231.wind_x = (float)3.1551383E38F;
            p231.wind_y = (float)1.8512374E38F;
            p231.wind_z = (float) -1.6132138E38F;
            p231.var_horiz = (float)2.338532E37F;
            p231.var_vert = (float)2.0996567E38F;
            p231.wind_alt = (float)9.784038E35F;
            p231.horiz_accuracy = (float) -1.1536318E38F;
            p231.vert_accuracy = (float)5.7486124E37F;
            CommunicationChannel.instance.send(p231); //===============================
            GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.time_usec = (ulong)1377353011022981026L;
            p232.gps_id = (byte)(byte)121;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS)GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY;
            p232.time_week_ms = (uint)1810069853U;
            p232.time_week = (ushort)(ushort)63485;
            p232.fix_type = (byte)(byte)171;
            p232.lat = (int) -1902965975;
            p232.lon = (int) -401959241;
            p232.alt = (float)8.2521407E37F;
            p232.hdop = (float)1.4410857E38F;
            p232.vdop = (float) -1.6213628E38F;
            p232.vn = (float)1.087626E38F;
            p232.ve = (float) -3.2736831E38F;
            p232.vd = (float) -8.0173277E37F;
            p232.speed_accuracy = (float) -2.9417968E38F;
            p232.horiz_accuracy = (float) -1.4912185E38F;
            p232.vert_accuracy = (float)3.1497016E38F;
            p232.satellites_visible = (byte)(byte)218;
            CommunicationChannel.instance.send(p232); //===============================
            GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.flags = (byte)(byte)220;
            p233.len = (byte)(byte)113;
            p233.data__SET(new byte[180], 0);
            CommunicationChannel.instance.send(p233); //===============================
            HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
            p234.custom_mode = (uint)3793350625U;
            p234.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR;
            p234.roll = (short)(short)672;
            p234.pitch = (short)(short) -8055;
            p234.heading = (ushort)(ushort)36814;
            p234.throttle = (sbyte)(sbyte) - 57;
            p234.heading_sp = (short)(short)31409;
            p234.latitude = (int) -1075811187;
            p234.longitude = (int)415573665;
            p234.altitude_amsl = (short)(short) -28892;
            p234.altitude_sp = (short)(short) -10205;
            p234.airspeed = (byte)(byte)78;
            p234.airspeed_sp = (byte)(byte)231;
            p234.groundspeed = (byte)(byte)232;
            p234.climb_rate = (sbyte)(sbyte) - 26;
            p234.gps_nsat = (byte)(byte)135;
            p234.gps_fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC;
            p234.battery_remaining = (byte)(byte)73;
            p234.temperature = (sbyte)(sbyte)99;
            p234.temperature_air = (sbyte)(sbyte) - 82;
            p234.failsafe = (byte)(byte)83;
            p234.wp_num = (byte)(byte)179;
            p234.wp_distance = (ushort)(ushort)37611;
            CommunicationChannel.instance.send(p234); //===============================
            VIBRATION p241 = CommunicationChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.time_usec = (ulong)7406194672592132715L;
            p241.vibration_x = (float)1.6454698E38F;
            p241.vibration_y = (float) -1.0073432E38F;
            p241.vibration_z = (float) -2.8545546E38F;
            p241.clipping_0 = (uint)3296691218U;
            p241.clipping_1 = (uint)3238734060U;
            p241.clipping_2 = (uint)3733838204U;
            CommunicationChannel.instance.send(p241); //===============================
            HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.latitude = (int) -1928489094;
            p242.longitude = (int) -998395907;
            p242.altitude = (int)922487776;
            p242.x = (float)1.5060209E38F;
            p242.y = (float)5.596362E37F;
            p242.z = (float)3.3744782E38F;
            p242.q_SET(new float[4], 0);
            p242.approach_x = (float) -2.680054E37F;
            p242.approach_y = (float) -1.826914E38F;
            p242.approach_z = (float)1.7380826E38F;
            p242.time_usec_SET((ulong)7691333496980391522L, PH);
            CommunicationChannel.instance.send(p242); //===============================
            SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.target_system = (byte)(byte)203;
            p243.latitude = (int) -870542923;
            p243.longitude = (int) -789801760;
            p243.altitude = (int) -358985068;
            p243.x = (float)5.324515E37F;
            p243.y = (float) -2.5525569E38F;
            p243.z = (float)2.8178696E38F;
            p243.q_SET(new float[4], 0);
            p243.approach_x = (float)8.447487E37F;
            p243.approach_y = (float) -4.4738527E36F;
            p243.approach_z = (float) -2.140387E38F;
            p243.time_usec_SET((ulong)7268160791821904385L, PH);
            CommunicationChannel.instance.send(p243); //===============================
            MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.message_id = (ushort)(ushort)28876;
            p244.interval_us = (int)618381586;
            CommunicationChannel.instance.send(p244); //===============================
            EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.vtol_state = (MAV_VTOL_STATE)MAV_VTOL_STATE.MAV_VTOL_STATE_MC;
            p245.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING;
            CommunicationChannel.instance.send(p245); //===============================
            ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.ICAO_address = (uint)874604582U;
            p246.lat = (int) -1128520673;
            p246.lon = (int) -1279465049;
            p246.altitude_type = (ADSB_ALTITUDE_TYPE)ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC;
            p246.altitude = (int) -811649222;
            p246.heading = (ushort)(ushort)60965;
            p246.hor_velocity = (ushort)(ushort)33811;
            p246.ver_velocity = (short)(short)18293;
            p246.callsign_SET("DEMO", PH);
            p246.emitter_type = (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_GLIDER;
            p246.tslc = (byte)(byte)193;
            p246.flags = (ADSB_FLAGS)ADSB_FLAGS.ADSB_FLAGS_SIMULATED;
            p246.squawk = (ushort)(ushort)22140;
            CommunicationChannel.instance.send(p246); //===============================
            COLLISION p247 = CommunicationChannel.new_COLLISION();
            PH.setPack(p247);
            p247.src_ = (MAV_COLLISION_SRC)MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT;
            p247.id = (uint)3195629205U;
            p247.action = (MAV_COLLISION_ACTION)MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_PERPENDICULAR;
            p247.threat_level = (MAV_COLLISION_THREAT_LEVEL)MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH;
            p247.time_to_minimum_delta = (float) -2.0086129E38F;
            p247.altitude_minimum_delta = (float)4.220826E37F;
            p247.horizontal_minimum_delta = (float) -2.488599E38F;
            CommunicationChannel.instance.send(p247); //===============================
            V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.target_network = (byte)(byte)176;
            p248.target_system = (byte)(byte)142;
            p248.target_component = (byte)(byte)241;
            p248.message_type = (ushort)(ushort)29830;
            p248.payload_SET(new byte[249], 0);
            CommunicationChannel.instance.send(p248); //===============================
            MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.address = (ushort)(ushort)413;
            p249.ver = (byte)(byte)83;
            p249.type = (byte)(byte)123;
            p249.value_SET(new sbyte[32], 0);
            CommunicationChannel.instance.send(p249); //===============================
            DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.name_SET("DEMO", PH);
            p250.time_usec = (ulong)5090321371334609619L;
            p250.x = (float)3.186642E38F;
            p250.y = (float) -2.6496473E37F;
            p250.z = (float) -2.462848E38F;
            CommunicationChannel.instance.send(p250); //===============================
            NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.time_boot_ms = (uint)2041231419U;
            p251.name_SET("DEMO", PH);
            p251.value = (float)9.792253E37F;
            CommunicationChannel.instance.send(p251); //===============================
            NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.time_boot_ms = (uint)722010365U;
            p252.name_SET("DEMO", PH);
            p252.value = (int)1242177692;
            CommunicationChannel.instance.send(p252); //===============================
            STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.severity = (MAV_SEVERITY)MAV_SEVERITY.MAV_SEVERITY_INFO;
            p253.text_SET("DEMO", PH);
            CommunicationChannel.instance.send(p253); //===============================
            DEBUG p254 = CommunicationChannel.new_DEBUG();
            PH.setPack(p254);
            p254.time_boot_ms = (uint)1280351594U;
            p254.ind = (byte)(byte)180;
            p254.value = (float)2.3241848E38F;
            CommunicationChannel.instance.send(p254); //===============================
            SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.target_system = (byte)(byte)74;
            p256.target_component = (byte)(byte)118;
            p256.secret_key_SET(new byte[32], 0);
            p256.initial_timestamp = (ulong)8726291539389969611L;
            CommunicationChannel.instance.send(p256); //===============================
            BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.time_boot_ms = (uint)2689554937U;
            p257.last_change_ms = (uint)2676609733U;
            p257.state = (byte)(byte)17;
            CommunicationChannel.instance.send(p257); //===============================
            PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_system = (byte)(byte)174;
            p258.target_component = (byte)(byte)35;
            p258.tune_SET("DEMO", PH);
            CommunicationChannel.instance.send(p258); //===============================
            CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.time_boot_ms = (uint)383687569U;
            p259.vendor_name_SET(new byte[32], 0);
            p259.model_name_SET(new byte[32], 0);
            p259.firmware_version = (uint)2212679047U;
            p259.focal_length = (float)1.2218215E38F;
            p259.sensor_size_h = (float) -1.7164273E38F;
            p259.sensor_size_v = (float)2.9869604E38F;
            p259.resolution_h = (ushort)(ushort)11482;
            p259.resolution_v = (ushort)(ushort)49182;
            p259.lens_id = (byte)(byte)120;
            p259.flags = (CAMERA_CAP_FLAGS)CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE;
            p259.cam_definition_version = (ushort)(ushort)21239;
            p259.cam_definition_uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p259); //===============================
            CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.time_boot_ms = (uint)2471663898U;
            p260.mode_id = (CAMERA_MODE)CAMERA_MODE.CAMERA_MODE_VIDEO;
            CommunicationChannel.instance.send(p260); //===============================
            STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.time_boot_ms = (uint)1606594106U;
            p261.storage_id = (byte)(byte)172;
            p261.storage_count = (byte)(byte)106;
            p261.status = (byte)(byte)155;
            p261.total_capacity = (float) -9.706666E37F;
            p261.used_capacity = (float)8.0549506E37F;
            p261.available_capacity = (float) -2.0912541E38F;
            p261.read_speed = (float)1.5151011E38F;
            p261.write_speed = (float)1.1080403E38F;
            CommunicationChannel.instance.send(p261); //===============================
            CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.time_boot_ms = (uint)2230745547U;
            p262.image_status = (byte)(byte)227;
            p262.video_status = (byte)(byte)243;
            p262.image_interval = (float)1.6228713E37F;
            p262.recording_time_ms = (uint)1353076748U;
            p262.available_capacity = (float) -2.4107226E38F;
            CommunicationChannel.instance.send(p262); //===============================
            CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.time_boot_ms = (uint)3232653230U;
            p263.time_utc = (ulong)7664118208897253113L;
            p263.camera_id = (byte)(byte)197;
            p263.lat = (int) -857050457;
            p263.lon = (int) -2021954391;
            p263.alt = (int) -1415392350;
            p263.relative_alt = (int) -1035691885;
            p263.q_SET(new float[4], 0);
            p263.image_index = (int)1266255010;
            p263.capture_result = (sbyte)(sbyte)53;
            p263.file_url_SET("DEMO", PH);
            CommunicationChannel.instance.send(p263); //===============================
            FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.time_boot_ms = (uint)2696356102U;
            p264.arming_time_utc = (ulong)3500410580867292464L;
            p264.takeoff_time_utc = (ulong)4296233987078009678L;
            p264.flight_uuid = (ulong)5284477208069331590L;
            CommunicationChannel.instance.send(p264); //===============================
            MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.time_boot_ms = (uint)1229857180U;
            p265.roll = (float)1.5242718E38F;
            p265.pitch = (float)2.9264073E38F;
            p265.yaw = (float) -1.6460624E38F;
            CommunicationChannel.instance.send(p265); //===============================
            LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.target_system = (byte)(byte)39;
            p266.target_component = (byte)(byte)203;
            p266.sequence = (ushort)(ushort)11332;
            p266.length = (byte)(byte)48;
            p266.first_message_offset = (byte)(byte)214;
            p266.data__SET(new byte[249], 0);
            CommunicationChannel.instance.send(p266); //===============================
            LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.target_system = (byte)(byte)29;
            p267.target_component = (byte)(byte)108;
            p267.sequence = (ushort)(ushort)39684;
            p267.length = (byte)(byte)63;
            p267.first_message_offset = (byte)(byte)149;
            p267.data__SET(new byte[249], 0);
            CommunicationChannel.instance.send(p267); //===============================
            LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_system = (byte)(byte)233;
            p268.target_component = (byte)(byte)234;
            p268.sequence = (ushort)(ushort)35841;
            CommunicationChannel.instance.send(p268); //===============================
            VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.camera_id = (byte)(byte)43;
            p269.status = (byte)(byte)150;
            p269.framerate = (float) -1.4466003E38F;
            p269.resolution_h = (ushort)(ushort)41081;
            p269.resolution_v = (ushort)(ushort)1373;
            p269.bitrate = (uint)2567101694U;
            p269.rotation = (ushort)(ushort)1199;
            p269.uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p269); //===============================
            SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.target_system = (byte)(byte)25;
            p270.target_component = (byte)(byte)235;
            p270.camera_id = (byte)(byte)24;
            p270.framerate = (float) -1.6062487E38F;
            p270.resolution_h = (ushort)(ushort)29759;
            p270.resolution_v = (ushort)(ushort)19284;
            p270.bitrate = (uint)503685924U;
            p270.rotation = (ushort)(ushort)40889;
            p270.uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p270); //===============================
            WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.ssid_SET("DEMO", PH);
            p299.password_SET("DEMO", PH);
            CommunicationChannel.instance.send(p299); //===============================
            PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.version = (ushort)(ushort)2113;
            p300.min_version = (ushort)(ushort)26784;
            p300.max_version = (ushort)(ushort)51643;
            p300.spec_version_hash_SET(new byte[8], 0);
            p300.library_version_hash_SET(new byte[8], 0);
            CommunicationChannel.instance.send(p300); //===============================
            UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.time_usec = (ulong)705144546655738938L;
            p310.uptime_sec = (uint)3247277016U;
            p310.health = (UAVCAN_NODE_HEALTH)UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK;
            p310.mode = (UAVCAN_NODE_MODE)UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE;
            p310.sub_mode = (byte)(byte)221;
            p310.vendor_specific_status_code = (ushort)(ushort)55426;
            CommunicationChannel.instance.send(p310); //===============================
            UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.time_usec = (ulong)8228695585212700181L;
            p311.uptime_sec = (uint)865293679U;
            p311.name_SET("DEMO", PH);
            p311.hw_version_major = (byte)(byte)26;
            p311.hw_version_minor = (byte)(byte)83;
            p311.hw_unique_id_SET(new byte[16], 0);
            p311.sw_version_major = (byte)(byte)242;
            p311.sw_version_minor = (byte)(byte)12;
            p311.sw_vcs_commit = (uint)3311091945U;
            CommunicationChannel.instance.send(p311); //===============================
            PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.target_system = (byte)(byte)220;
            p320.target_component = (byte)(byte)41;
            p320.param_id_SET("DEMO", PH);
            p320.param_index = (short)(short)6687;
            CommunicationChannel.instance.send(p320); //===============================
            PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_system = (byte)(byte)3;
            p321.target_component = (byte)(byte)39;
            CommunicationChannel.instance.send(p321); //===============================
            PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_id_SET("DEMO", PH);
            p322.param_value_SET("DEMO", PH);
            p322.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16;
            p322.param_count = (ushort)(ushort)49778;
            p322.param_index = (ushort)(ushort)30559;
            CommunicationChannel.instance.send(p322); //===============================
            PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.target_system = (byte)(byte)54;
            p323.target_component = (byte)(byte)195;
            p323.param_id_SET("DEMO", PH);
            p323.param_value_SET("DEMO", PH);
            p323.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64;
            CommunicationChannel.instance.send(p323); //===============================
            PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_id_SET("DEMO", PH);
            p324.param_value_SET("DEMO", PH);
            p324.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM;
            p324.param_result = (PARAM_ACK)PARAM_ACK.PARAM_ACK_FAILED;
            CommunicationChannel.instance.send(p324); //===============================
            OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.time_usec = (ulong)6218204014238391791L;
            p330.sensor_type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND;
            p330.distances_SET(new ushort[72], 0);
            p330.increment = (byte)(byte)151;
            p330.min_distance = (ushort)(ushort)1943;
            p330.max_distance = (ushort)(ushort)42753;
            CommunicationChannel.instance.send(p330); //===============================
        }
    }
}


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
            CommunicationChannel.instance.OnAUTOPILOT_VERSIONReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnLANDING_TARGETReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnESTIMATOR_STATUSReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnWIND_COVReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnGPS_INPUTReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnGPS_RTCM_DATAReceive += (src, ph, pack) =>
            {
                byte flags = pack.flags;
                byte len = pack.len;
                byte[] data_ = pack.data_;
            };
            CommunicationChannel.instance.OnHIGH_LATENCYReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnVIBRATIONReceive += (src, ph, pack) =>
            {
                ulong time_usec = pack.time_usec;
                float vibration_x = pack.vibration_x;
                float vibration_y = pack.vibration_y;
                float vibration_z = pack.vibration_z;
                uint clipping_0 = pack.clipping_0;
                uint clipping_1 = pack.clipping_1;
                uint clipping_2 = pack.clipping_2;
            };
            CommunicationChannel.instance.OnHOME_POSITIONReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnSET_HOME_POSITIONReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnMESSAGE_INTERVALReceive += (src, ph, pack) =>
            {
                ushort message_id = pack.message_id;
                int interval_us = pack.interval_us;
            };
            CommunicationChannel.instance.OnEXTENDED_SYS_STATEReceive += (src, ph, pack) =>
            {
                MAV_VTOL_STATE vtol_state = pack.vtol_state;
                MAV_LANDED_STATE landed_state = pack.landed_state;
            };
            CommunicationChannel.instance.OnADSB_VEHICLEReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnCOLLISIONReceive += (src, ph, pack) =>
            {
                MAV_COLLISION_SRC src_ = pack.src_;
                uint id = pack.id;
                MAV_COLLISION_ACTION action = pack.action;
                MAV_COLLISION_THREAT_LEVEL threat_level = pack.threat_level;
                float time_to_minimum_delta = pack.time_to_minimum_delta;
                float altitude_minimum_delta = pack.altitude_minimum_delta;
                float horizontal_minimum_delta = pack.horizontal_minimum_delta;
            };
            CommunicationChannel.instance.OnV2_EXTENSIONReceive += (src, ph, pack) =>
            {
                byte target_network = pack.target_network;
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                ushort message_type = pack.message_type;
                byte[] payload = pack.payload;
            };
            CommunicationChannel.instance.OnMEMORY_VECTReceive += (src, ph, pack) =>
            {
                ushort address = pack.address;
                byte ver = pack.ver;
                byte type = pack.type;
                sbyte[] value = pack.value;
            };
            CommunicationChannel.instance.OnDEBUG_VECTReceive += (src, ph, pack) =>
            {
                string name = pack.name_TRY(ph);
                ulong time_usec = pack.time_usec;
                float x = pack.x;
                float y = pack.y;
                float z = pack.z;
            };
            CommunicationChannel.instance.OnNAMED_VALUE_FLOATReceive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                string name = pack.name_TRY(ph);
                float value = pack.value;
            };
            CommunicationChannel.instance.OnNAMED_VALUE_INTReceive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                string name = pack.name_TRY(ph);
                int value = pack.value;
            };
            CommunicationChannel.instance.OnSTATUSTEXTReceive += (src, ph, pack) =>
            {
                MAV_SEVERITY severity = pack.severity;
                string text = pack.text_TRY(ph);
            };
            CommunicationChannel.instance.OnDEBUGReceive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                byte ind = pack.ind;
                float value = pack.value;
            };
            CommunicationChannel.instance.OnSETUP_SIGNINGReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                byte[] secret_key = pack.secret_key;
                ulong initial_timestamp = pack.initial_timestamp;
            };
            CommunicationChannel.instance.OnBUTTON_CHANGEReceive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                uint last_change_ms = pack.last_change_ms;
                byte state = pack.state;
            };
            CommunicationChannel.instance.OnPLAY_TUNEReceive += (src, ph, pack) =>
            {
                byte target_system = pack.target_system;
                byte target_component = pack.target_component;
                string tune = pack.tune_TRY(ph);
            };
            CommunicationChannel.instance.OnCAMERA_INFORMATIONReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnCAMERA_SETTINGSReceive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                CAMERA_MODE mode_id = pack.mode_id;
            };
            CommunicationChannel.instance.OnSTORAGE_INFORMATIONReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnCAMERA_CAPTURE_STATUSReceive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                byte image_status = pack.image_status;
                byte video_status = pack.video_status;
                float image_interval = pack.image_interval;
                uint recording_time_ms = pack.recording_time_ms;
                float available_capacity = pack.available_capacity;
            };
            CommunicationChannel.instance.OnCAMERA_IMAGE_CAPTUREDReceive += (src, ph, pack) =>
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
            CommunicationChannel.instance.OnFLIGHT_INFORMATIONReceive += (src, ph, pack) =>
            {
                uint time_boot_ms = pack.time_boot_ms;
                ulong arming_time_utc = pack.arming_time_utc;
                ulong takeoff_time_utc = pack.takeoff_time_utc;
                ulong flight_uuid = pack.flight_uuid;
            };
            VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.usec = (ulong)2294303341299410205L;
            p103.x = (float) -1.5711579E38F;
            p103.y = (float) -3.261716E38F;
            p103.z = (float)2.599702E38F;
            CommunicationChannel.instance.send(p103); //===============================
            VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.usec = (ulong)6318242730081291015L;
            p104.x = (float) -5.121353E37F;
            p104.y = (float) -2.9232858E38F;
            p104.z = (float) -2.9508476E38F;
            p104.roll = (float)1.879225E38F;
            p104.pitch = (float)1.259525E37F;
            p104.yaw = (float)1.9783824E37F;
            CommunicationChannel.instance.send(p104); //===============================
            HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.time_usec = (ulong)4597406820968203089L;
            p105.xacc = (float) -3.1263728E38F;
            p105.yacc = (float)1.2832624E38F;
            p105.zacc = (float) -3.257041E38F;
            p105.xgyro = (float)1.859818E38F;
            p105.ygyro = (float) -2.510312E38F;
            p105.zgyro = (float) -1.9529512E38F;
            p105.xmag = (float) -1.8742735E38F;
            p105.ymag = (float) -8.2966576E37F;
            p105.zmag = (float)2.8161215E38F;
            p105.abs_pressure = (float)4.892124E37F;
            p105.diff_pressure = (float)2.4678086E38F;
            p105.pressure_alt = (float) -2.8031755E37F;
            p105.temperature = (float) -3.2714448E38F;
            p105.fields_updated = (ushort)(ushort)17721;
            CommunicationChannel.instance.send(p105); //===============================
            OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.time_usec = (ulong)8206740187361650987L;
            p106.sensor_id = (byte)(byte)197;
            p106.integration_time_us = (uint)2010411961U;
            p106.integrated_x = (float) -9.338276E37F;
            p106.integrated_y = (float) -1.9511139E38F;
            p106.integrated_xgyro = (float) -1.8431966E38F;
            p106.integrated_ygyro = (float)2.687926E36F;
            p106.integrated_zgyro = (float) -4.4210816E37F;
            p106.temperature = (short)(short) -24333;
            p106.quality = (byte)(byte)157;
            p106.time_delta_distance_us = (uint)2665636243U;
            p106.distance = (float) -1.987668E38F;
            CommunicationChannel.instance.send(p106); //===============================
            HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.time_usec = (ulong)6234972507341134929L;
            p107.xacc = (float)1.7135594E38F;
            p107.yacc = (float) -1.6892532E38F;
            p107.zacc = (float)1.6531407E38F;
            p107.xgyro = (float)5.681297E37F;
            p107.ygyro = (float) -1.6486802E38F;
            p107.zgyro = (float)1.1548786E38F;
            p107.xmag = (float)9.863921E37F;
            p107.ymag = (float)3.4188058E37F;
            p107.zmag = (float)1.530959E38F;
            p107.abs_pressure = (float) -1.9150643E38F;
            p107.diff_pressure = (float)2.9847957E37F;
            p107.pressure_alt = (float)2.349327E38F;
            p107.temperature = (float)1.4955396E38F;
            p107.fields_updated = (uint)4173066272U;
            CommunicationChannel.instance.send(p107); //===============================
            SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.q1 = (float)2.299149E38F;
            p108.q2 = (float)1.9023783E38F;
            p108.q3 = (float)2.178351E38F;
            p108.q4 = (float)1.6289061E38F;
            p108.roll = (float)1.4578873E38F;
            p108.pitch = (float)1.4262633E38F;
            p108.yaw = (float) -3.0162578E38F;
            p108.xacc = (float) -1.6570871E38F;
            p108.yacc = (float)9.77105E37F;
            p108.zacc = (float)1.8472894E38F;
            p108.xgyro = (float) -4.481005E37F;
            p108.ygyro = (float) -7.8273784E37F;
            p108.zgyro = (float) -6.998892E37F;
            p108.lat = (float)1.9226465E38F;
            p108.lon = (float) -2.5466587E38F;
            p108.alt = (float)7.3075286E37F;
            p108.std_dev_horz = (float)3.815789E37F;
            p108.std_dev_vert = (float)1.5801314E38F;
            p108.vn = (float)1.1318201E37F;
            p108.ve = (float) -8.551549E37F;
            p108.vd = (float)3.8370475E37F;
            CommunicationChannel.instance.send(p108); //===============================
            RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.rssi = (byte)(byte)254;
            p109.remrssi = (byte)(byte)233;
            p109.txbuf = (byte)(byte)137;
            p109.noise = (byte)(byte)99;
            p109.remnoise = (byte)(byte)58;
            p109.rxerrors = (ushort)(ushort)49449;
            p109.fixed_ = (ushort)(ushort)33473;
            CommunicationChannel.instance.send(p109); //===============================
            FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_network = (byte)(byte)35;
            p110.target_system = (byte)(byte)184;
            p110.target_component = (byte)(byte)195;
            p110.payload_SET(new byte[251], 0);
            CommunicationChannel.instance.send(p110); //===============================
            TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.tc1 = (long) -8534652278618334717L;
            p111.ts1 = (long) -2566397989468578036L;
            CommunicationChannel.instance.send(p111); //===============================
            CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.time_usec = (ulong)8141527123209308554L;
            p112.seq = (uint)2466773646U;
            CommunicationChannel.instance.send(p112); //===============================
            HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.time_usec = (ulong)1234515661401464975L;
            p113.fix_type = (byte)(byte)207;
            p113.lat = (int)84543451;
            p113.lon = (int)158195275;
            p113.alt = (int) -728302454;
            p113.eph = (ushort)(ushort)11150;
            p113.epv = (ushort)(ushort)15170;
            p113.vel = (ushort)(ushort)34820;
            p113.vn = (short)(short) -26080;
            p113.ve = (short)(short)21819;
            p113.vd = (short)(short) -32436;
            p113.cog = (ushort)(ushort)18154;
            p113.satellites_visible = (byte)(byte)131;
            CommunicationChannel.instance.send(p113); //===============================
            HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.time_usec = (ulong)4019680951073322541L;
            p114.sensor_id = (byte)(byte)191;
            p114.integration_time_us = (uint)2414983095U;
            p114.integrated_x = (float) -3.060113E38F;
            p114.integrated_y = (float)1.9524793E38F;
            p114.integrated_xgyro = (float) -7.9566367E37F;
            p114.integrated_ygyro = (float)8.772624E37F;
            p114.integrated_zgyro = (float)1.8585431E38F;
            p114.temperature = (short)(short) -26282;
            p114.quality = (byte)(byte)237;
            p114.time_delta_distance_us = (uint)3586797999U;
            p114.distance = (float)1.4713957E38F;
            CommunicationChannel.instance.send(p114); //===============================
            HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.time_usec = (ulong)4083073646227263905L;
            p115.attitude_quaternion_SET(new float[4], 0);
            p115.rollspeed = (float) -7.5808705E37F;
            p115.pitchspeed = (float) -2.4703525E38F;
            p115.yawspeed = (float) -2.0476993E38F;
            p115.lat = (int) -1672814387;
            p115.lon = (int) -1794938282;
            p115.alt = (int) -1279065324;
            p115.vx = (short)(short)25465;
            p115.vy = (short)(short) -31843;
            p115.vz = (short)(short) -17677;
            p115.ind_airspeed = (ushort)(ushort)7854;
            p115.true_airspeed = (ushort)(ushort)59857;
            p115.xacc = (short)(short)5414;
            p115.yacc = (short)(short)1646;
            p115.zacc = (short)(short) -24629;
            CommunicationChannel.instance.send(p115); //===============================
            SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.time_boot_ms = (uint)2265803620U;
            p116.xacc = (short)(short)4676;
            p116.yacc = (short)(short) -21220;
            p116.zacc = (short)(short) -23802;
            p116.xgyro = (short)(short) -10043;
            p116.ygyro = (short)(short) -12762;
            p116.zgyro = (short)(short)2770;
            p116.xmag = (short)(short) -1565;
            p116.ymag = (short)(short)9522;
            p116.zmag = (short)(short) -24015;
            CommunicationChannel.instance.send(p116); //===============================
            LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.target_system = (byte)(byte)172;
            p117.target_component = (byte)(byte)110;
            p117.start = (ushort)(ushort)24965;
            p117.end = (ushort)(ushort)9829;
            CommunicationChannel.instance.send(p117); //===============================
            LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.id = (ushort)(ushort)43957;
            p118.num_logs = (ushort)(ushort)32332;
            p118.last_log_num = (ushort)(ushort)53257;
            p118.time_utc = (uint)327163764U;
            p118.size = (uint)1857608217U;
            CommunicationChannel.instance.send(p118); //===============================
            LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.target_system = (byte)(byte)225;
            p119.target_component = (byte)(byte)6;
            p119.id = (ushort)(ushort)25229;
            p119.ofs = (uint)514892003U;
            p119.count = (uint)2230354813U;
            CommunicationChannel.instance.send(p119); //===============================
            LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.id = (ushort)(ushort)2116;
            p120.ofs = (uint)273034127U;
            p120.count = (byte)(byte)58;
            p120.data__SET(new byte[90], 0);
            CommunicationChannel.instance.send(p120); //===============================
            LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_system = (byte)(byte)249;
            p121.target_component = (byte)(byte)220;
            CommunicationChannel.instance.send(p121); //===============================
            LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_system = (byte)(byte)73;
            p122.target_component = (byte)(byte)100;
            CommunicationChannel.instance.send(p122); //===============================
            GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.target_system = (byte)(byte)116;
            p123.target_component = (byte)(byte)77;
            p123.len = (byte)(byte)107;
            p123.data__SET(new byte[110], 0);
            CommunicationChannel.instance.send(p123); //===============================
            GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.time_usec = (ulong)334249182925212966L;
            p124.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX;
            p124.lat = (int)2018002686;
            p124.lon = (int) -582579414;
            p124.alt = (int)871240761;
            p124.eph = (ushort)(ushort)53283;
            p124.epv = (ushort)(ushort)43085;
            p124.vel = (ushort)(ushort)4223;
            p124.cog = (ushort)(ushort)65256;
            p124.satellites_visible = (byte)(byte)126;
            p124.dgps_numch = (byte)(byte)16;
            p124.dgps_age = (uint)1457458491U;
            CommunicationChannel.instance.send(p124); //===============================
            POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vcc = (ushort)(ushort)63154;
            p125.Vservo = (ushort)(ushort)22298;
            p125.flags = (MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID);
            CommunicationChannel.instance.send(p125); //===============================
            SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.device = SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS2;
            p126.flags = (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE);
            p126.timeout = (ushort)(ushort)53417;
            p126.baudrate = (uint)3507343200U;
            p126.count = (byte)(byte)75;
            p126.data__SET(new byte[70], 0);
            CommunicationChannel.instance.send(p126); //===============================
            GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.time_last_baseline_ms = (uint)838716718U;
            p127.rtk_receiver_id = (byte)(byte)63;
            p127.wn = (ushort)(ushort)43298;
            p127.tow = (uint)2605045574U;
            p127.rtk_health = (byte)(byte)84;
            p127.rtk_rate = (byte)(byte)212;
            p127.nsats = (byte)(byte)172;
            p127.baseline_coords_type = (byte)(byte)92;
            p127.baseline_a_mm = (int)200043085;
            p127.baseline_b_mm = (int)1782893613;
            p127.baseline_c_mm = (int)1567773118;
            p127.accuracy = (uint)3794299827U;
            p127.iar_num_hypotheses = (int) -1808815181;
            CommunicationChannel.instance.send(p127); //===============================
            GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.time_last_baseline_ms = (uint)1088547480U;
            p128.rtk_receiver_id = (byte)(byte)226;
            p128.wn = (ushort)(ushort)53248;
            p128.tow = (uint)1591673648U;
            p128.rtk_health = (byte)(byte)202;
            p128.rtk_rate = (byte)(byte)231;
            p128.nsats = (byte)(byte)61;
            p128.baseline_coords_type = (byte)(byte)197;
            p128.baseline_a_mm = (int)1833538005;
            p128.baseline_b_mm = (int) -401623435;
            p128.baseline_c_mm = (int)270972790;
            p128.accuracy = (uint)1761196411U;
            p128.iar_num_hypotheses = (int)991938350;
            CommunicationChannel.instance.send(p128); //===============================
            SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.time_boot_ms = (uint)1812242411U;
            p129.xacc = (short)(short)22076;
            p129.yacc = (short)(short) -5254;
            p129.zacc = (short)(short) -28949;
            p129.xgyro = (short)(short)29928;
            p129.ygyro = (short)(short)17537;
            p129.zgyro = (short)(short) -18938;
            p129.xmag = (short)(short) -14703;
            p129.ymag = (short)(short)840;
            p129.zmag = (short)(short) -18039;
            CommunicationChannel.instance.send(p129); //===============================
            DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.type = (byte)(byte)54;
            p130.size = (uint)3297077867U;
            p130.width = (ushort)(ushort)51387;
            p130.height = (ushort)(ushort)61587;
            p130.packets = (ushort)(ushort)11605;
            p130.payload = (byte)(byte)39;
            p130.jpg_quality = (byte)(byte)175;
            CommunicationChannel.instance.send(p130); //===============================
            ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.seqnr = (ushort)(ushort)11783;
            p131.data__SET(new byte[253], 0);
            CommunicationChannel.instance.send(p131); //===============================
            DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.time_boot_ms = (uint)2401141674U;
            p132.min_distance = (ushort)(ushort)42420;
            p132.max_distance = (ushort)(ushort)4504;
            p132.current_distance = (ushort)(ushort)55513;
            p132.type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER;
            p132.id = (byte)(byte)96;
            p132.orientation = MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_PITCH_90;
            p132.covariance = (byte)(byte)228;
            CommunicationChannel.instance.send(p132); //===============================
            TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.lat = (int) -139257105;
            p133.lon = (int)289772026;
            p133.grid_spacing = (ushort)(ushort)53600;
            p133.mask = (ulong)1004736302721010157L;
            CommunicationChannel.instance.send(p133); //===============================
            TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.lat = (int)2046405164;
            p134.lon = (int) -940573730;
            p134.grid_spacing = (ushort)(ushort)25064;
            p134.gridbit = (byte)(byte)72;
            p134.data__SET(new short[16], 0);
            CommunicationChannel.instance.send(p134); //===============================
            TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lat = (int) -826161953;
            p135.lon = (int) -167524082;
            CommunicationChannel.instance.send(p135); //===============================
            TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.lat = (int)1975401274;
            p136.lon = (int)199919566;
            p136.spacing = (ushort)(ushort)35216;
            p136.terrain_height = (float)1.0507114E38F;
            p136.current_height = (float)2.2587812E38F;
            p136.pending = (ushort)(ushort)15722;
            p136.loaded = (ushort)(ushort)41804;
            CommunicationChannel.instance.send(p136); //===============================
            SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.time_boot_ms = (uint)2088034905U;
            p137.press_abs = (float) -2.662939E38F;
            p137.press_diff = (float)1.4339366E36F;
            p137.temperature = (short)(short)6880;
            CommunicationChannel.instance.send(p137); //===============================
            ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.time_usec = (ulong)8363930805741609830L;
            p138.q_SET(new float[4], 0);
            p138.x = (float) -2.7087012E38F;
            p138.y = (float)1.294032E38F;
            p138.z = (float)1.1409144E38F;
            CommunicationChannel.instance.send(p138); //===============================
            SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.time_usec = (ulong)3055055299424831978L;
            p139.group_mlx = (byte)(byte)107;
            p139.target_system = (byte)(byte)19;
            p139.target_component = (byte)(byte)77;
            p139.controls_SET(new float[8], 0);
            CommunicationChannel.instance.send(p139); //===============================
            ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.time_usec = (ulong)928906082536813506L;
            p140.group_mlx = (byte)(byte)165;
            p140.controls_SET(new float[8], 0);
            CommunicationChannel.instance.send(p140); //===============================
            ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.time_usec = (ulong)7893221261124292119L;
            p141.altitude_monotonic = (float)1.4510772E38F;
            p141.altitude_amsl = (float) -9.998288E37F;
            p141.altitude_local = (float)3.2202905E38F;
            p141.altitude_relative = (float)2.9772423E38F;
            p141.altitude_terrain = (float)4.3529214E36F;
            p141.bottom_clearance = (float)1.4730335E38F;
            CommunicationChannel.instance.send(p141); //===============================
            RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.request_id = (byte)(byte)9;
            p142.uri_type = (byte)(byte)180;
            p142.uri_SET(new byte[120], 0);
            p142.transfer_type = (byte)(byte)14;
            p142.storage_SET(new byte[120], 0);
            CommunicationChannel.instance.send(p142); //===============================
            SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.time_boot_ms = (uint)2118451050U;
            p143.press_abs = (float)1.8222623E38F;
            p143.press_diff = (float) -2.3522581E38F;
            p143.temperature = (short)(short)30201;
            CommunicationChannel.instance.send(p143); //===============================
            FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.timestamp = (ulong)9163309302510232577L;
            p144.est_capabilities = (byte)(byte)35;
            p144.lat = (int)335117155;
            p144.lon = (int)1949358633;
            p144.alt = (float) -3.8000588E37F;
            p144.vel_SET(new float[3], 0);
            p144.acc_SET(new float[3], 0);
            p144.attitude_q_SET(new float[4], 0);
            p144.rates_SET(new float[3], 0);
            p144.position_cov_SET(new float[3], 0);
            p144.custom_state = (ulong)7494121882720589586L;
            CommunicationChannel.instance.send(p144); //===============================
            CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.time_usec = (ulong)4499144375025927513L;
            p146.x_acc = (float)3.1263846E38F;
            p146.y_acc = (float)1.4461176E38F;
            p146.z_acc = (float) -1.6648042E38F;
            p146.x_vel = (float)2.1831353E38F;
            p146.y_vel = (float)1.536919E38F;
            p146.z_vel = (float) -2.2833697E38F;
            p146.x_pos = (float) -2.0439085E38F;
            p146.y_pos = (float) -2.8850156E38F;
            p146.z_pos = (float) -3.032425E38F;
            p146.airspeed = (float) -8.0307724E37F;
            p146.vel_variance_SET(new float[3], 0);
            p146.pos_variance_SET(new float[3], 0);
            p146.q_SET(new float[4], 0);
            p146.roll_rate = (float)1.2951903E38F;
            p146.pitch_rate = (float) -2.007182E38F;
            p146.yaw_rate = (float) -3.2166099E38F;
            CommunicationChannel.instance.send(p146); //===============================
            BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.id = (byte)(byte)153;
            p147.battery_function = MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_PROPULSION;
            p147.type = MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIPO;
            p147.temperature = (short)(short)16840;
            p147.voltages_SET(new ushort[10], 0);
            p147.current_battery = (short)(short) -14022;
            p147.current_consumed = (int) -1753951793;
            p147.energy_consumed = (int)416475903;
            p147.battery_remaining = (sbyte)(sbyte)18;
            CommunicationChannel.instance.send(p147); //===============================
            AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_INT);
            p148.flight_sw_version = (uint)3793250775U;
            p148.middleware_sw_version = (uint)455448911U;
            p148.os_sw_version = (uint)2930038980U;
            p148.board_version = (uint)3237918433U;
            p148.flight_custom_version_SET(new byte[8], 0);
            p148.middleware_custom_version_SET(new byte[8], 0);
            p148.os_custom_version_SET(new byte[8], 0);
            p148.vendor_id = (ushort)(ushort)48912;
            p148.product_id = (ushort)(ushort)27214;
            p148.uid = (ulong)1978321499688424955L;
            p148.uid2_SET(new byte[18], 0, PH);
            CommunicationChannel.instance.send(p148); //===============================
            LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.time_usec = (ulong)1643819777527817976L;
            p149.target_num = (byte)(byte)160;
            p149.frame = MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
            p149.angle_x = (float)1.9976954E38F;
            p149.angle_y = (float)4.773237E37F;
            p149.distance = (float) -1.3702636E38F;
            p149.size_x = (float) -2.6896473E38F;
            p149.size_y = (float) -1.2855635E38F;
            p149.x_SET((float) -2.1039217E38F, PH);
            p149.y_SET((float)9.447278E37F, PH);
            p149.z_SET((float)1.6594616E37F, PH);
            p149.q_SET(new float[4], 0, PH);
            p149.type = LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER;
            p149.position_valid_SET((byte)(byte)140, PH);
            CommunicationChannel.instance.send(p149); //===============================
            SENSOR_OFFSETS p150 = CommunicationChannel.new_SENSOR_OFFSETS();
            PH.setPack(p150);
            p150.mag_ofs_x = (short)(short) -10575;
            p150.mag_ofs_y = (short)(short) -21335;
            p150.mag_ofs_z = (short)(short)19588;
            p150.mag_declination = (float) -3.0883628E38F;
            p150.raw_press = (int) -2050915891;
            p150.raw_temp = (int)2084146341;
            p150.gyro_cal_x = (float)3.163729E38F;
            p150.gyro_cal_y = (float) -8.783625E37F;
            p150.gyro_cal_z = (float)2.137225E38F;
            p150.accel_cal_x = (float) -7.53122E37F;
            p150.accel_cal_y = (float)3.223556E38F;
            p150.accel_cal_z = (float) -2.6786429E38F;
            CommunicationChannel.instance.send(p150); //===============================
            SET_MAG_OFFSETS p151 = CommunicationChannel.new_SET_MAG_OFFSETS();
            PH.setPack(p151);
            p151.target_system = (byte)(byte)68;
            p151.target_component = (byte)(byte)248;
            p151.mag_ofs_x = (short)(short)22569;
            p151.mag_ofs_y = (short)(short)31418;
            p151.mag_ofs_z = (short)(short)28072;
            CommunicationChannel.instance.send(p151); //===============================
            MEMINFO p152 = CommunicationChannel.new_MEMINFO();
            PH.setPack(p152);
            p152.brkval = (ushort)(ushort)49237;
            p152.freemem = (ushort)(ushort)41382;
            p152.freemem32_SET((uint)1641272551U, PH);
            CommunicationChannel.instance.send(p152); //===============================
            AP_ADC p153 = CommunicationChannel.new_AP_ADC();
            PH.setPack(p153);
            p153.adc1 = (ushort)(ushort)31401;
            p153.adc2 = (ushort)(ushort)55007;
            p153.adc3 = (ushort)(ushort)50553;
            p153.adc4 = (ushort)(ushort)45563;
            p153.adc5 = (ushort)(ushort)7830;
            p153.adc6 = (ushort)(ushort)51228;
            CommunicationChannel.instance.send(p153); //===============================
            DIGICAM_CONFIGURE p154 = CommunicationChannel.new_DIGICAM_CONFIGURE();
            PH.setPack(p154);
            p154.target_system = (byte)(byte)162;
            p154.target_component = (byte)(byte)212;
            p154.mode = (byte)(byte)90;
            p154.shutter_speed = (ushort)(ushort)57492;
            p154.aperture = (byte)(byte)145;
            p154.iso = (byte)(byte)114;
            p154.exposure_type = (byte)(byte)150;
            p154.command_id = (byte)(byte)43;
            p154.engine_cut_off = (byte)(byte)133;
            p154.extra_param = (byte)(byte)93;
            p154.extra_value = (float) -9.049945E37F;
            CommunicationChannel.instance.send(p154); //===============================
            DIGICAM_CONTROL p155 = CommunicationChannel.new_DIGICAM_CONTROL();
            PH.setPack(p155);
            p155.target_system = (byte)(byte)45;
            p155.target_component = (byte)(byte)117;
            p155.session = (byte)(byte)191;
            p155.zoom_pos = (byte)(byte)2;
            p155.zoom_step = (sbyte)(sbyte) - 120;
            p155.focus_lock = (byte)(byte)163;
            p155.shot = (byte)(byte)58;
            p155.command_id = (byte)(byte)118;
            p155.extra_param = (byte)(byte)178;
            p155.extra_value = (float)5.145225E37F;
            CommunicationChannel.instance.send(p155); //===============================
            MOUNT_CONFIGURE p156 = CommunicationChannel.new_MOUNT_CONFIGURE();
            PH.setPack(p156);
            p156.target_system = (byte)(byte)206;
            p156.target_component = (byte)(byte)120;
            p156.mount_mode = MAV_MOUNT_MODE.MAV_MOUNT_MODE_MAVLINK_TARGETING;
            p156.stab_roll = (byte)(byte)43;
            p156.stab_pitch = (byte)(byte)64;
            p156.stab_yaw = (byte)(byte)216;
            CommunicationChannel.instance.send(p156); //===============================
            MOUNT_CONTROL p157 = CommunicationChannel.new_MOUNT_CONTROL();
            PH.setPack(p157);
            p157.target_system = (byte)(byte)108;
            p157.target_component = (byte)(byte)109;
            p157.input_a = (int)1161470894;
            p157.input_b = (int) -334632231;
            p157.input_c = (int) -1525751370;
            p157.save_position = (byte)(byte)204;
            CommunicationChannel.instance.send(p157); //===============================
            MOUNT_STATUS p158 = CommunicationChannel.new_MOUNT_STATUS();
            PH.setPack(p158);
            p158.target_system = (byte)(byte)65;
            p158.target_component = (byte)(byte)227;
            p158.pointing_a = (int) -441569150;
            p158.pointing_b = (int) -1919217028;
            p158.pointing_c = (int)1159072774;
            CommunicationChannel.instance.send(p158); //===============================
            FENCE_POINT p160 = CommunicationChannel.new_FENCE_POINT();
            PH.setPack(p160);
            p160.target_system = (byte)(byte)244;
            p160.target_component = (byte)(byte)68;
            p160.idx = (byte)(byte)154;
            p160.count = (byte)(byte)25;
            p160.lat = (float)3.0934474E38F;
            p160.lng = (float)2.27587E38F;
            CommunicationChannel.instance.send(p160); //===============================
            FENCE_FETCH_POINT p161 = CommunicationChannel.new_FENCE_FETCH_POINT();
            PH.setPack(p161);
            p161.target_system = (byte)(byte)21;
            p161.target_component = (byte)(byte)166;
            p161.idx = (byte)(byte)251;
            CommunicationChannel.instance.send(p161); //===============================
            FENCE_STATUS p162 = CommunicationChannel.new_FENCE_STATUS();
            PH.setPack(p162);
            p162.breach_status = (byte)(byte)239;
            p162.breach_count = (ushort)(ushort)45638;
            p162.breach_type = FENCE_BREACH.FENCE_BREACH_BOUNDARY;
            p162.breach_time = (uint)3645111042U;
            CommunicationChannel.instance.send(p162); //===============================
            AHRS p163 = CommunicationChannel.new_AHRS();
            PH.setPack(p163);
            p163.omegaIx = (float) -3.147289E38F;
            p163.omegaIy = (float)2.3504078E38F;
            p163.omegaIz = (float) -1.3869494E38F;
            p163.accel_weight = (float)3.3857238E38F;
            p163.renorm_val = (float)2.2717615E38F;
            p163.error_rp = (float) -1.5813292E38F;
            p163.error_yaw = (float)2.9990603E38F;
            CommunicationChannel.instance.send(p163); //===============================
            SIMSTATE p164 = CommunicationChannel.new_SIMSTATE();
            PH.setPack(p164);
            p164.roll = (float)6.9112965E37F;
            p164.pitch = (float) -1.2360751E38F;
            p164.yaw = (float)2.0883566E38F;
            p164.xacc = (float)2.3756156E38F;
            p164.yacc = (float) -1.3918894E38F;
            p164.zacc = (float) -1.0726734E38F;
            p164.xgyro = (float)2.7793367E38F;
            p164.ygyro = (float)2.4543626E38F;
            p164.zgyro = (float)7.4022276E37F;
            p164.lat = (int) -1689656463;
            p164.lng = (int) -322596829;
            CommunicationChannel.instance.send(p164); //===============================
            HWSTATUS p165 = CommunicationChannel.new_HWSTATUS();
            PH.setPack(p165);
            p165.Vcc = (ushort)(ushort)54766;
            p165.I2Cerr = (byte)(byte)121;
            CommunicationChannel.instance.send(p165); //===============================
            RADIO p166 = CommunicationChannel.new_RADIO();
            PH.setPack(p166);
            p166.rssi = (byte)(byte)200;
            p166.remrssi = (byte)(byte)139;
            p166.txbuf = (byte)(byte)93;
            p166.noise = (byte)(byte)80;
            p166.remnoise = (byte)(byte)216;
            p166.rxerrors = (ushort)(ushort)47036;
            p166.fixed_ = (ushort)(ushort)3010;
            CommunicationChannel.instance.send(p166); //===============================
            LIMITS_STATUS p167 = CommunicationChannel.new_LIMITS_STATUS();
            PH.setPack(p167);
            p167.limits_state = LIMITS_STATE.LIMITS_RECOVERING;
            p167.last_trigger = (uint)402345171U;
            p167.last_action = (uint)2353657573U;
            p167.last_recovery = (uint)3968467915U;
            p167.last_clear = (uint)1429587347U;
            p167.breach_count = (ushort)(ushort)14743;
            p167.mods_enabled = (LIMIT_MODULE.LIMIT_ALTITUDE);
            p167.mods_required = (LIMIT_MODULE.LIMIT_GPSLOCK |
                                  LIMIT_MODULE.LIMIT_ALTITUDE);
            p167.mods_triggered = (LIMIT_MODULE.LIMIT_GEOFENCE);
            CommunicationChannel.instance.send(p167); //===============================
            WIND p168 = CommunicationChannel.new_WIND();
            PH.setPack(p168);
            p168.direction = (float) -2.1419563E38F;
            p168.speed = (float)8.352947E37F;
            p168.speed_z = (float) -1.5938424E38F;
            CommunicationChannel.instance.send(p168); //===============================
            DATA16 p169 = CommunicationChannel.new_DATA16();
            PH.setPack(p169);
            p169.type = (byte)(byte)212;
            p169.len = (byte)(byte)80;
            p169.data__SET(new byte[16], 0);
            CommunicationChannel.instance.send(p169); //===============================
            DATA32 p170 = CommunicationChannel.new_DATA32();
            PH.setPack(p170);
            p170.type = (byte)(byte)143;
            p170.len = (byte)(byte)197;
            p170.data__SET(new byte[32], 0);
            CommunicationChannel.instance.send(p170); //===============================
            DATA64 p171 = CommunicationChannel.new_DATA64();
            PH.setPack(p171);
            p171.type = (byte)(byte)107;
            p171.len = (byte)(byte)112;
            p171.data__SET(new byte[64], 0);
            CommunicationChannel.instance.send(p171); //===============================
            DATA96 p172 = CommunicationChannel.new_DATA96();
            PH.setPack(p172);
            p172.type = (byte)(byte)43;
            p172.len = (byte)(byte)243;
            p172.data__SET(new byte[96], 0);
            CommunicationChannel.instance.send(p172); //===============================
            RANGEFINDER p173 = CommunicationChannel.new_RANGEFINDER();
            PH.setPack(p173);
            p173.distance = (float)1.9364016E37F;
            p173.voltage = (float)2.4336354E38F;
            CommunicationChannel.instance.send(p173); //===============================
            AIRSPEED_AUTOCAL p174 = CommunicationChannel.new_AIRSPEED_AUTOCAL();
            PH.setPack(p174);
            p174.vx = (float) -2.8593375E37F;
            p174.vy = (float) -2.456305E38F;
            p174.vz = (float) -1.6619115E37F;
            p174.diff_pressure = (float) -3.267997E38F;
            p174.EAS2TAS = (float)2.2320086E38F;
            p174.ratio = (float) -1.3861501E38F;
            p174.state_x = (float)1.2202028E38F;
            p174.state_y = (float) -2.6127183E38F;
            p174.state_z = (float)2.5706875E38F;
            p174.Pax = (float)2.541778E38F;
            p174.Pby = (float) -1.1428372E38F;
            p174.Pcz = (float) -2.796918E38F;
            CommunicationChannel.instance.send(p174); //===============================
            RALLY_POINT p175 = CommunicationChannel.new_RALLY_POINT();
            PH.setPack(p175);
            p175.target_system = (byte)(byte)231;
            p175.target_component = (byte)(byte)201;
            p175.idx = (byte)(byte)233;
            p175.count = (byte)(byte)65;
            p175.lat = (int)1687495395;
            p175.lng = (int)1076456806;
            p175.alt = (short)(short)30448;
            p175.break_alt = (short)(short) -21526;
            p175.land_dir = (ushort)(ushort)48803;
            p175.flags = RALLY_FLAGS.FAVORABLE_WIND;
            CommunicationChannel.instance.send(p175); //===============================
            RALLY_FETCH_POINT p176 = CommunicationChannel.new_RALLY_FETCH_POINT();
            PH.setPack(p176);
            p176.target_system = (byte)(byte)93;
            p176.target_component = (byte)(byte)53;
            p176.idx = (byte)(byte)51;
            CommunicationChannel.instance.send(p176); //===============================
            COMPASSMOT_STATUS p177 = CommunicationChannel.new_COMPASSMOT_STATUS();
            PH.setPack(p177);
            p177.throttle = (ushort)(ushort)21025;
            p177.current = (float) -9.603254E37F;
            p177.interference = (ushort)(ushort)50211;
            p177.CompensationX = (float)1.919289E38F;
            p177.CompensationY = (float)1.9865726E38F;
            p177.CompensationZ = (float)3.275592E38F;
            CommunicationChannel.instance.send(p177); //===============================
            AHRS2 p178 = CommunicationChannel.new_AHRS2();
            PH.setPack(p178);
            p178.roll = (float)2.445835E38F;
            p178.pitch = (float) -7.6650714E37F;
            p178.yaw = (float)2.5299892E38F;
            p178.altitude = (float) -1.2330407E38F;
            p178.lat = (int) -86390247;
            p178.lng = (int)835479041;
            CommunicationChannel.instance.send(p178); //===============================
            CAMERA_STATUS p179 = CommunicationChannel.new_CAMERA_STATUS();
            PH.setPack(p179);
            p179.time_usec = (ulong)2646884005843022868L;
            p179.target_system = (byte)(byte)170;
            p179.cam_idx = (byte)(byte)239;
            p179.img_idx = (ushort)(ushort)55794;
            p179.event_id = CAMERA_STATUS_TYPES.CAMERA_STATUS_TYPE_TRIGGER;
            p179.p1 = (float) -2.7014314E38F;
            p179.p2 = (float)1.3347104E38F;
            p179.p3 = (float) -7.4960596E37F;
            p179.p4 = (float)3.2367195E38F;
            CommunicationChannel.instance.send(p179); //===============================
            CAMERA_FEEDBACK p180 = CommunicationChannel.new_CAMERA_FEEDBACK();
            PH.setPack(p180);
            p180.time_usec = (ulong)3584235378397686440L;
            p180.target_system = (byte)(byte)253;
            p180.cam_idx = (byte)(byte)71;
            p180.img_idx = (ushort)(ushort)56076;
            p180.lat = (int)437228331;
            p180.lng = (int)1240502882;
            p180.alt_msl = (float)1.263639E38F;
            p180.alt_rel = (float)1.871646E38F;
            p180.roll = (float)2.4913066E38F;
            p180.pitch = (float) -7.923162E36F;
            p180.yaw = (float) -3.735765E36F;
            p180.foc_len = (float)2.4420364E38F;
            p180.flags = CAMERA_FEEDBACK_FLAGS.CAMERA_FEEDBACK_PHOTO;
            CommunicationChannel.instance.send(p180); //===============================
            BATTERY2 p181 = CommunicationChannel.new_BATTERY2();
            PH.setPack(p181);
            p181.voltage = (ushort)(ushort)32568;
            p181.current_battery = (short)(short)24529;
            CommunicationChannel.instance.send(p181); //===============================
            AHRS3 p182 = CommunicationChannel.new_AHRS3();
            PH.setPack(p182);
            p182.roll = (float)1.4489703E38F;
            p182.pitch = (float) -3.0587659E38F;
            p182.yaw = (float) -1.4565872E38F;
            p182.altitude = (float)3.0754735E38F;
            p182.lat = (int)2108860413;
            p182.lng = (int) -622226939;
            p182.v1 = (float) -9.715298E36F;
            p182.v2 = (float)5.901798E37F;
            p182.v3 = (float) -1.3692195E38F;
            p182.v4 = (float)8.444453E36F;
            CommunicationChannel.instance.send(p182); //===============================
            AUTOPILOT_VERSION_REQUEST p183 = CommunicationChannel.new_AUTOPILOT_VERSION_REQUEST();
            PH.setPack(p183);
            p183.target_system = (byte)(byte)32;
            p183.target_component = (byte)(byte)156;
            CommunicationChannel.instance.send(p183); //===============================
            REMOTE_LOG_DATA_BLOCK p184 = CommunicationChannel.new_REMOTE_LOG_DATA_BLOCK();
            PH.setPack(p184);
            p184.target_system = (byte)(byte)248;
            p184.target_component = (byte)(byte)156;
            p184.seqno = MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS.MAV_REMOTE_LOG_DATA_BLOCK_START;
            p184.data__SET(new byte[200], 0);
            CommunicationChannel.instance.send(p184); //===============================
            REMOTE_LOG_BLOCK_STATUS p185 = CommunicationChannel.new_REMOTE_LOG_BLOCK_STATUS();
            PH.setPack(p185);
            p185.target_system = (byte)(byte)17;
            p185.target_component = (byte)(byte)80;
            p185.seqno = (uint)3162272068U;
            p185.status = MAV_REMOTE_LOG_DATA_BLOCK_STATUSES.MAV_REMOTE_LOG_DATA_BLOCK_NACK;
            CommunicationChannel.instance.send(p185); //===============================
            LED_CONTROL p186 = CommunicationChannel.new_LED_CONTROL();
            PH.setPack(p186);
            p186.target_system = (byte)(byte)124;
            p186.target_component = (byte)(byte)243;
            p186.instance = (byte)(byte)41;
            p186.pattern = (byte)(byte)98;
            p186.custom_len = (byte)(byte)127;
            p186.custom_bytes_SET(new byte[24], 0);
            CommunicationChannel.instance.send(p186); //===============================
            MAG_CAL_PROGRESS p191 = CommunicationChannel.new_MAG_CAL_PROGRESS();
            PH.setPack(p191);
            p191.compass_id = (byte)(byte)246;
            p191.cal_mask = (byte)(byte)160;
            p191.cal_status = MAG_CAL_STATUS.MAG_CAL_RUNNING_STEP_ONE;
            p191.attempt = (byte)(byte)159;
            p191.completion_pct = (byte)(byte)171;
            p191.completion_mask_SET(new byte[10], 0);
            p191.direction_x = (float)4.79733E37F;
            p191.direction_y = (float)2.364078E38F;
            p191.direction_z = (float) -1.8188433E38F;
            CommunicationChannel.instance.send(p191); //===============================
            MAG_CAL_REPORT p192 = CommunicationChannel.new_MAG_CAL_REPORT();
            PH.setPack(p192);
            p192.compass_id = (byte)(byte)159;
            p192.cal_mask = (byte)(byte)82;
            p192.cal_status = MAG_CAL_STATUS.MAG_CAL_NOT_STARTED;
            p192.autosaved = (byte)(byte)200;
            p192.fitness = (float) -1.0426655E38F;
            p192.ofs_x = (float)3.2595392E38F;
            p192.ofs_y = (float)2.0079078E38F;
            p192.ofs_z = (float)7.426759E37F;
            p192.diag_x = (float) -2.112401E38F;
            p192.diag_y = (float)3.1637967E38F;
            p192.diag_z = (float)3.331122E38F;
            p192.offdiag_x = (float) -2.2881744E38F;
            p192.offdiag_y = (float)1.7225299E38F;
            p192.offdiag_z = (float) -2.7477444E38F;
            CommunicationChannel.instance.send(p192); //===============================
            EKF_STATUS_REPORT p193 = CommunicationChannel.new_EKF_STATUS_REPORT();
            PH.setPack(p193);
            p193.flags = (EKF_STATUS_FLAGS.EKF_PRED_POS_HORIZ_ABS |
                          EKF_STATUS_FLAGS.EKF_ATTITUDE |
                          EKF_STATUS_FLAGS.EKF_VELOCITY_HORIZ |
                          EKF_STATUS_FLAGS.EKF_VELOCITY_VERT);
            p193.velocity_variance = (float)1.9381968E38F;
            p193.pos_horiz_variance = (float)2.8041413E38F;
            p193.pos_vert_variance = (float) -1.6182408E38F;
            p193.compass_variance = (float)3.3203658E37F;
            p193.terrain_alt_variance = (float)2.9520255E36F;
            CommunicationChannel.instance.send(p193); //===============================
            PID_TUNING p194 = CommunicationChannel.new_PID_TUNING();
            PH.setPack(p194);
            p194.axis = PID_TUNING_AXIS.PID_TUNING_PITCH;
            p194.desired = (float)3.1491747E38F;
            p194.achieved = (float)3.5773845E37F;
            p194.FF = (float)1.8892594E37F;
            p194.P = (float) -3.146449E38F;
            p194.I = (float)7.4367174E37F;
            p194.D = (float) -2.7297345E38F;
            CommunicationChannel.instance.send(p194); //===============================
            GIMBAL_REPORT p200 = CommunicationChannel.new_GIMBAL_REPORT();
            PH.setPack(p200);
            p200.target_system = (byte)(byte)248;
            p200.target_component = (byte)(byte)46;
            p200.delta_time = (float)1.1154812E37F;
            p200.delta_angle_x = (float)2.3187028E37F;
            p200.delta_angle_y = (float)2.1161094E38F;
            p200.delta_angle_z = (float) -1.5678096E37F;
            p200.delta_velocity_x = (float)2.0140587E38F;
            p200.delta_velocity_y = (float) -1.772709E38F;
            p200.delta_velocity_z = (float) -3.5342225E37F;
            p200.joint_roll = (float) -1.0013686E38F;
            p200.joint_el = (float)2.3419242E38F;
            p200.joint_az = (float)1.2765355E38F;
            CommunicationChannel.instance.send(p200); //===============================
            GIMBAL_CONTROL p201 = CommunicationChannel.new_GIMBAL_CONTROL();
            PH.setPack(p201);
            p201.target_system = (byte)(byte)244;
            p201.target_component = (byte)(byte)172;
            p201.demanded_rate_x = (float) -3.8510916E37F;
            p201.demanded_rate_y = (float)3.2687679E38F;
            p201.demanded_rate_z = (float) -1.6871751E38F;
            CommunicationChannel.instance.send(p201); //===============================
            GIMBAL_TORQUE_CMD_REPORT p214 = CommunicationChannel.new_GIMBAL_TORQUE_CMD_REPORT();
            PH.setPack(p214);
            p214.target_system = (byte)(byte)38;
            p214.target_component = (byte)(byte)240;
            p214.rl_torque_cmd = (short)(short)14007;
            p214.el_torque_cmd = (short)(short)30786;
            p214.az_torque_cmd = (short)(short) -11358;
            CommunicationChannel.instance.send(p214); //===============================
            GOPRO_HEARTBEAT p215 = CommunicationChannel.new_GOPRO_HEARTBEAT();
            PH.setPack(p215);
            p215.status = GOPRO_HEARTBEAT_STATUS.GOPRO_HEARTBEAT_STATUS_ERROR;
            p215.capture_mode = GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_VIDEO;
            p215.flags = GOPRO_HEARTBEAT_FLAGS.GOPRO_FLAG_RECORDING;
            CommunicationChannel.instance.send(p215); //===============================
            GOPRO_GET_REQUEST p216 = CommunicationChannel.new_GOPRO_GET_REQUEST();
            PH.setPack(p216);
            p216.target_system = (byte)(byte)9;
            p216.target_component = (byte)(byte)122;
            p216.cmd_id = GOPRO_COMMAND.GOPRO_COMMAND_PROTUNE_EXPOSURE;
            CommunicationChannel.instance.send(p216); //===============================
            GOPRO_GET_RESPONSE p217 = CommunicationChannel.new_GOPRO_GET_RESPONSE();
            PH.setPack(p217);
            p217.cmd_id = GOPRO_COMMAND.GOPRO_COMMAND_PROTUNE_WHITE_BALANCE;
            p217.status = GOPRO_REQUEST_STATUS.GOPRO_REQUEST_SUCCESS;
            p217.value_SET(new byte[4], 0);
            CommunicationChannel.instance.send(p217); //===============================
            GOPRO_SET_REQUEST p218 = CommunicationChannel.new_GOPRO_SET_REQUEST();
            PH.setPack(p218);
            p218.target_system = (byte)(byte)52;
            p218.target_component = (byte)(byte)250;
            p218.cmd_id = GOPRO_COMMAND.GOPRO_COMMAND_TIME;
            p218.value_SET(new byte[4], 0);
            CommunicationChannel.instance.send(p218); //===============================
            GOPRO_SET_RESPONSE p219 = CommunicationChannel.new_GOPRO_SET_RESPONSE();
            PH.setPack(p219);
            p219.cmd_id = GOPRO_COMMAND.GOPRO_COMMAND_PROTUNE_COLOUR;
            p219.status = GOPRO_REQUEST_STATUS.GOPRO_REQUEST_FAILED;
            CommunicationChannel.instance.send(p219); //===============================
            RPM p226 = CommunicationChannel.new_RPM();
            PH.setPack(p226);
            p226.rpm1 = (float) -1.6933609E38F;
            p226.rpm2 = (float)3.367512E38F;
            CommunicationChannel.instance.send(p226); //===============================
            ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.time_usec = (ulong)3999608560109212885L;
            p230.flags = (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS);
            p230.vel_ratio = (float) -2.7694127E38F;
            p230.pos_horiz_ratio = (float)2.667946E38F;
            p230.pos_vert_ratio = (float)2.8389053E37F;
            p230.mag_ratio = (float) -6.161219E37F;
            p230.hagl_ratio = (float) -1.2625306E38F;
            p230.tas_ratio = (float)2.6736915E38F;
            p230.pos_horiz_accuracy = (float)2.6835557E38F;
            p230.pos_vert_accuracy = (float) -3.3909068E37F;
            CommunicationChannel.instance.send(p230); //===============================
            WIND_COV p231 = CommunicationChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.time_usec = (ulong)6826191038175042636L;
            p231.wind_x = (float)1.7152868E38F;
            p231.wind_y = (float)2.6280514E38F;
            p231.wind_z = (float) -2.0698053E38F;
            p231.var_horiz = (float)3.0888793E38F;
            p231.var_vert = (float) -1.737868E38F;
            p231.wind_alt = (float) -2.6445558E38F;
            p231.horiz_accuracy = (float)3.222288E38F;
            p231.vert_accuracy = (float) -1.9897417E38F;
            CommunicationChannel.instance.send(p231); //===============================
            GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.time_usec = (ulong)3769903448535646039L;
            p232.gps_id = (byte)(byte)196;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ);
            p232.time_week_ms = (uint)2562422000U;
            p232.time_week = (ushort)(ushort)8809;
            p232.fix_type = (byte)(byte)51;
            p232.lat = (int)824748598;
            p232.lon = (int)1261131089;
            p232.alt = (float)2.92031E38F;
            p232.hdop = (float) -1.7421044E38F;
            p232.vdop = (float)1.7296535E38F;
            p232.vn = (float) -2.6405083E38F;
            p232.ve = (float) -2.242584E38F;
            p232.vd = (float)2.5871978E38F;
            p232.speed_accuracy = (float) -2.00719E38F;
            p232.horiz_accuracy = (float) -2.6516419E37F;
            p232.vert_accuracy = (float) -8.719419E37F;
            p232.satellites_visible = (byte)(byte)12;
            CommunicationChannel.instance.send(p232); //===============================
            GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.flags = (byte)(byte)139;
            p233.len = (byte)(byte)194;
            p233.data__SET(new byte[180], 0);
            CommunicationChannel.instance.send(p233); //===============================
            HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED);
            p234.custom_mode = (uint)472516732U;
            p234.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF;
            p234.roll = (short)(short)28056;
            p234.pitch = (short)(short) -20734;
            p234.heading = (ushort)(ushort)7470;
            p234.throttle = (sbyte)(sbyte) - 91;
            p234.heading_sp = (short)(short) -26052;
            p234.latitude = (int)925417282;
            p234.longitude = (int)1922979273;
            p234.altitude_amsl = (short)(short)24524;
            p234.altitude_sp = (short)(short) -30708;
            p234.airspeed = (byte)(byte)142;
            p234.airspeed_sp = (byte)(byte)90;
            p234.groundspeed = (byte)(byte)122;
            p234.climb_rate = (sbyte)(sbyte)93;
            p234.gps_nsat = (byte)(byte)249;
            p234.gps_fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_PPP;
            p234.battery_remaining = (byte)(byte)174;
            p234.temperature = (sbyte)(sbyte)44;
            p234.temperature_air = (sbyte)(sbyte)66;
            p234.failsafe = (byte)(byte)182;
            p234.wp_num = (byte)(byte)134;
            p234.wp_distance = (ushort)(ushort)21582;
            CommunicationChannel.instance.send(p234); //===============================
            VIBRATION p241 = CommunicationChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.time_usec = (ulong)2072642263962865674L;
            p241.vibration_x = (float) -1.2626759E38F;
            p241.vibration_y = (float) -7.0575803E37F;
            p241.vibration_z = (float)1.2611582E38F;
            p241.clipping_0 = (uint)3024337922U;
            p241.clipping_1 = (uint)2342940377U;
            p241.clipping_2 = (uint)1987521756U;
            CommunicationChannel.instance.send(p241); //===============================
            HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.latitude = (int) -997705556;
            p242.longitude = (int) -1127660088;
            p242.altitude = (int) -872808067;
            p242.x = (float) -3.5956483E37F;
            p242.y = (float) -3.1513198E38F;
            p242.z = (float)1.2347893E38F;
            p242.q_SET(new float[4], 0);
            p242.approach_x = (float)2.336096E38F;
            p242.approach_y = (float)2.6775355E38F;
            p242.approach_z = (float) -3.2825338E38F;
            p242.time_usec_SET((ulong)8964309916681220959L, PH);
            CommunicationChannel.instance.send(p242); //===============================
            SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.target_system = (byte)(byte)71;
            p243.latitude = (int)1836870865;
            p243.longitude = (int)1686897340;
            p243.altitude = (int) -925796843;
            p243.x = (float) -9.1325555E36F;
            p243.y = (float)2.8972075E38F;
            p243.z = (float)4.5246554E36F;
            p243.q_SET(new float[4], 0);
            p243.approach_x = (float) -2.910723E38F;
            p243.approach_y = (float)1.614949E38F;
            p243.approach_z = (float) -1.322929E38F;
            p243.time_usec_SET((ulong)4479879851797755171L, PH);
            CommunicationChannel.instance.send(p243); //===============================
            MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.message_id = (ushort)(ushort)24526;
            p244.interval_us = (int) -1922563535;
            CommunicationChannel.instance.send(p244); //===============================
            EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.vtol_state = MAV_VTOL_STATE.MAV_VTOL_STATE_UNDEFINED;
            p245.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR;
            CommunicationChannel.instance.send(p245); //===============================
            ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.ICAO_address = (uint)2689819052U;
            p246.lat = (int) -1946702969;
            p246.lon = (int)903314969;
            p246.altitude_type = ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC;
            p246.altitude = (int) -1558056479;
            p246.heading = (ushort)(ushort)63056;
            p246.hor_velocity = (ushort)(ushort)53884;
            p246.ver_velocity = (short)(short)14164;
            p246.callsign_SET("DEMO", PH);
            p246.emitter_type = ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UNASSGINED3;
            p246.tslc = (byte)(byte)80;
            p246.flags = (ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS |
                          ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE |
                          ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY);
            p246.squawk = (ushort)(ushort)24837;
            CommunicationChannel.instance.send(p246); //===============================
            COLLISION p247 = CommunicationChannel.new_COLLISION();
            PH.setPack(p247);
            p247.src_ = MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB;
            p247.id = (uint)364804454U;
            p247.action = MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_REPORT;
            p247.threat_level = MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW;
            p247.time_to_minimum_delta = (float) -2.8032328E38F;
            p247.altitude_minimum_delta = (float) -2.477035E38F;
            p247.horizontal_minimum_delta = (float) -1.384316E38F;
            CommunicationChannel.instance.send(p247); //===============================
            V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.target_network = (byte)(byte)116;
            p248.target_system = (byte)(byte)82;
            p248.target_component = (byte)(byte)174;
            p248.message_type = (ushort)(ushort)26994;
            p248.payload_SET(new byte[249], 0);
            CommunicationChannel.instance.send(p248); //===============================
            MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.address = (ushort)(ushort)42333;
            p249.ver = (byte)(byte)159;
            p249.type = (byte)(byte)44;
            p249.value_SET(new sbyte[32], 0);
            CommunicationChannel.instance.send(p249); //===============================
            DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.name_SET("DEMO", PH);
            p250.time_usec = (ulong)3917290547786565003L;
            p250.x = (float)5.750935E37F;
            p250.y = (float) -1.924375E38F;
            p250.z = (float) -1.3392422E38F;
            CommunicationChannel.instance.send(p250); //===============================
            NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.time_boot_ms = (uint)2065854625U;
            p251.name_SET("DEMO", PH);
            p251.value = (float)1.0319586E38F;
            CommunicationChannel.instance.send(p251); //===============================
            NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.time_boot_ms = (uint)3941061977U;
            p252.name_SET("DEMO", PH);
            p252.value = (int)1172385280;
            CommunicationChannel.instance.send(p252); //===============================
            STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.severity = MAV_SEVERITY.MAV_SEVERITY_ALERT;
            p253.text_SET("DEMO", PH);
            CommunicationChannel.instance.send(p253); //===============================
            DEBUG p254 = CommunicationChannel.new_DEBUG();
            PH.setPack(p254);
            p254.time_boot_ms = (uint)3539227295U;
            p254.ind = (byte)(byte)245;
            p254.value = (float)5.6154183E37F;
            CommunicationChannel.instance.send(p254); //===============================
            SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.target_system = (byte)(byte)124;
            p256.target_component = (byte)(byte)205;
            p256.secret_key_SET(new byte[32], 0);
            p256.initial_timestamp = (ulong)7467567152912038476L;
            CommunicationChannel.instance.send(p256); //===============================
            BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.time_boot_ms = (uint)281638391U;
            p257.last_change_ms = (uint)889039480U;
            p257.state = (byte)(byte)205;
            CommunicationChannel.instance.send(p257); //===============================
            PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_system = (byte)(byte)186;
            p258.target_component = (byte)(byte)128;
            p258.tune_SET("DEMO", PH);
            CommunicationChannel.instance.send(p258); //===============================
            CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.time_boot_ms = (uint)66698103U;
            p259.vendor_name_SET(new byte[32], 0);
            p259.model_name_SET(new byte[32], 0);
            p259.firmware_version = (uint)2579456330U;
            p259.focal_length = (float) -6.981558E36F;
            p259.sensor_size_h = (float)1.1875928E38F;
            p259.sensor_size_v = (float)3.1276163E38F;
            p259.resolution_h = (ushort)(ushort)54334;
            p259.resolution_v = (ushort)(ushort)28295;
            p259.lens_id = (byte)(byte)184;
            p259.flags = (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE |
                          CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_MODES |
                          CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
                          CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE);
            p259.cam_definition_version = (ushort)(ushort)15724;
            p259.cam_definition_uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p259); //===============================
            CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.time_boot_ms = (uint)1796642129U;
            p260.mode_id = CAMERA_MODE.CAMERA_MODE_VIDEO;
            CommunicationChannel.instance.send(p260); //===============================
            STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.time_boot_ms = (uint)2389832255U;
            p261.storage_id = (byte)(byte)207;
            p261.storage_count = (byte)(byte)34;
            p261.status = (byte)(byte)182;
            p261.total_capacity = (float)2.4396135E38F;
            p261.used_capacity = (float)1.1147741E37F;
            p261.available_capacity = (float) -2.9635164E38F;
            p261.read_speed = (float)2.2846098E38F;
            p261.write_speed = (float) -2.500887E38F;
            CommunicationChannel.instance.send(p261); //===============================
            CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.time_boot_ms = (uint)1150199303U;
            p262.image_status = (byte)(byte)228;
            p262.video_status = (byte)(byte)73;
            p262.image_interval = (float) -3.0492688E38F;
            p262.recording_time_ms = (uint)405351019U;
            p262.available_capacity = (float) -2.4185394E38F;
            CommunicationChannel.instance.send(p262); //===============================
            CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.time_boot_ms = (uint)2487477567U;
            p263.time_utc = (ulong)786757930655879680L;
            p263.camera_id = (byte)(byte)62;
            p263.lat = (int) -359303074;
            p263.lon = (int) -1207958666;
            p263.alt = (int) -158876640;
            p263.relative_alt = (int)1049681529;
            p263.q_SET(new float[4], 0);
            p263.image_index = (int) -785659336;
            p263.capture_result = (sbyte)(sbyte)110;
            p263.file_url_SET("DEMO", PH);
            CommunicationChannel.instance.send(p263); //===============================
            FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.time_boot_ms = (uint)994825690U;
            p264.arming_time_utc = (ulong)2814190813313607548L;
            p264.takeoff_time_utc = (ulong)2994854465850464177L;
            p264.flight_uuid = (ulong)5416832270929194471L;
            CommunicationChannel.instance.send(p264); //===============================
            MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.time_boot_ms = (uint)2302518819U;
            p265.roll = (float) -1.1870652E38F;
            p265.pitch = (float) -1.3015462E38F;
            p265.yaw = (float) -6.758307E37F;
            CommunicationChannel.instance.send(p265); //===============================
            LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.target_system = (byte)(byte)1;
            p266.target_component = (byte)(byte)117;
            p266.sequence = (ushort)(ushort)31366;
            p266.length = (byte)(byte)184;
            p266.first_message_offset = (byte)(byte)61;
            p266.data__SET(new byte[249], 0);
            CommunicationChannel.instance.send(p266); //===============================
            LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.target_system = (byte)(byte)55;
            p267.target_component = (byte)(byte)29;
            p267.sequence = (ushort)(ushort)37372;
            p267.length = (byte)(byte)79;
            p267.first_message_offset = (byte)(byte)31;
            p267.data__SET(new byte[249], 0);
            CommunicationChannel.instance.send(p267); //===============================
            LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_system = (byte)(byte)201;
            p268.target_component = (byte)(byte)5;
            p268.sequence = (ushort)(ushort)12992;
            CommunicationChannel.instance.send(p268); //===============================
            VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.camera_id = (byte)(byte)132;
            p269.status = (byte)(byte)174;
            p269.framerate = (float) -1.4201845E38F;
            p269.resolution_h = (ushort)(ushort)18673;
            p269.resolution_v = (ushort)(ushort)14986;
            p269.bitrate = (uint)2680241964U;
            p269.rotation = (ushort)(ushort)21654;
            p269.uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p269); //===============================
            SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.target_system = (byte)(byte)213;
            p270.target_component = (byte)(byte)151;
            p270.camera_id = (byte)(byte)18;
            p270.framerate = (float) -1.9826915E38F;
            p270.resolution_h = (ushort)(ushort)60379;
            p270.resolution_v = (ushort)(ushort)32012;
            p270.bitrate = (uint)2294489828U;
            p270.rotation = (ushort)(ushort)32476;
            p270.uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p270); //===============================
            WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.ssid_SET("DEMO", PH);
            p299.password_SET("DEMO", PH);
            CommunicationChannel.instance.send(p299); //===============================
            PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.version = (ushort)(ushort)30139;
            p300.min_version = (ushort)(ushort)65283;
            p300.max_version = (ushort)(ushort)64688;
            p300.spec_version_hash_SET(new byte[8], 0);
            p300.library_version_hash_SET(new byte[8], 0);
            CommunicationChannel.instance.send(p300); //===============================
            UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.time_usec = (ulong)5175382611103519098L;
            p310.uptime_sec = (uint)2060924898U;
            p310.health = UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK;
            p310.mode = UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION;
            p310.sub_mode = (byte)(byte)50;
            p310.vendor_specific_status_code = (ushort)(ushort)29147;
            CommunicationChannel.instance.send(p310); //===============================
            UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.time_usec = (ulong)1610121781926364284L;
            p311.uptime_sec = (uint)627904451U;
            p311.name_SET("DEMO", PH);
            p311.hw_version_major = (byte)(byte)95;
            p311.hw_version_minor = (byte)(byte)196;
            p311.hw_unique_id_SET(new byte[16], 0);
            p311.sw_version_major = (byte)(byte)18;
            p311.sw_version_minor = (byte)(byte)126;
            p311.sw_vcs_commit = (uint)666079922U;
            CommunicationChannel.instance.send(p311); //===============================
            PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.target_system = (byte)(byte)133;
            p320.target_component = (byte)(byte)209;
            p320.param_id_SET("DEMO", PH);
            p320.param_index = (short)(short)1676;
            CommunicationChannel.instance.send(p320); //===============================
            PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_system = (byte)(byte)246;
            p321.target_component = (byte)(byte)90;
            CommunicationChannel.instance.send(p321); //===============================
            PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_id_SET("DEMO", PH);
            p322.param_value_SET("DEMO", PH);
            p322.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64;
            p322.param_count = (ushort)(ushort)46658;
            p322.param_index = (ushort)(ushort)5692;
            CommunicationChannel.instance.send(p322); //===============================
            PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.target_system = (byte)(byte)72;
            p323.target_component = (byte)(byte)35;
            p323.param_id_SET("DEMO", PH);
            p323.param_value_SET("DEMO", PH);
            p323.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16;
            CommunicationChannel.instance.send(p323); //===============================
            PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_id_SET("DEMO", PH);
            p324.param_value_SET("DEMO", PH);
            p324.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32;
            p324.param_result = PARAM_ACK.PARAM_ACK_ACCEPTED;
            CommunicationChannel.instance.send(p324); //===============================
            OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.time_usec = (ulong)7023769493146220661L;
            p330.sensor_type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR;
            p330.distances_SET(new ushort[72], 0);
            p330.increment = (byte)(byte)69;
            p330.min_distance = (ushort)(ushort)29479;
            p330.max_distance = (ushort)(ushort)64147;
            CommunicationChannel.instance.send(p330); //===============================
            UAVIONIX_ADSB_OUT_CFG p10001 = CommunicationChannel.new_UAVIONIX_ADSB_OUT_CFG();
            PH.setPack(p10001);
            p10001.ICAO = (uint)1622692713U;
            p10001.callsign_SET("DEMO", PH);
            p10001.emitterType = ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UNASSIGNED2;
            p10001.aircraftSize = UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE.UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L65_67M;
            p10001.gpsOffsetLat = UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_2M;
            p10001.gpsOffsetLon = UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_APPLIED_BY_SENSOR;
            p10001.stallSpeed = (ushort)(ushort)17409;
            p10001.rfSelect = UAVIONIX_ADSB_OUT_RF_SELECT.UAVIONIX_ADSB_OUT_RF_SELECT_RX_ENABLED;
            CommunicationChannel.instance.send(p10001); //===============================
            UAVIONIX_ADSB_OUT_DYNAMIC p10002 = CommunicationChannel.new_UAVIONIX_ADSB_OUT_DYNAMIC();
            PH.setPack(p10002);
            p10002.utcTime = (uint)2932453318U;
            p10002.gpsLat = (int)686994784;
            p10002.gpsLon = (int) -407655984;
            p10002.gpsAlt = (int)1200967349;
            p10002.gpsFix = UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX.UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_NONE_0;
            p10002.numSats = (byte)(byte)206;
            p10002.baroAltMSL = (int)1344872309;
            p10002.accuracyHor = (uint)4093380243U;
            p10002.accuracyVert = (ushort)(ushort)31872;
            p10002.accuracyVel = (ushort)(ushort)22682;
            p10002.velVert = (short)(short)7538;
            p10002.velNS = (short)(short)20467;
            p10002.VelEW = (short)(short) -7315;
            p10002.emergencyStatus = UAVIONIX_ADSB_EMERGENCY_STATUS.UAVIONIX_ADSB_OUT_GENERAL_EMERGENCY;
            p10002.state = (UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_NICBARO_CROSSCHECKED |
                            UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_IDENT);
            p10002.squawk = (ushort)(ushort)50618;
            CommunicationChannel.instance.send(p10002); //===============================
            UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT p10003 = CommunicationChannel.new_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT();
            PH.setPack(p10003);
            p10003.rfHealth = UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_INITIALIZING;
            CommunicationChannel.instance.send(p10003); //===============================
            DEVICE_OP_READ p11000 = CommunicationChannel.new_DEVICE_OP_READ();
            PH.setPack(p11000);
            p11000.target_system = (byte)(byte)210;
            p11000.target_component = (byte)(byte)180;
            p11000.request_id = (uint)1019773120U;
            p11000.bustype = DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_I2C;
            p11000.bus = (byte)(byte)134;
            p11000.address = (byte)(byte)62;
            p11000.busname_SET("DEMO", PH);
            p11000.regstart = (byte)(byte)61;
            p11000.count = (byte)(byte)43;
            CommunicationChannel.instance.send(p11000); //===============================
            DEVICE_OP_READ_REPLY p11001 = CommunicationChannel.new_DEVICE_OP_READ_REPLY();
            PH.setPack(p11001);
            p11001.request_id = (uint)871115704U;
            p11001.result = (byte)(byte)146;
            p11001.regstart = (byte)(byte)3;
            p11001.count = (byte)(byte)214;
            p11001.data__SET(new byte[128], 0);
            CommunicationChannel.instance.send(p11001); //===============================
            DEVICE_OP_WRITE p11002 = CommunicationChannel.new_DEVICE_OP_WRITE();
            PH.setPack(p11002);
            p11002.target_system = (byte)(byte)144;
            p11002.target_component = (byte)(byte)127;
            p11002.request_id = (uint)823958476U;
            p11002.bustype = DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_I2C;
            p11002.bus = (byte)(byte)122;
            p11002.address = (byte)(byte)202;
            p11002.busname_SET("DEMO", PH);
            p11002.regstart = (byte)(byte)171;
            p11002.count = (byte)(byte)193;
            p11002.data__SET(new byte[128], 0);
            CommunicationChannel.instance.send(p11002); //===============================
            DEVICE_OP_WRITE_REPLY p11003 = CommunicationChannel.new_DEVICE_OP_WRITE_REPLY();
            PH.setPack(p11003);
            p11003.request_id = (uint)4159393061U;
            p11003.result = (byte)(byte)249;
            CommunicationChannel.instance.send(p11003); //===============================
            ADAP_TUNING p11010 = CommunicationChannel.new_ADAP_TUNING();
            PH.setPack(p11010);
            p11010.axis = PID_TUNING_AXIS.PID_TUNING_STEER;
            p11010.desired = (float)2.6833768E38F;
            p11010.achieved = (float) -1.393047E38F;
            p11010.error = (float) -1.8068907E38F;
            p11010.theta = (float) -2.4263559E38F;
            p11010.omega = (float) -2.3679458E38F;
            p11010.sigma = (float)3.059803E38F;
            p11010.theta_dot = (float)1.994606E38F;
            p11010.omega_dot = (float)5.154125E37F;
            p11010.sigma_dot = (float)6.884198E36F;
            p11010.f = (float)3.1025903E38F;
            p11010.f_dot = (float)2.4726643E38F;
            p11010.u = (float)2.216159E38F;
            CommunicationChannel.instance.send(p11010); //===============================
            VISION_POSITION_DELTA p11011 = CommunicationChannel.new_VISION_POSITION_DELTA();
            PH.setPack(p11011);
            p11011.time_usec = (ulong)1166062074997220402L;
            p11011.time_delta_usec = (ulong)356544606682240266L;
            p11011.angle_delta_SET(new float[3], 0);
            p11011.position_delta_SET(new float[3], 0);
            p11011.confidence = (float) -1.8681197E38F;
            CommunicationChannel.instance.send(p11011); //===============================
        }
    }
}

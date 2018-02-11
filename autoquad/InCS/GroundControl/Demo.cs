
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
            POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.time_boot_ms = (uint)1997574100U;
            p3.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL;
            p3.type_mask = (ushort)(ushort)7350;
            p3.x = (float) -2.6557232E37F;
            p3.y = (float) -6.4652E37F;
            p3.z = (float)8.2965237E37F;
            p3.vx = (float)2.6211353E38F;
            p3.vy = (float)3.05752E38F;
            p3.vz = (float) -1.6259948E38F;
            p3.afx = (float) -1.2587099E38F;
            p3.afy = (float)1.0730821E38F;
            p3.afz = (float)2.5247763E38F;
            p3.yaw = (float) -2.3365875E38F;
            p3.yaw_rate = (float)3.7297708E37F;
            CommunicationChannel.instance.send(p3); //===============================
            COMMAND_INT p75 = CommunicationChannel.new_COMMAND_INT();
            PH.setPack(p75);
            p75.target_system = (byte)(byte)64;
            p75.target_component = (byte)(byte)59;
            p75.frame = MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p75.command = MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE;
            p75.current = (byte)(byte)28;
            p75.autocontinue = (byte)(byte)81;
            p75.param1 = (float)2.6436302E38F;
            p75.param2 = (float)5.5232703E37F;
            p75.param3 = (float)2.4176338E36F;
            p75.param4 = (float) -2.3632407E38F;
            p75.x = (int)504088137;
            p75.y = (int)1677925620;
            p75.z = (float) -3.6866436E37F;
            CommunicationChannel.instance.send(p75); //===============================
            COMMAND_LONG p76 = CommunicationChannel.new_COMMAND_LONG();
            PH.setPack(p76);
            p76.target_system = (byte)(byte)226;
            p76.target_component = (byte)(byte)54;
            p76.command = MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION;
            p76.confirmation = (byte)(byte)214;
            p76.param1 = (float) -3.2033074E38F;
            p76.param2 = (float) -3.4213284E37F;
            p76.param3 = (float) -2.8076428E38F;
            p76.param4 = (float)1.3929744E38F;
            p76.param5 = (float)2.421028E37F;
            p76.param6 = (float)1.4563351E38F;
            p76.param7 = (float) -6.1790107E37F;
            CommunicationChannel.instance.send(p76); //===============================
            COMMAND_ACK p77 = CommunicationChannel.new_COMMAND_ACK();
            PH.setPack(p77);
            p77.command = MAV_CMD.MAV_CMD_NAV_DELAY;
            p77.result = MAV_RESULT.MAV_RESULT_ACCEPTED;
            p77.progress_SET((byte)(byte)14, PH);
            p77.result_param2_SET((int)1250220302, PH);
            p77.target_system_SET((byte)(byte)178, PH);
            p77.target_component_SET((byte)(byte)1, PH);
            CommunicationChannel.instance.send(p77); //===============================
            MANUAL_SETPOINT p81 = CommunicationChannel.new_MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.time_boot_ms = (uint)4043792627U;
            p81.roll = (float)2.9092778E38F;
            p81.pitch = (float) -1.5157862E38F;
            p81.yaw = (float) -2.0152042E37F;
            p81.thrust = (float) -9.451572E37F;
            p81.mode_switch = (byte)(byte)129;
            p81.manual_override_switch = (byte)(byte)250;
            CommunicationChannel.instance.send(p81); //===============================
            SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.time_boot_ms = (uint)605116133U;
            p82.target_system = (byte)(byte)81;
            p82.target_component = (byte)(byte)158;
            p82.type_mask = (byte)(byte)110;
            p82.q_SET(new float[4], 0);
            p82.body_roll_rate = (float) -1.0840405E38F;
            p82.body_pitch_rate = (float)3.2987224E38F;
            p82.body_yaw_rate = (float)1.5300003E38F;
            p82.thrust = (float) -6.6972257E36F;
            CommunicationChannel.instance.send(p82); //===============================
            ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.time_boot_ms = (uint)960069836U;
            p83.type_mask = (byte)(byte)245;
            p83.q_SET(new float[4], 0);
            p83.body_roll_rate = (float)1.3462271E38F;
            p83.body_pitch_rate = (float)7.329994E37F;
            p83.body_yaw_rate = (float)2.0023008E38F;
            p83.thrust = (float) -1.0145663E38F;
            CommunicationChannel.instance.send(p83); //===============================
            SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.time_boot_ms = (uint)2428666633U;
            p84.target_system = (byte)(byte)28;
            p84.target_component = (byte)(byte)184;
            p84.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_ENU;
            p84.type_mask = (ushort)(ushort)4711;
            p84.x = (float)3.4006226E38F;
            p84.y = (float)2.361191E38F;
            p84.z = (float)2.1562157E38F;
            p84.vx = (float) -1.7732473E38F;
            p84.vy = (float)1.5966626E38F;
            p84.vz = (float) -8.917545E37F;
            p84.afx = (float)3.1175607E38F;
            p84.afy = (float) -1.910882E38F;
            p84.afz = (float) -1.3166786E38F;
            p84.yaw = (float)1.2210776E38F;
            p84.yaw_rate = (float)2.4365182E38F;
            CommunicationChannel.instance.send(p84); //===============================
            SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.time_boot_ms = (uint)972240862U;
            p86.target_system = (byte)(byte)85;
            p86.target_component = (byte)(byte)148;
            p86.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_NED;
            p86.type_mask = (ushort)(ushort)52536;
            p86.lat_int = (int)1385238167;
            p86.lon_int = (int)357376763;
            p86.alt = (float) -7.894263E37F;
            p86.vx = (float) -1.477363E38F;
            p86.vy = (float) -5.319022E37F;
            p86.vz = (float)3.1257597E38F;
            p86.afx = (float)6.4730747E37F;
            p86.afy = (float)1.9979559E37F;
            p86.afz = (float) -2.7257585E38F;
            p86.yaw = (float)2.1272741E37F;
            p86.yaw_rate = (float) -2.735763E38F;
            CommunicationChannel.instance.send(p86); //===============================
            POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.time_boot_ms = (uint)1498930060U;
            p87.coordinate_frame = MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p87.type_mask = (ushort)(ushort)2053;
            p87.lat_int = (int)1064354586;
            p87.lon_int = (int)1886531366;
            p87.alt = (float) -1.1220248E38F;
            p87.vx = (float) -1.4141294E38F;
            p87.vy = (float)2.5080838E38F;
            p87.vz = (float)1.1826911E37F;
            p87.afx = (float) -2.9418096E38F;
            p87.afy = (float) -1.0342333E38F;
            p87.afz = (float)9.463432E37F;
            p87.yaw = (float)3.2751252E38F;
            p87.yaw_rate = (float) -2.7676508E38F;
            CommunicationChannel.instance.send(p87); //===============================
            LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.time_boot_ms = (uint)936734746U;
            p89.x = (float)3.0800718E37F;
            p89.y = (float)2.3891711E38F;
            p89.z = (float) -3.6651572E37F;
            p89.roll = (float) -8.803E37F;
            p89.pitch = (float)1.5035097E38F;
            p89.yaw = (float) -6.778362E37F;
            CommunicationChannel.instance.send(p89); //===============================
            HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.time_usec = (ulong)9088480838805661316L;
            p90.roll = (float)1.0505415E38F;
            p90.pitch = (float) -3.1898193E36F;
            p90.yaw = (float)3.1490827E37F;
            p90.rollspeed = (float)1.6448797E38F;
            p90.pitchspeed = (float) -3.2556949E38F;
            p90.yawspeed = (float)3.2870774E38F;
            p90.lat = (int) -724738103;
            p90.lon = (int)1593779022;
            p90.alt = (int) -888391863;
            p90.vx = (short)(short)10747;
            p90.vy = (short)(short) -23790;
            p90.vz = (short)(short)27825;
            p90.xacc = (short)(short) -16761;
            p90.yacc = (short)(short) -28574;
            p90.zacc = (short)(short)6047;
            CommunicationChannel.instance.send(p90); //===============================
            HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.time_usec = (ulong)8724249968630678978L;
            p91.roll_ailerons = (float)3.370739E38F;
            p91.pitch_elevator = (float) -1.1227876E38F;
            p91.yaw_rudder = (float) -1.6392903E38F;
            p91.throttle = (float)3.314925E38F;
            p91.aux1 = (float)2.230155E37F;
            p91.aux2 = (float) -2.8195977E38F;
            p91.aux3 = (float)2.2919303E37F;
            p91.aux4 = (float) -1.726659E38F;
            p91.mode = MAV_MODE.MAV_MODE_PREFLIGHT;
            p91.nav_mode = (byte)(byte)100;
            CommunicationChannel.instance.send(p91); //===============================
            HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.time_usec = (ulong)1400551071173036978L;
            p92.chan1_raw = (ushort)(ushort)12937;
            p92.chan2_raw = (ushort)(ushort)11529;
            p92.chan3_raw = (ushort)(ushort)21957;
            p92.chan4_raw = (ushort)(ushort)1985;
            p92.chan5_raw = (ushort)(ushort)41710;
            p92.chan6_raw = (ushort)(ushort)27382;
            p92.chan7_raw = (ushort)(ushort)21345;
            p92.chan8_raw = (ushort)(ushort)10055;
            p92.chan9_raw = (ushort)(ushort)21663;
            p92.chan10_raw = (ushort)(ushort)28523;
            p92.chan11_raw = (ushort)(ushort)15660;
            p92.chan12_raw = (ushort)(ushort)39712;
            p92.rssi = (byte)(byte)181;
            CommunicationChannel.instance.send(p92); //===============================
            HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.time_usec = (ulong)1704151941382060657L;
            p93.controls_SET(new float[16], 0);
            p93.mode = MAV_MODE.MAV_MODE_STABILIZE_DISARMED;
            p93.flags = (ulong)4776714950708685252L;
            CommunicationChannel.instance.send(p93); //===============================
            OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.time_usec = (ulong)894481794152598005L;
            p100.sensor_id = (byte)(byte)26;
            p100.flow_x = (short)(short)6420;
            p100.flow_y = (short)(short)4524;
            p100.flow_comp_m_x = (float) -1.3289211E38F;
            p100.flow_comp_m_y = (float)2.2345058E38F;
            p100.quality = (byte)(byte)152;
            p100.ground_distance = (float) -9.710452E37F;
            p100.flow_rate_x_SET((float)1.8613314E37F, PH);
            p100.flow_rate_y_SET((float) -5.3564536E37F, PH);
            CommunicationChannel.instance.send(p100); //===============================
            GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.usec = (ulong)339194697937167620L;
            p101.x = (float)1.8194857E38F;
            p101.y = (float)3.272791E38F;
            p101.z = (float)3.1181292E38F;
            p101.roll = (float) -1.7167135E38F;
            p101.pitch = (float)2.2542468E38F;
            p101.yaw = (float) -1.6368093E38F;
            CommunicationChannel.instance.send(p101); //===============================
            VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.usec = (ulong)4084361361218662366L;
            p102.x = (float)1.3760629E38F;
            p102.y = (float) -7.353127E37F;
            p102.z = (float) -2.7686304E38F;
            p102.roll = (float) -2.5979014E38F;
            p102.pitch = (float)1.5954346E38F;
            p102.yaw = (float)1.9209594E38F;
            CommunicationChannel.instance.send(p102); //===============================
            VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.usec = (ulong)8790921857008366657L;
            p103.x = (float) -5.9569285E37F;
            p103.y = (float) -1.105149E38F;
            p103.z = (float) -2.1267532E38F;
            CommunicationChannel.instance.send(p103); //===============================
            VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.usec = (ulong)5084279482375205231L;
            p104.x = (float) -3.0453344E38F;
            p104.y = (float) -2.4588714E38F;
            p104.z = (float) -8.4792307E37F;
            p104.roll = (float) -2.0449174E38F;
            p104.pitch = (float) -1.3171496E38F;
            p104.yaw = (float) -3.0376121E38F;
            CommunicationChannel.instance.send(p104); //===============================
            HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.time_usec = (ulong)4378710112755781207L;
            p105.xacc = (float)2.5462754E38F;
            p105.yacc = (float) -3.209731E37F;
            p105.zacc = (float)4.4823993E37F;
            p105.xgyro = (float) -2.0706478E38F;
            p105.ygyro = (float) -2.1312926E38F;
            p105.zgyro = (float) -1.2843639E38F;
            p105.xmag = (float)6.6117856E37F;
            p105.ymag = (float)6.722682E37F;
            p105.zmag = (float)7.7729333E37F;
            p105.abs_pressure = (float) -5.01584E37F;
            p105.diff_pressure = (float)2.0348386E38F;
            p105.pressure_alt = (float)4.9240645E37F;
            p105.temperature = (float) -5.064053E37F;
            p105.fields_updated = (ushort)(ushort)14449;
            CommunicationChannel.instance.send(p105); //===============================
            OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.time_usec = (ulong)873993143307815422L;
            p106.sensor_id = (byte)(byte)46;
            p106.integration_time_us = (uint)422485320U;
            p106.integrated_x = (float)9.398473E36F;
            p106.integrated_y = (float) -1.6944022E38F;
            p106.integrated_xgyro = (float) -1.1653237E38F;
            p106.integrated_ygyro = (float)1.7385536E38F;
            p106.integrated_zgyro = (float)1.1992813E38F;
            p106.temperature = (short)(short) -19482;
            p106.quality = (byte)(byte)209;
            p106.time_delta_distance_us = (uint)4025285810U;
            p106.distance = (float)2.0319123E38F;
            CommunicationChannel.instance.send(p106); //===============================
            HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.time_usec = (ulong)5798752963289939481L;
            p107.xacc = (float) -2.1284051E37F;
            p107.yacc = (float)2.7152402E38F;
            p107.zacc = (float) -2.0564406E38F;
            p107.xgyro = (float)5.2023017E37F;
            p107.ygyro = (float)4.35548E37F;
            p107.zgyro = (float) -1.5002386E38F;
            p107.xmag = (float) -1.4485926E38F;
            p107.ymag = (float)9.424634E37F;
            p107.zmag = (float) -1.4017717E37F;
            p107.abs_pressure = (float) -1.3530318E38F;
            p107.diff_pressure = (float) -9.44995E37F;
            p107.pressure_alt = (float) -4.5451815E37F;
            p107.temperature = (float)5.295512E37F;
            p107.fields_updated = (uint)3299497593U;
            CommunicationChannel.instance.send(p107); //===============================
            SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.q1 = (float) -1.3902411E38F;
            p108.q2 = (float)1.1829692E38F;
            p108.q3 = (float)2.7488788E37F;
            p108.q4 = (float)2.8711966E38F;
            p108.roll = (float)1.2697474E38F;
            p108.pitch = (float)1.8058332E38F;
            p108.yaw = (float)3.2813012E38F;
            p108.xacc = (float) -1.08380624E36F;
            p108.yacc = (float) -3.020139E38F;
            p108.zacc = (float) -2.3316573E38F;
            p108.xgyro = (float) -2.6597247E38F;
            p108.ygyro = (float)3.196135E38F;
            p108.zgyro = (float) -2.4722541E38F;
            p108.lat = (float) -1.8193344E38F;
            p108.lon = (float)2.5011456E38F;
            p108.alt = (float)1.4052084E38F;
            p108.std_dev_horz = (float) -2.4091702E38F;
            p108.std_dev_vert = (float)1.8220342E38F;
            p108.vn = (float)7.947596E37F;
            p108.ve = (float)2.1950463E38F;
            p108.vd = (float) -1.9507325E38F;
            CommunicationChannel.instance.send(p108); //===============================
            RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.rssi = (byte)(byte)17;
            p109.remrssi = (byte)(byte)199;
            p109.txbuf = (byte)(byte)58;
            p109.noise = (byte)(byte)120;
            p109.remnoise = (byte)(byte)54;
            p109.rxerrors = (ushort)(ushort)11479;
            p109.fixed_ = (ushort)(ushort)24650;
            CommunicationChannel.instance.send(p109); //===============================
            FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_network = (byte)(byte)93;
            p110.target_system = (byte)(byte)247;
            p110.target_component = (byte)(byte)81;
            p110.payload_SET(new byte[251], 0);
            CommunicationChannel.instance.send(p110); //===============================
            TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.tc1 = (long)2379890592984174917L;
            p111.ts1 = (long) -6643050593349613527L;
            CommunicationChannel.instance.send(p111); //===============================
            CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.time_usec = (ulong)4099245379357772059L;
            p112.seq = (uint)3719212287U;
            CommunicationChannel.instance.send(p112); //===============================
            HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.time_usec = (ulong)1731397767018765403L;
            p113.fix_type = (byte)(byte)164;
            p113.lat = (int)44958382;
            p113.lon = (int) -1279705385;
            p113.alt = (int) -191053287;
            p113.eph = (ushort)(ushort)7398;
            p113.epv = (ushort)(ushort)64083;
            p113.vel = (ushort)(ushort)4878;
            p113.vn = (short)(short)10930;
            p113.ve = (short)(short)7693;
            p113.vd = (short)(short) -21718;
            p113.cog = (ushort)(ushort)7010;
            p113.satellites_visible = (byte)(byte)91;
            CommunicationChannel.instance.send(p113); //===============================
            HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.time_usec = (ulong)6429763870217150659L;
            p114.sensor_id = (byte)(byte)19;
            p114.integration_time_us = (uint)1184147439U;
            p114.integrated_x = (float)1.762486E38F;
            p114.integrated_y = (float)1.949263E38F;
            p114.integrated_xgyro = (float)2.5805287E38F;
            p114.integrated_ygyro = (float) -6.2923945E37F;
            p114.integrated_zgyro = (float) -3.0263452E38F;
            p114.temperature = (short)(short) -32603;
            p114.quality = (byte)(byte)189;
            p114.time_delta_distance_us = (uint)9827957U;
            p114.distance = (float)3.3487228E38F;
            CommunicationChannel.instance.send(p114); //===============================
            HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.time_usec = (ulong)3079243790872570105L;
            p115.attitude_quaternion_SET(new float[4], 0);
            p115.rollspeed = (float)2.8575165E38F;
            p115.pitchspeed = (float)1.7726205E38F;
            p115.yawspeed = (float) -7.6893246E37F;
            p115.lat = (int)1676960210;
            p115.lon = (int)1518770729;
            p115.alt = (int)1969761809;
            p115.vx = (short)(short)5749;
            p115.vy = (short)(short) -14463;
            p115.vz = (short)(short)29637;
            p115.ind_airspeed = (ushort)(ushort)19222;
            p115.true_airspeed = (ushort)(ushort)24352;
            p115.xacc = (short)(short) -10517;
            p115.yacc = (short)(short)12406;
            p115.zacc = (short)(short) -21933;
            CommunicationChannel.instance.send(p115); //===============================
            SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.time_boot_ms = (uint)3841768895U;
            p116.xacc = (short)(short) -13673;
            p116.yacc = (short)(short)3579;
            p116.zacc = (short)(short) -23050;
            p116.xgyro = (short)(short)2840;
            p116.ygyro = (short)(short)6485;
            p116.zgyro = (short)(short)13071;
            p116.xmag = (short)(short)9925;
            p116.ymag = (short)(short) -28609;
            p116.zmag = (short)(short)25660;
            CommunicationChannel.instance.send(p116); //===============================
            LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.target_system = (byte)(byte)182;
            p117.target_component = (byte)(byte)5;
            p117.start = (ushort)(ushort)65233;
            p117.end = (ushort)(ushort)60349;
            CommunicationChannel.instance.send(p117); //===============================
            LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.id = (ushort)(ushort)26862;
            p118.num_logs = (ushort)(ushort)5027;
            p118.last_log_num = (ushort)(ushort)58731;
            p118.time_utc = (uint)1444150176U;
            p118.size = (uint)928562025U;
            CommunicationChannel.instance.send(p118); //===============================
            LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.target_system = (byte)(byte)204;
            p119.target_component = (byte)(byte)44;
            p119.id = (ushort)(ushort)43074;
            p119.ofs = (uint)3134088498U;
            p119.count = (uint)2228289U;
            CommunicationChannel.instance.send(p119); //===============================
            LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.id = (ushort)(ushort)11044;
            p120.ofs = (uint)1871541400U;
            p120.count = (byte)(byte)226;
            p120.data__SET(new byte[90], 0);
            CommunicationChannel.instance.send(p120); //===============================
            LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_system = (byte)(byte)68;
            p121.target_component = (byte)(byte)246;
            CommunicationChannel.instance.send(p121); //===============================
            LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_system = (byte)(byte)89;
            p122.target_component = (byte)(byte)67;
            CommunicationChannel.instance.send(p122); //===============================
            GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.target_system = (byte)(byte)69;
            p123.target_component = (byte)(byte)121;
            p123.len = (byte)(byte)93;
            p123.data__SET(new byte[110], 0);
            CommunicationChannel.instance.send(p123); //===============================
            GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.time_usec = (ulong)3421119847691431777L;
            p124.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT;
            p124.lat = (int) -1663444498;
            p124.lon = (int)1414961217;
            p124.alt = (int)222018480;
            p124.eph = (ushort)(ushort)18949;
            p124.epv = (ushort)(ushort)28313;
            p124.vel = (ushort)(ushort)57487;
            p124.cog = (ushort)(ushort)13205;
            p124.satellites_visible = (byte)(byte)93;
            p124.dgps_numch = (byte)(byte)195;
            p124.dgps_age = (uint)310920143U;
            CommunicationChannel.instance.send(p124); //===============================
            POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vcc = (ushort)(ushort)4068;
            p125.Vservo = (ushort)(ushort)2512;
            p125.flags = (MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT |
                          MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED |
                          MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID |
                          MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED);
            CommunicationChannel.instance.send(p125); //===============================
            SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.device = SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS2;
            p126.flags = (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING |
                          SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI |
                          SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY);
            p126.timeout = (ushort)(ushort)33311;
            p126.baudrate = (uint)1177553835U;
            p126.count = (byte)(byte)244;
            p126.data__SET(new byte[70], 0);
            CommunicationChannel.instance.send(p126); //===============================
            GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.time_last_baseline_ms = (uint)3987770193U;
            p127.rtk_receiver_id = (byte)(byte)153;
            p127.wn = (ushort)(ushort)16684;
            p127.tow = (uint)1977155232U;
            p127.rtk_health = (byte)(byte)229;
            p127.rtk_rate = (byte)(byte)190;
            p127.nsats = (byte)(byte)14;
            p127.baseline_coords_type = (byte)(byte)247;
            p127.baseline_a_mm = (int) -302777029;
            p127.baseline_b_mm = (int) -1100723846;
            p127.baseline_c_mm = (int) -1415779680;
            p127.accuracy = (uint)396502368U;
            p127.iar_num_hypotheses = (int) -1110740270;
            CommunicationChannel.instance.send(p127); //===============================
            GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.time_last_baseline_ms = (uint)3038218955U;
            p128.rtk_receiver_id = (byte)(byte)26;
            p128.wn = (ushort)(ushort)21768;
            p128.tow = (uint)2120903222U;
            p128.rtk_health = (byte)(byte)163;
            p128.rtk_rate = (byte)(byte)215;
            p128.nsats = (byte)(byte)8;
            p128.baseline_coords_type = (byte)(byte)187;
            p128.baseline_a_mm = (int)345014170;
            p128.baseline_b_mm = (int)2050531074;
            p128.baseline_c_mm = (int)1703097807;
            p128.accuracy = (uint)1608329539U;
            p128.iar_num_hypotheses = (int)17357040;
            CommunicationChannel.instance.send(p128); //===============================
            SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.time_boot_ms = (uint)3853274546U;
            p129.xacc = (short)(short) -29945;
            p129.yacc = (short)(short) -29063;
            p129.zacc = (short)(short) -22847;
            p129.xgyro = (short)(short) -17812;
            p129.ygyro = (short)(short) -11422;
            p129.zgyro = (short)(short)15722;
            p129.xmag = (short)(short) -12193;
            p129.ymag = (short)(short) -15178;
            p129.zmag = (short)(short)6553;
            CommunicationChannel.instance.send(p129); //===============================
            DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.type = (byte)(byte)159;
            p130.size = (uint)2913015274U;
            p130.width = (ushort)(ushort)20836;
            p130.height = (ushort)(ushort)33507;
            p130.packets = (ushort)(ushort)13579;
            p130.payload = (byte)(byte)147;
            p130.jpg_quality = (byte)(byte)231;
            CommunicationChannel.instance.send(p130); //===============================
            ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.seqnr = (ushort)(ushort)31602;
            p131.data__SET(new byte[253], 0);
            CommunicationChannel.instance.send(p131); //===============================
            DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.time_boot_ms = (uint)1812616392U;
            p132.min_distance = (ushort)(ushort)17341;
            p132.max_distance = (ushort)(ushort)55387;
            p132.current_distance = (ushort)(ushort)45962;
            p132.type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND;
            p132.id = (byte)(byte)177;
            p132.orientation = MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_90;
            p132.covariance = (byte)(byte)197;
            CommunicationChannel.instance.send(p132); //===============================
            TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.lat = (int) -593566693;
            p133.lon = (int) -860310690;
            p133.grid_spacing = (ushort)(ushort)27410;
            p133.mask = (ulong)6721022404552971809L;
            CommunicationChannel.instance.send(p133); //===============================
            TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.lat = (int) -1012539038;
            p134.lon = (int) -12154303;
            p134.grid_spacing = (ushort)(ushort)44352;
            p134.gridbit = (byte)(byte)243;
            p134.data__SET(new short[16], 0);
            CommunicationChannel.instance.send(p134); //===============================
            TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lat = (int)819450440;
            p135.lon = (int) -329449942;
            CommunicationChannel.instance.send(p135); //===============================
            TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.lat = (int)400939442;
            p136.lon = (int) -18225697;
            p136.spacing = (ushort)(ushort)10799;
            p136.terrain_height = (float) -2.8602812E38F;
            p136.current_height = (float)1.1365546E38F;
            p136.pending = (ushort)(ushort)22392;
            p136.loaded = (ushort)(ushort)16975;
            CommunicationChannel.instance.send(p136); //===============================
            SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.time_boot_ms = (uint)2865510457U;
            p137.press_abs = (float)1.2197453E38F;
            p137.press_diff = (float) -2.0211285E38F;
            p137.temperature = (short)(short) -8910;
            CommunicationChannel.instance.send(p137); //===============================
            ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.time_usec = (ulong)8130711337286017196L;
            p138.q_SET(new float[4], 0);
            p138.x = (float) -6.924982E36F;
            p138.y = (float) -1.3037336E37F;
            p138.z = (float)4.255962E37F;
            CommunicationChannel.instance.send(p138); //===============================
            SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.time_usec = (ulong)4592125085738516365L;
            p139.group_mlx = (byte)(byte)194;
            p139.target_system = (byte)(byte)37;
            p139.target_component = (byte)(byte)181;
            p139.controls_SET(new float[8], 0);
            CommunicationChannel.instance.send(p139); //===============================
            ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.time_usec = (ulong)1773760474167449433L;
            p140.group_mlx = (byte)(byte)121;
            p140.controls_SET(new float[8], 0);
            CommunicationChannel.instance.send(p140); //===============================
            ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.time_usec = (ulong)3481729793264153259L;
            p141.altitude_monotonic = (float)1.7011883E38F;
            p141.altitude_amsl = (float)1.1402186E37F;
            p141.altitude_local = (float) -2.2989233E38F;
            p141.altitude_relative = (float) -1.3966196E38F;
            p141.altitude_terrain = (float) -1.6683939E38F;
            p141.bottom_clearance = (float)5.704765E37F;
            CommunicationChannel.instance.send(p141); //===============================
            RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.request_id = (byte)(byte)14;
            p142.uri_type = (byte)(byte)73;
            p142.uri_SET(new byte[120], 0);
            p142.transfer_type = (byte)(byte)105;
            p142.storage_SET(new byte[120], 0);
            CommunicationChannel.instance.send(p142); //===============================
            SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.time_boot_ms = (uint)2878250536U;
            p143.press_abs = (float) -1.0744893E38F;
            p143.press_diff = (float)3.1994327E38F;
            p143.temperature = (short)(short)2702;
            CommunicationChannel.instance.send(p143); //===============================
            FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.timestamp = (ulong)1854470544354866797L;
            p144.est_capabilities = (byte)(byte)70;
            p144.lat = (int)1188674256;
            p144.lon = (int) -302336534;
            p144.alt = (float) -1.916467E38F;
            p144.vel_SET(new float[3], 0);
            p144.acc_SET(new float[3], 0);
            p144.attitude_q_SET(new float[4], 0);
            p144.rates_SET(new float[3], 0);
            p144.position_cov_SET(new float[3], 0);
            p144.custom_state = (ulong)5389648329854449304L;
            CommunicationChannel.instance.send(p144); //===============================
            CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.time_usec = (ulong)8103970802570867079L;
            p146.x_acc = (float)4.5945185E37F;
            p146.y_acc = (float)1.3167169E38F;
            p146.z_acc = (float)3.652369E37F;
            p146.x_vel = (float)1.7475215E38F;
            p146.y_vel = (float)1.1565637E38F;
            p146.z_vel = (float) -2.9635531E38F;
            p146.x_pos = (float)1.937408E38F;
            p146.y_pos = (float) -2.3987746E38F;
            p146.z_pos = (float) -1.6738472E38F;
            p146.airspeed = (float)3.2820983E38F;
            p146.vel_variance_SET(new float[3], 0);
            p146.pos_variance_SET(new float[3], 0);
            p146.q_SET(new float[4], 0);
            p146.roll_rate = (float)1.0277062E38F;
            p146.pitch_rate = (float) -1.6782646E38F;
            p146.yaw_rate = (float)1.7868716E38F;
            CommunicationChannel.instance.send(p146); //===============================
            BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.id = (byte)(byte)8;
            p147.battery_function = MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_PROPULSION;
            p147.type = MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_NIMH;
            p147.temperature = (short)(short)22491;
            p147.voltages_SET(new ushort[10], 0);
            p147.current_battery = (short)(short)10386;
            p147.current_consumed = (int) -1073518189;
            p147.energy_consumed = (int)506581861;
            p147.battery_remaining = (sbyte)(sbyte) - 125;
            CommunicationChannel.instance.send(p147); //===============================
            AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_TERRAIN |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMMAND_INT);
            p148.flight_sw_version = (uint)1963976492U;
            p148.middleware_sw_version = (uint)2551778576U;
            p148.os_sw_version = (uint)2960600120U;
            p148.board_version = (uint)1258275399U;
            p148.flight_custom_version_SET(new byte[8], 0);
            p148.middleware_custom_version_SET(new byte[8], 0);
            p148.os_custom_version_SET(new byte[8], 0);
            p148.vendor_id = (ushort)(ushort)41550;
            p148.product_id = (ushort)(ushort)6546;
            p148.uid = (ulong)1689692354007496690L;
            p148.uid2_SET(new byte[18], 0, PH);
            CommunicationChannel.instance.send(p148); //===============================
            LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.time_usec = (ulong)1417355660940407931L;
            p149.target_num = (byte)(byte)27;
            p149.frame = MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p149.angle_x = (float) -4.158624E37F;
            p149.angle_y = (float)2.9904474E38F;
            p149.distance = (float) -1.1313441E38F;
            p149.size_x = (float) -2.6640543E38F;
            p149.size_y = (float) -2.4291295E38F;
            p149.x_SET((float)2.6812863E38F, PH);
            p149.y_SET((float) -2.355677E37F, PH);
            p149.z_SET((float)2.9219906E38F, PH);
            p149.q_SET(new float[4], 0, PH);
            p149.type = LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_FIDUCIAL;
            p149.position_valid_SET((byte)(byte)19, PH);
            CommunicationChannel.instance.send(p149); //===============================
            AQ_TELEMETRY_F p150 = CommunicationChannel.new_AQ_TELEMETRY_F();
            PH.setPack(p150);
            p150.Index = (ushort)(ushort)51151;
            p150.value1 = (float)3.2267308E38F;
            p150.value2 = (float) -9.199361E36F;
            p150.value3 = (float)1.2871958E38F;
            p150.value4 = (float)3.0987573E38F;
            p150.value5 = (float)2.6728165E38F;
            p150.value6 = (float) -1.5791453E38F;
            p150.value7 = (float) -3.1604578E38F;
            p150.value8 = (float) -1.207613E38F;
            p150.value9 = (float)2.0054318E38F;
            p150.value10 = (float) -1.3357435E38F;
            p150.value11 = (float)2.1340482E38F;
            p150.value12 = (float)1.2848919E38F;
            p150.value13 = (float)3.0314187E38F;
            p150.value14 = (float) -2.2798159E37F;
            p150.value15 = (float)1.2024297E38F;
            p150.value16 = (float)5.7744294E37F;
            p150.value17 = (float) -3.303754E38F;
            p150.value18 = (float)3.251235E38F;
            p150.value19 = (float) -1.3959274E38F;
            p150.value20 = (float)4.8639503E36F;
            CommunicationChannel.instance.send(p150); //===============================
            AQ_ESC_TELEMETRY p152 = CommunicationChannel.new_AQ_ESC_TELEMETRY();
            PH.setPack(p152);
            p152.time_boot_ms = (uint)3295392457U;
            p152.seq = (byte)(byte)89;
            p152.num_motors = (byte)(byte)149;
            p152.num_in_seq = (byte)(byte)237;
            p152.escid_SET(new byte[4], 0);
            p152.status_age_SET(new ushort[4], 0);
            p152.data_version_SET(new byte[4], 0);
            p152.data0_SET(new uint[4], 0);
            p152.data1_SET(new uint[4], 0);
            CommunicationChannel.instance.send(p152); //===============================
            ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.time_usec = (ulong)7133404402876879216L;
            p230.flags = (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS);
            p230.vel_ratio = (float)1.4295865E38F;
            p230.pos_horiz_ratio = (float)6.7821107E37F;
            p230.pos_vert_ratio = (float) -8.678819E37F;
            p230.mag_ratio = (float)1.1268072E38F;
            p230.hagl_ratio = (float)1.847304E38F;
            p230.tas_ratio = (float) -4.42569E36F;
            p230.pos_horiz_accuracy = (float) -9.927928E37F;
            p230.pos_vert_accuracy = (float) -8.887087E37F;
            CommunicationChannel.instance.send(p230); //===============================
            WIND_COV p231 = CommunicationChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.time_usec = (ulong)405889542341831843L;
            p231.wind_x = (float)3.1037567E38F;
            p231.wind_y = (float) -2.1059127E38F;
            p231.wind_z = (float) -1.817945E38F;
            p231.var_horiz = (float)1.3350497E38F;
            p231.var_vert = (float)2.8350821E38F;
            p231.wind_alt = (float) -2.391329E37F;
            p231.horiz_accuracy = (float)2.5229843E38F;
            p231.vert_accuracy = (float) -1.5349035E38F;
            CommunicationChannel.instance.send(p231); //===============================
            GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.time_usec = (ulong)2642077811687766143L;
            p232.gps_id = (byte)(byte)221;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY);
            p232.time_week_ms = (uint)3047717492U;
            p232.time_week = (ushort)(ushort)25561;
            p232.fix_type = (byte)(byte)70;
            p232.lat = (int) -626480254;
            p232.lon = (int)1609462116;
            p232.alt = (float) -3.24495E38F;
            p232.hdop = (float) -2.3059094E38F;
            p232.vdop = (float)1.5735484E38F;
            p232.vn = (float)1.9304584E37F;
            p232.ve = (float) -1.8292701E38F;
            p232.vd = (float) -3.1356488E38F;
            p232.speed_accuracy = (float)2.142891E38F;
            p232.horiz_accuracy = (float) -2.6629293E38F;
            p232.vert_accuracy = (float)2.4302748E38F;
            p232.satellites_visible = (byte)(byte)13;
            CommunicationChannel.instance.send(p232); //===============================
            GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.flags = (byte)(byte)135;
            p233.len = (byte)(byte)103;
            p233.data__SET(new byte[180], 0);
            CommunicationChannel.instance.send(p233); //===============================
            HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED);
            p234.custom_mode = (uint)616493049U;
            p234.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND;
            p234.roll = (short)(short)25471;
            p234.pitch = (short)(short) -11182;
            p234.heading = (ushort)(ushort)37485;
            p234.throttle = (sbyte)(sbyte) - 112;
            p234.heading_sp = (short)(short) -798;
            p234.latitude = (int) -1234961286;
            p234.longitude = (int) -673538264;
            p234.altitude_amsl = (short)(short)799;
            p234.altitude_sp = (short)(short) -17119;
            p234.airspeed = (byte)(byte)206;
            p234.airspeed_sp = (byte)(byte)137;
            p234.groundspeed = (byte)(byte)71;
            p234.climb_rate = (sbyte)(sbyte)7;
            p234.gps_nsat = (byte)(byte)13;
            p234.gps_fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED;
            p234.battery_remaining = (byte)(byte)63;
            p234.temperature = (sbyte)(sbyte)118;
            p234.temperature_air = (sbyte)(sbyte) - 81;
            p234.failsafe = (byte)(byte)106;
            p234.wp_num = (byte)(byte)22;
            p234.wp_distance = (ushort)(ushort)26289;
            CommunicationChannel.instance.send(p234); //===============================
            VIBRATION p241 = CommunicationChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.time_usec = (ulong)5831204664759403229L;
            p241.vibration_x = (float) -2.0441458E38F;
            p241.vibration_y = (float)4.1966402E37F;
            p241.vibration_z = (float) -1.3149007E38F;
            p241.clipping_0 = (uint)3017445236U;
            p241.clipping_1 = (uint)1610494816U;
            p241.clipping_2 = (uint)1934206358U;
            CommunicationChannel.instance.send(p241); //===============================
            HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.latitude = (int)35054200;
            p242.longitude = (int) -1427161257;
            p242.altitude = (int)808073526;
            p242.x = (float)1.5869958E38F;
            p242.y = (float)1.8622056E37F;
            p242.z = (float) -1.18308E38F;
            p242.q_SET(new float[4], 0);
            p242.approach_x = (float) -4.167831E37F;
            p242.approach_y = (float)8.641526E34F;
            p242.approach_z = (float)2.3009274E38F;
            p242.time_usec_SET((ulong)7837302264722172024L, PH);
            CommunicationChannel.instance.send(p242); //===============================
            SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.target_system = (byte)(byte)18;
            p243.latitude = (int) -796010915;
            p243.longitude = (int)1719169635;
            p243.altitude = (int)13362695;
            p243.x = (float)9.653798E36F;
            p243.y = (float)6.6915934E37F;
            p243.z = (float) -1.2572604E38F;
            p243.q_SET(new float[4], 0);
            p243.approach_x = (float)1.3027806E38F;
            p243.approach_y = (float) -2.5346842E38F;
            p243.approach_z = (float)2.3441375E37F;
            p243.time_usec_SET((ulong)6659710658610310694L, PH);
            CommunicationChannel.instance.send(p243); //===============================
            MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.message_id = (ushort)(ushort)21674;
            p244.interval_us = (int)1065564626;
            CommunicationChannel.instance.send(p244); //===============================
            EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.vtol_state = MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_FW;
            p245.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR;
            CommunicationChannel.instance.send(p245); //===============================
            ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.ICAO_address = (uint)209516465U;
            p246.lat = (int)1727419964;
            p246.lon = (int)904668897;
            p246.altitude_type = ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC;
            p246.altitude = (int) -1911707910;
            p246.heading = (ushort)(ushort)11520;
            p246.hor_velocity = (ushort)(ushort)21953;
            p246.ver_velocity = (short)(short) -12115;
            p246.callsign_SET("DEMO", PH);
            p246.emitter_type = ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_EMERGENCY_SURFACE;
            p246.tslc = (byte)(byte)104;
            p246.flags = (ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING |
                          ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS);
            p246.squawk = (ushort)(ushort)51209;
            CommunicationChannel.instance.send(p246); //===============================
            COLLISION p247 = CommunicationChannel.new_COLLISION();
            PH.setPack(p247);
            p247.src_ = MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB;
            p247.id = (uint)2536499451U;
            p247.action = MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_PERPENDICULAR;
            p247.threat_level = (MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE |
                                 MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH);
            p247.time_to_minimum_delta = (float)5.141032E37F;
            p247.altitude_minimum_delta = (float) -3.2247064E38F;
            p247.horizontal_minimum_delta = (float)1.2639924E38F;
            CommunicationChannel.instance.send(p247); //===============================
            V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.target_network = (byte)(byte)1;
            p248.target_system = (byte)(byte)121;
            p248.target_component = (byte)(byte)215;
            p248.message_type = (ushort)(ushort)8335;
            p248.payload_SET(new byte[249], 0);
            CommunicationChannel.instance.send(p248); //===============================
            MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.address = (ushort)(ushort)38120;
            p249.ver = (byte)(byte)26;
            p249.type = (byte)(byte)194;
            p249.value_SET(new sbyte[32], 0);
            CommunicationChannel.instance.send(p249); //===============================
            DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.name_SET("DEMO", PH);
            p250.time_usec = (ulong)1880903634373495086L;
            p250.x = (float) -2.2266149E38F;
            p250.y = (float)8.204201E37F;
            p250.z = (float)1.7080616E38F;
            CommunicationChannel.instance.send(p250); //===============================
            NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.time_boot_ms = (uint)3438456748U;
            p251.name_SET("DEMO", PH);
            p251.value = (float)2.8228386E38F;
            CommunicationChannel.instance.send(p251); //===============================
            NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.time_boot_ms = (uint)238670801U;
            p252.name_SET("DEMO", PH);
            p252.value = (int)1955644797;
            CommunicationChannel.instance.send(p252); //===============================
            STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.severity = MAV_SEVERITY.MAV_SEVERITY_ERROR;
            p253.text_SET("DEMO", PH);
            CommunicationChannel.instance.send(p253); //===============================
            DEBUG p254 = CommunicationChannel.new_DEBUG();
            PH.setPack(p254);
            p254.time_boot_ms = (uint)1739540896U;
            p254.ind = (byte)(byte)197;
            p254.value = (float)3.9570672E37F;
            CommunicationChannel.instance.send(p254); //===============================
            SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.target_system = (byte)(byte)101;
            p256.target_component = (byte)(byte)83;
            p256.secret_key_SET(new byte[32], 0);
            p256.initial_timestamp = (ulong)3145387189366761611L;
            CommunicationChannel.instance.send(p256); //===============================
            BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.time_boot_ms = (uint)1092809346U;
            p257.last_change_ms = (uint)2637330163U;
            p257.state = (byte)(byte)249;
            CommunicationChannel.instance.send(p257); //===============================
            PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_system = (byte)(byte)94;
            p258.target_component = (byte)(byte)196;
            p258.tune_SET("DEMO", PH);
            CommunicationChannel.instance.send(p258); //===============================
            CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.time_boot_ms = (uint)3112695758U;
            p259.vendor_name_SET(new byte[32], 0);
            p259.model_name_SET(new byte[32], 0);
            p259.firmware_version = (uint)640406498U;
            p259.focal_length = (float)4.284545E37F;
            p259.sensor_size_h = (float)1.1160198E38F;
            p259.sensor_size_v = (float)1.4759542E38F;
            p259.resolution_h = (ushort)(ushort)5741;
            p259.resolution_v = (ushort)(ushort)20310;
            p259.lens_id = (byte)(byte)218;
            p259.flags = (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE);
            p259.cam_definition_version = (ushort)(ushort)47758;
            p259.cam_definition_uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p259); //===============================
            CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.time_boot_ms = (uint)2896244284U;
            p260.mode_id = (CAMERA_MODE.CAMERA_MODE_IMAGE |
                            CAMERA_MODE.CAMERA_MODE_VIDEO);
            CommunicationChannel.instance.send(p260); //===============================
            STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.time_boot_ms = (uint)788271440U;
            p261.storage_id = (byte)(byte)114;
            p261.storage_count = (byte)(byte)159;
            p261.status = (byte)(byte)160;
            p261.total_capacity = (float) -2.2250714E38F;
            p261.used_capacity = (float) -2.5625096E38F;
            p261.available_capacity = (float) -1.3126215E38F;
            p261.read_speed = (float)6.147345E37F;
            p261.write_speed = (float) -1.8758971E38F;
            CommunicationChannel.instance.send(p261); //===============================
            CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.time_boot_ms = (uint)2298151546U;
            p262.image_status = (byte)(byte)122;
            p262.video_status = (byte)(byte)120;
            p262.image_interval = (float) -5.049244E37F;
            p262.recording_time_ms = (uint)3525762569U;
            p262.available_capacity = (float)1.3967362E38F;
            CommunicationChannel.instance.send(p262); //===============================
            CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.time_boot_ms = (uint)1465038974U;
            p263.time_utc = (ulong)8704566215484763801L;
            p263.camera_id = (byte)(byte)236;
            p263.lat = (int) -1751419291;
            p263.lon = (int)427226132;
            p263.alt = (int)1570923138;
            p263.relative_alt = (int) -77061268;
            p263.q_SET(new float[4], 0);
            p263.image_index = (int)1259986690;
            p263.capture_result = (sbyte)(sbyte)18;
            p263.file_url_SET("DEMO", PH);
            CommunicationChannel.instance.send(p263); //===============================
            FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.time_boot_ms = (uint)1821064776U;
            p264.arming_time_utc = (ulong)1652241910215192184L;
            p264.takeoff_time_utc = (ulong)2688652927212764065L;
            p264.flight_uuid = (ulong)4541261400495886252L;
            CommunicationChannel.instance.send(p264); //===============================
            MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.time_boot_ms = (uint)4093930994U;
            p265.roll = (float) -9.13638E37F;
            p265.pitch = (float)3.2326939E37F;
            p265.yaw = (float) -4.8947473E37F;
            CommunicationChannel.instance.send(p265); //===============================
            LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.target_system = (byte)(byte)33;
            p266.target_component = (byte)(byte)13;
            p266.sequence = (ushort)(ushort)59876;
            p266.length = (byte)(byte)129;
            p266.first_message_offset = (byte)(byte)91;
            p266.data__SET(new byte[249], 0);
            CommunicationChannel.instance.send(p266); //===============================
            LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.target_system = (byte)(byte)70;
            p267.target_component = (byte)(byte)5;
            p267.sequence = (ushort)(ushort)2396;
            p267.length = (byte)(byte)130;
            p267.first_message_offset = (byte)(byte)206;
            p267.data__SET(new byte[249], 0);
            CommunicationChannel.instance.send(p267); //===============================
            LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_system = (byte)(byte)205;
            p268.target_component = (byte)(byte)27;
            p268.sequence = (ushort)(ushort)20647;
            CommunicationChannel.instance.send(p268); //===============================
            VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.camera_id = (byte)(byte)41;
            p269.status = (byte)(byte)12;
            p269.framerate = (float) -4.419557E37F;
            p269.resolution_h = (ushort)(ushort)32900;
            p269.resolution_v = (ushort)(ushort)6768;
            p269.bitrate = (uint)3726253900U;
            p269.rotation = (ushort)(ushort)20267;
            p269.uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p269); //===============================
            SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.target_system = (byte)(byte)214;
            p270.target_component = (byte)(byte)211;
            p270.camera_id = (byte)(byte)180;
            p270.framerate = (float) -1.6059951E38F;
            p270.resolution_h = (ushort)(ushort)41968;
            p270.resolution_v = (ushort)(ushort)51715;
            p270.bitrate = (uint)2427697937U;
            p270.rotation = (ushort)(ushort)64847;
            p270.uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p270); //===============================
            WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.ssid_SET("DEMO", PH);
            p299.password_SET("DEMO", PH);
            CommunicationChannel.instance.send(p299); //===============================
            PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.version = (ushort)(ushort)46510;
            p300.min_version = (ushort)(ushort)51247;
            p300.max_version = (ushort)(ushort)3140;
            p300.spec_version_hash_SET(new byte[8], 0);
            p300.library_version_hash_SET(new byte[8], 0);
            CommunicationChannel.instance.send(p300); //===============================
            UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.time_usec = (ulong)2291990231891391412L;
            p310.uptime_sec = (uint)1251731745U;
            p310.health = UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK;
            p310.mode = UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OPERATIONAL;
            p310.sub_mode = (byte)(byte)236;
            p310.vendor_specific_status_code = (ushort)(ushort)56840;
            CommunicationChannel.instance.send(p310); //===============================
            UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.time_usec = (ulong)321054422401319239L;
            p311.uptime_sec = (uint)3348520115U;
            p311.name_SET("DEMO", PH);
            p311.hw_version_major = (byte)(byte)158;
            p311.hw_version_minor = (byte)(byte)229;
            p311.hw_unique_id_SET(new byte[16], 0);
            p311.sw_version_major = (byte)(byte)2;
            p311.sw_version_minor = (byte)(byte)81;
            p311.sw_vcs_commit = (uint)3894547142U;
            CommunicationChannel.instance.send(p311); //===============================
            PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.target_system = (byte)(byte)168;
            p320.target_component = (byte)(byte)97;
            p320.param_id_SET("DEMO", PH);
            p320.param_index = (short)(short)31990;
            CommunicationChannel.instance.send(p320); //===============================
            PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_system = (byte)(byte)244;
            p321.target_component = (byte)(byte)222;
            CommunicationChannel.instance.send(p321); //===============================
            PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_id_SET("DEMO", PH);
            p322.param_value_SET("DEMO", PH);
            p322.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16;
            p322.param_count = (ushort)(ushort)53999;
            p322.param_index = (ushort)(ushort)32322;
            CommunicationChannel.instance.send(p322); //===============================
            PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.target_system = (byte)(byte)139;
            p323.target_component = (byte)(byte)247;
            p323.param_id_SET("DEMO", PH);
            p323.param_value_SET("DEMO", PH);
            p323.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM;
            CommunicationChannel.instance.send(p323); //===============================
            PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_id_SET("DEMO", PH);
            p324.param_value_SET("DEMO", PH);
            p324.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32;
            p324.param_result = PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED;
            CommunicationChannel.instance.send(p324); //===============================
            OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.time_usec = (ulong)7221258471869149024L;
            p330.sensor_type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN;
            p330.distances_SET(new ushort[72], 0);
            p330.increment = (byte)(byte)208;
            p330.min_distance = (ushort)(ushort)56825;
            p330.max_distance = (ushort)(ushort)9896;
            CommunicationChannel.instance.send(p330); //===============================
        }
    }
}

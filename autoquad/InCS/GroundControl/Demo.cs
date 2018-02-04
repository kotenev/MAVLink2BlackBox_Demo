
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
            POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.time_boot_ms = (uint)3108886492U;
            p3.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT;
            p3.type_mask = (ushort)(ushort)16020;
            p3.x = (float) -1.8077219E38F;
            p3.y = (float) -1.6043883E38F;
            p3.z = (float)1.7072098E38F;
            p3.vx = (float)6.626549E37F;
            p3.vy = (float)5.3165413E37F;
            p3.vz = (float)2.5270158E38F;
            p3.afx = (float) -2.4556516E38F;
            p3.afy = (float) -2.9625325E38F;
            p3.afz = (float)2.8581102E37F;
            p3.yaw = (float) -3.045441E38F;
            p3.yaw_rate = (float)2.1615082E38F;
            CommunicationChannel.instance.send(p3); //===============================
            VFR_HUD p74 = CommunicationChannel.new_VFR_HUD();
            PH.setPack(p74);
            p74.airspeed = (float) -1.7920758E38F;
            p74.groundspeed = (float) -1.398158E38F;
            p74.heading = (short)(short) -20503;
            p74.throttle = (ushort)(ushort)11597;
            p74.alt = (float)4.3325286E37F;
            p74.climb = (float) -9.641611E37F;
            CommunicationChannel.instance.send(p74); //===============================
            COMMAND_INT p75 = CommunicationChannel.new_COMMAND_INT();
            PH.setPack(p75);
            p75.target_system = (byte)(byte)83;
            p75.target_component = (byte)(byte)166;
            p75.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_INT;
            p75.command = (MAV_CMD)MAV_CMD.MAV_CMD_DO_SET_SERVO;
            p75.current = (byte)(byte)28;
            p75.autocontinue = (byte)(byte)123;
            p75.param1 = (float) -1.7624254E38F;
            p75.param2 = (float) -3.0134965E38F;
            p75.param3 = (float) -2.559948E38F;
            p75.param4 = (float) -1.9370389E38F;
            p75.x = (int) -1131798392;
            p75.y = (int)1096156178;
            p75.z = (float)1.5646782E37F;
            CommunicationChannel.instance.send(p75); //===============================
            COMMAND_LONG p76 = CommunicationChannel.new_COMMAND_LONG();
            PH.setPack(p76);
            p76.target_system = (byte)(byte)6;
            p76.target_component = (byte)(byte)199;
            p76.command = (MAV_CMD)MAV_CMD.MAV_CMD_DO_GUIDED_MASTER;
            p76.confirmation = (byte)(byte)255;
            p76.param1 = (float)2.6638385E38F;
            p76.param2 = (float) -2.3882111E38F;
            p76.param3 = (float)1.3913016E38F;
            p76.param4 = (float)1.4881839E38F;
            p76.param5 = (float) -2.588995E38F;
            p76.param6 = (float) -1.3257153E38F;
            p76.param7 = (float)2.1301033E38F;
            CommunicationChannel.instance.send(p76); //===============================
            COMMAND_ACK p77 = CommunicationChannel.new_COMMAND_ACK();
            PH.setPack(p77);
            p77.command = (MAV_CMD)MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS;
            p77.result = (MAV_RESULT)MAV_RESULT.MAV_RESULT_FAILED;
            p77.progress_SET((byte)(byte)56, PH);
            p77.result_param2_SET((int)1091856570, PH);
            p77.target_system_SET((byte)(byte)251, PH);
            p77.target_component_SET((byte)(byte)44, PH);
            CommunicationChannel.instance.send(p77); //===============================
            MANUAL_SETPOINT p81 = CommunicationChannel.new_MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.time_boot_ms = (uint)471449683U;
            p81.roll = (float)1.5517607E38F;
            p81.pitch = (float)1.5869446E38F;
            p81.yaw = (float)9.719898E37F;
            p81.thrust = (float)1.5295031E38F;
            p81.mode_switch = (byte)(byte)73;
            p81.manual_override_switch = (byte)(byte)88;
            CommunicationChannel.instance.send(p81); //===============================
            SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.time_boot_ms = (uint)3621305273U;
            p82.target_system = (byte)(byte)79;
            p82.target_component = (byte)(byte)96;
            p82.type_mask = (byte)(byte)72;
            p82.q_SET(new float[4], 0);
            p82.body_roll_rate = (float) -3.391015E38F;
            p82.body_pitch_rate = (float) -8.703746E37F;
            p82.body_yaw_rate = (float) -2.3664248E38F;
            p82.thrust = (float)1.870914E38F;
            CommunicationChannel.instance.send(p82); //===============================
            ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.time_boot_ms = (uint)948282931U;
            p83.type_mask = (byte)(byte)90;
            p83.q_SET(new float[4], 0);
            p83.body_roll_rate = (float)4.5790136E37F;
            p83.body_pitch_rate = (float) -1.4017815E38F;
            p83.body_yaw_rate = (float)1.0175837E38F;
            p83.thrust = (float)2.3458937E38F;
            CommunicationChannel.instance.send(p83); //===============================
            SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.time_boot_ms = (uint)2538218597U;
            p84.target_system = (byte)(byte)155;
            p84.target_component = (byte)(byte)180;
            p84.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_ENU;
            p84.type_mask = (ushort)(ushort)59868;
            p84.x = (float) -1.1217255E38F;
            p84.y = (float)1.6090833E37F;
            p84.z = (float)1.4698652E38F;
            p84.vx = (float) -3.1618812E38F;
            p84.vy = (float) -2.96772E38F;
            p84.vz = (float)7.398532E37F;
            p84.afx = (float) -2.6429627E38F;
            p84.afy = (float) -1.6344357E38F;
            p84.afz = (float)4.7456668E36F;
            p84.yaw = (float)1.5450747E38F;
            p84.yaw_rate = (float) -1.1132931E38F;
            CommunicationChannel.instance.send(p84); //===============================
            SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.time_boot_ms = (uint)3402773032U;
            p86.target_system = (byte)(byte)58;
            p86.target_component = (byte)(byte)31;
            p86.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_ENU;
            p86.type_mask = (ushort)(ushort)6186;
            p86.lat_int = (int) -759477908;
            p86.lon_int = (int)1571025890;
            p86.alt = (float) -7.095681E37F;
            p86.vx = (float)8.1445943E37F;
            p86.vy = (float)1.5262057E38F;
            p86.vz = (float)1.0847915E38F;
            p86.afx = (float)3.0983547E38F;
            p86.afy = (float)1.6397437E37F;
            p86.afz = (float)3.0608154E37F;
            p86.yaw = (float)1.8591853E38F;
            p86.yaw_rate = (float)3.0195301E38F;
            CommunicationChannel.instance.send(p86); //===============================
            POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.time_boot_ms = (uint)4265947204U;
            p87.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT;
            p87.type_mask = (ushort)(ushort)62448;
            p87.lat_int = (int) -520821896;
            p87.lon_int = (int)893561911;
            p87.alt = (float) -1.4882328E38F;
            p87.vx = (float)1.7884242E38F;
            p87.vy = (float)1.3594906E38F;
            p87.vz = (float)1.918222E38F;
            p87.afx = (float) -2.2366993E38F;
            p87.afy = (float) -2.9235382E38F;
            p87.afz = (float)5.9955396E37F;
            p87.yaw = (float) -4.8976497E37F;
            p87.yaw_rate = (float)1.1988279E38F;
            CommunicationChannel.instance.send(p87); //===============================
            LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.time_boot_ms = (uint)2501900929U;
            p89.x = (float) -2.3682147E38F;
            p89.y = (float)2.0052346E38F;
            p89.z = (float)7.733742E37F;
            p89.roll = (float)6.408644E37F;
            p89.pitch = (float)1.547874E36F;
            p89.yaw = (float)9.924119E37F;
            CommunicationChannel.instance.send(p89); //===============================
            HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.time_usec = (ulong)8504858100613269737L;
            p90.roll = (float)1.8819496E38F;
            p90.pitch = (float)8.2069383E37F;
            p90.yaw = (float)1.9689332E38F;
            p90.rollspeed = (float)2.526433E38F;
            p90.pitchspeed = (float)3.187633E38F;
            p90.yawspeed = (float)1.3233408E38F;
            p90.lat = (int) -1995097861;
            p90.lon = (int) -768866590;
            p90.alt = (int) -1953750829;
            p90.vx = (short)(short) -29015;
            p90.vy = (short)(short) -7231;
            p90.vz = (short)(short)27929;
            p90.xacc = (short)(short) -18717;
            p90.yacc = (short)(short) -10494;
            p90.zacc = (short)(short)19884;
            CommunicationChannel.instance.send(p90); //===============================
            HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.time_usec = (ulong)2655587998700657804L;
            p91.roll_ailerons = (float)2.1884941E38F;
            p91.pitch_elevator = (float)1.8928934E38F;
            p91.yaw_rudder = (float) -1.2200201E38F;
            p91.throttle = (float) -3.277575E38F;
            p91.aux1 = (float) -1.6520085E38F;
            p91.aux2 = (float)8.0840804E36F;
            p91.aux3 = (float)1.7428989E38F;
            p91.aux4 = (float)1.0585674E38F;
            p91.mode = (MAV_MODE)MAV_MODE.MAV_MODE_TEST_DISARMED;
            p91.nav_mode = (byte)(byte)26;
            CommunicationChannel.instance.send(p91); //===============================
            HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.time_usec = (ulong)812208480002725486L;
            p92.chan1_raw = (ushort)(ushort)31875;
            p92.chan2_raw = (ushort)(ushort)23403;
            p92.chan3_raw = (ushort)(ushort)17341;
            p92.chan4_raw = (ushort)(ushort)35977;
            p92.chan5_raw = (ushort)(ushort)30781;
            p92.chan6_raw = (ushort)(ushort)18086;
            p92.chan7_raw = (ushort)(ushort)63452;
            p92.chan8_raw = (ushort)(ushort)10137;
            p92.chan9_raw = (ushort)(ushort)25643;
            p92.chan10_raw = (ushort)(ushort)23811;
            p92.chan11_raw = (ushort)(ushort)48856;
            p92.chan12_raw = (ushort)(ushort)40085;
            p92.rssi = (byte)(byte)224;
            CommunicationChannel.instance.send(p92); //===============================
            HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.time_usec = (ulong)557221209320132725L;
            p93.controls_SET(new float[16], 0);
            p93.mode = (MAV_MODE)MAV_MODE.MAV_MODE_STABILIZE_DISARMED;
            p93.flags = (ulong)8739527194519366570L;
            CommunicationChannel.instance.send(p93); //===============================
            OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.time_usec = (ulong)3470673451598446750L;
            p100.sensor_id = (byte)(byte)10;
            p100.flow_x = (short)(short) -27727;
            p100.flow_y = (short)(short)8014;
            p100.flow_comp_m_x = (float)2.3947995E38F;
            p100.flow_comp_m_y = (float) -1.3456011E38F;
            p100.quality = (byte)(byte)51;
            p100.ground_distance = (float) -2.8059706E38F;
            p100.flow_rate_x_SET((float) -3.078271E37F, PH);
            p100.flow_rate_y_SET((float)3.1555624E38F, PH);
            CommunicationChannel.instance.send(p100); //===============================
            GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.usec = (ulong)7155299698516638271L;
            p101.x = (float)1.624352E38F;
            p101.y = (float)2.828624E38F;
            p101.z = (float)1.0537253E37F;
            p101.roll = (float)2.8705897E38F;
            p101.pitch = (float)1.1165118E38F;
            p101.yaw = (float) -2.4964642E38F;
            CommunicationChannel.instance.send(p101); //===============================
            VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.usec = (ulong)8719425891207152232L;
            p102.x = (float)6.95027E37F;
            p102.y = (float)3.149183E38F;
            p102.z = (float) -2.4490466E38F;
            p102.roll = (float) -2.579736E38F;
            p102.pitch = (float)1.3848801E37F;
            p102.yaw = (float) -2.519211E38F;
            CommunicationChannel.instance.send(p102); //===============================
            VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.usec = (ulong)2888745754877590933L;
            p103.x = (float) -2.24988E38F;
            p103.y = (float)1.8693153E38F;
            p103.z = (float) -2.0071521E38F;
            CommunicationChannel.instance.send(p103); //===============================
            VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.usec = (ulong)271954122822479361L;
            p104.x = (float)3.0680335E38F;
            p104.y = (float) -7.905644E37F;
            p104.z = (float) -1.0093119E38F;
            p104.roll = (float)1.2520056E38F;
            p104.pitch = (float) -7.3532117E37F;
            p104.yaw = (float)1.4590742E38F;
            CommunicationChannel.instance.send(p104); //===============================
            HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.time_usec = (ulong)225150312741033369L;
            p105.xacc = (float) -2.9530243E38F;
            p105.yacc = (float)4.12642E36F;
            p105.zacc = (float) -3.1411838E38F;
            p105.xgyro = (float)3.1712553E38F;
            p105.ygyro = (float) -2.6663775E38F;
            p105.zgyro = (float)2.4392968E38F;
            p105.xmag = (float) -6.8972306E37F;
            p105.ymag = (float) -4.54679E37F;
            p105.zmag = (float) -2.0374555E38F;
            p105.abs_pressure = (float)2.3374607E38F;
            p105.diff_pressure = (float) -3.2028718E38F;
            p105.pressure_alt = (float) -3.256738E38F;
            p105.temperature = (float)1.2715736E38F;
            p105.fields_updated = (ushort)(ushort)23181;
            CommunicationChannel.instance.send(p105); //===============================
            OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.time_usec = (ulong)6947138119651021400L;
            p106.sensor_id = (byte)(byte)241;
            p106.integration_time_us = (uint)2642490377U;
            p106.integrated_x = (float) -3.2526332E38F;
            p106.integrated_y = (float) -3.0691265E38F;
            p106.integrated_xgyro = (float)2.15438E38F;
            p106.integrated_ygyro = (float) -9.547927E37F;
            p106.integrated_zgyro = (float)5.119547E37F;
            p106.temperature = (short)(short) -23498;
            p106.quality = (byte)(byte)186;
            p106.time_delta_distance_us = (uint)2301879893U;
            p106.distance = (float)9.793718E37F;
            CommunicationChannel.instance.send(p106); //===============================
            HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.time_usec = (ulong)5022725178598377511L;
            p107.xacc = (float) -2.0924013E38F;
            p107.yacc = (float) -1.5048637E38F;
            p107.zacc = (float)2.8141833E38F;
            p107.xgyro = (float)7.9160054E37F;
            p107.ygyro = (float)1.0068595E38F;
            p107.zgyro = (float)1.4124841E38F;
            p107.xmag = (float)1.5211945E38F;
            p107.ymag = (float)1.6832795E38F;
            p107.zmag = (float)2.776813E38F;
            p107.abs_pressure = (float)3.3652235E38F;
            p107.diff_pressure = (float)2.6579617E38F;
            p107.pressure_alt = (float)1.5424338E38F;
            p107.temperature = (float)4.2075192E37F;
            p107.fields_updated = (uint)3422946671U;
            CommunicationChannel.instance.send(p107); //===============================
            SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.q1 = (float)7.4293696E37F;
            p108.q2 = (float) -3.0423748E38F;
            p108.q3 = (float)3.2061535E38F;
            p108.q4 = (float)6.736086E37F;
            p108.roll = (float)4.6690584E37F;
            p108.pitch = (float) -1.2109069E38F;
            p108.yaw = (float)3.2414576E38F;
            p108.xacc = (float) -1.2822259E38F;
            p108.yacc = (float)1.5812247E36F;
            p108.zacc = (float)1.8780317E38F;
            p108.xgyro = (float)3.0886347E37F;
            p108.ygyro = (float) -1.2444172E38F;
            p108.zgyro = (float)2.6228107E38F;
            p108.lat = (float)2.3990304E38F;
            p108.lon = (float) -1.6585865E38F;
            p108.alt = (float) -2.2259602E38F;
            p108.std_dev_horz = (float)2.746409E38F;
            p108.std_dev_vert = (float)6.4316606E37F;
            p108.vn = (float)6.980466E37F;
            p108.ve = (float) -2.612867E38F;
            p108.vd = (float)1.9772337E38F;
            CommunicationChannel.instance.send(p108); //===============================
            RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.rssi = (byte)(byte)254;
            p109.remrssi = (byte)(byte)60;
            p109.txbuf = (byte)(byte)244;
            p109.noise = (byte)(byte)234;
            p109.remnoise = (byte)(byte)92;
            p109.rxerrors = (ushort)(ushort)35399;
            p109.fixed_ = (ushort)(ushort)1182;
            CommunicationChannel.instance.send(p109); //===============================
            FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_network = (byte)(byte)12;
            p110.target_system = (byte)(byte)234;
            p110.target_component = (byte)(byte)245;
            p110.payload_SET(new byte[251], 0);
            CommunicationChannel.instance.send(p110); //===============================
            TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.tc1 = (long)1505826074776508672L;
            p111.ts1 = (long)5055945696406678007L;
            CommunicationChannel.instance.send(p111); //===============================
            CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.time_usec = (ulong)9187574741850495885L;
            p112.seq = (uint)2661306042U;
            CommunicationChannel.instance.send(p112); //===============================
            HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.time_usec = (ulong)1952713302728576296L;
            p113.fix_type = (byte)(byte)37;
            p113.lat = (int)1499189294;
            p113.lon = (int) -1430792965;
            p113.alt = (int) -1924339302;
            p113.eph = (ushort)(ushort)37934;
            p113.epv = (ushort)(ushort)31456;
            p113.vel = (ushort)(ushort)10830;
            p113.vn = (short)(short) -30048;
            p113.ve = (short)(short) -5232;
            p113.vd = (short)(short) -16584;
            p113.cog = (ushort)(ushort)35804;
            p113.satellites_visible = (byte)(byte)244;
            CommunicationChannel.instance.send(p113); //===============================
            HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.time_usec = (ulong)1615629026449271595L;
            p114.sensor_id = (byte)(byte)150;
            p114.integration_time_us = (uint)978150711U;
            p114.integrated_x = (float) -1.1875847E38F;
            p114.integrated_y = (float)1.4340588E37F;
            p114.integrated_xgyro = (float) -9.700803E37F;
            p114.integrated_ygyro = (float) -2.3971313E38F;
            p114.integrated_zgyro = (float) -2.9629053E38F;
            p114.temperature = (short)(short) -21389;
            p114.quality = (byte)(byte)54;
            p114.time_delta_distance_us = (uint)959811579U;
            p114.distance = (float) -3.0977904E38F;
            CommunicationChannel.instance.send(p114); //===============================
            HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.time_usec = (ulong)8279871241667661023L;
            p115.attitude_quaternion_SET(new float[4], 0);
            p115.rollspeed = (float) -2.5634445E38F;
            p115.pitchspeed = (float) -1.172348E37F;
            p115.yawspeed = (float)2.4995672E38F;
            p115.lat = (int) -1209638976;
            p115.lon = (int)600810043;
            p115.alt = (int)1140974968;
            p115.vx = (short)(short)22208;
            p115.vy = (short)(short)3846;
            p115.vz = (short)(short)8582;
            p115.ind_airspeed = (ushort)(ushort)4021;
            p115.true_airspeed = (ushort)(ushort)37171;
            p115.xacc = (short)(short)5265;
            p115.yacc = (short)(short)15003;
            p115.zacc = (short)(short) -18558;
            CommunicationChannel.instance.send(p115); //===============================
            SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.time_boot_ms = (uint)2784181301U;
            p116.xacc = (short)(short)21515;
            p116.yacc = (short)(short) -4189;
            p116.zacc = (short)(short)18769;
            p116.xgyro = (short)(short)31478;
            p116.ygyro = (short)(short) -3585;
            p116.zgyro = (short)(short) -1970;
            p116.xmag = (short)(short)4837;
            p116.ymag = (short)(short) -20769;
            p116.zmag = (short)(short)5155;
            CommunicationChannel.instance.send(p116); //===============================
            LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.target_system = (byte)(byte)20;
            p117.target_component = (byte)(byte)114;
            p117.start = (ushort)(ushort)5016;
            p117.end = (ushort)(ushort)29962;
            CommunicationChannel.instance.send(p117); //===============================
            LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.id = (ushort)(ushort)6290;
            p118.num_logs = (ushort)(ushort)2692;
            p118.last_log_num = (ushort)(ushort)18150;
            p118.time_utc = (uint)4288485677U;
            p118.size = (uint)1591478946U;
            CommunicationChannel.instance.send(p118); //===============================
            LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.target_system = (byte)(byte)122;
            p119.target_component = (byte)(byte)15;
            p119.id = (ushort)(ushort)45800;
            p119.ofs = (uint)4006049739U;
            p119.count = (uint)2596795131U;
            CommunicationChannel.instance.send(p119); //===============================
            LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.id = (ushort)(ushort)9109;
            p120.ofs = (uint)1307176758U;
            p120.count = (byte)(byte)106;
            p120.data__SET(new byte[90], 0);
            CommunicationChannel.instance.send(p120); //===============================
            LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_system = (byte)(byte)160;
            p121.target_component = (byte)(byte)15;
            CommunicationChannel.instance.send(p121); //===============================
            LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_system = (byte)(byte)111;
            p122.target_component = (byte)(byte)157;
            CommunicationChannel.instance.send(p122); //===============================
            GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.target_system = (byte)(byte)16;
            p123.target_component = (byte)(byte)57;
            p123.len = (byte)(byte)29;
            p123.data__SET(new byte[110], 0);
            CommunicationChannel.instance.send(p123); //===============================
            GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.time_usec = (ulong)5428464732354997731L;
            p124.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED;
            p124.lat = (int) -1934235157;
            p124.lon = (int)510222082;
            p124.alt = (int)360914835;
            p124.eph = (ushort)(ushort)41869;
            p124.epv = (ushort)(ushort)25836;
            p124.vel = (ushort)(ushort)11518;
            p124.cog = (ushort)(ushort)20959;
            p124.satellites_visible = (byte)(byte)200;
            p124.dgps_numch = (byte)(byte)110;
            p124.dgps_age = (uint)2109239079U;
            CommunicationChannel.instance.send(p124); //===============================
            POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vcc = (ushort)(ushort)50091;
            p125.Vservo = (ushort)(ushort)44287;
            p125.flags = (MAV_POWER_STATUS)MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID;
            CommunicationChannel.instance.send(p125); //===============================
            SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.device = (SERIAL_CONTROL_DEV)SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2;
            p126.flags = (SERIAL_CONTROL_FLAG)SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND;
            p126.timeout = (ushort)(ushort)40629;
            p126.baudrate = (uint)501491596U;
            p126.count = (byte)(byte)249;
            p126.data__SET(new byte[70], 0);
            CommunicationChannel.instance.send(p126); //===============================
            GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.time_last_baseline_ms = (uint)524664925U;
            p127.rtk_receiver_id = (byte)(byte)60;
            p127.wn = (ushort)(ushort)61538;
            p127.tow = (uint)2699876069U;
            p127.rtk_health = (byte)(byte)52;
            p127.rtk_rate = (byte)(byte)244;
            p127.nsats = (byte)(byte)109;
            p127.baseline_coords_type = (byte)(byte)173;
            p127.baseline_a_mm = (int)92555843;
            p127.baseline_b_mm = (int) -110992986;
            p127.baseline_c_mm = (int)1678086971;
            p127.accuracy = (uint)667552234U;
            p127.iar_num_hypotheses = (int) -457760577;
            CommunicationChannel.instance.send(p127); //===============================
            GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.time_last_baseline_ms = (uint)157910711U;
            p128.rtk_receiver_id = (byte)(byte)69;
            p128.wn = (ushort)(ushort)60947;
            p128.tow = (uint)4099945254U;
            p128.rtk_health = (byte)(byte)103;
            p128.rtk_rate = (byte)(byte)220;
            p128.nsats = (byte)(byte)236;
            p128.baseline_coords_type = (byte)(byte)78;
            p128.baseline_a_mm = (int)1887659974;
            p128.baseline_b_mm = (int) -2113670472;
            p128.baseline_c_mm = (int) -442311010;
            p128.accuracy = (uint)1551496309U;
            p128.iar_num_hypotheses = (int) -813866545;
            CommunicationChannel.instance.send(p128); //===============================
            SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.time_boot_ms = (uint)2464427593U;
            p129.xacc = (short)(short) -28639;
            p129.yacc = (short)(short)13724;
            p129.zacc = (short)(short) -12050;
            p129.xgyro = (short)(short)27254;
            p129.ygyro = (short)(short)13030;
            p129.zgyro = (short)(short) -23166;
            p129.xmag = (short)(short) -5287;
            p129.ymag = (short)(short) -5430;
            p129.zmag = (short)(short) -7461;
            CommunicationChannel.instance.send(p129); //===============================
            DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.type = (byte)(byte)146;
            p130.size = (uint)712817749U;
            p130.width = (ushort)(ushort)25195;
            p130.height = (ushort)(ushort)26717;
            p130.packets = (ushort)(ushort)638;
            p130.payload = (byte)(byte)135;
            p130.jpg_quality = (byte)(byte)79;
            CommunicationChannel.instance.send(p130); //===============================
            ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.seqnr = (ushort)(ushort)46275;
            p131.data__SET(new byte[253], 0);
            CommunicationChannel.instance.send(p131); //===============================
            DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.time_boot_ms = (uint)3185349264U;
            p132.min_distance = (ushort)(ushort)44215;
            p132.max_distance = (ushort)(ushort)41319;
            p132.current_distance = (ushort)(ushort)26403;
            p132.type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN;
            p132.id = (byte)(byte)79;
            p132.orientation = (MAV_SENSOR_ORIENTATION)MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_PITCH_270;
            p132.covariance = (byte)(byte)160;
            CommunicationChannel.instance.send(p132); //===============================
            TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.lat = (int)1826088362;
            p133.lon = (int)1729703025;
            p133.grid_spacing = (ushort)(ushort)56068;
            p133.mask = (ulong)4823375089788428343L;
            CommunicationChannel.instance.send(p133); //===============================
            TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.lat = (int) -587053806;
            p134.lon = (int) -1585671350;
            p134.grid_spacing = (ushort)(ushort)11285;
            p134.gridbit = (byte)(byte)251;
            p134.data__SET(new short[16], 0);
            CommunicationChannel.instance.send(p134); //===============================
            TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lat = (int) -1342445469;
            p135.lon = (int)1336617938;
            CommunicationChannel.instance.send(p135); //===============================
            TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.lat = (int)828858702;
            p136.lon = (int)1823004475;
            p136.spacing = (ushort)(ushort)43286;
            p136.terrain_height = (float)3.1517429E38F;
            p136.current_height = (float) -9.278364E37F;
            p136.pending = (ushort)(ushort)50545;
            p136.loaded = (ushort)(ushort)52681;
            CommunicationChannel.instance.send(p136); //===============================
            SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.time_boot_ms = (uint)1036519561U;
            p137.press_abs = (float)4.5851845E37F;
            p137.press_diff = (float) -1.9528416E37F;
            p137.temperature = (short)(short)25510;
            CommunicationChannel.instance.send(p137); //===============================
            ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.time_usec = (ulong)6896795255014787014L;
            p138.q_SET(new float[4], 0);
            p138.x = (float)2.1643197E38F;
            p138.y = (float)8.922282E37F;
            p138.z = (float) -6.630082E37F;
            CommunicationChannel.instance.send(p138); //===============================
            SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.time_usec = (ulong)533916664768570864L;
            p139.group_mlx = (byte)(byte)79;
            p139.target_system = (byte)(byte)56;
            p139.target_component = (byte)(byte)81;
            p139.controls_SET(new float[8], 0);
            CommunicationChannel.instance.send(p139); //===============================
            ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.time_usec = (ulong)5175344346357698188L;
            p140.group_mlx = (byte)(byte)247;
            p140.controls_SET(new float[8], 0);
            CommunicationChannel.instance.send(p140); //===============================
            ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.time_usec = (ulong)8857800823438094827L;
            p141.altitude_monotonic = (float) -1.5943081E38F;
            p141.altitude_amsl = (float) -4.163037E37F;
            p141.altitude_local = (float)8.2864844E37F;
            p141.altitude_relative = (float) -1.2656546E37F;
            p141.altitude_terrain = (float) -1.8924819E38F;
            p141.bottom_clearance = (float)9.7939E37F;
            CommunicationChannel.instance.send(p141); //===============================
            RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.request_id = (byte)(byte)50;
            p142.uri_type = (byte)(byte)127;
            p142.uri_SET(new byte[120], 0);
            p142.transfer_type = (byte)(byte)220;
            p142.storage_SET(new byte[120], 0);
            CommunicationChannel.instance.send(p142); //===============================
            SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.time_boot_ms = (uint)3890539152U;
            p143.press_abs = (float) -1.8889576E38F;
            p143.press_diff = (float) -3.1670615E38F;
            p143.temperature = (short)(short) -4909;
            CommunicationChannel.instance.send(p143); //===============================
            FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.timestamp = (ulong)6105465833657833701L;
            p144.est_capabilities = (byte)(byte)207;
            p144.lat = (int)298262072;
            p144.lon = (int) -1305060361;
            p144.alt = (float)1.238065E38F;
            p144.vel_SET(new float[3], 0);
            p144.acc_SET(new float[3], 0);
            p144.attitude_q_SET(new float[4], 0);
            p144.rates_SET(new float[3], 0);
            p144.position_cov_SET(new float[3], 0);
            p144.custom_state = (ulong)2740271032480780007L;
            CommunicationChannel.instance.send(p144); //===============================
            CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.time_usec = (ulong)4754951079795417232L;
            p146.x_acc = (float)1.0879836E38F;
            p146.y_acc = (float)3.2177603E38F;
            p146.z_acc = (float)2.6013574E38F;
            p146.x_vel = (float) -5.673011E37F;
            p146.y_vel = (float) -9.937693E37F;
            p146.z_vel = (float) -1.9205827E38F;
            p146.x_pos = (float) -2.6300012E38F;
            p146.y_pos = (float) -1.3151804E38F;
            p146.z_pos = (float)1.9288048E38F;
            p146.airspeed = (float) -2.319009E38F;
            p146.vel_variance_SET(new float[3], 0);
            p146.pos_variance_SET(new float[3], 0);
            p146.q_SET(new float[4], 0);
            p146.roll_rate = (float) -2.7213014E38F;
            p146.pitch_rate = (float)1.889122E38F;
            p146.yaw_rate = (float)1.0148626E38F;
            CommunicationChannel.instance.send(p146); //===============================
            BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.id = (byte)(byte)18;
            p147.battery_function = (MAV_BATTERY_FUNCTION)MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_PROPULSION;
            p147.type = (MAV_BATTERY_TYPE)MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LION;
            p147.temperature = (short)(short)17737;
            p147.voltages_SET(new ushort[10], 0);
            p147.current_battery = (short)(short) -23607;
            p147.current_consumed = (int) -1699475441;
            p147.energy_consumed = (int)737753214;
            p147.battery_remaining = (sbyte)(sbyte)3;
            CommunicationChannel.instance.send(p147); //===============================
            AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY)MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT;
            p148.flight_sw_version = (uint)4099567069U;
            p148.middleware_sw_version = (uint)870434611U;
            p148.os_sw_version = (uint)3220084129U;
            p148.board_version = (uint)2105169444U;
            p148.flight_custom_version_SET(new byte[8], 0);
            p148.middleware_custom_version_SET(new byte[8], 0);
            p148.os_custom_version_SET(new byte[8], 0);
            p148.vendor_id = (ushort)(ushort)48456;
            p148.product_id = (ushort)(ushort)15479;
            p148.uid = (ulong)3000225849362383956L;
            p148.uid2_SET(new byte[18], 0, PH);
            CommunicationChannel.instance.send(p148); //===============================
            LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.time_usec = (ulong)6701043138426976013L;
            p149.target_num = (byte)(byte)238;
            p149.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_MISSION;
            p149.angle_x = (float)3.9489464E37F;
            p149.angle_y = (float) -2.6728725E38F;
            p149.distance = (float)1.8591289E38F;
            p149.size_x = (float) -1.0597664E38F;
            p149.size_y = (float)2.5952785E38F;
            p149.x_SET((float)8.833137E37F, PH);
            p149.y_SET((float)6.529748E37F, PH);
            p149.z_SET((float) -2.0101783E37F, PH);
            p149.q_SET(new float[4], 0, PH);
            p149.type = (LANDING_TARGET_TYPE)LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_RADIO_BEACON;
            p149.position_valid_SET((byte)(byte)230, PH);
            CommunicationChannel.instance.send(p149); //===============================
            AQ_TELEMETRY_F p150 = CommunicationChannel.new_AQ_TELEMETRY_F();
            PH.setPack(p150);
            p150.Index = (ushort)(ushort)7358;
            p150.value1 = (float) -2.2754835E38F;
            p150.value2 = (float) -1.2629817E38F;
            p150.value3 = (float)2.0347334E38F;
            p150.value4 = (float)4.733871E37F;
            p150.value5 = (float) -7.1296934E37F;
            p150.value6 = (float) -1.1119783E38F;
            p150.value7 = (float)9.038592E37F;
            p150.value8 = (float) -1.5449175E38F;
            p150.value9 = (float)9.860683E36F;
            p150.value10 = (float)1.3178783E38F;
            p150.value11 = (float)1.4915697E37F;
            p150.value12 = (float)1.5345044E38F;
            p150.value13 = (float)3.371487E37F;
            p150.value14 = (float)1.9061482E38F;
            p150.value15 = (float) -2.357112E38F;
            p150.value16 = (float)1.1363603E38F;
            p150.value17 = (float) -2.6063919E38F;
            p150.value18 = (float)3.0158039E38F;
            p150.value19 = (float)2.2706646E38F;
            p150.value20 = (float) -6.140195E37F;
            CommunicationChannel.instance.send(p150); //===============================
            AQ_ESC_TELEMETRY p152 = CommunicationChannel.new_AQ_ESC_TELEMETRY();
            PH.setPack(p152);
            p152.time_boot_ms = (uint)1176645033U;
            p152.seq = (byte)(byte)241;
            p152.num_motors = (byte)(byte)100;
            p152.num_in_seq = (byte)(byte)82;
            p152.escid_SET(new byte[4], 0);
            p152.status_age_SET(new ushort[4], 0);
            p152.data_version_SET(new byte[4], 0);
            p152.data0_SET(new uint[4], 0);
            p152.data1_SET(new uint[4], 0);
            CommunicationChannel.instance.send(p152); //===============================
            ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.time_usec = (ulong)7580203846699661835L;
            p230.flags = (ESTIMATOR_STATUS_FLAGS)ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS;
            p230.vel_ratio = (float) -7.812255E37F;
            p230.pos_horiz_ratio = (float) -5.893997E36F;
            p230.pos_vert_ratio = (float) -1.8304402E38F;
            p230.mag_ratio = (float) -1.4431215E38F;
            p230.hagl_ratio = (float) -1.97003E38F;
            p230.tas_ratio = (float)1.1983581E38F;
            p230.pos_horiz_accuracy = (float) -1.8000302E38F;
            p230.pos_vert_accuracy = (float)1.6445567E38F;
            CommunicationChannel.instance.send(p230); //===============================
            WIND_COV p231 = CommunicationChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.time_usec = (ulong)4006445802788616160L;
            p231.wind_x = (float)1.5088232E38F;
            p231.wind_y = (float)3.1459435E38F;
            p231.wind_z = (float) -8.493237E37F;
            p231.var_horiz = (float) -7.595929E37F;
            p231.var_vert = (float)3.1633643E38F;
            p231.wind_alt = (float) -3.3590968E38F;
            p231.horiz_accuracy = (float)2.5303718E38F;
            p231.vert_accuracy = (float)1.1182648E38F;
            CommunicationChannel.instance.send(p231); //===============================
            GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.time_usec = (ulong)5482658332996117176L;
            p232.gps_id = (byte)(byte)73;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS)GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT;
            p232.time_week_ms = (uint)4223260261U;
            p232.time_week = (ushort)(ushort)18107;
            p232.fix_type = (byte)(byte)8;
            p232.lat = (int)871872599;
            p232.lon = (int)1659813584;
            p232.alt = (float)1.6910764E38F;
            p232.hdop = (float) -2.5901566E38F;
            p232.vdop = (float) -2.0991925E38F;
            p232.vn = (float) -2.5127336E38F;
            p232.ve = (float) -1.94723E38F;
            p232.vd = (float) -1.8058275E38F;
            p232.speed_accuracy = (float) -1.4728428E38F;
            p232.horiz_accuracy = (float)7.1367517E37F;
            p232.vert_accuracy = (float)1.6795613E38F;
            p232.satellites_visible = (byte)(byte)123;
            CommunicationChannel.instance.send(p232); //===============================
            GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.flags = (byte)(byte)163;
            p233.len = (byte)(byte)149;
            p233.data__SET(new byte[180], 0);
            CommunicationChannel.instance.send(p233); //===============================
            HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED;
            p234.custom_mode = (uint)1141169180U;
            p234.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND;
            p234.roll = (short)(short)23382;
            p234.pitch = (short)(short) -12431;
            p234.heading = (ushort)(ushort)53056;
            p234.throttle = (sbyte)(sbyte) - 3;
            p234.heading_sp = (short)(short)26534;
            p234.latitude = (int) -733209518;
            p234.longitude = (int)764302756;
            p234.altitude_amsl = (short)(short) -22131;
            p234.altitude_sp = (short)(short) -12454;
            p234.airspeed = (byte)(byte)131;
            p234.airspeed_sp = (byte)(byte)93;
            p234.groundspeed = (byte)(byte)195;
            p234.climb_rate = (sbyte)(sbyte) - 46;
            p234.gps_nsat = (byte)(byte)236;
            p234.gps_fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS;
            p234.battery_remaining = (byte)(byte)145;
            p234.temperature = (sbyte)(sbyte)3;
            p234.temperature_air = (sbyte)(sbyte)14;
            p234.failsafe = (byte)(byte)70;
            p234.wp_num = (byte)(byte)96;
            p234.wp_distance = (ushort)(ushort)41144;
            CommunicationChannel.instance.send(p234); //===============================
            VIBRATION p241 = CommunicationChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.time_usec = (ulong)3762957795945525468L;
            p241.vibration_x = (float) -4.875292E37F;
            p241.vibration_y = (float) -1.7919695E38F;
            p241.vibration_z = (float)1.6864301E38F;
            p241.clipping_0 = (uint)3493643083U;
            p241.clipping_1 = (uint)331966163U;
            p241.clipping_2 = (uint)4015174304U;
            CommunicationChannel.instance.send(p241); //===============================
            HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.latitude = (int) -1971432580;
            p242.longitude = (int)894930463;
            p242.altitude = (int)2104208751;
            p242.x = (float) -1.751748E38F;
            p242.y = (float) -1.6675598E38F;
            p242.z = (float)4.1662762E37F;
            p242.q_SET(new float[4], 0);
            p242.approach_x = (float) -8.1960893E37F;
            p242.approach_y = (float) -2.7095552E37F;
            p242.approach_z = (float) -4.6861605E37F;
            p242.time_usec_SET((ulong)3564733015904255144L, PH);
            CommunicationChannel.instance.send(p242); //===============================
            SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.target_system = (byte)(byte)234;
            p243.latitude = (int) -949385582;
            p243.longitude = (int) -450298298;
            p243.altitude = (int) -2120468894;
            p243.x = (float)3.3308854E38F;
            p243.y = (float)7.167707E37F;
            p243.z = (float) -8.800338E37F;
            p243.q_SET(new float[4], 0);
            p243.approach_x = (float)3.7069615E37F;
            p243.approach_y = (float) -1.3901379E38F;
            p243.approach_z = (float)2.259801E38F;
            p243.time_usec_SET((ulong)808290587193694046L, PH);
            CommunicationChannel.instance.send(p243); //===============================
            MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.message_id = (ushort)(ushort)40692;
            p244.interval_us = (int) -1306436926;
            CommunicationChannel.instance.send(p244); //===============================
            EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.vtol_state = (MAV_VTOL_STATE)MAV_VTOL_STATE.MAV_VTOL_STATE_UNDEFINED;
            p245.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND;
            CommunicationChannel.instance.send(p245); //===============================
            ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.ICAO_address = (uint)1881162684U;
            p246.lat = (int)1377014365;
            p246.lon = (int)490087159;
            p246.altitude_type = (ADSB_ALTITUDE_TYPE)ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
            p246.altitude = (int)452736151;
            p246.heading = (ushort)(ushort)10095;
            p246.hor_velocity = (ushort)(ushort)56739;
            p246.ver_velocity = (short)(short)24102;
            p246.callsign_SET("DEMO", PH);
            p246.emitter_type = (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_PARACHUTE;
            p246.tslc = (byte)(byte)50;
            p246.flags = (ADSB_FLAGS)ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY;
            p246.squawk = (ushort)(ushort)51952;
            CommunicationChannel.instance.send(p246); //===============================
            COLLISION p247 = CommunicationChannel.new_COLLISION();
            PH.setPack(p247);
            p247.src_ = (MAV_COLLISION_SRC)MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB;
            p247.id = (uint)1762364793U;
            p247.action = (MAV_COLLISION_ACTION)MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_NONE;
            p247.threat_level = (MAV_COLLISION_THREAT_LEVEL)MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE;
            p247.time_to_minimum_delta = (float) -2.8726293E38F;
            p247.altitude_minimum_delta = (float) -9.788694E37F;
            p247.horizontal_minimum_delta = (float)8.6801196E36F;
            CommunicationChannel.instance.send(p247); //===============================
            V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.target_network = (byte)(byte)29;
            p248.target_system = (byte)(byte)98;
            p248.target_component = (byte)(byte)191;
            p248.message_type = (ushort)(ushort)53630;
            p248.payload_SET(new byte[249], 0);
            CommunicationChannel.instance.send(p248); //===============================
            MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.address = (ushort)(ushort)57670;
            p249.ver = (byte)(byte)121;
            p249.type = (byte)(byte)38;
            p249.value_SET(new sbyte[32], 0);
            CommunicationChannel.instance.send(p249); //===============================
            DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.name_SET("DEMO", PH);
            p250.time_usec = (ulong)1787597165026232880L;
            p250.x = (float) -1.5256151E38F;
            p250.y = (float) -1.9167897E38F;
            p250.z = (float) -2.2369962E38F;
            CommunicationChannel.instance.send(p250); //===============================
            NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.time_boot_ms = (uint)1175244002U;
            p251.name_SET("DEMO", PH);
            p251.value = (float) -4.9554566E37F;
            CommunicationChannel.instance.send(p251); //===============================
            NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.time_boot_ms = (uint)754555995U;
            p252.name_SET("DEMO", PH);
            p252.value = (int)1154837436;
            CommunicationChannel.instance.send(p252); //===============================
            STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.severity = (MAV_SEVERITY)MAV_SEVERITY.MAV_SEVERITY_ALERT;
            p253.text_SET("DEMO", PH);
            CommunicationChannel.instance.send(p253); //===============================
            DEBUG p254 = CommunicationChannel.new_DEBUG();
            PH.setPack(p254);
            p254.time_boot_ms = (uint)3414440786U;
            p254.ind = (byte)(byte)13;
            p254.value = (float)2.6936973E38F;
            CommunicationChannel.instance.send(p254); //===============================
            SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.target_system = (byte)(byte)242;
            p256.target_component = (byte)(byte)116;
            p256.secret_key_SET(new byte[32], 0);
            p256.initial_timestamp = (ulong)7737554588190385735L;
            CommunicationChannel.instance.send(p256); //===============================
            BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.time_boot_ms = (uint)3737269238U;
            p257.last_change_ms = (uint)2028196997U;
            p257.state = (byte)(byte)83;
            CommunicationChannel.instance.send(p257); //===============================
            PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_system = (byte)(byte)34;
            p258.target_component = (byte)(byte)239;
            p258.tune_SET("DEMO", PH);
            CommunicationChannel.instance.send(p258); //===============================
            CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.time_boot_ms = (uint)3911289025U;
            p259.vendor_name_SET(new byte[32], 0);
            p259.model_name_SET(new byte[32], 0);
            p259.firmware_version = (uint)1861628005U;
            p259.focal_length = (float) -2.4281831E38F;
            p259.sensor_size_h = (float) -2.53102E38F;
            p259.sensor_size_v = (float) -2.7476221E38F;
            p259.resolution_h = (ushort)(ushort)9094;
            p259.resolution_v = (ushort)(ushort)43379;
            p259.lens_id = (byte)(byte)97;
            p259.flags = (CAMERA_CAP_FLAGS)CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE;
            p259.cam_definition_version = (ushort)(ushort)34826;
            p259.cam_definition_uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p259); //===============================
            CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.time_boot_ms = (uint)1290896058U;
            p260.mode_id = (CAMERA_MODE)CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY;
            CommunicationChannel.instance.send(p260); //===============================
            STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.time_boot_ms = (uint)3856153122U;
            p261.storage_id = (byte)(byte)59;
            p261.storage_count = (byte)(byte)95;
            p261.status = (byte)(byte)95;
            p261.total_capacity = (float)1.1855223E37F;
            p261.used_capacity = (float)6.839094E37F;
            p261.available_capacity = (float)2.4437466E38F;
            p261.read_speed = (float) -2.0577728E37F;
            p261.write_speed = (float) -2.6965186E38F;
            CommunicationChannel.instance.send(p261); //===============================
            CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.time_boot_ms = (uint)2566844176U;
            p262.image_status = (byte)(byte)59;
            p262.video_status = (byte)(byte)60;
            p262.image_interval = (float) -9.683199E37F;
            p262.recording_time_ms = (uint)3742452973U;
            p262.available_capacity = (float) -1.5976427E38F;
            CommunicationChannel.instance.send(p262); //===============================
            CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.time_boot_ms = (uint)2336111152U;
            p263.time_utc = (ulong)4484575294371236719L;
            p263.camera_id = (byte)(byte)213;
            p263.lat = (int) -1758305781;
            p263.lon = (int) -582898483;
            p263.alt = (int) -927676969;
            p263.relative_alt = (int)1547915115;
            p263.q_SET(new float[4], 0);
            p263.image_index = (int)656527870;
            p263.capture_result = (sbyte)(sbyte) - 28;
            p263.file_url_SET("DEMO", PH);
            CommunicationChannel.instance.send(p263); //===============================
            FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.time_boot_ms = (uint)725619761U;
            p264.arming_time_utc = (ulong)7866640448110672890L;
            p264.takeoff_time_utc = (ulong)7214718906015798551L;
            p264.flight_uuid = (ulong)6636051130012245837L;
            CommunicationChannel.instance.send(p264); //===============================
            MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.time_boot_ms = (uint)2794579044U;
            p265.roll = (float)2.7631927E38F;
            p265.pitch = (float)2.2670288E38F;
            p265.yaw = (float)6.2148624E37F;
            CommunicationChannel.instance.send(p265); //===============================
            LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.target_system = (byte)(byte)17;
            p266.target_component = (byte)(byte)165;
            p266.sequence = (ushort)(ushort)39156;
            p266.length = (byte)(byte)229;
            p266.first_message_offset = (byte)(byte)184;
            p266.data__SET(new byte[249], 0);
            CommunicationChannel.instance.send(p266); //===============================
            LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.target_system = (byte)(byte)76;
            p267.target_component = (byte)(byte)30;
            p267.sequence = (ushort)(ushort)36992;
            p267.length = (byte)(byte)33;
            p267.first_message_offset = (byte)(byte)101;
            p267.data__SET(new byte[249], 0);
            CommunicationChannel.instance.send(p267); //===============================
            LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_system = (byte)(byte)157;
            p268.target_component = (byte)(byte)234;
            p268.sequence = (ushort)(ushort)47302;
            CommunicationChannel.instance.send(p268); //===============================
            VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.camera_id = (byte)(byte)106;
            p269.status = (byte)(byte)99;
            p269.framerate = (float)1.2680704E38F;
            p269.resolution_h = (ushort)(ushort)54926;
            p269.resolution_v = (ushort)(ushort)23556;
            p269.bitrate = (uint)3148169169U;
            p269.rotation = (ushort)(ushort)38077;
            p269.uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p269); //===============================
            SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.target_system = (byte)(byte)126;
            p270.target_component = (byte)(byte)71;
            p270.camera_id = (byte)(byte)134;
            p270.framerate = (float)2.9008744E38F;
            p270.resolution_h = (ushort)(ushort)38269;
            p270.resolution_v = (ushort)(ushort)6028;
            p270.bitrate = (uint)1030926037U;
            p270.rotation = (ushort)(ushort)42393;
            p270.uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p270); //===============================
            WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.ssid_SET("DEMO", PH);
            p299.password_SET("DEMO", PH);
            CommunicationChannel.instance.send(p299); //===============================
            PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.version = (ushort)(ushort)35102;
            p300.min_version = (ushort)(ushort)53706;
            p300.max_version = (ushort)(ushort)39019;
            p300.spec_version_hash_SET(new byte[8], 0);
            p300.library_version_hash_SET(new byte[8], 0);
            CommunicationChannel.instance.send(p300); //===============================
            UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.time_usec = (ulong)2897500646613743631L;
            p310.uptime_sec = (uint)2380096350U;
            p310.health = (UAVCAN_NODE_HEALTH)UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL;
            p310.mode = (UAVCAN_NODE_MODE)UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OPERATIONAL;
            p310.sub_mode = (byte)(byte)142;
            p310.vendor_specific_status_code = (ushort)(ushort)56157;
            CommunicationChannel.instance.send(p310); //===============================
            UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.time_usec = (ulong)3102978288842970253L;
            p311.uptime_sec = (uint)1308006138U;
            p311.name_SET("DEMO", PH);
            p311.hw_version_major = (byte)(byte)251;
            p311.hw_version_minor = (byte)(byte)25;
            p311.hw_unique_id_SET(new byte[16], 0);
            p311.sw_version_major = (byte)(byte)234;
            p311.sw_version_minor = (byte)(byte)146;
            p311.sw_vcs_commit = (uint)273064584U;
            CommunicationChannel.instance.send(p311); //===============================
            PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.target_system = (byte)(byte)141;
            p320.target_component = (byte)(byte)106;
            p320.param_id_SET("DEMO", PH);
            p320.param_index = (short)(short) -3434;
            CommunicationChannel.instance.send(p320); //===============================
            PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_system = (byte)(byte)8;
            p321.target_component = (byte)(byte)74;
            CommunicationChannel.instance.send(p321); //===============================
            PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_id_SET("DEMO", PH);
            p322.param_value_SET("DEMO", PH);
            p322.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16;
            p322.param_count = (ushort)(ushort)33005;
            p322.param_index = (ushort)(ushort)6835;
            CommunicationChannel.instance.send(p322); //===============================
            PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.target_system = (byte)(byte)177;
            p323.target_component = (byte)(byte)36;
            p323.param_id_SET("DEMO", PH);
            p323.param_value_SET("DEMO", PH);
            p323.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM;
            CommunicationChannel.instance.send(p323); //===============================
            PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_id_SET("DEMO", PH);
            p324.param_value_SET("DEMO", PH);
            p324.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32;
            p324.param_result = (PARAM_ACK)PARAM_ACK.PARAM_ACK_ACCEPTED;
            CommunicationChannel.instance.send(p324); //===============================
            OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.time_usec = (ulong)254671687689042318L;
            p330.sensor_type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED;
            p330.distances_SET(new ushort[72], 0);
            p330.increment = (byte)(byte)168;
            p330.min_distance = (ushort)(ushort)28709;
            p330.max_distance = (ushort)(ushort)10393;
            CommunicationChannel.instance.send(p330); //===============================
        }
    }
}


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
            p3.time_boot_ms = (uint)87214344U;
            p3.coordinate_frame = MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p3.type_mask = (ushort)(ushort)50186;
            p3.x = (float)2.4057716E37F;
            p3.y = (float) -2.7246294E37F;
            p3.z = (float) -3.3519266E38F;
            p3.vx = (float)3.0524102E38F;
            p3.vy = (float)3.2460356E38F;
            p3.vz = (float) -2.5324556E38F;
            p3.afx = (float)3.081506E36F;
            p3.afy = (float) -2.371936E38F;
            p3.afz = (float) -2.1324712E38F;
            p3.yaw = (float) -1.5358386E38F;
            p3.yaw_rate = (float)9.139279E37F;
            CommunicationChannel.instance.send(p3); //===============================
            COMMAND_INT p75 = CommunicationChannel.new_COMMAND_INT();
            PH.setPack(p75);
            p75.target_system = (byte)(byte)154;
            p75.target_component = (byte)(byte)10;
            p75.frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
            p75.command = MAV_CMD.MAV_CMD_DO_SET_REVERSE;
            p75.current = (byte)(byte)83;
            p75.autocontinue = (byte)(byte)86;
            p75.param1 = (float)1.0895534E38F;
            p75.param2 = (float) -1.2935325E38F;
            p75.param3 = (float) -1.350051E38F;
            p75.param4 = (float) -2.2107269E38F;
            p75.x = (int) -833404875;
            p75.y = (int) -542264312;
            p75.z = (float)1.6754803E38F;
            CommunicationChannel.instance.send(p75); //===============================
            COMMAND_LONG p76 = CommunicationChannel.new_COMMAND_LONG();
            PH.setPack(p76);
            p76.target_system = (byte)(byte)157;
            p76.target_component = (byte)(byte)255;
            p76.command = MAV_CMD.MAV_CMD_DO_GO_AROUND;
            p76.confirmation = (byte)(byte)164;
            p76.param1 = (float) -2.4322836E38F;
            p76.param2 = (float)6.356006E37F;
            p76.param3 = (float)1.8682947E38F;
            p76.param4 = (float)3.1586926E38F;
            p76.param5 = (float) -2.2320453E38F;
            p76.param6 = (float)1.7006802E38F;
            p76.param7 = (float) -1.2993628E37F;
            CommunicationChannel.instance.send(p76); //===============================
            COMMAND_ACK p77 = CommunicationChannel.new_COMMAND_ACK();
            PH.setPack(p77);
            p77.command = MAV_CMD.MAV_CMD_USER_1;
            p77.result = MAV_RESULT.MAV_RESULT_FAILED;
            p77.progress_SET((byte)(byte)251, PH);
            p77.result_param2_SET((int)1639052600, PH);
            p77.target_system_SET((byte)(byte)124, PH);
            p77.target_component_SET((byte)(byte)171, PH);
            CommunicationChannel.instance.send(p77); //===============================
            MANUAL_SETPOINT p81 = CommunicationChannel.new_MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.time_boot_ms = (uint)1312691294U;
            p81.roll = (float)1.549208E38F;
            p81.pitch = (float)1.5380247E38F;
            p81.yaw = (float)2.3724982E38F;
            p81.thrust = (float) -2.7692929E38F;
            p81.mode_switch = (byte)(byte)9;
            p81.manual_override_switch = (byte)(byte)22;
            CommunicationChannel.instance.send(p81); //===============================
            SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.time_boot_ms = (uint)1584926325U;
            p82.target_system = (byte)(byte)231;
            p82.target_component = (byte)(byte)162;
            p82.type_mask = (byte)(byte)106;
            p82.q_SET(new float[4], 0);
            p82.body_roll_rate = (float)2.215097E38F;
            p82.body_pitch_rate = (float)3.4252246E37F;
            p82.body_yaw_rate = (float)8.3447923E37F;
            p82.thrust = (float) -2.472728E38F;
            CommunicationChannel.instance.send(p82); //===============================
            ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.time_boot_ms = (uint)4133034720U;
            p83.type_mask = (byte)(byte)118;
            p83.q_SET(new float[4], 0);
            p83.body_roll_rate = (float) -2.2184368E38F;
            p83.body_pitch_rate = (float)1.1940523E38F;
            p83.body_yaw_rate = (float)2.7314808E38F;
            p83.thrust = (float)2.3727572E38F;
            CommunicationChannel.instance.send(p83); //===============================
            SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.time_boot_ms = (uint)3743350446U;
            p84.target_system = (byte)(byte)173;
            p84.target_component = (byte)(byte)50;
            p84.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_ENU;
            p84.type_mask = (ushort)(ushort)18723;
            p84.x = (float) -2.3481133E37F;
            p84.y = (float)1.4396463E38F;
            p84.z = (float)2.5668667E38F;
            p84.vx = (float)1.5149106E38F;
            p84.vy = (float) -5.42907E37F;
            p84.vz = (float) -5.784957E37F;
            p84.afx = (float)3.3831035E38F;
            p84.afy = (float)2.4797882E37F;
            p84.afz = (float) -2.6877032E38F;
            p84.yaw = (float) -3.0141836E37F;
            p84.yaw_rate = (float)9.994386E37F;
            CommunicationChannel.instance.send(p84); //===============================
            SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.time_boot_ms = (uint)3062813630U;
            p86.target_system = (byte)(byte)187;
            p86.target_component = (byte)(byte)67;
            p86.coordinate_frame = MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p86.type_mask = (ushort)(ushort)5381;
            p86.lat_int = (int) -512471338;
            p86.lon_int = (int) -1789565254;
            p86.alt = (float) -3.014054E38F;
            p86.vx = (float) -2.5808535E38F;
            p86.vy = (float) -3.255363E38F;
            p86.vz = (float)9.294484E37F;
            p86.afx = (float) -7.992166E37F;
            p86.afy = (float)4.650253E37F;
            p86.afz = (float) -2.2450788E36F;
            p86.yaw = (float) -2.2211023E38F;
            p86.yaw_rate = (float)1.4552034E37F;
            CommunicationChannel.instance.send(p86); //===============================
            POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.time_boot_ms = (uint)3942770132U;
            p87.coordinate_frame = MAV_FRAME.MAV_FRAME_MISSION;
            p87.type_mask = (ushort)(ushort)7056;
            p87.lat_int = (int)1356572638;
            p87.lon_int = (int)2127206309;
            p87.alt = (float)4.8421236E37F;
            p87.vx = (float) -5.2551617E37F;
            p87.vy = (float)2.2003242E38F;
            p87.vz = (float)1.5665432E38F;
            p87.afx = (float)1.690415E38F;
            p87.afy = (float)3.3068635E38F;
            p87.afz = (float) -2.4204713E37F;
            p87.yaw = (float)2.8207595E38F;
            p87.yaw_rate = (float) -1.01473624E37F;
            CommunicationChannel.instance.send(p87); //===============================
            LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.time_boot_ms = (uint)3430692278U;
            p89.x = (float)3.7980868E37F;
            p89.y = (float) -1.1968115E38F;
            p89.z = (float)5.8710386E37F;
            p89.roll = (float)1.0862719E38F;
            p89.pitch = (float) -1.9281063E38F;
            p89.yaw = (float) -1.6172388E38F;
            CommunicationChannel.instance.send(p89); //===============================
            HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.time_usec = (ulong)2675404504985455373L;
            p90.roll = (float)4.885883E37F;
            p90.pitch = (float) -1.1947053E38F;
            p90.yaw = (float)2.8787735E38F;
            p90.rollspeed = (float)2.7444743E38F;
            p90.pitchspeed = (float)3.3445124E38F;
            p90.yawspeed = (float)3.0670224E38F;
            p90.lat = (int) -1471784356;
            p90.lon = (int) -1821419314;
            p90.alt = (int)1948860062;
            p90.vx = (short)(short)28835;
            p90.vy = (short)(short) -9912;
            p90.vz = (short)(short) -32588;
            p90.xacc = (short)(short)13798;
            p90.yacc = (short)(short)2806;
            p90.zacc = (short)(short)3509;
            CommunicationChannel.instance.send(p90); //===============================
            HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.time_usec = (ulong)8625588939338735138L;
            p91.roll_ailerons = (float)2.8262647E38F;
            p91.pitch_elevator = (float)7.885734E37F;
            p91.yaw_rudder = (float)1.9367448E38F;
            p91.throttle = (float)6.020515E37F;
            p91.aux1 = (float)9.468861E37F;
            p91.aux2 = (float) -1.4558177E38F;
            p91.aux3 = (float)3.3617496E38F;
            p91.aux4 = (float)2.1245948E38F;
            p91.mode = MAV_MODE.MAV_MODE_GUIDED_DISARMED;
            p91.nav_mode = (byte)(byte)88;
            CommunicationChannel.instance.send(p91); //===============================
            HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.time_usec = (ulong)666935894560507255L;
            p92.chan1_raw = (ushort)(ushort)54202;
            p92.chan2_raw = (ushort)(ushort)51836;
            p92.chan3_raw = (ushort)(ushort)54168;
            p92.chan4_raw = (ushort)(ushort)19936;
            p92.chan5_raw = (ushort)(ushort)41611;
            p92.chan6_raw = (ushort)(ushort)33988;
            p92.chan7_raw = (ushort)(ushort)30257;
            p92.chan8_raw = (ushort)(ushort)29089;
            p92.chan9_raw = (ushort)(ushort)11865;
            p92.chan10_raw = (ushort)(ushort)36908;
            p92.chan11_raw = (ushort)(ushort)3246;
            p92.chan12_raw = (ushort)(ushort)28947;
            p92.rssi = (byte)(byte)170;
            CommunicationChannel.instance.send(p92); //===============================
            HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.time_usec = (ulong)3420986054701428300L;
            p93.controls_SET(new float[16], 0);
            p93.mode = MAV_MODE.MAV_MODE_AUTO_DISARMED;
            p93.flags = (ulong)2904030227826993232L;
            CommunicationChannel.instance.send(p93); //===============================
            OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.time_usec = (ulong)2028728984709947595L;
            p100.sensor_id = (byte)(byte)16;
            p100.flow_x = (short)(short) -5718;
            p100.flow_y = (short)(short)10443;
            p100.flow_comp_m_x = (float)1.3138347E38F;
            p100.flow_comp_m_y = (float) -1.8323969E38F;
            p100.quality = (byte)(byte)12;
            p100.ground_distance = (float) -2.5995352E38F;
            p100.flow_rate_x_SET((float)2.646987E38F, PH);
            p100.flow_rate_y_SET((float)7.1752656E36F, PH);
            CommunicationChannel.instance.send(p100); //===============================
            GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.usec = (ulong)3449623470592738725L;
            p101.x = (float) -1.6824308E38F;
            p101.y = (float)3.1703183E38F;
            p101.z = (float) -3.1604286E38F;
            p101.roll = (float) -2.8259325E38F;
            p101.pitch = (float)2.1911495E37F;
            p101.yaw = (float)3.1772977E38F;
            CommunicationChannel.instance.send(p101); //===============================
            VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.usec = (ulong)167619983150400119L;
            p102.x = (float) -2.3637895E38F;
            p102.y = (float) -3.0256978E38F;
            p102.z = (float) -2.7624607E38F;
            p102.roll = (float) -3.5998136E37F;
            p102.pitch = (float) -1.1349687E38F;
            p102.yaw = (float)2.55402E38F;
            CommunicationChannel.instance.send(p102); //===============================
            VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.usec = (ulong)1511905099746868696L;
            p103.x = (float) -2.8926714E38F;
            p103.y = (float)1.3308042E38F;
            p103.z = (float) -1.0523545E38F;
            CommunicationChannel.instance.send(p103); //===============================
            VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.usec = (ulong)2950176529356052138L;
            p104.x = (float) -1.6367393E38F;
            p104.y = (float)1.165402E38F;
            p104.z = (float)1.0332017E38F;
            p104.roll = (float) -2.6687315E38F;
            p104.pitch = (float) -2.9992674E37F;
            p104.yaw = (float) -3.1683306E38F;
            CommunicationChannel.instance.send(p104); //===============================
            HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.time_usec = (ulong)5018576416911972324L;
            p105.xacc = (float)1.4797888E38F;
            p105.yacc = (float) -2.8797261E38F;
            p105.zacc = (float)8.797244E37F;
            p105.xgyro = (float) -2.9974142E38F;
            p105.ygyro = (float) -8.3151277E37F;
            p105.zgyro = (float)9.546421E37F;
            p105.xmag = (float) -6.3765497E37F;
            p105.ymag = (float) -1.0248112E38F;
            p105.zmag = (float) -5.275005E37F;
            p105.abs_pressure = (float) -3.2979366E38F;
            p105.diff_pressure = (float) -2.7658526E37F;
            p105.pressure_alt = (float) -2.2565134E38F;
            p105.temperature = (float)1.2325217E38F;
            p105.fields_updated = (ushort)(ushort)19266;
            CommunicationChannel.instance.send(p105); //===============================
            OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.time_usec = (ulong)5433746683812445123L;
            p106.sensor_id = (byte)(byte)99;
            p106.integration_time_us = (uint)4287016890U;
            p106.integrated_x = (float)2.730016E38F;
            p106.integrated_y = (float)1.5280846E38F;
            p106.integrated_xgyro = (float)2.0220455E38F;
            p106.integrated_ygyro = (float)1.0051736E38F;
            p106.integrated_zgyro = (float)1.3785498E38F;
            p106.temperature = (short)(short)25874;
            p106.quality = (byte)(byte)149;
            p106.time_delta_distance_us = (uint)1802316350U;
            p106.distance = (float) -7.2898154E37F;
            CommunicationChannel.instance.send(p106); //===============================
            HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.time_usec = (ulong)8946547968434735222L;
            p107.xacc = (float)1.718357E38F;
            p107.yacc = (float) -1.7677663E38F;
            p107.zacc = (float)2.1099314E38F;
            p107.xgyro = (float)7.346857E37F;
            p107.ygyro = (float) -1.2113685E38F;
            p107.zgyro = (float)2.6497165E38F;
            p107.xmag = (float) -6.01495E37F;
            p107.ymag = (float) -8.66137E37F;
            p107.zmag = (float)1.8193804E38F;
            p107.abs_pressure = (float)3.0902563E38F;
            p107.diff_pressure = (float) -7.152807E37F;
            p107.pressure_alt = (float) -5.720999E37F;
            p107.temperature = (float) -1.499906E37F;
            p107.fields_updated = (uint)428880469U;
            CommunicationChannel.instance.send(p107); //===============================
            SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.q1 = (float)2.7554256E38F;
            p108.q2 = (float) -8.4119945E37F;
            p108.q3 = (float) -3.389599E38F;
            p108.q4 = (float)2.4479226E38F;
            p108.roll = (float) -1.2306513E38F;
            p108.pitch = (float)2.1151866E38F;
            p108.yaw = (float) -6.2044236E37F;
            p108.xacc = (float) -1.1078886E38F;
            p108.yacc = (float) -8.9512954E36F;
            p108.zacc = (float) -1.3915916E38F;
            p108.xgyro = (float)6.786759E37F;
            p108.ygyro = (float) -7.9356834E37F;
            p108.zgyro = (float) -1.9681444E38F;
            p108.lat = (float)2.7258157E38F;
            p108.lon = (float)2.1486511E38F;
            p108.alt = (float)3.1254124E38F;
            p108.std_dev_horz = (float)1.7123674E38F;
            p108.std_dev_vert = (float) -6.66816E37F;
            p108.vn = (float) -1.0699817E38F;
            p108.ve = (float) -1.7528026E38F;
            p108.vd = (float) -2.799495E38F;
            CommunicationChannel.instance.send(p108); //===============================
            RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.rssi = (byte)(byte)29;
            p109.remrssi = (byte)(byte)226;
            p109.txbuf = (byte)(byte)104;
            p109.noise = (byte)(byte)155;
            p109.remnoise = (byte)(byte)28;
            p109.rxerrors = (ushort)(ushort)41276;
            p109.fixed_ = (ushort)(ushort)60128;
            CommunicationChannel.instance.send(p109); //===============================
            FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_network = (byte)(byte)116;
            p110.target_system = (byte)(byte)41;
            p110.target_component = (byte)(byte)225;
            p110.payload_SET(new byte[251], 0);
            CommunicationChannel.instance.send(p110); //===============================
            TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.tc1 = (long) -3625683210484954238L;
            p111.ts1 = (long)6594470323921158950L;
            CommunicationChannel.instance.send(p111); //===============================
            CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.time_usec = (ulong)3305888519007619056L;
            p112.seq = (uint)2483976438U;
            CommunicationChannel.instance.send(p112); //===============================
            HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.time_usec = (ulong)8810967105084412463L;
            p113.fix_type = (byte)(byte)198;
            p113.lat = (int) -1474414465;
            p113.lon = (int)105827397;
            p113.alt = (int)984453940;
            p113.eph = (ushort)(ushort)10835;
            p113.epv = (ushort)(ushort)51444;
            p113.vel = (ushort)(ushort)44800;
            p113.vn = (short)(short) -4449;
            p113.ve = (short)(short) -15821;
            p113.vd = (short)(short)21538;
            p113.cog = (ushort)(ushort)52768;
            p113.satellites_visible = (byte)(byte)2;
            CommunicationChannel.instance.send(p113); //===============================
            HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.time_usec = (ulong)7624465084841759184L;
            p114.sensor_id = (byte)(byte)43;
            p114.integration_time_us = (uint)3990349639U;
            p114.integrated_x = (float)1.6193403E38F;
            p114.integrated_y = (float)5.8210313E37F;
            p114.integrated_xgyro = (float) -4.8362483E37F;
            p114.integrated_ygyro = (float)1.1431196E38F;
            p114.integrated_zgyro = (float)1.7954001E38F;
            p114.temperature = (short)(short) -28746;
            p114.quality = (byte)(byte)22;
            p114.time_delta_distance_us = (uint)3755217371U;
            p114.distance = (float) -2.971609E37F;
            CommunicationChannel.instance.send(p114); //===============================
            HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.time_usec = (ulong)8935869262137367558L;
            p115.attitude_quaternion_SET(new float[4], 0);
            p115.rollspeed = (float) -6.5512665E37F;
            p115.pitchspeed = (float)6.663345E37F;
            p115.yawspeed = (float)2.5779993E38F;
            p115.lat = (int)327710343;
            p115.lon = (int) -1431536414;
            p115.alt = (int) -951757209;
            p115.vx = (short)(short)17084;
            p115.vy = (short)(short)29492;
            p115.vz = (short)(short)23984;
            p115.ind_airspeed = (ushort)(ushort)61528;
            p115.true_airspeed = (ushort)(ushort)53573;
            p115.xacc = (short)(short)31585;
            p115.yacc = (short)(short) -26258;
            p115.zacc = (short)(short) -27054;
            CommunicationChannel.instance.send(p115); //===============================
            SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.time_boot_ms = (uint)2130566031U;
            p116.xacc = (short)(short) -32210;
            p116.yacc = (short)(short)25678;
            p116.zacc = (short)(short)29389;
            p116.xgyro = (short)(short)21045;
            p116.ygyro = (short)(short)17169;
            p116.zgyro = (short)(short)6883;
            p116.xmag = (short)(short) -741;
            p116.ymag = (short)(short) -7091;
            p116.zmag = (short)(short)2118;
            CommunicationChannel.instance.send(p116); //===============================
            LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.target_system = (byte)(byte)204;
            p117.target_component = (byte)(byte)75;
            p117.start = (ushort)(ushort)62317;
            p117.end = (ushort)(ushort)42361;
            CommunicationChannel.instance.send(p117); //===============================
            LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.id = (ushort)(ushort)4343;
            p118.num_logs = (ushort)(ushort)46232;
            p118.last_log_num = (ushort)(ushort)17443;
            p118.time_utc = (uint)3224740914U;
            p118.size = (uint)2552656047U;
            CommunicationChannel.instance.send(p118); //===============================
            LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.target_system = (byte)(byte)162;
            p119.target_component = (byte)(byte)124;
            p119.id = (ushort)(ushort)3885;
            p119.ofs = (uint)4013727066U;
            p119.count = (uint)3407967317U;
            CommunicationChannel.instance.send(p119); //===============================
            LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.id = (ushort)(ushort)8075;
            p120.ofs = (uint)2997730220U;
            p120.count = (byte)(byte)199;
            p120.data__SET(new byte[90], 0);
            CommunicationChannel.instance.send(p120); //===============================
            LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_system = (byte)(byte)233;
            p121.target_component = (byte)(byte)24;
            CommunicationChannel.instance.send(p121); //===============================
            LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_system = (byte)(byte)168;
            p122.target_component = (byte)(byte)21;
            CommunicationChannel.instance.send(p122); //===============================
            GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.target_system = (byte)(byte)40;
            p123.target_component = (byte)(byte)102;
            p123.len = (byte)(byte)121;
            p123.data__SET(new byte[110], 0);
            CommunicationChannel.instance.send(p123); //===============================
            GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.time_usec = (ulong)1390621067837715031L;
            p124.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX;
            p124.lat = (int) -401874791;
            p124.lon = (int)16487960;
            p124.alt = (int) -622791948;
            p124.eph = (ushort)(ushort)45250;
            p124.epv = (ushort)(ushort)16362;
            p124.vel = (ushort)(ushort)21289;
            p124.cog = (ushort)(ushort)56561;
            p124.satellites_visible = (byte)(byte)149;
            p124.dgps_numch = (byte)(byte)106;
            p124.dgps_age = (uint)957992099U;
            CommunicationChannel.instance.send(p124); //===============================
            POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vcc = (ushort)(ushort)61004;
            p125.Vservo = (ushort)(ushort)21437;
            p125.flags = (MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID);
            CommunicationChannel.instance.send(p125); //===============================
            SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.device = SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2;
            p126.flags = (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND |
                          SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                          SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI);
            p126.timeout = (ushort)(ushort)9332;
            p126.baudrate = (uint)651295392U;
            p126.count = (byte)(byte)39;
            p126.data__SET(new byte[70], 0);
            CommunicationChannel.instance.send(p126); //===============================
            GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.time_last_baseline_ms = (uint)3011478728U;
            p127.rtk_receiver_id = (byte)(byte)47;
            p127.wn = (ushort)(ushort)24259;
            p127.tow = (uint)2599359924U;
            p127.rtk_health = (byte)(byte)200;
            p127.rtk_rate = (byte)(byte)138;
            p127.nsats = (byte)(byte)140;
            p127.baseline_coords_type = (byte)(byte)16;
            p127.baseline_a_mm = (int)1005208829;
            p127.baseline_b_mm = (int) -460595739;
            p127.baseline_c_mm = (int)1001652362;
            p127.accuracy = (uint)2884254553U;
            p127.iar_num_hypotheses = (int)1580658906;
            CommunicationChannel.instance.send(p127); //===============================
            GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.time_last_baseline_ms = (uint)886785186U;
            p128.rtk_receiver_id = (byte)(byte)196;
            p128.wn = (ushort)(ushort)21583;
            p128.tow = (uint)4018910589U;
            p128.rtk_health = (byte)(byte)115;
            p128.rtk_rate = (byte)(byte)230;
            p128.nsats = (byte)(byte)104;
            p128.baseline_coords_type = (byte)(byte)94;
            p128.baseline_a_mm = (int)388151108;
            p128.baseline_b_mm = (int) -854697296;
            p128.baseline_c_mm = (int) -1443144147;
            p128.accuracy = (uint)1073920694U;
            p128.iar_num_hypotheses = (int) -455864335;
            CommunicationChannel.instance.send(p128); //===============================
            SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.time_boot_ms = (uint)651816234U;
            p129.xacc = (short)(short)3081;
            p129.yacc = (short)(short)32242;
            p129.zacc = (short)(short)23827;
            p129.xgyro = (short)(short)8916;
            p129.ygyro = (short)(short) -26805;
            p129.zgyro = (short)(short)11891;
            p129.xmag = (short)(short) -1866;
            p129.ymag = (short)(short) -32611;
            p129.zmag = (short)(short) -18121;
            CommunicationChannel.instance.send(p129); //===============================
            DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.type = (byte)(byte)93;
            p130.size = (uint)3643881398U;
            p130.width = (ushort)(ushort)57550;
            p130.height = (ushort)(ushort)8985;
            p130.packets = (ushort)(ushort)60641;
            p130.payload = (byte)(byte)78;
            p130.jpg_quality = (byte)(byte)88;
            CommunicationChannel.instance.send(p130); //===============================
            ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.seqnr = (ushort)(ushort)60263;
            p131.data__SET(new byte[253], 0);
            CommunicationChannel.instance.send(p131); //===============================
            DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.time_boot_ms = (uint)1523166136U;
            p132.min_distance = (ushort)(ushort)46857;
            p132.max_distance = (ushort)(ushort)56339;
            p132.current_distance = (ushort)(ushort)24097;
            p132.type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED;
            p132.id = (byte)(byte)167;
            p132.orientation = MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_YAW_45;
            p132.covariance = (byte)(byte)218;
            CommunicationChannel.instance.send(p132); //===============================
            TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.lat = (int) -1874768783;
            p133.lon = (int) -1340060202;
            p133.grid_spacing = (ushort)(ushort)11285;
            p133.mask = (ulong)3609625944944070274L;
            CommunicationChannel.instance.send(p133); //===============================
            TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.lat = (int) -1221373438;
            p134.lon = (int) -1391431665;
            p134.grid_spacing = (ushort)(ushort)742;
            p134.gridbit = (byte)(byte)149;
            p134.data__SET(new short[16], 0);
            CommunicationChannel.instance.send(p134); //===============================
            TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lat = (int)879658774;
            p135.lon = (int)67721090;
            CommunicationChannel.instance.send(p135); //===============================
            TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.lat = (int)1882565136;
            p136.lon = (int)643212329;
            p136.spacing = (ushort)(ushort)8333;
            p136.terrain_height = (float) -1.0393533E38F;
            p136.current_height = (float) -3.3309507E37F;
            p136.pending = (ushort)(ushort)12717;
            p136.loaded = (ushort)(ushort)59474;
            CommunicationChannel.instance.send(p136); //===============================
            SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.time_boot_ms = (uint)1890065497U;
            p137.press_abs = (float) -3.74096E37F;
            p137.press_diff = (float) -2.8136436E38F;
            p137.temperature = (short)(short)16985;
            CommunicationChannel.instance.send(p137); //===============================
            ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.time_usec = (ulong)2005716376889465373L;
            p138.q_SET(new float[4], 0);
            p138.x = (float) -2.3246597E38F;
            p138.y = (float)1.9261687E38F;
            p138.z = (float)3.2166804E38F;
            CommunicationChannel.instance.send(p138); //===============================
            SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.time_usec = (ulong)3843221931226629758L;
            p139.group_mlx = (byte)(byte)179;
            p139.target_system = (byte)(byte)139;
            p139.target_component = (byte)(byte)181;
            p139.controls_SET(new float[8], 0);
            CommunicationChannel.instance.send(p139); //===============================
            ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.time_usec = (ulong)3922002672343680537L;
            p140.group_mlx = (byte)(byte)6;
            p140.controls_SET(new float[8], 0);
            CommunicationChannel.instance.send(p140); //===============================
            ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.time_usec = (ulong)4031216405280455685L;
            p141.altitude_monotonic = (float) -6.6142723E37F;
            p141.altitude_amsl = (float)3.3629318E38F;
            p141.altitude_local = (float)1.7247222E38F;
            p141.altitude_relative = (float) -1.2982399E38F;
            p141.altitude_terrain = (float) -2.0855603E38F;
            p141.bottom_clearance = (float)1.2478929E38F;
            CommunicationChannel.instance.send(p141); //===============================
            RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.request_id = (byte)(byte)98;
            p142.uri_type = (byte)(byte)180;
            p142.uri_SET(new byte[120], 0);
            p142.transfer_type = (byte)(byte)37;
            p142.storage_SET(new byte[120], 0);
            CommunicationChannel.instance.send(p142); //===============================
            SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.time_boot_ms = (uint)394020892U;
            p143.press_abs = (float) -2.7671295E38F;
            p143.press_diff = (float)4.2601974E37F;
            p143.temperature = (short)(short) -19151;
            CommunicationChannel.instance.send(p143); //===============================
            FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.timestamp = (ulong)2627302001944507454L;
            p144.est_capabilities = (byte)(byte)226;
            p144.lat = (int)640939638;
            p144.lon = (int) -1049114327;
            p144.alt = (float) -1.3314275E38F;
            p144.vel_SET(new float[3], 0);
            p144.acc_SET(new float[3], 0);
            p144.attitude_q_SET(new float[4], 0);
            p144.rates_SET(new float[3], 0);
            p144.position_cov_SET(new float[3], 0);
            p144.custom_state = (ulong)3660022960663741877L;
            CommunicationChannel.instance.send(p144); //===============================
            CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.time_usec = (ulong)2105115876662929741L;
            p146.x_acc = (float)3.1481214E38F;
            p146.y_acc = (float)6.617228E37F;
            p146.z_acc = (float)1.7559028E38F;
            p146.x_vel = (float)5.2319834E37F;
            p146.y_vel = (float)2.5854152E38F;
            p146.z_vel = (float) -6.3243104E37F;
            p146.x_pos = (float)6.4246763E37F;
            p146.y_pos = (float) -5.002673E37F;
            p146.z_pos = (float)1.2423692E38F;
            p146.airspeed = (float)1.5352995E38F;
            p146.vel_variance_SET(new float[3], 0);
            p146.pos_variance_SET(new float[3], 0);
            p146.q_SET(new float[4], 0);
            p146.roll_rate = (float)3.2225577E38F;
            p146.pitch_rate = (float)2.924818E38F;
            p146.yaw_rate = (float) -2.944646E38F;
            CommunicationChannel.instance.send(p146); //===============================
            BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.id = (byte)(byte)89;
            p147.battery_function = MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_ALL;
            p147.type = MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIFE;
            p147.temperature = (short)(short)11440;
            p147.voltages_SET(new ushort[10], 0);
            p147.current_battery = (short)(short) -14289;
            p147.current_consumed = (int) -1679824049;
            p147.energy_consumed = (int)1136632867;
            p147.battery_remaining = (sbyte)(sbyte)26;
            CommunicationChannel.instance.send(p147); //===============================
            AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_TERRAIN |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMMAND_INT);
            p148.flight_sw_version = (uint)219415255U;
            p148.middleware_sw_version = (uint)1098594838U;
            p148.os_sw_version = (uint)2848413929U;
            p148.board_version = (uint)504811588U;
            p148.flight_custom_version_SET(new byte[8], 0);
            p148.middleware_custom_version_SET(new byte[8], 0);
            p148.os_custom_version_SET(new byte[8], 0);
            p148.vendor_id = (ushort)(ushort)33902;
            p148.product_id = (ushort)(ushort)13093;
            p148.uid = (ulong)1499545935430205315L;
            p148.uid2_SET(new byte[18], 0, PH);
            CommunicationChannel.instance.send(p148); //===============================
            LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.time_usec = (ulong)7582224179629057283L;
            p149.target_num = (byte)(byte)60;
            p149.frame = MAV_FRAME.MAV_FRAME_MISSION;
            p149.angle_x = (float)2.6869436E38F;
            p149.angle_y = (float)2.716834E38F;
            p149.distance = (float) -2.4687175E38F;
            p149.size_x = (float) -3.2158547E38F;
            p149.size_y = (float) -6.92719E37F;
            p149.x_SET((float) -9.920911E37F, PH);
            p149.y_SET((float) -2.1196923E38F, PH);
            p149.z_SET((float)2.1846717E38F, PH);
            p149.q_SET(new float[4], 0, PH);
            p149.type = LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER;
            p149.position_valid_SET((byte)(byte)75, PH);
            CommunicationChannel.instance.send(p149); //===============================
            AQ_TELEMETRY_F p150 = CommunicationChannel.new_AQ_TELEMETRY_F();
            PH.setPack(p150);
            p150.Index = (ushort)(ushort)31141;
            p150.value1 = (float) -2.6316198E37F;
            p150.value2 = (float) -1.1369786E38F;
            p150.value3 = (float)1.5432112E38F;
            p150.value4 = (float)2.0666735E38F;
            p150.value5 = (float) -2.0096783E38F;
            p150.value6 = (float)1.9778956E38F;
            p150.value7 = (float)2.8609874E38F;
            p150.value8 = (float) -6.761833E37F;
            p150.value9 = (float) -1.6085813E38F;
            p150.value10 = (float)2.7645508E38F;
            p150.value11 = (float)5.2484056E37F;
            p150.value12 = (float)2.3825698E37F;
            p150.value13 = (float) -1.10585E37F;
            p150.value14 = (float) -2.4899538E38F;
            p150.value15 = (float) -1.1554268E38F;
            p150.value16 = (float) -1.6711573E38F;
            p150.value17 = (float) -2.876363E38F;
            p150.value18 = (float) -2.1278355E38F;
            p150.value19 = (float)2.9162462E38F;
            p150.value20 = (float) -2.3318504E38F;
            CommunicationChannel.instance.send(p150); //===============================
            AQ_ESC_TELEMETRY p152 = CommunicationChannel.new_AQ_ESC_TELEMETRY();
            PH.setPack(p152);
            p152.time_boot_ms = (uint)2111278202U;
            p152.seq = (byte)(byte)69;
            p152.num_motors = (byte)(byte)85;
            p152.num_in_seq = (byte)(byte)16;
            p152.escid_SET(new byte[4], 0);
            p152.status_age_SET(new ushort[4], 0);
            p152.data_version_SET(new byte[4], 0);
            p152.data0_SET(new uint[4], 0);
            p152.data1_SET(new uint[4], 0);
            CommunicationChannel.instance.send(p152); //===============================
            ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.time_usec = (ulong)4333017280155496773L;
            p230.flags = (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE);
            p230.vel_ratio = (float) -2.5368153E37F;
            p230.pos_horiz_ratio = (float)1.2098486E37F;
            p230.pos_vert_ratio = (float) -1.2019486E38F;
            p230.mag_ratio = (float) -1.7731237E38F;
            p230.hagl_ratio = (float) -1.1253428E38F;
            p230.tas_ratio = (float) -2.3017373E38F;
            p230.pos_horiz_accuracy = (float)2.1911752E38F;
            p230.pos_vert_accuracy = (float)1.647025E38F;
            CommunicationChannel.instance.send(p230); //===============================
            WIND_COV p231 = CommunicationChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.time_usec = (ulong)1212519755642221249L;
            p231.wind_x = (float)2.7377533E38F;
            p231.wind_y = (float)1.1895299E38F;
            p231.wind_z = (float) -1.9411724E36F;
            p231.var_horiz = (float) -9.562767E37F;
            p231.var_vert = (float)1.0050087E38F;
            p231.wind_alt = (float) -2.3713725E38F;
            p231.horiz_accuracy = (float) -2.2542168E38F;
            p231.vert_accuracy = (float) -2.6824795E38F;
            CommunicationChannel.instance.send(p231); //===============================
            GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.time_usec = (ulong)4574383088272688283L;
            p232.gps_id = (byte)(byte)143;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT);
            p232.time_week_ms = (uint)1436256302U;
            p232.time_week = (ushort)(ushort)10595;
            p232.fix_type = (byte)(byte)225;
            p232.lat = (int) -1304440918;
            p232.lon = (int)86240553;
            p232.alt = (float)1.590479E38F;
            p232.hdop = (float)5.075258E37F;
            p232.vdop = (float)3.3214626E38F;
            p232.vn = (float)1.1550626E38F;
            p232.ve = (float)8.62677E37F;
            p232.vd = (float) -2.1163745E38F;
            p232.speed_accuracy = (float)3.1630943E38F;
            p232.horiz_accuracy = (float)2.5546987E38F;
            p232.vert_accuracy = (float)1.7432985E38F;
            p232.satellites_visible = (byte)(byte)233;
            CommunicationChannel.instance.send(p232); //===============================
            GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.flags = (byte)(byte)84;
            p233.len = (byte)(byte)253;
            p233.data__SET(new byte[180], 0);
            CommunicationChannel.instance.send(p233); //===============================
            HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED);
            p234.custom_mode = (uint)4017411502U;
            p234.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND;
            p234.roll = (short)(short) -25413;
            p234.pitch = (short)(short)23559;
            p234.heading = (ushort)(ushort)60465;
            p234.throttle = (sbyte)(sbyte) - 78;
            p234.heading_sp = (short)(short) -31207;
            p234.latitude = (int)1377573720;
            p234.longitude = (int) -754084869;
            p234.altitude_amsl = (short)(short)28765;
            p234.altitude_sp = (short)(short) -15658;
            p234.airspeed = (byte)(byte)214;
            p234.airspeed_sp = (byte)(byte)197;
            p234.groundspeed = (byte)(byte)201;
            p234.climb_rate = (sbyte)(sbyte) - 26;
            p234.gps_nsat = (byte)(byte)184;
            p234.gps_fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS;
            p234.battery_remaining = (byte)(byte)30;
            p234.temperature = (sbyte)(sbyte)67;
            p234.temperature_air = (sbyte)(sbyte) - 46;
            p234.failsafe = (byte)(byte)181;
            p234.wp_num = (byte)(byte)15;
            p234.wp_distance = (ushort)(ushort)6383;
            CommunicationChannel.instance.send(p234); //===============================
            VIBRATION p241 = CommunicationChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.time_usec = (ulong)8717933449511676751L;
            p241.vibration_x = (float) -1.6737052E37F;
            p241.vibration_y = (float) -1.8515139E37F;
            p241.vibration_z = (float)8.2790966E37F;
            p241.clipping_0 = (uint)3422448446U;
            p241.clipping_1 = (uint)1099116379U;
            p241.clipping_2 = (uint)630412054U;
            CommunicationChannel.instance.send(p241); //===============================
            HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.latitude = (int) -1542834897;
            p242.longitude = (int) -199308703;
            p242.altitude = (int) -1950279020;
            p242.x = (float)3.2190581E38F;
            p242.y = (float)2.9142959E38F;
            p242.z = (float) -9.675763E37F;
            p242.q_SET(new float[4], 0);
            p242.approach_x = (float) -2.0603227E38F;
            p242.approach_y = (float) -1.748198E37F;
            p242.approach_z = (float) -1.7325257E38F;
            p242.time_usec_SET((ulong)8061659243436167934L, PH);
            CommunicationChannel.instance.send(p242); //===============================
            SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.target_system = (byte)(byte)1;
            p243.latitude = (int) -1006543975;
            p243.longitude = (int)664806310;
            p243.altitude = (int)47068681;
            p243.x = (float) -2.5122261E38F;
            p243.y = (float)2.9621626E38F;
            p243.z = (float) -6.5214326E37F;
            p243.q_SET(new float[4], 0);
            p243.approach_x = (float)6.8454365E37F;
            p243.approach_y = (float)8.995264E37F;
            p243.approach_z = (float) -1.3862172E38F;
            p243.time_usec_SET((ulong)336378186057447878L, PH);
            CommunicationChannel.instance.send(p243); //===============================
            MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.message_id = (ushort)(ushort)51600;
            p244.interval_us = (int)891572848;
            CommunicationChannel.instance.send(p244); //===============================
            EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.vtol_state = MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_MC;
            p245.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR;
            CommunicationChannel.instance.send(p245); //===============================
            ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.ICAO_address = (uint)1331182250U;
            p246.lat = (int) -1748525711;
            p246.lon = (int)2146130644;
            p246.altitude_type = ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
            p246.altitude = (int)357235019;
            p246.heading = (ushort)(ushort)53646;
            p246.hor_velocity = (ushort)(ushort)28251;
            p246.ver_velocity = (short)(short)4809;
            p246.callsign_SET("DEMO", PH);
            p246.emitter_type = ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_SPACE;
            p246.tslc = (byte)(byte)241;
            p246.flags = (ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING |
                          ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK |
                          ADSB_FLAGS.ADSB_FLAGS_SIMULATED);
            p246.squawk = (ushort)(ushort)59829;
            CommunicationChannel.instance.send(p246); //===============================
            COLLISION p247 = CommunicationChannel.new_COLLISION();
            PH.setPack(p247);
            p247.src_ = MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB;
            p247.id = (uint)548685881U;
            p247.action = MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_HORIZONTALLY;
            p247.threat_level = MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH;
            p247.time_to_minimum_delta = (float) -1.79162E38F;
            p247.altitude_minimum_delta = (float) -8.98843E37F;
            p247.horizontal_minimum_delta = (float)4.1057644E37F;
            CommunicationChannel.instance.send(p247); //===============================
            V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.target_network = (byte)(byte)13;
            p248.target_system = (byte)(byte)107;
            p248.target_component = (byte)(byte)3;
            p248.message_type = (ushort)(ushort)7648;
            p248.payload_SET(new byte[249], 0);
            CommunicationChannel.instance.send(p248); //===============================
            MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.address = (ushort)(ushort)12738;
            p249.ver = (byte)(byte)130;
            p249.type = (byte)(byte)243;
            p249.value_SET(new sbyte[32], 0);
            CommunicationChannel.instance.send(p249); //===============================
            DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.name_SET("DEMO", PH);
            p250.time_usec = (ulong)5630087198711815452L;
            p250.x = (float) -2.1091657E38F;
            p250.y = (float)1.4616296E38F;
            p250.z = (float) -1.1257891E36F;
            CommunicationChannel.instance.send(p250); //===============================
            NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.time_boot_ms = (uint)2436634621U;
            p251.name_SET("DEMO", PH);
            p251.value = (float) -1.2213943E38F;
            CommunicationChannel.instance.send(p251); //===============================
            NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.time_boot_ms = (uint)4203975435U;
            p252.name_SET("DEMO", PH);
            p252.value = (int)180129272;
            CommunicationChannel.instance.send(p252); //===============================
            STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.severity = MAV_SEVERITY.MAV_SEVERITY_ERROR;
            p253.text_SET("DEMO", PH);
            CommunicationChannel.instance.send(p253); //===============================
            DEBUG p254 = CommunicationChannel.new_DEBUG();
            PH.setPack(p254);
            p254.time_boot_ms = (uint)290395015U;
            p254.ind = (byte)(byte)204;
            p254.value = (float)3.2264748E38F;
            CommunicationChannel.instance.send(p254); //===============================
            SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.target_system = (byte)(byte)175;
            p256.target_component = (byte)(byte)98;
            p256.secret_key_SET(new byte[32], 0);
            p256.initial_timestamp = (ulong)5969968482954694584L;
            CommunicationChannel.instance.send(p256); //===============================
            BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.time_boot_ms = (uint)700681562U;
            p257.last_change_ms = (uint)3706035195U;
            p257.state = (byte)(byte)212;
            CommunicationChannel.instance.send(p257); //===============================
            PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_system = (byte)(byte)39;
            p258.target_component = (byte)(byte)174;
            p258.tune_SET("DEMO", PH);
            CommunicationChannel.instance.send(p258); //===============================
            CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.time_boot_ms = (uint)3053917784U;
            p259.vendor_name_SET(new byte[32], 0);
            p259.model_name_SET(new byte[32], 0);
            p259.firmware_version = (uint)1831003046U;
            p259.focal_length = (float)1.7127274E38F;
            p259.sensor_size_h = (float)2.8711511E38F;
            p259.sensor_size_v = (float) -2.249298E38F;
            p259.resolution_h = (ushort)(ushort)43995;
            p259.resolution_v = (ushort)(ushort)22390;
            p259.lens_id = (byte)(byte)165;
            p259.flags = (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE |
                          CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO);
            p259.cam_definition_version = (ushort)(ushort)31419;
            p259.cam_definition_uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p259); //===============================
            CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.time_boot_ms = (uint)3131815053U;
            p260.mode_id = CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY;
            CommunicationChannel.instance.send(p260); //===============================
            STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.time_boot_ms = (uint)2997613137U;
            p261.storage_id = (byte)(byte)140;
            p261.storage_count = (byte)(byte)224;
            p261.status = (byte)(byte)212;
            p261.total_capacity = (float)1.3086291E38F;
            p261.used_capacity = (float) -3.0214586E38F;
            p261.available_capacity = (float) -4.1458632E37F;
            p261.read_speed = (float)1.4884985E38F;
            p261.write_speed = (float)1.3668846E38F;
            CommunicationChannel.instance.send(p261); //===============================
            CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.time_boot_ms = (uint)2571006092U;
            p262.image_status = (byte)(byte)195;
            p262.video_status = (byte)(byte)233;
            p262.image_interval = (float) -2.6978795E38F;
            p262.recording_time_ms = (uint)1582243720U;
            p262.available_capacity = (float) -6.9393597E37F;
            CommunicationChannel.instance.send(p262); //===============================
            CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.time_boot_ms = (uint)1864886078U;
            p263.time_utc = (ulong)5747157535370331043L;
            p263.camera_id = (byte)(byte)91;
            p263.lat = (int)1419364869;
            p263.lon = (int)824032572;
            p263.alt = (int) -1446465562;
            p263.relative_alt = (int)1267646012;
            p263.q_SET(new float[4], 0);
            p263.image_index = (int)508855821;
            p263.capture_result = (sbyte)(sbyte)83;
            p263.file_url_SET("DEMO", PH);
            CommunicationChannel.instance.send(p263); //===============================
            FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.time_boot_ms = (uint)3390410527U;
            p264.arming_time_utc = (ulong)2579001409604450682L;
            p264.takeoff_time_utc = (ulong)1332289097951429502L;
            p264.flight_uuid = (ulong)818514515349390453L;
            CommunicationChannel.instance.send(p264); //===============================
            MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.time_boot_ms = (uint)3179218679U;
            p265.roll = (float) -2.3737097E37F;
            p265.pitch = (float) -1.5643494E38F;
            p265.yaw = (float) -8.57346E37F;
            CommunicationChannel.instance.send(p265); //===============================
            LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.target_system = (byte)(byte)221;
            p266.target_component = (byte)(byte)183;
            p266.sequence = (ushort)(ushort)5667;
            p266.length = (byte)(byte)130;
            p266.first_message_offset = (byte)(byte)153;
            p266.data__SET(new byte[249], 0);
            CommunicationChannel.instance.send(p266); //===============================
            LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.target_system = (byte)(byte)117;
            p267.target_component = (byte)(byte)86;
            p267.sequence = (ushort)(ushort)6883;
            p267.length = (byte)(byte)161;
            p267.first_message_offset = (byte)(byte)165;
            p267.data__SET(new byte[249], 0);
            CommunicationChannel.instance.send(p267); //===============================
            LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_system = (byte)(byte)26;
            p268.target_component = (byte)(byte)32;
            p268.sequence = (ushort)(ushort)37588;
            CommunicationChannel.instance.send(p268); //===============================
            VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.camera_id = (byte)(byte)137;
            p269.status = (byte)(byte)21;
            p269.framerate = (float) -1.193653E38F;
            p269.resolution_h = (ushort)(ushort)6989;
            p269.resolution_v = (ushort)(ushort)5945;
            p269.bitrate = (uint)2929328567U;
            p269.rotation = (ushort)(ushort)6847;
            p269.uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p269); //===============================
            SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.target_system = (byte)(byte)125;
            p270.target_component = (byte)(byte)54;
            p270.camera_id = (byte)(byte)126;
            p270.framerate = (float)3.1151114E38F;
            p270.resolution_h = (ushort)(ushort)27153;
            p270.resolution_v = (ushort)(ushort)61465;
            p270.bitrate = (uint)4211522831U;
            p270.rotation = (ushort)(ushort)10118;
            p270.uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p270); //===============================
            WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.ssid_SET("DEMO", PH);
            p299.password_SET("DEMO", PH);
            CommunicationChannel.instance.send(p299); //===============================
            PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.version = (ushort)(ushort)12624;
            p300.min_version = (ushort)(ushort)50080;
            p300.max_version = (ushort)(ushort)42271;
            p300.spec_version_hash_SET(new byte[8], 0);
            p300.library_version_hash_SET(new byte[8], 0);
            CommunicationChannel.instance.send(p300); //===============================
            UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.time_usec = (ulong)5492530866211486037L;
            p310.uptime_sec = (uint)448836824U;
            p310.health = UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING;
            p310.mode = UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE;
            p310.sub_mode = (byte)(byte)73;
            p310.vendor_specific_status_code = (ushort)(ushort)22600;
            CommunicationChannel.instance.send(p310); //===============================
            UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.time_usec = (ulong)8627977917361851402L;
            p311.uptime_sec = (uint)1238339511U;
            p311.name_SET("DEMO", PH);
            p311.hw_version_major = (byte)(byte)73;
            p311.hw_version_minor = (byte)(byte)34;
            p311.hw_unique_id_SET(new byte[16], 0);
            p311.sw_version_major = (byte)(byte)8;
            p311.sw_version_minor = (byte)(byte)254;
            p311.sw_vcs_commit = (uint)1838805491U;
            CommunicationChannel.instance.send(p311); //===============================
            PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.target_system = (byte)(byte)73;
            p320.target_component = (byte)(byte)34;
            p320.param_id_SET("DEMO", PH);
            p320.param_index = (short)(short)13945;
            CommunicationChannel.instance.send(p320); //===============================
            PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_system = (byte)(byte)14;
            p321.target_component = (byte)(byte)114;
            CommunicationChannel.instance.send(p321); //===============================
            PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_id_SET("DEMO", PH);
            p322.param_value_SET("DEMO", PH);
            p322.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16;
            p322.param_count = (ushort)(ushort)52857;
            p322.param_index = (ushort)(ushort)31954;
            CommunicationChannel.instance.send(p322); //===============================
            PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.target_system = (byte)(byte)208;
            p323.target_component = (byte)(byte)161;
            p323.param_id_SET("DEMO", PH);
            p323.param_value_SET("DEMO", PH);
            p323.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT64;
            CommunicationChannel.instance.send(p323); //===============================
            PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_id_SET("DEMO", PH);
            p324.param_value_SET("DEMO", PH);
            p324.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16;
            p324.param_result = PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED;
            CommunicationChannel.instance.send(p324); //===============================
            OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.time_usec = (ulong)1769946929470646139L;
            p330.sensor_type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR;
            p330.distances_SET(new ushort[72], 0);
            p330.increment = (byte)(byte)2;
            p330.min_distance = (ushort)(ushort)56257;
            p330.max_distance = (ushort)(ushort)60438;
            CommunicationChannel.instance.send(p330); //===============================
        }
    }
}

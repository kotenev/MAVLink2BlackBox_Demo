
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
            POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.time_boot_ms = (uint)3316940779U;
            p87.coordinate_frame = MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p87.type_mask = (ushort)(ushort)25186;
            p87.lat_int = (int)852336731;
            p87.lon_int = (int) -1300676620;
            p87.alt = (float)1.0485404E38F;
            p87.vx = (float)5.767941E37F;
            p87.vy = (float)3.3227688E38F;
            p87.vz = (float) -1.8955427E38F;
            p87.afx = (float)1.4005769E38F;
            p87.afy = (float)2.4241664E38F;
            p87.afz = (float)2.338048E38F;
            p87.yaw = (float)2.7153507E36F;
            p87.yaw_rate = (float)8.3717197E37F;
            CommunicationChannel.instance.send(p87); //===============================
            LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.time_boot_ms = (uint)2921876381U;
            p89.x = (float) -2.700881E38F;
            p89.y = (float) -1.1613329E38F;
            p89.z = (float)1.4829393E38F;
            p89.roll = (float)2.3236612E38F;
            p89.pitch = (float)2.5475723E38F;
            p89.yaw = (float) -2.8300784E38F;
            CommunicationChannel.instance.send(p89); //===============================
            HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.time_usec = (ulong)8995386034279598488L;
            p90.roll = (float) -3.1042E38F;
            p90.pitch = (float) -2.2070383E38F;
            p90.yaw = (float)3.0519063E38F;
            p90.rollspeed = (float) -3.3085283E37F;
            p90.pitchspeed = (float)1.0106691E38F;
            p90.yawspeed = (float) -2.5042111E38F;
            p90.lat = (int)1361132724;
            p90.lon = (int)1524279637;
            p90.alt = (int) -880218627;
            p90.vx = (short)(short) -10651;
            p90.vy = (short)(short) -2227;
            p90.vz = (short)(short)29458;
            p90.xacc = (short)(short)6974;
            p90.yacc = (short)(short)10073;
            p90.zacc = (short)(short)19856;
            CommunicationChannel.instance.send(p90); //===============================
            HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.time_usec = (ulong)4247620117877316494L;
            p91.roll_ailerons = (float)2.149285E38F;
            p91.pitch_elevator = (float) -1.4894653E38F;
            p91.yaw_rudder = (float)1.3736671E38F;
            p91.throttle = (float) -2.583802E38F;
            p91.aux1 = (float) -1.1856392E38F;
            p91.aux2 = (float)1.0286233E38F;
            p91.aux3 = (float) -1.1294424E38F;
            p91.aux4 = (float)1.2944489E38F;
            p91.mode = MAV_MODE.MAV_MODE_MANUAL_DISARMED;
            p91.nav_mode = (byte)(byte)104;
            CommunicationChannel.instance.send(p91); //===============================
            HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.time_usec = (ulong)8963068581915985569L;
            p92.chan1_raw = (ushort)(ushort)57206;
            p92.chan2_raw = (ushort)(ushort)3026;
            p92.chan3_raw = (ushort)(ushort)48540;
            p92.chan4_raw = (ushort)(ushort)58441;
            p92.chan5_raw = (ushort)(ushort)2233;
            p92.chan6_raw = (ushort)(ushort)35110;
            p92.chan7_raw = (ushort)(ushort)25764;
            p92.chan8_raw = (ushort)(ushort)6480;
            p92.chan9_raw = (ushort)(ushort)57580;
            p92.chan10_raw = (ushort)(ushort)43246;
            p92.chan11_raw = (ushort)(ushort)39752;
            p92.chan12_raw = (ushort)(ushort)19783;
            p92.rssi = (byte)(byte)147;
            CommunicationChannel.instance.send(p92); //===============================
            HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.time_usec = (ulong)3172652904481237085L;
            p93.controls_SET(new float[16], 0);
            p93.mode = MAV_MODE.MAV_MODE_PREFLIGHT;
            p93.flags = (ulong)4846960323949406424L;
            CommunicationChannel.instance.send(p93); //===============================
            OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.time_usec = (ulong)2815415540561492494L;
            p100.sensor_id = (byte)(byte)82;
            p100.flow_x = (short)(short)18919;
            p100.flow_y = (short)(short)10287;
            p100.flow_comp_m_x = (float) -4.7489046E37F;
            p100.flow_comp_m_y = (float)1.675492E36F;
            p100.quality = (byte)(byte)161;
            p100.ground_distance = (float)4.3785407E37F;
            p100.flow_rate_x_SET((float)4.4638293E37F, PH);
            p100.flow_rate_y_SET((float) -3.3993602E38F, PH);
            CommunicationChannel.instance.send(p100); //===============================
            GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.usec = (ulong)4277183624234920944L;
            p101.x = (float)3.2493398E38F;
            p101.y = (float)2.5988861E38F;
            p101.z = (float)1.6177681E38F;
            p101.roll = (float)5.2727886E37F;
            p101.pitch = (float) -1.0334084E38F;
            p101.yaw = (float) -3.0008847E38F;
            CommunicationChannel.instance.send(p101); //===============================
            VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.usec = (ulong)1646548886266665616L;
            p102.x = (float) -2.769231E38F;
            p102.y = (float) -1.5485715E38F;
            p102.z = (float)2.5390741E38F;
            p102.roll = (float)4.8762127E37F;
            p102.pitch = (float)2.009108E38F;
            p102.yaw = (float) -1.2536688E38F;
            CommunicationChannel.instance.send(p102); //===============================
            VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.usec = (ulong)4399557195349060318L;
            p103.x = (float) -2.9424235E38F;
            p103.y = (float)2.7034712E38F;
            p103.z = (float) -2.7374075E38F;
            CommunicationChannel.instance.send(p103); //===============================
            VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.usec = (ulong)6983662500488830445L;
            p104.x = (float)1.5333424E38F;
            p104.y = (float) -4.387036E37F;
            p104.z = (float) -1.6679751E38F;
            p104.roll = (float)2.1006315E38F;
            p104.pitch = (float) -6.147524E37F;
            p104.yaw = (float)2.7955888E38F;
            CommunicationChannel.instance.send(p104); //===============================
            HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.time_usec = (ulong)1288343301412961824L;
            p105.xacc = (float) -1.2940204E38F;
            p105.yacc = (float)2.1526021E38F;
            p105.zacc = (float) -2.2879126E37F;
            p105.xgyro = (float)9.1437393E36F;
            p105.ygyro = (float) -2.4815812E38F;
            p105.zgyro = (float) -2.343707E38F;
            p105.xmag = (float)3.0994295E37F;
            p105.ymag = (float)2.037618E38F;
            p105.zmag = (float)7.8148175E37F;
            p105.abs_pressure = (float)3.847709E37F;
            p105.diff_pressure = (float) -8.735194E37F;
            p105.pressure_alt = (float) -5.2564334E37F;
            p105.temperature = (float)1.6968181E38F;
            p105.fields_updated = (ushort)(ushort)7216;
            CommunicationChannel.instance.send(p105); //===============================
            OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.time_usec = (ulong)7718044394609910535L;
            p106.sensor_id = (byte)(byte)106;
            p106.integration_time_us = (uint)1284584704U;
            p106.integrated_x = (float)2.1865867E38F;
            p106.integrated_y = (float)5.341021E37F;
            p106.integrated_xgyro = (float) -4.1143753E37F;
            p106.integrated_ygyro = (float) -1.6343494E38F;
            p106.integrated_zgyro = (float)2.4977071E38F;
            p106.temperature = (short)(short)14214;
            p106.quality = (byte)(byte)173;
            p106.time_delta_distance_us = (uint)4194934962U;
            p106.distance = (float)2.7949479E38F;
            CommunicationChannel.instance.send(p106); //===============================
            HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.time_usec = (ulong)3550604138650587245L;
            p107.xacc = (float) -1.0813348E38F;
            p107.yacc = (float)2.3402968E38F;
            p107.zacc = (float)2.550546E38F;
            p107.xgyro = (float) -2.7796387E38F;
            p107.ygyro = (float)1.2144931E38F;
            p107.zgyro = (float)2.3924283E38F;
            p107.xmag = (float)1.2055024E38F;
            p107.ymag = (float) -2.6757153E38F;
            p107.zmag = (float) -1.9543392E38F;
            p107.abs_pressure = (float) -4.4287615E37F;
            p107.diff_pressure = (float)2.583932E38F;
            p107.pressure_alt = (float) -3.3013757E38F;
            p107.temperature = (float) -1.8824382E38F;
            p107.fields_updated = (uint)2071247746U;
            CommunicationChannel.instance.send(p107); //===============================
            SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.q1 = (float) -6.9608135E37F;
            p108.q2 = (float) -1.8372829E38F;
            p108.q3 = (float)9.977017E36F;
            p108.q4 = (float) -1.761139E38F;
            p108.roll = (float) -1.6582423E38F;
            p108.pitch = (float) -9.102593E37F;
            p108.yaw = (float) -3.2772109E38F;
            p108.xacc = (float)1.880688E38F;
            p108.yacc = (float) -1.170617E38F;
            p108.zacc = (float) -1.2807522E38F;
            p108.xgyro = (float) -5.8707566E37F;
            p108.ygyro = (float)1.8417393E38F;
            p108.zgyro = (float) -9.322079E37F;
            p108.lat = (float) -3.2167443E38F;
            p108.lon = (float)6.2287234E37F;
            p108.alt = (float) -2.7732676E38F;
            p108.std_dev_horz = (float)1.5566589E38F;
            p108.std_dev_vert = (float)7.870076E37F;
            p108.vn = (float)2.6519792E38F;
            p108.ve = (float) -2.9826198E38F;
            p108.vd = (float)1.6060831E38F;
            CommunicationChannel.instance.send(p108); //===============================
            RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.rssi = (byte)(byte)201;
            p109.remrssi = (byte)(byte)187;
            p109.txbuf = (byte)(byte)43;
            p109.noise = (byte)(byte)93;
            p109.remnoise = (byte)(byte)114;
            p109.rxerrors = (ushort)(ushort)42825;
            p109.fixed_ = (ushort)(ushort)39647;
            CommunicationChannel.instance.send(p109); //===============================
            FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_network = (byte)(byte)64;
            p110.target_system = (byte)(byte)29;
            p110.target_component = (byte)(byte)189;
            p110.payload_SET(new byte[251], 0);
            CommunicationChannel.instance.send(p110); //===============================
            TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.tc1 = (long)8433566553677630939L;
            p111.ts1 = (long) -423103387970949618L;
            CommunicationChannel.instance.send(p111); //===============================
            CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.time_usec = (ulong)9031326443678109543L;
            p112.seq = (uint)4143378194U;
            CommunicationChannel.instance.send(p112); //===============================
            HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.time_usec = (ulong)3714768368614354488L;
            p113.fix_type = (byte)(byte)198;
            p113.lat = (int) -1929135000;
            p113.lon = (int) -1768708917;
            p113.alt = (int)1131140366;
            p113.eph = (ushort)(ushort)60882;
            p113.epv = (ushort)(ushort)55707;
            p113.vel = (ushort)(ushort)22850;
            p113.vn = (short)(short) -1427;
            p113.ve = (short)(short) -12637;
            p113.vd = (short)(short)16582;
            p113.cog = (ushort)(ushort)15246;
            p113.satellites_visible = (byte)(byte)73;
            CommunicationChannel.instance.send(p113); //===============================
            HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.time_usec = (ulong)6447150661575124315L;
            p114.sensor_id = (byte)(byte)25;
            p114.integration_time_us = (uint)1712077038U;
            p114.integrated_x = (float) -2.0613646E38F;
            p114.integrated_y = (float) -6.6077915E37F;
            p114.integrated_xgyro = (float)1.6007055E38F;
            p114.integrated_ygyro = (float) -2.2009733E38F;
            p114.integrated_zgyro = (float) -9.939352E37F;
            p114.temperature = (short)(short)5038;
            p114.quality = (byte)(byte)119;
            p114.time_delta_distance_us = (uint)306655731U;
            p114.distance = (float) -3.3086705E38F;
            CommunicationChannel.instance.send(p114); //===============================
            HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.time_usec = (ulong)6309171328369124573L;
            p115.attitude_quaternion_SET(new float[4], 0);
            p115.rollspeed = (float)2.2505498E38F;
            p115.pitchspeed = (float)3.3521483E38F;
            p115.yawspeed = (float)1.2982648E38F;
            p115.lat = (int) -2100794443;
            p115.lon = (int) -981106067;
            p115.alt = (int)1475585758;
            p115.vx = (short)(short) -7484;
            p115.vy = (short)(short)3703;
            p115.vz = (short)(short) -17627;
            p115.ind_airspeed = (ushort)(ushort)51402;
            p115.true_airspeed = (ushort)(ushort)44240;
            p115.xacc = (short)(short) -24636;
            p115.yacc = (short)(short)12140;
            p115.zacc = (short)(short) -17644;
            CommunicationChannel.instance.send(p115); //===============================
            SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.time_boot_ms = (uint)3093050754U;
            p116.xacc = (short)(short)20008;
            p116.yacc = (short)(short)18426;
            p116.zacc = (short)(short) -22463;
            p116.xgyro = (short)(short) -15498;
            p116.ygyro = (short)(short) -16813;
            p116.zgyro = (short)(short)32662;
            p116.xmag = (short)(short)26432;
            p116.ymag = (short)(short)2980;
            p116.zmag = (short)(short) -10877;
            CommunicationChannel.instance.send(p116); //===============================
            LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.target_system = (byte)(byte)26;
            p117.target_component = (byte)(byte)52;
            p117.start = (ushort)(ushort)22343;
            p117.end = (ushort)(ushort)9599;
            CommunicationChannel.instance.send(p117); //===============================
            LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.id = (ushort)(ushort)51243;
            p118.num_logs = (ushort)(ushort)34647;
            p118.last_log_num = (ushort)(ushort)15933;
            p118.time_utc = (uint)2109844977U;
            p118.size = (uint)3041862134U;
            CommunicationChannel.instance.send(p118); //===============================
            LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.target_system = (byte)(byte)77;
            p119.target_component = (byte)(byte)179;
            p119.id = (ushort)(ushort)28368;
            p119.ofs = (uint)2155745288U;
            p119.count = (uint)3849702424U;
            CommunicationChannel.instance.send(p119); //===============================
            LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.id = (ushort)(ushort)32505;
            p120.ofs = (uint)2657327506U;
            p120.count = (byte)(byte)211;
            p120.data__SET(new byte[90], 0);
            CommunicationChannel.instance.send(p120); //===============================
            LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_system = (byte)(byte)169;
            p121.target_component = (byte)(byte)111;
            CommunicationChannel.instance.send(p121); //===============================
            LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_system = (byte)(byte)211;
            p122.target_component = (byte)(byte)65;
            CommunicationChannel.instance.send(p122); //===============================
            GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.target_system = (byte)(byte)32;
            p123.target_component = (byte)(byte)194;
            p123.len = (byte)(byte)38;
            p123.data__SET(new byte[110], 0);
            CommunicationChannel.instance.send(p123); //===============================
            GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.time_usec = (ulong)872541684713224109L;
            p124.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX;
            p124.lat = (int)1969166375;
            p124.lon = (int)1050413560;
            p124.alt = (int)1634368882;
            p124.eph = (ushort)(ushort)25212;
            p124.epv = (ushort)(ushort)60373;
            p124.vel = (ushort)(ushort)20282;
            p124.cog = (ushort)(ushort)57501;
            p124.satellites_visible = (byte)(byte)116;
            p124.dgps_numch = (byte)(byte)86;
            p124.dgps_age = (uint)1796854543U;
            CommunicationChannel.instance.send(p124); //===============================
            POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vcc = (ushort)(ushort)38683;
            p125.Vservo = (ushort)(ushort)12844;
            p125.flags = (MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID |
                          MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID |
                          MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT);
            CommunicationChannel.instance.send(p125); //===============================
            SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.device = SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS2;
            p126.flags = (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING |
                          SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY |
                          SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI);
            p126.timeout = (ushort)(ushort)55152;
            p126.baudrate = (uint)3748394223U;
            p126.count = (byte)(byte)36;
            p126.data__SET(new byte[70], 0);
            CommunicationChannel.instance.send(p126); //===============================
            GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.time_last_baseline_ms = (uint)972035430U;
            p127.rtk_receiver_id = (byte)(byte)206;
            p127.wn = (ushort)(ushort)18897;
            p127.tow = (uint)3574314091U;
            p127.rtk_health = (byte)(byte)187;
            p127.rtk_rate = (byte)(byte)146;
            p127.nsats = (byte)(byte)200;
            p127.baseline_coords_type = (byte)(byte)180;
            p127.baseline_a_mm = (int) -1861271657;
            p127.baseline_b_mm = (int)207511468;
            p127.baseline_c_mm = (int)699886304;
            p127.accuracy = (uint)2189469989U;
            p127.iar_num_hypotheses = (int)2032991584;
            CommunicationChannel.instance.send(p127); //===============================
            GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.time_last_baseline_ms = (uint)2435188649U;
            p128.rtk_receiver_id = (byte)(byte)107;
            p128.wn = (ushort)(ushort)61951;
            p128.tow = (uint)2043243427U;
            p128.rtk_health = (byte)(byte)49;
            p128.rtk_rate = (byte)(byte)149;
            p128.nsats = (byte)(byte)93;
            p128.baseline_coords_type = (byte)(byte)175;
            p128.baseline_a_mm = (int) -1271563514;
            p128.baseline_b_mm = (int) -61231239;
            p128.baseline_c_mm = (int)2015768904;
            p128.accuracy = (uint)1821067147U;
            p128.iar_num_hypotheses = (int)266883772;
            CommunicationChannel.instance.send(p128); //===============================
            SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.time_boot_ms = (uint)3255994247U;
            p129.xacc = (short)(short) -23141;
            p129.yacc = (short)(short) -7914;
            p129.zacc = (short)(short) -32310;
            p129.xgyro = (short)(short)4811;
            p129.ygyro = (short)(short)22687;
            p129.zgyro = (short)(short)15576;
            p129.xmag = (short)(short)10089;
            p129.ymag = (short)(short) -2464;
            p129.zmag = (short)(short) -9233;
            CommunicationChannel.instance.send(p129); //===============================
            DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.type = (byte)(byte)114;
            p130.size = (uint)3615465270U;
            p130.width = (ushort)(ushort)13126;
            p130.height = (ushort)(ushort)17424;
            p130.packets = (ushort)(ushort)58034;
            p130.payload = (byte)(byte)75;
            p130.jpg_quality = (byte)(byte)243;
            CommunicationChannel.instance.send(p130); //===============================
            ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.seqnr = (ushort)(ushort)37532;
            p131.data__SET(new byte[253], 0);
            CommunicationChannel.instance.send(p131); //===============================
            DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.time_boot_ms = (uint)2553159970U;
            p132.min_distance = (ushort)(ushort)61167;
            p132.max_distance = (ushort)(ushort)25396;
            p132.current_distance = (ushort)(ushort)39535;
            p132.type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED;
            p132.id = (byte)(byte)75;
            p132.orientation = MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_PITCH_270;
            p132.covariance = (byte)(byte)248;
            CommunicationChannel.instance.send(p132); //===============================
            TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.lat = (int)261115285;
            p133.lon = (int)488682604;
            p133.grid_spacing = (ushort)(ushort)16871;
            p133.mask = (ulong)8713697211386939835L;
            CommunicationChannel.instance.send(p133); //===============================
            TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.lat = (int) -1627840770;
            p134.lon = (int)1000275267;
            p134.grid_spacing = (ushort)(ushort)18670;
            p134.gridbit = (byte)(byte)147;
            p134.data__SET(new short[16], 0);
            CommunicationChannel.instance.send(p134); //===============================
            TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lat = (int) -1343329902;
            p135.lon = (int)1623004672;
            CommunicationChannel.instance.send(p135); //===============================
            TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.lat = (int) -215446714;
            p136.lon = (int) -336002809;
            p136.spacing = (ushort)(ushort)1401;
            p136.terrain_height = (float)2.7375142E38F;
            p136.current_height = (float)4.0540293E37F;
            p136.pending = (ushort)(ushort)36329;
            p136.loaded = (ushort)(ushort)15318;
            CommunicationChannel.instance.send(p136); //===============================
            SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.time_boot_ms = (uint)815539368U;
            p137.press_abs = (float) -3.6733295E37F;
            p137.press_diff = (float)5.332713E37F;
            p137.temperature = (short)(short)3750;
            CommunicationChannel.instance.send(p137); //===============================
            ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.time_usec = (ulong)2715203683219930037L;
            p138.q_SET(new float[4], 0);
            p138.x = (float) -3.9672239E37F;
            p138.y = (float) -2.171781E38F;
            p138.z = (float) -2.0067515E38F;
            CommunicationChannel.instance.send(p138); //===============================
            SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.time_usec = (ulong)6392716195670022815L;
            p139.group_mlx = (byte)(byte)113;
            p139.target_system = (byte)(byte)232;
            p139.target_component = (byte)(byte)18;
            p139.controls_SET(new float[8], 0);
            CommunicationChannel.instance.send(p139); //===============================
            ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.time_usec = (ulong)9192114434316276227L;
            p140.group_mlx = (byte)(byte)163;
            p140.controls_SET(new float[8], 0);
            CommunicationChannel.instance.send(p140); //===============================
            ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.time_usec = (ulong)3591588432159096650L;
            p141.altitude_monotonic = (float) -1.6593951E38F;
            p141.altitude_amsl = (float)7.689618E36F;
            p141.altitude_local = (float) -7.155662E37F;
            p141.altitude_relative = (float) -4.3072726E36F;
            p141.altitude_terrain = (float) -7.266924E37F;
            p141.bottom_clearance = (float)1.3769593E38F;
            CommunicationChannel.instance.send(p141); //===============================
            RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.request_id = (byte)(byte)94;
            p142.uri_type = (byte)(byte)8;
            p142.uri_SET(new byte[120], 0);
            p142.transfer_type = (byte)(byte)176;
            p142.storage_SET(new byte[120], 0);
            CommunicationChannel.instance.send(p142); //===============================
            SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.time_boot_ms = (uint)535871513U;
            p143.press_abs = (float) -2.355499E38F;
            p143.press_diff = (float) -1.5429325E38F;
            p143.temperature = (short)(short) -10591;
            CommunicationChannel.instance.send(p143); //===============================
            FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.timestamp = (ulong)7992498656538524667L;
            p144.est_capabilities = (byte)(byte)165;
            p144.lat = (int) -1427726051;
            p144.lon = (int)1469016693;
            p144.alt = (float)2.3746769E38F;
            p144.vel_SET(new float[3], 0);
            p144.acc_SET(new float[3], 0);
            p144.attitude_q_SET(new float[4], 0);
            p144.rates_SET(new float[3], 0);
            p144.position_cov_SET(new float[3], 0);
            p144.custom_state = (ulong)6149750125188921038L;
            CommunicationChannel.instance.send(p144); //===============================
            CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.time_usec = (ulong)7193545956453902898L;
            p146.x_acc = (float) -1.295912E38F;
            p146.y_acc = (float) -5.269259E37F;
            p146.z_acc = (float)2.2480356E38F;
            p146.x_vel = (float) -3.0722668E38F;
            p146.y_vel = (float)7.4130894E37F;
            p146.z_vel = (float)1.4203314E37F;
            p146.x_pos = (float)1.7561409E38F;
            p146.y_pos = (float) -1.7401934E38F;
            p146.z_pos = (float)1.8386834E38F;
            p146.airspeed = (float) -1.3326568E38F;
            p146.vel_variance_SET(new float[3], 0);
            p146.pos_variance_SET(new float[3], 0);
            p146.q_SET(new float[4], 0);
            p146.roll_rate = (float) -5.2544264E37F;
            p146.pitch_rate = (float)1.6836551E37F;
            p146.yaw_rate = (float) -1.7334558E38F;
            CommunicationChannel.instance.send(p146); //===============================
            BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.id = (byte)(byte)50;
            p147.battery_function = MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_PROPULSION;
            p147.type = MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIPO;
            p147.temperature = (short)(short)4520;
            p147.voltages_SET(new ushort[10], 0);
            p147.current_battery = (short)(short)21994;
            p147.current_consumed = (int)2005298731;
            p147.energy_consumed = (int) -904250460;
            p147.battery_remaining = (sbyte)(sbyte)37;
            CommunicationChannel.instance.send(p147); //===============================
            AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_TERRAIN |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMMAND_INT);
            p148.flight_sw_version = (uint)2841688410U;
            p148.middleware_sw_version = (uint)1004199295U;
            p148.os_sw_version = (uint)2502110084U;
            p148.board_version = (uint)3449557531U;
            p148.flight_custom_version_SET(new byte[8], 0);
            p148.middleware_custom_version_SET(new byte[8], 0);
            p148.os_custom_version_SET(new byte[8], 0);
            p148.vendor_id = (ushort)(ushort)43427;
            p148.product_id = (ushort)(ushort)14532;
            p148.uid = (ulong)6376516978195483504L;
            p148.uid2_SET(new byte[18], 0, PH);
            CommunicationChannel.instance.send(p148); //===============================
            LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.time_usec = (ulong)71850180119599328L;
            p149.target_num = (byte)(byte)244;
            p149.frame = MAV_FRAME.MAV_FRAME_BODY_NED;
            p149.angle_x = (float) -3.46685E37F;
            p149.angle_y = (float) -8.1752166E37F;
            p149.distance = (float)4.513735E37F;
            p149.size_x = (float)3.020466E38F;
            p149.size_y = (float)1.8749445E38F;
            p149.x_SET((float) -1.4888993E38F, PH);
            p149.y_SET((float)2.455075E38F, PH);
            p149.z_SET((float)1.0233234E38F, PH);
            p149.q_SET(new float[4], 0, PH);
            p149.type = LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_RADIO_BEACON;
            p149.position_valid_SET((byte)(byte)239, PH);
            CommunicationChannel.instance.send(p149); //===============================
            FLEXIFUNCTION_SET p150 = CommunicationChannel.new_FLEXIFUNCTION_SET();
            PH.setPack(p150);
            p150.target_system = (byte)(byte)221;
            p150.target_component = (byte)(byte)153;
            CommunicationChannel.instance.send(p150); //===============================
            FLEXIFUNCTION_READ_REQ p151 = CommunicationChannel.new_FLEXIFUNCTION_READ_REQ();
            PH.setPack(p151);
            p151.target_system = (byte)(byte)191;
            p151.target_component = (byte)(byte)86;
            p151.read_req_type = (short)(short) -14016;
            p151.data_index = (short)(short)25223;
            CommunicationChannel.instance.send(p151); //===============================
            FLEXIFUNCTION_BUFFER_FUNCTION p152 = CommunicationChannel.new_FLEXIFUNCTION_BUFFER_FUNCTION();
            PH.setPack(p152);
            p152.target_system = (byte)(byte)150;
            p152.target_component = (byte)(byte)181;
            p152.func_index = (ushort)(ushort)56562;
            p152.func_count = (ushort)(ushort)20868;
            p152.data_address = (ushort)(ushort)49269;
            p152.data_size = (ushort)(ushort)19836;
            p152.data__SET(new sbyte[48], 0);
            CommunicationChannel.instance.send(p152); //===============================
            FLEXIFUNCTION_BUFFER_FUNCTION_ACK p153 = CommunicationChannel.new_FLEXIFUNCTION_BUFFER_FUNCTION_ACK();
            PH.setPack(p153);
            p153.target_system = (byte)(byte)33;
            p153.target_component = (byte)(byte)29;
            p153.func_index = (ushort)(ushort)44777;
            p153.result = (ushort)(ushort)8235;
            CommunicationChannel.instance.send(p153); //===============================
            FLEXIFUNCTION_DIRECTORY p155 = CommunicationChannel.new_FLEXIFUNCTION_DIRECTORY();
            PH.setPack(p155);
            p155.target_system = (byte)(byte)207;
            p155.target_component = (byte)(byte)165;
            p155.directory_type = (byte)(byte)176;
            p155.start_index = (byte)(byte)251;
            p155.count = (byte)(byte)65;
            p155.directory_data_SET(new sbyte[48], 0);
            CommunicationChannel.instance.send(p155); //===============================
            FLEXIFUNCTION_DIRECTORY_ACK p156 = CommunicationChannel.new_FLEXIFUNCTION_DIRECTORY_ACK();
            PH.setPack(p156);
            p156.target_system = (byte)(byte)43;
            p156.target_component = (byte)(byte)48;
            p156.directory_type = (byte)(byte)94;
            p156.start_index = (byte)(byte)105;
            p156.count = (byte)(byte)118;
            p156.result = (ushort)(ushort)12278;
            CommunicationChannel.instance.send(p156); //===============================
            FLEXIFUNCTION_COMMAND p157 = CommunicationChannel.new_FLEXIFUNCTION_COMMAND();
            PH.setPack(p157);
            p157.target_system = (byte)(byte)203;
            p157.target_component = (byte)(byte)146;
            p157.command_type = (byte)(byte)191;
            CommunicationChannel.instance.send(p157); //===============================
            FLEXIFUNCTION_COMMAND_ACK p158 = CommunicationChannel.new_FLEXIFUNCTION_COMMAND_ACK();
            PH.setPack(p158);
            p158.command_type = (ushort)(ushort)21211;
            p158.result = (ushort)(ushort)55627;
            CommunicationChannel.instance.send(p158); //===============================
            SERIAL_UDB_EXTRA_F2_A p170 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F2_A();
            PH.setPack(p170);
            p170.sue_time = (uint)2397587227U;
            p170.sue_status = (byte)(byte)207;
            p170.sue_latitude = (int)1452363035;
            p170.sue_longitude = (int) -1574796970;
            p170.sue_altitude = (int)1611188786;
            p170.sue_waypoint_index = (ushort)(ushort)47366;
            p170.sue_rmat0 = (short)(short)27689;
            p170.sue_rmat1 = (short)(short) -18675;
            p170.sue_rmat2 = (short)(short) -38;
            p170.sue_rmat3 = (short)(short)20354;
            p170.sue_rmat4 = (short)(short) -23568;
            p170.sue_rmat5 = (short)(short)18056;
            p170.sue_rmat6 = (short)(short)16742;
            p170.sue_rmat7 = (short)(short)16119;
            p170.sue_rmat8 = (short)(short)31525;
            p170.sue_cog = (ushort)(ushort)43755;
            p170.sue_sog = (short)(short) -1574;
            p170.sue_cpu_load = (ushort)(ushort)29622;
            p170.sue_air_speed_3DIMU = (ushort)(ushort)48174;
            p170.sue_estimated_wind_0 = (short)(short) -2625;
            p170.sue_estimated_wind_1 = (short)(short) -23214;
            p170.sue_estimated_wind_2 = (short)(short) -18063;
            p170.sue_magFieldEarth0 = (short)(short) -22822;
            p170.sue_magFieldEarth1 = (short)(short) -23441;
            p170.sue_magFieldEarth2 = (short)(short) -14264;
            p170.sue_svs = (short)(short)15448;
            p170.sue_hdop = (short)(short)22467;
            CommunicationChannel.instance.send(p170); //===============================
            SERIAL_UDB_EXTRA_F2_B p171 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F2_B();
            PH.setPack(p171);
            p171.sue_time = (uint)2480984459U;
            p171.sue_pwm_input_1 = (short)(short) -16433;
            p171.sue_pwm_input_2 = (short)(short) -26326;
            p171.sue_pwm_input_3 = (short)(short) -3201;
            p171.sue_pwm_input_4 = (short)(short)9565;
            p171.sue_pwm_input_5 = (short)(short) -14376;
            p171.sue_pwm_input_6 = (short)(short) -13421;
            p171.sue_pwm_input_7 = (short)(short)592;
            p171.sue_pwm_input_8 = (short)(short)7893;
            p171.sue_pwm_input_9 = (short)(short) -28425;
            p171.sue_pwm_input_10 = (short)(short)29499;
            p171.sue_pwm_input_11 = (short)(short) -9755;
            p171.sue_pwm_input_12 = (short)(short) -23009;
            p171.sue_pwm_output_1 = (short)(short)1255;
            p171.sue_pwm_output_2 = (short)(short)12441;
            p171.sue_pwm_output_3 = (short)(short)23629;
            p171.sue_pwm_output_4 = (short)(short) -20467;
            p171.sue_pwm_output_5 = (short)(short) -21521;
            p171.sue_pwm_output_6 = (short)(short) -8839;
            p171.sue_pwm_output_7 = (short)(short)3177;
            p171.sue_pwm_output_8 = (short)(short) -22986;
            p171.sue_pwm_output_9 = (short)(short)23983;
            p171.sue_pwm_output_10 = (short)(short)30884;
            p171.sue_pwm_output_11 = (short)(short)29659;
            p171.sue_pwm_output_12 = (short)(short) -29615;
            p171.sue_imu_location_x = (short)(short) -15610;
            p171.sue_imu_location_y = (short)(short)9487;
            p171.sue_imu_location_z = (short)(short) -32105;
            p171.sue_location_error_earth_x = (short)(short)21718;
            p171.sue_location_error_earth_y = (short)(short) -5312;
            p171.sue_location_error_earth_z = (short)(short)27269;
            p171.sue_flags = (uint)1339896127U;
            p171.sue_osc_fails = (short)(short) -10913;
            p171.sue_imu_velocity_x = (short)(short) -17678;
            p171.sue_imu_velocity_y = (short)(short) -13598;
            p171.sue_imu_velocity_z = (short)(short) -18431;
            p171.sue_waypoint_goal_x = (short)(short) -16108;
            p171.sue_waypoint_goal_y = (short)(short)4115;
            p171.sue_waypoint_goal_z = (short)(short)24979;
            p171.sue_aero_x = (short)(short) -25327;
            p171.sue_aero_y = (short)(short)32406;
            p171.sue_aero_z = (short)(short)13273;
            p171.sue_barom_temp = (short)(short)28916;
            p171.sue_barom_press = (int)503671849;
            p171.sue_barom_alt = (int)1255240000;
            p171.sue_bat_volt = (short)(short) -12252;
            p171.sue_bat_amp = (short)(short)740;
            p171.sue_bat_amp_hours = (short)(short)26377;
            p171.sue_desired_height = (short)(short) -10656;
            p171.sue_memory_stack_free = (short)(short)9305;
            CommunicationChannel.instance.send(p171); //===============================
            SERIAL_UDB_EXTRA_F4 p172 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F4();
            PH.setPack(p172);
            p172.sue_ROLL_STABILIZATION_AILERONS = (byte)(byte)213;
            p172.sue_ROLL_STABILIZATION_RUDDER = (byte)(byte)98;
            p172.sue_PITCH_STABILIZATION = (byte)(byte)145;
            p172.sue_YAW_STABILIZATION_RUDDER = (byte)(byte)104;
            p172.sue_YAW_STABILIZATION_AILERON = (byte)(byte)95;
            p172.sue_AILERON_NAVIGATION = (byte)(byte)13;
            p172.sue_RUDDER_NAVIGATION = (byte)(byte)137;
            p172.sue_ALTITUDEHOLD_STABILIZED = (byte)(byte)33;
            p172.sue_ALTITUDEHOLD_WAYPOINT = (byte)(byte)99;
            p172.sue_RACING_MODE = (byte)(byte)42;
            CommunicationChannel.instance.send(p172); //===============================
            SERIAL_UDB_EXTRA_F5 p173 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F5();
            PH.setPack(p173);
            p173.sue_YAWKP_AILERON = (float) -1.4493282E38F;
            p173.sue_YAWKD_AILERON = (float)1.6840908E37F;
            p173.sue_ROLLKP = (float)2.4964884E38F;
            p173.sue_ROLLKD = (float)1.663697E38F;
            CommunicationChannel.instance.send(p173); //===============================
            SERIAL_UDB_EXTRA_F6 p174 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F6();
            PH.setPack(p174);
            p174.sue_PITCHGAIN = (float) -2.7233331E38F;
            p174.sue_PITCHKD = (float)2.4779225E38F;
            p174.sue_RUDDER_ELEV_MIX = (float) -1.1114408E37F;
            p174.sue_ROLL_ELEV_MIX = (float)1.6954496E38F;
            p174.sue_ELEVATOR_BOOST = (float)3.0325985E38F;
            CommunicationChannel.instance.send(p174); //===============================
            SERIAL_UDB_EXTRA_F7 p175 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F7();
            PH.setPack(p175);
            p175.sue_YAWKP_RUDDER = (float) -2.873896E36F;
            p175.sue_YAWKD_RUDDER = (float)4.328094E37F;
            p175.sue_ROLLKP_RUDDER = (float) -6.266713E37F;
            p175.sue_ROLLKD_RUDDER = (float)7.73473E37F;
            p175.sue_RUDDER_BOOST = (float) -1.8201341E38F;
            p175.sue_RTL_PITCH_DOWN = (float) -1.7251033E38F;
            CommunicationChannel.instance.send(p175); //===============================
            SERIAL_UDB_EXTRA_F8 p176 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F8();
            PH.setPack(p176);
            p176.sue_HEIGHT_TARGET_MAX = (float) -2.3443892E38F;
            p176.sue_HEIGHT_TARGET_MIN = (float)2.0350806E38F;
            p176.sue_ALT_HOLD_THROTTLE_MIN = (float)2.862177E38F;
            p176.sue_ALT_HOLD_THROTTLE_MAX = (float) -2.9771819E38F;
            p176.sue_ALT_HOLD_PITCH_MIN = (float)2.7107885E38F;
            p176.sue_ALT_HOLD_PITCH_MAX = (float) -1.1091593E38F;
            p176.sue_ALT_HOLD_PITCH_HIGH = (float)9.786718E37F;
            CommunicationChannel.instance.send(p176); //===============================
            SERIAL_UDB_EXTRA_F13 p177 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F13();
            PH.setPack(p177);
            p177.sue_week_no = (short)(short) -13824;
            p177.sue_lat_origin = (int)1045729878;
            p177.sue_lon_origin = (int)1515489381;
            p177.sue_alt_origin = (int)1838527382;
            CommunicationChannel.instance.send(p177); //===============================
            SERIAL_UDB_EXTRA_F14 p178 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F14();
            PH.setPack(p178);
            p178.sue_WIND_ESTIMATION = (byte)(byte)177;
            p178.sue_GPS_TYPE = (byte)(byte)64;
            p178.sue_DR = (byte)(byte)185;
            p178.sue_BOARD_TYPE = (byte)(byte)76;
            p178.sue_AIRFRAME = (byte)(byte)166;
            p178.sue_RCON = (short)(short) -2391;
            p178.sue_TRAP_FLAGS = (short)(short) -28492;
            p178.sue_TRAP_SOURCE = (uint)307134750U;
            p178.sue_osc_fail_count = (short)(short) -22851;
            p178.sue_CLOCK_CONFIG = (byte)(byte)175;
            p178.sue_FLIGHT_PLAN_TYPE = (byte)(byte)4;
            CommunicationChannel.instance.send(p178); //===============================
            SERIAL_UDB_EXTRA_F15 p179 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F15();
            PH.setPack(p179);
            p179.sue_ID_VEHICLE_MODEL_NAME_SET(new byte[40], 0);
            p179.sue_ID_VEHICLE_REGISTRATION_SET(new byte[20], 0);
            CommunicationChannel.instance.send(p179); //===============================
            SERIAL_UDB_EXTRA_F16 p180 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F16();
            PH.setPack(p180);
            p180.sue_ID_LEAD_PILOT_SET(new byte[40], 0);
            p180.sue_ID_DIY_DRONES_URL_SET(new byte[70], 0);
            CommunicationChannel.instance.send(p180); //===============================
            ALTITUDES p181 = CommunicationChannel.new_ALTITUDES();
            PH.setPack(p181);
            p181.time_boot_ms = (uint)1388002043U;
            p181.alt_gps = (int)1382000288;
            p181.alt_imu = (int)1977792899;
            p181.alt_barometric = (int)1878580375;
            p181.alt_optical_flow = (int) -1256369102;
            p181.alt_range_finder = (int) -1659804688;
            p181.alt_extra = (int) -1597577746;
            CommunicationChannel.instance.send(p181); //===============================
            AIRSPEEDS p182 = CommunicationChannel.new_AIRSPEEDS();
            PH.setPack(p182);
            p182.time_boot_ms = (uint)1660189630U;
            p182.airspeed_imu = (short)(short) -4553;
            p182.airspeed_pitot = (short)(short)6658;
            p182.airspeed_hot_wire = (short)(short) -29924;
            p182.airspeed_ultrasonic = (short)(short)5365;
            p182.aoa = (short)(short)1309;
            p182.aoy = (short)(short)6761;
            CommunicationChannel.instance.send(p182); //===============================
            SERIAL_UDB_EXTRA_F17 p183 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F17();
            PH.setPack(p183);
            p183.sue_feed_forward = (float) -3.0851405E38F;
            p183.sue_turn_rate_nav = (float) -2.9936512E37F;
            p183.sue_turn_rate_fbw = (float)1.061852E38F;
            CommunicationChannel.instance.send(p183); //===============================
            SERIAL_UDB_EXTRA_F18 p184 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F18();
            PH.setPack(p184);
            p184.angle_of_attack_normal = (float) -3.8583476E37F;
            p184.angle_of_attack_inverted = (float) -1.649686E38F;
            p184.elevator_trim_normal = (float)1.6387679E38F;
            p184.elevator_trim_inverted = (float) -4.6127133E37F;
            p184.reference_speed = (float)2.3470506E38F;
            CommunicationChannel.instance.send(p184); //===============================
            SERIAL_UDB_EXTRA_F19 p185 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F19();
            PH.setPack(p185);
            p185.sue_aileron_output_channel = (byte)(byte)36;
            p185.sue_aileron_reversed = (byte)(byte)213;
            p185.sue_elevator_output_channel = (byte)(byte)48;
            p185.sue_elevator_reversed = (byte)(byte)246;
            p185.sue_throttle_output_channel = (byte)(byte)106;
            p185.sue_throttle_reversed = (byte)(byte)250;
            p185.sue_rudder_output_channel = (byte)(byte)21;
            p185.sue_rudder_reversed = (byte)(byte)102;
            CommunicationChannel.instance.send(p185); //===============================
            SERIAL_UDB_EXTRA_F20 p186 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F20();
            PH.setPack(p186);
            p186.sue_number_of_inputs = (byte)(byte)157;
            p186.sue_trim_value_input_1 = (short)(short) -7803;
            p186.sue_trim_value_input_2 = (short)(short)19410;
            p186.sue_trim_value_input_3 = (short)(short)2220;
            p186.sue_trim_value_input_4 = (short)(short)5485;
            p186.sue_trim_value_input_5 = (short)(short)20917;
            p186.sue_trim_value_input_6 = (short)(short) -22257;
            p186.sue_trim_value_input_7 = (short)(short)6521;
            p186.sue_trim_value_input_8 = (short)(short) -5172;
            p186.sue_trim_value_input_9 = (short)(short)18867;
            p186.sue_trim_value_input_10 = (short)(short) -30326;
            p186.sue_trim_value_input_11 = (short)(short)12603;
            p186.sue_trim_value_input_12 = (short)(short) -5634;
            CommunicationChannel.instance.send(p186); //===============================
            SERIAL_UDB_EXTRA_F21 p187 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F21();
            PH.setPack(p187);
            p187.sue_accel_x_offset = (short)(short) -16825;
            p187.sue_accel_y_offset = (short)(short) -3801;
            p187.sue_accel_z_offset = (short)(short)14810;
            p187.sue_gyro_x_offset = (short)(short)13808;
            p187.sue_gyro_y_offset = (short)(short) -22099;
            p187.sue_gyro_z_offset = (short)(short)12450;
            CommunicationChannel.instance.send(p187); //===============================
            SERIAL_UDB_EXTRA_F22 p188 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F22();
            PH.setPack(p188);
            p188.sue_accel_x_at_calibration = (short)(short)646;
            p188.sue_accel_y_at_calibration = (short)(short) -99;
            p188.sue_accel_z_at_calibration = (short)(short) -14168;
            p188.sue_gyro_x_at_calibration = (short)(short)2670;
            p188.sue_gyro_y_at_calibration = (short)(short) -14798;
            p188.sue_gyro_z_at_calibration = (short)(short) -16215;
            CommunicationChannel.instance.send(p188); //===============================
            ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.time_usec = (ulong)3380188262651324910L;
            p230.flags = (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL);
            p230.vel_ratio = (float)3.192422E38F;
            p230.pos_horiz_ratio = (float) -3.1278058E38F;
            p230.pos_vert_ratio = (float) -3.9877962E36F;
            p230.mag_ratio = (float) -2.522992E38F;
            p230.hagl_ratio = (float) -1.8428687E38F;
            p230.tas_ratio = (float) -2.2031786E38F;
            p230.pos_horiz_accuracy = (float)2.2256235E38F;
            p230.pos_vert_accuracy = (float)1.952323E38F;
            CommunicationChannel.instance.send(p230); //===============================
            WIND_COV p231 = CommunicationChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.time_usec = (ulong)2471403953960993977L;
            p231.wind_x = (float) -3.1569672E38F;
            p231.wind_y = (float)9.138797E37F;
            p231.wind_z = (float)2.2507274E38F;
            p231.var_horiz = (float)3.0135286E38F;
            p231.var_vert = (float)3.2419765E38F;
            p231.wind_alt = (float) -1.4423132E38F;
            p231.horiz_accuracy = (float) -2.0836845E38F;
            p231.vert_accuracy = (float)1.4414675E38F;
            CommunicationChannel.instance.send(p231); //===============================
            GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.time_usec = (ulong)4342006763179228830L;
            p232.gps_id = (byte)(byte)67;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP);
            p232.time_week_ms = (uint)1335505421U;
            p232.time_week = (ushort)(ushort)45686;
            p232.fix_type = (byte)(byte)43;
            p232.lat = (int)1376805978;
            p232.lon = (int)1673274326;
            p232.alt = (float) -1.6455834E38F;
            p232.hdop = (float)9.168822E37F;
            p232.vdop = (float) -2.8037176E37F;
            p232.vn = (float) -2.681848E38F;
            p232.ve = (float)3.3161156E38F;
            p232.vd = (float)2.617807E38F;
            p232.speed_accuracy = (float)1.5447961E38F;
            p232.horiz_accuracy = (float) -4.322673E37F;
            p232.vert_accuracy = (float)2.2662625E38F;
            p232.satellites_visible = (byte)(byte)233;
            CommunicationChannel.instance.send(p232); //===============================
            GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.flags = (byte)(byte)192;
            p233.len = (byte)(byte)38;
            p233.data__SET(new byte[180], 0);
            CommunicationChannel.instance.send(p233); //===============================
            HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED);
            p234.custom_mode = (uint)4192763728U;
            p234.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND;
            p234.roll = (short)(short) -16676;
            p234.pitch = (short)(short) -26347;
            p234.heading = (ushort)(ushort)41616;
            p234.throttle = (sbyte)(sbyte) - 6;
            p234.heading_sp = (short)(short)5504;
            p234.latitude = (int) -198660685;
            p234.longitude = (int) -1722809512;
            p234.altitude_amsl = (short)(short)30992;
            p234.altitude_sp = (short)(short) -31886;
            p234.airspeed = (byte)(byte)231;
            p234.airspeed_sp = (byte)(byte)150;
            p234.groundspeed = (byte)(byte)152;
            p234.climb_rate = (sbyte)(sbyte)32;
            p234.gps_nsat = (byte)(byte)194;
            p234.gps_fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX;
            p234.battery_remaining = (byte)(byte)27;
            p234.temperature = (sbyte)(sbyte)34;
            p234.temperature_air = (sbyte)(sbyte)78;
            p234.failsafe = (byte)(byte)36;
            p234.wp_num = (byte)(byte)163;
            p234.wp_distance = (ushort)(ushort)36468;
            CommunicationChannel.instance.send(p234); //===============================
            VIBRATION p241 = CommunicationChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.time_usec = (ulong)7366260013094889699L;
            p241.vibration_x = (float)1.9956601E38F;
            p241.vibration_y = (float)3.2576882E38F;
            p241.vibration_z = (float) -1.5070499E38F;
            p241.clipping_0 = (uint)284989495U;
            p241.clipping_1 = (uint)642441540U;
            p241.clipping_2 = (uint)841015625U;
            CommunicationChannel.instance.send(p241); //===============================
            HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.latitude = (int)92832306;
            p242.longitude = (int) -759982935;
            p242.altitude = (int)8423960;
            p242.x = (float) -1.99714E38F;
            p242.y = (float) -2.5294278E38F;
            p242.z = (float)3.0573165E38F;
            p242.q_SET(new float[4], 0);
            p242.approach_x = (float) -2.6835713E38F;
            p242.approach_y = (float)8.85198E37F;
            p242.approach_z = (float)1.4453595E38F;
            p242.time_usec_SET((ulong)7811855775437394618L, PH);
            CommunicationChannel.instance.send(p242); //===============================
            SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.target_system = (byte)(byte)134;
            p243.latitude = (int) -713639835;
            p243.longitude = (int) -892229837;
            p243.altitude = (int) -1795763252;
            p243.x = (float)2.6616233E38F;
            p243.y = (float)2.3963097E38F;
            p243.z = (float) -1.4416992E38F;
            p243.q_SET(new float[4], 0);
            p243.approach_x = (float) -5.394615E37F;
            p243.approach_y = (float)2.310935E38F;
            p243.approach_z = (float) -2.212852E38F;
            p243.time_usec_SET((ulong)4359402950164399296L, PH);
            CommunicationChannel.instance.send(p243); //===============================
            MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.message_id = (ushort)(ushort)22213;
            p244.interval_us = (int) -59073175;
            CommunicationChannel.instance.send(p244); //===============================
            EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.vtol_state = MAV_VTOL_STATE.MAV_VTOL_STATE_MC;
            p245.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING;
            CommunicationChannel.instance.send(p245); //===============================
            ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.ICAO_address = (uint)4177171910U;
            p246.lat = (int) -1159234527;
            p246.lon = (int)1477471554;
            p246.altitude_type = ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
            p246.altitude = (int) -482164050;
            p246.heading = (ushort)(ushort)40326;
            p246.hor_velocity = (ushort)(ushort)45640;
            p246.ver_velocity = (short)(short) -26134;
            p246.callsign_SET("DEMO", PH);
            p246.emitter_type = ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_HEAVY;
            p246.tslc = (byte)(byte)130;
            p246.flags = (ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS |
                          ADSB_FLAGS.ADSB_FLAGS_SIMULATED |
                          ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE |
                          ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN |
                          ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY);
            p246.squawk = (ushort)(ushort)31442;
            CommunicationChannel.instance.send(p246); //===============================
            COLLISION p247 = CommunicationChannel.new_COLLISION();
            PH.setPack(p247);
            p247.src_ = MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT;
            p247.id = (uint)168635609U;
            p247.action = MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_ASCEND_OR_DESCEND;
            p247.threat_level = (MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE |
                                 MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW);
            p247.time_to_minimum_delta = (float)1.4911383E38F;
            p247.altitude_minimum_delta = (float)5.5827895E37F;
            p247.horizontal_minimum_delta = (float) -1.8623575E37F;
            CommunicationChannel.instance.send(p247); //===============================
            V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.target_network = (byte)(byte)65;
            p248.target_system = (byte)(byte)234;
            p248.target_component = (byte)(byte)183;
            p248.message_type = (ushort)(ushort)23083;
            p248.payload_SET(new byte[249], 0);
            CommunicationChannel.instance.send(p248); //===============================
            MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.address = (ushort)(ushort)59164;
            p249.ver = (byte)(byte)116;
            p249.type = (byte)(byte)120;
            p249.value_SET(new sbyte[32], 0);
            CommunicationChannel.instance.send(p249); //===============================
            DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.name_SET("DEMO", PH);
            p250.time_usec = (ulong)8928327578989456657L;
            p250.x = (float)1.1377643E38F;
            p250.y = (float) -1.2200678E38F;
            p250.z = (float) -3.3667695E38F;
            CommunicationChannel.instance.send(p250); //===============================
            NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.time_boot_ms = (uint)2131533520U;
            p251.name_SET("DEMO", PH);
            p251.value = (float) -1.497876E38F;
            CommunicationChannel.instance.send(p251); //===============================
            NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.time_boot_ms = (uint)3091315626U;
            p252.name_SET("DEMO", PH);
            p252.value = (int) -894303503;
            CommunicationChannel.instance.send(p252); //===============================
            STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.severity = MAV_SEVERITY.MAV_SEVERITY_NOTICE;
            p253.text_SET("DEMO", PH);
            CommunicationChannel.instance.send(p253); //===============================
            DEBUG p254 = CommunicationChannel.new_DEBUG();
            PH.setPack(p254);
            p254.time_boot_ms = (uint)2255567188U;
            p254.ind = (byte)(byte)75;
            p254.value = (float) -1.6349051E38F;
            CommunicationChannel.instance.send(p254); //===============================
            SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.target_system = (byte)(byte)8;
            p256.target_component = (byte)(byte)190;
            p256.secret_key_SET(new byte[32], 0);
            p256.initial_timestamp = (ulong)8787159057177957116L;
            CommunicationChannel.instance.send(p256); //===============================
            BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.time_boot_ms = (uint)1102116390U;
            p257.last_change_ms = (uint)1974476183U;
            p257.state = (byte)(byte)4;
            CommunicationChannel.instance.send(p257); //===============================
            PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_system = (byte)(byte)21;
            p258.target_component = (byte)(byte)9;
            p258.tune_SET("DEMO", PH);
            CommunicationChannel.instance.send(p258); //===============================
            CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.time_boot_ms = (uint)2777491654U;
            p259.vendor_name_SET(new byte[32], 0);
            p259.model_name_SET(new byte[32], 0);
            p259.firmware_version = (uint)2492672406U;
            p259.focal_length = (float) -1.8241131E38F;
            p259.sensor_size_h = (float) -1.8743614E38F;
            p259.sensor_size_v = (float) -3.8913486E37F;
            p259.resolution_h = (ushort)(ushort)15096;
            p259.resolution_v = (ushort)(ushort)44628;
            p259.lens_id = (byte)(byte)143;
            p259.flags = (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
                          CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE |
                          CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE);
            p259.cam_definition_version = (ushort)(ushort)49442;
            p259.cam_definition_uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p259); //===============================
            CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.time_boot_ms = (uint)219573976U;
            p260.mode_id = (CAMERA_MODE.CAMERA_MODE_VIDEO);
            CommunicationChannel.instance.send(p260); //===============================
            STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.time_boot_ms = (uint)2377456194U;
            p261.storage_id = (byte)(byte)230;
            p261.storage_count = (byte)(byte)63;
            p261.status = (byte)(byte)54;
            p261.total_capacity = (float)6.959579E37F;
            p261.used_capacity = (float) -3.3626123E37F;
            p261.available_capacity = (float) -1.128671E38F;
            p261.read_speed = (float) -4.6956526E37F;
            p261.write_speed = (float) -6.475662E37F;
            CommunicationChannel.instance.send(p261); //===============================
            CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.time_boot_ms = (uint)4222992410U;
            p262.image_status = (byte)(byte)206;
            p262.video_status = (byte)(byte)97;
            p262.image_interval = (float)6.8042084E37F;
            p262.recording_time_ms = (uint)1655667852U;
            p262.available_capacity = (float) -3.0802962E38F;
            CommunicationChannel.instance.send(p262); //===============================
            CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.time_boot_ms = (uint)238227679U;
            p263.time_utc = (ulong)3266162583781802118L;
            p263.camera_id = (byte)(byte)205;
            p263.lat = (int) -804373180;
            p263.lon = (int) -522371742;
            p263.alt = (int) -877904596;
            p263.relative_alt = (int)278915853;
            p263.q_SET(new float[4], 0);
            p263.image_index = (int)3816219;
            p263.capture_result = (sbyte)(sbyte)90;
            p263.file_url_SET("DEMO", PH);
            CommunicationChannel.instance.send(p263); //===============================
            FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.time_boot_ms = (uint)2104194611U;
            p264.arming_time_utc = (ulong)8481591242039296573L;
            p264.takeoff_time_utc = (ulong)3714285497601164788L;
            p264.flight_uuid = (ulong)2800224128098664944L;
            CommunicationChannel.instance.send(p264); //===============================
            MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.time_boot_ms = (uint)742819882U;
            p265.roll = (float)3.1958628E38F;
            p265.pitch = (float)1.2377189E38F;
            p265.yaw = (float) -1.0648146E38F;
            CommunicationChannel.instance.send(p265); //===============================
            LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.target_system = (byte)(byte)17;
            p266.target_component = (byte)(byte)183;
            p266.sequence = (ushort)(ushort)18470;
            p266.length = (byte)(byte)157;
            p266.first_message_offset = (byte)(byte)216;
            p266.data__SET(new byte[249], 0);
            CommunicationChannel.instance.send(p266); //===============================
            LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.target_system = (byte)(byte)177;
            p267.target_component = (byte)(byte)212;
            p267.sequence = (ushort)(ushort)43911;
            p267.length = (byte)(byte)41;
            p267.first_message_offset = (byte)(byte)157;
            p267.data__SET(new byte[249], 0);
            CommunicationChannel.instance.send(p267); //===============================
            LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_system = (byte)(byte)61;
            p268.target_component = (byte)(byte)130;
            p268.sequence = (ushort)(ushort)36455;
            CommunicationChannel.instance.send(p268); //===============================
            VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.camera_id = (byte)(byte)164;
            p269.status = (byte)(byte)161;
            p269.framerate = (float) -5.1825825E36F;
            p269.resolution_h = (ushort)(ushort)20807;
            p269.resolution_v = (ushort)(ushort)24277;
            p269.bitrate = (uint)1012891745U;
            p269.rotation = (ushort)(ushort)4995;
            p269.uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p269); //===============================
            SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.target_system = (byte)(byte)17;
            p270.target_component = (byte)(byte)123;
            p270.camera_id = (byte)(byte)134;
            p270.framerate = (float) -1.4102338E38F;
            p270.resolution_h = (ushort)(ushort)31628;
            p270.resolution_v = (ushort)(ushort)20329;
            p270.bitrate = (uint)2956325316U;
            p270.rotation = (ushort)(ushort)55340;
            p270.uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p270); //===============================
            WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.ssid_SET("DEMO", PH);
            p299.password_SET("DEMO", PH);
            CommunicationChannel.instance.send(p299); //===============================
            PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.version = (ushort)(ushort)1361;
            p300.min_version = (ushort)(ushort)30903;
            p300.max_version = (ushort)(ushort)16728;
            p300.spec_version_hash_SET(new byte[8], 0);
            p300.library_version_hash_SET(new byte[8], 0);
            CommunicationChannel.instance.send(p300); //===============================
            UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.time_usec = (ulong)5302771756998832274L;
            p310.uptime_sec = (uint)3247121078U;
            p310.health = UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK;
            p310.mode = UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE;
            p310.sub_mode = (byte)(byte)32;
            p310.vendor_specific_status_code = (ushort)(ushort)25083;
            CommunicationChannel.instance.send(p310); //===============================
            UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.time_usec = (ulong)6972153309971842668L;
            p311.uptime_sec = (uint)2719551748U;
            p311.name_SET("DEMO", PH);
            p311.hw_version_major = (byte)(byte)50;
            p311.hw_version_minor = (byte)(byte)3;
            p311.hw_unique_id_SET(new byte[16], 0);
            p311.sw_version_major = (byte)(byte)121;
            p311.sw_version_minor = (byte)(byte)79;
            p311.sw_vcs_commit = (uint)3232061280U;
            CommunicationChannel.instance.send(p311); //===============================
            PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.target_system = (byte)(byte)67;
            p320.target_component = (byte)(byte)133;
            p320.param_id_SET("DEMO", PH);
            p320.param_index = (short)(short)13237;
            CommunicationChannel.instance.send(p320); //===============================
            PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_system = (byte)(byte)99;
            p321.target_component = (byte)(byte)58;
            CommunicationChannel.instance.send(p321); //===============================
            PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_id_SET("DEMO", PH);
            p322.param_value_SET("DEMO", PH);
            p322.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16;
            p322.param_count = (ushort)(ushort)51219;
            p322.param_index = (ushort)(ushort)12224;
            CommunicationChannel.instance.send(p322); //===============================
            PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.target_system = (byte)(byte)123;
            p323.target_component = (byte)(byte)39;
            p323.param_id_SET("DEMO", PH);
            p323.param_value_SET("DEMO", PH);
            p323.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT8;
            CommunicationChannel.instance.send(p323); //===============================
            PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_id_SET("DEMO", PH);
            p324.param_value_SET("DEMO", PH);
            p324.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT8;
            p324.param_result = PARAM_ACK.PARAM_ACK_IN_PROGRESS;
            CommunicationChannel.instance.send(p324); //===============================
            OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.time_usec = (ulong)3694431170257208847L;
            p330.sensor_type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND;
            p330.distances_SET(new ushort[72], 0);
            p330.increment = (byte)(byte)249;
            p330.min_distance = (ushort)(ushort)43290;
            p330.max_distance = (ushort)(ushort)48224;
            CommunicationChannel.instance.send(p330); //===============================
        }
    }
}

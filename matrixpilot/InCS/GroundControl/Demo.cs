
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
            p87.time_boot_ms = (uint)2604838258U;
            p87.coordinate_frame = MAV_FRAME.MAV_FRAME_MISSION;
            p87.type_mask = (ushort)(ushort)38227;
            p87.lat_int = (int) -1435306980;
            p87.lon_int = (int)2133762077;
            p87.alt = (float)1.7821245E38F;
            p87.vx = (float) -2.9280672E38F;
            p87.vy = (float) -8.6278385E36F;
            p87.vz = (float)1.8348852E37F;
            p87.afx = (float) -1.9469986E38F;
            p87.afy = (float) -3.2906642E38F;
            p87.afz = (float) -3.7202411E37F;
            p87.yaw = (float) -3.3667938E38F;
            p87.yaw_rate = (float) -1.3239999E38F;
            CommunicationChannel.instance.send(p87); //===============================
            LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.time_boot_ms = (uint)3633519878U;
            p89.x = (float) -2.026728E38F;
            p89.y = (float) -2.4815966E38F;
            p89.z = (float)3.2567842E38F;
            p89.roll = (float)1.8580085E37F;
            p89.pitch = (float)2.2424277E38F;
            p89.yaw = (float) -2.3578664E37F;
            CommunicationChannel.instance.send(p89); //===============================
            HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.time_usec = (ulong)8970548476776174600L;
            p90.roll = (float) -5.0157385E37F;
            p90.pitch = (float) -6.011853E37F;
            p90.yaw = (float)1.0344494E38F;
            p90.rollspeed = (float)1.775041E38F;
            p90.pitchspeed = (float)1.5345042E38F;
            p90.yawspeed = (float) -1.2558426E38F;
            p90.lat = (int) -1297115182;
            p90.lon = (int)1094439027;
            p90.alt = (int) -1413885772;
            p90.vx = (short)(short)31933;
            p90.vy = (short)(short) -7550;
            p90.vz = (short)(short)28911;
            p90.xacc = (short)(short) -31216;
            p90.yacc = (short)(short)3735;
            p90.zacc = (short)(short) -780;
            CommunicationChannel.instance.send(p90); //===============================
            HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.time_usec = (ulong)8377409999677101934L;
            p91.roll_ailerons = (float)1.2561433E38F;
            p91.pitch_elevator = (float) -9.82256E37F;
            p91.yaw_rudder = (float) -2.4323503E38F;
            p91.throttle = (float)3.3392322E38F;
            p91.aux1 = (float)7.7698037E37F;
            p91.aux2 = (float)2.691027E37F;
            p91.aux3 = (float)3.3555784E38F;
            p91.aux4 = (float)1.9501275E38F;
            p91.mode = MAV_MODE.MAV_MODE_PREFLIGHT;
            p91.nav_mode = (byte)(byte)185;
            CommunicationChannel.instance.send(p91); //===============================
            HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.time_usec = (ulong)6551349761553945356L;
            p92.chan1_raw = (ushort)(ushort)18442;
            p92.chan2_raw = (ushort)(ushort)12381;
            p92.chan3_raw = (ushort)(ushort)8177;
            p92.chan4_raw = (ushort)(ushort)44967;
            p92.chan5_raw = (ushort)(ushort)36164;
            p92.chan6_raw = (ushort)(ushort)31529;
            p92.chan7_raw = (ushort)(ushort)7628;
            p92.chan8_raw = (ushort)(ushort)31609;
            p92.chan9_raw = (ushort)(ushort)7929;
            p92.chan10_raw = (ushort)(ushort)61855;
            p92.chan11_raw = (ushort)(ushort)14983;
            p92.chan12_raw = (ushort)(ushort)29667;
            p92.rssi = (byte)(byte)176;
            CommunicationChannel.instance.send(p92); //===============================
            HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.time_usec = (ulong)2261879549712298044L;
            p93.controls_SET(new float[16], 0);
            p93.mode = MAV_MODE.MAV_MODE_AUTO_ARMED;
            p93.flags = (ulong)1680155357013362119L;
            CommunicationChannel.instance.send(p93); //===============================
            OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.time_usec = (ulong)2261790480774732779L;
            p100.sensor_id = (byte)(byte)131;
            p100.flow_x = (short)(short)18160;
            p100.flow_y = (short)(short) -16448;
            p100.flow_comp_m_x = (float) -1.2098072E38F;
            p100.flow_comp_m_y = (float)2.6348876E38F;
            p100.quality = (byte)(byte)17;
            p100.ground_distance = (float) -2.4238449E38F;
            p100.flow_rate_x_SET((float) -2.0995967E38F, PH);
            p100.flow_rate_y_SET((float) -1.5494479E38F, PH);
            CommunicationChannel.instance.send(p100); //===============================
            GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.usec = (ulong)8075116141296155585L;
            p101.x = (float)3.2970375E38F;
            p101.y = (float)3.1629748E38F;
            p101.z = (float) -2.1021714E38F;
            p101.roll = (float)4.407541E37F;
            p101.pitch = (float) -1.7802206E38F;
            p101.yaw = (float)2.6741982E38F;
            CommunicationChannel.instance.send(p101); //===============================
            VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.usec = (ulong)2420104063684996874L;
            p102.x = (float) -1.2484074E38F;
            p102.y = (float) -3.0184824E38F;
            p102.z = (float) -2.421708E38F;
            p102.roll = (float) -2.5061195E38F;
            p102.pitch = (float) -2.1962535E38F;
            p102.yaw = (float) -1.0386078E38F;
            CommunicationChannel.instance.send(p102); //===============================
            VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.usec = (ulong)8462090199132902893L;
            p103.x = (float)2.873044E38F;
            p103.y = (float)2.305526E38F;
            p103.z = (float)1.591471E38F;
            CommunicationChannel.instance.send(p103); //===============================
            VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.usec = (ulong)6580255181202997715L;
            p104.x = (float) -2.5972297E38F;
            p104.y = (float)7.1834154E37F;
            p104.z = (float)1.4912057E37F;
            p104.roll = (float) -1.3718586E38F;
            p104.pitch = (float)7.3715196E37F;
            p104.yaw = (float) -1.7229112E38F;
            CommunicationChannel.instance.send(p104); //===============================
            HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.time_usec = (ulong)4592464433186428176L;
            p105.xacc = (float)1.9935029E38F;
            p105.yacc = (float) -1.812396E38F;
            p105.zacc = (float)2.2936738E38F;
            p105.xgyro = (float)3.2361503E38F;
            p105.ygyro = (float) -1.5312433E38F;
            p105.zgyro = (float) -2.1588862E38F;
            p105.xmag = (float) -1.4909906E38F;
            p105.ymag = (float)2.2810678E37F;
            p105.zmag = (float) -1.1601799E38F;
            p105.abs_pressure = (float) -1.1436714E38F;
            p105.diff_pressure = (float)2.9119934E38F;
            p105.pressure_alt = (float) -2.416865E38F;
            p105.temperature = (float) -2.0838864E38F;
            p105.fields_updated = (ushort)(ushort)41041;
            CommunicationChannel.instance.send(p105); //===============================
            OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.time_usec = (ulong)5264592745266038778L;
            p106.sensor_id = (byte)(byte)98;
            p106.integration_time_us = (uint)2901648547U;
            p106.integrated_x = (float) -3.754022E37F;
            p106.integrated_y = (float) -2.913623E38F;
            p106.integrated_xgyro = (float) -5.6616237E37F;
            p106.integrated_ygyro = (float)2.4438563E38F;
            p106.integrated_zgyro = (float) -2.8683012E38F;
            p106.temperature = (short)(short) -28494;
            p106.quality = (byte)(byte)31;
            p106.time_delta_distance_us = (uint)4213180967U;
            p106.distance = (float) -4.0975558E37F;
            CommunicationChannel.instance.send(p106); //===============================
            HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.time_usec = (ulong)2135779934847513765L;
            p107.xacc = (float) -2.9085263E38F;
            p107.yacc = (float)1.6953259E38F;
            p107.zacc = (float)4.6887855E37F;
            p107.xgyro = (float)2.611669E38F;
            p107.ygyro = (float)3.0575747E38F;
            p107.zgyro = (float) -1.0761738E38F;
            p107.xmag = (float) -2.8786005E38F;
            p107.ymag = (float)1.0405749E38F;
            p107.zmag = (float) -1.7118638E38F;
            p107.abs_pressure = (float) -1.6153472E38F;
            p107.diff_pressure = (float)1.2821124E38F;
            p107.pressure_alt = (float) -1.24147E38F;
            p107.temperature = (float) -2.9927164E38F;
            p107.fields_updated = (uint)1218322050U;
            CommunicationChannel.instance.send(p107); //===============================
            SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.q1 = (float)5.866184E37F;
            p108.q2 = (float) -2.381739E38F;
            p108.q3 = (float)3.2169048E38F;
            p108.q4 = (float)3.3193E38F;
            p108.roll = (float)2.3750588E38F;
            p108.pitch = (float) -1.4777293E38F;
            p108.yaw = (float) -1.3113589E38F;
            p108.xacc = (float)2.2438566E38F;
            p108.yacc = (float) -1.352466E38F;
            p108.zacc = (float)2.5179557E38F;
            p108.xgyro = (float) -3.373126E38F;
            p108.ygyro = (float)1.2046498E38F;
            p108.zgyro = (float)2.524608E38F;
            p108.lat = (float)3.2314537E38F;
            p108.lon = (float)3.6473852E37F;
            p108.alt = (float)5.9330657E37F;
            p108.std_dev_horz = (float) -1.6034897E38F;
            p108.std_dev_vert = (float) -3.4439975E37F;
            p108.vn = (float) -3.4392137E35F;
            p108.ve = (float) -1.535569E38F;
            p108.vd = (float)3.3339034E38F;
            CommunicationChannel.instance.send(p108); //===============================
            RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.rssi = (byte)(byte)37;
            p109.remrssi = (byte)(byte)87;
            p109.txbuf = (byte)(byte)172;
            p109.noise = (byte)(byte)151;
            p109.remnoise = (byte)(byte)177;
            p109.rxerrors = (ushort)(ushort)29561;
            p109.fixed_ = (ushort)(ushort)55258;
            CommunicationChannel.instance.send(p109); //===============================
            FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_network = (byte)(byte)11;
            p110.target_system = (byte)(byte)87;
            p110.target_component = (byte)(byte)84;
            p110.payload_SET(new byte[251], 0);
            CommunicationChannel.instance.send(p110); //===============================
            TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.tc1 = (long)3876964115324109921L;
            p111.ts1 = (long)1808932911918231038L;
            CommunicationChannel.instance.send(p111); //===============================
            CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.time_usec = (ulong)6459542756814517010L;
            p112.seq = (uint)1125232737U;
            CommunicationChannel.instance.send(p112); //===============================
            HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.time_usec = (ulong)665452695529269109L;
            p113.fix_type = (byte)(byte)118;
            p113.lat = (int) -1969984562;
            p113.lon = (int) -1865023101;
            p113.alt = (int)291583586;
            p113.eph = (ushort)(ushort)62546;
            p113.epv = (ushort)(ushort)6836;
            p113.vel = (ushort)(ushort)5814;
            p113.vn = (short)(short) -32606;
            p113.ve = (short)(short)10866;
            p113.vd = (short)(short) -28378;
            p113.cog = (ushort)(ushort)42681;
            p113.satellites_visible = (byte)(byte)6;
            CommunicationChannel.instance.send(p113); //===============================
            HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.time_usec = (ulong)3853686828228126850L;
            p114.sensor_id = (byte)(byte)159;
            p114.integration_time_us = (uint)2651143821U;
            p114.integrated_x = (float)2.6940924E38F;
            p114.integrated_y = (float) -1.0286608E38F;
            p114.integrated_xgyro = (float) -7.629812E37F;
            p114.integrated_ygyro = (float) -9.883868E37F;
            p114.integrated_zgyro = (float) -4.4733047E37F;
            p114.temperature = (short)(short) -5945;
            p114.quality = (byte)(byte)184;
            p114.time_delta_distance_us = (uint)2170277821U;
            p114.distance = (float) -2.7434358E38F;
            CommunicationChannel.instance.send(p114); //===============================
            HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.time_usec = (ulong)3033587598144355237L;
            p115.attitude_quaternion_SET(new float[4], 0);
            p115.rollspeed = (float)2.4176034E38F;
            p115.pitchspeed = (float)1.854743E37F;
            p115.yawspeed = (float) -3.364667E38F;
            p115.lat = (int)621531162;
            p115.lon = (int)1874310922;
            p115.alt = (int) -76958028;
            p115.vx = (short)(short) -8055;
            p115.vy = (short)(short) -19514;
            p115.vz = (short)(short) -5454;
            p115.ind_airspeed = (ushort)(ushort)3769;
            p115.true_airspeed = (ushort)(ushort)8399;
            p115.xacc = (short)(short) -14784;
            p115.yacc = (short)(short) -15380;
            p115.zacc = (short)(short)20541;
            CommunicationChannel.instance.send(p115); //===============================
            SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.time_boot_ms = (uint)2255072786U;
            p116.xacc = (short)(short) -1264;
            p116.yacc = (short)(short)11680;
            p116.zacc = (short)(short) -14938;
            p116.xgyro = (short)(short) -29573;
            p116.ygyro = (short)(short) -16299;
            p116.zgyro = (short)(short)21809;
            p116.xmag = (short)(short) -32393;
            p116.ymag = (short)(short) -6842;
            p116.zmag = (short)(short) -24805;
            CommunicationChannel.instance.send(p116); //===============================
            LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.target_system = (byte)(byte)247;
            p117.target_component = (byte)(byte)199;
            p117.start = (ushort)(ushort)49142;
            p117.end = (ushort)(ushort)22947;
            CommunicationChannel.instance.send(p117); //===============================
            LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.id = (ushort)(ushort)8917;
            p118.num_logs = (ushort)(ushort)43173;
            p118.last_log_num = (ushort)(ushort)26543;
            p118.time_utc = (uint)2577252467U;
            p118.size = (uint)3097686899U;
            CommunicationChannel.instance.send(p118); //===============================
            LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.target_system = (byte)(byte)161;
            p119.target_component = (byte)(byte)27;
            p119.id = (ushort)(ushort)45904;
            p119.ofs = (uint)1569606680U;
            p119.count = (uint)2589785818U;
            CommunicationChannel.instance.send(p119); //===============================
            LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.id = (ushort)(ushort)27977;
            p120.ofs = (uint)1093790155U;
            p120.count = (byte)(byte)80;
            p120.data__SET(new byte[90], 0);
            CommunicationChannel.instance.send(p120); //===============================
            LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_system = (byte)(byte)99;
            p121.target_component = (byte)(byte)96;
            CommunicationChannel.instance.send(p121); //===============================
            LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_system = (byte)(byte)159;
            p122.target_component = (byte)(byte)68;
            CommunicationChannel.instance.send(p122); //===============================
            GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.target_system = (byte)(byte)9;
            p123.target_component = (byte)(byte)23;
            p123.len = (byte)(byte)197;
            p123.data__SET(new byte[110], 0);
            CommunicationChannel.instance.send(p123); //===============================
            GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.time_usec = (ulong)6787438724980688159L;
            p124.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS;
            p124.lat = (int) -1105407766;
            p124.lon = (int) -1135940333;
            p124.alt = (int)2146008297;
            p124.eph = (ushort)(ushort)45336;
            p124.epv = (ushort)(ushort)60944;
            p124.vel = (ushort)(ushort)39915;
            p124.cog = (ushort)(ushort)35940;
            p124.satellites_visible = (byte)(byte)196;
            p124.dgps_numch = (byte)(byte)173;
            p124.dgps_age = (uint)2159568440U;
            CommunicationChannel.instance.send(p124); //===============================
            POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vcc = (ushort)(ushort)52997;
            p125.Vservo = (ushort)(ushort)36245;
            p125.flags = (MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED |
                          MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT |
                          MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT |
                          MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID |
                          MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID);
            CommunicationChannel.instance.send(p125); //===============================
            SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.device = SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS2;
            p126.flags = (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING |
                          SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI);
            p126.timeout = (ushort)(ushort)2185;
            p126.baudrate = (uint)1773446227U;
            p126.count = (byte)(byte)2;
            p126.data__SET(new byte[70], 0);
            CommunicationChannel.instance.send(p126); //===============================
            GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.time_last_baseline_ms = (uint)3962125506U;
            p127.rtk_receiver_id = (byte)(byte)37;
            p127.wn = (ushort)(ushort)39509;
            p127.tow = (uint)981132707U;
            p127.rtk_health = (byte)(byte)120;
            p127.rtk_rate = (byte)(byte)111;
            p127.nsats = (byte)(byte)186;
            p127.baseline_coords_type = (byte)(byte)14;
            p127.baseline_a_mm = (int)734148384;
            p127.baseline_b_mm = (int) -2041257639;
            p127.baseline_c_mm = (int)1223042913;
            p127.accuracy = (uint)570002346U;
            p127.iar_num_hypotheses = (int) -106498590;
            CommunicationChannel.instance.send(p127); //===============================
            GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.time_last_baseline_ms = (uint)1828574016U;
            p128.rtk_receiver_id = (byte)(byte)162;
            p128.wn = (ushort)(ushort)17584;
            p128.tow = (uint)2834604992U;
            p128.rtk_health = (byte)(byte)127;
            p128.rtk_rate = (byte)(byte)24;
            p128.nsats = (byte)(byte)104;
            p128.baseline_coords_type = (byte)(byte)195;
            p128.baseline_a_mm = (int)1912540229;
            p128.baseline_b_mm = (int)664709559;
            p128.baseline_c_mm = (int) -362407225;
            p128.accuracy = (uint)3541770373U;
            p128.iar_num_hypotheses = (int) -149247024;
            CommunicationChannel.instance.send(p128); //===============================
            SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.time_boot_ms = (uint)188900336U;
            p129.xacc = (short)(short) -10039;
            p129.yacc = (short)(short) -14034;
            p129.zacc = (short)(short)25122;
            p129.xgyro = (short)(short) -8836;
            p129.ygyro = (short)(short) -1056;
            p129.zgyro = (short)(short)18794;
            p129.xmag = (short)(short) -22269;
            p129.ymag = (short)(short) -5145;
            p129.zmag = (short)(short) -5171;
            CommunicationChannel.instance.send(p129); //===============================
            DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.type = (byte)(byte)89;
            p130.size = (uint)2819633527U;
            p130.width = (ushort)(ushort)42957;
            p130.height = (ushort)(ushort)63433;
            p130.packets = (ushort)(ushort)43537;
            p130.payload = (byte)(byte)93;
            p130.jpg_quality = (byte)(byte)152;
            CommunicationChannel.instance.send(p130); //===============================
            ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.seqnr = (ushort)(ushort)48266;
            p131.data__SET(new byte[253], 0);
            CommunicationChannel.instance.send(p131); //===============================
            DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.time_boot_ms = (uint)1861175715U;
            p132.min_distance = (ushort)(ushort)22635;
            p132.max_distance = (ushort)(ushort)53613;
            p132.current_distance = (ushort)(ushort)34635;
            p132.type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND;
            p132.id = (byte)(byte)203;
            p132.orientation = MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_135;
            p132.covariance = (byte)(byte)121;
            CommunicationChannel.instance.send(p132); //===============================
            TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.lat = (int) -1578593060;
            p133.lon = (int)624229072;
            p133.grid_spacing = (ushort)(ushort)62465;
            p133.mask = (ulong)5409914375760114477L;
            CommunicationChannel.instance.send(p133); //===============================
            TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.lat = (int)931340913;
            p134.lon = (int)748884629;
            p134.grid_spacing = (ushort)(ushort)27830;
            p134.gridbit = (byte)(byte)167;
            p134.data__SET(new short[16], 0);
            CommunicationChannel.instance.send(p134); //===============================
            TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lat = (int)1371328050;
            p135.lon = (int)1876311382;
            CommunicationChannel.instance.send(p135); //===============================
            TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.lat = (int) -975907829;
            p136.lon = (int)416628910;
            p136.spacing = (ushort)(ushort)43397;
            p136.terrain_height = (float)3.2629515E38F;
            p136.current_height = (float)3.0076816E38F;
            p136.pending = (ushort)(ushort)62230;
            p136.loaded = (ushort)(ushort)53716;
            CommunicationChannel.instance.send(p136); //===============================
            SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.time_boot_ms = (uint)3713383581U;
            p137.press_abs = (float) -3.1412297E38F;
            p137.press_diff = (float) -1.5236266E38F;
            p137.temperature = (short)(short)15178;
            CommunicationChannel.instance.send(p137); //===============================
            ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.time_usec = (ulong)6369255754798574568L;
            p138.q_SET(new float[4], 0);
            p138.x = (float) -3.383261E38F;
            p138.y = (float)4.810116E37F;
            p138.z = (float)1.509704E38F;
            CommunicationChannel.instance.send(p138); //===============================
            SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.time_usec = (ulong)7553905484430571513L;
            p139.group_mlx = (byte)(byte)129;
            p139.target_system = (byte)(byte)145;
            p139.target_component = (byte)(byte)88;
            p139.controls_SET(new float[8], 0);
            CommunicationChannel.instance.send(p139); //===============================
            ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.time_usec = (ulong)2494102846073328675L;
            p140.group_mlx = (byte)(byte)113;
            p140.controls_SET(new float[8], 0);
            CommunicationChannel.instance.send(p140); //===============================
            ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.time_usec = (ulong)2881593060132214932L;
            p141.altitude_monotonic = (float) -5.088251E37F;
            p141.altitude_amsl = (float) -3.3522442E38F;
            p141.altitude_local = (float)2.9705404E38F;
            p141.altitude_relative = (float)1.5709156E38F;
            p141.altitude_terrain = (float)1.602931E38F;
            p141.bottom_clearance = (float) -1.8655464E38F;
            CommunicationChannel.instance.send(p141); //===============================
            RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.request_id = (byte)(byte)82;
            p142.uri_type = (byte)(byte)143;
            p142.uri_SET(new byte[120], 0);
            p142.transfer_type = (byte)(byte)195;
            p142.storage_SET(new byte[120], 0);
            CommunicationChannel.instance.send(p142); //===============================
            SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.time_boot_ms = (uint)3201029290U;
            p143.press_abs = (float) -3.2548006E38F;
            p143.press_diff = (float)2.611299E38F;
            p143.temperature = (short)(short) -16669;
            CommunicationChannel.instance.send(p143); //===============================
            FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.timestamp = (ulong)8136965533288403905L;
            p144.est_capabilities = (byte)(byte)39;
            p144.lat = (int)1918016872;
            p144.lon = (int) -738099835;
            p144.alt = (float)2.667324E38F;
            p144.vel_SET(new float[3], 0);
            p144.acc_SET(new float[3], 0);
            p144.attitude_q_SET(new float[4], 0);
            p144.rates_SET(new float[3], 0);
            p144.position_cov_SET(new float[3], 0);
            p144.custom_state = (ulong)2534847572410233721L;
            CommunicationChannel.instance.send(p144); //===============================
            CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.time_usec = (ulong)8242413895582305898L;
            p146.x_acc = (float)1.2337871E37F;
            p146.y_acc = (float)1.2237476E38F;
            p146.z_acc = (float)1.7937116E38F;
            p146.x_vel = (float)2.0181872E38F;
            p146.y_vel = (float) -1.6789524E38F;
            p146.z_vel = (float) -1.4005247E38F;
            p146.x_pos = (float) -3.1011086E38F;
            p146.y_pos = (float) -5.369078E37F;
            p146.z_pos = (float)1.2668778E38F;
            p146.airspeed = (float)7.244958E37F;
            p146.vel_variance_SET(new float[3], 0);
            p146.pos_variance_SET(new float[3], 0);
            p146.q_SET(new float[4], 0);
            p146.roll_rate = (float) -9.912124E37F;
            p146.pitch_rate = (float) -1.068425E38F;
            p146.yaw_rate = (float)2.2447141E38F;
            CommunicationChannel.instance.send(p146); //===============================
            BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.id = (byte)(byte)68;
            p147.battery_function = MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_UNKNOWN;
            p147.type = MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIFE;
            p147.temperature = (short)(short)29485;
            p147.voltages_SET(new ushort[10], 0);
            p147.current_battery = (short)(short)13877;
            p147.current_consumed = (int)386779658;
            p147.energy_consumed = (int) -1535306824;
            p147.battery_remaining = (sbyte)(sbyte) - 2;
            CommunicationChannel.instance.send(p147); //===============================
            AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_INT |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMMAND_INT);
            p148.flight_sw_version = (uint)4221826496U;
            p148.middleware_sw_version = (uint)416625051U;
            p148.os_sw_version = (uint)2882374855U;
            p148.board_version = (uint)2938993377U;
            p148.flight_custom_version_SET(new byte[8], 0);
            p148.middleware_custom_version_SET(new byte[8], 0);
            p148.os_custom_version_SET(new byte[8], 0);
            p148.vendor_id = (ushort)(ushort)19577;
            p148.product_id = (ushort)(ushort)37530;
            p148.uid = (ulong)2886208166240353637L;
            p148.uid2_SET(new byte[18], 0, PH);
            CommunicationChannel.instance.send(p148); //===============================
            LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.time_usec = (ulong)8376744672657234811L;
            p149.target_num = (byte)(byte)23;
            p149.frame = MAV_FRAME.MAV_FRAME_GLOBAL_INT;
            p149.angle_x = (float) -1.5200047E38F;
            p149.angle_y = (float) -1.0391153E38F;
            p149.distance = (float) -2.6593304E38F;
            p149.size_x = (float)9.388008E37F;
            p149.size_y = (float)2.2345802E38F;
            p149.x_SET((float)7.399252E37F, PH);
            p149.y_SET((float)2.5410644E38F, PH);
            p149.z_SET((float)5.1077395E36F, PH);
            p149.q_SET(new float[4], 0, PH);
            p149.type = LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_FIDUCIAL;
            p149.position_valid_SET((byte)(byte)129, PH);
            CommunicationChannel.instance.send(p149); //===============================
            FLEXIFUNCTION_SET p150 = CommunicationChannel.new_FLEXIFUNCTION_SET();
            PH.setPack(p150);
            p150.target_system = (byte)(byte)88;
            p150.target_component = (byte)(byte)102;
            CommunicationChannel.instance.send(p150); //===============================
            FLEXIFUNCTION_READ_REQ p151 = CommunicationChannel.new_FLEXIFUNCTION_READ_REQ();
            PH.setPack(p151);
            p151.target_system = (byte)(byte)11;
            p151.target_component = (byte)(byte)183;
            p151.read_req_type = (short)(short)8473;
            p151.data_index = (short)(short)24444;
            CommunicationChannel.instance.send(p151); //===============================
            FLEXIFUNCTION_BUFFER_FUNCTION p152 = CommunicationChannel.new_FLEXIFUNCTION_BUFFER_FUNCTION();
            PH.setPack(p152);
            p152.target_system = (byte)(byte)193;
            p152.target_component = (byte)(byte)112;
            p152.func_index = (ushort)(ushort)6024;
            p152.func_count = (ushort)(ushort)13863;
            p152.data_address = (ushort)(ushort)54207;
            p152.data_size = (ushort)(ushort)28595;
            p152.data__SET(new sbyte[48], 0);
            CommunicationChannel.instance.send(p152); //===============================
            FLEXIFUNCTION_BUFFER_FUNCTION_ACK p153 = CommunicationChannel.new_FLEXIFUNCTION_BUFFER_FUNCTION_ACK();
            PH.setPack(p153);
            p153.target_system = (byte)(byte)213;
            p153.target_component = (byte)(byte)208;
            p153.func_index = (ushort)(ushort)43747;
            p153.result = (ushort)(ushort)62535;
            CommunicationChannel.instance.send(p153); //===============================
            FLEXIFUNCTION_DIRECTORY p155 = CommunicationChannel.new_FLEXIFUNCTION_DIRECTORY();
            PH.setPack(p155);
            p155.target_system = (byte)(byte)9;
            p155.target_component = (byte)(byte)190;
            p155.directory_type = (byte)(byte)226;
            p155.start_index = (byte)(byte)220;
            p155.count = (byte)(byte)246;
            p155.directory_data_SET(new sbyte[48], 0);
            CommunicationChannel.instance.send(p155); //===============================
            FLEXIFUNCTION_DIRECTORY_ACK p156 = CommunicationChannel.new_FLEXIFUNCTION_DIRECTORY_ACK();
            PH.setPack(p156);
            p156.target_system = (byte)(byte)196;
            p156.target_component = (byte)(byte)67;
            p156.directory_type = (byte)(byte)136;
            p156.start_index = (byte)(byte)199;
            p156.count = (byte)(byte)215;
            p156.result = (ushort)(ushort)5394;
            CommunicationChannel.instance.send(p156); //===============================
            FLEXIFUNCTION_COMMAND p157 = CommunicationChannel.new_FLEXIFUNCTION_COMMAND();
            PH.setPack(p157);
            p157.target_system = (byte)(byte)61;
            p157.target_component = (byte)(byte)106;
            p157.command_type = (byte)(byte)190;
            CommunicationChannel.instance.send(p157); //===============================
            FLEXIFUNCTION_COMMAND_ACK p158 = CommunicationChannel.new_FLEXIFUNCTION_COMMAND_ACK();
            PH.setPack(p158);
            p158.command_type = (ushort)(ushort)28281;
            p158.result = (ushort)(ushort)61321;
            CommunicationChannel.instance.send(p158); //===============================
            SERIAL_UDB_EXTRA_F2_A p170 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F2_A();
            PH.setPack(p170);
            p170.sue_time = (uint)3389607504U;
            p170.sue_status = (byte)(byte)20;
            p170.sue_latitude = (int) -278103280;
            p170.sue_longitude = (int)1059330993;
            p170.sue_altitude = (int)1629781012;
            p170.sue_waypoint_index = (ushort)(ushort)39880;
            p170.sue_rmat0 = (short)(short)1960;
            p170.sue_rmat1 = (short)(short)12094;
            p170.sue_rmat2 = (short)(short) -9130;
            p170.sue_rmat3 = (short)(short) -12445;
            p170.sue_rmat4 = (short)(short)12292;
            p170.sue_rmat5 = (short)(short)18530;
            p170.sue_rmat6 = (short)(short)3503;
            p170.sue_rmat7 = (short)(short) -38;
            p170.sue_rmat8 = (short)(short) -18768;
            p170.sue_cog = (ushort)(ushort)41863;
            p170.sue_sog = (short)(short)17099;
            p170.sue_cpu_load = (ushort)(ushort)22235;
            p170.sue_air_speed_3DIMU = (ushort)(ushort)50491;
            p170.sue_estimated_wind_0 = (short)(short) -11687;
            p170.sue_estimated_wind_1 = (short)(short)12471;
            p170.sue_estimated_wind_2 = (short)(short)8501;
            p170.sue_magFieldEarth0 = (short)(short) -6726;
            p170.sue_magFieldEarth1 = (short)(short) -24753;
            p170.sue_magFieldEarth2 = (short)(short) -15396;
            p170.sue_svs = (short)(short)19935;
            p170.sue_hdop = (short)(short) -25428;
            CommunicationChannel.instance.send(p170); //===============================
            SERIAL_UDB_EXTRA_F2_B p171 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F2_B();
            PH.setPack(p171);
            p171.sue_time = (uint)851048279U;
            p171.sue_pwm_input_1 = (short)(short) -25443;
            p171.sue_pwm_input_2 = (short)(short)24430;
            p171.sue_pwm_input_3 = (short)(short) -1383;
            p171.sue_pwm_input_4 = (short)(short)4459;
            p171.sue_pwm_input_5 = (short)(short) -30459;
            p171.sue_pwm_input_6 = (short)(short) -31468;
            p171.sue_pwm_input_7 = (short)(short)26255;
            p171.sue_pwm_input_8 = (short)(short)18056;
            p171.sue_pwm_input_9 = (short)(short) -15119;
            p171.sue_pwm_input_10 = (short)(short)10665;
            p171.sue_pwm_input_11 = (short)(short)15546;
            p171.sue_pwm_input_12 = (short)(short) -9711;
            p171.sue_pwm_output_1 = (short)(short) -10887;
            p171.sue_pwm_output_2 = (short)(short)6715;
            p171.sue_pwm_output_3 = (short)(short) -21251;
            p171.sue_pwm_output_4 = (short)(short) -27959;
            p171.sue_pwm_output_5 = (short)(short) -32589;
            p171.sue_pwm_output_6 = (short)(short)342;
            p171.sue_pwm_output_7 = (short)(short) -6505;
            p171.sue_pwm_output_8 = (short)(short) -24261;
            p171.sue_pwm_output_9 = (short)(short) -20892;
            p171.sue_pwm_output_10 = (short)(short) -9953;
            p171.sue_pwm_output_11 = (short)(short) -2263;
            p171.sue_pwm_output_12 = (short)(short)30259;
            p171.sue_imu_location_x = (short)(short)18260;
            p171.sue_imu_location_y = (short)(short) -21870;
            p171.sue_imu_location_z = (short)(short)15794;
            p171.sue_location_error_earth_x = (short)(short)10787;
            p171.sue_location_error_earth_y = (short)(short) -24539;
            p171.sue_location_error_earth_z = (short)(short) -4198;
            p171.sue_flags = (uint)9660882U;
            p171.sue_osc_fails = (short)(short)23786;
            p171.sue_imu_velocity_x = (short)(short) -6479;
            p171.sue_imu_velocity_y = (short)(short)25750;
            p171.sue_imu_velocity_z = (short)(short) -12270;
            p171.sue_waypoint_goal_x = (short)(short)17010;
            p171.sue_waypoint_goal_y = (short)(short) -29711;
            p171.sue_waypoint_goal_z = (short)(short)21252;
            p171.sue_aero_x = (short)(short)1867;
            p171.sue_aero_y = (short)(short)5216;
            p171.sue_aero_z = (short)(short) -31305;
            p171.sue_barom_temp = (short)(short)14606;
            p171.sue_barom_press = (int) -1519420296;
            p171.sue_barom_alt = (int)2030748011;
            p171.sue_bat_volt = (short)(short)31847;
            p171.sue_bat_amp = (short)(short) -13377;
            p171.sue_bat_amp_hours = (short)(short)16086;
            p171.sue_desired_height = (short)(short) -31979;
            p171.sue_memory_stack_free = (short)(short) -23848;
            CommunicationChannel.instance.send(p171); //===============================
            SERIAL_UDB_EXTRA_F4 p172 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F4();
            PH.setPack(p172);
            p172.sue_ROLL_STABILIZATION_AILERONS = (byte)(byte)81;
            p172.sue_ROLL_STABILIZATION_RUDDER = (byte)(byte)103;
            p172.sue_PITCH_STABILIZATION = (byte)(byte)7;
            p172.sue_YAW_STABILIZATION_RUDDER = (byte)(byte)48;
            p172.sue_YAW_STABILIZATION_AILERON = (byte)(byte)188;
            p172.sue_AILERON_NAVIGATION = (byte)(byte)64;
            p172.sue_RUDDER_NAVIGATION = (byte)(byte)234;
            p172.sue_ALTITUDEHOLD_STABILIZED = (byte)(byte)42;
            p172.sue_ALTITUDEHOLD_WAYPOINT = (byte)(byte)78;
            p172.sue_RACING_MODE = (byte)(byte)87;
            CommunicationChannel.instance.send(p172); //===============================
            SERIAL_UDB_EXTRA_F5 p173 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F5();
            PH.setPack(p173);
            p173.sue_YAWKP_AILERON = (float) -2.8614576E38F;
            p173.sue_YAWKD_AILERON = (float) -4.3313167E37F;
            p173.sue_ROLLKP = (float)2.569175E38F;
            p173.sue_ROLLKD = (float) -3.2003592E38F;
            CommunicationChannel.instance.send(p173); //===============================
            SERIAL_UDB_EXTRA_F6 p174 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F6();
            PH.setPack(p174);
            p174.sue_PITCHGAIN = (float) -6.087991E37F;
            p174.sue_PITCHKD = (float) -2.7994051E38F;
            p174.sue_RUDDER_ELEV_MIX = (float)2.9157357E38F;
            p174.sue_ROLL_ELEV_MIX = (float)1.9258517E38F;
            p174.sue_ELEVATOR_BOOST = (float)9.465479E37F;
            CommunicationChannel.instance.send(p174); //===============================
            SERIAL_UDB_EXTRA_F7 p175 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F7();
            PH.setPack(p175);
            p175.sue_YAWKP_RUDDER = (float)2.930584E38F;
            p175.sue_YAWKD_RUDDER = (float)1.5929986E38F;
            p175.sue_ROLLKP_RUDDER = (float)2.8921477E38F;
            p175.sue_ROLLKD_RUDDER = (float) -1.2913572E38F;
            p175.sue_RUDDER_BOOST = (float) -3.2223333E38F;
            p175.sue_RTL_PITCH_DOWN = (float)1.6106591E38F;
            CommunicationChannel.instance.send(p175); //===============================
            SERIAL_UDB_EXTRA_F8 p176 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F8();
            PH.setPack(p176);
            p176.sue_HEIGHT_TARGET_MAX = (float)3.239552E38F;
            p176.sue_HEIGHT_TARGET_MIN = (float)1.2523354E38F;
            p176.sue_ALT_HOLD_THROTTLE_MIN = (float) -1.7527438E38F;
            p176.sue_ALT_HOLD_THROTTLE_MAX = (float) -2.6593304E38F;
            p176.sue_ALT_HOLD_PITCH_MIN = (float)2.850928E37F;
            p176.sue_ALT_HOLD_PITCH_MAX = (float) -3.1365633E38F;
            p176.sue_ALT_HOLD_PITCH_HIGH = (float)3.1294837E38F;
            CommunicationChannel.instance.send(p176); //===============================
            SERIAL_UDB_EXTRA_F13 p177 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F13();
            PH.setPack(p177);
            p177.sue_week_no = (short)(short) -31256;
            p177.sue_lat_origin = (int)70711653;
            p177.sue_lon_origin = (int) -17064509;
            p177.sue_alt_origin = (int) -311624975;
            CommunicationChannel.instance.send(p177); //===============================
            SERIAL_UDB_EXTRA_F14 p178 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F14();
            PH.setPack(p178);
            p178.sue_WIND_ESTIMATION = (byte)(byte)119;
            p178.sue_GPS_TYPE = (byte)(byte)121;
            p178.sue_DR = (byte)(byte)63;
            p178.sue_BOARD_TYPE = (byte)(byte)199;
            p178.sue_AIRFRAME = (byte)(byte)55;
            p178.sue_RCON = (short)(short)12650;
            p178.sue_TRAP_FLAGS = (short)(short)25299;
            p178.sue_TRAP_SOURCE = (uint)240209222U;
            p178.sue_osc_fail_count = (short)(short)15343;
            p178.sue_CLOCK_CONFIG = (byte)(byte)121;
            p178.sue_FLIGHT_PLAN_TYPE = (byte)(byte)120;
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
            p181.time_boot_ms = (uint)2192785477U;
            p181.alt_gps = (int) -1801933886;
            p181.alt_imu = (int)1771482328;
            p181.alt_barometric = (int)258372488;
            p181.alt_optical_flow = (int)454894802;
            p181.alt_range_finder = (int) -1982731251;
            p181.alt_extra = (int) -1856099920;
            CommunicationChannel.instance.send(p181); //===============================
            AIRSPEEDS p182 = CommunicationChannel.new_AIRSPEEDS();
            PH.setPack(p182);
            p182.time_boot_ms = (uint)3681836394U;
            p182.airspeed_imu = (short)(short)24067;
            p182.airspeed_pitot = (short)(short) -23277;
            p182.airspeed_hot_wire = (short)(short) -15985;
            p182.airspeed_ultrasonic = (short)(short)26272;
            p182.aoa = (short)(short)31643;
            p182.aoy = (short)(short)15469;
            CommunicationChannel.instance.send(p182); //===============================
            SERIAL_UDB_EXTRA_F17 p183 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F17();
            PH.setPack(p183);
            p183.sue_feed_forward = (float) -2.9457183E37F;
            p183.sue_turn_rate_nav = (float) -1.5987542E38F;
            p183.sue_turn_rate_fbw = (float) -1.706218E38F;
            CommunicationChannel.instance.send(p183); //===============================
            SERIAL_UDB_EXTRA_F18 p184 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F18();
            PH.setPack(p184);
            p184.angle_of_attack_normal = (float) -2.5481304E38F;
            p184.angle_of_attack_inverted = (float)2.7242828E38F;
            p184.elevator_trim_normal = (float) -2.4915508E38F;
            p184.elevator_trim_inverted = (float)1.5114995E37F;
            p184.reference_speed = (float)1.0636821E37F;
            CommunicationChannel.instance.send(p184); //===============================
            SERIAL_UDB_EXTRA_F19 p185 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F19();
            PH.setPack(p185);
            p185.sue_aileron_output_channel = (byte)(byte)27;
            p185.sue_aileron_reversed = (byte)(byte)117;
            p185.sue_elevator_output_channel = (byte)(byte)53;
            p185.sue_elevator_reversed = (byte)(byte)46;
            p185.sue_throttle_output_channel = (byte)(byte)150;
            p185.sue_throttle_reversed = (byte)(byte)193;
            p185.sue_rudder_output_channel = (byte)(byte)88;
            p185.sue_rudder_reversed = (byte)(byte)112;
            CommunicationChannel.instance.send(p185); //===============================
            SERIAL_UDB_EXTRA_F20 p186 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F20();
            PH.setPack(p186);
            p186.sue_number_of_inputs = (byte)(byte)177;
            p186.sue_trim_value_input_1 = (short)(short) -4927;
            p186.sue_trim_value_input_2 = (short)(short) -22551;
            p186.sue_trim_value_input_3 = (short)(short)10239;
            p186.sue_trim_value_input_4 = (short)(short) -24839;
            p186.sue_trim_value_input_5 = (short)(short) -21854;
            p186.sue_trim_value_input_6 = (short)(short) -27974;
            p186.sue_trim_value_input_7 = (short)(short) -14094;
            p186.sue_trim_value_input_8 = (short)(short)3462;
            p186.sue_trim_value_input_9 = (short)(short) -23785;
            p186.sue_trim_value_input_10 = (short)(short) -11849;
            p186.sue_trim_value_input_11 = (short)(short) -2072;
            p186.sue_trim_value_input_12 = (short)(short)618;
            CommunicationChannel.instance.send(p186); //===============================
            SERIAL_UDB_EXTRA_F21 p187 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F21();
            PH.setPack(p187);
            p187.sue_accel_x_offset = (short)(short)16349;
            p187.sue_accel_y_offset = (short)(short)10471;
            p187.sue_accel_z_offset = (short)(short)32351;
            p187.sue_gyro_x_offset = (short)(short) -1054;
            p187.sue_gyro_y_offset = (short)(short)25061;
            p187.sue_gyro_z_offset = (short)(short)22072;
            CommunicationChannel.instance.send(p187); //===============================
            SERIAL_UDB_EXTRA_F22 p188 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F22();
            PH.setPack(p188);
            p188.sue_accel_x_at_calibration = (short)(short)16355;
            p188.sue_accel_y_at_calibration = (short)(short)20407;
            p188.sue_accel_z_at_calibration = (short)(short) -9560;
            p188.sue_gyro_x_at_calibration = (short)(short) -17996;
            p188.sue_gyro_y_at_calibration = (short)(short) -14512;
            p188.sue_gyro_z_at_calibration = (short)(short) -32472;
            CommunicationChannel.instance.send(p188); //===============================
            ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.time_usec = (ulong)4990325551653726341L;
            p230.flags = (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ);
            p230.vel_ratio = (float)2.058558E38F;
            p230.pos_horiz_ratio = (float)3.3931989E38F;
            p230.pos_vert_ratio = (float) -2.382659E38F;
            p230.mag_ratio = (float) -3.017313E38F;
            p230.hagl_ratio = (float)1.656875E38F;
            p230.tas_ratio = (float)1.2575601E38F;
            p230.pos_horiz_accuracy = (float)1.6673762E38F;
            p230.pos_vert_accuracy = (float)3.7363058E37F;
            CommunicationChannel.instance.send(p230); //===============================
            WIND_COV p231 = CommunicationChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.time_usec = (ulong)1346883309759811998L;
            p231.wind_x = (float) -1.6846514E37F;
            p231.wind_y = (float) -1.4072992E38F;
            p231.wind_z = (float)3.3608991E38F;
            p231.var_horiz = (float) -1.967498E38F;
            p231.var_vert = (float) -3.1551112E38F;
            p231.wind_alt = (float) -5.9859313E37F;
            p231.horiz_accuracy = (float)2.8302794E38F;
            p231.vert_accuracy = (float)6.028153E37F;
            CommunicationChannel.instance.send(p231); //===============================
            GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.time_usec = (ulong)6963148698766351103L;
            p232.gps_id = (byte)(byte)4;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ);
            p232.time_week_ms = (uint)1986385017U;
            p232.time_week = (ushort)(ushort)7431;
            p232.fix_type = (byte)(byte)201;
            p232.lat = (int) -1455713899;
            p232.lon = (int) -1495502078;
            p232.alt = (float)2.8042857E38F;
            p232.hdop = (float)1.2206488E38F;
            p232.vdop = (float) -2.9097254E38F;
            p232.vn = (float) -6.335941E37F;
            p232.ve = (float)3.1302423E37F;
            p232.vd = (float) -1.4522906E38F;
            p232.speed_accuracy = (float)3.3494154E38F;
            p232.horiz_accuracy = (float)4.0484455E37F;
            p232.vert_accuracy = (float)2.6619109E38F;
            p232.satellites_visible = (byte)(byte)213;
            CommunicationChannel.instance.send(p232); //===============================
            GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.flags = (byte)(byte)25;
            p233.len = (byte)(byte)227;
            p233.data__SET(new byte[180], 0);
            CommunicationChannel.instance.send(p233); //===============================
            HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED);
            p234.custom_mode = (uint)2922577511U;
            p234.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR;
            p234.roll = (short)(short) -19528;
            p234.pitch = (short)(short) -13306;
            p234.heading = (ushort)(ushort)49090;
            p234.throttle = (sbyte)(sbyte)84;
            p234.heading_sp = (short)(short)20178;
            p234.latitude = (int) -1156402130;
            p234.longitude = (int) -859826442;
            p234.altitude_amsl = (short)(short) -19017;
            p234.altitude_sp = (short)(short) -26001;
            p234.airspeed = (byte)(byte)62;
            p234.airspeed_sp = (byte)(byte)45;
            p234.groundspeed = (byte)(byte)167;
            p234.climb_rate = (sbyte)(sbyte)99;
            p234.gps_nsat = (byte)(byte)81;
            p234.gps_fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX;
            p234.battery_remaining = (byte)(byte)10;
            p234.temperature = (sbyte)(sbyte) - 114;
            p234.temperature_air = (sbyte)(sbyte) - 86;
            p234.failsafe = (byte)(byte)129;
            p234.wp_num = (byte)(byte)44;
            p234.wp_distance = (ushort)(ushort)46208;
            CommunicationChannel.instance.send(p234); //===============================
            VIBRATION p241 = CommunicationChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.time_usec = (ulong)4487695739081242713L;
            p241.vibration_x = (float) -1.969233E38F;
            p241.vibration_y = (float)7.8454064E37F;
            p241.vibration_z = (float) -2.3437292E38F;
            p241.clipping_0 = (uint)1058388772U;
            p241.clipping_1 = (uint)3495409897U;
            p241.clipping_2 = (uint)3053877803U;
            CommunicationChannel.instance.send(p241); //===============================
            HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.latitude = (int)335841132;
            p242.longitude = (int) -1719790278;
            p242.altitude = (int)224298651;
            p242.x = (float)5.615932E37F;
            p242.y = (float)8.843817E37F;
            p242.z = (float)7.363361E37F;
            p242.q_SET(new float[4], 0);
            p242.approach_x = (float)2.276558E38F;
            p242.approach_y = (float)2.1716867E38F;
            p242.approach_z = (float) -4.9322713E37F;
            p242.time_usec_SET((ulong)6109623772876520890L, PH);
            CommunicationChannel.instance.send(p242); //===============================
            SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.target_system = (byte)(byte)45;
            p243.latitude = (int)613239420;
            p243.longitude = (int)1302094291;
            p243.altitude = (int) -1841003653;
            p243.x = (float) -1.7174613E38F;
            p243.y = (float) -2.2924068E38F;
            p243.z = (float) -1.4886463E38F;
            p243.q_SET(new float[4], 0);
            p243.approach_x = (float) -2.1914726E38F;
            p243.approach_y = (float) -1.5652149E38F;
            p243.approach_z = (float) -3.027482E38F;
            p243.time_usec_SET((ulong)2730868085908668202L, PH);
            CommunicationChannel.instance.send(p243); //===============================
            MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.message_id = (ushort)(ushort)1500;
            p244.interval_us = (int)2103929374;
            CommunicationChannel.instance.send(p244); //===============================
            EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.vtol_state = MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_MC;
            p245.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR;
            CommunicationChannel.instance.send(p245); //===============================
            ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.ICAO_address = (uint)3862019632U;
            p246.lat = (int) -224473248;
            p246.lon = (int) -1286850326;
            p246.altitude_type = ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
            p246.altitude = (int) -1439782138;
            p246.heading = (ushort)(ushort)40335;
            p246.hor_velocity = (ushort)(ushort)41831;
            p246.ver_velocity = (short)(short) -5736;
            p246.callsign_SET("DEMO", PH);
            p246.emitter_type = ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UAV;
            p246.tslc = (byte)(byte)239;
            p246.flags = (ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN);
            p246.squawk = (ushort)(ushort)36776;
            CommunicationChannel.instance.send(p246); //===============================
            COLLISION p247 = CommunicationChannel.new_COLLISION();
            PH.setPack(p247);
            p247.src_ = MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB;
            p247.id = (uint)1603907049U;
            p247.action = MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_PERPENDICULAR;
            p247.threat_level = MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW;
            p247.time_to_minimum_delta = (float) -3.1904731E35F;
            p247.altitude_minimum_delta = (float)3.208769E38F;
            p247.horizontal_minimum_delta = (float) -2.0972537E38F;
            CommunicationChannel.instance.send(p247); //===============================
            V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.target_network = (byte)(byte)123;
            p248.target_system = (byte)(byte)174;
            p248.target_component = (byte)(byte)234;
            p248.message_type = (ushort)(ushort)64436;
            p248.payload_SET(new byte[249], 0);
            CommunicationChannel.instance.send(p248); //===============================
            MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.address = (ushort)(ushort)38120;
            p249.ver = (byte)(byte)21;
            p249.type = (byte)(byte)151;
            p249.value_SET(new sbyte[32], 0);
            CommunicationChannel.instance.send(p249); //===============================
            DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.name_SET("DEMO", PH);
            p250.time_usec = (ulong)2673771539042560142L;
            p250.x = (float)2.7355433E38F;
            p250.y = (float) -5.5610437E37F;
            p250.z = (float) -2.3814496E38F;
            CommunicationChannel.instance.send(p250); //===============================
            NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.time_boot_ms = (uint)45783014U;
            p251.name_SET("DEMO", PH);
            p251.value = (float) -2.6444824E38F;
            CommunicationChannel.instance.send(p251); //===============================
            NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.time_boot_ms = (uint)59843525U;
            p252.name_SET("DEMO", PH);
            p252.value = (int) -161709093;
            CommunicationChannel.instance.send(p252); //===============================
            STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.severity = MAV_SEVERITY.MAV_SEVERITY_CRITICAL;
            p253.text_SET("DEMO", PH);
            CommunicationChannel.instance.send(p253); //===============================
            DEBUG p254 = CommunicationChannel.new_DEBUG();
            PH.setPack(p254);
            p254.time_boot_ms = (uint)3322519681U;
            p254.ind = (byte)(byte)97;
            p254.value = (float) -1.9454212E38F;
            CommunicationChannel.instance.send(p254); //===============================
            SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.target_system = (byte)(byte)33;
            p256.target_component = (byte)(byte)147;
            p256.secret_key_SET(new byte[32], 0);
            p256.initial_timestamp = (ulong)1953842922580999386L;
            CommunicationChannel.instance.send(p256); //===============================
            BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.time_boot_ms = (uint)3139360495U;
            p257.last_change_ms = (uint)4074744036U;
            p257.state = (byte)(byte)230;
            CommunicationChannel.instance.send(p257); //===============================
            PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_system = (byte)(byte)102;
            p258.target_component = (byte)(byte)2;
            p258.tune_SET("DEMO", PH);
            CommunicationChannel.instance.send(p258); //===============================
            CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.time_boot_ms = (uint)2589437128U;
            p259.vendor_name_SET(new byte[32], 0);
            p259.model_name_SET(new byte[32], 0);
            p259.firmware_version = (uint)596011307U;
            p259.focal_length = (float)6.6644007E37F;
            p259.sensor_size_h = (float) -8.959581E37F;
            p259.sensor_size_v = (float) -2.3191217E38F;
            p259.resolution_h = (ushort)(ushort)50978;
            p259.resolution_v = (ushort)(ushort)65083;
            p259.lens_id = (byte)(byte)229;
            p259.flags = (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE |
                          CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_MODES |
                          CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO);
            p259.cam_definition_version = (ushort)(ushort)51826;
            p259.cam_definition_uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p259); //===============================
            CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.time_boot_ms = (uint)707064847U;
            p260.mode_id = CAMERA_MODE.CAMERA_MODE_IMAGE;
            CommunicationChannel.instance.send(p260); //===============================
            STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.time_boot_ms = (uint)2997732834U;
            p261.storage_id = (byte)(byte)243;
            p261.storage_count = (byte)(byte)233;
            p261.status = (byte)(byte)244;
            p261.total_capacity = (float) -2.0600182E38F;
            p261.used_capacity = (float)2.813132E38F;
            p261.available_capacity = (float) -9.158003E37F;
            p261.read_speed = (float) -2.0271155E37F;
            p261.write_speed = (float)9.795432E37F;
            CommunicationChannel.instance.send(p261); //===============================
            CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.time_boot_ms = (uint)3581880365U;
            p262.image_status = (byte)(byte)69;
            p262.video_status = (byte)(byte)140;
            p262.image_interval = (float)3.1416096E38F;
            p262.recording_time_ms = (uint)2640020080U;
            p262.available_capacity = (float) -1.6213878E38F;
            CommunicationChannel.instance.send(p262); //===============================
            CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.time_boot_ms = (uint)3958770852U;
            p263.time_utc = (ulong)8493486457592720044L;
            p263.camera_id = (byte)(byte)9;
            p263.lat = (int)1543055875;
            p263.lon = (int) -807450646;
            p263.alt = (int) -1230156280;
            p263.relative_alt = (int)1606619154;
            p263.q_SET(new float[4], 0);
            p263.image_index = (int)93406904;
            p263.capture_result = (sbyte)(sbyte)62;
            p263.file_url_SET("DEMO", PH);
            CommunicationChannel.instance.send(p263); //===============================
            FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.time_boot_ms = (uint)1900168671U;
            p264.arming_time_utc = (ulong)241512822052844246L;
            p264.takeoff_time_utc = (ulong)5098223529723750588L;
            p264.flight_uuid = (ulong)8721714935223505643L;
            CommunicationChannel.instance.send(p264); //===============================
            MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.time_boot_ms = (uint)3818366745U;
            p265.roll = (float) -1.0946417E38F;
            p265.pitch = (float)1.218896E38F;
            p265.yaw = (float) -3.0071153E38F;
            CommunicationChannel.instance.send(p265); //===============================
            LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.target_system = (byte)(byte)113;
            p266.target_component = (byte)(byte)38;
            p266.sequence = (ushort)(ushort)10842;
            p266.length = (byte)(byte)28;
            p266.first_message_offset = (byte)(byte)49;
            p266.data__SET(new byte[249], 0);
            CommunicationChannel.instance.send(p266); //===============================
            LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.target_system = (byte)(byte)5;
            p267.target_component = (byte)(byte)174;
            p267.sequence = (ushort)(ushort)18484;
            p267.length = (byte)(byte)25;
            p267.first_message_offset = (byte)(byte)143;
            p267.data__SET(new byte[249], 0);
            CommunicationChannel.instance.send(p267); //===============================
            LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_system = (byte)(byte)27;
            p268.target_component = (byte)(byte)58;
            p268.sequence = (ushort)(ushort)40962;
            CommunicationChannel.instance.send(p268); //===============================
            VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.camera_id = (byte)(byte)57;
            p269.status = (byte)(byte)101;
            p269.framerate = (float) -3.6726677E37F;
            p269.resolution_h = (ushort)(ushort)39613;
            p269.resolution_v = (ushort)(ushort)56521;
            p269.bitrate = (uint)1855978698U;
            p269.rotation = (ushort)(ushort)41840;
            p269.uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p269); //===============================
            SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.target_system = (byte)(byte)216;
            p270.target_component = (byte)(byte)190;
            p270.camera_id = (byte)(byte)78;
            p270.framerate = (float) -1.4521497E38F;
            p270.resolution_h = (ushort)(ushort)34790;
            p270.resolution_v = (ushort)(ushort)31903;
            p270.bitrate = (uint)2035999312U;
            p270.rotation = (ushort)(ushort)16367;
            p270.uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p270); //===============================
            WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.ssid_SET("DEMO", PH);
            p299.password_SET("DEMO", PH);
            CommunicationChannel.instance.send(p299); //===============================
            PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.version = (ushort)(ushort)3738;
            p300.min_version = (ushort)(ushort)20906;
            p300.max_version = (ushort)(ushort)31610;
            p300.spec_version_hash_SET(new byte[8], 0);
            p300.library_version_hash_SET(new byte[8], 0);
            CommunicationChannel.instance.send(p300); //===============================
            UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.time_usec = (ulong)2946630623892077172L;
            p310.uptime_sec = (uint)2312557632U;
            p310.health = UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR;
            p310.mode = UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE;
            p310.sub_mode = (byte)(byte)78;
            p310.vendor_specific_status_code = (ushort)(ushort)47138;
            CommunicationChannel.instance.send(p310); //===============================
            UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.time_usec = (ulong)5625102830047465603L;
            p311.uptime_sec = (uint)841494461U;
            p311.name_SET("DEMO", PH);
            p311.hw_version_major = (byte)(byte)196;
            p311.hw_version_minor = (byte)(byte)52;
            p311.hw_unique_id_SET(new byte[16], 0);
            p311.sw_version_major = (byte)(byte)248;
            p311.sw_version_minor = (byte)(byte)117;
            p311.sw_vcs_commit = (uint)1321131177U;
            CommunicationChannel.instance.send(p311); //===============================
            PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.target_system = (byte)(byte)246;
            p320.target_component = (byte)(byte)5;
            p320.param_id_SET("DEMO", PH);
            p320.param_index = (short)(short) -16695;
            CommunicationChannel.instance.send(p320); //===============================
            PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_system = (byte)(byte)106;
            p321.target_component = (byte)(byte)48;
            CommunicationChannel.instance.send(p321); //===============================
            PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_id_SET("DEMO", PH);
            p322.param_value_SET("DEMO", PH);
            p322.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32;
            p322.param_count = (ushort)(ushort)55414;
            p322.param_index = (ushort)(ushort)65348;
            CommunicationChannel.instance.send(p322); //===============================
            PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.target_system = (byte)(byte)184;
            p323.target_component = (byte)(byte)146;
            p323.param_id_SET("DEMO", PH);
            p323.param_value_SET("DEMO", PH);
            p323.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32;
            CommunicationChannel.instance.send(p323); //===============================
            PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_id_SET("DEMO", PH);
            p324.param_value_SET("DEMO", PH);
            p324.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64;
            p324.param_result = PARAM_ACK.PARAM_ACK_FAILED;
            CommunicationChannel.instance.send(p324); //===============================
            OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.time_usec = (ulong)5414192338559960281L;
            p330.sensor_type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED;
            p330.distances_SET(new ushort[72], 0);
            p330.increment = (byte)(byte)199;
            p330.min_distance = (ushort)(ushort)22128;
            p330.max_distance = (ushort)(ushort)37004;
            CommunicationChannel.instance.send(p330); //===============================
        }
    }
}

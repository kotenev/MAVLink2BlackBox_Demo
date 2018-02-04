
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
            SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.time_boot_ms = (uint)230736914U;
            p86.target_system = (byte)(byte)202;
            p86.target_component = (byte)(byte)146;
            p86.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT;
            p86.type_mask = (ushort)(ushort)33237;
            p86.lat_int = (int) -1042736998;
            p86.lon_int = (int)1074848867;
            p86.alt = (float)1.296329E38F;
            p86.vx = (float)2.9860242E38F;
            p86.vy = (float) -1.2804594E38F;
            p86.vz = (float)6.0556906E37F;
            p86.afx = (float) -1.7615303E38F;
            p86.afy = (float)1.8791663E38F;
            p86.afz = (float)1.0368185E38F;
            p86.yaw = (float) -2.8754504E38F;
            p86.yaw_rate = (float) -2.1039434E38F;
            CommunicationChannel.instance.send(p86); //===============================
            POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.time_boot_ms = (uint)1155365608U;
            p87.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL;
            p87.type_mask = (ushort)(ushort)34325;
            p87.lat_int = (int)1081203776;
            p87.lon_int = (int)1067258420;
            p87.alt = (float)2.7075415E38F;
            p87.vx = (float)6.240268E36F;
            p87.vy = (float) -1.4696326E38F;
            p87.vz = (float)2.7484653E37F;
            p87.afx = (float) -1.9414258E38F;
            p87.afy = (float) -2.1432866E38F;
            p87.afz = (float) -1.2966027E38F;
            p87.yaw = (float)2.0417714E38F;
            p87.yaw_rate = (float) -2.5516377E38F;
            CommunicationChannel.instance.send(p87); //===============================
            LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.time_boot_ms = (uint)1422902829U;
            p89.x = (float) -3.239717E38F;
            p89.y = (float)2.0918949E38F;
            p89.z = (float) -1.4413291E38F;
            p89.roll = (float)2.4229776E38F;
            p89.pitch = (float) -2.8582367E38F;
            p89.yaw = (float) -1.2298805E38F;
            CommunicationChannel.instance.send(p89); //===============================
            HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.time_usec = (ulong)9186120605112404749L;
            p90.roll = (float) -1.3745113E38F;
            p90.pitch = (float) -2.0341679E38F;
            p90.yaw = (float) -8.697856E37F;
            p90.rollspeed = (float) -2.920129E38F;
            p90.pitchspeed = (float) -4.7085543E37F;
            p90.yawspeed = (float) -3.057886E38F;
            p90.lat = (int)1083198635;
            p90.lon = (int) -846586894;
            p90.alt = (int) -1872794128;
            p90.vx = (short)(short) -22683;
            p90.vy = (short)(short) -17710;
            p90.vz = (short)(short)8996;
            p90.xacc = (short)(short) -3135;
            p90.yacc = (short)(short) -19266;
            p90.zacc = (short)(short)5248;
            CommunicationChannel.instance.send(p90); //===============================
            HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.time_usec = (ulong)9119782064169370312L;
            p91.roll_ailerons = (float)1.0411858E38F;
            p91.pitch_elevator = (float)3.202855E38F;
            p91.yaw_rudder = (float) -1.4744434E38F;
            p91.throttle = (float)2.2025612E38F;
            p91.aux1 = (float)1.9928766E38F;
            p91.aux2 = (float)2.7768359E38F;
            p91.aux3 = (float) -2.2097967E38F;
            p91.aux4 = (float) -2.9491277E38F;
            p91.mode = (MAV_MODE)MAV_MODE.MAV_MODE_PREFLIGHT;
            p91.nav_mode = (byte)(byte)201;
            CommunicationChannel.instance.send(p91); //===============================
            HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.time_usec = (ulong)683095557672679013L;
            p92.chan1_raw = (ushort)(ushort)7108;
            p92.chan2_raw = (ushort)(ushort)33861;
            p92.chan3_raw = (ushort)(ushort)1340;
            p92.chan4_raw = (ushort)(ushort)20626;
            p92.chan5_raw = (ushort)(ushort)27780;
            p92.chan6_raw = (ushort)(ushort)178;
            p92.chan7_raw = (ushort)(ushort)24100;
            p92.chan8_raw = (ushort)(ushort)33301;
            p92.chan9_raw = (ushort)(ushort)10816;
            p92.chan10_raw = (ushort)(ushort)10629;
            p92.chan11_raw = (ushort)(ushort)2192;
            p92.chan12_raw = (ushort)(ushort)55313;
            p92.rssi = (byte)(byte)206;
            CommunicationChannel.instance.send(p92); //===============================
            HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.time_usec = (ulong)5169246745551037511L;
            p93.controls_SET(new float[16], 0);
            p93.mode = (MAV_MODE)MAV_MODE.MAV_MODE_STABILIZE_DISARMED;
            p93.flags = (ulong)8949578305731633795L;
            CommunicationChannel.instance.send(p93); //===============================
            OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.time_usec = (ulong)2875562479480670237L;
            p100.sensor_id = (byte)(byte)120;
            p100.flow_x = (short)(short)31514;
            p100.flow_y = (short)(short) -30848;
            p100.flow_comp_m_x = (float) -2.6030603E38F;
            p100.flow_comp_m_y = (float) -3.0113571E38F;
            p100.quality = (byte)(byte)77;
            p100.ground_distance = (float) -1.5436923E38F;
            p100.flow_rate_x_SET((float)4.260293E37F, PH);
            p100.flow_rate_y_SET((float) -2.0468775E38F, PH);
            CommunicationChannel.instance.send(p100); //===============================
            GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.usec = (ulong)1349034394535792666L;
            p101.x = (float) -8.034402E37F;
            p101.y = (float)1.049739E38F;
            p101.z = (float) -2.8490652E38F;
            p101.roll = (float)5.865819E37F;
            p101.pitch = (float)7.638877E37F;
            p101.yaw = (float)2.4020685E38F;
            CommunicationChannel.instance.send(p101); //===============================
            VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.usec = (ulong)4455761975981800574L;
            p102.x = (float) -1.9004372E38F;
            p102.y = (float)3.0610705E38F;
            p102.z = (float) -2.070625E38F;
            p102.roll = (float) -2.4705896E38F;
            p102.pitch = (float)3.0874249E38F;
            p102.yaw = (float) -2.2205598E37F;
            CommunicationChannel.instance.send(p102); //===============================
            VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.usec = (ulong)9138912244967703746L;
            p103.x = (float)2.8468333E38F;
            p103.y = (float) -2.3902584E38F;
            p103.z = (float)9.875591E37F;
            CommunicationChannel.instance.send(p103); //===============================
            VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.usec = (ulong)8130589608789253281L;
            p104.x = (float)2.5376071E38F;
            p104.y = (float) -3.3764715E38F;
            p104.z = (float) -3.2876488E38F;
            p104.roll = (float)2.1134952E38F;
            p104.pitch = (float)8.045457E36F;
            p104.yaw = (float) -2.3482274E38F;
            CommunicationChannel.instance.send(p104); //===============================
            HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.time_usec = (ulong)4391264093285387453L;
            p105.xacc = (float)2.3017695E38F;
            p105.yacc = (float) -2.5914125E36F;
            p105.zacc = (float)2.5620058E38F;
            p105.xgyro = (float)1.5906983E38F;
            p105.ygyro = (float) -3.0683933E38F;
            p105.zgyro = (float)1.8932688E38F;
            p105.xmag = (float) -9.077237E37F;
            p105.ymag = (float)3.3637825E38F;
            p105.zmag = (float) -2.3324098E38F;
            p105.abs_pressure = (float) -6.313796E37F;
            p105.diff_pressure = (float) -3.2467505E36F;
            p105.pressure_alt = (float)1.3018814E38F;
            p105.temperature = (float) -1.8952652E38F;
            p105.fields_updated = (ushort)(ushort)5881;
            CommunicationChannel.instance.send(p105); //===============================
            OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.time_usec = (ulong)9109843254648531257L;
            p106.sensor_id = (byte)(byte)191;
            p106.integration_time_us = (uint)1756739415U;
            p106.integrated_x = (float)8.25829E37F;
            p106.integrated_y = (float) -1.566798E36F;
            p106.integrated_xgyro = (float)3.3551704E38F;
            p106.integrated_ygyro = (float)1.3154326E38F;
            p106.integrated_zgyro = (float) -7.436008E37F;
            p106.temperature = (short)(short) -10502;
            p106.quality = (byte)(byte)124;
            p106.time_delta_distance_us = (uint)2010750446U;
            p106.distance = (float)1.4477647E38F;
            CommunicationChannel.instance.send(p106); //===============================
            HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.time_usec = (ulong)3380367450485542548L;
            p107.xacc = (float) -8.652621E37F;
            p107.yacc = (float)1.9140766E38F;
            p107.zacc = (float) -2.838016E37F;
            p107.xgyro = (float)1.4434418E38F;
            p107.ygyro = (float) -1.64025E38F;
            p107.zgyro = (float)3.3479417E38F;
            p107.xmag = (float) -1.4223074E37F;
            p107.ymag = (float) -2.7224273E38F;
            p107.zmag = (float) -1.5409544E38F;
            p107.abs_pressure = (float)1.5421057E38F;
            p107.diff_pressure = (float) -7.658996E37F;
            p107.pressure_alt = (float) -1.0855642E38F;
            p107.temperature = (float)3.3300106E38F;
            p107.fields_updated = (uint)2299421648U;
            CommunicationChannel.instance.send(p107); //===============================
            SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.q1 = (float)2.4333107E38F;
            p108.q2 = (float) -7.609793E37F;
            p108.q3 = (float) -1.848443E38F;
            p108.q4 = (float) -1.3798435E38F;
            p108.roll = (float)2.1873873E38F;
            p108.pitch = (float) -2.1048527E38F;
            p108.yaw = (float) -3.2876443E38F;
            p108.xacc = (float) -1.7815813E37F;
            p108.yacc = (float) -3.0628098E38F;
            p108.zacc = (float) -3.4459555E37F;
            p108.xgyro = (float)8.914643E37F;
            p108.ygyro = (float)1.6750105E38F;
            p108.zgyro = (float)6.249709E37F;
            p108.lat = (float)2.2943277E38F;
            p108.lon = (float)1.2432618E38F;
            p108.alt = (float)1.3963069E38F;
            p108.std_dev_horz = (float)4.7895085E37F;
            p108.std_dev_vert = (float) -4.696751E37F;
            p108.vn = (float) -2.681859E38F;
            p108.ve = (float) -1.0312011E38F;
            p108.vd = (float) -3.1578517E38F;
            CommunicationChannel.instance.send(p108); //===============================
            RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.rssi = (byte)(byte)251;
            p109.remrssi = (byte)(byte)46;
            p109.txbuf = (byte)(byte)179;
            p109.noise = (byte)(byte)150;
            p109.remnoise = (byte)(byte)88;
            p109.rxerrors = (ushort)(ushort)16831;
            p109.fixed_ = (ushort)(ushort)37742;
            CommunicationChannel.instance.send(p109); //===============================
            FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_network = (byte)(byte)105;
            p110.target_system = (byte)(byte)229;
            p110.target_component = (byte)(byte)219;
            p110.payload_SET(new byte[251], 0);
            CommunicationChannel.instance.send(p110); //===============================
            TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.tc1 = (long)9128456684002088027L;
            p111.ts1 = (long) -405568294886362367L;
            CommunicationChannel.instance.send(p111); //===============================
            CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.time_usec = (ulong)5208306860771390941L;
            p112.seq = (uint)3396267090U;
            CommunicationChannel.instance.send(p112); //===============================
            HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.time_usec = (ulong)1391100405374619703L;
            p113.fix_type = (byte)(byte)218;
            p113.lat = (int) -1615764642;
            p113.lon = (int) -2007717334;
            p113.alt = (int) -700136230;
            p113.eph = (ushort)(ushort)7129;
            p113.epv = (ushort)(ushort)48262;
            p113.vel = (ushort)(ushort)5776;
            p113.vn = (short)(short)24078;
            p113.ve = (short)(short) -8752;
            p113.vd = (short)(short) -27398;
            p113.cog = (ushort)(ushort)48799;
            p113.satellites_visible = (byte)(byte)242;
            CommunicationChannel.instance.send(p113); //===============================
            HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.time_usec = (ulong)6689934706017933375L;
            p114.sensor_id = (byte)(byte)28;
            p114.integration_time_us = (uint)3203722114U;
            p114.integrated_x = (float)2.544131E38F;
            p114.integrated_y = (float)3.2640646E38F;
            p114.integrated_xgyro = (float)2.8476807E38F;
            p114.integrated_ygyro = (float) -1.2398297E37F;
            p114.integrated_zgyro = (float)8.5049493E37F;
            p114.temperature = (short)(short) -2118;
            p114.quality = (byte)(byte)42;
            p114.time_delta_distance_us = (uint)1807213724U;
            p114.distance = (float) -5.7206415E37F;
            CommunicationChannel.instance.send(p114); //===============================
            HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.time_usec = (ulong)1252842823846681295L;
            p115.attitude_quaternion_SET(new float[4], 0);
            p115.rollspeed = (float)3.1946359E38F;
            p115.pitchspeed = (float) -6.3167815E37F;
            p115.yawspeed = (float)1.924381E38F;
            p115.lat = (int)1986731103;
            p115.lon = (int) -638204722;
            p115.alt = (int)2028666096;
            p115.vx = (short)(short) -26282;
            p115.vy = (short)(short)13646;
            p115.vz = (short)(short)14146;
            p115.ind_airspeed = (ushort)(ushort)14572;
            p115.true_airspeed = (ushort)(ushort)56553;
            p115.xacc = (short)(short) -19510;
            p115.yacc = (short)(short)17606;
            p115.zacc = (short)(short)31295;
            CommunicationChannel.instance.send(p115); //===============================
            SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.time_boot_ms = (uint)1432053347U;
            p116.xacc = (short)(short)31421;
            p116.yacc = (short)(short) -2989;
            p116.zacc = (short)(short) -3772;
            p116.xgyro = (short)(short) -16506;
            p116.ygyro = (short)(short) -27508;
            p116.zgyro = (short)(short)23453;
            p116.xmag = (short)(short)15759;
            p116.ymag = (short)(short) -6340;
            p116.zmag = (short)(short) -13290;
            CommunicationChannel.instance.send(p116); //===============================
            LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.target_system = (byte)(byte)9;
            p117.target_component = (byte)(byte)118;
            p117.start = (ushort)(ushort)55762;
            p117.end = (ushort)(ushort)22764;
            CommunicationChannel.instance.send(p117); //===============================
            LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.id = (ushort)(ushort)32425;
            p118.num_logs = (ushort)(ushort)53187;
            p118.last_log_num = (ushort)(ushort)13890;
            p118.time_utc = (uint)2525662181U;
            p118.size = (uint)1852757793U;
            CommunicationChannel.instance.send(p118); //===============================
            LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.target_system = (byte)(byte)185;
            p119.target_component = (byte)(byte)185;
            p119.id = (ushort)(ushort)31661;
            p119.ofs = (uint)1485182793U;
            p119.count = (uint)1239088109U;
            CommunicationChannel.instance.send(p119); //===============================
            LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.id = (ushort)(ushort)49956;
            p120.ofs = (uint)3178358051U;
            p120.count = (byte)(byte)144;
            p120.data__SET(new byte[90], 0);
            CommunicationChannel.instance.send(p120); //===============================
            LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_system = (byte)(byte)66;
            p121.target_component = (byte)(byte)197;
            CommunicationChannel.instance.send(p121); //===============================
            LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_system = (byte)(byte)193;
            p122.target_component = (byte)(byte)159;
            CommunicationChannel.instance.send(p122); //===============================
            GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.target_system = (byte)(byte)95;
            p123.target_component = (byte)(byte)195;
            p123.len = (byte)(byte)77;
            p123.data__SET(new byte[110], 0);
            CommunicationChannel.instance.send(p123); //===============================
            GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.time_usec = (ulong)442056434348178712L;
            p124.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX;
            p124.lat = (int) -1207757580;
            p124.lon = (int)1107084567;
            p124.alt = (int)592301017;
            p124.eph = (ushort)(ushort)6929;
            p124.epv = (ushort)(ushort)42280;
            p124.vel = (ushort)(ushort)37019;
            p124.cog = (ushort)(ushort)22204;
            p124.satellites_visible = (byte)(byte)145;
            p124.dgps_numch = (byte)(byte)89;
            p124.dgps_age = (uint)768340211U;
            CommunicationChannel.instance.send(p124); //===============================
            POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vcc = (ushort)(ushort)56;
            p125.Vservo = (ushort)(ushort)4270;
            p125.flags = (MAV_POWER_STATUS)MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID;
            CommunicationChannel.instance.send(p125); //===============================
            SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.device = (SERIAL_CONTROL_DEV)SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS2;
            p126.flags = (SERIAL_CONTROL_FLAG)SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY;
            p126.timeout = (ushort)(ushort)17580;
            p126.baudrate = (uint)2615422520U;
            p126.count = (byte)(byte)225;
            p126.data__SET(new byte[70], 0);
            CommunicationChannel.instance.send(p126); //===============================
            GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.time_last_baseline_ms = (uint)1925486820U;
            p127.rtk_receiver_id = (byte)(byte)219;
            p127.wn = (ushort)(ushort)44267;
            p127.tow = (uint)4203358508U;
            p127.rtk_health = (byte)(byte)230;
            p127.rtk_rate = (byte)(byte)71;
            p127.nsats = (byte)(byte)131;
            p127.baseline_coords_type = (byte)(byte)130;
            p127.baseline_a_mm = (int) -1278796008;
            p127.baseline_b_mm = (int)1109513172;
            p127.baseline_c_mm = (int) -1369642488;
            p127.accuracy = (uint)2965889452U;
            p127.iar_num_hypotheses = (int) -781302641;
            CommunicationChannel.instance.send(p127); //===============================
            GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.time_last_baseline_ms = (uint)1385070799U;
            p128.rtk_receiver_id = (byte)(byte)13;
            p128.wn = (ushort)(ushort)16660;
            p128.tow = (uint)3568498593U;
            p128.rtk_health = (byte)(byte)29;
            p128.rtk_rate = (byte)(byte)25;
            p128.nsats = (byte)(byte)184;
            p128.baseline_coords_type = (byte)(byte)178;
            p128.baseline_a_mm = (int) -1309775337;
            p128.baseline_b_mm = (int) -1561354682;
            p128.baseline_c_mm = (int)1255540111;
            p128.accuracy = (uint)3577316440U;
            p128.iar_num_hypotheses = (int)534075498;
            CommunicationChannel.instance.send(p128); //===============================
            SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.time_boot_ms = (uint)1577134742U;
            p129.xacc = (short)(short) -23842;
            p129.yacc = (short)(short)29335;
            p129.zacc = (short)(short) -6621;
            p129.xgyro = (short)(short) -21843;
            p129.ygyro = (short)(short)32248;
            p129.zgyro = (short)(short) -3750;
            p129.xmag = (short)(short) -4001;
            p129.ymag = (short)(short)7299;
            p129.zmag = (short)(short)9712;
            CommunicationChannel.instance.send(p129); //===============================
            DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.type = (byte)(byte)55;
            p130.size = (uint)1623555000U;
            p130.width = (ushort)(ushort)19746;
            p130.height = (ushort)(ushort)46569;
            p130.packets = (ushort)(ushort)21731;
            p130.payload = (byte)(byte)224;
            p130.jpg_quality = (byte)(byte)58;
            CommunicationChannel.instance.send(p130); //===============================
            ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.seqnr = (ushort)(ushort)44970;
            p131.data__SET(new byte[253], 0);
            CommunicationChannel.instance.send(p131); //===============================
            DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.time_boot_ms = (uint)2350119747U;
            p132.min_distance = (ushort)(ushort)6401;
            p132.max_distance = (ushort)(ushort)32249;
            p132.current_distance = (ushort)(ushort)31201;
            p132.type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN;
            p132.id = (byte)(byte)61;
            p132.orientation = (MAV_SENSOR_ORIENTATION)MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_90;
            p132.covariance = (byte)(byte)106;
            CommunicationChannel.instance.send(p132); //===============================
            TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.lat = (int)1677178475;
            p133.lon = (int) -559877932;
            p133.grid_spacing = (ushort)(ushort)54477;
            p133.mask = (ulong)8782674926144947892L;
            CommunicationChannel.instance.send(p133); //===============================
            TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.lat = (int)93274712;
            p134.lon = (int) -165772346;
            p134.grid_spacing = (ushort)(ushort)46190;
            p134.gridbit = (byte)(byte)105;
            p134.data__SET(new short[16], 0);
            CommunicationChannel.instance.send(p134); //===============================
            TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lat = (int)1818330310;
            p135.lon = (int)1332829916;
            CommunicationChannel.instance.send(p135); //===============================
            TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.lat = (int)1798445517;
            p136.lon = (int)703887654;
            p136.spacing = (ushort)(ushort)48142;
            p136.terrain_height = (float)2.3772323E38F;
            p136.current_height = (float) -1.7956042E38F;
            p136.pending = (ushort)(ushort)3034;
            p136.loaded = (ushort)(ushort)30135;
            CommunicationChannel.instance.send(p136); //===============================
            SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.time_boot_ms = (uint)3973351646U;
            p137.press_abs = (float)1.2359159E38F;
            p137.press_diff = (float)1.3265715E38F;
            p137.temperature = (short)(short) -17164;
            CommunicationChannel.instance.send(p137); //===============================
            ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.time_usec = (ulong)4048331517958435878L;
            p138.q_SET(new float[4], 0);
            p138.x = (float)2.3710244E38F;
            p138.y = (float)3.2828258E38F;
            p138.z = (float)3.0997542E38F;
            CommunicationChannel.instance.send(p138); //===============================
            SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.time_usec = (ulong)7239811349039778823L;
            p139.group_mlx = (byte)(byte)251;
            p139.target_system = (byte)(byte)13;
            p139.target_component = (byte)(byte)91;
            p139.controls_SET(new float[8], 0);
            CommunicationChannel.instance.send(p139); //===============================
            ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.time_usec = (ulong)5338489911272170515L;
            p140.group_mlx = (byte)(byte)186;
            p140.controls_SET(new float[8], 0);
            CommunicationChannel.instance.send(p140); //===============================
            ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.time_usec = (ulong)4171762324259205224L;
            p141.altitude_monotonic = (float) -1.0461679E38F;
            p141.altitude_amsl = (float) -1.6046032E38F;
            p141.altitude_local = (float)1.1880843E38F;
            p141.altitude_relative = (float) -8.68749E37F;
            p141.altitude_terrain = (float) -2.3510442E38F;
            p141.bottom_clearance = (float)2.9223598E38F;
            CommunicationChannel.instance.send(p141); //===============================
            RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.request_id = (byte)(byte)239;
            p142.uri_type = (byte)(byte)38;
            p142.uri_SET(new byte[120], 0);
            p142.transfer_type = (byte)(byte)123;
            p142.storage_SET(new byte[120], 0);
            CommunicationChannel.instance.send(p142); //===============================
            SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.time_boot_ms = (uint)3702002937U;
            p143.press_abs = (float)1.2091108E38F;
            p143.press_diff = (float)1.3201845E38F;
            p143.temperature = (short)(short) -12778;
            CommunicationChannel.instance.send(p143); //===============================
            FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.timestamp = (ulong)6532051638784524079L;
            p144.est_capabilities = (byte)(byte)190;
            p144.lat = (int) -1458804249;
            p144.lon = (int)417638855;
            p144.alt = (float) -2.2820729E38F;
            p144.vel_SET(new float[3], 0);
            p144.acc_SET(new float[3], 0);
            p144.attitude_q_SET(new float[4], 0);
            p144.rates_SET(new float[3], 0);
            p144.position_cov_SET(new float[3], 0);
            p144.custom_state = (ulong)6865475281793663628L;
            CommunicationChannel.instance.send(p144); //===============================
            CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.time_usec = (ulong)4985838794070396689L;
            p146.x_acc = (float)3.7420922E36F;
            p146.y_acc = (float)1.5474067E38F;
            p146.z_acc = (float)3.056869E38F;
            p146.x_vel = (float) -1.225939E37F;
            p146.y_vel = (float) -2.235682E38F;
            p146.z_vel = (float) -1.463121E38F;
            p146.x_pos = (float) -3.2402348E38F;
            p146.y_pos = (float) -3.3221232E38F;
            p146.z_pos = (float)8.923293E37F;
            p146.airspeed = (float) -1.4100165E38F;
            p146.vel_variance_SET(new float[3], 0);
            p146.pos_variance_SET(new float[3], 0);
            p146.q_SET(new float[4], 0);
            p146.roll_rate = (float) -2.7134016E38F;
            p146.pitch_rate = (float)6.0242985E37F;
            p146.yaw_rate = (float)2.6895804E38F;
            CommunicationChannel.instance.send(p146); //===============================
            BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.id = (byte)(byte)165;
            p147.battery_function = (MAV_BATTERY_FUNCTION)MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_UNKNOWN;
            p147.type = (MAV_BATTERY_TYPE)MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIFE;
            p147.temperature = (short)(short)5442;
            p147.voltages_SET(new ushort[10], 0);
            p147.current_battery = (short)(short) -31387;
            p147.current_consumed = (int)929068030;
            p147.energy_consumed = (int)2061811220;
            p147.battery_remaining = (sbyte)(sbyte) - 5;
            CommunicationChannel.instance.send(p147); //===============================
            AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY)MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION;
            p148.flight_sw_version = (uint)1873278634U;
            p148.middleware_sw_version = (uint)617900417U;
            p148.os_sw_version = (uint)65931682U;
            p148.board_version = (uint)3293579232U;
            p148.flight_custom_version_SET(new byte[8], 0);
            p148.middleware_custom_version_SET(new byte[8], 0);
            p148.os_custom_version_SET(new byte[8], 0);
            p148.vendor_id = (ushort)(ushort)47815;
            p148.product_id = (ushort)(ushort)65038;
            p148.uid = (ulong)5107960873546029394L;
            p148.uid2_SET(new byte[18], 0, PH);
            CommunicationChannel.instance.send(p148); //===============================
            LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.time_usec = (ulong)9220314495447706570L;
            p149.target_num = (byte)(byte)146;
            p149.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_INT;
            p149.angle_x = (float) -1.9622383E38F;
            p149.angle_y = (float)2.8951397E38F;
            p149.distance = (float) -2.0343782E37F;
            p149.size_x = (float)8.735998E37F;
            p149.size_y = (float) -2.2591817E38F;
            p149.x_SET((float) -1.6449071E38F, PH);
            p149.y_SET((float)7.046343E37F, PH);
            p149.z_SET((float) -5.861456E37F, PH);
            p149.q_SET(new float[4], 0, PH);
            p149.type = (LANDING_TARGET_TYPE)LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON;
            p149.position_valid_SET((byte)(byte)245, PH);
            CommunicationChannel.instance.send(p149); //===============================
            FLEXIFUNCTION_SET p150 = CommunicationChannel.new_FLEXIFUNCTION_SET();
            PH.setPack(p150);
            p150.target_system = (byte)(byte)39;
            p150.target_component = (byte)(byte)209;
            CommunicationChannel.instance.send(p150); //===============================
            FLEXIFUNCTION_READ_REQ p151 = CommunicationChannel.new_FLEXIFUNCTION_READ_REQ();
            PH.setPack(p151);
            p151.target_system = (byte)(byte)98;
            p151.target_component = (byte)(byte)64;
            p151.read_req_type = (short)(short)2258;
            p151.data_index = (short)(short) -19233;
            CommunicationChannel.instance.send(p151); //===============================
            FLEXIFUNCTION_BUFFER_FUNCTION p152 = CommunicationChannel.new_FLEXIFUNCTION_BUFFER_FUNCTION();
            PH.setPack(p152);
            p152.target_system = (byte)(byte)217;
            p152.target_component = (byte)(byte)144;
            p152.func_index = (ushort)(ushort)55851;
            p152.func_count = (ushort)(ushort)16716;
            p152.data_address = (ushort)(ushort)59087;
            p152.data_size = (ushort)(ushort)60992;
            p152.data__SET(new sbyte[48], 0);
            CommunicationChannel.instance.send(p152); //===============================
            FLEXIFUNCTION_BUFFER_FUNCTION_ACK p153 = CommunicationChannel.new_FLEXIFUNCTION_BUFFER_FUNCTION_ACK();
            PH.setPack(p153);
            p153.target_system = (byte)(byte)35;
            p153.target_component = (byte)(byte)83;
            p153.func_index = (ushort)(ushort)15718;
            p153.result = (ushort)(ushort)41800;
            CommunicationChannel.instance.send(p153); //===============================
            FLEXIFUNCTION_DIRECTORY p155 = CommunicationChannel.new_FLEXIFUNCTION_DIRECTORY();
            PH.setPack(p155);
            p155.target_system = (byte)(byte)117;
            p155.target_component = (byte)(byte)200;
            p155.directory_type = (byte)(byte)16;
            p155.start_index = (byte)(byte)139;
            p155.count = (byte)(byte)189;
            p155.directory_data_SET(new sbyte[48], 0);
            CommunicationChannel.instance.send(p155); //===============================
            FLEXIFUNCTION_DIRECTORY_ACK p156 = CommunicationChannel.new_FLEXIFUNCTION_DIRECTORY_ACK();
            PH.setPack(p156);
            p156.target_system = (byte)(byte)118;
            p156.target_component = (byte)(byte)114;
            p156.directory_type = (byte)(byte)240;
            p156.start_index = (byte)(byte)226;
            p156.count = (byte)(byte)65;
            p156.result = (ushort)(ushort)57511;
            CommunicationChannel.instance.send(p156); //===============================
            FLEXIFUNCTION_COMMAND p157 = CommunicationChannel.new_FLEXIFUNCTION_COMMAND();
            PH.setPack(p157);
            p157.target_system = (byte)(byte)28;
            p157.target_component = (byte)(byte)153;
            p157.command_type = (byte)(byte)171;
            CommunicationChannel.instance.send(p157); //===============================
            FLEXIFUNCTION_COMMAND_ACK p158 = CommunicationChannel.new_FLEXIFUNCTION_COMMAND_ACK();
            PH.setPack(p158);
            p158.command_type = (ushort)(ushort)54409;
            p158.result = (ushort)(ushort)62275;
            CommunicationChannel.instance.send(p158); //===============================
            SERIAL_UDB_EXTRA_F2_A p170 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F2_A();
            PH.setPack(p170);
            p170.sue_time = (uint)2158336079U;
            p170.sue_status = (byte)(byte)78;
            p170.sue_latitude = (int) -373249643;
            p170.sue_longitude = (int)328929728;
            p170.sue_altitude = (int)88017013;
            p170.sue_waypoint_index = (ushort)(ushort)14577;
            p170.sue_rmat0 = (short)(short) -1937;
            p170.sue_rmat1 = (short)(short) -14066;
            p170.sue_rmat2 = (short)(short) -5332;
            p170.sue_rmat3 = (short)(short) -9540;
            p170.sue_rmat4 = (short)(short)22817;
            p170.sue_rmat5 = (short)(short)7641;
            p170.sue_rmat6 = (short)(short) -11306;
            p170.sue_rmat7 = (short)(short) -13602;
            p170.sue_rmat8 = (short)(short)3799;
            p170.sue_cog = (ushort)(ushort)8558;
            p170.sue_sog = (short)(short) -26030;
            p170.sue_cpu_load = (ushort)(ushort)11639;
            p170.sue_air_speed_3DIMU = (ushort)(ushort)58543;
            p170.sue_estimated_wind_0 = (short)(short)0;
            p170.sue_estimated_wind_1 = (short)(short) -29151;
            p170.sue_estimated_wind_2 = (short)(short) -4622;
            p170.sue_magFieldEarth0 = (short)(short)588;
            p170.sue_magFieldEarth1 = (short)(short) -7646;
            p170.sue_magFieldEarth2 = (short)(short) -20313;
            p170.sue_svs = (short)(short) -1826;
            p170.sue_hdop = (short)(short)9237;
            CommunicationChannel.instance.send(p170); //===============================
            SERIAL_UDB_EXTRA_F2_B p171 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F2_B();
            PH.setPack(p171);
            p171.sue_time = (uint)3162389772U;
            p171.sue_pwm_input_1 = (short)(short)27214;
            p171.sue_pwm_input_2 = (short)(short) -23766;
            p171.sue_pwm_input_3 = (short)(short) -21187;
            p171.sue_pwm_input_4 = (short)(short)14681;
            p171.sue_pwm_input_5 = (short)(short) -30205;
            p171.sue_pwm_input_6 = (short)(short)20115;
            p171.sue_pwm_input_7 = (short)(short)21887;
            p171.sue_pwm_input_8 = (short)(short) -20155;
            p171.sue_pwm_input_9 = (short)(short)10761;
            p171.sue_pwm_input_10 = (short)(short)3748;
            p171.sue_pwm_input_11 = (short)(short)28167;
            p171.sue_pwm_input_12 = (short)(short)26300;
            p171.sue_pwm_output_1 = (short)(short) -18459;
            p171.sue_pwm_output_2 = (short)(short) -17735;
            p171.sue_pwm_output_3 = (short)(short) -7774;
            p171.sue_pwm_output_4 = (short)(short)15444;
            p171.sue_pwm_output_5 = (short)(short) -13709;
            p171.sue_pwm_output_6 = (short)(short) -11277;
            p171.sue_pwm_output_7 = (short)(short)26277;
            p171.sue_pwm_output_8 = (short)(short) -14622;
            p171.sue_pwm_output_9 = (short)(short)29311;
            p171.sue_pwm_output_10 = (short)(short)13947;
            p171.sue_pwm_output_11 = (short)(short) -26149;
            p171.sue_pwm_output_12 = (short)(short) -25418;
            p171.sue_imu_location_x = (short)(short) -16659;
            p171.sue_imu_location_y = (short)(short) -23221;
            p171.sue_imu_location_z = (short)(short) -1686;
            p171.sue_location_error_earth_x = (short)(short) -13613;
            p171.sue_location_error_earth_y = (short)(short) -12046;
            p171.sue_location_error_earth_z = (short)(short)21242;
            p171.sue_flags = (uint)3536726603U;
            p171.sue_osc_fails = (short)(short) -20302;
            p171.sue_imu_velocity_x = (short)(short)4817;
            p171.sue_imu_velocity_y = (short)(short) -718;
            p171.sue_imu_velocity_z = (short)(short) -16865;
            p171.sue_waypoint_goal_x = (short)(short) -32594;
            p171.sue_waypoint_goal_y = (short)(short) -3973;
            p171.sue_waypoint_goal_z = (short)(short)17030;
            p171.sue_aero_x = (short)(short)5002;
            p171.sue_aero_y = (short)(short)7073;
            p171.sue_aero_z = (short)(short)14158;
            p171.sue_barom_temp = (short)(short)19567;
            p171.sue_barom_press = (int) -1569065587;
            p171.sue_barom_alt = (int)1403816332;
            p171.sue_bat_volt = (short)(short)21722;
            p171.sue_bat_amp = (short)(short)4591;
            p171.sue_bat_amp_hours = (short)(short) -31267;
            p171.sue_desired_height = (short)(short) -29977;
            p171.sue_memory_stack_free = (short)(short)27037;
            CommunicationChannel.instance.send(p171); //===============================
            SERIAL_UDB_EXTRA_F4 p172 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F4();
            PH.setPack(p172);
            p172.sue_ROLL_STABILIZATION_AILERONS = (byte)(byte)239;
            p172.sue_ROLL_STABILIZATION_RUDDER = (byte)(byte)86;
            p172.sue_PITCH_STABILIZATION = (byte)(byte)175;
            p172.sue_YAW_STABILIZATION_RUDDER = (byte)(byte)13;
            p172.sue_YAW_STABILIZATION_AILERON = (byte)(byte)69;
            p172.sue_AILERON_NAVIGATION = (byte)(byte)89;
            p172.sue_RUDDER_NAVIGATION = (byte)(byte)179;
            p172.sue_ALTITUDEHOLD_STABILIZED = (byte)(byte)245;
            p172.sue_ALTITUDEHOLD_WAYPOINT = (byte)(byte)74;
            p172.sue_RACING_MODE = (byte)(byte)37;
            CommunicationChannel.instance.send(p172); //===============================
            SERIAL_UDB_EXTRA_F5 p173 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F5();
            PH.setPack(p173);
            p173.sue_YAWKP_AILERON = (float) -1.2510573E38F;
            p173.sue_YAWKD_AILERON = (float)7.488138E36F;
            p173.sue_ROLLKP = (float)8.713378E37F;
            p173.sue_ROLLKD = (float) -2.897282E38F;
            CommunicationChannel.instance.send(p173); //===============================
            SERIAL_UDB_EXTRA_F6 p174 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F6();
            PH.setPack(p174);
            p174.sue_PITCHGAIN = (float) -2.5303876E38F;
            p174.sue_PITCHKD = (float)1.2655674E38F;
            p174.sue_RUDDER_ELEV_MIX = (float) -9.282819E36F;
            p174.sue_ROLL_ELEV_MIX = (float) -2.5634278E38F;
            p174.sue_ELEVATOR_BOOST = (float) -1.7373312E38F;
            CommunicationChannel.instance.send(p174); //===============================
            SERIAL_UDB_EXTRA_F7 p175 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F7();
            PH.setPack(p175);
            p175.sue_YAWKP_RUDDER = (float)2.1061542E38F;
            p175.sue_YAWKD_RUDDER = (float) -2.9834763E38F;
            p175.sue_ROLLKP_RUDDER = (float) -2.0473561E38F;
            p175.sue_ROLLKD_RUDDER = (float)1.894412E38F;
            p175.sue_RUDDER_BOOST = (float) -1.6311777E38F;
            p175.sue_RTL_PITCH_DOWN = (float) -1.1092649E38F;
            CommunicationChannel.instance.send(p175); //===============================
            SERIAL_UDB_EXTRA_F8 p176 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F8();
            PH.setPack(p176);
            p176.sue_HEIGHT_TARGET_MAX = (float) -6.2780797E37F;
            p176.sue_HEIGHT_TARGET_MIN = (float)1.6256211E38F;
            p176.sue_ALT_HOLD_THROTTLE_MIN = (float)2.3850053E38F;
            p176.sue_ALT_HOLD_THROTTLE_MAX = (float) -9.432712E37F;
            p176.sue_ALT_HOLD_PITCH_MIN = (float) -2.5487693E38F;
            p176.sue_ALT_HOLD_PITCH_MAX = (float) -1.1088594E38F;
            p176.sue_ALT_HOLD_PITCH_HIGH = (float) -8.74097E37F;
            CommunicationChannel.instance.send(p176); //===============================
            SERIAL_UDB_EXTRA_F13 p177 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F13();
            PH.setPack(p177);
            p177.sue_week_no = (short)(short) -27774;
            p177.sue_lat_origin = (int) -614468431;
            p177.sue_lon_origin = (int)269664014;
            p177.sue_alt_origin = (int)946015020;
            CommunicationChannel.instance.send(p177); //===============================
            SERIAL_UDB_EXTRA_F14 p178 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F14();
            PH.setPack(p178);
            p178.sue_WIND_ESTIMATION = (byte)(byte)116;
            p178.sue_GPS_TYPE = (byte)(byte)179;
            p178.sue_DR = (byte)(byte)43;
            p178.sue_BOARD_TYPE = (byte)(byte)5;
            p178.sue_AIRFRAME = (byte)(byte)150;
            p178.sue_RCON = (short)(short)23610;
            p178.sue_TRAP_FLAGS = (short)(short)11160;
            p178.sue_TRAP_SOURCE = (uint)3834900004U;
            p178.sue_osc_fail_count = (short)(short)32581;
            p178.sue_CLOCK_CONFIG = (byte)(byte)231;
            p178.sue_FLIGHT_PLAN_TYPE = (byte)(byte)90;
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
            p181.time_boot_ms = (uint)4139096851U;
            p181.alt_gps = (int)1596205428;
            p181.alt_imu = (int) -1375476900;
            p181.alt_barometric = (int)268197489;
            p181.alt_optical_flow = (int)1490525942;
            p181.alt_range_finder = (int) -438844925;
            p181.alt_extra = (int) -1770290906;
            CommunicationChannel.instance.send(p181); //===============================
            AIRSPEEDS p182 = CommunicationChannel.new_AIRSPEEDS();
            PH.setPack(p182);
            p182.time_boot_ms = (uint)2575132662U;
            p182.airspeed_imu = (short)(short)12316;
            p182.airspeed_pitot = (short)(short)31430;
            p182.airspeed_hot_wire = (short)(short) -29402;
            p182.airspeed_ultrasonic = (short)(short)15708;
            p182.aoa = (short)(short) -10840;
            p182.aoy = (short)(short)851;
            CommunicationChannel.instance.send(p182); //===============================
            SERIAL_UDB_EXTRA_F17 p183 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F17();
            PH.setPack(p183);
            p183.sue_feed_forward = (float)9.918706E37F;
            p183.sue_turn_rate_nav = (float) -2.5281882E38F;
            p183.sue_turn_rate_fbw = (float)1.8842324E37F;
            CommunicationChannel.instance.send(p183); //===============================
            SERIAL_UDB_EXTRA_F18 p184 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F18();
            PH.setPack(p184);
            p184.angle_of_attack_normal = (float) -1.5295947E38F;
            p184.angle_of_attack_inverted = (float) -1.5297366E38F;
            p184.elevator_trim_normal = (float) -2.0444614E38F;
            p184.elevator_trim_inverted = (float)2.3209352E38F;
            p184.reference_speed = (float) -1.4399699E38F;
            CommunicationChannel.instance.send(p184); //===============================
            SERIAL_UDB_EXTRA_F19 p185 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F19();
            PH.setPack(p185);
            p185.sue_aileron_output_channel = (byte)(byte)32;
            p185.sue_aileron_reversed = (byte)(byte)231;
            p185.sue_elevator_output_channel = (byte)(byte)216;
            p185.sue_elevator_reversed = (byte)(byte)189;
            p185.sue_throttle_output_channel = (byte)(byte)51;
            p185.sue_throttle_reversed = (byte)(byte)16;
            p185.sue_rudder_output_channel = (byte)(byte)132;
            p185.sue_rudder_reversed = (byte)(byte)145;
            CommunicationChannel.instance.send(p185); //===============================
            SERIAL_UDB_EXTRA_F20 p186 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F20();
            PH.setPack(p186);
            p186.sue_number_of_inputs = (byte)(byte)184;
            p186.sue_trim_value_input_1 = (short)(short)17091;
            p186.sue_trim_value_input_2 = (short)(short)25885;
            p186.sue_trim_value_input_3 = (short)(short) -10046;
            p186.sue_trim_value_input_4 = (short)(short) -4771;
            p186.sue_trim_value_input_5 = (short)(short) -14340;
            p186.sue_trim_value_input_6 = (short)(short) -14849;
            p186.sue_trim_value_input_7 = (short)(short) -6799;
            p186.sue_trim_value_input_8 = (short)(short) -31417;
            p186.sue_trim_value_input_9 = (short)(short)5907;
            p186.sue_trim_value_input_10 = (short)(short)12692;
            p186.sue_trim_value_input_11 = (short)(short)20488;
            p186.sue_trim_value_input_12 = (short)(short)16388;
            CommunicationChannel.instance.send(p186); //===============================
            SERIAL_UDB_EXTRA_F21 p187 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F21();
            PH.setPack(p187);
            p187.sue_accel_x_offset = (short)(short)21242;
            p187.sue_accel_y_offset = (short)(short) -17482;
            p187.sue_accel_z_offset = (short)(short)4799;
            p187.sue_gyro_x_offset = (short)(short) -1210;
            p187.sue_gyro_y_offset = (short)(short)3707;
            p187.sue_gyro_z_offset = (short)(short) -22021;
            CommunicationChannel.instance.send(p187); //===============================
            SERIAL_UDB_EXTRA_F22 p188 = CommunicationChannel.new_SERIAL_UDB_EXTRA_F22();
            PH.setPack(p188);
            p188.sue_accel_x_at_calibration = (short)(short)15797;
            p188.sue_accel_y_at_calibration = (short)(short) -17966;
            p188.sue_accel_z_at_calibration = (short)(short)23356;
            p188.sue_gyro_x_at_calibration = (short)(short) -9974;
            p188.sue_gyro_y_at_calibration = (short)(short)28285;
            p188.sue_gyro_z_at_calibration = (short)(short)3475;
            CommunicationChannel.instance.send(p188); //===============================
            ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.time_usec = (ulong)475756448132854002L;
            p230.flags = (ESTIMATOR_STATUS_FLAGS)ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT;
            p230.vel_ratio = (float)1.6183548E38F;
            p230.pos_horiz_ratio = (float)4.7004956E37F;
            p230.pos_vert_ratio = (float) -1.2557244E38F;
            p230.mag_ratio = (float) -3.3424519E38F;
            p230.hagl_ratio = (float) -2.4347084E38F;
            p230.tas_ratio = (float)7.7481355E37F;
            p230.pos_horiz_accuracy = (float)3.1286523E38F;
            p230.pos_vert_accuracy = (float)1.8358467E38F;
            CommunicationChannel.instance.send(p230); //===============================
            WIND_COV p231 = CommunicationChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.time_usec = (ulong)6229228714890288727L;
            p231.wind_x = (float)1.1347755E38F;
            p231.wind_y = (float)1.9742392E38F;
            p231.wind_z = (float)1.0727278E38F;
            p231.var_horiz = (float) -2.8278772E38F;
            p231.var_vert = (float) -1.3107172E38F;
            p231.wind_alt = (float)1.4827639E38F;
            p231.horiz_accuracy = (float)2.62427E38F;
            p231.vert_accuracy = (float) -1.5442026E38F;
            CommunicationChannel.instance.send(p231); //===============================
            GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.time_usec = (ulong)4209175652232948470L;
            p232.gps_id = (byte)(byte)177;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS)GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY;
            p232.time_week_ms = (uint)572868306U;
            p232.time_week = (ushort)(ushort)25882;
            p232.fix_type = (byte)(byte)50;
            p232.lat = (int)1066815134;
            p232.lon = (int)1440234032;
            p232.alt = (float) -4.731801E37F;
            p232.hdop = (float) -6.2687396E37F;
            p232.vdop = (float) -3.338179E38F;
            p232.vn = (float)3.1391203E38F;
            p232.ve = (float) -3.1825987E38F;
            p232.vd = (float) -3.1770089E38F;
            p232.speed_accuracy = (float)3.2556046E38F;
            p232.horiz_accuracy = (float) -2.0375976E38F;
            p232.vert_accuracy = (float)1.5158304E38F;
            p232.satellites_visible = (byte)(byte)40;
            CommunicationChannel.instance.send(p232); //===============================
            GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.flags = (byte)(byte)162;
            p233.len = (byte)(byte)254;
            p233.data__SET(new byte[180], 0);
            CommunicationChannel.instance.send(p233); //===============================
            HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED;
            p234.custom_mode = (uint)1038470164U;
            p234.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING;
            p234.roll = (short)(short) -23463;
            p234.pitch = (short)(short) -4368;
            p234.heading = (ushort)(ushort)4160;
            p234.throttle = (sbyte)(sbyte) - 115;
            p234.heading_sp = (short)(short) -15046;
            p234.latitude = (int) -1666635051;
            p234.longitude = (int)1483128803;
            p234.altitude_amsl = (short)(short) -18547;
            p234.altitude_sp = (short)(short) -10979;
            p234.airspeed = (byte)(byte)196;
            p234.airspeed_sp = (byte)(byte)118;
            p234.groundspeed = (byte)(byte)12;
            p234.climb_rate = (sbyte)(sbyte) - 48;
            p234.gps_nsat = (byte)(byte)16;
            p234.gps_fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS;
            p234.battery_remaining = (byte)(byte)20;
            p234.temperature = (sbyte)(sbyte) - 26;
            p234.temperature_air = (sbyte)(sbyte)116;
            p234.failsafe = (byte)(byte)254;
            p234.wp_num = (byte)(byte)165;
            p234.wp_distance = (ushort)(ushort)46873;
            CommunicationChannel.instance.send(p234); //===============================
            VIBRATION p241 = CommunicationChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.time_usec = (ulong)3267859368134008201L;
            p241.vibration_x = (float)3.231716E37F;
            p241.vibration_y = (float) -8.051688E37F;
            p241.vibration_z = (float)2.9261223E38F;
            p241.clipping_0 = (uint)401265893U;
            p241.clipping_1 = (uint)1803837819U;
            p241.clipping_2 = (uint)4126520355U;
            CommunicationChannel.instance.send(p241); //===============================
            HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.latitude = (int)999078654;
            p242.longitude = (int)549515407;
            p242.altitude = (int) -747438213;
            p242.x = (float)1.712437E38F;
            p242.y = (float)3.280257E38F;
            p242.z = (float) -2.026985E38F;
            p242.q_SET(new float[4], 0);
            p242.approach_x = (float)1.7954182E38F;
            p242.approach_y = (float)2.1507492E36F;
            p242.approach_z = (float)1.2503518E38F;
            p242.time_usec_SET((ulong)993069647305911956L, PH);
            CommunicationChannel.instance.send(p242); //===============================
            SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.target_system = (byte)(byte)12;
            p243.latitude = (int) -1804874016;
            p243.longitude = (int)2073936340;
            p243.altitude = (int)590965292;
            p243.x = (float)2.1323098E38F;
            p243.y = (float) -1.2371349E38F;
            p243.z = (float) -2.9681712E38F;
            p243.q_SET(new float[4], 0);
            p243.approach_x = (float) -4.012548E37F;
            p243.approach_y = (float) -2.0376321E38F;
            p243.approach_z = (float) -4.5190365E37F;
            p243.time_usec_SET((ulong)8711496296774835554L, PH);
            CommunicationChannel.instance.send(p243); //===============================
            MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.message_id = (ushort)(ushort)17937;
            p244.interval_us = (int) -1417634349;
            CommunicationChannel.instance.send(p244); //===============================
            EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.vtol_state = (MAV_VTOL_STATE)MAV_VTOL_STATE.MAV_VTOL_STATE_MC;
            p245.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND;
            CommunicationChannel.instance.send(p245); //===============================
            ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.ICAO_address = (uint)1405125007U;
            p246.lat = (int) -76040937;
            p246.lon = (int) -1509645418;
            p246.altitude_type = (ADSB_ALTITUDE_TYPE)ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC;
            p246.altitude = (int) -1760870033;
            p246.heading = (ushort)(ushort)61033;
            p246.hor_velocity = (ushort)(ushort)8713;
            p246.ver_velocity = (short)(short)14981;
            p246.callsign_SET("DEMO", PH);
            p246.emitter_type = (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_HEAVY;
            p246.tslc = (byte)(byte)226;
            p246.flags = (ADSB_FLAGS)ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING;
            p246.squawk = (ushort)(ushort)57805;
            CommunicationChannel.instance.send(p246); //===============================
            COLLISION p247 = CommunicationChannel.new_COLLISION();
            PH.setPack(p247);
            p247.src_ = (MAV_COLLISION_SRC)MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT;
            p247.id = (uint)1676600668U;
            p247.action = (MAV_COLLISION_ACTION)MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_RTL;
            p247.threat_level = (MAV_COLLISION_THREAT_LEVEL)MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW;
            p247.time_to_minimum_delta = (float)3.315949E38F;
            p247.altitude_minimum_delta = (float)2.6109952E38F;
            p247.horizontal_minimum_delta = (float)2.650005E38F;
            CommunicationChannel.instance.send(p247); //===============================
            V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.target_network = (byte)(byte)19;
            p248.target_system = (byte)(byte)49;
            p248.target_component = (byte)(byte)166;
            p248.message_type = (ushort)(ushort)14474;
            p248.payload_SET(new byte[249], 0);
            CommunicationChannel.instance.send(p248); //===============================
            MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.address = (ushort)(ushort)58874;
            p249.ver = (byte)(byte)204;
            p249.type = (byte)(byte)28;
            p249.value_SET(new sbyte[32], 0);
            CommunicationChannel.instance.send(p249); //===============================
            DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.name_SET("DEMO", PH);
            p250.time_usec = (ulong)7128482336247183774L;
            p250.x = (float) -1.8111364E38F;
            p250.y = (float)3.2883157E38F;
            p250.z = (float)2.3700733E37F;
            CommunicationChannel.instance.send(p250); //===============================
            NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.time_boot_ms = (uint)2888462286U;
            p251.name_SET("DEMO", PH);
            p251.value = (float)4.8143037E37F;
            CommunicationChannel.instance.send(p251); //===============================
            NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.time_boot_ms = (uint)2599597096U;
            p252.name_SET("DEMO", PH);
            p252.value = (int)807978763;
            CommunicationChannel.instance.send(p252); //===============================
            STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.severity = (MAV_SEVERITY)MAV_SEVERITY.MAV_SEVERITY_ERROR;
            p253.text_SET("DEMO", PH);
            CommunicationChannel.instance.send(p253); //===============================
            DEBUG p254 = CommunicationChannel.new_DEBUG();
            PH.setPack(p254);
            p254.time_boot_ms = (uint)2062090381U;
            p254.ind = (byte)(byte)44;
            p254.value = (float)2.2781856E38F;
            CommunicationChannel.instance.send(p254); //===============================
            SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.target_system = (byte)(byte)146;
            p256.target_component = (byte)(byte)202;
            p256.secret_key_SET(new byte[32], 0);
            p256.initial_timestamp = (ulong)8183253709610150294L;
            CommunicationChannel.instance.send(p256); //===============================
            BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.time_boot_ms = (uint)595541311U;
            p257.last_change_ms = (uint)1717784794U;
            p257.state = (byte)(byte)178;
            CommunicationChannel.instance.send(p257); //===============================
            PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_system = (byte)(byte)44;
            p258.target_component = (byte)(byte)201;
            p258.tune_SET("DEMO", PH);
            CommunicationChannel.instance.send(p258); //===============================
            CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.time_boot_ms = (uint)3881706627U;
            p259.vendor_name_SET(new byte[32], 0);
            p259.model_name_SET(new byte[32], 0);
            p259.firmware_version = (uint)1820768867U;
            p259.focal_length = (float)6.380487E37F;
            p259.sensor_size_h = (float)7.7925464E37F;
            p259.sensor_size_v = (float)1.6763513E38F;
            p259.resolution_h = (ushort)(ushort)41599;
            p259.resolution_v = (ushort)(ushort)46366;
            p259.lens_id = (byte)(byte)39;
            p259.flags = (CAMERA_CAP_FLAGS)CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE;
            p259.cam_definition_version = (ushort)(ushort)26819;
            p259.cam_definition_uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p259); //===============================
            CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.time_boot_ms = (uint)213312805U;
            p260.mode_id = (CAMERA_MODE)CAMERA_MODE.CAMERA_MODE_IMAGE;
            CommunicationChannel.instance.send(p260); //===============================
            STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.time_boot_ms = (uint)259759099U;
            p261.storage_id = (byte)(byte)153;
            p261.storage_count = (byte)(byte)115;
            p261.status = (byte)(byte)1;
            p261.total_capacity = (float) -1.8787607E37F;
            p261.used_capacity = (float) -2.2832186E38F;
            p261.available_capacity = (float)1.2035875E38F;
            p261.read_speed = (float)1.6204487E38F;
            p261.write_speed = (float) -2.498176E38F;
            CommunicationChannel.instance.send(p261); //===============================
            CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.time_boot_ms = (uint)2589704794U;
            p262.image_status = (byte)(byte)9;
            p262.video_status = (byte)(byte)41;
            p262.image_interval = (float) -2.346453E38F;
            p262.recording_time_ms = (uint)2450836429U;
            p262.available_capacity = (float) -3.0411742E37F;
            CommunicationChannel.instance.send(p262); //===============================
            CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.time_boot_ms = (uint)2016029974U;
            p263.time_utc = (ulong)6188327973276243254L;
            p263.camera_id = (byte)(byte)29;
            p263.lat = (int)121572866;
            p263.lon = (int) -2117827748;
            p263.alt = (int)1075007327;
            p263.relative_alt = (int)1792322618;
            p263.q_SET(new float[4], 0);
            p263.image_index = (int)248351091;
            p263.capture_result = (sbyte)(sbyte) - 96;
            p263.file_url_SET("DEMO", PH);
            CommunicationChannel.instance.send(p263); //===============================
            FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.time_boot_ms = (uint)4163362128U;
            p264.arming_time_utc = (ulong)8987779917121350887L;
            p264.takeoff_time_utc = (ulong)6163085676334207730L;
            p264.flight_uuid = (ulong)5088152034347571270L;
            CommunicationChannel.instance.send(p264); //===============================
            MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.time_boot_ms = (uint)1474636966U;
            p265.roll = (float) -1.1294559E38F;
            p265.pitch = (float)1.4946206E38F;
            p265.yaw = (float)1.7885382E38F;
            CommunicationChannel.instance.send(p265); //===============================
            LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.target_system = (byte)(byte)70;
            p266.target_component = (byte)(byte)45;
            p266.sequence = (ushort)(ushort)26472;
            p266.length = (byte)(byte)233;
            p266.first_message_offset = (byte)(byte)220;
            p266.data__SET(new byte[249], 0);
            CommunicationChannel.instance.send(p266); //===============================
            LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.target_system = (byte)(byte)86;
            p267.target_component = (byte)(byte)13;
            p267.sequence = (ushort)(ushort)64066;
            p267.length = (byte)(byte)151;
            p267.first_message_offset = (byte)(byte)155;
            p267.data__SET(new byte[249], 0);
            CommunicationChannel.instance.send(p267); //===============================
            LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_system = (byte)(byte)206;
            p268.target_component = (byte)(byte)179;
            p268.sequence = (ushort)(ushort)18362;
            CommunicationChannel.instance.send(p268); //===============================
            VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.camera_id = (byte)(byte)50;
            p269.status = (byte)(byte)67;
            p269.framerate = (float)2.0867164E38F;
            p269.resolution_h = (ushort)(ushort)44861;
            p269.resolution_v = (ushort)(ushort)3237;
            p269.bitrate = (uint)2407485771U;
            p269.rotation = (ushort)(ushort)20006;
            p269.uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p269); //===============================
            SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.target_system = (byte)(byte)76;
            p270.target_component = (byte)(byte)157;
            p270.camera_id = (byte)(byte)96;
            p270.framerate = (float)1.1078071E38F;
            p270.resolution_h = (ushort)(ushort)55111;
            p270.resolution_v = (ushort)(ushort)61064;
            p270.bitrate = (uint)754377321U;
            p270.rotation = (ushort)(ushort)10052;
            p270.uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p270); //===============================
            WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.ssid_SET("DEMO", PH);
            p299.password_SET("DEMO", PH);
            CommunicationChannel.instance.send(p299); //===============================
            PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.version = (ushort)(ushort)17644;
            p300.min_version = (ushort)(ushort)9527;
            p300.max_version = (ushort)(ushort)54603;
            p300.spec_version_hash_SET(new byte[8], 0);
            p300.library_version_hash_SET(new byte[8], 0);
            CommunicationChannel.instance.send(p300); //===============================
            UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.time_usec = (ulong)6961074608872644136L;
            p310.uptime_sec = (uint)4117187379U;
            p310.health = (UAVCAN_NODE_HEALTH)UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK;
            p310.mode = (UAVCAN_NODE_MODE)UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION;
            p310.sub_mode = (byte)(byte)14;
            p310.vendor_specific_status_code = (ushort)(ushort)51787;
            CommunicationChannel.instance.send(p310); //===============================
            UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.time_usec = (ulong)2036098804516961342L;
            p311.uptime_sec = (uint)111052524U;
            p311.name_SET("DEMO", PH);
            p311.hw_version_major = (byte)(byte)125;
            p311.hw_version_minor = (byte)(byte)113;
            p311.hw_unique_id_SET(new byte[16], 0);
            p311.sw_version_major = (byte)(byte)99;
            p311.sw_version_minor = (byte)(byte)39;
            p311.sw_vcs_commit = (uint)4141173341U;
            CommunicationChannel.instance.send(p311); //===============================
            PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.target_system = (byte)(byte)103;
            p320.target_component = (byte)(byte)58;
            p320.param_id_SET("DEMO", PH);
            p320.param_index = (short)(short) -27822;
            CommunicationChannel.instance.send(p320); //===============================
            PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_system = (byte)(byte)35;
            p321.target_component = (byte)(byte)18;
            CommunicationChannel.instance.send(p321); //===============================
            PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_id_SET("DEMO", PH);
            p322.param_value_SET("DEMO", PH);
            p322.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32;
            p322.param_count = (ushort)(ushort)4182;
            p322.param_index = (ushort)(ushort)80;
            CommunicationChannel.instance.send(p322); //===============================
            PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.target_system = (byte)(byte)60;
            p323.target_component = (byte)(byte)14;
            p323.param_id_SET("DEMO", PH);
            p323.param_value_SET("DEMO", PH);
            p323.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32;
            CommunicationChannel.instance.send(p323); //===============================
            PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_id_SET("DEMO", PH);
            p324.param_value_SET("DEMO", PH);
            p324.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16;
            p324.param_result = (PARAM_ACK)PARAM_ACK.PARAM_ACK_FAILED;
            CommunicationChannel.instance.send(p324); //===============================
            OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.time_usec = (ulong)7040310293735630118L;
            p330.sensor_type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER;
            p330.distances_SET(new ushort[72], 0);
            p330.increment = (byte)(byte)36;
            p330.min_distance = (ushort)(ushort)56000;
            p330.max_distance = (ushort)(ushort)61323;
            CommunicationChannel.instance.send(p330); //===============================
        }
    }
}

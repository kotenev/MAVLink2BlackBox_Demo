
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
            POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.time_boot_ms = (uint)4135539130U;
            p3.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL;
            p3.type_mask = (ushort)(ushort)22097;
            p3.x = (float)2.8344513E38F;
            p3.y = (float)4.4110413E37F;
            p3.z = (float)1.2547053E38F;
            p3.vx = (float)1.1894588E38F;
            p3.vy = (float)1.1213304E38F;
            p3.vz = (float) -1.0906434E38F;
            p3.afx = (float)1.4659353E38F;
            p3.afy = (float)1.9887988E38F;
            p3.afz = (float)2.8895728E38F;
            p3.yaw = (float)2.3952408E38F;
            p3.yaw_rate = (float)3.1214251E38F;
            CommunicationChannel.instance.send(p3); //===============================
            COMMAND_LONG p76 = CommunicationChannel.new_COMMAND_LONG();
            PH.setPack(p76);
            p76.target_system = (byte)(byte)67;
            p76.target_component = (byte)(byte)215;
            p76.command = MAV_CMD.MAV_CMD_DO_FOLLOW;
            p76.confirmation = (byte)(byte)226;
            p76.param1 = (float) -1.0744019E38F;
            p76.param2 = (float)1.9023987E38F;
            p76.param3 = (float) -3.25471E38F;
            p76.param4 = (float)3.128363E38F;
            p76.param5 = (float)2.665565E38F;
            p76.param6 = (float) -4.9908834E37F;
            p76.param7 = (float) -2.2565801E38F;
            CommunicationChannel.instance.send(p76); //===============================
            COMMAND_ACK p77 = CommunicationChannel.new_COMMAND_ACK();
            PH.setPack(p77);
            p77.command = MAV_CMD.MAV_CMD_USER_3;
            p77.result = MAV_RESULT.MAV_RESULT_FAILED;
            p77.progress_SET((byte)(byte)142, PH);
            p77.result_param2_SET((int) -250160324, PH);
            p77.target_system_SET((byte)(byte)242, PH);
            p77.target_component_SET((byte)(byte)79, PH);
            CommunicationChannel.instance.send(p77); //===============================
            MANUAL_SETPOINT p81 = CommunicationChannel.new_MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.time_boot_ms = (uint)846028196U;
            p81.roll = (float) -2.1100902E37F;
            p81.pitch = (float) -2.7150224E38F;
            p81.yaw = (float)2.5178979E38F;
            p81.thrust = (float) -1.3066451E38F;
            p81.mode_switch = (byte)(byte)62;
            p81.manual_override_switch = (byte)(byte)184;
            CommunicationChannel.instance.send(p81); //===============================
            SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.time_boot_ms = (uint)641701883U;
            p82.target_system = (byte)(byte)169;
            p82.target_component = (byte)(byte)65;
            p82.type_mask = (byte)(byte)199;
            p82.q_SET(new float[4], 0);
            p82.body_roll_rate = (float) -1.1396828E38F;
            p82.body_pitch_rate = (float)1.6531057E38F;
            p82.body_yaw_rate = (float)3.2728868E38F;
            p82.thrust = (float) -3.345224E38F;
            CommunicationChannel.instance.send(p82); //===============================
            ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.time_boot_ms = (uint)1269539190U;
            p83.type_mask = (byte)(byte)205;
            p83.q_SET(new float[4], 0);
            p83.body_roll_rate = (float) -1.1047778E38F;
            p83.body_pitch_rate = (float)2.1559218E38F;
            p83.body_yaw_rate = (float)1.8273958E38F;
            p83.thrust = (float) -1.2441052E38F;
            CommunicationChannel.instance.send(p83); //===============================
            SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.time_boot_ms = (uint)2988026218U;
            p84.target_system = (byte)(byte)171;
            p84.target_component = (byte)(byte)5;
            p84.coordinate_frame = MAV_FRAME.MAV_FRAME_MISSION;
            p84.type_mask = (ushort)(ushort)51134;
            p84.x = (float) -1.9569943E37F;
            p84.y = (float) -6.5554376E37F;
            p84.z = (float)9.677332E37F;
            p84.vx = (float)2.6121695E38F;
            p84.vy = (float)2.4243325E38F;
            p84.vz = (float) -1.1311115E38F;
            p84.afx = (float)5.185477E37F;
            p84.afy = (float)2.9621624E38F;
            p84.afz = (float)7.7148994E36F;
            p84.yaw = (float) -1.3360663E38F;
            p84.yaw_rate = (float)1.100719E38F;
            CommunicationChannel.instance.send(p84); //===============================
            SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.time_boot_ms = (uint)860845986U;
            p86.target_system = (byte)(byte)202;
            p86.target_component = (byte)(byte)121;
            p86.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
            p86.type_mask = (ushort)(ushort)16200;
            p86.lat_int = (int) -804439768;
            p86.lon_int = (int) -1644946165;
            p86.alt = (float)2.2112491E38F;
            p86.vx = (float) -1.1895717E38F;
            p86.vy = (float)1.080185E38F;
            p86.vz = (float) -2.3238147E38F;
            p86.afx = (float) -2.5910626E38F;
            p86.afy = (float)3.2376845E38F;
            p86.afz = (float) -2.4978917E38F;
            p86.yaw = (float) -3.503516E37F;
            p86.yaw_rate = (float)2.6405355E38F;
            CommunicationChannel.instance.send(p86); //===============================
            POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.time_boot_ms = (uint)2335702074U;
            p87.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_INT;
            p87.type_mask = (ushort)(ushort)29882;
            p87.lat_int = (int) -930999268;
            p87.lon_int = (int) -1130210841;
            p87.alt = (float) -3.1567398E38F;
            p87.vx = (float) -2.8919516E38F;
            p87.vy = (float)3.0685444E38F;
            p87.vz = (float) -1.6877801E38F;
            p87.afx = (float)2.5784107E38F;
            p87.afy = (float) -1.0788453E38F;
            p87.afz = (float) -5.40011E37F;
            p87.yaw = (float) -1.9098821E38F;
            p87.yaw_rate = (float) -2.0531371E37F;
            CommunicationChannel.instance.send(p87); //===============================
            LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.time_boot_ms = (uint)2279782930U;
            p89.x = (float)1.4198672E38F;
            p89.y = (float) -3.3951555E38F;
            p89.z = (float)1.0403212E38F;
            p89.roll = (float) -3.1026775E38F;
            p89.pitch = (float) -2.0200142E38F;
            p89.yaw = (float) -5.226452E37F;
            CommunicationChannel.instance.send(p89); //===============================
            HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.time_usec = (ulong)2653922260570695401L;
            p90.roll = (float)2.3049291E38F;
            p90.pitch = (float)2.4684187E37F;
            p90.yaw = (float) -2.480919E38F;
            p90.rollspeed = (float)2.470661E38F;
            p90.pitchspeed = (float) -2.4641943E38F;
            p90.yawspeed = (float) -2.2991263E38F;
            p90.lat = (int)542872557;
            p90.lon = (int) -1901970236;
            p90.alt = (int) -1037887583;
            p90.vx = (short)(short)9998;
            p90.vy = (short)(short) -8833;
            p90.vz = (short)(short) -17623;
            p90.xacc = (short)(short) -31316;
            p90.yacc = (short)(short) -167;
            p90.zacc = (short)(short)20490;
            CommunicationChannel.instance.send(p90); //===============================
            HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.time_usec = (ulong)6707687875662902401L;
            p91.roll_ailerons = (float)2.0181064E38F;
            p91.pitch_elevator = (float) -1.755293E38F;
            p91.yaw_rudder = (float)3.0709607E38F;
            p91.throttle = (float) -3.2929857E38F;
            p91.aux1 = (float) -2.1167368E37F;
            p91.aux2 = (float) -3.2902965E38F;
            p91.aux3 = (float) -8.1898575E37F;
            p91.aux4 = (float) -2.9058162E38F;
            p91.mode = MAV_MODE.MAV_MODE_TEST_DISARMED;
            p91.nav_mode = (byte)(byte)193;
            CommunicationChannel.instance.send(p91); //===============================
            HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.time_usec = (ulong)2611892290970178980L;
            p92.chan1_raw = (ushort)(ushort)61251;
            p92.chan2_raw = (ushort)(ushort)22629;
            p92.chan3_raw = (ushort)(ushort)24616;
            p92.chan4_raw = (ushort)(ushort)48755;
            p92.chan5_raw = (ushort)(ushort)11338;
            p92.chan6_raw = (ushort)(ushort)1919;
            p92.chan7_raw = (ushort)(ushort)28632;
            p92.chan8_raw = (ushort)(ushort)61832;
            p92.chan9_raw = (ushort)(ushort)41253;
            p92.chan10_raw = (ushort)(ushort)37888;
            p92.chan11_raw = (ushort)(ushort)47333;
            p92.chan12_raw = (ushort)(ushort)33178;
            p92.rssi = (byte)(byte)6;
            CommunicationChannel.instance.send(p92); //===============================
            HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.time_usec = (ulong)5020621252683386670L;
            p93.controls_SET(new float[16], 0);
            p93.mode = MAV_MODE.MAV_MODE_AUTO_ARMED;
            p93.flags = (ulong)7243809577621659820L;
            CommunicationChannel.instance.send(p93); //===============================
            OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.time_usec = (ulong)7728583182240336285L;
            p100.sensor_id = (byte)(byte)211;
            p100.flow_x = (short)(short)12534;
            p100.flow_y = (short)(short) -19559;
            p100.flow_comp_m_x = (float) -1.8241875E37F;
            p100.flow_comp_m_y = (float) -1.4104633E38F;
            p100.quality = (byte)(byte)107;
            p100.ground_distance = (float) -2.3147292E38F;
            p100.flow_rate_x_SET((float) -1.9759635E38F, PH);
            p100.flow_rate_y_SET((float)2.8294878E38F, PH);
            CommunicationChannel.instance.send(p100); //===============================
            GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.usec = (ulong)7409628053752779925L;
            p101.x = (float) -1.2448834E38F;
            p101.y = (float)2.2656247E38F;
            p101.z = (float)3.0855833E38F;
            p101.roll = (float)3.518751E37F;
            p101.pitch = (float) -2.367857E38F;
            p101.yaw = (float) -3.3570927E38F;
            CommunicationChannel.instance.send(p101); //===============================
            VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.usec = (ulong)9152424950496666685L;
            p102.x = (float)1.2306851E37F;
            p102.y = (float)2.4806333E37F;
            p102.z = (float)2.3618085E37F;
            p102.roll = (float)2.016603E38F;
            p102.pitch = (float) -3.1167598E38F;
            p102.yaw = (float)3.2701138E38F;
            CommunicationChannel.instance.send(p102); //===============================
            VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.usec = (ulong)364676471630821626L;
            p103.x = (float)2.091108E38F;
            p103.y = (float) -2.909324E38F;
            p103.z = (float)4.0999547E37F;
            CommunicationChannel.instance.send(p103); //===============================
            VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.usec = (ulong)7389409601291828242L;
            p104.x = (float)1.0363141E38F;
            p104.y = (float) -2.9261355E38F;
            p104.z = (float)2.0854096E38F;
            p104.roll = (float)1.3654323E38F;
            p104.pitch = (float)3.1883137E38F;
            p104.yaw = (float)2.052164E38F;
            CommunicationChannel.instance.send(p104); //===============================
            HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.time_usec = (ulong)7752198381429355433L;
            p105.xacc = (float) -3.0996309E38F;
            p105.yacc = (float) -1.0967569E38F;
            p105.zacc = (float)5.9027004E37F;
            p105.xgyro = (float) -2.3308332E38F;
            p105.ygyro = (float) -5.2091465E37F;
            p105.zgyro = (float) -9.48283E37F;
            p105.xmag = (float)4.0594124E36F;
            p105.ymag = (float)2.6718515E38F;
            p105.zmag = (float) -1.4499596E38F;
            p105.abs_pressure = (float)8.245348E37F;
            p105.diff_pressure = (float) -2.0665636E38F;
            p105.pressure_alt = (float)2.120118E38F;
            p105.temperature = (float) -2.2476464E37F;
            p105.fields_updated = (ushort)(ushort)37982;
            CommunicationChannel.instance.send(p105); //===============================
            OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.time_usec = (ulong)5412050699835719577L;
            p106.sensor_id = (byte)(byte)23;
            p106.integration_time_us = (uint)3115211398U;
            p106.integrated_x = (float) -3.1835546E38F;
            p106.integrated_y = (float) -1.9651541E38F;
            p106.integrated_xgyro = (float) -5.227324E37F;
            p106.integrated_ygyro = (float)8.3822067E37F;
            p106.integrated_zgyro = (float)1.144567E38F;
            p106.temperature = (short)(short)15910;
            p106.quality = (byte)(byte)20;
            p106.time_delta_distance_us = (uint)734986900U;
            p106.distance = (float) -1.838492E38F;
            CommunicationChannel.instance.send(p106); //===============================
            HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.time_usec = (ulong)6220599616634851299L;
            p107.xacc = (float)2.4893632E38F;
            p107.yacc = (float)3.3896182E38F;
            p107.zacc = (float) -7.3560416E37F;
            p107.xgyro = (float)8.3552763E37F;
            p107.ygyro = (float) -2.9045266E38F;
            p107.zgyro = (float) -2.4972528E38F;
            p107.xmag = (float) -1.4665181E38F;
            p107.ymag = (float) -1.3986526E38F;
            p107.zmag = (float) -9.328357E37F;
            p107.abs_pressure = (float) -3.2679615E38F;
            p107.diff_pressure = (float) -2.5130634E38F;
            p107.pressure_alt = (float)1.695823E38F;
            p107.temperature = (float) -2.6536448E38F;
            p107.fields_updated = (uint)2627918490U;
            CommunicationChannel.instance.send(p107); //===============================
            SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.q1 = (float)1.3740223E38F;
            p108.q2 = (float)1.1454803E38F;
            p108.q3 = (float)1.7339855E37F;
            p108.q4 = (float)2.4365816E38F;
            p108.roll = (float)2.9542157E38F;
            p108.pitch = (float) -2.8619906E38F;
            p108.yaw = (float)1.001669E38F;
            p108.xacc = (float) -2.8553984E38F;
            p108.yacc = (float) -1.6354126E38F;
            p108.zacc = (float)7.7898017E37F;
            p108.xgyro = (float) -2.6743294E38F;
            p108.ygyro = (float)1.9230783E38F;
            p108.zgyro = (float) -2.4843757E38F;
            p108.lat = (float) -2.3601616E38F;
            p108.lon = (float) -3.25212E38F;
            p108.alt = (float)2.8654643E38F;
            p108.std_dev_horz = (float) -3.2186795E38F;
            p108.std_dev_vert = (float) -2.6974295E38F;
            p108.vn = (float)2.1741565E38F;
            p108.ve = (float)2.4293816E38F;
            p108.vd = (float) -9.299638E37F;
            CommunicationChannel.instance.send(p108); //===============================
            RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.rssi = (byte)(byte)16;
            p109.remrssi = (byte)(byte)48;
            p109.txbuf = (byte)(byte)153;
            p109.noise = (byte)(byte)48;
            p109.remnoise = (byte)(byte)31;
            p109.rxerrors = (ushort)(ushort)50013;
            p109.fixed_ = (ushort)(ushort)41623;
            CommunicationChannel.instance.send(p109); //===============================
            FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_network = (byte)(byte)34;
            p110.target_system = (byte)(byte)202;
            p110.target_component = (byte)(byte)219;
            p110.payload_SET(new byte[251], 0);
            CommunicationChannel.instance.send(p110); //===============================
            TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.tc1 = (long) -4380986720187447025L;
            p111.ts1 = (long)2304093068238763229L;
            CommunicationChannel.instance.send(p111); //===============================
            CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.time_usec = (ulong)8515488735237239632L;
            p112.seq = (uint)2829017184U;
            CommunicationChannel.instance.send(p112); //===============================
            HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.time_usec = (ulong)2473929766573986989L;
            p113.fix_type = (byte)(byte)162;
            p113.lat = (int)1422979535;
            p113.lon = (int) -1691902840;
            p113.alt = (int) -1724322472;
            p113.eph = (ushort)(ushort)49497;
            p113.epv = (ushort)(ushort)50215;
            p113.vel = (ushort)(ushort)62179;
            p113.vn = (short)(short)5177;
            p113.ve = (short)(short)30612;
            p113.vd = (short)(short) -5591;
            p113.cog = (ushort)(ushort)33958;
            p113.satellites_visible = (byte)(byte)139;
            CommunicationChannel.instance.send(p113); //===============================
            HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.time_usec = (ulong)2248432365214401899L;
            p114.sensor_id = (byte)(byte)153;
            p114.integration_time_us = (uint)1036011754U;
            p114.integrated_x = (float) -3.0568593E38F;
            p114.integrated_y = (float) -3.0956529E38F;
            p114.integrated_xgyro = (float) -7.7933734E37F;
            p114.integrated_ygyro = (float) -1.2010541E38F;
            p114.integrated_zgyro = (float)1.9872076E38F;
            p114.temperature = (short)(short) -10991;
            p114.quality = (byte)(byte)184;
            p114.time_delta_distance_us = (uint)815609018U;
            p114.distance = (float) -9.650139E37F;
            CommunicationChannel.instance.send(p114); //===============================
            HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.time_usec = (ulong)3361109211274814829L;
            p115.attitude_quaternion_SET(new float[4], 0);
            p115.rollspeed = (float) -2.0193337E38F;
            p115.pitchspeed = (float) -2.7517192E38F;
            p115.yawspeed = (float) -5.901386E37F;
            p115.lat = (int)1477760031;
            p115.lon = (int) -1432664344;
            p115.alt = (int)110443096;
            p115.vx = (short)(short) -3036;
            p115.vy = (short)(short)10238;
            p115.vz = (short)(short)23549;
            p115.ind_airspeed = (ushort)(ushort)60699;
            p115.true_airspeed = (ushort)(ushort)6642;
            p115.xacc = (short)(short) -25209;
            p115.yacc = (short)(short)29375;
            p115.zacc = (short)(short) -26029;
            CommunicationChannel.instance.send(p115); //===============================
            SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.time_boot_ms = (uint)918925947U;
            p116.xacc = (short)(short) -32284;
            p116.yacc = (short)(short) -14789;
            p116.zacc = (short)(short) -6159;
            p116.xgyro = (short)(short) -14162;
            p116.ygyro = (short)(short) -17138;
            p116.zgyro = (short)(short) -25093;
            p116.xmag = (short)(short)26385;
            p116.ymag = (short)(short)30425;
            p116.zmag = (short)(short)6872;
            CommunicationChannel.instance.send(p116); //===============================
            LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.target_system = (byte)(byte)224;
            p117.target_component = (byte)(byte)89;
            p117.start = (ushort)(ushort)22813;
            p117.end = (ushort)(ushort)8501;
            CommunicationChannel.instance.send(p117); //===============================
            LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.id = (ushort)(ushort)63907;
            p118.num_logs = (ushort)(ushort)49772;
            p118.last_log_num = (ushort)(ushort)8470;
            p118.time_utc = (uint)3025461448U;
            p118.size = (uint)3845370475U;
            CommunicationChannel.instance.send(p118); //===============================
            LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.target_system = (byte)(byte)247;
            p119.target_component = (byte)(byte)22;
            p119.id = (ushort)(ushort)62809;
            p119.ofs = (uint)4093546927U;
            p119.count = (uint)816834262U;
            CommunicationChannel.instance.send(p119); //===============================
            LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.id = (ushort)(ushort)30298;
            p120.ofs = (uint)746887155U;
            p120.count = (byte)(byte)44;
            p120.data__SET(new byte[90], 0);
            CommunicationChannel.instance.send(p120); //===============================
            LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_system = (byte)(byte)96;
            p121.target_component = (byte)(byte)215;
            CommunicationChannel.instance.send(p121); //===============================
            LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_system = (byte)(byte)139;
            p122.target_component = (byte)(byte)247;
            CommunicationChannel.instance.send(p122); //===============================
            GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.target_system = (byte)(byte)49;
            p123.target_component = (byte)(byte)225;
            p123.len = (byte)(byte)213;
            p123.data__SET(new byte[110], 0);
            CommunicationChannel.instance.send(p123); //===============================
            GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.time_usec = (ulong)4271502524147559075L;
            p124.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX;
            p124.lat = (int) -529264474;
            p124.lon = (int)1082325032;
            p124.alt = (int) -356195306;
            p124.eph = (ushort)(ushort)16102;
            p124.epv = (ushort)(ushort)5725;
            p124.vel = (ushort)(ushort)12083;
            p124.cog = (ushort)(ushort)42141;
            p124.satellites_visible = (byte)(byte)80;
            p124.dgps_numch = (byte)(byte)0;
            p124.dgps_age = (uint)1949915764U;
            CommunicationChannel.instance.send(p124); //===============================
            POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vcc = (ushort)(ushort)13977;
            p125.Vservo = (ushort)(ushort)15487;
            p125.flags = (MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED |
                          MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED |
                          MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID |
                          MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT);
            CommunicationChannel.instance.send(p125); //===============================
            SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.device = SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_SHELL;
            p126.flags = (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND |
                          SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI);
            p126.timeout = (ushort)(ushort)39532;
            p126.baudrate = (uint)1914213392U;
            p126.count = (byte)(byte)41;
            p126.data__SET(new byte[70], 0);
            CommunicationChannel.instance.send(p126); //===============================
            GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.time_last_baseline_ms = (uint)2497072894U;
            p127.rtk_receiver_id = (byte)(byte)202;
            p127.wn = (ushort)(ushort)17458;
            p127.tow = (uint)2385341760U;
            p127.rtk_health = (byte)(byte)146;
            p127.rtk_rate = (byte)(byte)173;
            p127.nsats = (byte)(byte)70;
            p127.baseline_coords_type = (byte)(byte)65;
            p127.baseline_a_mm = (int)559248038;
            p127.baseline_b_mm = (int)1147121921;
            p127.baseline_c_mm = (int) -1502560587;
            p127.accuracy = (uint)1889420783U;
            p127.iar_num_hypotheses = (int) -1295318077;
            CommunicationChannel.instance.send(p127); //===============================
            GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.time_last_baseline_ms = (uint)1934586151U;
            p128.rtk_receiver_id = (byte)(byte)3;
            p128.wn = (ushort)(ushort)26644;
            p128.tow = (uint)1623236085U;
            p128.rtk_health = (byte)(byte)178;
            p128.rtk_rate = (byte)(byte)120;
            p128.nsats = (byte)(byte)241;
            p128.baseline_coords_type = (byte)(byte)37;
            p128.baseline_a_mm = (int) -600304443;
            p128.baseline_b_mm = (int) -1518391254;
            p128.baseline_c_mm = (int) -1947563513;
            p128.accuracy = (uint)550257960U;
            p128.iar_num_hypotheses = (int)181507808;
            CommunicationChannel.instance.send(p128); //===============================
            SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.time_boot_ms = (uint)2175771051U;
            p129.xacc = (short)(short) -25017;
            p129.yacc = (short)(short)5769;
            p129.zacc = (short)(short)14959;
            p129.xgyro = (short)(short)313;
            p129.ygyro = (short)(short)9227;
            p129.zgyro = (short)(short)18259;
            p129.xmag = (short)(short)28197;
            p129.ymag = (short)(short)29545;
            p129.zmag = (short)(short) -23667;
            CommunicationChannel.instance.send(p129); //===============================
            DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.type = (byte)(byte)66;
            p130.size = (uint)3540890354U;
            p130.width = (ushort)(ushort)23808;
            p130.height = (ushort)(ushort)1175;
            p130.packets = (ushort)(ushort)6952;
            p130.payload = (byte)(byte)189;
            p130.jpg_quality = (byte)(byte)41;
            CommunicationChannel.instance.send(p130); //===============================
            ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.seqnr = (ushort)(ushort)27216;
            p131.data__SET(new byte[253], 0);
            CommunicationChannel.instance.send(p131); //===============================
            DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.time_boot_ms = (uint)159655401U;
            p132.min_distance = (ushort)(ushort)51687;
            p132.max_distance = (ushort)(ushort)58959;
            p132.current_distance = (ushort)(ushort)59927;
            p132.type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER;
            p132.id = (byte)(byte)49;
            p132.orientation = MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180;
            p132.covariance = (byte)(byte)152;
            CommunicationChannel.instance.send(p132); //===============================
            TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.lat = (int)752860791;
            p133.lon = (int) -225570950;
            p133.grid_spacing = (ushort)(ushort)55734;
            p133.mask = (ulong)7046664608708385458L;
            CommunicationChannel.instance.send(p133); //===============================
            TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.lat = (int)152385973;
            p134.lon = (int) -1529698318;
            p134.grid_spacing = (ushort)(ushort)60980;
            p134.gridbit = (byte)(byte)68;
            p134.data__SET(new short[16], 0);
            CommunicationChannel.instance.send(p134); //===============================
            TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lat = (int)2145344419;
            p135.lon = (int)2111333138;
            CommunicationChannel.instance.send(p135); //===============================
            TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.lat = (int) -1371919075;
            p136.lon = (int)1452416983;
            p136.spacing = (ushort)(ushort)5874;
            p136.terrain_height = (float) -3.3222907E38F;
            p136.current_height = (float) -1.9523852E38F;
            p136.pending = (ushort)(ushort)283;
            p136.loaded = (ushort)(ushort)63248;
            CommunicationChannel.instance.send(p136); //===============================
            SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.time_boot_ms = (uint)669217873U;
            p137.press_abs = (float)1.4661467E38F;
            p137.press_diff = (float)1.7833453E38F;
            p137.temperature = (short)(short) -23499;
            CommunicationChannel.instance.send(p137); //===============================
            ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.time_usec = (ulong)332384210748115183L;
            p138.q_SET(new float[4], 0);
            p138.x = (float)2.7192598E38F;
            p138.y = (float) -1.3174411E38F;
            p138.z = (float)3.2411536E38F;
            CommunicationChannel.instance.send(p138); //===============================
            SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.time_usec = (ulong)2505795404658085700L;
            p139.group_mlx = (byte)(byte)6;
            p139.target_system = (byte)(byte)162;
            p139.target_component = (byte)(byte)207;
            p139.controls_SET(new float[8], 0);
            CommunicationChannel.instance.send(p139); //===============================
            ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.time_usec = (ulong)1943391377370147099L;
            p140.group_mlx = (byte)(byte)108;
            p140.controls_SET(new float[8], 0);
            CommunicationChannel.instance.send(p140); //===============================
            ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.time_usec = (ulong)845886570606333024L;
            p141.altitude_monotonic = (float) -2.602424E38F;
            p141.altitude_amsl = (float)5.025768E37F;
            p141.altitude_local = (float) -9.434949E37F;
            p141.altitude_relative = (float) -1.9812286E38F;
            p141.altitude_terrain = (float) -3.0490674E38F;
            p141.bottom_clearance = (float)1.5509186E38F;
            CommunicationChannel.instance.send(p141); //===============================
            RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.request_id = (byte)(byte)164;
            p142.uri_type = (byte)(byte)26;
            p142.uri_SET(new byte[120], 0);
            p142.transfer_type = (byte)(byte)126;
            p142.storage_SET(new byte[120], 0);
            CommunicationChannel.instance.send(p142); //===============================
            SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.time_boot_ms = (uint)1305526792U;
            p143.press_abs = (float) -4.0850855E36F;
            p143.press_diff = (float)1.5747249E38F;
            p143.temperature = (short)(short) -204;
            CommunicationChannel.instance.send(p143); //===============================
            FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.timestamp = (ulong)3351674135716469823L;
            p144.est_capabilities = (byte)(byte)30;
            p144.lat = (int)2103668159;
            p144.lon = (int)1752621180;
            p144.alt = (float)2.785336E38F;
            p144.vel_SET(new float[3], 0);
            p144.acc_SET(new float[3], 0);
            p144.attitude_q_SET(new float[4], 0);
            p144.rates_SET(new float[3], 0);
            p144.position_cov_SET(new float[3], 0);
            p144.custom_state = (ulong)2228251133006444164L;
            CommunicationChannel.instance.send(p144); //===============================
            CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.time_usec = (ulong)6778349778090410787L;
            p146.x_acc = (float) -1.6504474E38F;
            p146.y_acc = (float) -2.4459736E38F;
            p146.z_acc = (float)2.4093878E38F;
            p146.x_vel = (float)3.9296487E37F;
            p146.y_vel = (float)2.2637173E38F;
            p146.z_vel = (float) -1.8411607E38F;
            p146.x_pos = (float)1.7576623E37F;
            p146.y_pos = (float)1.9915945E38F;
            p146.z_pos = (float) -3.0613825E38F;
            p146.airspeed = (float)2.1750623E38F;
            p146.vel_variance_SET(new float[3], 0);
            p146.pos_variance_SET(new float[3], 0);
            p146.q_SET(new float[4], 0);
            p146.roll_rate = (float)1.0797889E38F;
            p146.pitch_rate = (float) -2.5906665E38F;
            p146.yaw_rate = (float) -7.2594506E37F;
            CommunicationChannel.instance.send(p146); //===============================
            BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.id = (byte)(byte)105;
            p147.battery_function = MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_ALL;
            p147.type = MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIFE;
            p147.temperature = (short)(short)25779;
            p147.voltages_SET(new ushort[10], 0);
            p147.current_battery = (short)(short)15704;
            p147.current_consumed = (int) -1241656198;
            p147.energy_consumed = (int)397342845;
            p147.battery_remaining = (sbyte)(sbyte) - 46;
            CommunicationChannel.instance.send(p147); //===============================
            AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION);
            p148.flight_sw_version = (uint)1559314606U;
            p148.middleware_sw_version = (uint)680841947U;
            p148.os_sw_version = (uint)3661226553U;
            p148.board_version = (uint)117705593U;
            p148.flight_custom_version_SET(new byte[8], 0);
            p148.middleware_custom_version_SET(new byte[8], 0);
            p148.os_custom_version_SET(new byte[8], 0);
            p148.vendor_id = (ushort)(ushort)52874;
            p148.product_id = (ushort)(ushort)21819;
            p148.uid = (ulong)8694945277153666841L;
            p148.uid2_SET(new byte[18], 0, PH);
            CommunicationChannel.instance.send(p148); //===============================
            LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.time_usec = (ulong)1549873834071370174L;
            p149.target_num = (byte)(byte)61;
            p149.frame = MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p149.angle_x = (float) -2.003067E38F;
            p149.angle_y = (float)3.0416937E38F;
            p149.distance = (float)1.2782339E38F;
            p149.size_x = (float) -1.0897533E36F;
            p149.size_y = (float) -9.743385E37F;
            p149.x_SET((float)1.9975192E38F, PH);
            p149.y_SET((float)2.362718E38F, PH);
            p149.z_SET((float)1.6286094E38F, PH);
            p149.q_SET(new float[4], 0, PH);
            p149.type = LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER;
            p149.position_valid_SET((byte)(byte)215, PH);
            CommunicationChannel.instance.send(p149); //===============================
            SCRIPT_ITEM p180 = CommunicationChannel.new_SCRIPT_ITEM();
            PH.setPack(p180);
            p180.target_system = (byte)(byte)42;
            p180.target_component = (byte)(byte)214;
            p180.seq = (ushort)(ushort)64495;
            p180.name_SET("DEMO", PH);
            CommunicationChannel.instance.send(p180); //===============================
            SCRIPT_REQUEST p181 = CommunicationChannel.new_SCRIPT_REQUEST();
            PH.setPack(p181);
            p181.target_system = (byte)(byte)87;
            p181.target_component = (byte)(byte)60;
            p181.seq = (ushort)(ushort)42630;
            CommunicationChannel.instance.send(p181); //===============================
            SCRIPT_REQUEST_LIST p182 = CommunicationChannel.new_SCRIPT_REQUEST_LIST();
            PH.setPack(p182);
            p182.target_system = (byte)(byte)55;
            p182.target_component = (byte)(byte)95;
            CommunicationChannel.instance.send(p182); //===============================
            SCRIPT_COUNT p183 = CommunicationChannel.new_SCRIPT_COUNT();
            PH.setPack(p183);
            p183.target_system = (byte)(byte)223;
            p183.target_component = (byte)(byte)176;
            p183.count = (ushort)(ushort)39571;
            CommunicationChannel.instance.send(p183); //===============================
            SCRIPT_CURRENT p184 = CommunicationChannel.new_SCRIPT_CURRENT();
            PH.setPack(p184);
            p184.seq = (ushort)(ushort)32456;
            CommunicationChannel.instance.send(p184); //===============================
            ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.time_usec = (ulong)91878550341889470L;
            p230.flags = (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL);
            p230.vel_ratio = (float)2.6611746E38F;
            p230.pos_horiz_ratio = (float) -6.446678E37F;
            p230.pos_vert_ratio = (float)3.0398643E38F;
            p230.mag_ratio = (float) -3.203029E38F;
            p230.hagl_ratio = (float) -4.1162955E37F;
            p230.tas_ratio = (float)3.0163531E38F;
            p230.pos_horiz_accuracy = (float) -3.485446E36F;
            p230.pos_vert_accuracy = (float)4.7890907E37F;
            CommunicationChannel.instance.send(p230); //===============================
            WIND_COV p231 = CommunicationChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.time_usec = (ulong)8931573781758596269L;
            p231.wind_x = (float)5.8667874E37F;
            p231.wind_y = (float)1.7047065E37F;
            p231.wind_z = (float)1.8276646E38F;
            p231.var_horiz = (float) -2.4019987E38F;
            p231.var_vert = (float)3.0154589E38F;
            p231.wind_alt = (float) -1.5768071E38F;
            p231.horiz_accuracy = (float)3.1782146E38F;
            p231.vert_accuracy = (float) -3.9321036E37F;
            CommunicationChannel.instance.send(p231); //===============================
            GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.time_usec = (ulong)9007598028435621413L;
            p232.gps_id = (byte)(byte)42;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT);
            p232.time_week_ms = (uint)1381378930U;
            p232.time_week = (ushort)(ushort)10163;
            p232.fix_type = (byte)(byte)39;
            p232.lat = (int) -213764989;
            p232.lon = (int) -519839049;
            p232.alt = (float)2.2676312E37F;
            p232.hdop = (float)7.1681864E37F;
            p232.vdop = (float) -2.1613863E38F;
            p232.vn = (float)3.0402626E38F;
            p232.ve = (float) -1.3598718E37F;
            p232.vd = (float) -1.347639E38F;
            p232.speed_accuracy = (float)2.8525169E38F;
            p232.horiz_accuracy = (float) -1.6795111E38F;
            p232.vert_accuracy = (float)2.3390358E38F;
            p232.satellites_visible = (byte)(byte)193;
            CommunicationChannel.instance.send(p232); //===============================
            GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.flags = (byte)(byte)118;
            p233.len = (byte)(byte)219;
            p233.data__SET(new byte[180], 0);
            CommunicationChannel.instance.send(p233); //===============================
            HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED);
            p234.custom_mode = (uint)2328297386U;
            p234.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING;
            p234.roll = (short)(short) -17337;
            p234.pitch = (short)(short)6907;
            p234.heading = (ushort)(ushort)41814;
            p234.throttle = (sbyte)(sbyte)11;
            p234.heading_sp = (short)(short) -9066;
            p234.latitude = (int) -130005473;
            p234.longitude = (int) -1213140777;
            p234.altitude_amsl = (short)(short) -6073;
            p234.altitude_sp = (short)(short)2473;
            p234.airspeed = (byte)(byte)250;
            p234.airspeed_sp = (byte)(byte)23;
            p234.groundspeed = (byte)(byte)155;
            p234.climb_rate = (sbyte)(sbyte) - 120;
            p234.gps_nsat = (byte)(byte)62;
            p234.gps_fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS;
            p234.battery_remaining = (byte)(byte)240;
            p234.temperature = (sbyte)(sbyte) - 79;
            p234.temperature_air = (sbyte)(sbyte) - 6;
            p234.failsafe = (byte)(byte)58;
            p234.wp_num = (byte)(byte)60;
            p234.wp_distance = (ushort)(ushort)63557;
            CommunicationChannel.instance.send(p234); //===============================
            VIBRATION p241 = CommunicationChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.time_usec = (ulong)5735121704374010170L;
            p241.vibration_x = (float)3.2831664E38F;
            p241.vibration_y = (float) -1.5374992E38F;
            p241.vibration_z = (float)7.0440667E37F;
            p241.clipping_0 = (uint)1317513270U;
            p241.clipping_1 = (uint)1691003303U;
            p241.clipping_2 = (uint)3378789971U;
            CommunicationChannel.instance.send(p241); //===============================
            HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.latitude = (int)1658186957;
            p242.longitude = (int)2031920646;
            p242.altitude = (int) -978857370;
            p242.x = (float) -1.7933074E38F;
            p242.y = (float) -2.1398638E38F;
            p242.z = (float) -5.4280763E37F;
            p242.q_SET(new float[4], 0);
            p242.approach_x = (float) -3.1850685E37F;
            p242.approach_y = (float)2.2852588E38F;
            p242.approach_z = (float) -2.969512E38F;
            p242.time_usec_SET((ulong)8901447261571476194L, PH);
            CommunicationChannel.instance.send(p242); //===============================
            SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.target_system = (byte)(byte)199;
            p243.latitude = (int)1876022747;
            p243.longitude = (int)848599196;
            p243.altitude = (int) -522148467;
            p243.x = (float)6.786049E37F;
            p243.y = (float) -3.070372E38F;
            p243.z = (float)3.0235045E38F;
            p243.q_SET(new float[4], 0);
            p243.approach_x = (float) -3.3308343E38F;
            p243.approach_y = (float) -1.7532001E38F;
            p243.approach_z = (float) -2.4219296E38F;
            p243.time_usec_SET((ulong)4963308190728458164L, PH);
            CommunicationChannel.instance.send(p243); //===============================
            MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.message_id = (ushort)(ushort)19066;
            p244.interval_us = (int)2087152498;
            CommunicationChannel.instance.send(p244); //===============================
            EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.vtol_state = MAV_VTOL_STATE.MAV_VTOL_STATE_FW;
            p245.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF;
            CommunicationChannel.instance.send(p245); //===============================
            ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.ICAO_address = (uint)3485783565U;
            p246.lat = (int)1373861512;
            p246.lon = (int)1050643437;
            p246.altitude_type = ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC;
            p246.altitude = (int)1373441192;
            p246.heading = (ushort)(ushort)53447;
            p246.hor_velocity = (ushort)(ushort)58119;
            p246.ver_velocity = (short)(short) -14016;
            p246.callsign_SET("DEMO", PH);
            p246.emitter_type = ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_GLIDER;
            p246.tslc = (byte)(byte)66;
            p246.flags = (ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE |
                          ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN);
            p246.squawk = (ushort)(ushort)26777;
            CommunicationChannel.instance.send(p246); //===============================
            COLLISION p247 = CommunicationChannel.new_COLLISION();
            PH.setPack(p247);
            p247.src_ = MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT;
            p247.id = (uint)996306672U;
            p247.action = MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_NONE;
            p247.threat_level = (MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH |
                                 MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE);
            p247.time_to_minimum_delta = (float) -1.5925805E38F;
            p247.altitude_minimum_delta = (float) -5.648552E37F;
            p247.horizontal_minimum_delta = (float) -3.239883E38F;
            CommunicationChannel.instance.send(p247); //===============================
            V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.target_network = (byte)(byte)134;
            p248.target_system = (byte)(byte)60;
            p248.target_component = (byte)(byte)165;
            p248.message_type = (ushort)(ushort)16984;
            p248.payload_SET(new byte[249], 0);
            CommunicationChannel.instance.send(p248); //===============================
            MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.address = (ushort)(ushort)14776;
            p249.ver = (byte)(byte)233;
            p249.type = (byte)(byte)115;
            p249.value_SET(new sbyte[32], 0);
            CommunicationChannel.instance.send(p249); //===============================
            DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.name_SET("DEMO", PH);
            p250.time_usec = (ulong)4227332995012562074L;
            p250.x = (float) -2.3757014E38F;
            p250.y = (float)2.4417482E38F;
            p250.z = (float) -1.80783E38F;
            CommunicationChannel.instance.send(p250); //===============================
            NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.time_boot_ms = (uint)3097832590U;
            p251.name_SET("DEMO", PH);
            p251.value = (float) -1.371201E38F;
            CommunicationChannel.instance.send(p251); //===============================
            NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.time_boot_ms = (uint)3361295093U;
            p252.name_SET("DEMO", PH);
            p252.value = (int)138811397;
            CommunicationChannel.instance.send(p252); //===============================
            STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.severity = MAV_SEVERITY.MAV_SEVERITY_INFO;
            p253.text_SET("DEMO", PH);
            CommunicationChannel.instance.send(p253); //===============================
            DEBUG p254 = CommunicationChannel.new_DEBUG();
            PH.setPack(p254);
            p254.time_boot_ms = (uint)4158568151U;
            p254.ind = (byte)(byte)96;
            p254.value = (float)2.9745847E38F;
            CommunicationChannel.instance.send(p254); //===============================
            SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.target_system = (byte)(byte)207;
            p256.target_component = (byte)(byte)1;
            p256.secret_key_SET(new byte[32], 0);
            p256.initial_timestamp = (ulong)2269552679016005087L;
            CommunicationChannel.instance.send(p256); //===============================
            BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.time_boot_ms = (uint)246850619U;
            p257.last_change_ms = (uint)3054911475U;
            p257.state = (byte)(byte)76;
            CommunicationChannel.instance.send(p257); //===============================
            PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_system = (byte)(byte)116;
            p258.target_component = (byte)(byte)153;
            p258.tune_SET("DEMO", PH);
            CommunicationChannel.instance.send(p258); //===============================
            CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.time_boot_ms = (uint)2681785178U;
            p259.vendor_name_SET(new byte[32], 0);
            p259.model_name_SET(new byte[32], 0);
            p259.firmware_version = (uint)3851468099U;
            p259.focal_length = (float)2.2578808E38F;
            p259.sensor_size_h = (float)2.0141434E37F;
            p259.sensor_size_v = (float) -1.8523736E38F;
            p259.resolution_h = (ushort)(ushort)30678;
            p259.resolution_v = (ushort)(ushort)27120;
            p259.lens_id = (byte)(byte)4;
            p259.flags = (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE |
                          CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
                          CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO);
            p259.cam_definition_version = (ushort)(ushort)43447;
            p259.cam_definition_uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p259); //===============================
            CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.time_boot_ms = (uint)4153024586U;
            p260.mode_id = (CAMERA_MODE.CAMERA_MODE_VIDEO);
            CommunicationChannel.instance.send(p260); //===============================
            STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.time_boot_ms = (uint)3471792751U;
            p261.storage_id = (byte)(byte)226;
            p261.storage_count = (byte)(byte)142;
            p261.status = (byte)(byte)108;
            p261.total_capacity = (float) -1.3999234E38F;
            p261.used_capacity = (float)3.3330013E38F;
            p261.available_capacity = (float) -3.236121E38F;
            p261.read_speed = (float) -5.444628E37F;
            p261.write_speed = (float)1.6254826E38F;
            CommunicationChannel.instance.send(p261); //===============================
            CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.time_boot_ms = (uint)900607788U;
            p262.image_status = (byte)(byte)187;
            p262.video_status = (byte)(byte)226;
            p262.image_interval = (float)3.1123289E38F;
            p262.recording_time_ms = (uint)4211327859U;
            p262.available_capacity = (float)7.8667095E37F;
            CommunicationChannel.instance.send(p262); //===============================
            CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.time_boot_ms = (uint)4087653929U;
            p263.time_utc = (ulong)4910726395266782400L;
            p263.camera_id = (byte)(byte)66;
            p263.lat = (int)490642447;
            p263.lon = (int) -548304138;
            p263.alt = (int)1117018590;
            p263.relative_alt = (int)370206915;
            p263.q_SET(new float[4], 0);
            p263.image_index = (int) -1296522082;
            p263.capture_result = (sbyte)(sbyte) - 63;
            p263.file_url_SET("DEMO", PH);
            CommunicationChannel.instance.send(p263); //===============================
            FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.time_boot_ms = (uint)364953810U;
            p264.arming_time_utc = (ulong)8702287825800855223L;
            p264.takeoff_time_utc = (ulong)8962170268197400135L;
            p264.flight_uuid = (ulong)2741542020564488395L;
            CommunicationChannel.instance.send(p264); //===============================
            MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.time_boot_ms = (uint)652510105U;
            p265.roll = (float)2.4679508E38F;
            p265.pitch = (float) -1.9333162E38F;
            p265.yaw = (float) -2.6479416E38F;
            CommunicationChannel.instance.send(p265); //===============================
            LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.target_system = (byte)(byte)221;
            p266.target_component = (byte)(byte)54;
            p266.sequence = (ushort)(ushort)44966;
            p266.length = (byte)(byte)137;
            p266.first_message_offset = (byte)(byte)71;
            p266.data__SET(new byte[249], 0);
            CommunicationChannel.instance.send(p266); //===============================
            LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.target_system = (byte)(byte)127;
            p267.target_component = (byte)(byte)116;
            p267.sequence = (ushort)(ushort)62684;
            p267.length = (byte)(byte)127;
            p267.first_message_offset = (byte)(byte)35;
            p267.data__SET(new byte[249], 0);
            CommunicationChannel.instance.send(p267); //===============================
            LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_system = (byte)(byte)55;
            p268.target_component = (byte)(byte)7;
            p268.sequence = (ushort)(ushort)4150;
            CommunicationChannel.instance.send(p268); //===============================
            VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.camera_id = (byte)(byte)31;
            p269.status = (byte)(byte)39;
            p269.framerate = (float) -2.578424E38F;
            p269.resolution_h = (ushort)(ushort)57770;
            p269.resolution_v = (ushort)(ushort)28038;
            p269.bitrate = (uint)3975963102U;
            p269.rotation = (ushort)(ushort)21973;
            p269.uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p269); //===============================
            SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.target_system = (byte)(byte)136;
            p270.target_component = (byte)(byte)8;
            p270.camera_id = (byte)(byte)201;
            p270.framerate = (float)2.346548E38F;
            p270.resolution_h = (ushort)(ushort)15788;
            p270.resolution_v = (ushort)(ushort)45244;
            p270.bitrate = (uint)2442755652U;
            p270.rotation = (ushort)(ushort)51467;
            p270.uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p270); //===============================
            WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.ssid_SET("DEMO", PH);
            p299.password_SET("DEMO", PH);
            CommunicationChannel.instance.send(p299); //===============================
            PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.version = (ushort)(ushort)48383;
            p300.min_version = (ushort)(ushort)64104;
            p300.max_version = (ushort)(ushort)61145;
            p300.spec_version_hash_SET(new byte[8], 0);
            p300.library_version_hash_SET(new byte[8], 0);
            CommunicationChannel.instance.send(p300); //===============================
            UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.time_usec = (ulong)436578957027572630L;
            p310.uptime_sec = (uint)2634979178U;
            p310.health = UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR;
            p310.mode = UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE;
            p310.sub_mode = (byte)(byte)46;
            p310.vendor_specific_status_code = (ushort)(ushort)8854;
            CommunicationChannel.instance.send(p310); //===============================
            UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.time_usec = (ulong)7761361041888297956L;
            p311.uptime_sec = (uint)680717513U;
            p311.name_SET("DEMO", PH);
            p311.hw_version_major = (byte)(byte)17;
            p311.hw_version_minor = (byte)(byte)33;
            p311.hw_unique_id_SET(new byte[16], 0);
            p311.sw_version_major = (byte)(byte)162;
            p311.sw_version_minor = (byte)(byte)5;
            p311.sw_vcs_commit = (uint)602066770U;
            CommunicationChannel.instance.send(p311); //===============================
            PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.target_system = (byte)(byte)255;
            p320.target_component = (byte)(byte)21;
            p320.param_id_SET("DEMO", PH);
            p320.param_index = (short)(short)24145;
            CommunicationChannel.instance.send(p320); //===============================
            PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_system = (byte)(byte)250;
            p321.target_component = (byte)(byte)68;
            CommunicationChannel.instance.send(p321); //===============================
            PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_id_SET("DEMO", PH);
            p322.param_value_SET("DEMO", PH);
            p322.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16;
            p322.param_count = (ushort)(ushort)43547;
            p322.param_index = (ushort)(ushort)32420;
            CommunicationChannel.instance.send(p322); //===============================
            PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.target_system = (byte)(byte)233;
            p323.target_component = (byte)(byte)226;
            p323.param_id_SET("DEMO", PH);
            p323.param_value_SET("DEMO", PH);
            p323.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64;
            CommunicationChannel.instance.send(p323); //===============================
            PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_id_SET("DEMO", PH);
            p324.param_value_SET("DEMO", PH);
            p324.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT64;
            p324.param_result = PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED;
            CommunicationChannel.instance.send(p324); //===============================
            OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.time_usec = (ulong)344268551457717295L;
            p330.sensor_type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED;
            p330.distances_SET(new ushort[72], 0);
            p330.increment = (byte)(byte)188;
            p330.min_distance = (ushort)(ushort)29826;
            p330.max_distance = (ushort)(ushort)63252;
            CommunicationChannel.instance.send(p330); //===============================
        }
    }
}

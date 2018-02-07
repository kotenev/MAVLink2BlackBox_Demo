
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
            POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.time_boot_ms = (uint)3808052113U;
            p3.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_ENU;
            p3.type_mask = (ushort)(ushort)50184;
            p3.x = (float) -2.7641927E38F;
            p3.y = (float) -1.5604041E38F;
            p3.z = (float)1.8782582E38F;
            p3.vx = (float) -3.3240803E38F;
            p3.vy = (float)2.5124192E38F;
            p3.vz = (float) -2.2888356E38F;
            p3.afx = (float) -2.5267449E38F;
            p3.afy = (float)3.3658E38F;
            p3.afz = (float) -2.3240492E38F;
            p3.yaw = (float) -2.1103796E38F;
            p3.yaw_rate = (float)1.69205E37F;
            CommunicationChannel.instance.send(p3); //===============================
            COMMAND_ACK p77 = CommunicationChannel.new_COMMAND_ACK();
            PH.setPack(p77);
            p77.command = MAV_CMD.MAV_CMD_DO_LAND_START;
            p77.result = MAV_RESULT.MAV_RESULT_TEMPORARILY_REJECTED;
            p77.progress_SET((byte)(byte)15, PH);
            p77.result_param2_SET((int) -1987892784, PH);
            p77.target_system_SET((byte)(byte)97, PH);
            p77.target_component_SET((byte)(byte)175, PH);
            CommunicationChannel.instance.send(p77); //===============================
            MANUAL_SETPOINT p81 = CommunicationChannel.new_MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.time_boot_ms = (uint)3055579862U;
            p81.roll = (float) -5.417315E37F;
            p81.pitch = (float)2.5722732E38F;
            p81.yaw = (float)8.5448385E36F;
            p81.thrust = (float)2.9685118E38F;
            p81.mode_switch = (byte)(byte)75;
            p81.manual_override_switch = (byte)(byte)49;
            CommunicationChannel.instance.send(p81); //===============================
            SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.time_boot_ms = (uint)3570170553U;
            p82.target_system = (byte)(byte)148;
            p82.target_component = (byte)(byte)218;
            p82.type_mask = (byte)(byte)188;
            p82.q_SET(new float[4], 0);
            p82.body_roll_rate = (float) -1.5754462E38F;
            p82.body_pitch_rate = (float)1.934127E38F;
            p82.body_yaw_rate = (float) -1.0865879E38F;
            p82.thrust = (float)2.9304025E38F;
            CommunicationChannel.instance.send(p82); //===============================
            ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.time_boot_ms = (uint)2379032917U;
            p83.type_mask = (byte)(byte)121;
            p83.q_SET(new float[4], 0);
            p83.body_roll_rate = (float) -1.6402731E38F;
            p83.body_pitch_rate = (float) -8.670166E37F;
            p83.body_yaw_rate = (float)1.2233455E38F;
            p83.thrust = (float)1.236112E38F;
            CommunicationChannel.instance.send(p83); //===============================
            SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.time_boot_ms = (uint)486465362U;
            p84.target_system = (byte)(byte)136;
            p84.target_component = (byte)(byte)151;
            p84.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_ENU;
            p84.type_mask = (ushort)(ushort)55980;
            p84.x = (float) -1.3346966E38F;
            p84.y = (float)1.5920504E38F;
            p84.z = (float) -2.4321132E37F;
            p84.vx = (float)2.2326739E38F;
            p84.vy = (float) -1.5441823E38F;
            p84.vz = (float)3.4899032E37F;
            p84.afx = (float) -3.248308E37F;
            p84.afy = (float)2.947188E38F;
            p84.afz = (float) -3.5879096E37F;
            p84.yaw = (float)6.960939E37F;
            p84.yaw_rate = (float)1.3216064E38F;
            CommunicationChannel.instance.send(p84); //===============================
            SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.time_boot_ms = (uint)1955034901U;
            p86.target_system = (byte)(byte)167;
            p86.target_component = (byte)(byte)162;
            p86.coordinate_frame = MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p86.type_mask = (ushort)(ushort)37653;
            p86.lat_int = (int)167726205;
            p86.lon_int = (int)1000831124;
            p86.alt = (float)1.7424204E38F;
            p86.vx = (float) -1.9297956E38F;
            p86.vy = (float) -1.0605195E38F;
            p86.vz = (float)1.6774952E38F;
            p86.afx = (float) -8.3364603E37F;
            p86.afy = (float) -6.4631024E37F;
            p86.afz = (float) -5.4260506E37F;
            p86.yaw = (float) -2.6798489E38F;
            p86.yaw_rate = (float) -3.0725356E38F;
            CommunicationChannel.instance.send(p86); //===============================
            POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.time_boot_ms = (uint)3785482723U;
            p87.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_INT;
            p87.type_mask = (ushort)(ushort)7688;
            p87.lat_int = (int)1154802320;
            p87.lon_int = (int) -2004885999;
            p87.alt = (float)1.1601644E38F;
            p87.vx = (float)3.389331E38F;
            p87.vy = (float) -7.347005E37F;
            p87.vz = (float) -2.4028343E38F;
            p87.afx = (float) -5.518815E37F;
            p87.afy = (float) -2.292497E38F;
            p87.afz = (float) -2.2598403E38F;
            p87.yaw = (float)1.5780789E38F;
            p87.yaw_rate = (float)6.4529414E37F;
            CommunicationChannel.instance.send(p87); //===============================
            LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.time_boot_ms = (uint)2334169542U;
            p89.x = (float)1.1439082E38F;
            p89.y = (float)1.2604742E38F;
            p89.z = (float)2.85016E38F;
            p89.roll = (float)2.878676E38F;
            p89.pitch = (float) -3.0577625E38F;
            p89.yaw = (float)3.26984E38F;
            CommunicationChannel.instance.send(p89); //===============================
            HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.time_usec = (ulong)5332947671865040305L;
            p90.roll = (float) -1.6016947E38F;
            p90.pitch = (float) -2.2296136E38F;
            p90.yaw = (float) -2.722925E38F;
            p90.rollspeed = (float)2.7235775E38F;
            p90.pitchspeed = (float) -1.321987E37F;
            p90.yawspeed = (float)9.069954E37F;
            p90.lat = (int) -1159360618;
            p90.lon = (int) -152807049;
            p90.alt = (int) -2111855899;
            p90.vx = (short)(short)14368;
            p90.vy = (short)(short)7255;
            p90.vz = (short)(short) -23493;
            p90.xacc = (short)(short)1733;
            p90.yacc = (short)(short) -7666;
            p90.zacc = (short)(short)11352;
            CommunicationChannel.instance.send(p90); //===============================
            HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.time_usec = (ulong)7251700572794564085L;
            p91.roll_ailerons = (float)1.528655E38F;
            p91.pitch_elevator = (float) -3.1771725E38F;
            p91.yaw_rudder = (float)1.2972346E38F;
            p91.throttle = (float) -8.133437E37F;
            p91.aux1 = (float) -1.1811641E38F;
            p91.aux2 = (float)1.7676732E38F;
            p91.aux3 = (float) -2.0032315E37F;
            p91.aux4 = (float) -2.3928361E38F;
            p91.mode = MAV_MODE.MAV_MODE_AUTO_DISARMED;
            p91.nav_mode = (byte)(byte)112;
            CommunicationChannel.instance.send(p91); //===============================
            HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.time_usec = (ulong)4965887090911306090L;
            p92.chan1_raw = (ushort)(ushort)1336;
            p92.chan2_raw = (ushort)(ushort)18988;
            p92.chan3_raw = (ushort)(ushort)51323;
            p92.chan4_raw = (ushort)(ushort)42282;
            p92.chan5_raw = (ushort)(ushort)62310;
            p92.chan6_raw = (ushort)(ushort)887;
            p92.chan7_raw = (ushort)(ushort)610;
            p92.chan8_raw = (ushort)(ushort)46549;
            p92.chan9_raw = (ushort)(ushort)9080;
            p92.chan10_raw = (ushort)(ushort)1694;
            p92.chan11_raw = (ushort)(ushort)37973;
            p92.chan12_raw = (ushort)(ushort)33012;
            p92.rssi = (byte)(byte)193;
            CommunicationChannel.instance.send(p92); //===============================
            HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.time_usec = (ulong)7613941268260556541L;
            p93.controls_SET(new float[16], 0);
            p93.mode = MAV_MODE.MAV_MODE_AUTO_ARMED;
            p93.flags = (ulong)882493635544234442L;
            CommunicationChannel.instance.send(p93); //===============================
            OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.time_usec = (ulong)5237315878361118519L;
            p100.sensor_id = (byte)(byte)159;
            p100.flow_x = (short)(short)24311;
            p100.flow_y = (short)(short)32599;
            p100.flow_comp_m_x = (float) -1.5880936E38F;
            p100.flow_comp_m_y = (float) -2.212176E38F;
            p100.quality = (byte)(byte)148;
            p100.ground_distance = (float) -2.9954428E38F;
            p100.flow_rate_x_SET((float)1.4611285E38F, PH);
            p100.flow_rate_y_SET((float) -2.0244018E37F, PH);
            CommunicationChannel.instance.send(p100); //===============================
            GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.usec = (ulong)1522712248014496055L;
            p101.x = (float) -4.9156595E37F;
            p101.y = (float)2.1343508E38F;
            p101.z = (float)1.5399491E38F;
            p101.roll = (float)2.073249E38F;
            p101.pitch = (float) -3.7757719E37F;
            p101.yaw = (float)2.1259135E38F;
            CommunicationChannel.instance.send(p101); //===============================
            VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.usec = (ulong)3252143911900955726L;
            p102.x = (float)2.3598746E38F;
            p102.y = (float)6.5742444E37F;
            p102.z = (float)2.8211183E38F;
            p102.roll = (float)1.8357163E38F;
            p102.pitch = (float) -3.4010159E38F;
            p102.yaw = (float)1.5434902E38F;
            CommunicationChannel.instance.send(p102); //===============================
            VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.usec = (ulong)2330847569500323709L;
            p103.x = (float)2.3184506E38F;
            p103.y = (float)5.2419563E37F;
            p103.z = (float)1.180571E38F;
            CommunicationChannel.instance.send(p103); //===============================
            VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.usec = (ulong)4537737438135232732L;
            p104.x = (float)2.2298252E38F;
            p104.y = (float)1.8427285E38F;
            p104.z = (float)3.2500117E38F;
            p104.roll = (float)3.2605975E38F;
            p104.pitch = (float)1.7104554E38F;
            p104.yaw = (float) -3.1275067E37F;
            CommunicationChannel.instance.send(p104); //===============================
            HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.time_usec = (ulong)8173744557744815585L;
            p105.xacc = (float)2.7005862E38F;
            p105.yacc = (float) -1.7427396E37F;
            p105.zacc = (float)1.9058196E38F;
            p105.xgyro = (float) -1.4813662E38F;
            p105.ygyro = (float)2.106666E38F;
            p105.zgyro = (float) -4.6299515E36F;
            p105.xmag = (float) -1.6894275E38F;
            p105.ymag = (float) -2.4503986E38F;
            p105.zmag = (float)9.233501E37F;
            p105.abs_pressure = (float)2.2462612E38F;
            p105.diff_pressure = (float)1.5125398E38F;
            p105.pressure_alt = (float)9.806971E37F;
            p105.temperature = (float)2.264956E38F;
            p105.fields_updated = (ushort)(ushort)41213;
            CommunicationChannel.instance.send(p105); //===============================
            OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.time_usec = (ulong)936778554864628636L;
            p106.sensor_id = (byte)(byte)83;
            p106.integration_time_us = (uint)2247370514U;
            p106.integrated_x = (float)1.3048119E38F;
            p106.integrated_y = (float) -2.6097216E38F;
            p106.integrated_xgyro = (float)4.687655E37F;
            p106.integrated_ygyro = (float)3.2240952E35F;
            p106.integrated_zgyro = (float)1.118068E38F;
            p106.temperature = (short)(short) -13311;
            p106.quality = (byte)(byte)38;
            p106.time_delta_distance_us = (uint)2265243116U;
            p106.distance = (float) -2.837563E38F;
            CommunicationChannel.instance.send(p106); //===============================
            HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.time_usec = (ulong)3281254549512001803L;
            p107.xacc = (float)3.0738935E38F;
            p107.yacc = (float)3.6135699E37F;
            p107.zacc = (float) -1.6404221E36F;
            p107.xgyro = (float) -1.1436758E38F;
            p107.ygyro = (float)1.172006E38F;
            p107.zgyro = (float)3.2889576E38F;
            p107.xmag = (float)2.5842696E38F;
            p107.ymag = (float) -2.490278E38F;
            p107.zmag = (float)3.3685077E38F;
            p107.abs_pressure = (float)1.8806927E38F;
            p107.diff_pressure = (float) -3.1373174E38F;
            p107.pressure_alt = (float) -2.7629234E38F;
            p107.temperature = (float) -1.737085E38F;
            p107.fields_updated = (uint)988644496U;
            CommunicationChannel.instance.send(p107); //===============================
            SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.q1 = (float)1.6437117E38F;
            p108.q2 = (float) -3.0820831E38F;
            p108.q3 = (float) -3.0026757E38F;
            p108.q4 = (float)4.663472E37F;
            p108.roll = (float) -2.5763246E38F;
            p108.pitch = (float)2.8844815E38F;
            p108.yaw = (float)2.2486008E38F;
            p108.xacc = (float)3.0424551E38F;
            p108.yacc = (float)3.1923596E38F;
            p108.zacc = (float)3.1005853E38F;
            p108.xgyro = (float)4.2327647E37F;
            p108.ygyro = (float)1.0698584E38F;
            p108.zgyro = (float) -2.2524713E38F;
            p108.lat = (float) -2.408475E38F;
            p108.lon = (float) -2.2125235E38F;
            p108.alt = (float)2.3799998E38F;
            p108.std_dev_horz = (float)1.1479085E38F;
            p108.std_dev_vert = (float) -3.285551E38F;
            p108.vn = (float) -3.3232982E38F;
            p108.ve = (float)2.3111228E38F;
            p108.vd = (float)2.6365155E37F;
            CommunicationChannel.instance.send(p108); //===============================
            RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.rssi = (byte)(byte)6;
            p109.remrssi = (byte)(byte)57;
            p109.txbuf = (byte)(byte)224;
            p109.noise = (byte)(byte)62;
            p109.remnoise = (byte)(byte)130;
            p109.rxerrors = (ushort)(ushort)37374;
            p109.fixed_ = (ushort)(ushort)40642;
            CommunicationChannel.instance.send(p109); //===============================
            FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_network = (byte)(byte)193;
            p110.target_system = (byte)(byte)59;
            p110.target_component = (byte)(byte)200;
            p110.payload_SET(new byte[251], 0);
            CommunicationChannel.instance.send(p110); //===============================
            TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.tc1 = (long) -6603434832640983787L;
            p111.ts1 = (long)4126673039982138520L;
            CommunicationChannel.instance.send(p111); //===============================
            CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.time_usec = (ulong)1687276188801425452L;
            p112.seq = (uint)3267914995U;
            CommunicationChannel.instance.send(p112); //===============================
            HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.time_usec = (ulong)6630905829355206026L;
            p113.fix_type = (byte)(byte)93;
            p113.lat = (int) -1805778964;
            p113.lon = (int) -30788335;
            p113.alt = (int)1440309454;
            p113.eph = (ushort)(ushort)50527;
            p113.epv = (ushort)(ushort)4800;
            p113.vel = (ushort)(ushort)3016;
            p113.vn = (short)(short)16909;
            p113.ve = (short)(short)27369;
            p113.vd = (short)(short)5726;
            p113.cog = (ushort)(ushort)25106;
            p113.satellites_visible = (byte)(byte)119;
            CommunicationChannel.instance.send(p113); //===============================
            HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.time_usec = (ulong)8348193825572143093L;
            p114.sensor_id = (byte)(byte)141;
            p114.integration_time_us = (uint)2218975319U;
            p114.integrated_x = (float)2.3927363E38F;
            p114.integrated_y = (float)1.216387E38F;
            p114.integrated_xgyro = (float) -3.2336692E37F;
            p114.integrated_ygyro = (float) -2.5558256E38F;
            p114.integrated_zgyro = (float)2.2576969E38F;
            p114.temperature = (short)(short) -15545;
            p114.quality = (byte)(byte)25;
            p114.time_delta_distance_us = (uint)1813764127U;
            p114.distance = (float) -1.7417791E38F;
            CommunicationChannel.instance.send(p114); //===============================
            HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.time_usec = (ulong)249964377541709929L;
            p115.attitude_quaternion_SET(new float[4], 0);
            p115.rollspeed = (float)3.2881771E38F;
            p115.pitchspeed = (float) -1.3296419E38F;
            p115.yawspeed = (float)3.33223E37F;
            p115.lat = (int)1304818565;
            p115.lon = (int) -1239492940;
            p115.alt = (int) -1771052626;
            p115.vx = (short)(short)2442;
            p115.vy = (short)(short) -8843;
            p115.vz = (short)(short)20019;
            p115.ind_airspeed = (ushort)(ushort)49924;
            p115.true_airspeed = (ushort)(ushort)24871;
            p115.xacc = (short)(short)24443;
            p115.yacc = (short)(short)19396;
            p115.zacc = (short)(short) -5833;
            CommunicationChannel.instance.send(p115); //===============================
            SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.time_boot_ms = (uint)1564263102U;
            p116.xacc = (short)(short)10822;
            p116.yacc = (short)(short) -27692;
            p116.zacc = (short)(short)11925;
            p116.xgyro = (short)(short)24112;
            p116.ygyro = (short)(short)241;
            p116.zgyro = (short)(short)29121;
            p116.xmag = (short)(short)20541;
            p116.ymag = (short)(short)6303;
            p116.zmag = (short)(short)3244;
            CommunicationChannel.instance.send(p116); //===============================
            LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.target_system = (byte)(byte)242;
            p117.target_component = (byte)(byte)47;
            p117.start = (ushort)(ushort)37415;
            p117.end = (ushort)(ushort)52139;
            CommunicationChannel.instance.send(p117); //===============================
            LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.id = (ushort)(ushort)9550;
            p118.num_logs = (ushort)(ushort)38428;
            p118.last_log_num = (ushort)(ushort)19052;
            p118.time_utc = (uint)2941973894U;
            p118.size = (uint)3088252033U;
            CommunicationChannel.instance.send(p118); //===============================
            LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.target_system = (byte)(byte)63;
            p119.target_component = (byte)(byte)145;
            p119.id = (ushort)(ushort)27454;
            p119.ofs = (uint)299021634U;
            p119.count = (uint)3188612793U;
            CommunicationChannel.instance.send(p119); //===============================
            LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.id = (ushort)(ushort)62492;
            p120.ofs = (uint)2156995811U;
            p120.count = (byte)(byte)62;
            p120.data__SET(new byte[90], 0);
            CommunicationChannel.instance.send(p120); //===============================
            LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_system = (byte)(byte)213;
            p121.target_component = (byte)(byte)199;
            CommunicationChannel.instance.send(p121); //===============================
            LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_system = (byte)(byte)66;
            p122.target_component = (byte)(byte)254;
            CommunicationChannel.instance.send(p122); //===============================
            GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.target_system = (byte)(byte)184;
            p123.target_component = (byte)(byte)87;
            p123.len = (byte)(byte)249;
            p123.data__SET(new byte[110], 0);
            CommunicationChannel.instance.send(p123); //===============================
            GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.time_usec = (ulong)3314234651057856028L;
            p124.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX;
            p124.lat = (int)1201905611;
            p124.lon = (int)1366754372;
            p124.alt = (int)1241225059;
            p124.eph = (ushort)(ushort)12049;
            p124.epv = (ushort)(ushort)3236;
            p124.vel = (ushort)(ushort)10301;
            p124.cog = (ushort)(ushort)15201;
            p124.satellites_visible = (byte)(byte)28;
            p124.dgps_numch = (byte)(byte)238;
            p124.dgps_age = (uint)509229788U;
            CommunicationChannel.instance.send(p124); //===============================
            POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vcc = (ushort)(ushort)43900;
            p125.Vservo = (ushort)(ushort)36167;
            p125.flags = (MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID |
                          MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED);
            CommunicationChannel.instance.send(p125); //===============================
            SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.device = SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1;
            p126.flags = (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI);
            p126.timeout = (ushort)(ushort)14823;
            p126.baudrate = (uint)2928015282U;
            p126.count = (byte)(byte)173;
            p126.data__SET(new byte[70], 0);
            CommunicationChannel.instance.send(p126); //===============================
            GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.time_last_baseline_ms = (uint)368834854U;
            p127.rtk_receiver_id = (byte)(byte)73;
            p127.wn = (ushort)(ushort)36401;
            p127.tow = (uint)469384280U;
            p127.rtk_health = (byte)(byte)70;
            p127.rtk_rate = (byte)(byte)235;
            p127.nsats = (byte)(byte)182;
            p127.baseline_coords_type = (byte)(byte)188;
            p127.baseline_a_mm = (int)2143603042;
            p127.baseline_b_mm = (int)993145803;
            p127.baseline_c_mm = (int) -1481990932;
            p127.accuracy = (uint)2012922804U;
            p127.iar_num_hypotheses = (int) -1918191370;
            CommunicationChannel.instance.send(p127); //===============================
            GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.time_last_baseline_ms = (uint)432738868U;
            p128.rtk_receiver_id = (byte)(byte)35;
            p128.wn = (ushort)(ushort)40679;
            p128.tow = (uint)1049700131U;
            p128.rtk_health = (byte)(byte)109;
            p128.rtk_rate = (byte)(byte)98;
            p128.nsats = (byte)(byte)126;
            p128.baseline_coords_type = (byte)(byte)42;
            p128.baseline_a_mm = (int)1698103682;
            p128.baseline_b_mm = (int)908981577;
            p128.baseline_c_mm = (int)769828518;
            p128.accuracy = (uint)2004709359U;
            p128.iar_num_hypotheses = (int) -66008938;
            CommunicationChannel.instance.send(p128); //===============================
            SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.time_boot_ms = (uint)2850203787U;
            p129.xacc = (short)(short)6550;
            p129.yacc = (short)(short) -6774;
            p129.zacc = (short)(short)10727;
            p129.xgyro = (short)(short) -22890;
            p129.ygyro = (short)(short)22166;
            p129.zgyro = (short)(short)27528;
            p129.xmag = (short)(short)16361;
            p129.ymag = (short)(short)29137;
            p129.zmag = (short)(short) -5221;
            CommunicationChannel.instance.send(p129); //===============================
            DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.type = (byte)(byte)213;
            p130.size = (uint)3164215555U;
            p130.width = (ushort)(ushort)29366;
            p130.height = (ushort)(ushort)36505;
            p130.packets = (ushort)(ushort)9769;
            p130.payload = (byte)(byte)243;
            p130.jpg_quality = (byte)(byte)86;
            CommunicationChannel.instance.send(p130); //===============================
            ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.seqnr = (ushort)(ushort)40564;
            p131.data__SET(new byte[253], 0);
            CommunicationChannel.instance.send(p131); //===============================
            DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.time_boot_ms = (uint)341147099U;
            p132.min_distance = (ushort)(ushort)9715;
            p132.max_distance = (ushort)(ushort)37743;
            p132.current_distance = (ushort)(ushort)23187;
            p132.type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER;
            p132.id = (byte)(byte)63;
            p132.orientation = MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_90_YAW_270;
            p132.covariance = (byte)(byte)243;
            CommunicationChannel.instance.send(p132); //===============================
            TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.lat = (int) -1911044976;
            p133.lon = (int) -683718504;
            p133.grid_spacing = (ushort)(ushort)39927;
            p133.mask = (ulong)5635889990253505797L;
            CommunicationChannel.instance.send(p133); //===============================
            TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.lat = (int) -718586208;
            p134.lon = (int) -1265315296;
            p134.grid_spacing = (ushort)(ushort)36484;
            p134.gridbit = (byte)(byte)22;
            p134.data__SET(new short[16], 0);
            CommunicationChannel.instance.send(p134); //===============================
            TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lat = (int)867750903;
            p135.lon = (int)87832440;
            CommunicationChannel.instance.send(p135); //===============================
            TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.lat = (int) -672039022;
            p136.lon = (int)1348621761;
            p136.spacing = (ushort)(ushort)41708;
            p136.terrain_height = (float)3.3130028E37F;
            p136.current_height = (float)1.704331E38F;
            p136.pending = (ushort)(ushort)48261;
            p136.loaded = (ushort)(ushort)60044;
            CommunicationChannel.instance.send(p136); //===============================
            SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.time_boot_ms = (uint)261603525U;
            p137.press_abs = (float) -3.3730426E38F;
            p137.press_diff = (float)1.8356942E38F;
            p137.temperature = (short)(short) -27884;
            CommunicationChannel.instance.send(p137); //===============================
            ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.time_usec = (ulong)4344283745065174614L;
            p138.q_SET(new float[4], 0);
            p138.x = (float)1.991025E38F;
            p138.y = (float)2.478633E38F;
            p138.z = (float)3.364016E38F;
            CommunicationChannel.instance.send(p138); //===============================
            SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.time_usec = (ulong)6581270872627150257L;
            p139.group_mlx = (byte)(byte)40;
            p139.target_system = (byte)(byte)37;
            p139.target_component = (byte)(byte)1;
            p139.controls_SET(new float[8], 0);
            CommunicationChannel.instance.send(p139); //===============================
            ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.time_usec = (ulong)5308526924170164074L;
            p140.group_mlx = (byte)(byte)118;
            p140.controls_SET(new float[8], 0);
            CommunicationChannel.instance.send(p140); //===============================
            ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.time_usec = (ulong)248295237869402081L;
            p141.altitude_monotonic = (float)2.7466429E38F;
            p141.altitude_amsl = (float)8.128483E37F;
            p141.altitude_local = (float)1.643797E37F;
            p141.altitude_relative = (float) -1.3849191E38F;
            p141.altitude_terrain = (float) -6.832295E37F;
            p141.bottom_clearance = (float)2.5781168E38F;
            CommunicationChannel.instance.send(p141); //===============================
            RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.request_id = (byte)(byte)69;
            p142.uri_type = (byte)(byte)225;
            p142.uri_SET(new byte[120], 0);
            p142.transfer_type = (byte)(byte)91;
            p142.storage_SET(new byte[120], 0);
            CommunicationChannel.instance.send(p142); //===============================
            SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.time_boot_ms = (uint)326744595U;
            p143.press_abs = (float)1.1681918E38F;
            p143.press_diff = (float)1.1051084E38F;
            p143.temperature = (short)(short)6430;
            CommunicationChannel.instance.send(p143); //===============================
            FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.timestamp = (ulong)4443060785626679527L;
            p144.est_capabilities = (byte)(byte)252;
            p144.lat = (int) -1923855556;
            p144.lon = (int) -475106943;
            p144.alt = (float) -2.325258E38F;
            p144.vel_SET(new float[3], 0);
            p144.acc_SET(new float[3], 0);
            p144.attitude_q_SET(new float[4], 0);
            p144.rates_SET(new float[3], 0);
            p144.position_cov_SET(new float[3], 0);
            p144.custom_state = (ulong)707295720235790733L;
            CommunicationChannel.instance.send(p144); //===============================
            CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.time_usec = (ulong)8283621802979647712L;
            p146.x_acc = (float) -2.276615E38F;
            p146.y_acc = (float) -1.1413284E38F;
            p146.z_acc = (float)2.5346193E38F;
            p146.x_vel = (float)2.854421E37F;
            p146.y_vel = (float) -5.8104753E37F;
            p146.z_vel = (float) -1.1846755E38F;
            p146.x_pos = (float) -2.753369E38F;
            p146.y_pos = (float) -8.603148E37F;
            p146.z_pos = (float) -1.988005E38F;
            p146.airspeed = (float) -8.3193206E37F;
            p146.vel_variance_SET(new float[3], 0);
            p146.pos_variance_SET(new float[3], 0);
            p146.q_SET(new float[4], 0);
            p146.roll_rate = (float) -2.8091926E38F;
            p146.pitch_rate = (float) -1.8271737E38F;
            p146.yaw_rate = (float) -3.246839E38F;
            CommunicationChannel.instance.send(p146); //===============================
            BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.id = (byte)(byte)161;
            p147.battery_function = MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_AVIONICS;
            p147.type = MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LION;
            p147.temperature = (short)(short) -6939;
            p147.voltages_SET(new ushort[10], 0);
            p147.current_battery = (short)(short)19092;
            p147.current_consumed = (int) -2009522773;
            p147.energy_consumed = (int)1055321718;
            p147.battery_remaining = (sbyte)(sbyte)103;
            CommunicationChannel.instance.send(p147); //===============================
            AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED);
            p148.flight_sw_version = (uint)2777725531U;
            p148.middleware_sw_version = (uint)2779851459U;
            p148.os_sw_version = (uint)686142573U;
            p148.board_version = (uint)1693373023U;
            p148.flight_custom_version_SET(new byte[8], 0);
            p148.middleware_custom_version_SET(new byte[8], 0);
            p148.os_custom_version_SET(new byte[8], 0);
            p148.vendor_id = (ushort)(ushort)7656;
            p148.product_id = (ushort)(ushort)754;
            p148.uid = (ulong)2285488487647113826L;
            p148.uid2_SET(new byte[18], 0, PH);
            CommunicationChannel.instance.send(p148); //===============================
            LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.time_usec = (ulong)4172256095163578311L;
            p149.target_num = (byte)(byte)131;
            p149.frame = MAV_FRAME.MAV_FRAME_LOCAL_ENU;
            p149.angle_x = (float) -2.4232954E38F;
            p149.angle_y = (float)4.170036E36F;
            p149.distance = (float)1.6903453E38F;
            p149.size_x = (float) -2.365078E38F;
            p149.size_y = (float) -2.7605191E38F;
            p149.x_SET((float)2.6671486E38F, PH);
            p149.y_SET((float) -1.9723076E38F, PH);
            p149.z_SET((float) -2.8501248E38F, PH);
            p149.q_SET(new float[4], 0, PH);
            p149.type = LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON;
            p149.position_valid_SET((byte)(byte)64, PH);
            CommunicationChannel.instance.send(p149); //===============================
            ARRAY_TEST_0 p150 = CommunicationChannel.new_ARRAY_TEST_0();
            PH.setPack(p150);
            p150.v1 = (byte)(byte)189;
            p150.ar_i8_SET(new sbyte[4], 0);
            p150.ar_u8_SET(new byte[4], 0);
            p150.ar_u16_SET(new ushort[4], 0);
            p150.ar_u32_SET(new uint[4], 0);
            CommunicationChannel.instance.send(p150); //===============================
            ARRAY_TEST_1 p151 = CommunicationChannel.new_ARRAY_TEST_1();
            PH.setPack(p151);
            p151.ar_u32_SET(new uint[4], 0);
            CommunicationChannel.instance.send(p151); //===============================
            ARRAY_TEST_3 p153 = CommunicationChannel.new_ARRAY_TEST_3();
            PH.setPack(p153);
            p153.v = (byte)(byte)71;
            p153.ar_u32_SET(new uint[4], 0);
            CommunicationChannel.instance.send(p153); //===============================
            ARRAY_TEST_4 p154 = CommunicationChannel.new_ARRAY_TEST_4();
            PH.setPack(p154);
            p154.ar_u32_SET(new uint[4], 0);
            p154.v = (byte)(byte)0;
            CommunicationChannel.instance.send(p154); //===============================
            ARRAY_TEST_5 p155 = CommunicationChannel.new_ARRAY_TEST_5();
            PH.setPack(p155);
            p155.c1_SET("DEMO", PH);
            p155.c2_SET("DEMO", PH);
            CommunicationChannel.instance.send(p155); //===============================
            ARRAY_TEST_6 p156 = CommunicationChannel.new_ARRAY_TEST_6();
            PH.setPack(p156);
            p156.v1 = (byte)(byte)104;
            p156.v2 = (ushort)(ushort)12696;
            p156.v3 = (uint)3715121350U;
            p156.ar_u32_SET(new uint[2], 0);
            p156.ar_i32_SET(new int[2], 0);
            p156.ar_u16_SET(new ushort[2], 0);
            p156.ar_i16_SET(new short[2], 0);
            p156.ar_u8_SET(new byte[2], 0);
            p156.ar_i8_SET(new sbyte[2], 0);
            p156.ar_c_SET("DEMO", PH);
            p156.ar_d_SET(new double[2], 0);
            p156.ar_f_SET(new float[2], 0);
            CommunicationChannel.instance.send(p156); //===============================
            ARRAY_TEST_7 p157 = CommunicationChannel.new_ARRAY_TEST_7();
            PH.setPack(p157);
            p157.ar_d_SET(new double[2], 0);
            p157.ar_f_SET(new float[2], 0);
            p157.ar_u32_SET(new uint[2], 0);
            p157.ar_i32_SET(new int[2], 0);
            p157.ar_u16_SET(new ushort[2], 0);
            p157.ar_i16_SET(new short[2], 0);
            p157.ar_u8_SET(new byte[2], 0);
            p157.ar_i8_SET(new sbyte[2], 0);
            p157.ar_c_SET("DEMO", PH);
            CommunicationChannel.instance.send(p157); //===============================
            ARRAY_TEST_8 p158 = CommunicationChannel.new_ARRAY_TEST_8();
            PH.setPack(p158);
            p158.v3 = (uint)3859753986U;
            p158.ar_d_SET(new double[2], 0);
            p158.ar_u16_SET(new ushort[2], 0);
            CommunicationChannel.instance.send(p158); //===============================
            ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.time_usec = (ulong)2449389447178809147L;
            p230.flags = (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT);
            p230.vel_ratio = (float) -1.899192E38F;
            p230.pos_horiz_ratio = (float)2.9479772E38F;
            p230.pos_vert_ratio = (float) -2.8116075E38F;
            p230.mag_ratio = (float) -1.0029454E38F;
            p230.hagl_ratio = (float) -2.3187638E38F;
            p230.tas_ratio = (float) -4.807427E37F;
            p230.pos_horiz_accuracy = (float) -1.56868E38F;
            p230.pos_vert_accuracy = (float) -5.8609805E37F;
            CommunicationChannel.instance.send(p230); //===============================
            WIND_COV p231 = CommunicationChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.time_usec = (ulong)2182456064750595326L;
            p231.wind_x = (float) -2.3014947E38F;
            p231.wind_y = (float)6.4929464E37F;
            p231.wind_z = (float) -3.1632205E38F;
            p231.var_horiz = (float) -8.80744E37F;
            p231.var_vert = (float)2.819991E38F;
            p231.wind_alt = (float) -2.7457677E37F;
            p231.horiz_accuracy = (float) -7.3111936E37F;
            p231.vert_accuracy = (float)2.814524E38F;
            CommunicationChannel.instance.send(p231); //===============================
            GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.time_usec = (ulong)5868792754009243422L;
            p232.gps_id = (byte)(byte)39;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ);
            p232.time_week_ms = (uint)4000280033U;
            p232.time_week = (ushort)(ushort)29389;
            p232.fix_type = (byte)(byte)35;
            p232.lat = (int)809706563;
            p232.lon = (int) -1598716228;
            p232.alt = (float)1.3924658E38F;
            p232.hdop = (float) -4.6516484E37F;
            p232.vdop = (float) -7.520736E37F;
            p232.vn = (float) -2.5014318E38F;
            p232.ve = (float) -8.680151E37F;
            p232.vd = (float) -5.23978E37F;
            p232.speed_accuracy = (float)1.2934612E38F;
            p232.horiz_accuracy = (float) -6.665862E37F;
            p232.vert_accuracy = (float)3.2615143E38F;
            p232.satellites_visible = (byte)(byte)33;
            CommunicationChannel.instance.send(p232); //===============================
            GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.flags = (byte)(byte)172;
            p233.len = (byte)(byte)178;
            p233.data__SET(new byte[180], 0);
            CommunicationChannel.instance.send(p233); //===============================
            HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED);
            p234.custom_mode = (uint)1461568814U;
            p234.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR;
            p234.roll = (short)(short)19649;
            p234.pitch = (short)(short) -5530;
            p234.heading = (ushort)(ushort)56520;
            p234.throttle = (sbyte)(sbyte)67;
            p234.heading_sp = (short)(short) -2940;
            p234.latitude = (int) -711345515;
            p234.longitude = (int)83237111;
            p234.altitude_amsl = (short)(short) -15825;
            p234.altitude_sp = (short)(short) -1626;
            p234.airspeed = (byte)(byte)105;
            p234.airspeed_sp = (byte)(byte)177;
            p234.groundspeed = (byte)(byte)213;
            p234.climb_rate = (sbyte)(sbyte) - 121;
            p234.gps_nsat = (byte)(byte)114;
            p234.gps_fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS;
            p234.battery_remaining = (byte)(byte)177;
            p234.temperature = (sbyte)(sbyte)108;
            p234.temperature_air = (sbyte)(sbyte) - 11;
            p234.failsafe = (byte)(byte)132;
            p234.wp_num = (byte)(byte)167;
            p234.wp_distance = (ushort)(ushort)34170;
            CommunicationChannel.instance.send(p234); //===============================
            VIBRATION p241 = CommunicationChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.time_usec = (ulong)6292374963068098877L;
            p241.vibration_x = (float) -1.432438E38F;
            p241.vibration_y = (float)1.0384326E38F;
            p241.vibration_z = (float)2.0544164E38F;
            p241.clipping_0 = (uint)3695874368U;
            p241.clipping_1 = (uint)2724844517U;
            p241.clipping_2 = (uint)878565088U;
            CommunicationChannel.instance.send(p241); //===============================
            HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.latitude = (int)1683174368;
            p242.longitude = (int)1792339640;
            p242.altitude = (int) -479856230;
            p242.x = (float) -2.6706425E38F;
            p242.y = (float) -4.5054356E37F;
            p242.z = (float)3.1595678E38F;
            p242.q_SET(new float[4], 0);
            p242.approach_x = (float)2.791536E38F;
            p242.approach_y = (float)1.6242672E38F;
            p242.approach_z = (float)3.1250502E38F;
            p242.time_usec_SET((ulong)1023682568814324752L, PH);
            CommunicationChannel.instance.send(p242); //===============================
            SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.target_system = (byte)(byte)60;
            p243.latitude = (int) -2029313615;
            p243.longitude = (int)1350592228;
            p243.altitude = (int)1639711861;
            p243.x = (float)2.1169055E38F;
            p243.y = (float) -1.4899722E37F;
            p243.z = (float)7.826464E37F;
            p243.q_SET(new float[4], 0);
            p243.approach_x = (float)1.4238553E38F;
            p243.approach_y = (float) -4.6218277E37F;
            p243.approach_z = (float)1.6003933E38F;
            p243.time_usec_SET((ulong)3251841154104429860L, PH);
            CommunicationChannel.instance.send(p243); //===============================
            MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.message_id = (ushort)(ushort)31547;
            p244.interval_us = (int)121416299;
            CommunicationChannel.instance.send(p244); //===============================
            EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.vtol_state = MAV_VTOL_STATE.MAV_VTOL_STATE_MC;
            p245.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF;
            CommunicationChannel.instance.send(p245); //===============================
            ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.ICAO_address = (uint)3686543177U;
            p246.lat = (int) -2069504512;
            p246.lon = (int) -181334092;
            p246.altitude_type = ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
            p246.altitude = (int) -1378652813;
            p246.heading = (ushort)(ushort)24493;
            p246.hor_velocity = (ushort)(ushort)46331;
            p246.ver_velocity = (short)(short)22950;
            p246.callsign_SET("DEMO", PH);
            p246.emitter_type = ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_POINT_OBSTACLE;
            p246.tslc = (byte)(byte)54;
            p246.flags = (ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY |
                          ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN |
                          ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING);
            p246.squawk = (ushort)(ushort)32754;
            CommunicationChannel.instance.send(p246); //===============================
            COLLISION p247 = CommunicationChannel.new_COLLISION();
            PH.setPack(p247);
            p247.src_ = MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB;
            p247.id = (uint)3181027474U;
            p247.action = MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_HOVER;
            p247.threat_level = MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE;
            p247.time_to_minimum_delta = (float)1.9750404E38F;
            p247.altitude_minimum_delta = (float) -1.0621342E38F;
            p247.horizontal_minimum_delta = (float)3.0282913E38F;
            CommunicationChannel.instance.send(p247); //===============================
            V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.target_network = (byte)(byte)74;
            p248.target_system = (byte)(byte)64;
            p248.target_component = (byte)(byte)157;
            p248.message_type = (ushort)(ushort)64835;
            p248.payload_SET(new byte[249], 0);
            CommunicationChannel.instance.send(p248); //===============================
            MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.address = (ushort)(ushort)35944;
            p249.ver = (byte)(byte)68;
            p249.type = (byte)(byte)219;
            p249.value_SET(new sbyte[32], 0);
            CommunicationChannel.instance.send(p249); //===============================
            DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.name_SET("DEMO", PH);
            p250.time_usec = (ulong)6291678846910054905L;
            p250.x = (float)3.211898E38F;
            p250.y = (float)3.093809E38F;
            p250.z = (float)1.928698E38F;
            CommunicationChannel.instance.send(p250); //===============================
            NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.time_boot_ms = (uint)3960143792U;
            p251.name_SET("DEMO", PH);
            p251.value = (float) -3.778539E37F;
            CommunicationChannel.instance.send(p251); //===============================
            NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.time_boot_ms = (uint)2172860775U;
            p252.name_SET("DEMO", PH);
            p252.value = (int) -2146499871;
            CommunicationChannel.instance.send(p252); //===============================
            STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.severity = MAV_SEVERITY.MAV_SEVERITY_ALERT;
            p253.text_SET("DEMO", PH);
            CommunicationChannel.instance.send(p253); //===============================
            DEBUG p254 = CommunicationChannel.new_DEBUG();
            PH.setPack(p254);
            p254.time_boot_ms = (uint)3594684928U;
            p254.ind = (byte)(byte)29;
            p254.value = (float)1.7977446E38F;
            CommunicationChannel.instance.send(p254); //===============================
            SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.target_system = (byte)(byte)204;
            p256.target_component = (byte)(byte)152;
            p256.secret_key_SET(new byte[32], 0);
            p256.initial_timestamp = (ulong)4734301949714191489L;
            CommunicationChannel.instance.send(p256); //===============================
            BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.time_boot_ms = (uint)1231298693U;
            p257.last_change_ms = (uint)2212839639U;
            p257.state = (byte)(byte)158;
            CommunicationChannel.instance.send(p257); //===============================
            PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_system = (byte)(byte)48;
            p258.target_component = (byte)(byte)185;
            p258.tune_SET("DEMO", PH);
            CommunicationChannel.instance.send(p258); //===============================
            CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.time_boot_ms = (uint)2764903958U;
            p259.vendor_name_SET(new byte[32], 0);
            p259.model_name_SET(new byte[32], 0);
            p259.firmware_version = (uint)1697682585U;
            p259.focal_length = (float)2.1371532E38F;
            p259.sensor_size_h = (float)9.526269E37F;
            p259.sensor_size_v = (float) -2.8525244E38F;
            p259.resolution_h = (ushort)(ushort)1332;
            p259.resolution_v = (ushort)(ushort)61462;
            p259.lens_id = (byte)(byte)51;
            p259.flags = (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
                          CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
                          CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE);
            p259.cam_definition_version = (ushort)(ushort)8210;
            p259.cam_definition_uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p259); //===============================
            CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.time_boot_ms = (uint)647141020U;
            p260.mode_id = CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY;
            CommunicationChannel.instance.send(p260); //===============================
            STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.time_boot_ms = (uint)430888009U;
            p261.storage_id = (byte)(byte)39;
            p261.storage_count = (byte)(byte)195;
            p261.status = (byte)(byte)108;
            p261.total_capacity = (float)2.4435919E38F;
            p261.used_capacity = (float) -1.929201E37F;
            p261.available_capacity = (float) -5.126056E37F;
            p261.read_speed = (float)1.0084079E38F;
            p261.write_speed = (float)3.350914E38F;
            CommunicationChannel.instance.send(p261); //===============================
            CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.time_boot_ms = (uint)4164595364U;
            p262.image_status = (byte)(byte)115;
            p262.video_status = (byte)(byte)12;
            p262.image_interval = (float) -5.8444017E37F;
            p262.recording_time_ms = (uint)252069439U;
            p262.available_capacity = (float) -3.224523E38F;
            CommunicationChannel.instance.send(p262); //===============================
            CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.time_boot_ms = (uint)3729051103U;
            p263.time_utc = (ulong)5886190418756425323L;
            p263.camera_id = (byte)(byte)193;
            p263.lat = (int) -426752321;
            p263.lon = (int) -137713680;
            p263.alt = (int) -1525944289;
            p263.relative_alt = (int) -252778584;
            p263.q_SET(new float[4], 0);
            p263.image_index = (int) -2032296563;
            p263.capture_result = (sbyte)(sbyte) - 107;
            p263.file_url_SET("DEMO", PH);
            CommunicationChannel.instance.send(p263); //===============================
            FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.time_boot_ms = (uint)1504284795U;
            p264.arming_time_utc = (ulong)4211544448098458929L;
            p264.takeoff_time_utc = (ulong)484307108111931298L;
            p264.flight_uuid = (ulong)2908202936444389805L;
            CommunicationChannel.instance.send(p264); //===============================
            MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.time_boot_ms = (uint)620671835U;
            p265.roll = (float)6.740122E37F;
            p265.pitch = (float) -1.0355066E37F;
            p265.yaw = (float) -2.8666127E38F;
            CommunicationChannel.instance.send(p265); //===============================
            LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.target_system = (byte)(byte)51;
            p266.target_component = (byte)(byte)197;
            p266.sequence = (ushort)(ushort)19517;
            p266.length = (byte)(byte)56;
            p266.first_message_offset = (byte)(byte)157;
            p266.data__SET(new byte[249], 0);
            CommunicationChannel.instance.send(p266); //===============================
            LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.target_system = (byte)(byte)102;
            p267.target_component = (byte)(byte)240;
            p267.sequence = (ushort)(ushort)42612;
            p267.length = (byte)(byte)64;
            p267.first_message_offset = (byte)(byte)172;
            p267.data__SET(new byte[249], 0);
            CommunicationChannel.instance.send(p267); //===============================
            LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_system = (byte)(byte)77;
            p268.target_component = (byte)(byte)58;
            p268.sequence = (ushort)(ushort)4796;
            CommunicationChannel.instance.send(p268); //===============================
            VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.camera_id = (byte)(byte)215;
            p269.status = (byte)(byte)192;
            p269.framerate = (float) -2.9151398E38F;
            p269.resolution_h = (ushort)(ushort)62425;
            p269.resolution_v = (ushort)(ushort)19436;
            p269.bitrate = (uint)2860139808U;
            p269.rotation = (ushort)(ushort)1900;
            p269.uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p269); //===============================
            SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.target_system = (byte)(byte)35;
            p270.target_component = (byte)(byte)214;
            p270.camera_id = (byte)(byte)103;
            p270.framerate = (float)2.1442574E38F;
            p270.resolution_h = (ushort)(ushort)24545;
            p270.resolution_v = (ushort)(ushort)23514;
            p270.bitrate = (uint)2117386999U;
            p270.rotation = (ushort)(ushort)51063;
            p270.uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p270); //===============================
            WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.ssid_SET("DEMO", PH);
            p299.password_SET("DEMO", PH);
            CommunicationChannel.instance.send(p299); //===============================
            PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.version = (ushort)(ushort)22633;
            p300.min_version = (ushort)(ushort)11749;
            p300.max_version = (ushort)(ushort)3376;
            p300.spec_version_hash_SET(new byte[8], 0);
            p300.library_version_hash_SET(new byte[8], 0);
            CommunicationChannel.instance.send(p300); //===============================
            UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.time_usec = (ulong)8535757063934214879L;
            p310.uptime_sec = (uint)3267996153U;
            p310.health = UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING;
            p310.mode = UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE;
            p310.sub_mode = (byte)(byte)232;
            p310.vendor_specific_status_code = (ushort)(ushort)28945;
            CommunicationChannel.instance.send(p310); //===============================
            UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.time_usec = (ulong)7955673795974713667L;
            p311.uptime_sec = (uint)904950409U;
            p311.name_SET("DEMO", PH);
            p311.hw_version_major = (byte)(byte)141;
            p311.hw_version_minor = (byte)(byte)36;
            p311.hw_unique_id_SET(new byte[16], 0);
            p311.sw_version_major = (byte)(byte)193;
            p311.sw_version_minor = (byte)(byte)57;
            p311.sw_vcs_commit = (uint)372835840U;
            CommunicationChannel.instance.send(p311); //===============================
            PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.target_system = (byte)(byte)255;
            p320.target_component = (byte)(byte)206;
            p320.param_id_SET("DEMO", PH);
            p320.param_index = (short)(short) -31833;
            CommunicationChannel.instance.send(p320); //===============================
            PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_system = (byte)(byte)4;
            p321.target_component = (byte)(byte)138;
            CommunicationChannel.instance.send(p321); //===============================
            PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_id_SET("DEMO", PH);
            p322.param_value_SET("DEMO", PH);
            p322.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT8;
            p322.param_count = (ushort)(ushort)13239;
            p322.param_index = (ushort)(ushort)60595;
            CommunicationChannel.instance.send(p322); //===============================
            PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.target_system = (byte)(byte)40;
            p323.target_component = (byte)(byte)178;
            p323.param_id_SET("DEMO", PH);
            p323.param_value_SET("DEMO", PH);
            p323.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT64;
            CommunicationChannel.instance.send(p323); //===============================
            PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_id_SET("DEMO", PH);
            p324.param_value_SET("DEMO", PH);
            p324.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM;
            p324.param_result = PARAM_ACK.PARAM_ACK_IN_PROGRESS;
            CommunicationChannel.instance.send(p324); //===============================
            OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.time_usec = (ulong)2216107608001421260L;
            p330.sensor_type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR;
            p330.distances_SET(new ushort[72], 0);
            p330.increment = (byte)(byte)200;
            p330.min_distance = (ushort)(ushort)48946;
            p330.max_distance = (ushort)(ushort)474;
            CommunicationChannel.instance.send(p330); //===============================
        }
    }
}

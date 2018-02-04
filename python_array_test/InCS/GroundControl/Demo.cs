
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
            POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.time_boot_ms = (uint)663746032U;
            p3.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
            p3.type_mask = (ushort)(ushort)57919;
            p3.x = (float) -1.5243642E38F;
            p3.y = (float)2.866823E38F;
            p3.z = (float)3.9853254E37F;
            p3.vx = (float) -1.726884E38F;
            p3.vy = (float)1.477872E38F;
            p3.vz = (float)1.3010335E38F;
            p3.afx = (float) -1.5609338E38F;
            p3.afy = (float) -2.8028887E38F;
            p3.afz = (float) -2.5316982E38F;
            p3.yaw = (float) -1.6955237E38F;
            p3.yaw_rate = (float) -2.0900632E38F;
            CommunicationChannel.instance.send(p3); //===============================
            COMMAND_LONG p76 = CommunicationChannel.new_COMMAND_LONG();
            PH.setPack(p76);
            p76.target_system = (byte)(byte)65;
            p76.target_component = (byte)(byte)238;
            p76.command = (MAV_CMD)MAV_CMD.MAV_CMD_NAV_WAYPOINT;
            p76.confirmation = (byte)(byte)196;
            p76.param1 = (float)1.5852793E38F;
            p76.param2 = (float)3.2217675E38F;
            p76.param3 = (float) -2.0298802E38F;
            p76.param4 = (float) -1.0022555E38F;
            p76.param5 = (float)1.4388013E38F;
            p76.param6 = (float) -1.350759E38F;
            p76.param7 = (float)1.4961126E38F;
            CommunicationChannel.instance.send(p76); //===============================
            COMMAND_ACK p77 = CommunicationChannel.new_COMMAND_ACK();
            PH.setPack(p77);
            p77.command = (MAV_CMD)MAV_CMD.MAV_CMD_DO_PARACHUTE;
            p77.result = (MAV_RESULT)MAV_RESULT.MAV_RESULT_TEMPORARILY_REJECTED;
            p77.progress_SET((byte)(byte)136, PH);
            p77.result_param2_SET((int)910748641, PH);
            p77.target_system_SET((byte)(byte)170, PH);
            p77.target_component_SET((byte)(byte)93, PH);
            CommunicationChannel.instance.send(p77); //===============================
            MANUAL_SETPOINT p81 = CommunicationChannel.new_MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.time_boot_ms = (uint)540230530U;
            p81.roll = (float)3.2152667E37F;
            p81.pitch = (float) -1.079935E38F;
            p81.yaw = (float) -3.3444075E38F;
            p81.thrust = (float) -3.3146033E38F;
            p81.mode_switch = (byte)(byte)206;
            p81.manual_override_switch = (byte)(byte)172;
            CommunicationChannel.instance.send(p81); //===============================
            SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.time_boot_ms = (uint)4082908871U;
            p82.target_system = (byte)(byte)193;
            p82.target_component = (byte)(byte)214;
            p82.type_mask = (byte)(byte)131;
            p82.q_SET(new float[4], 0);
            p82.body_roll_rate = (float)2.102368E38F;
            p82.body_pitch_rate = (float)8.295153E37F;
            p82.body_yaw_rate = (float) -1.9372109E38F;
            p82.thrust = (float)3.3403983E38F;
            CommunicationChannel.instance.send(p82); //===============================
            ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.time_boot_ms = (uint)1203632086U;
            p83.type_mask = (byte)(byte)236;
            p83.q_SET(new float[4], 0);
            p83.body_roll_rate = (float)3.3325697E38F;
            p83.body_pitch_rate = (float)2.1017353E38F;
            p83.body_yaw_rate = (float) -2.9434165E38F;
            p83.thrust = (float)1.7066262E38F;
            CommunicationChannel.instance.send(p83); //===============================
            SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.time_boot_ms = (uint)1150391620U;
            p84.target_system = (byte)(byte)129;
            p84.target_component = (byte)(byte)12;
            p84.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p84.type_mask = (ushort)(ushort)63843;
            p84.x = (float) -1.6182514E38F;
            p84.y = (float) -2.1732026E38F;
            p84.z = (float)1.757617E38F;
            p84.vx = (float) -3.6964988E37F;
            p84.vy = (float) -2.3714498E38F;
            p84.vz = (float) -2.805464E38F;
            p84.afx = (float) -1.3858266E38F;
            p84.afy = (float)2.8997054E37F;
            p84.afz = (float)1.2048138E38F;
            p84.yaw = (float)1.6928825E38F;
            p84.yaw_rate = (float)3.0804506E38F;
            CommunicationChannel.instance.send(p84); //===============================
            SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.time_boot_ms = (uint)3438934641U;
            p86.target_system = (byte)(byte)2;
            p86.target_component = (byte)(byte)139;
            p86.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p86.type_mask = (ushort)(ushort)62956;
            p86.lat_int = (int)1739569956;
            p86.lon_int = (int) -734395104;
            p86.alt = (float)5.8572754E37F;
            p86.vx = (float)1.2994764E38F;
            p86.vy = (float)2.6467605E38F;
            p86.vz = (float)2.9899892E38F;
            p86.afx = (float) -1.4168524E37F;
            p86.afy = (float)1.2192782E38F;
            p86.afz = (float)3.1120045E38F;
            p86.yaw = (float) -2.4304462E38F;
            p86.yaw_rate = (float) -1.2616358E38F;
            CommunicationChannel.instance.send(p86); //===============================
            POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.time_boot_ms = (uint)1928453062U;
            p87.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_MISSION;
            p87.type_mask = (ushort)(ushort)50561;
            p87.lat_int = (int) -582413022;
            p87.lon_int = (int)747046946;
            p87.alt = (float)1.7896286E38F;
            p87.vx = (float)8.866402E37F;
            p87.vy = (float)1.6951897E38F;
            p87.vz = (float)2.729851E38F;
            p87.afx = (float) -7.0093584E37F;
            p87.afy = (float) -1.642596E38F;
            p87.afz = (float) -2.560894E38F;
            p87.yaw = (float)1.9056922E38F;
            p87.yaw_rate = (float) -3.0026694E38F;
            CommunicationChannel.instance.send(p87); //===============================
            LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.time_boot_ms = (uint)3451338913U;
            p89.x = (float)7.4022515E37F;
            p89.y = (float)2.2534585E38F;
            p89.z = (float) -2.3010777E38F;
            p89.roll = (float)5.916551E37F;
            p89.pitch = (float)2.7120427E38F;
            p89.yaw = (float)1.846639E38F;
            CommunicationChannel.instance.send(p89); //===============================
            HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.time_usec = (ulong)5766255344271371270L;
            p90.roll = (float) -1.2319232E38F;
            p90.pitch = (float) -3.0337706E38F;
            p90.yaw = (float)1.3373046E38F;
            p90.rollspeed = (float) -2.934728E37F;
            p90.pitchspeed = (float)3.2023302E38F;
            p90.yawspeed = (float) -7.022666E37F;
            p90.lat = (int) -430773083;
            p90.lon = (int) -429452699;
            p90.alt = (int)1912705705;
            p90.vx = (short)(short) -15386;
            p90.vy = (short)(short)19475;
            p90.vz = (short)(short)24213;
            p90.xacc = (short)(short) -13084;
            p90.yacc = (short)(short) -15369;
            p90.zacc = (short)(short) -17701;
            CommunicationChannel.instance.send(p90); //===============================
            HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.time_usec = (ulong)7411920505540759077L;
            p91.roll_ailerons = (float)5.695512E36F;
            p91.pitch_elevator = (float)8.59861E37F;
            p91.yaw_rudder = (float)2.3543718E38F;
            p91.throttle = (float)2.7376432E38F;
            p91.aux1 = (float) -2.8822724E38F;
            p91.aux2 = (float) -4.2447577E37F;
            p91.aux3 = (float)1.1728751E38F;
            p91.aux4 = (float) -1.90166E38F;
            p91.mode = (MAV_MODE)MAV_MODE.MAV_MODE_AUTO_DISARMED;
            p91.nav_mode = (byte)(byte)203;
            CommunicationChannel.instance.send(p91); //===============================
            HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.time_usec = (ulong)7535456089897844360L;
            p92.chan1_raw = (ushort)(ushort)39287;
            p92.chan2_raw = (ushort)(ushort)111;
            p92.chan3_raw = (ushort)(ushort)59634;
            p92.chan4_raw = (ushort)(ushort)424;
            p92.chan5_raw = (ushort)(ushort)30390;
            p92.chan6_raw = (ushort)(ushort)52248;
            p92.chan7_raw = (ushort)(ushort)18275;
            p92.chan8_raw = (ushort)(ushort)18157;
            p92.chan9_raw = (ushort)(ushort)1716;
            p92.chan10_raw = (ushort)(ushort)14127;
            p92.chan11_raw = (ushort)(ushort)46385;
            p92.chan12_raw = (ushort)(ushort)6303;
            p92.rssi = (byte)(byte)31;
            CommunicationChannel.instance.send(p92); //===============================
            HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.time_usec = (ulong)6270976290465758947L;
            p93.controls_SET(new float[16], 0);
            p93.mode = (MAV_MODE)MAV_MODE.MAV_MODE_PREFLIGHT;
            p93.flags = (ulong)2546542577446993565L;
            CommunicationChannel.instance.send(p93); //===============================
            OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.time_usec = (ulong)8771867796908477549L;
            p100.sensor_id = (byte)(byte)247;
            p100.flow_x = (short)(short)12357;
            p100.flow_y = (short)(short) -8564;
            p100.flow_comp_m_x = (float) -2.7644308E38F;
            p100.flow_comp_m_y = (float)1.5528064E38F;
            p100.quality = (byte)(byte)217;
            p100.ground_distance = (float) -7.027176E37F;
            p100.flow_rate_x_SET((float) -1.2010759E38F, PH);
            p100.flow_rate_y_SET((float) -4.1276022E36F, PH);
            CommunicationChannel.instance.send(p100); //===============================
            GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.usec = (ulong)7384422594266290536L;
            p101.x = (float) -3.382334E38F;
            p101.y = (float) -1.4953787E38F;
            p101.z = (float) -3.342458E38F;
            p101.roll = (float)1.4968877E38F;
            p101.pitch = (float)2.9888786E38F;
            p101.yaw = (float) -2.5079823E37F;
            CommunicationChannel.instance.send(p101); //===============================
            VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.usec = (ulong)3722699840501217626L;
            p102.x = (float) -1.7521065E38F;
            p102.y = (float)1.1174856E38F;
            p102.z = (float) -8.3073486E36F;
            p102.roll = (float)1.0604383E38F;
            p102.pitch = (float)2.737563E38F;
            p102.yaw = (float)1.8538053E38F;
            CommunicationChannel.instance.send(p102); //===============================
            VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.usec = (ulong)7132286598041268829L;
            p103.x = (float)2.338745E38F;
            p103.y = (float)1.4673156E38F;
            p103.z = (float) -2.6072833E38F;
            CommunicationChannel.instance.send(p103); //===============================
            VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.usec = (ulong)1467920528778603660L;
            p104.x = (float) -1.3400301E38F;
            p104.y = (float)4.5608923E37F;
            p104.z = (float) -2.5154508E38F;
            p104.roll = (float)2.6755427E38F;
            p104.pitch = (float)1.9252256E38F;
            p104.yaw = (float)4.60579E37F;
            CommunicationChannel.instance.send(p104); //===============================
            HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.time_usec = (ulong)3010744551579756969L;
            p105.xacc = (float) -1.54891E38F;
            p105.yacc = (float)3.3269622E38F;
            p105.zacc = (float) -1.1903981E37F;
            p105.xgyro = (float)1.5600485E38F;
            p105.ygyro = (float) -3.3950944E38F;
            p105.zgyro = (float)6.432617E37F;
            p105.xmag = (float)3.1008387E38F;
            p105.ymag = (float)2.0250469E38F;
            p105.zmag = (float) -6.8846317E37F;
            p105.abs_pressure = (float) -1.5856864E38F;
            p105.diff_pressure = (float)3.4446329E37F;
            p105.pressure_alt = (float) -1.897238E38F;
            p105.temperature = (float)2.629569E38F;
            p105.fields_updated = (ushort)(ushort)34931;
            CommunicationChannel.instance.send(p105); //===============================
            OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.time_usec = (ulong)8192963463873266812L;
            p106.sensor_id = (byte)(byte)76;
            p106.integration_time_us = (uint)1469497024U;
            p106.integrated_x = (float) -1.2132486E38F;
            p106.integrated_y = (float)2.1847585E38F;
            p106.integrated_xgyro = (float) -7.221022E37F;
            p106.integrated_ygyro = (float)2.3347176E37F;
            p106.integrated_zgyro = (float)1.684464E38F;
            p106.temperature = (short)(short) -2860;
            p106.quality = (byte)(byte)231;
            p106.time_delta_distance_us = (uint)123835679U;
            p106.distance = (float) -1.3165238E38F;
            CommunicationChannel.instance.send(p106); //===============================
            HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.time_usec = (ulong)1807957615941681209L;
            p107.xacc = (float)2.715366E38F;
            p107.yacc = (float) -1.530966E38F;
            p107.zacc = (float)3.0647757E38F;
            p107.xgyro = (float) -3.655356E37F;
            p107.ygyro = (float) -1.3498184E38F;
            p107.zgyro = (float)1.1254184E38F;
            p107.xmag = (float)2.743371E38F;
            p107.ymag = (float) -2.9924696E38F;
            p107.zmag = (float) -2.848021E38F;
            p107.abs_pressure = (float) -1.2594263E38F;
            p107.diff_pressure = (float) -2.8344351E38F;
            p107.pressure_alt = (float) -2.0006471E37F;
            p107.temperature = (float) -1.5608241E38F;
            p107.fields_updated = (uint)3480159399U;
            CommunicationChannel.instance.send(p107); //===============================
            SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.q1 = (float)2.9783388E38F;
            p108.q2 = (float)3.192925E38F;
            p108.q3 = (float) -1.5689802E38F;
            p108.q4 = (float) -3.6708606E37F;
            p108.roll = (float)2.0508683E37F;
            p108.pitch = (float) -2.7213258E38F;
            p108.yaw = (float)2.9643912E38F;
            p108.xacc = (float)1.9361694E38F;
            p108.yacc = (float)1.931584E38F;
            p108.zacc = (float) -3.3520978E38F;
            p108.xgyro = (float)1.1663499E38F;
            p108.ygyro = (float)8.777642E37F;
            p108.zgyro = (float) -2.123547E38F;
            p108.lat = (float)1.5754876E38F;
            p108.lon = (float)2.5908524E37F;
            p108.alt = (float) -3.1036612E37F;
            p108.std_dev_horz = (float)4.6836977E37F;
            p108.std_dev_vert = (float)1.730681E38F;
            p108.vn = (float)1.9110857E38F;
            p108.ve = (float) -1.70882E37F;
            p108.vd = (float)2.0555407E38F;
            CommunicationChannel.instance.send(p108); //===============================
            RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.rssi = (byte)(byte)111;
            p109.remrssi = (byte)(byte)128;
            p109.txbuf = (byte)(byte)169;
            p109.noise = (byte)(byte)13;
            p109.remnoise = (byte)(byte)229;
            p109.rxerrors = (ushort)(ushort)19125;
            p109.fixed_ = (ushort)(ushort)56217;
            CommunicationChannel.instance.send(p109); //===============================
            FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_network = (byte)(byte)4;
            p110.target_system = (byte)(byte)41;
            p110.target_component = (byte)(byte)220;
            p110.payload_SET(new byte[251], 0);
            CommunicationChannel.instance.send(p110); //===============================
            TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.tc1 = (long)5110419899414494669L;
            p111.ts1 = (long) -3957321698246132869L;
            CommunicationChannel.instance.send(p111); //===============================
            CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.time_usec = (ulong)4496832414870683477L;
            p112.seq = (uint)2147484957U;
            CommunicationChannel.instance.send(p112); //===============================
            HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.time_usec = (ulong)2505201950202102759L;
            p113.fix_type = (byte)(byte)160;
            p113.lat = (int)826496734;
            p113.lon = (int) -1086470593;
            p113.alt = (int) -262730623;
            p113.eph = (ushort)(ushort)12487;
            p113.epv = (ushort)(ushort)28192;
            p113.vel = (ushort)(ushort)17405;
            p113.vn = (short)(short) -23296;
            p113.ve = (short)(short)32197;
            p113.vd = (short)(short) -18674;
            p113.cog = (ushort)(ushort)20166;
            p113.satellites_visible = (byte)(byte)120;
            CommunicationChannel.instance.send(p113); //===============================
            HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.time_usec = (ulong)726795770610125110L;
            p114.sensor_id = (byte)(byte)26;
            p114.integration_time_us = (uint)2302777037U;
            p114.integrated_x = (float) -1.4087979E38F;
            p114.integrated_y = (float) -3.401955E38F;
            p114.integrated_xgyro = (float)2.8725086E38F;
            p114.integrated_ygyro = (float) -4.548392E37F;
            p114.integrated_zgyro = (float)1.9392955E38F;
            p114.temperature = (short)(short) -28078;
            p114.quality = (byte)(byte)95;
            p114.time_delta_distance_us = (uint)1629439777U;
            p114.distance = (float) -2.3845321E38F;
            CommunicationChannel.instance.send(p114); //===============================
            HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.time_usec = (ulong)5454091148097684752L;
            p115.attitude_quaternion_SET(new float[4], 0);
            p115.rollspeed = (float)2.3579678E38F;
            p115.pitchspeed = (float)5.736366E37F;
            p115.yawspeed = (float)3.5765388E37F;
            p115.lat = (int)200961259;
            p115.lon = (int)395919475;
            p115.alt = (int) -1816192384;
            p115.vx = (short)(short)10404;
            p115.vy = (short)(short)27983;
            p115.vz = (short)(short)22016;
            p115.ind_airspeed = (ushort)(ushort)22915;
            p115.true_airspeed = (ushort)(ushort)46441;
            p115.xacc = (short)(short) -19355;
            p115.yacc = (short)(short) -30489;
            p115.zacc = (short)(short) -30685;
            CommunicationChannel.instance.send(p115); //===============================
            SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.time_boot_ms = (uint)3690707862U;
            p116.xacc = (short)(short) -25535;
            p116.yacc = (short)(short) -32262;
            p116.zacc = (short)(short)10822;
            p116.xgyro = (short)(short)28357;
            p116.ygyro = (short)(short) -28237;
            p116.zgyro = (short)(short) -28137;
            p116.xmag = (short)(short)11600;
            p116.ymag = (short)(short)30369;
            p116.zmag = (short)(short)16498;
            CommunicationChannel.instance.send(p116); //===============================
            LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.target_system = (byte)(byte)71;
            p117.target_component = (byte)(byte)245;
            p117.start = (ushort)(ushort)10803;
            p117.end = (ushort)(ushort)47130;
            CommunicationChannel.instance.send(p117); //===============================
            LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.id = (ushort)(ushort)56829;
            p118.num_logs = (ushort)(ushort)57282;
            p118.last_log_num = (ushort)(ushort)48480;
            p118.time_utc = (uint)3922842141U;
            p118.size = (uint)1497291437U;
            CommunicationChannel.instance.send(p118); //===============================
            LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.target_system = (byte)(byte)124;
            p119.target_component = (byte)(byte)235;
            p119.id = (ushort)(ushort)32792;
            p119.ofs = (uint)888276453U;
            p119.count = (uint)3608778781U;
            CommunicationChannel.instance.send(p119); //===============================
            LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.id = (ushort)(ushort)6753;
            p120.ofs = (uint)3248842620U;
            p120.count = (byte)(byte)183;
            p120.data__SET(new byte[90], 0);
            CommunicationChannel.instance.send(p120); //===============================
            LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_system = (byte)(byte)153;
            p121.target_component = (byte)(byte)96;
            CommunicationChannel.instance.send(p121); //===============================
            LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_system = (byte)(byte)168;
            p122.target_component = (byte)(byte)210;
            CommunicationChannel.instance.send(p122); //===============================
            GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.target_system = (byte)(byte)2;
            p123.target_component = (byte)(byte)186;
            p123.len = (byte)(byte)192;
            p123.data__SET(new byte[110], 0);
            CommunicationChannel.instance.send(p123); //===============================
            GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.time_usec = (ulong)7294481922095374739L;
            p124.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX;
            p124.lat = (int) -2097921671;
            p124.lon = (int)2128282124;
            p124.alt = (int)975312916;
            p124.eph = (ushort)(ushort)35196;
            p124.epv = (ushort)(ushort)53912;
            p124.vel = (ushort)(ushort)1546;
            p124.cog = (ushort)(ushort)12063;
            p124.satellites_visible = (byte)(byte)44;
            p124.dgps_numch = (byte)(byte)220;
            p124.dgps_age = (uint)4067499223U;
            CommunicationChannel.instance.send(p124); //===============================
            POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vcc = (ushort)(ushort)466;
            p125.Vservo = (ushort)(ushort)49621;
            p125.flags = (MAV_POWER_STATUS)MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID;
            CommunicationChannel.instance.send(p125); //===============================
            SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.device = (SERIAL_CONTROL_DEV)SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1;
            p126.flags = (SERIAL_CONTROL_FLAG)SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND;
            p126.timeout = (ushort)(ushort)29659;
            p126.baudrate = (uint)1106307641U;
            p126.count = (byte)(byte)136;
            p126.data__SET(new byte[70], 0);
            CommunicationChannel.instance.send(p126); //===============================
            GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.time_last_baseline_ms = (uint)1361926287U;
            p127.rtk_receiver_id = (byte)(byte)1;
            p127.wn = (ushort)(ushort)50535;
            p127.tow = (uint)2526968818U;
            p127.rtk_health = (byte)(byte)108;
            p127.rtk_rate = (byte)(byte)140;
            p127.nsats = (byte)(byte)191;
            p127.baseline_coords_type = (byte)(byte)153;
            p127.baseline_a_mm = (int) -1417170298;
            p127.baseline_b_mm = (int)1237958523;
            p127.baseline_c_mm = (int)2039650034;
            p127.accuracy = (uint)3827272851U;
            p127.iar_num_hypotheses = (int) -415477593;
            CommunicationChannel.instance.send(p127); //===============================
            GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.time_last_baseline_ms = (uint)2622020580U;
            p128.rtk_receiver_id = (byte)(byte)219;
            p128.wn = (ushort)(ushort)6247;
            p128.tow = (uint)305818286U;
            p128.rtk_health = (byte)(byte)108;
            p128.rtk_rate = (byte)(byte)179;
            p128.nsats = (byte)(byte)192;
            p128.baseline_coords_type = (byte)(byte)240;
            p128.baseline_a_mm = (int) -592977519;
            p128.baseline_b_mm = (int) -341784132;
            p128.baseline_c_mm = (int) -1251758757;
            p128.accuracy = (uint)664063254U;
            p128.iar_num_hypotheses = (int) -206820679;
            CommunicationChannel.instance.send(p128); //===============================
            SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.time_boot_ms = (uint)3451949062U;
            p129.xacc = (short)(short) -22471;
            p129.yacc = (short)(short) -14327;
            p129.zacc = (short)(short) -31523;
            p129.xgyro = (short)(short) -30975;
            p129.ygyro = (short)(short) -10556;
            p129.zgyro = (short)(short) -12294;
            p129.xmag = (short)(short)15756;
            p129.ymag = (short)(short) -12984;
            p129.zmag = (short)(short) -17416;
            CommunicationChannel.instance.send(p129); //===============================
            DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.type = (byte)(byte)139;
            p130.size = (uint)314244261U;
            p130.width = (ushort)(ushort)5;
            p130.height = (ushort)(ushort)19150;
            p130.packets = (ushort)(ushort)21723;
            p130.payload = (byte)(byte)0;
            p130.jpg_quality = (byte)(byte)84;
            CommunicationChannel.instance.send(p130); //===============================
            ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.seqnr = (ushort)(ushort)53488;
            p131.data__SET(new byte[253], 0);
            CommunicationChannel.instance.send(p131); //===============================
            DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.time_boot_ms = (uint)3642037865U;
            p132.min_distance = (ushort)(ushort)28646;
            p132.max_distance = (ushort)(ushort)62689;
            p132.current_distance = (ushort)(ushort)688;
            p132.type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED;
            p132.id = (byte)(byte)82;
            p132.orientation = (MAV_SENSOR_ORIENTATION)MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180;
            p132.covariance = (byte)(byte)148;
            CommunicationChannel.instance.send(p132); //===============================
            TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.lat = (int)92415424;
            p133.lon = (int) -1773648480;
            p133.grid_spacing = (ushort)(ushort)48591;
            p133.mask = (ulong)7227571025021255241L;
            CommunicationChannel.instance.send(p133); //===============================
            TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.lat = (int) -2027585914;
            p134.lon = (int) -387864886;
            p134.grid_spacing = (ushort)(ushort)64663;
            p134.gridbit = (byte)(byte)253;
            p134.data__SET(new short[16], 0);
            CommunicationChannel.instance.send(p134); //===============================
            TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lat = (int)1278872831;
            p135.lon = (int)862672575;
            CommunicationChannel.instance.send(p135); //===============================
            TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.lat = (int)395534428;
            p136.lon = (int)975428185;
            p136.spacing = (ushort)(ushort)15603;
            p136.terrain_height = (float)1.6483383E38F;
            p136.current_height = (float) -1.1991508E38F;
            p136.pending = (ushort)(ushort)47785;
            p136.loaded = (ushort)(ushort)4357;
            CommunicationChannel.instance.send(p136); //===============================
            SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.time_boot_ms = (uint)373102387U;
            p137.press_abs = (float)3.374189E38F;
            p137.press_diff = (float) -1.8772942E38F;
            p137.temperature = (short)(short)4380;
            CommunicationChannel.instance.send(p137); //===============================
            ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.time_usec = (ulong)1485818847933522066L;
            p138.q_SET(new float[4], 0);
            p138.x = (float)3.3261293E37F;
            p138.y = (float)3.5463522E37F;
            p138.z = (float)3.2631209E38F;
            CommunicationChannel.instance.send(p138); //===============================
            SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.time_usec = (ulong)5660301533689955594L;
            p139.group_mlx = (byte)(byte)181;
            p139.target_system = (byte)(byte)218;
            p139.target_component = (byte)(byte)44;
            p139.controls_SET(new float[8], 0);
            CommunicationChannel.instance.send(p139); //===============================
            ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.time_usec = (ulong)8381091610743790546L;
            p140.group_mlx = (byte)(byte)89;
            p140.controls_SET(new float[8], 0);
            CommunicationChannel.instance.send(p140); //===============================
            ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.time_usec = (ulong)1652530710403246802L;
            p141.altitude_monotonic = (float)8.836124E37F;
            p141.altitude_amsl = (float) -2.8037178E38F;
            p141.altitude_local = (float) -2.361295E38F;
            p141.altitude_relative = (float) -3.1150145E38F;
            p141.altitude_terrain = (float)9.655856E37F;
            p141.bottom_clearance = (float)2.2766755E38F;
            CommunicationChannel.instance.send(p141); //===============================
            RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.request_id = (byte)(byte)207;
            p142.uri_type = (byte)(byte)193;
            p142.uri_SET(new byte[120], 0);
            p142.transfer_type = (byte)(byte)211;
            p142.storage_SET(new byte[120], 0);
            CommunicationChannel.instance.send(p142); //===============================
            SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.time_boot_ms = (uint)2033440827U;
            p143.press_abs = (float) -6.488179E37F;
            p143.press_diff = (float)2.6626153E38F;
            p143.temperature = (short)(short) -28836;
            CommunicationChannel.instance.send(p143); //===============================
            FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.timestamp = (ulong)1684271215340262395L;
            p144.est_capabilities = (byte)(byte)97;
            p144.lat = (int) -934775600;
            p144.lon = (int) -682239598;
            p144.alt = (float)3.366484E38F;
            p144.vel_SET(new float[3], 0);
            p144.acc_SET(new float[3], 0);
            p144.attitude_q_SET(new float[4], 0);
            p144.rates_SET(new float[3], 0);
            p144.position_cov_SET(new float[3], 0);
            p144.custom_state = (ulong)7579815745673855705L;
            CommunicationChannel.instance.send(p144); //===============================
            CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.time_usec = (ulong)7392912292534675326L;
            p146.x_acc = (float) -2.1531005E37F;
            p146.y_acc = (float)3.212624E38F;
            p146.z_acc = (float) -4.0316973E37F;
            p146.x_vel = (float)7.5933706E37F;
            p146.y_vel = (float)1.1077873E38F;
            p146.z_vel = (float)2.5073054E38F;
            p146.x_pos = (float)2.6133765E38F;
            p146.y_pos = (float) -2.6116742E38F;
            p146.z_pos = (float) -3.3961512E38F;
            p146.airspeed = (float)2.7126709E38F;
            p146.vel_variance_SET(new float[3], 0);
            p146.pos_variance_SET(new float[3], 0);
            p146.q_SET(new float[4], 0);
            p146.roll_rate = (float)1.6821761E38F;
            p146.pitch_rate = (float) -1.0856648E38F;
            p146.yaw_rate = (float)2.254488E38F;
            CommunicationChannel.instance.send(p146); //===============================
            BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.id = (byte)(byte)192;
            p147.battery_function = (MAV_BATTERY_FUNCTION)MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_UNKNOWN;
            p147.type = (MAV_BATTERY_TYPE)MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LION;
            p147.temperature = (short)(short) -17468;
            p147.voltages_SET(new ushort[10], 0);
            p147.current_battery = (short)(short)6038;
            p147.current_consumed = (int) -347398432;
            p147.energy_consumed = (int)686741110;
            p147.battery_remaining = (sbyte)(sbyte)60;
            CommunicationChannel.instance.send(p147); //===============================
            AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY)MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT;
            p148.flight_sw_version = (uint)1508424039U;
            p148.middleware_sw_version = (uint)3835849411U;
            p148.os_sw_version = (uint)1443114756U;
            p148.board_version = (uint)3974879369U;
            p148.flight_custom_version_SET(new byte[8], 0);
            p148.middleware_custom_version_SET(new byte[8], 0);
            p148.os_custom_version_SET(new byte[8], 0);
            p148.vendor_id = (ushort)(ushort)29533;
            p148.product_id = (ushort)(ushort)2049;
            p148.uid = (ulong)1816740596088320953L;
            p148.uid2_SET(new byte[18], 0, PH);
            CommunicationChannel.instance.send(p148); //===============================
            LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.time_usec = (ulong)178274910213310192L;
            p149.target_num = (byte)(byte)48;
            p149.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL;
            p149.angle_x = (float)3.1773504E38F;
            p149.angle_y = (float)2.6052236E38F;
            p149.distance = (float) -2.0169871E37F;
            p149.size_x = (float)3.1448458E38F;
            p149.size_y = (float) -1.8180779E38F;
            p149.x_SET((float) -3.1043617E37F, PH);
            p149.y_SET((float)5.2440763E37F, PH);
            p149.z_SET((float)1.4870105E38F, PH);
            p149.q_SET(new float[4], 0, PH);
            p149.type = (LANDING_TARGET_TYPE)LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_FIDUCIAL;
            p149.position_valid_SET((byte)(byte)148, PH);
            CommunicationChannel.instance.send(p149); //===============================
            ARRAY_TEST_0 p150 = CommunicationChannel.new_ARRAY_TEST_0();
            PH.setPack(p150);
            p150.v1 = (byte)(byte)208;
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
            p153.v = (byte)(byte)188;
            p153.ar_u32_SET(new uint[4], 0);
            CommunicationChannel.instance.send(p153); //===============================
            ARRAY_TEST_4 p154 = CommunicationChannel.new_ARRAY_TEST_4();
            PH.setPack(p154);
            p154.ar_u32_SET(new uint[4], 0);
            p154.v = (byte)(byte)112;
            CommunicationChannel.instance.send(p154); //===============================
            ARRAY_TEST_5 p155 = CommunicationChannel.new_ARRAY_TEST_5();
            PH.setPack(p155);
            p155.c1_SET("DEMO", PH);
            p155.c2_SET("DEMO", PH);
            CommunicationChannel.instance.send(p155); //===============================
            ARRAY_TEST_6 p156 = CommunicationChannel.new_ARRAY_TEST_6();
            PH.setPack(p156);
            p156.v1 = (byte)(byte)26;
            p156.v2 = (ushort)(ushort)7396;
            p156.v3 = (uint)2970499242U;
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
            p158.v3 = (uint)4282391921U;
            p158.ar_d_SET(new double[2], 0);
            p158.ar_u16_SET(new ushort[2], 0);
            CommunicationChannel.instance.send(p158); //===============================
            ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.time_usec = (ulong)2249596043346442571L;
            p230.flags = (ESTIMATOR_STATUS_FLAGS)ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL;
            p230.vel_ratio = (float)3.1142881E38F;
            p230.pos_horiz_ratio = (float) -2.8601007E38F;
            p230.pos_vert_ratio = (float)2.7811745E38F;
            p230.mag_ratio = (float)2.2963048E38F;
            p230.hagl_ratio = (float)1.8929334E38F;
            p230.tas_ratio = (float) -1.7854936E38F;
            p230.pos_horiz_accuracy = (float)2.093299E38F;
            p230.pos_vert_accuracy = (float) -1.4646777E38F;
            CommunicationChannel.instance.send(p230); //===============================
            WIND_COV p231 = CommunicationChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.time_usec = (ulong)6951654196744719468L;
            p231.wind_x = (float) -1.776879E38F;
            p231.wind_y = (float) -5.466672E37F;
            p231.wind_z = (float) -3.9031132E37F;
            p231.var_horiz = (float) -9.670662E37F;
            p231.var_vert = (float) -1.7558823E38F;
            p231.wind_alt = (float)2.4814096E38F;
            p231.horiz_accuracy = (float)1.2711476E38F;
            p231.vert_accuracy = (float) -9.974006E37F;
            CommunicationChannel.instance.send(p231); //===============================
            GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.time_usec = (ulong)1432168272579479993L;
            p232.gps_id = (byte)(byte)230;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS)GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY;
            p232.time_week_ms = (uint)2039364196U;
            p232.time_week = (ushort)(ushort)61089;
            p232.fix_type = (byte)(byte)84;
            p232.lat = (int) -1172386812;
            p232.lon = (int) -894133226;
            p232.alt = (float) -2.704078E38F;
            p232.hdop = (float) -3.0057136E38F;
            p232.vdop = (float) -8.692568E37F;
            p232.vn = (float)1.790093E38F;
            p232.ve = (float) -1.3994558E38F;
            p232.vd = (float) -3.3165816E38F;
            p232.speed_accuracy = (float) -1.5338974E38F;
            p232.horiz_accuracy = (float)1.1222936E38F;
            p232.vert_accuracy = (float) -2.7303486E38F;
            p232.satellites_visible = (byte)(byte)249;
            CommunicationChannel.instance.send(p232); //===============================
            GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.flags = (byte)(byte)222;
            p233.len = (byte)(byte)18;
            p233.data__SET(new byte[180], 0);
            CommunicationChannel.instance.send(p233); //===============================
            HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED;
            p234.custom_mode = (uint)1247011791U;
            p234.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING;
            p234.roll = (short)(short)20695;
            p234.pitch = (short)(short) -17680;
            p234.heading = (ushort)(ushort)37557;
            p234.throttle = (sbyte)(sbyte)30;
            p234.heading_sp = (short)(short)13103;
            p234.latitude = (int) -984296706;
            p234.longitude = (int)311171490;
            p234.altitude_amsl = (short)(short) -8514;
            p234.altitude_sp = (short)(short) -7346;
            p234.airspeed = (byte)(byte)20;
            p234.airspeed_sp = (byte)(byte)48;
            p234.groundspeed = (byte)(byte)193;
            p234.climb_rate = (sbyte)(sbyte)23;
            p234.gps_nsat = (byte)(byte)236;
            p234.gps_fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX;
            p234.battery_remaining = (byte)(byte)39;
            p234.temperature = (sbyte)(sbyte) - 80;
            p234.temperature_air = (sbyte)(sbyte)39;
            p234.failsafe = (byte)(byte)37;
            p234.wp_num = (byte)(byte)239;
            p234.wp_distance = (ushort)(ushort)63608;
            CommunicationChannel.instance.send(p234); //===============================
            VIBRATION p241 = CommunicationChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.time_usec = (ulong)1941326470824288673L;
            p241.vibration_x = (float) -1.4509336E38F;
            p241.vibration_y = (float) -2.8600456E37F;
            p241.vibration_z = (float) -1.9701252E38F;
            p241.clipping_0 = (uint)309791413U;
            p241.clipping_1 = (uint)2800096627U;
            p241.clipping_2 = (uint)77567535U;
            CommunicationChannel.instance.send(p241); //===============================
            HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.latitude = (int) -1080085603;
            p242.longitude = (int)37553861;
            p242.altitude = (int) -3258276;
            p242.x = (float)9.430837E37F;
            p242.y = (float)3.2075195E38F;
            p242.z = (float)3.132345E38F;
            p242.q_SET(new float[4], 0);
            p242.approach_x = (float) -1.8752844E38F;
            p242.approach_y = (float)1.9068315E38F;
            p242.approach_z = (float)4.422087E37F;
            p242.time_usec_SET((ulong)9027829458982128L, PH);
            CommunicationChannel.instance.send(p242); //===============================
            SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.target_system = (byte)(byte)244;
            p243.latitude = (int)1246743773;
            p243.longitude = (int) -2124214452;
            p243.altitude = (int) -1723355162;
            p243.x = (float)1.0993016E38F;
            p243.y = (float)5.1642356E37F;
            p243.z = (float) -6.9280127E37F;
            p243.q_SET(new float[4], 0);
            p243.approach_x = (float)2.114255E38F;
            p243.approach_y = (float)2.9061586E38F;
            p243.approach_z = (float)2.2129894E38F;
            p243.time_usec_SET((ulong)7704635669220947967L, PH);
            CommunicationChannel.instance.send(p243); //===============================
            MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.message_id = (ushort)(ushort)1793;
            p244.interval_us = (int)1405086199;
            CommunicationChannel.instance.send(p244); //===============================
            EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.vtol_state = (MAV_VTOL_STATE)MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_MC;
            p245.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED;
            CommunicationChannel.instance.send(p245); //===============================
            ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.ICAO_address = (uint)171673815U;
            p246.lat = (int) -623639693;
            p246.lon = (int) -1255143242;
            p246.altitude_type = (ADSB_ALTITUDE_TYPE)ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
            p246.altitude = (int) -1758103296;
            p246.heading = (ushort)(ushort)44660;
            p246.hor_velocity = (ushort)(ushort)33982;
            p246.ver_velocity = (short)(short)2081;
            p246.callsign_SET("DEMO", PH);
            p246.emitter_type = (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_EMERGENCY_SURFACE;
            p246.tslc = (byte)(byte)190;
            p246.flags = (ADSB_FLAGS)ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE;
            p246.squawk = (ushort)(ushort)22550;
            CommunicationChannel.instance.send(p246); //===============================
            COLLISION p247 = CommunicationChannel.new_COLLISION();
            PH.setPack(p247);
            p247.src_ = (MAV_COLLISION_SRC)MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB;
            p247.id = (uint)2213910688U;
            p247.action = (MAV_COLLISION_ACTION)MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_PERPENDICULAR;
            p247.threat_level = (MAV_COLLISION_THREAT_LEVEL)MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW;
            p247.time_to_minimum_delta = (float) -2.5089535E38F;
            p247.altitude_minimum_delta = (float) -7.359679E37F;
            p247.horizontal_minimum_delta = (float)1.0285248E38F;
            CommunicationChannel.instance.send(p247); //===============================
            V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.target_network = (byte)(byte)211;
            p248.target_system = (byte)(byte)114;
            p248.target_component = (byte)(byte)93;
            p248.message_type = (ushort)(ushort)52881;
            p248.payload_SET(new byte[249], 0);
            CommunicationChannel.instance.send(p248); //===============================
            MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.address = (ushort)(ushort)3969;
            p249.ver = (byte)(byte)74;
            p249.type = (byte)(byte)25;
            p249.value_SET(new sbyte[32], 0);
            CommunicationChannel.instance.send(p249); //===============================
            DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.name_SET("DEMO", PH);
            p250.time_usec = (ulong)4999044874834261698L;
            p250.x = (float)3.3878191E38F;
            p250.y = (float)2.3199243E38F;
            p250.z = (float)8.459678E37F;
            CommunicationChannel.instance.send(p250); //===============================
            NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.time_boot_ms = (uint)4117800241U;
            p251.name_SET("DEMO", PH);
            p251.value = (float)2.160821E38F;
            CommunicationChannel.instance.send(p251); //===============================
            NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.time_boot_ms = (uint)3620909176U;
            p252.name_SET("DEMO", PH);
            p252.value = (int)1628133331;
            CommunicationChannel.instance.send(p252); //===============================
            STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.severity = (MAV_SEVERITY)MAV_SEVERITY.MAV_SEVERITY_WARNING;
            p253.text_SET("DEMO", PH);
            CommunicationChannel.instance.send(p253); //===============================
            DEBUG p254 = CommunicationChannel.new_DEBUG();
            PH.setPack(p254);
            p254.time_boot_ms = (uint)3445050822U;
            p254.ind = (byte)(byte)95;
            p254.value = (float) -1.4660744E38F;
            CommunicationChannel.instance.send(p254); //===============================
            SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.target_system = (byte)(byte)83;
            p256.target_component = (byte)(byte)35;
            p256.secret_key_SET(new byte[32], 0);
            p256.initial_timestamp = (ulong)3067243972243882641L;
            CommunicationChannel.instance.send(p256); //===============================
            BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.time_boot_ms = (uint)1059130971U;
            p257.last_change_ms = (uint)2355370134U;
            p257.state = (byte)(byte)7;
            CommunicationChannel.instance.send(p257); //===============================
            PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_system = (byte)(byte)11;
            p258.target_component = (byte)(byte)136;
            p258.tune_SET("DEMO", PH);
            CommunicationChannel.instance.send(p258); //===============================
            CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.time_boot_ms = (uint)362202800U;
            p259.vendor_name_SET(new byte[32], 0);
            p259.model_name_SET(new byte[32], 0);
            p259.firmware_version = (uint)3374492148U;
            p259.focal_length = (float)3.4316334E37F;
            p259.sensor_size_h = (float) -3.0903721E38F;
            p259.sensor_size_v = (float) -5.4837525E37F;
            p259.resolution_h = (ushort)(ushort)59413;
            p259.resolution_v = (ushort)(ushort)23976;
            p259.lens_id = (byte)(byte)9;
            p259.flags = (CAMERA_CAP_FLAGS)CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE;
            p259.cam_definition_version = (ushort)(ushort)2619;
            p259.cam_definition_uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p259); //===============================
            CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.time_boot_ms = (uint)1205084593U;
            p260.mode_id = (CAMERA_MODE)CAMERA_MODE.CAMERA_MODE_IMAGE;
            CommunicationChannel.instance.send(p260); //===============================
            STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.time_boot_ms = (uint)2130193769U;
            p261.storage_id = (byte)(byte)46;
            p261.storage_count = (byte)(byte)251;
            p261.status = (byte)(byte)117;
            p261.total_capacity = (float)8.685449E37F;
            p261.used_capacity = (float)3.073707E38F;
            p261.available_capacity = (float)1.6923512E38F;
            p261.read_speed = (float)2.0846078E38F;
            p261.write_speed = (float) -3.2080568E37F;
            CommunicationChannel.instance.send(p261); //===============================
            CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.time_boot_ms = (uint)2507428261U;
            p262.image_status = (byte)(byte)82;
            p262.video_status = (byte)(byte)248;
            p262.image_interval = (float)1.8284752E38F;
            p262.recording_time_ms = (uint)3723520971U;
            p262.available_capacity = (float)3.0893596E38F;
            CommunicationChannel.instance.send(p262); //===============================
            CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.time_boot_ms = (uint)3923977851U;
            p263.time_utc = (ulong)5740443670735154104L;
            p263.camera_id = (byte)(byte)215;
            p263.lat = (int)1561779259;
            p263.lon = (int) -2123465417;
            p263.alt = (int) -2113970038;
            p263.relative_alt = (int) -1048221579;
            p263.q_SET(new float[4], 0);
            p263.image_index = (int) -316554434;
            p263.capture_result = (sbyte)(sbyte) - 22;
            p263.file_url_SET("DEMO", PH);
            CommunicationChannel.instance.send(p263); //===============================
            FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.time_boot_ms = (uint)2915655313U;
            p264.arming_time_utc = (ulong)4321196442498014953L;
            p264.takeoff_time_utc = (ulong)7863656541937488730L;
            p264.flight_uuid = (ulong)6945606706432605453L;
            CommunicationChannel.instance.send(p264); //===============================
            MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.time_boot_ms = (uint)3626834678U;
            p265.roll = (float)2.673713E38F;
            p265.pitch = (float) -9.530631E37F;
            p265.yaw = (float)2.6870922E37F;
            CommunicationChannel.instance.send(p265); //===============================
            LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.target_system = (byte)(byte)243;
            p266.target_component = (byte)(byte)20;
            p266.sequence = (ushort)(ushort)53071;
            p266.length = (byte)(byte)54;
            p266.first_message_offset = (byte)(byte)125;
            p266.data__SET(new byte[249], 0);
            CommunicationChannel.instance.send(p266); //===============================
            LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.target_system = (byte)(byte)114;
            p267.target_component = (byte)(byte)214;
            p267.sequence = (ushort)(ushort)37365;
            p267.length = (byte)(byte)67;
            p267.first_message_offset = (byte)(byte)3;
            p267.data__SET(new byte[249], 0);
            CommunicationChannel.instance.send(p267); //===============================
            LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_system = (byte)(byte)61;
            p268.target_component = (byte)(byte)69;
            p268.sequence = (ushort)(ushort)28804;
            CommunicationChannel.instance.send(p268); //===============================
            VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.camera_id = (byte)(byte)30;
            p269.status = (byte)(byte)148;
            p269.framerate = (float) -6.2424186E37F;
            p269.resolution_h = (ushort)(ushort)8506;
            p269.resolution_v = (ushort)(ushort)11725;
            p269.bitrate = (uint)521701002U;
            p269.rotation = (ushort)(ushort)27765;
            p269.uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p269); //===============================
            SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.target_system = (byte)(byte)185;
            p270.target_component = (byte)(byte)96;
            p270.camera_id = (byte)(byte)26;
            p270.framerate = (float) -1.5318328E38F;
            p270.resolution_h = (ushort)(ushort)34922;
            p270.resolution_v = (ushort)(ushort)16672;
            p270.bitrate = (uint)3126144840U;
            p270.rotation = (ushort)(ushort)53059;
            p270.uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p270); //===============================
            WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.ssid_SET("DEMO", PH);
            p299.password_SET("DEMO", PH);
            CommunicationChannel.instance.send(p299); //===============================
            PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.version = (ushort)(ushort)58565;
            p300.min_version = (ushort)(ushort)42882;
            p300.max_version = (ushort)(ushort)21790;
            p300.spec_version_hash_SET(new byte[8], 0);
            p300.library_version_hash_SET(new byte[8], 0);
            CommunicationChannel.instance.send(p300); //===============================
            UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.time_usec = (ulong)9001021037558288734L;
            p310.uptime_sec = (uint)3705654525U;
            p310.health = (UAVCAN_NODE_HEALTH)UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL;
            p310.mode = (UAVCAN_NODE_MODE)UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION;
            p310.sub_mode = (byte)(byte)208;
            p310.vendor_specific_status_code = (ushort)(ushort)64257;
            CommunicationChannel.instance.send(p310); //===============================
            UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.time_usec = (ulong)1506368823021083305L;
            p311.uptime_sec = (uint)3461587637U;
            p311.name_SET("DEMO", PH);
            p311.hw_version_major = (byte)(byte)159;
            p311.hw_version_minor = (byte)(byte)3;
            p311.hw_unique_id_SET(new byte[16], 0);
            p311.sw_version_major = (byte)(byte)144;
            p311.sw_version_minor = (byte)(byte)135;
            p311.sw_vcs_commit = (uint)2231605361U;
            CommunicationChannel.instance.send(p311); //===============================
            PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.target_system = (byte)(byte)203;
            p320.target_component = (byte)(byte)153;
            p320.param_id_SET("DEMO", PH);
            p320.param_index = (short)(short) -18492;
            CommunicationChannel.instance.send(p320); //===============================
            PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_system = (byte)(byte)249;
            p321.target_component = (byte)(byte)195;
            CommunicationChannel.instance.send(p321); //===============================
            PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_id_SET("DEMO", PH);
            p322.param_value_SET("DEMO", PH);
            p322.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT64;
            p322.param_count = (ushort)(ushort)61951;
            p322.param_index = (ushort)(ushort)42539;
            CommunicationChannel.instance.send(p322); //===============================
            PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.target_system = (byte)(byte)33;
            p323.target_component = (byte)(byte)87;
            p323.param_id_SET("DEMO", PH);
            p323.param_value_SET("DEMO", PH);
            p323.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16;
            CommunicationChannel.instance.send(p323); //===============================
            PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_id_SET("DEMO", PH);
            p324.param_value_SET("DEMO", PH);
            p324.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT64;
            p324.param_result = (PARAM_ACK)PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED;
            CommunicationChannel.instance.send(p324); //===============================
            OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.time_usec = (ulong)6579015635127197703L;
            p330.sensor_type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED;
            p330.distances_SET(new ushort[72], 0);
            p330.increment = (byte)(byte)75;
            p330.min_distance = (ushort)(ushort)25686;
            p330.max_distance = (ushort)(ushort)24383;
            CommunicationChannel.instance.send(p330); //===============================
        }
    }
}

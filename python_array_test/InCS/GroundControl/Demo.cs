
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
            p3.time_boot_ms = (uint)2772026903U;
            p3.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_NED;
            p3.type_mask = (ushort)(ushort)14863;
            p3.x = (float)2.8329407E38F;
            p3.y = (float) -8.541338E37F;
            p3.z = (float) -2.7682978E38F;
            p3.vx = (float)1.3673451E38F;
            p3.vy = (float)1.7738083E38F;
            p3.vz = (float)3.190661E38F;
            p3.afx = (float) -3.1988826E38F;
            p3.afy = (float) -8.53496E37F;
            p3.afz = (float)2.4093599E37F;
            p3.yaw = (float) -1.7279558E38F;
            p3.yaw_rate = (float)2.4764717E38F;
            CommunicationChannel.instance.send(p3); //===============================
            COMMAND_ACK p77 = CommunicationChannel.new_COMMAND_ACK();
            PH.setPack(p77);
            p77.command = MAV_CMD.MAV_CMD_NAV_SPLINE_WAYPOINT;
            p77.result = MAV_RESULT.MAV_RESULT_UNSUPPORTED;
            p77.progress_SET((byte)(byte)205, PH);
            p77.result_param2_SET((int)570556816, PH);
            p77.target_system_SET((byte)(byte)25, PH);
            p77.target_component_SET((byte)(byte)100, PH);
            CommunicationChannel.instance.send(p77); //===============================
            MANUAL_SETPOINT p81 = CommunicationChannel.new_MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.time_boot_ms = (uint)3829481946U;
            p81.roll = (float)4.356873E36F;
            p81.pitch = (float)1.9024904E38F;
            p81.yaw = (float) -1.311286E38F;
            p81.thrust = (float) -3.2720902E37F;
            p81.mode_switch = (byte)(byte)181;
            p81.manual_override_switch = (byte)(byte)158;
            CommunicationChannel.instance.send(p81); //===============================
            SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.time_boot_ms = (uint)1117316979U;
            p82.target_system = (byte)(byte)190;
            p82.target_component = (byte)(byte)2;
            p82.type_mask = (byte)(byte)163;
            p82.q_SET(new float[4], 0);
            p82.body_roll_rate = (float) -3.0070745E38F;
            p82.body_pitch_rate = (float) -1.3096255E38F;
            p82.body_yaw_rate = (float) -1.1376776E38F;
            p82.thrust = (float) -2.4315285E38F;
            CommunicationChannel.instance.send(p82); //===============================
            ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.time_boot_ms = (uint)195196476U;
            p83.type_mask = (byte)(byte)36;
            p83.q_SET(new float[4], 0);
            p83.body_roll_rate = (float)1.1847285E38F;
            p83.body_pitch_rate = (float)1.6395131E38F;
            p83.body_yaw_rate = (float) -2.4481627E38F;
            p83.thrust = (float) -1.1109239E38F;
            CommunicationChannel.instance.send(p83); //===============================
            SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.time_boot_ms = (uint)1988677556U;
            p84.target_system = (byte)(byte)1;
            p84.target_component = (byte)(byte)67;
            p84.coordinate_frame = MAV_FRAME.MAV_FRAME_BODY_NED;
            p84.type_mask = (ushort)(ushort)59087;
            p84.x = (float)6.4838376E37F;
            p84.y = (float)2.2701134E38F;
            p84.z = (float) -2.8221304E38F;
            p84.vx = (float) -1.0887239E37F;
            p84.vy = (float) -2.9267032E38F;
            p84.vz = (float)6.2714174E37F;
            p84.afx = (float) -2.7322507E38F;
            p84.afy = (float)1.961144E36F;
            p84.afz = (float) -2.394595E38F;
            p84.yaw = (float) -2.506026E38F;
            p84.yaw_rate = (float)1.5720574E38F;
            CommunicationChannel.instance.send(p84); //===============================
            SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.time_boot_ms = (uint)1696103902U;
            p86.target_system = (byte)(byte)31;
            p86.target_component = (byte)(byte)125;
            p86.coordinate_frame = MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p86.type_mask = (ushort)(ushort)11616;
            p86.lat_int = (int) -1839592932;
            p86.lon_int = (int) -2106168501;
            p86.alt = (float)3.2331031E38F;
            p86.vx = (float) -1.3374559E38F;
            p86.vy = (float)1.2492153E37F;
            p86.vz = (float)3.3021152E38F;
            p86.afx = (float)8.2539616E37F;
            p86.afy = (float)1.9753783E38F;
            p86.afz = (float) -1.3687309E38F;
            p86.yaw = (float)2.4795436E38F;
            p86.yaw_rate = (float)1.638416E38F;
            CommunicationChannel.instance.send(p86); //===============================
            POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.time_boot_ms = (uint)2400312281U;
            p87.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
            p87.type_mask = (ushort)(ushort)55008;
            p87.lat_int = (int)757734263;
            p87.lon_int = (int) -1711581762;
            p87.alt = (float)1.0741596E38F;
            p87.vx = (float) -5.172014E37F;
            p87.vy = (float) -8.823801E37F;
            p87.vz = (float)1.2502953E38F;
            p87.afx = (float) -8.2739027E37F;
            p87.afy = (float) -2.694917E38F;
            p87.afz = (float) -3.271642E38F;
            p87.yaw = (float)2.5486756E38F;
            p87.yaw_rate = (float)1.7257584E38F;
            CommunicationChannel.instance.send(p87); //===============================
            LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.time_boot_ms = (uint)2473444246U;
            p89.x = (float)6.8779943E37F;
            p89.y = (float) -1.3480132E38F;
            p89.z = (float) -7.2466626E37F;
            p89.roll = (float) -2.4302073E38F;
            p89.pitch = (float) -1.6255514E38F;
            p89.yaw = (float) -6.5505515E36F;
            CommunicationChannel.instance.send(p89); //===============================
            HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.time_usec = (ulong)3665995440658383868L;
            p90.roll = (float)2.2268926E38F;
            p90.pitch = (float) -2.2165088E38F;
            p90.yaw = (float) -2.1237399E38F;
            p90.rollspeed = (float) -3.3997052E38F;
            p90.pitchspeed = (float) -1.2377422E38F;
            p90.yawspeed = (float) -2.8958932E38F;
            p90.lat = (int) -2107573445;
            p90.lon = (int)1307798037;
            p90.alt = (int) -1534383771;
            p90.vx = (short)(short)2715;
            p90.vy = (short)(short)4543;
            p90.vz = (short)(short) -8035;
            p90.xacc = (short)(short) -11234;
            p90.yacc = (short)(short) -19772;
            p90.zacc = (short)(short) -31070;
            CommunicationChannel.instance.send(p90); //===============================
            HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.time_usec = (ulong)1749507447953255823L;
            p91.roll_ailerons = (float)2.759971E38F;
            p91.pitch_elevator = (float)1.9543081E38F;
            p91.yaw_rudder = (float) -3.1036616E38F;
            p91.throttle = (float) -2.1412316E38F;
            p91.aux1 = (float) -7.8003404E37F;
            p91.aux2 = (float)3.3032875E38F;
            p91.aux3 = (float)1.0410007E38F;
            p91.aux4 = (float)1.3929488E38F;
            p91.mode = MAV_MODE.MAV_MODE_GUIDED_DISARMED;
            p91.nav_mode = (byte)(byte)148;
            CommunicationChannel.instance.send(p91); //===============================
            HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.time_usec = (ulong)9186303644112471809L;
            p92.chan1_raw = (ushort)(ushort)31684;
            p92.chan2_raw = (ushort)(ushort)33565;
            p92.chan3_raw = (ushort)(ushort)50282;
            p92.chan4_raw = (ushort)(ushort)63942;
            p92.chan5_raw = (ushort)(ushort)6337;
            p92.chan6_raw = (ushort)(ushort)29385;
            p92.chan7_raw = (ushort)(ushort)18406;
            p92.chan8_raw = (ushort)(ushort)25871;
            p92.chan9_raw = (ushort)(ushort)40612;
            p92.chan10_raw = (ushort)(ushort)25484;
            p92.chan11_raw = (ushort)(ushort)4558;
            p92.chan12_raw = (ushort)(ushort)18621;
            p92.rssi = (byte)(byte)235;
            CommunicationChannel.instance.send(p92); //===============================
            HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.time_usec = (ulong)289646937000341288L;
            p93.controls_SET(new float[16], 0);
            p93.mode = MAV_MODE.MAV_MODE_GUIDED_DISARMED;
            p93.flags = (ulong)6333442453746375738L;
            CommunicationChannel.instance.send(p93); //===============================
            OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.time_usec = (ulong)5799942215293232757L;
            p100.sensor_id = (byte)(byte)6;
            p100.flow_x = (short)(short)2245;
            p100.flow_y = (short)(short)8402;
            p100.flow_comp_m_x = (float) -1.0517399E38F;
            p100.flow_comp_m_y = (float)1.8219654E38F;
            p100.quality = (byte)(byte)43;
            p100.ground_distance = (float) -1.849683E38F;
            p100.flow_rate_x_SET((float) -2.6357304E38F, PH);
            p100.flow_rate_y_SET((float) -4.832256E37F, PH);
            CommunicationChannel.instance.send(p100); //===============================
            GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.usec = (ulong)245116907854039606L;
            p101.x = (float)3.7452103E36F;
            p101.y = (float)2.7294517E38F;
            p101.z = (float)3.2651448E38F;
            p101.roll = (float) -7.149044E37F;
            p101.pitch = (float) -2.154634E38F;
            p101.yaw = (float) -2.4573433E38F;
            CommunicationChannel.instance.send(p101); //===============================
            VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.usec = (ulong)3108997074891053163L;
            p102.x = (float) -3.3752627E38F;
            p102.y = (float)8.263372E35F;
            p102.z = (float) -2.6864087E37F;
            p102.roll = (float)1.2372278E38F;
            p102.pitch = (float)9.786336E37F;
            p102.yaw = (float) -2.1939357E38F;
            CommunicationChannel.instance.send(p102); //===============================
            VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.usec = (ulong)5407303828667895730L;
            p103.x = (float) -3.9243007E37F;
            p103.y = (float) -9.6231616E36F;
            p103.z = (float) -3.2718214E38F;
            CommunicationChannel.instance.send(p103); //===============================
            VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.usec = (ulong)1445525630699074341L;
            p104.x = (float)1.6781465E38F;
            p104.y = (float) -1.3511546E38F;
            p104.z = (float)2.9397606E38F;
            p104.roll = (float) -1.2288644E38F;
            p104.pitch = (float)1.242193E38F;
            p104.yaw = (float) -7.1932767E37F;
            CommunicationChannel.instance.send(p104); //===============================
            HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.time_usec = (ulong)1999336644561741491L;
            p105.xacc = (float) -1.7280372E38F;
            p105.yacc = (float)3.3240565E38F;
            p105.zacc = (float)9.106402E37F;
            p105.xgyro = (float) -2.9091409E38F;
            p105.ygyro = (float) -3.0108403E38F;
            p105.zgyro = (float)3.1492165E38F;
            p105.xmag = (float) -2.9896243E38F;
            p105.ymag = (float)1.6184546E38F;
            p105.zmag = (float)2.4461434E38F;
            p105.abs_pressure = (float) -1.1097828E38F;
            p105.diff_pressure = (float)3.031113E37F;
            p105.pressure_alt = (float) -2.0295228E38F;
            p105.temperature = (float)9.42101E37F;
            p105.fields_updated = (ushort)(ushort)42737;
            CommunicationChannel.instance.send(p105); //===============================
            OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.time_usec = (ulong)876198718847727062L;
            p106.sensor_id = (byte)(byte)237;
            p106.integration_time_us = (uint)2907864193U;
            p106.integrated_x = (float) -2.7093673E38F;
            p106.integrated_y = (float)3.344692E38F;
            p106.integrated_xgyro = (float)1.8345285E38F;
            p106.integrated_ygyro = (float)2.4045196E38F;
            p106.integrated_zgyro = (float) -1.9950774E38F;
            p106.temperature = (short)(short) -22081;
            p106.quality = (byte)(byte)106;
            p106.time_delta_distance_us = (uint)2107879938U;
            p106.distance = (float)3.0345434E38F;
            CommunicationChannel.instance.send(p106); //===============================
            HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.time_usec = (ulong)7857744210434727176L;
            p107.xacc = (float) -1.2363382E38F;
            p107.yacc = (float)2.056817E38F;
            p107.zacc = (float)1.4610385E38F;
            p107.xgyro = (float)2.2121565E37F;
            p107.ygyro = (float)3.19435E38F;
            p107.zgyro = (float) -1.4029385E38F;
            p107.xmag = (float) -9.622701E37F;
            p107.ymag = (float) -1.0664868E38F;
            p107.zmag = (float)1.2097301E38F;
            p107.abs_pressure = (float) -7.360912E37F;
            p107.diff_pressure = (float) -9.24636E37F;
            p107.pressure_alt = (float) -1.412106E38F;
            p107.temperature = (float) -2.6741736E38F;
            p107.fields_updated = (uint)170440607U;
            CommunicationChannel.instance.send(p107); //===============================
            SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.q1 = (float)5.29635E37F;
            p108.q2 = (float) -1.5857691E38F;
            p108.q3 = (float) -7.5044054E37F;
            p108.q4 = (float) -1.9142234E38F;
            p108.roll = (float)1.0869197E38F;
            p108.pitch = (float) -2.175329E38F;
            p108.yaw = (float) -1.9800794E38F;
            p108.xacc = (float)1.3782231E38F;
            p108.yacc = (float)2.4890585E38F;
            p108.zacc = (float)2.3412209E38F;
            p108.xgyro = (float) -1.9102338E38F;
            p108.ygyro = (float)1.1536587E38F;
            p108.zgyro = (float)1.3890317E38F;
            p108.lat = (float) -1.1341071E38F;
            p108.lon = (float)2.3784916E37F;
            p108.alt = (float) -3.373734E37F;
            p108.std_dev_horz = (float)1.9977527E37F;
            p108.std_dev_vert = (float)3.2179418E38F;
            p108.vn = (float) -3.1883757E38F;
            p108.ve = (float)2.7277634E38F;
            p108.vd = (float)1.7742506E38F;
            CommunicationChannel.instance.send(p108); //===============================
            RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.rssi = (byte)(byte)1;
            p109.remrssi = (byte)(byte)215;
            p109.txbuf = (byte)(byte)182;
            p109.noise = (byte)(byte)106;
            p109.remnoise = (byte)(byte)252;
            p109.rxerrors = (ushort)(ushort)9421;
            p109.fixed_ = (ushort)(ushort)52037;
            CommunicationChannel.instance.send(p109); //===============================
            FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_network = (byte)(byte)153;
            p110.target_system = (byte)(byte)73;
            p110.target_component = (byte)(byte)7;
            p110.payload_SET(new byte[251], 0);
            CommunicationChannel.instance.send(p110); //===============================
            TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.tc1 = (long) -7502906876435255637L;
            p111.ts1 = (long) -1610927381418822937L;
            CommunicationChannel.instance.send(p111); //===============================
            CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.time_usec = (ulong)2369185517841271480L;
            p112.seq = (uint)61502215U;
            CommunicationChannel.instance.send(p112); //===============================
            HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.time_usec = (ulong)3490668904619155763L;
            p113.fix_type = (byte)(byte)71;
            p113.lat = (int)1691771087;
            p113.lon = (int) -1844520260;
            p113.alt = (int)92154066;
            p113.eph = (ushort)(ushort)52209;
            p113.epv = (ushort)(ushort)40967;
            p113.vel = (ushort)(ushort)50898;
            p113.vn = (short)(short) -16090;
            p113.ve = (short)(short) -4082;
            p113.vd = (short)(short)25869;
            p113.cog = (ushort)(ushort)51659;
            p113.satellites_visible = (byte)(byte)71;
            CommunicationChannel.instance.send(p113); //===============================
            HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.time_usec = (ulong)6759963457275370401L;
            p114.sensor_id = (byte)(byte)208;
            p114.integration_time_us = (uint)846322963U;
            p114.integrated_x = (float)3.5996848E37F;
            p114.integrated_y = (float) -5.427212E37F;
            p114.integrated_xgyro = (float) -1.5825702E38F;
            p114.integrated_ygyro = (float)2.1431603E38F;
            p114.integrated_zgyro = (float)3.246128E38F;
            p114.temperature = (short)(short)30374;
            p114.quality = (byte)(byte)190;
            p114.time_delta_distance_us = (uint)2630475389U;
            p114.distance = (float) -4.9963115E37F;
            CommunicationChannel.instance.send(p114); //===============================
            HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.time_usec = (ulong)747476296014262079L;
            p115.attitude_quaternion_SET(new float[4], 0);
            p115.rollspeed = (float) -2.7714187E38F;
            p115.pitchspeed = (float)3.393076E38F;
            p115.yawspeed = (float) -4.3278717E37F;
            p115.lat = (int)856044069;
            p115.lon = (int)1503347141;
            p115.alt = (int) -669744374;
            p115.vx = (short)(short) -14841;
            p115.vy = (short)(short) -24076;
            p115.vz = (short)(short)27323;
            p115.ind_airspeed = (ushort)(ushort)19847;
            p115.true_airspeed = (ushort)(ushort)38293;
            p115.xacc = (short)(short)6610;
            p115.yacc = (short)(short) -19509;
            p115.zacc = (short)(short) -32150;
            CommunicationChannel.instance.send(p115); //===============================
            SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.time_boot_ms = (uint)109962670U;
            p116.xacc = (short)(short)19252;
            p116.yacc = (short)(short) -31791;
            p116.zacc = (short)(short) -25883;
            p116.xgyro = (short)(short)18715;
            p116.ygyro = (short)(short)1327;
            p116.zgyro = (short)(short) -11398;
            p116.xmag = (short)(short) -13022;
            p116.ymag = (short)(short)456;
            p116.zmag = (short)(short) -30174;
            CommunicationChannel.instance.send(p116); //===============================
            LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.target_system = (byte)(byte)151;
            p117.target_component = (byte)(byte)201;
            p117.start = (ushort)(ushort)12493;
            p117.end = (ushort)(ushort)24833;
            CommunicationChannel.instance.send(p117); //===============================
            LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.id = (ushort)(ushort)724;
            p118.num_logs = (ushort)(ushort)26422;
            p118.last_log_num = (ushort)(ushort)6620;
            p118.time_utc = (uint)950316072U;
            p118.size = (uint)2683224522U;
            CommunicationChannel.instance.send(p118); //===============================
            LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.target_system = (byte)(byte)159;
            p119.target_component = (byte)(byte)3;
            p119.id = (ushort)(ushort)17948;
            p119.ofs = (uint)2568761966U;
            p119.count = (uint)755765529U;
            CommunicationChannel.instance.send(p119); //===============================
            LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.id = (ushort)(ushort)42679;
            p120.ofs = (uint)2762852389U;
            p120.count = (byte)(byte)1;
            p120.data__SET(new byte[90], 0);
            CommunicationChannel.instance.send(p120); //===============================
            LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_system = (byte)(byte)95;
            p121.target_component = (byte)(byte)132;
            CommunicationChannel.instance.send(p121); //===============================
            LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_system = (byte)(byte)168;
            p122.target_component = (byte)(byte)174;
            CommunicationChannel.instance.send(p122); //===============================
            GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.target_system = (byte)(byte)108;
            p123.target_component = (byte)(byte)98;
            p123.len = (byte)(byte)150;
            p123.data__SET(new byte[110], 0);
            CommunicationChannel.instance.send(p123); //===============================
            GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.time_usec = (ulong)1977363312501712066L;
            p124.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS;
            p124.lat = (int)734289485;
            p124.lon = (int) -168516409;
            p124.alt = (int)818166063;
            p124.eph = (ushort)(ushort)4464;
            p124.epv = (ushort)(ushort)26454;
            p124.vel = (ushort)(ushort)7485;
            p124.cog = (ushort)(ushort)26036;
            p124.satellites_visible = (byte)(byte)16;
            p124.dgps_numch = (byte)(byte)143;
            p124.dgps_age = (uint)1967245919U;
            CommunicationChannel.instance.send(p124); //===============================
            POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vcc = (ushort)(ushort)11354;
            p125.Vservo = (ushort)(ushort)65262;
            p125.flags = (MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED |
                          MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID);
            CommunicationChannel.instance.send(p125); //===============================
            SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.device = SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_SHELL;
            p126.flags = (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND |
                          SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE);
            p126.timeout = (ushort)(ushort)15484;
            p126.baudrate = (uint)1563903381U;
            p126.count = (byte)(byte)231;
            p126.data__SET(new byte[70], 0);
            CommunicationChannel.instance.send(p126); //===============================
            GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.time_last_baseline_ms = (uint)3621643252U;
            p127.rtk_receiver_id = (byte)(byte)234;
            p127.wn = (ushort)(ushort)4885;
            p127.tow = (uint)3474983239U;
            p127.rtk_health = (byte)(byte)55;
            p127.rtk_rate = (byte)(byte)27;
            p127.nsats = (byte)(byte)152;
            p127.baseline_coords_type = (byte)(byte)96;
            p127.baseline_a_mm = (int) -2120773771;
            p127.baseline_b_mm = (int)815000411;
            p127.baseline_c_mm = (int) -1879210988;
            p127.accuracy = (uint)2577451723U;
            p127.iar_num_hypotheses = (int) -1420325318;
            CommunicationChannel.instance.send(p127); //===============================
            GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.time_last_baseline_ms = (uint)2818469211U;
            p128.rtk_receiver_id = (byte)(byte)41;
            p128.wn = (ushort)(ushort)38772;
            p128.tow = (uint)1563303687U;
            p128.rtk_health = (byte)(byte)199;
            p128.rtk_rate = (byte)(byte)3;
            p128.nsats = (byte)(byte)111;
            p128.baseline_coords_type = (byte)(byte)25;
            p128.baseline_a_mm = (int)871628033;
            p128.baseline_b_mm = (int)153217059;
            p128.baseline_c_mm = (int)946433426;
            p128.accuracy = (uint)2501548815U;
            p128.iar_num_hypotheses = (int) -1899350545;
            CommunicationChannel.instance.send(p128); //===============================
            SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.time_boot_ms = (uint)3275771512U;
            p129.xacc = (short)(short)1553;
            p129.yacc = (short)(short)30718;
            p129.zacc = (short)(short) -20878;
            p129.xgyro = (short)(short) -25505;
            p129.ygyro = (short)(short) -7599;
            p129.zgyro = (short)(short)874;
            p129.xmag = (short)(short)4011;
            p129.ymag = (short)(short)13824;
            p129.zmag = (short)(short) -2528;
            CommunicationChannel.instance.send(p129); //===============================
            DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.type = (byte)(byte)77;
            p130.size = (uint)3601684405U;
            p130.width = (ushort)(ushort)34564;
            p130.height = (ushort)(ushort)59867;
            p130.packets = (ushort)(ushort)44621;
            p130.payload = (byte)(byte)216;
            p130.jpg_quality = (byte)(byte)253;
            CommunicationChannel.instance.send(p130); //===============================
            ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.seqnr = (ushort)(ushort)58045;
            p131.data__SET(new byte[253], 0);
            CommunicationChannel.instance.send(p131); //===============================
            DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.time_boot_ms = (uint)2492226788U;
            p132.min_distance = (ushort)(ushort)44426;
            p132.max_distance = (ushort)(ushort)23863;
            p132.current_distance = (ushort)(ushort)40481;
            p132.type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN;
            p132.id = (byte)(byte)97;
            p132.orientation = MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_90_PITCH_90;
            p132.covariance = (byte)(byte)35;
            CommunicationChannel.instance.send(p132); //===============================
            TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.lat = (int) -721863473;
            p133.lon = (int)723041856;
            p133.grid_spacing = (ushort)(ushort)38418;
            p133.mask = (ulong)3800517229157318410L;
            CommunicationChannel.instance.send(p133); //===============================
            TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.lat = (int) -185864228;
            p134.lon = (int)1508930760;
            p134.grid_spacing = (ushort)(ushort)64455;
            p134.gridbit = (byte)(byte)146;
            p134.data__SET(new short[16], 0);
            CommunicationChannel.instance.send(p134); //===============================
            TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lat = (int)115193568;
            p135.lon = (int) -1240178454;
            CommunicationChannel.instance.send(p135); //===============================
            TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.lat = (int)595498119;
            p136.lon = (int)1196991557;
            p136.spacing = (ushort)(ushort)6476;
            p136.terrain_height = (float) -2.748413E38F;
            p136.current_height = (float) -2.8287862E38F;
            p136.pending = (ushort)(ushort)16249;
            p136.loaded = (ushort)(ushort)45879;
            CommunicationChannel.instance.send(p136); //===============================
            SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.time_boot_ms = (uint)3658376222U;
            p137.press_abs = (float) -3.044312E38F;
            p137.press_diff = (float)8.890816E37F;
            p137.temperature = (short)(short) -7704;
            CommunicationChannel.instance.send(p137); //===============================
            ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.time_usec = (ulong)4104202570444980063L;
            p138.q_SET(new float[4], 0);
            p138.x = (float)9.1682E37F;
            p138.y = (float) -2.4276036E37F;
            p138.z = (float) -2.9574698E38F;
            CommunicationChannel.instance.send(p138); //===============================
            SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.time_usec = (ulong)1771180978589134863L;
            p139.group_mlx = (byte)(byte)104;
            p139.target_system = (byte)(byte)217;
            p139.target_component = (byte)(byte)25;
            p139.controls_SET(new float[8], 0);
            CommunicationChannel.instance.send(p139); //===============================
            ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.time_usec = (ulong)1284489039035884831L;
            p140.group_mlx = (byte)(byte)187;
            p140.controls_SET(new float[8], 0);
            CommunicationChannel.instance.send(p140); //===============================
            ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.time_usec = (ulong)2561060369095783002L;
            p141.altitude_monotonic = (float) -1.742162E38F;
            p141.altitude_amsl = (float) -2.2298136E38F;
            p141.altitude_local = (float)6.864526E37F;
            p141.altitude_relative = (float)1.2839322E38F;
            p141.altitude_terrain = (float)2.4307906E38F;
            p141.bottom_clearance = (float)3.476067E37F;
            CommunicationChannel.instance.send(p141); //===============================
            RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.request_id = (byte)(byte)74;
            p142.uri_type = (byte)(byte)117;
            p142.uri_SET(new byte[120], 0);
            p142.transfer_type = (byte)(byte)244;
            p142.storage_SET(new byte[120], 0);
            CommunicationChannel.instance.send(p142); //===============================
            SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.time_boot_ms = (uint)2149571902U;
            p143.press_abs = (float)3.1961256E38F;
            p143.press_diff = (float) -5.56173E37F;
            p143.temperature = (short)(short) -18767;
            CommunicationChannel.instance.send(p143); //===============================
            FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.timestamp = (ulong)761043703682215664L;
            p144.est_capabilities = (byte)(byte)56;
            p144.lat = (int)841231113;
            p144.lon = (int)2012779122;
            p144.alt = (float)2.1499591E38F;
            p144.vel_SET(new float[3], 0);
            p144.acc_SET(new float[3], 0);
            p144.attitude_q_SET(new float[4], 0);
            p144.rates_SET(new float[3], 0);
            p144.position_cov_SET(new float[3], 0);
            p144.custom_state = (ulong)3357687840616929780L;
            CommunicationChannel.instance.send(p144); //===============================
            CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.time_usec = (ulong)6025309763676078962L;
            p146.x_acc = (float) -3.327264E38F;
            p146.y_acc = (float) -1.0573797E38F;
            p146.z_acc = (float)1.1912205E38F;
            p146.x_vel = (float) -2.812264E38F;
            p146.y_vel = (float) -1.981254E38F;
            p146.z_vel = (float) -1.6364449E38F;
            p146.x_pos = (float)2.1878647E38F;
            p146.y_pos = (float) -1.5909798E38F;
            p146.z_pos = (float) -2.823838E38F;
            p146.airspeed = (float)2.7100382E38F;
            p146.vel_variance_SET(new float[3], 0);
            p146.pos_variance_SET(new float[3], 0);
            p146.q_SET(new float[4], 0);
            p146.roll_rate = (float)3.3148944E38F;
            p146.pitch_rate = (float)1.1628067E38F;
            p146.yaw_rate = (float)2.7960693E38F;
            CommunicationChannel.instance.send(p146); //===============================
            BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.id = (byte)(byte)205;
            p147.battery_function = MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD;
            p147.type = MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIFE;
            p147.temperature = (short)(short)30676;
            p147.voltages_SET(new ushort[10], 0);
            p147.current_battery = (short)(short) -15741;
            p147.current_consumed = (int) -658066472;
            p147.energy_consumed = (int)541510683;
            p147.battery_remaining = (sbyte)(sbyte)85;
            CommunicationChannel.instance.send(p147); //===============================
            AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_TERRAIN |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_INT);
            p148.flight_sw_version = (uint)1586280257U;
            p148.middleware_sw_version = (uint)1458069044U;
            p148.os_sw_version = (uint)1605552916U;
            p148.board_version = (uint)804115271U;
            p148.flight_custom_version_SET(new byte[8], 0);
            p148.middleware_custom_version_SET(new byte[8], 0);
            p148.os_custom_version_SET(new byte[8], 0);
            p148.vendor_id = (ushort)(ushort)56848;
            p148.product_id = (ushort)(ushort)13361;
            p148.uid = (ulong)1683058384216639823L;
            p148.uid2_SET(new byte[18], 0, PH);
            CommunicationChannel.instance.send(p148); //===============================
            LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.time_usec = (ulong)8334338709324041008L;
            p149.target_num = (byte)(byte)34;
            p149.frame = MAV_FRAME.MAV_FRAME_BODY_NED;
            p149.angle_x = (float) -4.958071E37F;
            p149.angle_y = (float)9.769629E37F;
            p149.distance = (float)2.4978723E37F;
            p149.size_x = (float)3.3157217E38F;
            p149.size_y = (float)3.379516E38F;
            p149.x_SET((float)2.041251E38F, PH);
            p149.y_SET((float) -1.7165253E38F, PH);
            p149.z_SET((float)1.6000651E38F, PH);
            p149.q_SET(new float[4], 0, PH);
            p149.type = LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON;
            p149.position_valid_SET((byte)(byte)202, PH);
            CommunicationChannel.instance.send(p149); //===============================
            ARRAY_TEST_0 p150 = CommunicationChannel.new_ARRAY_TEST_0();
            PH.setPack(p150);
            p150.v1 = (byte)(byte)134;
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
            p153.v = (byte)(byte)0;
            p153.ar_u32_SET(new uint[4], 0);
            CommunicationChannel.instance.send(p153); //===============================
            ARRAY_TEST_4 p154 = CommunicationChannel.new_ARRAY_TEST_4();
            PH.setPack(p154);
            p154.ar_u32_SET(new uint[4], 0);
            p154.v = (byte)(byte)191;
            CommunicationChannel.instance.send(p154); //===============================
            ARRAY_TEST_5 p155 = CommunicationChannel.new_ARRAY_TEST_5();
            PH.setPack(p155);
            p155.c1_SET("DEMO", PH);
            p155.c2_SET("DEMO", PH);
            CommunicationChannel.instance.send(p155); //===============================
            ARRAY_TEST_6 p156 = CommunicationChannel.new_ARRAY_TEST_6();
            PH.setPack(p156);
            p156.v1 = (byte)(byte)139;
            p156.v2 = (ushort)(ushort)25809;
            p156.v3 = (uint)3862122163U;
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
            p158.v3 = (uint)2217711281U;
            p158.ar_d_SET(new double[2], 0);
            p158.ar_u16_SET(new ushort[2], 0);
            CommunicationChannel.instance.send(p158); //===============================
            ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.time_usec = (ulong)4270548087067612685L;
            p230.flags = (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ);
            p230.vel_ratio = (float) -2.0577174E38F;
            p230.pos_horiz_ratio = (float) -2.2431637E38F;
            p230.pos_vert_ratio = (float) -1.2717115E38F;
            p230.mag_ratio = (float) -6.793474E37F;
            p230.hagl_ratio = (float) -1.3100805E38F;
            p230.tas_ratio = (float)2.2665706E38F;
            p230.pos_horiz_accuracy = (float) -2.3037665E38F;
            p230.pos_vert_accuracy = (float) -2.234121E38F;
            CommunicationChannel.instance.send(p230); //===============================
            WIND_COV p231 = CommunicationChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.time_usec = (ulong)7118061769832405624L;
            p231.wind_x = (float)2.0736472E38F;
            p231.wind_y = (float) -3.2590703E38F;
            p231.wind_z = (float)7.943016E37F;
            p231.var_horiz = (float)6.359197E37F;
            p231.var_vert = (float)3.380053E37F;
            p231.wind_alt = (float)3.1195788E38F;
            p231.horiz_accuracy = (float)1.5513304E38F;
            p231.vert_accuracy = (float)2.4895913E38F;
            CommunicationChannel.instance.send(p231); //===============================
            GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.time_usec = (ulong)4298271908680693264L;
            p232.gps_id = (byte)(byte)9;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT);
            p232.time_week_ms = (uint)982956909U;
            p232.time_week = (ushort)(ushort)25808;
            p232.fix_type = (byte)(byte)208;
            p232.lat = (int)432925997;
            p232.lon = (int) -1185900857;
            p232.alt = (float) -1.2965167E38F;
            p232.hdop = (float)1.716385E38F;
            p232.vdop = (float) -3.2465006E38F;
            p232.vn = (float)2.9646563E38F;
            p232.ve = (float)2.446925E38F;
            p232.vd = (float) -4.4212403E37F;
            p232.speed_accuracy = (float)2.41618E38F;
            p232.horiz_accuracy = (float)5.708509E37F;
            p232.vert_accuracy = (float) -1.5068684E37F;
            p232.satellites_visible = (byte)(byte)173;
            CommunicationChannel.instance.send(p232); //===============================
            GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.flags = (byte)(byte)220;
            p233.len = (byte)(byte)62;
            p233.data__SET(new byte[180], 0);
            CommunicationChannel.instance.send(p233); //===============================
            HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED);
            p234.custom_mode = (uint)4090995330U;
            p234.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED;
            p234.roll = (short)(short)18089;
            p234.pitch = (short)(short) -27314;
            p234.heading = (ushort)(ushort)37409;
            p234.throttle = (sbyte)(sbyte) - 100;
            p234.heading_sp = (short)(short)10367;
            p234.latitude = (int)515626465;
            p234.longitude = (int)2068613294;
            p234.altitude_amsl = (short)(short)15714;
            p234.altitude_sp = (short)(short)22994;
            p234.airspeed = (byte)(byte)70;
            p234.airspeed_sp = (byte)(byte)84;
            p234.groundspeed = (byte)(byte)219;
            p234.climb_rate = (sbyte)(sbyte)104;
            p234.gps_nsat = (byte)(byte)166;
            p234.gps_fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_PPP;
            p234.battery_remaining = (byte)(byte)129;
            p234.temperature = (sbyte)(sbyte) - 123;
            p234.temperature_air = (sbyte)(sbyte)32;
            p234.failsafe = (byte)(byte)36;
            p234.wp_num = (byte)(byte)67;
            p234.wp_distance = (ushort)(ushort)47013;
            CommunicationChannel.instance.send(p234); //===============================
            VIBRATION p241 = CommunicationChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.time_usec = (ulong)7162222014395805350L;
            p241.vibration_x = (float)3.4653245E37F;
            p241.vibration_y = (float) -1.9746035E38F;
            p241.vibration_z = (float) -2.9127806E38F;
            p241.clipping_0 = (uint)3930204346U;
            p241.clipping_1 = (uint)2463877380U;
            p241.clipping_2 = (uint)4123013906U;
            CommunicationChannel.instance.send(p241); //===============================
            HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.latitude = (int) -1690453668;
            p242.longitude = (int)1883118727;
            p242.altitude = (int) -1930255022;
            p242.x = (float) -8.341865E37F;
            p242.y = (float) -8.4824586E37F;
            p242.z = (float) -6.156208E37F;
            p242.q_SET(new float[4], 0);
            p242.approach_x = (float)2.6270876E38F;
            p242.approach_y = (float) -2.4463797E38F;
            p242.approach_z = (float)9.0563076E36F;
            p242.time_usec_SET((ulong)7171537479760885778L, PH);
            CommunicationChannel.instance.send(p242); //===============================
            SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.target_system = (byte)(byte)56;
            p243.latitude = (int)1349696888;
            p243.longitude = (int)1567802557;
            p243.altitude = (int)1114668639;
            p243.x = (float)2.1291468E38F;
            p243.y = (float) -1.8361824E38F;
            p243.z = (float)5.182012E37F;
            p243.q_SET(new float[4], 0);
            p243.approach_x = (float)2.7588267E38F;
            p243.approach_y = (float)2.5675354E38F;
            p243.approach_z = (float) -2.7985659E38F;
            p243.time_usec_SET((ulong)5082346630863854164L, PH);
            CommunicationChannel.instance.send(p243); //===============================
            MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.message_id = (ushort)(ushort)21745;
            p244.interval_us = (int) -1386716135;
            CommunicationChannel.instance.send(p244); //===============================
            EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.vtol_state = MAV_VTOL_STATE.MAV_VTOL_STATE_UNDEFINED;
            p245.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED;
            CommunicationChannel.instance.send(p245); //===============================
            ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.ICAO_address = (uint)1655649662U;
            p246.lat = (int) -2146952594;
            p246.lon = (int) -1520403286;
            p246.altitude_type = ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
            p246.altitude = (int)2115046739;
            p246.heading = (ushort)(ushort)21945;
            p246.hor_velocity = (ushort)(ushort)4501;
            p246.ver_velocity = (short)(short) -16934;
            p246.callsign_SET("DEMO", PH);
            p246.emitter_type = ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UNASSIGNED;
            p246.tslc = (byte)(byte)186;
            p246.flags = (ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS |
                          ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING |
                          ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK |
                          ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE |
                          ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN);
            p246.squawk = (ushort)(ushort)47387;
            CommunicationChannel.instance.send(p246); //===============================
            COLLISION p247 = CommunicationChannel.new_COLLISION();
            PH.setPack(p247);
            p247.src_ = MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB;
            p247.id = (uint)2197345891U;
            p247.action = MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_REPORT;
            p247.threat_level = (MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH |
                                 MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE);
            p247.time_to_minimum_delta = (float) -1.102649E38F;
            p247.altitude_minimum_delta = (float) -1.5450661E38F;
            p247.horizontal_minimum_delta = (float) -2.3209435E38F;
            CommunicationChannel.instance.send(p247); //===============================
            V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.target_network = (byte)(byte)242;
            p248.target_system = (byte)(byte)123;
            p248.target_component = (byte)(byte)2;
            p248.message_type = (ushort)(ushort)39232;
            p248.payload_SET(new byte[249], 0);
            CommunicationChannel.instance.send(p248); //===============================
            MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.address = (ushort)(ushort)30444;
            p249.ver = (byte)(byte)108;
            p249.type = (byte)(byte)38;
            p249.value_SET(new sbyte[32], 0);
            CommunicationChannel.instance.send(p249); //===============================
            DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.name_SET("DEMO", PH);
            p250.time_usec = (ulong)5530657272296440432L;
            p250.x = (float)2.0357424E38F;
            p250.y = (float) -1.2151942E38F;
            p250.z = (float)1.2705801E38F;
            CommunicationChannel.instance.send(p250); //===============================
            NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.time_boot_ms = (uint)56474609U;
            p251.name_SET("DEMO", PH);
            p251.value = (float)5.3757315E37F;
            CommunicationChannel.instance.send(p251); //===============================
            NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.time_boot_ms = (uint)1315665239U;
            p252.name_SET("DEMO", PH);
            p252.value = (int) -1321512354;
            CommunicationChannel.instance.send(p252); //===============================
            STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.severity = MAV_SEVERITY.MAV_SEVERITY_EMERGENCY;
            p253.text_SET("DEMO", PH);
            CommunicationChannel.instance.send(p253); //===============================
            DEBUG p254 = CommunicationChannel.new_DEBUG();
            PH.setPack(p254);
            p254.time_boot_ms = (uint)2713735950U;
            p254.ind = (byte)(byte)99;
            p254.value = (float) -1.630148E36F;
            CommunicationChannel.instance.send(p254); //===============================
            SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.target_system = (byte)(byte)123;
            p256.target_component = (byte)(byte)95;
            p256.secret_key_SET(new byte[32], 0);
            p256.initial_timestamp = (ulong)8903819434445173110L;
            CommunicationChannel.instance.send(p256); //===============================
            BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.time_boot_ms = (uint)2290270508U;
            p257.last_change_ms = (uint)3335272803U;
            p257.state = (byte)(byte)60;
            CommunicationChannel.instance.send(p257); //===============================
            PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_system = (byte)(byte)212;
            p258.target_component = (byte)(byte)232;
            p258.tune_SET("DEMO", PH);
            CommunicationChannel.instance.send(p258); //===============================
            CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.time_boot_ms = (uint)3440649039U;
            p259.vendor_name_SET(new byte[32], 0);
            p259.model_name_SET(new byte[32], 0);
            p259.firmware_version = (uint)1886093230U;
            p259.focal_length = (float) -1.82448E38F;
            p259.sensor_size_h = (float)3.040681E37F;
            p259.sensor_size_v = (float)3.001993E37F;
            p259.resolution_h = (ushort)(ushort)51471;
            p259.resolution_v = (ushort)(ushort)39903;
            p259.lens_id = (byte)(byte)181;
            p259.flags = (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE |
                          CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
                          CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE);
            p259.cam_definition_version = (ushort)(ushort)22833;
            p259.cam_definition_uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p259); //===============================
            CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.time_boot_ms = (uint)2846983053U;
            p260.mode_id = (CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY);
            CommunicationChannel.instance.send(p260); //===============================
            STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.time_boot_ms = (uint)2991101346U;
            p261.storage_id = (byte)(byte)234;
            p261.storage_count = (byte)(byte)200;
            p261.status = (byte)(byte)194;
            p261.total_capacity = (float)2.45228E38F;
            p261.used_capacity = (float) -2.7014547E38F;
            p261.available_capacity = (float)3.9618044E37F;
            p261.read_speed = (float) -1.2275419E38F;
            p261.write_speed = (float)6.32872E37F;
            CommunicationChannel.instance.send(p261); //===============================
            CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.time_boot_ms = (uint)1290509793U;
            p262.image_status = (byte)(byte)180;
            p262.video_status = (byte)(byte)141;
            p262.image_interval = (float) -1.880158E38F;
            p262.recording_time_ms = (uint)4171484563U;
            p262.available_capacity = (float)4.867357E37F;
            CommunicationChannel.instance.send(p262); //===============================
            CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.time_boot_ms = (uint)2155948588U;
            p263.time_utc = (ulong)4307842361108331951L;
            p263.camera_id = (byte)(byte)9;
            p263.lat = (int)147791533;
            p263.lon = (int)1238796694;
            p263.alt = (int) -554858507;
            p263.relative_alt = (int) -920305623;
            p263.q_SET(new float[4], 0);
            p263.image_index = (int) -546026419;
            p263.capture_result = (sbyte)(sbyte) - 15;
            p263.file_url_SET("DEMO", PH);
            CommunicationChannel.instance.send(p263); //===============================
            FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.time_boot_ms = (uint)1175710073U;
            p264.arming_time_utc = (ulong)5647090742168868049L;
            p264.takeoff_time_utc = (ulong)8026628165954024187L;
            p264.flight_uuid = (ulong)1544629840040944674L;
            CommunicationChannel.instance.send(p264); //===============================
            MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.time_boot_ms = (uint)1518747751U;
            p265.roll = (float) -1.1942274E38F;
            p265.pitch = (float)1.9748108E38F;
            p265.yaw = (float)1.4351653E38F;
            CommunicationChannel.instance.send(p265); //===============================
            LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.target_system = (byte)(byte)244;
            p266.target_component = (byte)(byte)126;
            p266.sequence = (ushort)(ushort)57147;
            p266.length = (byte)(byte)224;
            p266.first_message_offset = (byte)(byte)114;
            p266.data__SET(new byte[249], 0);
            CommunicationChannel.instance.send(p266); //===============================
            LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.target_system = (byte)(byte)95;
            p267.target_component = (byte)(byte)183;
            p267.sequence = (ushort)(ushort)32312;
            p267.length = (byte)(byte)137;
            p267.first_message_offset = (byte)(byte)205;
            p267.data__SET(new byte[249], 0);
            CommunicationChannel.instance.send(p267); //===============================
            LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_system = (byte)(byte)126;
            p268.target_component = (byte)(byte)88;
            p268.sequence = (ushort)(ushort)20615;
            CommunicationChannel.instance.send(p268); //===============================
            VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.camera_id = (byte)(byte)200;
            p269.status = (byte)(byte)101;
            p269.framerate = (float)2.279811E38F;
            p269.resolution_h = (ushort)(ushort)29933;
            p269.resolution_v = (ushort)(ushort)63965;
            p269.bitrate = (uint)3552511469U;
            p269.rotation = (ushort)(ushort)25910;
            p269.uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p269); //===============================
            SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.target_system = (byte)(byte)181;
            p270.target_component = (byte)(byte)21;
            p270.camera_id = (byte)(byte)61;
            p270.framerate = (float)2.885347E38F;
            p270.resolution_h = (ushort)(ushort)25189;
            p270.resolution_v = (ushort)(ushort)50486;
            p270.bitrate = (uint)1622922237U;
            p270.rotation = (ushort)(ushort)44030;
            p270.uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p270); //===============================
            WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.ssid_SET("DEMO", PH);
            p299.password_SET("DEMO", PH);
            CommunicationChannel.instance.send(p299); //===============================
            PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.version = (ushort)(ushort)29520;
            p300.min_version = (ushort)(ushort)44313;
            p300.max_version = (ushort)(ushort)64555;
            p300.spec_version_hash_SET(new byte[8], 0);
            p300.library_version_hash_SET(new byte[8], 0);
            CommunicationChannel.instance.send(p300); //===============================
            UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.time_usec = (ulong)5044028413515528239L;
            p310.uptime_sec = (uint)2666717708U;
            p310.health = UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR;
            p310.mode = UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION;
            p310.sub_mode = (byte)(byte)191;
            p310.vendor_specific_status_code = (ushort)(ushort)5166;
            CommunicationChannel.instance.send(p310); //===============================
            UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.time_usec = (ulong)4167549737358897608L;
            p311.uptime_sec = (uint)1750951778U;
            p311.name_SET("DEMO", PH);
            p311.hw_version_major = (byte)(byte)197;
            p311.hw_version_minor = (byte)(byte)75;
            p311.hw_unique_id_SET(new byte[16], 0);
            p311.sw_version_major = (byte)(byte)27;
            p311.sw_version_minor = (byte)(byte)218;
            p311.sw_vcs_commit = (uint)403753030U;
            CommunicationChannel.instance.send(p311); //===============================
            PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.target_system = (byte)(byte)171;
            p320.target_component = (byte)(byte)208;
            p320.param_id_SET("DEMO", PH);
            p320.param_index = (short)(short) -6074;
            CommunicationChannel.instance.send(p320); //===============================
            PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_system = (byte)(byte)117;
            p321.target_component = (byte)(byte)78;
            CommunicationChannel.instance.send(p321); //===============================
            PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_id_SET("DEMO", PH);
            p322.param_value_SET("DEMO", PH);
            p322.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16;
            p322.param_count = (ushort)(ushort)26841;
            p322.param_index = (ushort)(ushort)9262;
            CommunicationChannel.instance.send(p322); //===============================
            PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.target_system = (byte)(byte)225;
            p323.target_component = (byte)(byte)217;
            p323.param_id_SET("DEMO", PH);
            p323.param_value_SET("DEMO", PH);
            p323.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT64;
            CommunicationChannel.instance.send(p323); //===============================
            PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_id_SET("DEMO", PH);
            p324.param_value_SET("DEMO", PH);
            p324.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32;
            p324.param_result = PARAM_ACK.PARAM_ACK_IN_PROGRESS;
            CommunicationChannel.instance.send(p324); //===============================
            OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.time_usec = (ulong)6436798552729997573L;
            p330.sensor_type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER;
            p330.distances_SET(new ushort[72], 0);
            p330.increment = (byte)(byte)6;
            p330.min_distance = (ushort)(ushort)13357;
            p330.max_distance = (ushort)(ushort)32593;
            CommunicationChannel.instance.send(p330); //===============================
        }
    }
}

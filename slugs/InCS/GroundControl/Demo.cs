
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
            POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.time_boot_ms = (uint)1002266820U;
            p3.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
            p3.type_mask = (ushort)(ushort)20420;
            p3.x = (float) -2.658842E38F;
            p3.y = (float)3.2253528E38F;
            p3.z = (float) -3.17131E38F;
            p3.vx = (float)3.1024986E38F;
            p3.vy = (float)1.8638198E38F;
            p3.vz = (float)6.2026413E37F;
            p3.afx = (float)6.143311E37F;
            p3.afy = (float)1.6038726E38F;
            p3.afz = (float) -1.3251653E38F;
            p3.yaw = (float) -1.0645735E38F;
            p3.yaw_rate = (float)5.788324E37F;
            CommunicationChannel.instance.send(p3); //===============================
            SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.time_boot_ms = (uint)2462654550U;
            p84.target_system = (byte)(byte)255;
            p84.target_component = (byte)(byte)217;
            p84.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
            p84.type_mask = (ushort)(ushort)39190;
            p84.x = (float) -2.2525224E38F;
            p84.y = (float) -1.7006069E38F;
            p84.z = (float)2.2207082E38F;
            p84.vx = (float) -5.9028723E37F;
            p84.vy = (float) -1.8255596E37F;
            p84.vz = (float) -1.4106109E37F;
            p84.afx = (float)1.881517E38F;
            p84.afy = (float)1.6183334E38F;
            p84.afz = (float)3.0457936E38F;
            p84.yaw = (float) -8.1416117E37F;
            p84.yaw_rate = (float)1.2181045E38F;
            CommunicationChannel.instance.send(p84); //===============================
            SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.time_boot_ms = (uint)2070489938U;
            p86.target_system = (byte)(byte)52;
            p86.target_component = (byte)(byte)252;
            p86.coordinate_frame = MAV_FRAME.MAV_FRAME_MISSION;
            p86.type_mask = (ushort)(ushort)24971;
            p86.lat_int = (int) -172952860;
            p86.lon_int = (int)24542088;
            p86.alt = (float) -1.3259156E38F;
            p86.vx = (float) -1.0977335E38F;
            p86.vy = (float) -2.6173594E38F;
            p86.vz = (float)1.1566057E38F;
            p86.afx = (float)7.617569E37F;
            p86.afy = (float) -3.3872048E38F;
            p86.afz = (float)1.7607058E38F;
            p86.yaw = (float)6.9509374E37F;
            p86.yaw_rate = (float)2.06074E38F;
            CommunicationChannel.instance.send(p86); //===============================
            POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.time_boot_ms = (uint)113291824U;
            p87.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT;
            p87.type_mask = (ushort)(ushort)38103;
            p87.lat_int = (int) -2069814495;
            p87.lon_int = (int) -1400744165;
            p87.alt = (float)1.4804007E38F;
            p87.vx = (float) -1.0387767E38F;
            p87.vy = (float) -2.264685E38F;
            p87.vz = (float) -2.2769696E38F;
            p87.afx = (float) -2.2286871E38F;
            p87.afy = (float) -1.0814661E38F;
            p87.afz = (float) -1.4785793E38F;
            p87.yaw = (float)1.1842266E38F;
            p87.yaw_rate = (float) -1.8856637E38F;
            CommunicationChannel.instance.send(p87); //===============================
            LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.time_boot_ms = (uint)2073967188U;
            p89.x = (float) -2.2551078E38F;
            p89.y = (float) -1.4346482E38F;
            p89.z = (float) -2.2762176E38F;
            p89.roll = (float)1.9529652E38F;
            p89.pitch = (float) -1.6794442E38F;
            p89.yaw = (float)1.1111813E38F;
            CommunicationChannel.instance.send(p89); //===============================
            HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.time_usec = (ulong)8212526732670003415L;
            p90.roll = (float)7.9597296E36F;
            p90.pitch = (float)8.554274E37F;
            p90.yaw = (float)1.2348897E38F;
            p90.rollspeed = (float) -2.9886333E38F;
            p90.pitchspeed = (float)1.9719133E38F;
            p90.yawspeed = (float) -3.3416055E38F;
            p90.lat = (int) -1091224977;
            p90.lon = (int) -1920298644;
            p90.alt = (int) -778370672;
            p90.vx = (short)(short)27246;
            p90.vy = (short)(short)13968;
            p90.vz = (short)(short)10423;
            p90.xacc = (short)(short)27079;
            p90.yacc = (short)(short)28356;
            p90.zacc = (short)(short)1058;
            CommunicationChannel.instance.send(p90); //===============================
            HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.time_usec = (ulong)5371831756223159866L;
            p91.roll_ailerons = (float)7.503183E35F;
            p91.pitch_elevator = (float) -5.808382E37F;
            p91.yaw_rudder = (float) -1.7545006E38F;
            p91.throttle = (float) -1.8144984E38F;
            p91.aux1 = (float)2.5273601E37F;
            p91.aux2 = (float) -1.803579E38F;
            p91.aux3 = (float)2.9696678E37F;
            p91.aux4 = (float)3.180679E38F;
            p91.mode = MAV_MODE.MAV_MODE_STABILIZE_ARMED;
            p91.nav_mode = (byte)(byte)27;
            CommunicationChannel.instance.send(p91); //===============================
            HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.time_usec = (ulong)833021007649595001L;
            p92.chan1_raw = (ushort)(ushort)2912;
            p92.chan2_raw = (ushort)(ushort)18657;
            p92.chan3_raw = (ushort)(ushort)39142;
            p92.chan4_raw = (ushort)(ushort)30381;
            p92.chan5_raw = (ushort)(ushort)24393;
            p92.chan6_raw = (ushort)(ushort)7920;
            p92.chan7_raw = (ushort)(ushort)59442;
            p92.chan8_raw = (ushort)(ushort)28778;
            p92.chan9_raw = (ushort)(ushort)34331;
            p92.chan10_raw = (ushort)(ushort)11193;
            p92.chan11_raw = (ushort)(ushort)12348;
            p92.chan12_raw = (ushort)(ushort)60146;
            p92.rssi = (byte)(byte)71;
            CommunicationChannel.instance.send(p92); //===============================
            HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.time_usec = (ulong)4392071138658632403L;
            p93.controls_SET(new float[16], 0);
            p93.mode = MAV_MODE.MAV_MODE_AUTO_ARMED;
            p93.flags = (ulong)4943518713859238572L;
            CommunicationChannel.instance.send(p93); //===============================
            OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.time_usec = (ulong)6785773686157683655L;
            p100.sensor_id = (byte)(byte)161;
            p100.flow_x = (short)(short) -32687;
            p100.flow_y = (short)(short) -1580;
            p100.flow_comp_m_x = (float)2.5873388E38F;
            p100.flow_comp_m_y = (float) -2.636036E38F;
            p100.quality = (byte)(byte)20;
            p100.ground_distance = (float) -5.892293E37F;
            p100.flow_rate_x_SET((float) -3.1538591E38F, PH);
            p100.flow_rate_y_SET((float)2.9067017E38F, PH);
            CommunicationChannel.instance.send(p100); //===============================
            GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.usec = (ulong)522191492431079878L;
            p101.x = (float)2.446406E38F;
            p101.y = (float)1.4610051E38F;
            p101.z = (float) -1.0651E38F;
            p101.roll = (float) -3.2701086E37F;
            p101.pitch = (float)5.404296E37F;
            p101.yaw = (float) -9.455138E37F;
            CommunicationChannel.instance.send(p101); //===============================
            VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.usec = (ulong)6317658789089211103L;
            p102.x = (float) -2.6056542E38F;
            p102.y = (float)2.8811394E38F;
            p102.z = (float)2.5658794E38F;
            p102.roll = (float)3.3995718E38F;
            p102.pitch = (float) -2.7891033E38F;
            p102.yaw = (float)3.3691904E38F;
            CommunicationChannel.instance.send(p102); //===============================
            VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.usec = (ulong)5782201460043922050L;
            p103.x = (float)2.3469E38F;
            p103.y = (float) -7.612487E37F;
            p103.z = (float)9.123184E37F;
            CommunicationChannel.instance.send(p103); //===============================
            VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.usec = (ulong)6788651958785981720L;
            p104.x = (float)2.5648725E38F;
            p104.y = (float)1.3271683E38F;
            p104.z = (float) -2.809023E38F;
            p104.roll = (float)3.0920909E38F;
            p104.pitch = (float) -2.1793763E38F;
            p104.yaw = (float) -2.9468122E38F;
            CommunicationChannel.instance.send(p104); //===============================
            HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.time_usec = (ulong)4954392068944994912L;
            p105.xacc = (float) -1.8275216E38F;
            p105.yacc = (float) -7.955347E37F;
            p105.zacc = (float) -6.4856326E37F;
            p105.xgyro = (float) -3.2174696E38F;
            p105.ygyro = (float)3.0047528E38F;
            p105.zgyro = (float) -1.2938095E38F;
            p105.xmag = (float) -2.7759226E38F;
            p105.ymag = (float)1.7011714E38F;
            p105.zmag = (float)9.939282E37F;
            p105.abs_pressure = (float)1.2975469E38F;
            p105.diff_pressure = (float) -1.1073772E38F;
            p105.pressure_alt = (float) -2.7911593E38F;
            p105.temperature = (float) -2.1481186E37F;
            p105.fields_updated = (ushort)(ushort)62821;
            CommunicationChannel.instance.send(p105); //===============================
            OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.time_usec = (ulong)1106067826919280850L;
            p106.sensor_id = (byte)(byte)41;
            p106.integration_time_us = (uint)2124660911U;
            p106.integrated_x = (float)2.9348877E37F;
            p106.integrated_y = (float)5.811673E36F;
            p106.integrated_xgyro = (float) -1.6554253E38F;
            p106.integrated_ygyro = (float) -1.5314333E38F;
            p106.integrated_zgyro = (float) -3.252297E38F;
            p106.temperature = (short)(short) -20127;
            p106.quality = (byte)(byte)248;
            p106.time_delta_distance_us = (uint)3918092923U;
            p106.distance = (float)3.3960998E38F;
            CommunicationChannel.instance.send(p106); //===============================
            HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.time_usec = (ulong)8960093595748189966L;
            p107.xacc = (float)3.8647865E37F;
            p107.yacc = (float) -2.8487305E38F;
            p107.zacc = (float) -2.3668878E38F;
            p107.xgyro = (float) -2.4101107E38F;
            p107.ygyro = (float) -2.6830014E38F;
            p107.zgyro = (float) -1.7393499E38F;
            p107.xmag = (float) -2.8171423E38F;
            p107.ymag = (float)2.4989397E38F;
            p107.zmag = (float) -2.9438194E38F;
            p107.abs_pressure = (float)1.0999617E37F;
            p107.diff_pressure = (float) -1.7730702E38F;
            p107.pressure_alt = (float) -7.9514337E37F;
            p107.temperature = (float)3.0401389E38F;
            p107.fields_updated = (uint)1199746575U;
            CommunicationChannel.instance.send(p107); //===============================
            SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.q1 = (float) -2.718119E38F;
            p108.q2 = (float)1.7600576E38F;
            p108.q3 = (float)2.8546415E38F;
            p108.q4 = (float)7.699638E37F;
            p108.roll = (float) -2.4428546E38F;
            p108.pitch = (float) -3.5508186E37F;
            p108.yaw = (float)1.697815E38F;
            p108.xacc = (float)2.275098E38F;
            p108.yacc = (float) -9.991409E37F;
            p108.zacc = (float)2.0544667E38F;
            p108.xgyro = (float)2.2329759E38F;
            p108.ygyro = (float) -3.3007954E38F;
            p108.zgyro = (float) -9.698633E37F;
            p108.lat = (float)2.440127E37F;
            p108.lon = (float) -1.2759796E38F;
            p108.alt = (float)8.246302E37F;
            p108.std_dev_horz = (float)2.9059032E37F;
            p108.std_dev_vert = (float)8.89375E37F;
            p108.vn = (float)3.6069076E37F;
            p108.ve = (float)2.7822943E37F;
            p108.vd = (float) -2.198349E38F;
            CommunicationChannel.instance.send(p108); //===============================
            RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.rssi = (byte)(byte)252;
            p109.remrssi = (byte)(byte)4;
            p109.txbuf = (byte)(byte)45;
            p109.noise = (byte)(byte)176;
            p109.remnoise = (byte)(byte)95;
            p109.rxerrors = (ushort)(ushort)45284;
            p109.fixed_ = (ushort)(ushort)38824;
            CommunicationChannel.instance.send(p109); //===============================
            FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_network = (byte)(byte)157;
            p110.target_system = (byte)(byte)194;
            p110.target_component = (byte)(byte)48;
            p110.payload_SET(new byte[251], 0);
            CommunicationChannel.instance.send(p110); //===============================
            TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.tc1 = (long) -6442339154213238422L;
            p111.ts1 = (long) -5150124415790299719L;
            CommunicationChannel.instance.send(p111); //===============================
            CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.time_usec = (ulong)1867619810196034402L;
            p112.seq = (uint)277259308U;
            CommunicationChannel.instance.send(p112); //===============================
            HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.time_usec = (ulong)5905786032858310820L;
            p113.fix_type = (byte)(byte)254;
            p113.lat = (int) -1781491637;
            p113.lon = (int)779753107;
            p113.alt = (int) -643538360;
            p113.eph = (ushort)(ushort)32553;
            p113.epv = (ushort)(ushort)40405;
            p113.vel = (ushort)(ushort)31381;
            p113.vn = (short)(short)5897;
            p113.ve = (short)(short) -19676;
            p113.vd = (short)(short) -20965;
            p113.cog = (ushort)(ushort)45692;
            p113.satellites_visible = (byte)(byte)124;
            CommunicationChannel.instance.send(p113); //===============================
            HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.time_usec = (ulong)8697300314049807589L;
            p114.sensor_id = (byte)(byte)128;
            p114.integration_time_us = (uint)1114896379U;
            p114.integrated_x = (float) -2.0671643E38F;
            p114.integrated_y = (float)1.2761873E38F;
            p114.integrated_xgyro = (float) -3.1377772E38F;
            p114.integrated_ygyro = (float)4.3578517E37F;
            p114.integrated_zgyro = (float)6.3713265E37F;
            p114.temperature = (short)(short)26261;
            p114.quality = (byte)(byte)20;
            p114.time_delta_distance_us = (uint)709597237U;
            p114.distance = (float)1.7345427E37F;
            CommunicationChannel.instance.send(p114); //===============================
            HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.time_usec = (ulong)73666090415316399L;
            p115.attitude_quaternion_SET(new float[4], 0);
            p115.rollspeed = (float) -1.4875414E38F;
            p115.pitchspeed = (float)1.5484898E38F;
            p115.yawspeed = (float) -3.1490577E38F;
            p115.lat = (int)1811163355;
            p115.lon = (int)2123278312;
            p115.alt = (int)138794893;
            p115.vx = (short)(short)8997;
            p115.vy = (short)(short) -893;
            p115.vz = (short)(short)18619;
            p115.ind_airspeed = (ushort)(ushort)8056;
            p115.true_airspeed = (ushort)(ushort)52452;
            p115.xacc = (short)(short) -27831;
            p115.yacc = (short)(short) -13050;
            p115.zacc = (short)(short)15784;
            CommunicationChannel.instance.send(p115); //===============================
            SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.time_boot_ms = (uint)475575425U;
            p116.xacc = (short)(short)20729;
            p116.yacc = (short)(short) -26312;
            p116.zacc = (short)(short)13232;
            p116.xgyro = (short)(short)28262;
            p116.ygyro = (short)(short) -27771;
            p116.zgyro = (short)(short) -16604;
            p116.xmag = (short)(short)32109;
            p116.ymag = (short)(short)28178;
            p116.zmag = (short)(short)4128;
            CommunicationChannel.instance.send(p116); //===============================
            LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.target_system = (byte)(byte)81;
            p117.target_component = (byte)(byte)62;
            p117.start = (ushort)(ushort)44164;
            p117.end = (ushort)(ushort)10657;
            CommunicationChannel.instance.send(p117); //===============================
            LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.id = (ushort)(ushort)30069;
            p118.num_logs = (ushort)(ushort)18661;
            p118.last_log_num = (ushort)(ushort)40237;
            p118.time_utc = (uint)2184497039U;
            p118.size = (uint)1334051771U;
            CommunicationChannel.instance.send(p118); //===============================
            LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.target_system = (byte)(byte)132;
            p119.target_component = (byte)(byte)241;
            p119.id = (ushort)(ushort)10392;
            p119.ofs = (uint)1036500423U;
            p119.count = (uint)486641923U;
            CommunicationChannel.instance.send(p119); //===============================
            LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.id = (ushort)(ushort)17593;
            p120.ofs = (uint)665155993U;
            p120.count = (byte)(byte)109;
            p120.data__SET(new byte[90], 0);
            CommunicationChannel.instance.send(p120); //===============================
            LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_system = (byte)(byte)38;
            p121.target_component = (byte)(byte)249;
            CommunicationChannel.instance.send(p121); //===============================
            LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_system = (byte)(byte)202;
            p122.target_component = (byte)(byte)144;
            CommunicationChannel.instance.send(p122); //===============================
            GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.target_system = (byte)(byte)111;
            p123.target_component = (byte)(byte)127;
            p123.len = (byte)(byte)157;
            p123.data__SET(new byte[110], 0);
            CommunicationChannel.instance.send(p123); //===============================
            GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.time_usec = (ulong)612785135312232866L;
            p124.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX;
            p124.lat = (int) -548120373;
            p124.lon = (int)613827759;
            p124.alt = (int)1567769990;
            p124.eph = (ushort)(ushort)32027;
            p124.epv = (ushort)(ushort)9935;
            p124.vel = (ushort)(ushort)54805;
            p124.cog = (ushort)(ushort)61506;
            p124.satellites_visible = (byte)(byte)255;
            p124.dgps_numch = (byte)(byte)16;
            p124.dgps_age = (uint)2738282902U;
            CommunicationChannel.instance.send(p124); //===============================
            POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vcc = (ushort)(ushort)29084;
            p125.Vservo = (ushort)(ushort)25074;
            p125.flags = (MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED |
                          MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED |
                          MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT |
                          MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID |
                          MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID);
            CommunicationChannel.instance.send(p125); //===============================
            SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.device = SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1;
            p126.flags = (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND |
                          SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY |
                          SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE);
            p126.timeout = (ushort)(ushort)45429;
            p126.baudrate = (uint)1855364380U;
            p126.count = (byte)(byte)29;
            p126.data__SET(new byte[70], 0);
            CommunicationChannel.instance.send(p126); //===============================
            GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.time_last_baseline_ms = (uint)3454381904U;
            p127.rtk_receiver_id = (byte)(byte)216;
            p127.wn = (ushort)(ushort)30378;
            p127.tow = (uint)3619722022U;
            p127.rtk_health = (byte)(byte)215;
            p127.rtk_rate = (byte)(byte)160;
            p127.nsats = (byte)(byte)205;
            p127.baseline_coords_type = (byte)(byte)83;
            p127.baseline_a_mm = (int)1115709380;
            p127.baseline_b_mm = (int)1537880992;
            p127.baseline_c_mm = (int) -2102931135;
            p127.accuracy = (uint)567825215U;
            p127.iar_num_hypotheses = (int) -240817824;
            CommunicationChannel.instance.send(p127); //===============================
            GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.time_last_baseline_ms = (uint)1352919560U;
            p128.rtk_receiver_id = (byte)(byte)194;
            p128.wn = (ushort)(ushort)19481;
            p128.tow = (uint)929345436U;
            p128.rtk_health = (byte)(byte)111;
            p128.rtk_rate = (byte)(byte)53;
            p128.nsats = (byte)(byte)49;
            p128.baseline_coords_type = (byte)(byte)166;
            p128.baseline_a_mm = (int) -328351596;
            p128.baseline_b_mm = (int) -269160718;
            p128.baseline_c_mm = (int)127782153;
            p128.accuracy = (uint)815431436U;
            p128.iar_num_hypotheses = (int) -527096575;
            CommunicationChannel.instance.send(p128); //===============================
            SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.time_boot_ms = (uint)1636254733U;
            p129.xacc = (short)(short) -5397;
            p129.yacc = (short)(short)8276;
            p129.zacc = (short)(short) -26725;
            p129.xgyro = (short)(short)23813;
            p129.ygyro = (short)(short) -21077;
            p129.zgyro = (short)(short)2431;
            p129.xmag = (short)(short) -9967;
            p129.ymag = (short)(short) -14718;
            p129.zmag = (short)(short)3475;
            CommunicationChannel.instance.send(p129); //===============================
            DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.type = (byte)(byte)93;
            p130.size = (uint)1393553783U;
            p130.width = (ushort)(ushort)62525;
            p130.height = (ushort)(ushort)65114;
            p130.packets = (ushort)(ushort)10716;
            p130.payload = (byte)(byte)43;
            p130.jpg_quality = (byte)(byte)244;
            CommunicationChannel.instance.send(p130); //===============================
            ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.seqnr = (ushort)(ushort)42709;
            p131.data__SET(new byte[253], 0);
            CommunicationChannel.instance.send(p131); //===============================
            DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.time_boot_ms = (uint)4218383338U;
            p132.min_distance = (ushort)(ushort)39734;
            p132.max_distance = (ushort)(ushort)50180;
            p132.current_distance = (ushort)(ushort)37818;
            p132.type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN;
            p132.id = (byte)(byte)170;
            p132.orientation = MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_135;
            p132.covariance = (byte)(byte)181;
            CommunicationChannel.instance.send(p132); //===============================
            TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.lat = (int) -1153570571;
            p133.lon = (int)1132292877;
            p133.grid_spacing = (ushort)(ushort)52477;
            p133.mask = (ulong)1711051378929879790L;
            CommunicationChannel.instance.send(p133); //===============================
            TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.lat = (int) -1353224718;
            p134.lon = (int)630433097;
            p134.grid_spacing = (ushort)(ushort)12276;
            p134.gridbit = (byte)(byte)215;
            p134.data__SET(new short[16], 0);
            CommunicationChannel.instance.send(p134); //===============================
            TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lat = (int) -77025638;
            p135.lon = (int) -1936147106;
            CommunicationChannel.instance.send(p135); //===============================
            TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.lat = (int)983921122;
            p136.lon = (int) -40712515;
            p136.spacing = (ushort)(ushort)39411;
            p136.terrain_height = (float) -7.581845E37F;
            p136.current_height = (float) -3.1469793E38F;
            p136.pending = (ushort)(ushort)31901;
            p136.loaded = (ushort)(ushort)33905;
            CommunicationChannel.instance.send(p136); //===============================
            SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.time_boot_ms = (uint)1743003814U;
            p137.press_abs = (float) -2.2516193E37F;
            p137.press_diff = (float) -1.5258011E38F;
            p137.temperature = (short)(short) -31173;
            CommunicationChannel.instance.send(p137); //===============================
            ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.time_usec = (ulong)4660843563385679051L;
            p138.q_SET(new float[4], 0);
            p138.x = (float)1.3488556E38F;
            p138.y = (float)1.8617447E38F;
            p138.z = (float)2.6180321E38F;
            CommunicationChannel.instance.send(p138); //===============================
            SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.time_usec = (ulong)6846788003903850964L;
            p139.group_mlx = (byte)(byte)89;
            p139.target_system = (byte)(byte)158;
            p139.target_component = (byte)(byte)26;
            p139.controls_SET(new float[8], 0);
            CommunicationChannel.instance.send(p139); //===============================
            ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.time_usec = (ulong)237746748741434898L;
            p140.group_mlx = (byte)(byte)3;
            p140.controls_SET(new float[8], 0);
            CommunicationChannel.instance.send(p140); //===============================
            ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.time_usec = (ulong)8224157612773319879L;
            p141.altitude_monotonic = (float)1.640217E38F;
            p141.altitude_amsl = (float) -2.6819655E38F;
            p141.altitude_local = (float)3.0290836E38F;
            p141.altitude_relative = (float)1.738511E38F;
            p141.altitude_terrain = (float) -1.5128542E37F;
            p141.bottom_clearance = (float)2.6203952E38F;
            CommunicationChannel.instance.send(p141); //===============================
            RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.request_id = (byte)(byte)197;
            p142.uri_type = (byte)(byte)162;
            p142.uri_SET(new byte[120], 0);
            p142.transfer_type = (byte)(byte)8;
            p142.storage_SET(new byte[120], 0);
            CommunicationChannel.instance.send(p142); //===============================
            SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.time_boot_ms = (uint)2425798106U;
            p143.press_abs = (float)8.4396404E37F;
            p143.press_diff = (float) -1.2398981E38F;
            p143.temperature = (short)(short) -31731;
            CommunicationChannel.instance.send(p143); //===============================
            FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.timestamp = (ulong)469373745769430955L;
            p144.est_capabilities = (byte)(byte)213;
            p144.lat = (int)1982481569;
            p144.lon = (int)753648007;
            p144.alt = (float)1.9265936E38F;
            p144.vel_SET(new float[3], 0);
            p144.acc_SET(new float[3], 0);
            p144.attitude_q_SET(new float[4], 0);
            p144.rates_SET(new float[3], 0);
            p144.position_cov_SET(new float[3], 0);
            p144.custom_state = (ulong)5629216115786702003L;
            CommunicationChannel.instance.send(p144); //===============================
            CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.time_usec = (ulong)6974072166056497457L;
            p146.x_acc = (float) -4.105085E37F;
            p146.y_acc = (float)2.898873E37F;
            p146.z_acc = (float) -1.1864812E38F;
            p146.x_vel = (float) -2.9694024E38F;
            p146.y_vel = (float) -6.9162565E37F;
            p146.z_vel = (float)1.925225E38F;
            p146.x_pos = (float) -1.6638303E38F;
            p146.y_pos = (float) -3.6713491E37F;
            p146.z_pos = (float) -5.588138E37F;
            p146.airspeed = (float) -2.9994708E38F;
            p146.vel_variance_SET(new float[3], 0);
            p146.pos_variance_SET(new float[3], 0);
            p146.q_SET(new float[4], 0);
            p146.roll_rate = (float) -1.0671273E38F;
            p146.pitch_rate = (float) -1.8336345E38F;
            p146.yaw_rate = (float) -8.362989E37F;
            CommunicationChannel.instance.send(p146); //===============================
            BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.id = (byte)(byte)200;
            p147.battery_function = MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_AVIONICS;
            p147.type = MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_NIMH;
            p147.temperature = (short)(short)17909;
            p147.voltages_SET(new ushort[10], 0);
            p147.current_battery = (short)(short) -17416;
            p147.current_consumed = (int) -1547648698;
            p147.energy_consumed = (int)158510486;
            p147.battery_remaining = (sbyte)(sbyte) - 77;
            CommunicationChannel.instance.send(p147); //===============================
            AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT);
            p148.flight_sw_version = (uint)876094565U;
            p148.middleware_sw_version = (uint)3252684336U;
            p148.os_sw_version = (uint)1542068040U;
            p148.board_version = (uint)1926902415U;
            p148.flight_custom_version_SET(new byte[8], 0);
            p148.middleware_custom_version_SET(new byte[8], 0);
            p148.os_custom_version_SET(new byte[8], 0);
            p148.vendor_id = (ushort)(ushort)12246;
            p148.product_id = (ushort)(ushort)32489;
            p148.uid = (ulong)1359528440872427244L;
            p148.uid2_SET(new byte[18], 0, PH);
            CommunicationChannel.instance.send(p148); //===============================
            LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.time_usec = (ulong)3347651030934649234L;
            p149.target_num = (byte)(byte)207;
            p149.frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
            p149.angle_x = (float) -1.3538985E38F;
            p149.angle_y = (float)3.005123E38F;
            p149.distance = (float) -9.618594E37F;
            p149.size_x = (float)3.3775518E38F;
            p149.size_y = (float)3.3737547E38F;
            p149.x_SET((float)1.2705601E38F, PH);
            p149.y_SET((float) -1.4941776E38F, PH);
            p149.z_SET((float)2.9133702E38F, PH);
            p149.q_SET(new float[4], 0, PH);
            p149.type = LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_FIDUCIAL;
            p149.position_valid_SET((byte)(byte)204, PH);
            CommunicationChannel.instance.send(p149); //===============================
            CPU_LOAD p170 = CommunicationChannel.new_CPU_LOAD();
            PH.setPack(p170);
            p170.sensLoad = (byte)(byte)211;
            p170.ctrlLoad = (byte)(byte)115;
            p170.batVolt = (ushort)(ushort)54870;
            CommunicationChannel.instance.send(p170); //===============================
            SENSOR_BIAS p172 = CommunicationChannel.new_SENSOR_BIAS();
            PH.setPack(p172);
            p172.axBias = (float)1.5938926E38F;
            p172.ayBias = (float) -2.1704048E38F;
            p172.azBias = (float) -8.602042E37F;
            p172.gxBias = (float)2.675816E38F;
            p172.gyBias = (float) -2.2665157E38F;
            p172.gzBias = (float)3.3289519E38F;
            CommunicationChannel.instance.send(p172); //===============================
            DIAGNOSTIC p173 = CommunicationChannel.new_DIAGNOSTIC();
            PH.setPack(p173);
            p173.diagFl1 = (float) -2.1446263E38F;
            p173.diagFl2 = (float) -2.7358522E38F;
            p173.diagFl3 = (float) -1.0156998E38F;
            p173.diagSh1 = (short)(short)21348;
            p173.diagSh2 = (short)(short) -2832;
            p173.diagSh3 = (short)(short)21154;
            CommunicationChannel.instance.send(p173); //===============================
            SLUGS_NAVIGATION p176 = CommunicationChannel.new_SLUGS_NAVIGATION();
            PH.setPack(p176);
            p176.u_m = (float) -2.3105794E38F;
            p176.phi_c = (float) -9.45136E37F;
            p176.theta_c = (float)1.4863983E38F;
            p176.psiDot_c = (float)1.9465408E38F;
            p176.ay_body = (float) -2.8667906E38F;
            p176.totalDist = (float) -9.483235E37F;
            p176.dist2Go = (float) -1.1328554E38F;
            p176.fromWP = (byte)(byte)122;
            p176.toWP = (byte)(byte)45;
            p176.h_c = (ushort)(ushort)44380;
            CommunicationChannel.instance.send(p176); //===============================
            DATA_LOG p177 = CommunicationChannel.new_DATA_LOG();
            PH.setPack(p177);
            p177.fl_1 = (float)9.266645E37F;
            p177.fl_2 = (float) -2.5538128E38F;
            p177.fl_3 = (float)1.314829E38F;
            p177.fl_4 = (float)2.1089885E38F;
            p177.fl_5 = (float)2.101404E38F;
            p177.fl_6 = (float)1.8746015E37F;
            CommunicationChannel.instance.send(p177); //===============================
            GPS_DATE_TIME p179 = CommunicationChannel.new_GPS_DATE_TIME();
            PH.setPack(p179);
            p179.year = (byte)(byte)16;
            p179.month = (byte)(byte)225;
            p179.day = (byte)(byte)63;
            p179.hour = (byte)(byte)176;
            p179.min = (byte)(byte)235;
            p179.sec = (byte)(byte)159;
            p179.clockStat = (byte)(byte)251;
            p179.visSat = (byte)(byte)175;
            p179.useSat = (byte)(byte)171;
            p179.GppGl = (byte)(byte)216;
            p179.sigUsedMask = (byte)(byte)188;
            p179.percentUsed = (byte)(byte)149;
            CommunicationChannel.instance.send(p179); //===============================
            MID_LVL_CMDS p180 = CommunicationChannel.new_MID_LVL_CMDS();
            PH.setPack(p180);
            p180.target = (byte)(byte)250;
            p180.hCommand = (float)2.9822069E38F;
            p180.uCommand = (float)5.241904E37F;
            p180.rCommand = (float)5.9931366E37F;
            CommunicationChannel.instance.send(p180); //===============================
            CTRL_SRFC_PT p181 = CommunicationChannel.new_CTRL_SRFC_PT();
            PH.setPack(p181);
            p181.target = (byte)(byte)101;
            p181.bitfieldPt = (ushort)(ushort)30110;
            CommunicationChannel.instance.send(p181); //===============================
            SLUGS_CAMERA_ORDER p184 = CommunicationChannel.new_SLUGS_CAMERA_ORDER();
            PH.setPack(p184);
            p184.target = (byte)(byte)235;
            p184.pan = (sbyte)(sbyte)89;
            p184.tilt = (sbyte)(sbyte)50;
            p184.zoom = (sbyte)(sbyte) - 24;
            p184.moveHome = (sbyte)(sbyte)53;
            CommunicationChannel.instance.send(p184); //===============================
            CONTROL_SURFACE p185 = CommunicationChannel.new_CONTROL_SURFACE();
            PH.setPack(p185);
            p185.target = (byte)(byte)214;
            p185.idSurface = (byte)(byte)18;
            p185.mControl = (float) -2.8268925E38F;
            p185.bControl = (float)2.8724963E38F;
            CommunicationChannel.instance.send(p185); //===============================
            SLUGS_MOBILE_LOCATION p186 = CommunicationChannel.new_SLUGS_MOBILE_LOCATION();
            PH.setPack(p186);
            p186.target = (byte)(byte)108;
            p186.latitude = (float) -1.9253353E38F;
            p186.longitude = (float) -3.254238E38F;
            CommunicationChannel.instance.send(p186); //===============================
            SLUGS_CONFIGURATION_CAMERA p188 = CommunicationChannel.new_SLUGS_CONFIGURATION_CAMERA();
            PH.setPack(p188);
            p188.target = (byte)(byte)145;
            p188.idOrder = (byte)(byte)174;
            p188.order = (byte)(byte)149;
            CommunicationChannel.instance.send(p188); //===============================
            ISR_LOCATION p189 = CommunicationChannel.new_ISR_LOCATION();
            PH.setPack(p189);
            p189.target = (byte)(byte)145;
            p189.latitude = (float)3.3080975E38F;
            p189.longitude = (float)3.144047E38F;
            p189.height = (float) -1.1871383E38F;
            p189.option1 = (byte)(byte)236;
            p189.option2 = (byte)(byte)54;
            p189.option3 = (byte)(byte)93;
            CommunicationChannel.instance.send(p189); //===============================
            VOLT_SENSOR p191 = CommunicationChannel.new_VOLT_SENSOR();
            PH.setPack(p191);
            p191.r2Type = (byte)(byte)97;
            p191.voltage = (ushort)(ushort)53396;
            p191.reading2 = (ushort)(ushort)30742;
            CommunicationChannel.instance.send(p191); //===============================
            PTZ_STATUS p192 = CommunicationChannel.new_PTZ_STATUS();
            PH.setPack(p192);
            p192.zoom = (byte)(byte)19;
            p192.pan = (short)(short) -22907;
            p192.tilt = (short)(short) -24557;
            CommunicationChannel.instance.send(p192); //===============================
            UAV_STATUS p193 = CommunicationChannel.new_UAV_STATUS();
            PH.setPack(p193);
            p193.target = (byte)(byte)24;
            p193.latitude = (float)1.3719633E38F;
            p193.longitude = (float)1.133218E37F;
            p193.altitude = (float)2.982095E38F;
            p193.speed = (float)4.398021E37F;
            p193.course = (float)2.2997456E36F;
            CommunicationChannel.instance.send(p193); //===============================
            STATUS_GPS p194 = CommunicationChannel.new_STATUS_GPS();
            PH.setPack(p194);
            p194.csFails = (ushort)(ushort)39714;
            p194.gpsQuality = (byte)(byte)167;
            p194.msgsType = (byte)(byte)177;
            p194.posStatus = (byte)(byte)6;
            p194.magVar = (float) -4.134168E37F;
            p194.magDir = (sbyte)(sbyte) - 12;
            p194.modeInd = (byte)(byte)133;
            CommunicationChannel.instance.send(p194); //===============================
            NOVATEL_DIAG p195 = CommunicationChannel.new_NOVATEL_DIAG();
            PH.setPack(p195);
            p195.timeStatus = (byte)(byte)28;
            p195.receiverStatus = (uint)4252033516U;
            p195.solStatus = (byte)(byte)229;
            p195.posType = (byte)(byte)91;
            p195.velType = (byte)(byte)232;
            p195.posSolAge = (float) -1.4076632E38F;
            p195.csFails = (ushort)(ushort)41403;
            CommunicationChannel.instance.send(p195); //===============================
            SENSOR_DIAG p196 = CommunicationChannel.new_SENSOR_DIAG();
            PH.setPack(p196);
            p196.float1 = (float) -3.394399E38F;
            p196.float2 = (float) -9.010522E37F;
            p196.int1 = (short)(short) -24140;
            p196.char1 = (sbyte)(sbyte) - 99;
            CommunicationChannel.instance.send(p196); //===============================
            BOOT p197 = CommunicationChannel.new_BOOT();
            PH.setPack(p197);
            p197.version = (uint)6813944U;
            CommunicationChannel.instance.send(p197); //===============================
            ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.time_usec = (ulong)3959641761811645468L;
            p230.flags = (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT);
            p230.vel_ratio = (float) -2.3957323E38F;
            p230.pos_horiz_ratio = (float)2.3267481E38F;
            p230.pos_vert_ratio = (float)3.0502063E38F;
            p230.mag_ratio = (float) -1.4128244E38F;
            p230.hagl_ratio = (float) -2.0762758E38F;
            p230.tas_ratio = (float) -1.3081428E38F;
            p230.pos_horiz_accuracy = (float)1.4097885E38F;
            p230.pos_vert_accuracy = (float) -1.6088432E38F;
            CommunicationChannel.instance.send(p230); //===============================
            WIND_COV p231 = CommunicationChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.time_usec = (ulong)6976096736325295186L;
            p231.wind_x = (float) -1.9352855E38F;
            p231.wind_y = (float)2.9268835E38F;
            p231.wind_z = (float)1.8631821E38F;
            p231.var_horiz = (float)1.6792479E38F;
            p231.var_vert = (float) -2.5864784E38F;
            p231.wind_alt = (float)7.0178927E37F;
            p231.horiz_accuracy = (float)1.2649907E38F;
            p231.vert_accuracy = (float)7.653171E37F;
            CommunicationChannel.instance.send(p231); //===============================
            GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.time_usec = (ulong)2678900155663942533L;
            p232.gps_id = (byte)(byte)2;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY);
            p232.time_week_ms = (uint)59931126U;
            p232.time_week = (ushort)(ushort)24023;
            p232.fix_type = (byte)(byte)129;
            p232.lat = (int)588370428;
            p232.lon = (int)728756336;
            p232.alt = (float) -2.4532183E38F;
            p232.hdop = (float)3.8599525E37F;
            p232.vdop = (float) -2.2866107E38F;
            p232.vn = (float)1.4964759E38F;
            p232.ve = (float) -4.0336447E37F;
            p232.vd = (float)1.1932682E38F;
            p232.speed_accuracy = (float)2.5833352E38F;
            p232.horiz_accuracy = (float)2.8940492E38F;
            p232.vert_accuracy = (float) -1.0299146E38F;
            p232.satellites_visible = (byte)(byte)252;
            CommunicationChannel.instance.send(p232); //===============================
            GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.flags = (byte)(byte)156;
            p233.len = (byte)(byte)170;
            p233.data__SET(new byte[180], 0);
            CommunicationChannel.instance.send(p233); //===============================
            HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED);
            p234.custom_mode = (uint)3682635096U;
            p234.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR;
            p234.roll = (short)(short)20701;
            p234.pitch = (short)(short) -16838;
            p234.heading = (ushort)(ushort)12973;
            p234.throttle = (sbyte)(sbyte)113;
            p234.heading_sp = (short)(short) -25053;
            p234.latitude = (int) -289029692;
            p234.longitude = (int) -1524631678;
            p234.altitude_amsl = (short)(short)10495;
            p234.altitude_sp = (short)(short)13005;
            p234.airspeed = (byte)(byte)233;
            p234.airspeed_sp = (byte)(byte)188;
            p234.groundspeed = (byte)(byte)255;
            p234.climb_rate = (sbyte)(sbyte) - 76;
            p234.gps_nsat = (byte)(byte)252;
            p234.gps_fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT;
            p234.battery_remaining = (byte)(byte)15;
            p234.temperature = (sbyte)(sbyte) - 93;
            p234.temperature_air = (sbyte)(sbyte)82;
            p234.failsafe = (byte)(byte)246;
            p234.wp_num = (byte)(byte)117;
            p234.wp_distance = (ushort)(ushort)36129;
            CommunicationChannel.instance.send(p234); //===============================
            VIBRATION p241 = CommunicationChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.time_usec = (ulong)1974567294918332838L;
            p241.vibration_x = (float) -1.2535258E38F;
            p241.vibration_y = (float) -3.445665E37F;
            p241.vibration_z = (float) -1.7065129E38F;
            p241.clipping_0 = (uint)958551132U;
            p241.clipping_1 = (uint)1002731840U;
            p241.clipping_2 = (uint)1684064980U;
            CommunicationChannel.instance.send(p241); //===============================
            HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.latitude = (int) -2141540008;
            p242.longitude = (int) -1581771634;
            p242.altitude = (int)263927435;
            p242.x = (float) -1.5913236E38F;
            p242.y = (float) -2.730049E38F;
            p242.z = (float)1.82811E38F;
            p242.q_SET(new float[4], 0);
            p242.approach_x = (float) -1.5960188E38F;
            p242.approach_y = (float) -2.058456E38F;
            p242.approach_z = (float)1.6247732E38F;
            p242.time_usec_SET((ulong)2397211898669945912L, PH);
            CommunicationChannel.instance.send(p242); //===============================
            SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.target_system = (byte)(byte)64;
            p243.latitude = (int)1462795513;
            p243.longitude = (int)1993900089;
            p243.altitude = (int)914997206;
            p243.x = (float) -3.1577982E38F;
            p243.y = (float)1.7348764E38F;
            p243.z = (float)1.9671013E38F;
            p243.q_SET(new float[4], 0);
            p243.approach_x = (float)1.3250197E38F;
            p243.approach_y = (float)2.3351327E38F;
            p243.approach_z = (float)6.3114807E37F;
            p243.time_usec_SET((ulong)6963248491242778624L, PH);
            CommunicationChannel.instance.send(p243); //===============================
            MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.message_id = (ushort)(ushort)64387;
            p244.interval_us = (int) -990383378;
            CommunicationChannel.instance.send(p244); //===============================
            EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.vtol_state = MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_FW;
            p245.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR;
            CommunicationChannel.instance.send(p245); //===============================
            ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.ICAO_address = (uint)1118650977U;
            p246.lat = (int)512594936;
            p246.lon = (int) -815303808;
            p246.altitude_type = ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
            p246.altitude = (int) -1865190102;
            p246.heading = (ushort)(ushort)4229;
            p246.hor_velocity = (ushort)(ushort)42502;
            p246.ver_velocity = (short)(short)27442;
            p246.callsign_SET("DEMO", PH);
            p246.emitter_type = ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UAV;
            p246.tslc = (byte)(byte)4;
            p246.flags = (ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING |
                          ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY);
            p246.squawk = (ushort)(ushort)65109;
            CommunicationChannel.instance.send(p246); //===============================
            COLLISION p247 = CommunicationChannel.new_COLLISION();
            PH.setPack(p247);
            p247.src_ = MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB;
            p247.id = (uint)1030701746U;
            p247.action = MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_PERPENDICULAR;
            p247.threat_level = MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW;
            p247.time_to_minimum_delta = (float) -1.6936577E38F;
            p247.altitude_minimum_delta = (float) -3.0512176E38F;
            p247.horizontal_minimum_delta = (float) -1.4320821E38F;
            CommunicationChannel.instance.send(p247); //===============================
            V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.target_network = (byte)(byte)93;
            p248.target_system = (byte)(byte)64;
            p248.target_component = (byte)(byte)14;
            p248.message_type = (ushort)(ushort)16243;
            p248.payload_SET(new byte[249], 0);
            CommunicationChannel.instance.send(p248); //===============================
            MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.address = (ushort)(ushort)23525;
            p249.ver = (byte)(byte)217;
            p249.type = (byte)(byte)34;
            p249.value_SET(new sbyte[32], 0);
            CommunicationChannel.instance.send(p249); //===============================
            DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.name_SET("DEMO", PH);
            p250.time_usec = (ulong)5473324469270809983L;
            p250.x = (float)3.2172445E37F;
            p250.y = (float) -2.895077E38F;
            p250.z = (float) -2.6341633E38F;
            CommunicationChannel.instance.send(p250); //===============================
            NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.time_boot_ms = (uint)3639020920U;
            p251.name_SET("DEMO", PH);
            p251.value = (float) -2.9190748E38F;
            CommunicationChannel.instance.send(p251); //===============================
            NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.time_boot_ms = (uint)892720017U;
            p252.name_SET("DEMO", PH);
            p252.value = (int)1919108665;
            CommunicationChannel.instance.send(p252); //===============================
            STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.severity = MAV_SEVERITY.MAV_SEVERITY_INFO;
            p253.text_SET("DEMO", PH);
            CommunicationChannel.instance.send(p253); //===============================
            DEBUG p254 = CommunicationChannel.new_DEBUG();
            PH.setPack(p254);
            p254.time_boot_ms = (uint)2391815813U;
            p254.ind = (byte)(byte)66;
            p254.value = (float)2.2392233E38F;
            CommunicationChannel.instance.send(p254); //===============================
            SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.target_system = (byte)(byte)229;
            p256.target_component = (byte)(byte)239;
            p256.secret_key_SET(new byte[32], 0);
            p256.initial_timestamp = (ulong)9121346812406861758L;
            CommunicationChannel.instance.send(p256); //===============================
            BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.time_boot_ms = (uint)3205473021U;
            p257.last_change_ms = (uint)2064845455U;
            p257.state = (byte)(byte)201;
            CommunicationChannel.instance.send(p257); //===============================
            PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_system = (byte)(byte)221;
            p258.target_component = (byte)(byte)156;
            p258.tune_SET("DEMO", PH);
            CommunicationChannel.instance.send(p258); //===============================
            CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.time_boot_ms = (uint)2859577870U;
            p259.vendor_name_SET(new byte[32], 0);
            p259.model_name_SET(new byte[32], 0);
            p259.firmware_version = (uint)1048807479U;
            p259.focal_length = (float) -3.03584E38F;
            p259.sensor_size_h = (float)3.2514423E38F;
            p259.sensor_size_v = (float)2.8518885E38F;
            p259.resolution_h = (ushort)(ushort)16876;
            p259.resolution_v = (ushort)(ushort)49995;
            p259.lens_id = (byte)(byte)160;
            p259.flags = (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE |
                          CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE);
            p259.cam_definition_version = (ushort)(ushort)52902;
            p259.cam_definition_uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p259); //===============================
            CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.time_boot_ms = (uint)3290040560U;
            p260.mode_id = CAMERA_MODE.CAMERA_MODE_VIDEO;
            CommunicationChannel.instance.send(p260); //===============================
            STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.time_boot_ms = (uint)1819711781U;
            p261.storage_id = (byte)(byte)224;
            p261.storage_count = (byte)(byte)244;
            p261.status = (byte)(byte)225;
            p261.total_capacity = (float)1.0315785E38F;
            p261.used_capacity = (float) -9.200259E37F;
            p261.available_capacity = (float) -3.3283085E38F;
            p261.read_speed = (float)2.9132383E38F;
            p261.write_speed = (float)8.113429E37F;
            CommunicationChannel.instance.send(p261); //===============================
            CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.time_boot_ms = (uint)3147324750U;
            p262.image_status = (byte)(byte)34;
            p262.video_status = (byte)(byte)254;
            p262.image_interval = (float) -1.741149E38F;
            p262.recording_time_ms = (uint)3094492939U;
            p262.available_capacity = (float) -3.1489522E38F;
            CommunicationChannel.instance.send(p262); //===============================
            CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.time_boot_ms = (uint)2046120347U;
            p263.time_utc = (ulong)2325534867923414735L;
            p263.camera_id = (byte)(byte)161;
            p263.lat = (int) -757831752;
            p263.lon = (int)1190350404;
            p263.alt = (int) -1932197584;
            p263.relative_alt = (int)1956050383;
            p263.q_SET(new float[4], 0);
            p263.image_index = (int) -1504539173;
            p263.capture_result = (sbyte)(sbyte)49;
            p263.file_url_SET("DEMO", PH);
            CommunicationChannel.instance.send(p263); //===============================
            FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.time_boot_ms = (uint)2621965139U;
            p264.arming_time_utc = (ulong)1038553047052291397L;
            p264.takeoff_time_utc = (ulong)6551339630146674488L;
            p264.flight_uuid = (ulong)6802917054686427028L;
            CommunicationChannel.instance.send(p264); //===============================
            MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.time_boot_ms = (uint)96680083U;
            p265.roll = (float)1.2004651E38F;
            p265.pitch = (float) -1.621376E38F;
            p265.yaw = (float)6.3980704E37F;
            CommunicationChannel.instance.send(p265); //===============================
            LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.target_system = (byte)(byte)11;
            p266.target_component = (byte)(byte)22;
            p266.sequence = (ushort)(ushort)48435;
            p266.length = (byte)(byte)160;
            p266.first_message_offset = (byte)(byte)237;
            p266.data__SET(new byte[249], 0);
            CommunicationChannel.instance.send(p266); //===============================
            LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.target_system = (byte)(byte)115;
            p267.target_component = (byte)(byte)67;
            p267.sequence = (ushort)(ushort)450;
            p267.length = (byte)(byte)154;
            p267.first_message_offset = (byte)(byte)31;
            p267.data__SET(new byte[249], 0);
            CommunicationChannel.instance.send(p267); //===============================
            LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_system = (byte)(byte)148;
            p268.target_component = (byte)(byte)54;
            p268.sequence = (ushort)(ushort)58758;
            CommunicationChannel.instance.send(p268); //===============================
            VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.camera_id = (byte)(byte)103;
            p269.status = (byte)(byte)190;
            p269.framerate = (float) -5.8928077E35F;
            p269.resolution_h = (ushort)(ushort)54337;
            p269.resolution_v = (ushort)(ushort)10249;
            p269.bitrate = (uint)858500617U;
            p269.rotation = (ushort)(ushort)25352;
            p269.uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p269); //===============================
            SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.target_system = (byte)(byte)210;
            p270.target_component = (byte)(byte)169;
            p270.camera_id = (byte)(byte)225;
            p270.framerate = (float) -2.24174E38F;
            p270.resolution_h = (ushort)(ushort)42249;
            p270.resolution_v = (ushort)(ushort)20166;
            p270.bitrate = (uint)3161767362U;
            p270.rotation = (ushort)(ushort)4446;
            p270.uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p270); //===============================
            WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.ssid_SET("DEMO", PH);
            p299.password_SET("DEMO", PH);
            CommunicationChannel.instance.send(p299); //===============================
            PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.version = (ushort)(ushort)49398;
            p300.min_version = (ushort)(ushort)64711;
            p300.max_version = (ushort)(ushort)3638;
            p300.spec_version_hash_SET(new byte[8], 0);
            p300.library_version_hash_SET(new byte[8], 0);
            CommunicationChannel.instance.send(p300); //===============================
            UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.time_usec = (ulong)562705274713845493L;
            p310.uptime_sec = (uint)2803094163U;
            p310.health = UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL;
            p310.mode = UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE;
            p310.sub_mode = (byte)(byte)228;
            p310.vendor_specific_status_code = (ushort)(ushort)20273;
            CommunicationChannel.instance.send(p310); //===============================
            UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.time_usec = (ulong)499541340371313336L;
            p311.uptime_sec = (uint)2031077147U;
            p311.name_SET("DEMO", PH);
            p311.hw_version_major = (byte)(byte)139;
            p311.hw_version_minor = (byte)(byte)198;
            p311.hw_unique_id_SET(new byte[16], 0);
            p311.sw_version_major = (byte)(byte)245;
            p311.sw_version_minor = (byte)(byte)108;
            p311.sw_vcs_commit = (uint)1704029395U;
            CommunicationChannel.instance.send(p311); //===============================
            PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.target_system = (byte)(byte)114;
            p320.target_component = (byte)(byte)243;
            p320.param_id_SET("DEMO", PH);
            p320.param_index = (short)(short) -19981;
            CommunicationChannel.instance.send(p320); //===============================
            PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_system = (byte)(byte)42;
            p321.target_component = (byte)(byte)77;
            CommunicationChannel.instance.send(p321); //===============================
            PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_id_SET("DEMO", PH);
            p322.param_value_SET("DEMO", PH);
            p322.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64;
            p322.param_count = (ushort)(ushort)11271;
            p322.param_index = (ushort)(ushort)37671;
            CommunicationChannel.instance.send(p322); //===============================
            PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.target_system = (byte)(byte)14;
            p323.target_component = (byte)(byte)245;
            p323.param_id_SET("DEMO", PH);
            p323.param_value_SET("DEMO", PH);
            p323.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32;
            CommunicationChannel.instance.send(p323); //===============================
            PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_id_SET("DEMO", PH);
            p324.param_value_SET("DEMO", PH);
            p324.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32;
            p324.param_result = PARAM_ACK.PARAM_ACK_ACCEPTED;
            CommunicationChannel.instance.send(p324); //===============================
            OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.time_usec = (ulong)2320754476669160475L;
            p330.sensor_type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN;
            p330.distances_SET(new ushort[72], 0);
            p330.increment = (byte)(byte)99;
            p330.min_distance = (ushort)(ushort)14456;
            p330.max_distance = (ushort)(ushort)57894;
            CommunicationChannel.instance.send(p330); //===============================
        }
    }
}

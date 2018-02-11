
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
            p3.time_boot_ms = (uint)379481467U;
            p3.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_NED;
            p3.type_mask = (ushort)(ushort)8981;
            p3.x = (float)4.4122785E37F;
            p3.y = (float) -5.553785E37F;
            p3.z = (float)2.7580396E38F;
            p3.vx = (float) -2.7250367E38F;
            p3.vy = (float)6.049964E36F;
            p3.vz = (float)1.6742994E38F;
            p3.afx = (float)1.6979482E38F;
            p3.afy = (float) -2.5923084E38F;
            p3.afz = (float) -1.8984412E38F;
            p3.yaw = (float) -8.1169346E37F;
            p3.yaw_rate = (float) -2.0068402E37F;
            CommunicationChannel.instance.send(p3); //===============================
            SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.time_boot_ms = (uint)3710047163U;
            p84.target_system = (byte)(byte)142;
            p84.target_component = (byte)(byte)83;
            p84.coordinate_frame = MAV_FRAME.MAV_FRAME_MISSION;
            p84.type_mask = (ushort)(ushort)34372;
            p84.x = (float)2.6120068E38F;
            p84.y = (float) -5.1730397E37F;
            p84.z = (float) -3.2656497E38F;
            p84.vx = (float) -3.3386047E38F;
            p84.vy = (float)3.3629267E38F;
            p84.vz = (float)2.8878198E38F;
            p84.afx = (float) -3.089469E38F;
            p84.afy = (float)1.0217694E38F;
            p84.afz = (float)2.521288E38F;
            p84.yaw = (float)5.076089E37F;
            p84.yaw_rate = (float)3.2824962E38F;
            CommunicationChannel.instance.send(p84); //===============================
            SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.time_boot_ms = (uint)278266797U;
            p86.target_system = (byte)(byte)159;
            p86.target_component = (byte)(byte)22;
            p86.coordinate_frame = MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p86.type_mask = (ushort)(ushort)6167;
            p86.lat_int = (int)2082585517;
            p86.lon_int = (int)2103502921;
            p86.alt = (float) -1.6004605E38F;
            p86.vx = (float)2.7189785E38F;
            p86.vy = (float) -2.5679072E38F;
            p86.vz = (float) -3.007089E38F;
            p86.afx = (float) -1.3414157E38F;
            p86.afy = (float) -9.030537E37F;
            p86.afz = (float) -2.3418372E38F;
            p86.yaw = (float) -1.6805062E38F;
            p86.yaw_rate = (float)1.1887812E38F;
            CommunicationChannel.instance.send(p86); //===============================
            POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.time_boot_ms = (uint)2641177108U;
            p87.coordinate_frame = MAV_FRAME.MAV_FRAME_MISSION;
            p87.type_mask = (ushort)(ushort)28813;
            p87.lat_int = (int) -1168618986;
            p87.lon_int = (int)1381909174;
            p87.alt = (float) -7.028887E37F;
            p87.vx = (float) -9.146946E37F;
            p87.vy = (float)1.3772975E38F;
            p87.vz = (float) -9.699096E37F;
            p87.afx = (float) -2.8805184E38F;
            p87.afy = (float)1.3111306E38F;
            p87.afz = (float)5.145755E36F;
            p87.yaw = (float)2.739965E38F;
            p87.yaw_rate = (float)2.3227379E38F;
            CommunicationChannel.instance.send(p87); //===============================
            LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.time_boot_ms = (uint)2035739151U;
            p89.x = (float)8.591831E37F;
            p89.y = (float) -2.529723E38F;
            p89.z = (float)3.2560626E38F;
            p89.roll = (float)2.7310938E38F;
            p89.pitch = (float) -3.087037E38F;
            p89.yaw = (float)1.3951869E38F;
            CommunicationChannel.instance.send(p89); //===============================
            HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.time_usec = (ulong)4689903027247487679L;
            p90.roll = (float)1.2392075E38F;
            p90.pitch = (float) -4.270424E37F;
            p90.yaw = (float)1.5061026E38F;
            p90.rollspeed = (float)2.8807764E38F;
            p90.pitchspeed = (float) -1.2470456E38F;
            p90.yawspeed = (float) -3.0550897E38F;
            p90.lat = (int) -671945580;
            p90.lon = (int) -1585491788;
            p90.alt = (int)1128135293;
            p90.vx = (short)(short) -15540;
            p90.vy = (short)(short) -21950;
            p90.vz = (short)(short) -21849;
            p90.xacc = (short)(short)18135;
            p90.yacc = (short)(short) -14592;
            p90.zacc = (short)(short)22291;
            CommunicationChannel.instance.send(p90); //===============================
            HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.time_usec = (ulong)4627223535612050048L;
            p91.roll_ailerons = (float) -1.358389E38F;
            p91.pitch_elevator = (float) -8.908837E37F;
            p91.yaw_rudder = (float)1.5059341E38F;
            p91.throttle = (float)1.9383274E38F;
            p91.aux1 = (float)2.1101083E38F;
            p91.aux2 = (float) -2.3102783E37F;
            p91.aux3 = (float)2.1303063E38F;
            p91.aux4 = (float) -3.2155462E38F;
            p91.mode = MAV_MODE.MAV_MODE_GUIDED_DISARMED;
            p91.nav_mode = (byte)(byte)160;
            CommunicationChannel.instance.send(p91); //===============================
            HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.time_usec = (ulong)873085194636379229L;
            p92.chan1_raw = (ushort)(ushort)64183;
            p92.chan2_raw = (ushort)(ushort)38015;
            p92.chan3_raw = (ushort)(ushort)27856;
            p92.chan4_raw = (ushort)(ushort)740;
            p92.chan5_raw = (ushort)(ushort)2208;
            p92.chan6_raw = (ushort)(ushort)6629;
            p92.chan7_raw = (ushort)(ushort)55994;
            p92.chan8_raw = (ushort)(ushort)60613;
            p92.chan9_raw = (ushort)(ushort)12786;
            p92.chan10_raw = (ushort)(ushort)62933;
            p92.chan11_raw = (ushort)(ushort)11130;
            p92.chan12_raw = (ushort)(ushort)38949;
            p92.rssi = (byte)(byte)212;
            CommunicationChannel.instance.send(p92); //===============================
            HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.time_usec = (ulong)3471618528547750091L;
            p93.controls_SET(new float[16], 0);
            p93.mode = MAV_MODE.MAV_MODE_TEST_ARMED;
            p93.flags = (ulong)8384480221790470149L;
            CommunicationChannel.instance.send(p93); //===============================
            OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.time_usec = (ulong)1269013914088042767L;
            p100.sensor_id = (byte)(byte)58;
            p100.flow_x = (short)(short)21654;
            p100.flow_y = (short)(short)23654;
            p100.flow_comp_m_x = (float)1.6046574E38F;
            p100.flow_comp_m_y = (float)2.132362E38F;
            p100.quality = (byte)(byte)243;
            p100.ground_distance = (float) -2.2861947E37F;
            p100.flow_rate_x_SET((float) -1.983397E38F, PH);
            p100.flow_rate_y_SET((float) -1.0882777E38F, PH);
            CommunicationChannel.instance.send(p100); //===============================
            GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.usec = (ulong)539821750374774550L;
            p101.x = (float)2.1700803E38F;
            p101.y = (float) -1.9150637E38F;
            p101.z = (float)9.928723E37F;
            p101.roll = (float)2.618078E38F;
            p101.pitch = (float) -3.181628E38F;
            p101.yaw = (float) -2.1378755E38F;
            CommunicationChannel.instance.send(p101); //===============================
            VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.usec = (ulong)1217462922753619425L;
            p102.x = (float)2.8134811E38F;
            p102.y = (float)1.3931836E38F;
            p102.z = (float) -5.6124236E37F;
            p102.roll = (float) -3.2249828E38F;
            p102.pitch = (float)2.4863052E38F;
            p102.yaw = (float) -1.1938078E38F;
            CommunicationChannel.instance.send(p102); //===============================
            VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.usec = (ulong)2772581729759392424L;
            p103.x = (float)6.555143E37F;
            p103.y = (float) -2.5996153E38F;
            p103.z = (float)1.2184787E38F;
            CommunicationChannel.instance.send(p103); //===============================
            VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.usec = (ulong)1116948563890508490L;
            p104.x = (float)1.6185242E38F;
            p104.y = (float) -2.1160082E38F;
            p104.z = (float)2.917256E38F;
            p104.roll = (float)6.1387983E37F;
            p104.pitch = (float) -9.068471E37F;
            p104.yaw = (float)2.7448726E38F;
            CommunicationChannel.instance.send(p104); //===============================
            HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.time_usec = (ulong)3981678703381944968L;
            p105.xacc = (float)1.8516008E38F;
            p105.yacc = (float) -7.075829E37F;
            p105.zacc = (float) -2.7773904E38F;
            p105.xgyro = (float)2.189339E38F;
            p105.ygyro = (float) -2.6744085E38F;
            p105.zgyro = (float)1.4121006E38F;
            p105.xmag = (float)1.788267E38F;
            p105.ymag = (float)1.4448243E38F;
            p105.zmag = (float)1.7706961E38F;
            p105.abs_pressure = (float)8.1036497E37F;
            p105.diff_pressure = (float) -1.456044E38F;
            p105.pressure_alt = (float) -1.418927E38F;
            p105.temperature = (float) -2.2952013E38F;
            p105.fields_updated = (ushort)(ushort)24836;
            CommunicationChannel.instance.send(p105); //===============================
            OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.time_usec = (ulong)7333817975936096434L;
            p106.sensor_id = (byte)(byte)56;
            p106.integration_time_us = (uint)2359686840U;
            p106.integrated_x = (float) -3.081425E38F;
            p106.integrated_y = (float)3.3679846E38F;
            p106.integrated_xgyro = (float) -2.4146263E38F;
            p106.integrated_ygyro = (float) -3.4469735E37F;
            p106.integrated_zgyro = (float)3.2063549E38F;
            p106.temperature = (short)(short)2543;
            p106.quality = (byte)(byte)254;
            p106.time_delta_distance_us = (uint)3120900928U;
            p106.distance = (float)2.6377091E38F;
            CommunicationChannel.instance.send(p106); //===============================
            HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.time_usec = (ulong)149577212519561485L;
            p107.xacc = (float)1.5511385E38F;
            p107.yacc = (float) -5.603924E36F;
            p107.zacc = (float)2.5463273E38F;
            p107.xgyro = (float) -1.4694067E38F;
            p107.ygyro = (float)2.8951048E37F;
            p107.zgyro = (float) -7.5718E37F;
            p107.xmag = (float)2.944731E38F;
            p107.ymag = (float)1.5006777E38F;
            p107.zmag = (float) -1.343137E38F;
            p107.abs_pressure = (float)2.9469013E38F;
            p107.diff_pressure = (float)1.8029932E38F;
            p107.pressure_alt = (float) -1.9405195E38F;
            p107.temperature = (float)1.8837874E38F;
            p107.fields_updated = (uint)1743205397U;
            CommunicationChannel.instance.send(p107); //===============================
            SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.q1 = (float) -2.4252251E38F;
            p108.q2 = (float) -1.2432434E38F;
            p108.q3 = (float) -7.135216E37F;
            p108.q4 = (float) -2.33525E38F;
            p108.roll = (float) -1.7353046E38F;
            p108.pitch = (float) -2.015524E38F;
            p108.yaw = (float)3.3068966E38F;
            p108.xacc = (float)1.9942805E38F;
            p108.yacc = (float) -3.7920262E37F;
            p108.zacc = (float) -2.9336579E38F;
            p108.xgyro = (float) -3.0763785E38F;
            p108.ygyro = (float) -9.638016E37F;
            p108.zgyro = (float) -1.3476587E38F;
            p108.lat = (float) -8.77563E37F;
            p108.lon = (float) -2.1930524E38F;
            p108.alt = (float) -1.2551812E38F;
            p108.std_dev_horz = (float) -2.8825847E38F;
            p108.std_dev_vert = (float)3.164808E38F;
            p108.vn = (float)5.4500244E37F;
            p108.ve = (float)2.4209282E37F;
            p108.vd = (float) -7.600997E37F;
            CommunicationChannel.instance.send(p108); //===============================
            RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.rssi = (byte)(byte)195;
            p109.remrssi = (byte)(byte)164;
            p109.txbuf = (byte)(byte)136;
            p109.noise = (byte)(byte)92;
            p109.remnoise = (byte)(byte)146;
            p109.rxerrors = (ushort)(ushort)60381;
            p109.fixed_ = (ushort)(ushort)14841;
            CommunicationChannel.instance.send(p109); //===============================
            FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_network = (byte)(byte)32;
            p110.target_system = (byte)(byte)243;
            p110.target_component = (byte)(byte)179;
            p110.payload_SET(new byte[251], 0);
            CommunicationChannel.instance.send(p110); //===============================
            TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.tc1 = (long) -2394610349894528316L;
            p111.ts1 = (long)368303375097328333L;
            CommunicationChannel.instance.send(p111); //===============================
            CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.time_usec = (ulong)232193811080262704L;
            p112.seq = (uint)1214486013U;
            CommunicationChannel.instance.send(p112); //===============================
            HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.time_usec = (ulong)6223333867806549569L;
            p113.fix_type = (byte)(byte)124;
            p113.lat = (int) -1316756947;
            p113.lon = (int) -1651755202;
            p113.alt = (int)666606920;
            p113.eph = (ushort)(ushort)14174;
            p113.epv = (ushort)(ushort)11580;
            p113.vel = (ushort)(ushort)30880;
            p113.vn = (short)(short)22932;
            p113.ve = (short)(short)16718;
            p113.vd = (short)(short) -29206;
            p113.cog = (ushort)(ushort)41901;
            p113.satellites_visible = (byte)(byte)246;
            CommunicationChannel.instance.send(p113); //===============================
            HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.time_usec = (ulong)3877257663210787434L;
            p114.sensor_id = (byte)(byte)127;
            p114.integration_time_us = (uint)2580515852U;
            p114.integrated_x = (float)6.4621785E37F;
            p114.integrated_y = (float) -2.1620638E37F;
            p114.integrated_xgyro = (float)2.3890583E38F;
            p114.integrated_ygyro = (float)7.663758E37F;
            p114.integrated_zgyro = (float) -1.3282974E37F;
            p114.temperature = (short)(short) -747;
            p114.quality = (byte)(byte)84;
            p114.time_delta_distance_us = (uint)1159987225U;
            p114.distance = (float)2.557859E37F;
            CommunicationChannel.instance.send(p114); //===============================
            HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.time_usec = (ulong)2522601407214692260L;
            p115.attitude_quaternion_SET(new float[4], 0);
            p115.rollspeed = (float)2.5342439E38F;
            p115.pitchspeed = (float)2.0036487E38F;
            p115.yawspeed = (float) -2.1171197E38F;
            p115.lat = (int) -1794033395;
            p115.lon = (int)2052446527;
            p115.alt = (int) -980980223;
            p115.vx = (short)(short) -23055;
            p115.vy = (short)(short)1477;
            p115.vz = (short)(short) -28657;
            p115.ind_airspeed = (ushort)(ushort)1576;
            p115.true_airspeed = (ushort)(ushort)16846;
            p115.xacc = (short)(short) -11797;
            p115.yacc = (short)(short) -2201;
            p115.zacc = (short)(short)28727;
            CommunicationChannel.instance.send(p115); //===============================
            SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.time_boot_ms = (uint)3639677442U;
            p116.xacc = (short)(short)30034;
            p116.yacc = (short)(short)24594;
            p116.zacc = (short)(short)3405;
            p116.xgyro = (short)(short)2235;
            p116.ygyro = (short)(short)28769;
            p116.zgyro = (short)(short)5196;
            p116.xmag = (short)(short)28908;
            p116.ymag = (short)(short)11752;
            p116.zmag = (short)(short) -25129;
            CommunicationChannel.instance.send(p116); //===============================
            LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.target_system = (byte)(byte)92;
            p117.target_component = (byte)(byte)249;
            p117.start = (ushort)(ushort)61480;
            p117.end = (ushort)(ushort)11884;
            CommunicationChannel.instance.send(p117); //===============================
            LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.id = (ushort)(ushort)48517;
            p118.num_logs = (ushort)(ushort)3467;
            p118.last_log_num = (ushort)(ushort)3100;
            p118.time_utc = (uint)2831596208U;
            p118.size = (uint)1123277087U;
            CommunicationChannel.instance.send(p118); //===============================
            LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.target_system = (byte)(byte)252;
            p119.target_component = (byte)(byte)65;
            p119.id = (ushort)(ushort)40708;
            p119.ofs = (uint)1256791905U;
            p119.count = (uint)737934376U;
            CommunicationChannel.instance.send(p119); //===============================
            LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.id = (ushort)(ushort)12514;
            p120.ofs = (uint)2394412610U;
            p120.count = (byte)(byte)123;
            p120.data__SET(new byte[90], 0);
            CommunicationChannel.instance.send(p120); //===============================
            LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_system = (byte)(byte)13;
            p121.target_component = (byte)(byte)180;
            CommunicationChannel.instance.send(p121); //===============================
            LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_system = (byte)(byte)197;
            p122.target_component = (byte)(byte)68;
            CommunicationChannel.instance.send(p122); //===============================
            GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.target_system = (byte)(byte)31;
            p123.target_component = (byte)(byte)89;
            p123.len = (byte)(byte)214;
            p123.data__SET(new byte[110], 0);
            CommunicationChannel.instance.send(p123); //===============================
            GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.time_usec = (ulong)661093336839269699L;
            p124.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS;
            p124.lat = (int) -699914270;
            p124.lon = (int) -2010715324;
            p124.alt = (int)7206436;
            p124.eph = (ushort)(ushort)25203;
            p124.epv = (ushort)(ushort)62139;
            p124.vel = (ushort)(ushort)34666;
            p124.cog = (ushort)(ushort)41094;
            p124.satellites_visible = (byte)(byte)49;
            p124.dgps_numch = (byte)(byte)189;
            p124.dgps_age = (uint)2921767063U;
            CommunicationChannel.instance.send(p124); //===============================
            POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vcc = (ushort)(ushort)34758;
            p125.Vservo = (ushort)(ushort)46843;
            p125.flags = (MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED |
                          MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT);
            CommunicationChannel.instance.send(p125); //===============================
            SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.device = SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS2;
            p126.flags = (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND |
                          SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING |
                          SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE);
            p126.timeout = (ushort)(ushort)9082;
            p126.baudrate = (uint)2714267550U;
            p126.count = (byte)(byte)233;
            p126.data__SET(new byte[70], 0);
            CommunicationChannel.instance.send(p126); //===============================
            GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.time_last_baseline_ms = (uint)3864553071U;
            p127.rtk_receiver_id = (byte)(byte)233;
            p127.wn = (ushort)(ushort)1543;
            p127.tow = (uint)1336160932U;
            p127.rtk_health = (byte)(byte)31;
            p127.rtk_rate = (byte)(byte)249;
            p127.nsats = (byte)(byte)16;
            p127.baseline_coords_type = (byte)(byte)223;
            p127.baseline_a_mm = (int) -112473799;
            p127.baseline_b_mm = (int)1582539500;
            p127.baseline_c_mm = (int)440780508;
            p127.accuracy = (uint)1465701749U;
            p127.iar_num_hypotheses = (int) -1526661965;
            CommunicationChannel.instance.send(p127); //===============================
            GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.time_last_baseline_ms = (uint)151287642U;
            p128.rtk_receiver_id = (byte)(byte)177;
            p128.wn = (ushort)(ushort)30103;
            p128.tow = (uint)3496486951U;
            p128.rtk_health = (byte)(byte)111;
            p128.rtk_rate = (byte)(byte)101;
            p128.nsats = (byte)(byte)147;
            p128.baseline_coords_type = (byte)(byte)50;
            p128.baseline_a_mm = (int)1522299649;
            p128.baseline_b_mm = (int) -1088009399;
            p128.baseline_c_mm = (int) -483612803;
            p128.accuracy = (uint)118257916U;
            p128.iar_num_hypotheses = (int) -2039481;
            CommunicationChannel.instance.send(p128); //===============================
            SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.time_boot_ms = (uint)3421222634U;
            p129.xacc = (short)(short)21224;
            p129.yacc = (short)(short)29136;
            p129.zacc = (short)(short)511;
            p129.xgyro = (short)(short) -22712;
            p129.ygyro = (short)(short)22041;
            p129.zgyro = (short)(short)16656;
            p129.xmag = (short)(short)3027;
            p129.ymag = (short)(short)2716;
            p129.zmag = (short)(short) -3894;
            CommunicationChannel.instance.send(p129); //===============================
            DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.type = (byte)(byte)217;
            p130.size = (uint)1844343684U;
            p130.width = (ushort)(ushort)21529;
            p130.height = (ushort)(ushort)50290;
            p130.packets = (ushort)(ushort)54373;
            p130.payload = (byte)(byte)91;
            p130.jpg_quality = (byte)(byte)13;
            CommunicationChannel.instance.send(p130); //===============================
            ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.seqnr = (ushort)(ushort)24335;
            p131.data__SET(new byte[253], 0);
            CommunicationChannel.instance.send(p131); //===============================
            DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.time_boot_ms = (uint)3638802274U;
            p132.min_distance = (ushort)(ushort)3935;
            p132.max_distance = (ushort)(ushort)36848;
            p132.current_distance = (ushort)(ushort)53442;
            p132.type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER;
            p132.id = (byte)(byte)178;
            p132.orientation = MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_180;
            p132.covariance = (byte)(byte)244;
            CommunicationChannel.instance.send(p132); //===============================
            TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.lat = (int) -1799412052;
            p133.lon = (int) -140771791;
            p133.grid_spacing = (ushort)(ushort)54787;
            p133.mask = (ulong)1425385951601786655L;
            CommunicationChannel.instance.send(p133); //===============================
            TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.lat = (int)2018611242;
            p134.lon = (int)38758088;
            p134.grid_spacing = (ushort)(ushort)36486;
            p134.gridbit = (byte)(byte)174;
            p134.data__SET(new short[16], 0);
            CommunicationChannel.instance.send(p134); //===============================
            TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lat = (int)1274762949;
            p135.lon = (int) -346647373;
            CommunicationChannel.instance.send(p135); //===============================
            TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.lat = (int)2054923834;
            p136.lon = (int)1398661821;
            p136.spacing = (ushort)(ushort)63703;
            p136.terrain_height = (float) -1.5415035E38F;
            p136.current_height = (float) -2.0466103E38F;
            p136.pending = (ushort)(ushort)59356;
            p136.loaded = (ushort)(ushort)14683;
            CommunicationChannel.instance.send(p136); //===============================
            SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.time_boot_ms = (uint)359070367U;
            p137.press_abs = (float)1.0972897E38F;
            p137.press_diff = (float)1.8033727E38F;
            p137.temperature = (short)(short) -29232;
            CommunicationChannel.instance.send(p137); //===============================
            ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.time_usec = (ulong)2581873081168496536L;
            p138.q_SET(new float[4], 0);
            p138.x = (float)1.2316159E38F;
            p138.y = (float)2.1993432E38F;
            p138.z = (float) -8.126438E37F;
            CommunicationChannel.instance.send(p138); //===============================
            SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.time_usec = (ulong)8302137762320776258L;
            p139.group_mlx = (byte)(byte)33;
            p139.target_system = (byte)(byte)155;
            p139.target_component = (byte)(byte)135;
            p139.controls_SET(new float[8], 0);
            CommunicationChannel.instance.send(p139); //===============================
            ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.time_usec = (ulong)161742647515044504L;
            p140.group_mlx = (byte)(byte)112;
            p140.controls_SET(new float[8], 0);
            CommunicationChannel.instance.send(p140); //===============================
            ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.time_usec = (ulong)2974271191849661362L;
            p141.altitude_monotonic = (float)1.6935421E38F;
            p141.altitude_amsl = (float)5.1340427E37F;
            p141.altitude_local = (float)3.4078264E36F;
            p141.altitude_relative = (float)3.3162366E38F;
            p141.altitude_terrain = (float) -1.3610192E38F;
            p141.bottom_clearance = (float)1.5398233E38F;
            CommunicationChannel.instance.send(p141); //===============================
            RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.request_id = (byte)(byte)189;
            p142.uri_type = (byte)(byte)108;
            p142.uri_SET(new byte[120], 0);
            p142.transfer_type = (byte)(byte)217;
            p142.storage_SET(new byte[120], 0);
            CommunicationChannel.instance.send(p142); //===============================
            SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.time_boot_ms = (uint)416769267U;
            p143.press_abs = (float)1.2952939E38F;
            p143.press_diff = (float)1.5251253E38F;
            p143.temperature = (short)(short)14278;
            CommunicationChannel.instance.send(p143); //===============================
            FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.timestamp = (ulong)1033344769058295768L;
            p144.est_capabilities = (byte)(byte)153;
            p144.lat = (int) -1532609341;
            p144.lon = (int) -1033740862;
            p144.alt = (float) -2.6619178E38F;
            p144.vel_SET(new float[3], 0);
            p144.acc_SET(new float[3], 0);
            p144.attitude_q_SET(new float[4], 0);
            p144.rates_SET(new float[3], 0);
            p144.position_cov_SET(new float[3], 0);
            p144.custom_state = (ulong)4119860234260452894L;
            CommunicationChannel.instance.send(p144); //===============================
            CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.time_usec = (ulong)2890390314843169441L;
            p146.x_acc = (float)2.851125E38F;
            p146.y_acc = (float)7.0928666E37F;
            p146.z_acc = (float)3.1423894E38F;
            p146.x_vel = (float)2.1313466E38F;
            p146.y_vel = (float) -1.9996338E38F;
            p146.z_vel = (float)2.6577414E38F;
            p146.x_pos = (float)5.3962887E37F;
            p146.y_pos = (float) -2.8292047E38F;
            p146.z_pos = (float)3.051635E38F;
            p146.airspeed = (float)1.139629E38F;
            p146.vel_variance_SET(new float[3], 0);
            p146.pos_variance_SET(new float[3], 0);
            p146.q_SET(new float[4], 0);
            p146.roll_rate = (float)1.5886793E38F;
            p146.pitch_rate = (float) -9.811282E37F;
            p146.yaw_rate = (float) -8.796736E37F;
            CommunicationChannel.instance.send(p146); //===============================
            BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.id = (byte)(byte)217;
            p147.battery_function = MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_PROPULSION;
            p147.type = MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_UNKNOWN;
            p147.temperature = (short)(short) -8714;
            p147.voltages_SET(new ushort[10], 0);
            p147.current_battery = (short)(short)15063;
            p147.current_consumed = (int)605787953;
            p147.energy_consumed = (int)1395890263;
            p147.battery_remaining = (sbyte)(sbyte) - 125;
            CommunicationChannel.instance.send(p147); //===============================
            AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_INT |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMMAND_INT);
            p148.flight_sw_version = (uint)534001863U;
            p148.middleware_sw_version = (uint)3871687065U;
            p148.os_sw_version = (uint)1238010449U;
            p148.board_version = (uint)1758812591U;
            p148.flight_custom_version_SET(new byte[8], 0);
            p148.middleware_custom_version_SET(new byte[8], 0);
            p148.os_custom_version_SET(new byte[8], 0);
            p148.vendor_id = (ushort)(ushort)18777;
            p148.product_id = (ushort)(ushort)38405;
            p148.uid = (ulong)778249603238712223L;
            p148.uid2_SET(new byte[18], 0, PH);
            CommunicationChannel.instance.send(p148); //===============================
            LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.time_usec = (ulong)8379352398917176810L;
            p149.target_num = (byte)(byte)216;
            p149.frame = MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p149.angle_x = (float)1.9746901E38F;
            p149.angle_y = (float)1.7674706E38F;
            p149.distance = (float)2.4309634E37F;
            p149.size_x = (float) -9.797725E36F;
            p149.size_y = (float)3.2819642E38F;
            p149.x_SET((float) -6.4020894E36F, PH);
            p149.y_SET((float)3.0418905E38F, PH);
            p149.z_SET((float) -2.5981742E38F, PH);
            p149.q_SET(new float[4], 0, PH);
            p149.type = LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_FIDUCIAL;
            p149.position_valid_SET((byte)(byte)130, PH);
            CommunicationChannel.instance.send(p149); //===============================
            CPU_LOAD p170 = CommunicationChannel.new_CPU_LOAD();
            PH.setPack(p170);
            p170.sensLoad = (byte)(byte)26;
            p170.ctrlLoad = (byte)(byte)0;
            p170.batVolt = (ushort)(ushort)48642;
            CommunicationChannel.instance.send(p170); //===============================
            SENSOR_BIAS p172 = CommunicationChannel.new_SENSOR_BIAS();
            PH.setPack(p172);
            p172.axBias = (float) -2.3574675E38F;
            p172.ayBias = (float)2.1302158E38F;
            p172.azBias = (float)3.063674E38F;
            p172.gxBias = (float)1.8256513E38F;
            p172.gyBias = (float) -2.5813709E38F;
            p172.gzBias = (float)2.2534818E38F;
            CommunicationChannel.instance.send(p172); //===============================
            DIAGNOSTIC p173 = CommunicationChannel.new_DIAGNOSTIC();
            PH.setPack(p173);
            p173.diagFl1 = (float) -5.8881E37F;
            p173.diagFl2 = (float) -2.4997025E38F;
            p173.diagFl3 = (float) -1.6097187E38F;
            p173.diagSh1 = (short)(short)3185;
            p173.diagSh2 = (short)(short) -29825;
            p173.diagSh3 = (short)(short)3762;
            CommunicationChannel.instance.send(p173); //===============================
            SLUGS_NAVIGATION p176 = CommunicationChannel.new_SLUGS_NAVIGATION();
            PH.setPack(p176);
            p176.u_m = (float)1.874071E38F;
            p176.phi_c = (float)3.3901504E38F;
            p176.theta_c = (float)2.2241273E38F;
            p176.psiDot_c = (float)2.445356E38F;
            p176.ay_body = (float)8.827024E37F;
            p176.totalDist = (float) -3.1247151E38F;
            p176.dist2Go = (float)1.3069729E38F;
            p176.fromWP = (byte)(byte)255;
            p176.toWP = (byte)(byte)78;
            p176.h_c = (ushort)(ushort)34860;
            CommunicationChannel.instance.send(p176); //===============================
            DATA_LOG p177 = CommunicationChannel.new_DATA_LOG();
            PH.setPack(p177);
            p177.fl_1 = (float) -2.0702813E38F;
            p177.fl_2 = (float)1.9161342E38F;
            p177.fl_3 = (float)2.8793357E38F;
            p177.fl_4 = (float) -1.8818985E38F;
            p177.fl_5 = (float) -2.5096411E38F;
            p177.fl_6 = (float)1.2094114E38F;
            CommunicationChannel.instance.send(p177); //===============================
            GPS_DATE_TIME p179 = CommunicationChannel.new_GPS_DATE_TIME();
            PH.setPack(p179);
            p179.year = (byte)(byte)207;
            p179.month = (byte)(byte)105;
            p179.day = (byte)(byte)162;
            p179.hour = (byte)(byte)56;
            p179.min = (byte)(byte)12;
            p179.sec = (byte)(byte)17;
            p179.clockStat = (byte)(byte)24;
            p179.visSat = (byte)(byte)203;
            p179.useSat = (byte)(byte)232;
            p179.GppGl = (byte)(byte)66;
            p179.sigUsedMask = (byte)(byte)116;
            p179.percentUsed = (byte)(byte)129;
            CommunicationChannel.instance.send(p179); //===============================
            MID_LVL_CMDS p180 = CommunicationChannel.new_MID_LVL_CMDS();
            PH.setPack(p180);
            p180.target = (byte)(byte)39;
            p180.hCommand = (float) -2.0918314E38F;
            p180.uCommand = (float) -2.5399475E38F;
            p180.rCommand = (float)1.2117705E38F;
            CommunicationChannel.instance.send(p180); //===============================
            CTRL_SRFC_PT p181 = CommunicationChannel.new_CTRL_SRFC_PT();
            PH.setPack(p181);
            p181.target = (byte)(byte)2;
            p181.bitfieldPt = (ushort)(ushort)45033;
            CommunicationChannel.instance.send(p181); //===============================
            SLUGS_CAMERA_ORDER p184 = CommunicationChannel.new_SLUGS_CAMERA_ORDER();
            PH.setPack(p184);
            p184.target = (byte)(byte)213;
            p184.pan = (sbyte)(sbyte) - 120;
            p184.tilt = (sbyte)(sbyte)76;
            p184.zoom = (sbyte)(sbyte) - 49;
            p184.moveHome = (sbyte)(sbyte)97;
            CommunicationChannel.instance.send(p184); //===============================
            CONTROL_SURFACE p185 = CommunicationChannel.new_CONTROL_SURFACE();
            PH.setPack(p185);
            p185.target = (byte)(byte)17;
            p185.idSurface = (byte)(byte)16;
            p185.mControl = (float) -2.3482075E38F;
            p185.bControl = (float) -3.3048661E38F;
            CommunicationChannel.instance.send(p185); //===============================
            SLUGS_MOBILE_LOCATION p186 = CommunicationChannel.new_SLUGS_MOBILE_LOCATION();
            PH.setPack(p186);
            p186.target = (byte)(byte)199;
            p186.latitude = (float)1.4920342E38F;
            p186.longitude = (float)5.7806465E37F;
            CommunicationChannel.instance.send(p186); //===============================
            SLUGS_CONFIGURATION_CAMERA p188 = CommunicationChannel.new_SLUGS_CONFIGURATION_CAMERA();
            PH.setPack(p188);
            p188.target = (byte)(byte)183;
            p188.idOrder = (byte)(byte)234;
            p188.order = (byte)(byte)176;
            CommunicationChannel.instance.send(p188); //===============================
            ISR_LOCATION p189 = CommunicationChannel.new_ISR_LOCATION();
            PH.setPack(p189);
            p189.target = (byte)(byte)144;
            p189.latitude = (float) -2.615518E38F;
            p189.longitude = (float) -7.9507183E37F;
            p189.height = (float) -2.4787891E38F;
            p189.option1 = (byte)(byte)33;
            p189.option2 = (byte)(byte)212;
            p189.option3 = (byte)(byte)185;
            CommunicationChannel.instance.send(p189); //===============================
            VOLT_SENSOR p191 = CommunicationChannel.new_VOLT_SENSOR();
            PH.setPack(p191);
            p191.r2Type = (byte)(byte)73;
            p191.voltage = (ushort)(ushort)37522;
            p191.reading2 = (ushort)(ushort)51886;
            CommunicationChannel.instance.send(p191); //===============================
            PTZ_STATUS p192 = CommunicationChannel.new_PTZ_STATUS();
            PH.setPack(p192);
            p192.zoom = (byte)(byte)59;
            p192.pan = (short)(short) -12649;
            p192.tilt = (short)(short)18871;
            CommunicationChannel.instance.send(p192); //===============================
            UAV_STATUS p193 = CommunicationChannel.new_UAV_STATUS();
            PH.setPack(p193);
            p193.target = (byte)(byte)146;
            p193.latitude = (float) -3.2494455E38F;
            p193.longitude = (float)2.583465E38F;
            p193.altitude = (float) -1.3931267E38F;
            p193.speed = (float)1.6059017E38F;
            p193.course = (float)2.589119E38F;
            CommunicationChannel.instance.send(p193); //===============================
            STATUS_GPS p194 = CommunicationChannel.new_STATUS_GPS();
            PH.setPack(p194);
            p194.csFails = (ushort)(ushort)24544;
            p194.gpsQuality = (byte)(byte)254;
            p194.msgsType = (byte)(byte)128;
            p194.posStatus = (byte)(byte)51;
            p194.magVar = (float) -2.6705334E38F;
            p194.magDir = (sbyte)(sbyte) - 67;
            p194.modeInd = (byte)(byte)222;
            CommunicationChannel.instance.send(p194); //===============================
            NOVATEL_DIAG p195 = CommunicationChannel.new_NOVATEL_DIAG();
            PH.setPack(p195);
            p195.timeStatus = (byte)(byte)225;
            p195.receiverStatus = (uint)4269700885U;
            p195.solStatus = (byte)(byte)60;
            p195.posType = (byte)(byte)203;
            p195.velType = (byte)(byte)240;
            p195.posSolAge = (float)5.4808075E37F;
            p195.csFails = (ushort)(ushort)13021;
            CommunicationChannel.instance.send(p195); //===============================
            SENSOR_DIAG p196 = CommunicationChannel.new_SENSOR_DIAG();
            PH.setPack(p196);
            p196.float1 = (float) -2.1830821E36F;
            p196.float2 = (float)1.1621273E38F;
            p196.int1 = (short)(short)14484;
            p196.char1 = (sbyte)(sbyte)87;
            CommunicationChannel.instance.send(p196); //===============================
            BOOT p197 = CommunicationChannel.new_BOOT();
            PH.setPack(p197);
            p197.version = (uint)2967103491U;
            CommunicationChannel.instance.send(p197); //===============================
            ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.time_usec = (ulong)5827160092541949050L;
            p230.flags = (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ);
            p230.vel_ratio = (float)3.21941E38F;
            p230.pos_horiz_ratio = (float) -1.5862317E37F;
            p230.pos_vert_ratio = (float)2.323771E38F;
            p230.mag_ratio = (float) -2.6780796E38F;
            p230.hagl_ratio = (float) -6.209473E37F;
            p230.tas_ratio = (float) -1.2675486E38F;
            p230.pos_horiz_accuracy = (float)1.2445195E38F;
            p230.pos_vert_accuracy = (float)1.7084571E38F;
            CommunicationChannel.instance.send(p230); //===============================
            WIND_COV p231 = CommunicationChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.time_usec = (ulong)8359532368510839099L;
            p231.wind_x = (float) -2.148892E38F;
            p231.wind_y = (float)2.2196655E38F;
            p231.wind_z = (float)2.259534E38F;
            p231.var_horiz = (float) -7.416695E37F;
            p231.var_vert = (float)2.3502615E37F;
            p231.wind_alt = (float)1.5165533E38F;
            p231.horiz_accuracy = (float) -1.5451841E38F;
            p231.vert_accuracy = (float) -3.2290063E38F;
            CommunicationChannel.instance.send(p231); //===============================
            GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.time_usec = (ulong)3923649064246462180L;
            p232.gps_id = (byte)(byte)206;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP);
            p232.time_week_ms = (uint)476209516U;
            p232.time_week = (ushort)(ushort)24558;
            p232.fix_type = (byte)(byte)174;
            p232.lat = (int)2048348534;
            p232.lon = (int) -680528561;
            p232.alt = (float) -3.0727863E38F;
            p232.hdop = (float) -2.9861544E38F;
            p232.vdop = (float)6.4251195E37F;
            p232.vn = (float)3.2320861E38F;
            p232.ve = (float) -1.7046765E38F;
            p232.vd = (float)2.0192013E38F;
            p232.speed_accuracy = (float) -5.7739487E37F;
            p232.horiz_accuracy = (float)1.949878E38F;
            p232.vert_accuracy = (float)8.874392E37F;
            p232.satellites_visible = (byte)(byte)203;
            CommunicationChannel.instance.send(p232); //===============================
            GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.flags = (byte)(byte)235;
            p233.len = (byte)(byte)252;
            p233.data__SET(new byte[180], 0);
            CommunicationChannel.instance.send(p233); //===============================
            HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED);
            p234.custom_mode = (uint)129456690U;
            p234.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED;
            p234.roll = (short)(short) -19840;
            p234.pitch = (short)(short) -16406;
            p234.heading = (ushort)(ushort)64313;
            p234.throttle = (sbyte)(sbyte)127;
            p234.heading_sp = (short)(short) -4633;
            p234.latitude = (int)778300747;
            p234.longitude = (int)2002945726;
            p234.altitude_amsl = (short)(short) -564;
            p234.altitude_sp = (short)(short)18356;
            p234.airspeed = (byte)(byte)110;
            p234.airspeed_sp = (byte)(byte)148;
            p234.groundspeed = (byte)(byte)193;
            p234.climb_rate = (sbyte)(sbyte)75;
            p234.gps_nsat = (byte)(byte)15;
            p234.gps_fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX;
            p234.battery_remaining = (byte)(byte)88;
            p234.temperature = (sbyte)(sbyte) - 124;
            p234.temperature_air = (sbyte)(sbyte) - 74;
            p234.failsafe = (byte)(byte)45;
            p234.wp_num = (byte)(byte)193;
            p234.wp_distance = (ushort)(ushort)32537;
            CommunicationChannel.instance.send(p234); //===============================
            VIBRATION p241 = CommunicationChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.time_usec = (ulong)8876096783066126194L;
            p241.vibration_x = (float) -1.7814894E38F;
            p241.vibration_y = (float) -3.1439702E38F;
            p241.vibration_z = (float) -2.847378E37F;
            p241.clipping_0 = (uint)1336442505U;
            p241.clipping_1 = (uint)479009825U;
            p241.clipping_2 = (uint)3807945213U;
            CommunicationChannel.instance.send(p241); //===============================
            HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.latitude = (int)1496149959;
            p242.longitude = (int) -614807250;
            p242.altitude = (int) -1969697390;
            p242.x = (float) -2.9084144E37F;
            p242.y = (float)2.9481255E38F;
            p242.z = (float)2.7391715E38F;
            p242.q_SET(new float[4], 0);
            p242.approach_x = (float)2.9838724E38F;
            p242.approach_y = (float) -1.2671573E38F;
            p242.approach_z = (float)1.1832199E38F;
            p242.time_usec_SET((ulong)8709322732566323715L, PH);
            CommunicationChannel.instance.send(p242); //===============================
            SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.target_system = (byte)(byte)31;
            p243.latitude = (int) -643305614;
            p243.longitude = (int) -1297144941;
            p243.altitude = (int) -1591358702;
            p243.x = (float)2.8000447E37F;
            p243.y = (float)7.256837E37F;
            p243.z = (float) -1.9351583E38F;
            p243.q_SET(new float[4], 0);
            p243.approach_x = (float)1.8789574E38F;
            p243.approach_y = (float) -2.794109E38F;
            p243.approach_z = (float) -8.641639E37F;
            p243.time_usec_SET((ulong)3541809702363951773L, PH);
            CommunicationChannel.instance.send(p243); //===============================
            MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.message_id = (ushort)(ushort)4386;
            p244.interval_us = (int) -1196183002;
            CommunicationChannel.instance.send(p244); //===============================
            EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.vtol_state = MAV_VTOL_STATE.MAV_VTOL_STATE_FW;
            p245.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND;
            CommunicationChannel.instance.send(p245); //===============================
            ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.ICAO_address = (uint)187612562U;
            p246.lat = (int) -826838740;
            p246.lon = (int) -931915536;
            p246.altitude_type = ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
            p246.altitude = (int) -217210712;
            p246.heading = (ushort)(ushort)40528;
            p246.hor_velocity = (ushort)(ushort)22329;
            p246.ver_velocity = (short)(short) -15963;
            p246.callsign_SET("DEMO", PH);
            p246.emitter_type = ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_LARGE;
            p246.tslc = (byte)(byte)74;
            p246.flags = (ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS);
            p246.squawk = (ushort)(ushort)36048;
            CommunicationChannel.instance.send(p246); //===============================
            COLLISION p247 = CommunicationChannel.new_COLLISION();
            PH.setPack(p247);
            p247.src_ = MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT;
            p247.id = (uint)4170260273U;
            p247.action = MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_RTL;
            p247.threat_level = (MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH);
            p247.time_to_minimum_delta = (float) -2.9278987E38F;
            p247.altitude_minimum_delta = (float) -3.282864E38F;
            p247.horizontal_minimum_delta = (float) -1.1627202E37F;
            CommunicationChannel.instance.send(p247); //===============================
            V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.target_network = (byte)(byte)70;
            p248.target_system = (byte)(byte)117;
            p248.target_component = (byte)(byte)104;
            p248.message_type = (ushort)(ushort)5160;
            p248.payload_SET(new byte[249], 0);
            CommunicationChannel.instance.send(p248); //===============================
            MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.address = (ushort)(ushort)35239;
            p249.ver = (byte)(byte)198;
            p249.type = (byte)(byte)117;
            p249.value_SET(new sbyte[32], 0);
            CommunicationChannel.instance.send(p249); //===============================
            DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.name_SET("DEMO", PH);
            p250.time_usec = (ulong)2383521262732663966L;
            p250.x = (float) -1.8535585E38F;
            p250.y = (float) -3.0803317E38F;
            p250.z = (float) -5.722367E37F;
            CommunicationChannel.instance.send(p250); //===============================
            NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.time_boot_ms = (uint)1062202408U;
            p251.name_SET("DEMO", PH);
            p251.value = (float) -8.6700836E36F;
            CommunicationChannel.instance.send(p251); //===============================
            NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.time_boot_ms = (uint)135019546U;
            p252.name_SET("DEMO", PH);
            p252.value = (int) -2015993479;
            CommunicationChannel.instance.send(p252); //===============================
            STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.severity = MAV_SEVERITY.MAV_SEVERITY_INFO;
            p253.text_SET("DEMO", PH);
            CommunicationChannel.instance.send(p253); //===============================
            DEBUG p254 = CommunicationChannel.new_DEBUG();
            PH.setPack(p254);
            p254.time_boot_ms = (uint)1410354575U;
            p254.ind = (byte)(byte)31;
            p254.value = (float) -2.0760898E38F;
            CommunicationChannel.instance.send(p254); //===============================
            SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.target_system = (byte)(byte)118;
            p256.target_component = (byte)(byte)113;
            p256.secret_key_SET(new byte[32], 0);
            p256.initial_timestamp = (ulong)3598213465323253188L;
            CommunicationChannel.instance.send(p256); //===============================
            BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.time_boot_ms = (uint)2275861659U;
            p257.last_change_ms = (uint)3394910642U;
            p257.state = (byte)(byte)67;
            CommunicationChannel.instance.send(p257); //===============================
            PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_system = (byte)(byte)42;
            p258.target_component = (byte)(byte)175;
            p258.tune_SET("DEMO", PH);
            CommunicationChannel.instance.send(p258); //===============================
            CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.time_boot_ms = (uint)272288073U;
            p259.vendor_name_SET(new byte[32], 0);
            p259.model_name_SET(new byte[32], 0);
            p259.firmware_version = (uint)1162984583U;
            p259.focal_length = (float) -2.2604977E38F;
            p259.sensor_size_h = (float)1.9927295E38F;
            p259.sensor_size_v = (float) -2.752231E38F;
            p259.resolution_h = (ushort)(ushort)54046;
            p259.resolution_v = (ushort)(ushort)5252;
            p259.lens_id = (byte)(byte)117;
            p259.flags = (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE |
                          CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE |
                          CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO);
            p259.cam_definition_version = (ushort)(ushort)28151;
            p259.cam_definition_uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p259); //===============================
            CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.time_boot_ms = (uint)1625067984U;
            p260.mode_id = (CAMERA_MODE.CAMERA_MODE_IMAGE);
            CommunicationChannel.instance.send(p260); //===============================
            STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.time_boot_ms = (uint)2001166089U;
            p261.storage_id = (byte)(byte)73;
            p261.storage_count = (byte)(byte)111;
            p261.status = (byte)(byte)75;
            p261.total_capacity = (float)1.5756533E38F;
            p261.used_capacity = (float)1.1842136E38F;
            p261.available_capacity = (float) -6.174187E37F;
            p261.read_speed = (float) -7.3578954E37F;
            p261.write_speed = (float)1.4972887E38F;
            CommunicationChannel.instance.send(p261); //===============================
            CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.time_boot_ms = (uint)4267567088U;
            p262.image_status = (byte)(byte)227;
            p262.video_status = (byte)(byte)59;
            p262.image_interval = (float) -2.489719E37F;
            p262.recording_time_ms = (uint)409888007U;
            p262.available_capacity = (float)1.9713441E38F;
            CommunicationChannel.instance.send(p262); //===============================
            CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.time_boot_ms = (uint)3171680224U;
            p263.time_utc = (ulong)2786531426081481746L;
            p263.camera_id = (byte)(byte)218;
            p263.lat = (int) -2006823317;
            p263.lon = (int)652932618;
            p263.alt = (int) -1298105752;
            p263.relative_alt = (int) -1568189209;
            p263.q_SET(new float[4], 0);
            p263.image_index = (int)677143138;
            p263.capture_result = (sbyte)(sbyte) - 50;
            p263.file_url_SET("DEMO", PH);
            CommunicationChannel.instance.send(p263); //===============================
            FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.time_boot_ms = (uint)177718620U;
            p264.arming_time_utc = (ulong)1297716189538206961L;
            p264.takeoff_time_utc = (ulong)213633358730736334L;
            p264.flight_uuid = (ulong)2622955326800850740L;
            CommunicationChannel.instance.send(p264); //===============================
            MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.time_boot_ms = (uint)2670869079U;
            p265.roll = (float) -1.9347096E38F;
            p265.pitch = (float) -2.204927E38F;
            p265.yaw = (float)3.0447657E38F;
            CommunicationChannel.instance.send(p265); //===============================
            LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.target_system = (byte)(byte)179;
            p266.target_component = (byte)(byte)219;
            p266.sequence = (ushort)(ushort)41890;
            p266.length = (byte)(byte)197;
            p266.first_message_offset = (byte)(byte)184;
            p266.data__SET(new byte[249], 0);
            CommunicationChannel.instance.send(p266); //===============================
            LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.target_system = (byte)(byte)250;
            p267.target_component = (byte)(byte)243;
            p267.sequence = (ushort)(ushort)10484;
            p267.length = (byte)(byte)88;
            p267.first_message_offset = (byte)(byte)138;
            p267.data__SET(new byte[249], 0);
            CommunicationChannel.instance.send(p267); //===============================
            LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_system = (byte)(byte)114;
            p268.target_component = (byte)(byte)46;
            p268.sequence = (ushort)(ushort)31825;
            CommunicationChannel.instance.send(p268); //===============================
            VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.camera_id = (byte)(byte)186;
            p269.status = (byte)(byte)196;
            p269.framerate = (float)1.835717E38F;
            p269.resolution_h = (ushort)(ushort)11141;
            p269.resolution_v = (ushort)(ushort)28553;
            p269.bitrate = (uint)3334550818U;
            p269.rotation = (ushort)(ushort)41338;
            p269.uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p269); //===============================
            SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.target_system = (byte)(byte)195;
            p270.target_component = (byte)(byte)160;
            p270.camera_id = (byte)(byte)103;
            p270.framerate = (float) -3.3530214E38F;
            p270.resolution_h = (ushort)(ushort)46948;
            p270.resolution_v = (ushort)(ushort)63634;
            p270.bitrate = (uint)3006019165U;
            p270.rotation = (ushort)(ushort)25641;
            p270.uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p270); //===============================
            WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.ssid_SET("DEMO", PH);
            p299.password_SET("DEMO", PH);
            CommunicationChannel.instance.send(p299); //===============================
            PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.version = (ushort)(ushort)44401;
            p300.min_version = (ushort)(ushort)34849;
            p300.max_version = (ushort)(ushort)15760;
            p300.spec_version_hash_SET(new byte[8], 0);
            p300.library_version_hash_SET(new byte[8], 0);
            CommunicationChannel.instance.send(p300); //===============================
            UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.time_usec = (ulong)7827643352254320444L;
            p310.uptime_sec = (uint)1736343816U;
            p310.health = UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR;
            p310.mode = UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OPERATIONAL;
            p310.sub_mode = (byte)(byte)158;
            p310.vendor_specific_status_code = (ushort)(ushort)64983;
            CommunicationChannel.instance.send(p310); //===============================
            UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.time_usec = (ulong)2547600436549107226L;
            p311.uptime_sec = (uint)1486722591U;
            p311.name_SET("DEMO", PH);
            p311.hw_version_major = (byte)(byte)208;
            p311.hw_version_minor = (byte)(byte)145;
            p311.hw_unique_id_SET(new byte[16], 0);
            p311.sw_version_major = (byte)(byte)21;
            p311.sw_version_minor = (byte)(byte)143;
            p311.sw_vcs_commit = (uint)1593894542U;
            CommunicationChannel.instance.send(p311); //===============================
            PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.target_system = (byte)(byte)217;
            p320.target_component = (byte)(byte)211;
            p320.param_id_SET("DEMO", PH);
            p320.param_index = (short)(short) -13776;
            CommunicationChannel.instance.send(p320); //===============================
            PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_system = (byte)(byte)208;
            p321.target_component = (byte)(byte)19;
            CommunicationChannel.instance.send(p321); //===============================
            PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_id_SET("DEMO", PH);
            p322.param_value_SET("DEMO", PH);
            p322.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8;
            p322.param_count = (ushort)(ushort)52058;
            p322.param_index = (ushort)(ushort)47365;
            CommunicationChannel.instance.send(p322); //===============================
            PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.target_system = (byte)(byte)185;
            p323.target_component = (byte)(byte)34;
            p323.param_id_SET("DEMO", PH);
            p323.param_value_SET("DEMO", PH);
            p323.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64;
            CommunicationChannel.instance.send(p323); //===============================
            PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_id_SET("DEMO", PH);
            p324.param_value_SET("DEMO", PH);
            p324.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32;
            p324.param_result = PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED;
            CommunicationChannel.instance.send(p324); //===============================
            OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.time_usec = (ulong)3361262621145204767L;
            p330.sensor_type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND;
            p330.distances_SET(new ushort[72], 0);
            p330.increment = (byte)(byte)196;
            p330.min_distance = (ushort)(ushort)48742;
            p330.max_distance = (ushort)(ushort)12162;
            CommunicationChannel.instance.send(p330); //===============================
        }
    }
}


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
            POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.time_boot_ms = (uint)4083072185U;
            p3.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_ENU;
            p3.type_mask = (ushort)(ushort)39991;
            p3.x = (float)8.682369E37F;
            p3.y = (float) -2.7277474E38F;
            p3.z = (float)1.0200598E38F;
            p3.vx = (float) -1.8607141E38F;
            p3.vy = (float)2.427603E38F;
            p3.vz = (float) -1.1109698E38F;
            p3.afx = (float) -3.0984155E38F;
            p3.afy = (float)1.2503688E38F;
            p3.afz = (float)2.2496837E38F;
            p3.yaw = (float)2.1218181E38F;
            p3.yaw_rate = (float)1.5688135E38F;
            CommunicationChannel.instance.send(p3); //===============================
            ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.time_boot_ms = (uint)1741711548U;
            p83.type_mask = (byte)(byte)248;
            p83.q_SET(new float[4], 0);
            p83.body_roll_rate = (float)2.4860398E37F;
            p83.body_pitch_rate = (float)2.568673E38F;
            p83.body_yaw_rate = (float) -2.6527546E38F;
            p83.thrust = (float) -2.0648714E37F;
            CommunicationChannel.instance.send(p83); //===============================
            SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.time_boot_ms = (uint)3403329966U;
            p84.target_system = (byte)(byte)171;
            p84.target_component = (byte)(byte)82;
            p84.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p84.type_mask = (ushort)(ushort)41035;
            p84.x = (float) -2.8667137E38F;
            p84.y = (float) -9.083203E37F;
            p84.z = (float)3.026652E38F;
            p84.vx = (float)1.6868594E38F;
            p84.vy = (float) -3.1233838E38F;
            p84.vz = (float) -1.7959514E38F;
            p84.afx = (float) -1.1907377E38F;
            p84.afy = (float)2.187987E37F;
            p84.afz = (float)3.0068812E38F;
            p84.yaw = (float)9.328156E37F;
            p84.yaw_rate = (float)1.2825831E38F;
            CommunicationChannel.instance.send(p84); //===============================
            SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.time_boot_ms = (uint)2023029902U;
            p86.target_system = (byte)(byte)97;
            p86.target_component = (byte)(byte)166;
            p86.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL;
            p86.type_mask = (ushort)(ushort)23091;
            p86.lat_int = (int)880885731;
            p86.lon_int = (int)1869938284;
            p86.alt = (float)2.5090923E38F;
            p86.vx = (float)2.132132E37F;
            p86.vy = (float)2.0680034E38F;
            p86.vz = (float) -2.1752588E37F;
            p86.afx = (float)3.2560583E38F;
            p86.afy = (float) -2.416339E38F;
            p86.afz = (float)3.2685332E38F;
            p86.yaw = (float) -3.0168415E38F;
            p86.yaw_rate = (float) -1.3346969E38F;
            CommunicationChannel.instance.send(p86); //===============================
            POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.time_boot_ms = (uint)760617513U;
            p87.coordinate_frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p87.type_mask = (ushort)(ushort)41399;
            p87.lat_int = (int) -112687114;
            p87.lon_int = (int) -1809654966;
            p87.alt = (float)8.712685E37F;
            p87.vx = (float) -2.966094E38F;
            p87.vy = (float)4.0622223E37F;
            p87.vz = (float)2.2894726E37F;
            p87.afx = (float)2.4169817E38F;
            p87.afy = (float)2.2711198E38F;
            p87.afz = (float) -3.103663E37F;
            p87.yaw = (float) -2.3469342E38F;
            p87.yaw_rate = (float)2.6571384E38F;
            CommunicationChannel.instance.send(p87); //===============================
            LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.time_boot_ms = (uint)3721999899U;
            p89.x = (float) -2.8144997E38F;
            p89.y = (float)6.3121353E37F;
            p89.z = (float)3.0028E38F;
            p89.roll = (float)2.9463348E38F;
            p89.pitch = (float) -7.2722154E37F;
            p89.yaw = (float) -2.180988E38F;
            CommunicationChannel.instance.send(p89); //===============================
            HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.time_usec = (ulong)9121340723596574496L;
            p90.roll = (float) -1.989905E38F;
            p90.pitch = (float)9.055912E37F;
            p90.yaw = (float) -1.1459992E38F;
            p90.rollspeed = (float)9.306374E37F;
            p90.pitchspeed = (float) -1.6653289E38F;
            p90.yawspeed = (float) -2.644433E38F;
            p90.lat = (int) -1647318736;
            p90.lon = (int) -1976478404;
            p90.alt = (int) -29939134;
            p90.vx = (short)(short)19089;
            p90.vy = (short)(short) -30225;
            p90.vz = (short)(short) -31221;
            p90.xacc = (short)(short) -11471;
            p90.yacc = (short)(short)19465;
            p90.zacc = (short)(short) -23825;
            CommunicationChannel.instance.send(p90); //===============================
            HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.time_usec = (ulong)857215221079130425L;
            p91.roll_ailerons = (float) -1.051119E38F;
            p91.pitch_elevator = (float)2.7609501E38F;
            p91.yaw_rudder = (float) -1.5249396E38F;
            p91.throttle = (float)3.278176E38F;
            p91.aux1 = (float)3.093718E37F;
            p91.aux2 = (float) -1.4634495E38F;
            p91.aux3 = (float) -2.867397E38F;
            p91.aux4 = (float) -2.6404326E38F;
            p91.mode = (MAV_MODE)MAV_MODE.MAV_MODE_GUIDED_DISARMED;
            p91.nav_mode = (byte)(byte)154;
            CommunicationChannel.instance.send(p91); //===============================
            HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.time_usec = (ulong)5679964189682998226L;
            p92.chan1_raw = (ushort)(ushort)1508;
            p92.chan2_raw = (ushort)(ushort)45600;
            p92.chan3_raw = (ushort)(ushort)52328;
            p92.chan4_raw = (ushort)(ushort)51813;
            p92.chan5_raw = (ushort)(ushort)57802;
            p92.chan6_raw = (ushort)(ushort)7559;
            p92.chan7_raw = (ushort)(ushort)47299;
            p92.chan8_raw = (ushort)(ushort)37999;
            p92.chan9_raw = (ushort)(ushort)9194;
            p92.chan10_raw = (ushort)(ushort)82;
            p92.chan11_raw = (ushort)(ushort)30886;
            p92.chan12_raw = (ushort)(ushort)62055;
            p92.rssi = (byte)(byte)210;
            CommunicationChannel.instance.send(p92); //===============================
            HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.time_usec = (ulong)4213078939800609813L;
            p93.controls_SET(new float[16], 0);
            p93.mode = (MAV_MODE)MAV_MODE.MAV_MODE_STABILIZE_ARMED;
            p93.flags = (ulong)6936736450898561456L;
            CommunicationChannel.instance.send(p93); //===============================
            OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.time_usec = (ulong)3269186499903676250L;
            p100.sensor_id = (byte)(byte)110;
            p100.flow_x = (short)(short) -19225;
            p100.flow_y = (short)(short)16806;
            p100.flow_comp_m_x = (float)1.5205809E38F;
            p100.flow_comp_m_y = (float)1.6550765E38F;
            p100.quality = (byte)(byte)12;
            p100.ground_distance = (float) -1.6461996E36F;
            p100.flow_rate_x_SET((float) -5.3535907E37F, PH);
            p100.flow_rate_y_SET((float)3.2688212E38F, PH);
            CommunicationChannel.instance.send(p100); //===============================
            GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.usec = (ulong)9001916439420957661L;
            p101.x = (float) -7.470738E37F;
            p101.y = (float)2.7576368E38F;
            p101.z = (float) -1.683942E38F;
            p101.roll = (float) -1.0279898E38F;
            p101.pitch = (float) -2.997377E38F;
            p101.yaw = (float) -3.2618972E38F;
            CommunicationChannel.instance.send(p101); //===============================
            VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.usec = (ulong)6562323246648172358L;
            p102.x = (float)1.5378625E38F;
            p102.y = (float)3.2706067E38F;
            p102.z = (float) -1.8372545E38F;
            p102.roll = (float) -5.2925396E37F;
            p102.pitch = (float)1.6112972E38F;
            p102.yaw = (float)2.1032796E38F;
            CommunicationChannel.instance.send(p102); //===============================
            VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.usec = (ulong)2935604782289085946L;
            p103.x = (float)2.5125529E38F;
            p103.y = (float) -1.6297064E38F;
            p103.z = (float)2.1328223E38F;
            CommunicationChannel.instance.send(p103); //===============================
            VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.usec = (ulong)8878971978829929183L;
            p104.x = (float) -2.675769E38F;
            p104.y = (float)2.5433886E37F;
            p104.z = (float) -3.1445377E38F;
            p104.roll = (float) -5.631673E37F;
            p104.pitch = (float)1.0229131E38F;
            p104.yaw = (float) -2.584683E38F;
            CommunicationChannel.instance.send(p104); //===============================
            HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.time_usec = (ulong)6295243194122814493L;
            p105.xacc = (float) -1.0980349E38F;
            p105.yacc = (float) -3.1641488E38F;
            p105.zacc = (float)1.5597885E38F;
            p105.xgyro = (float) -1.524764E38F;
            p105.ygyro = (float)2.6020072E38F;
            p105.zgyro = (float)7.4471065E37F;
            p105.xmag = (float) -3.2489104E38F;
            p105.ymag = (float)8.570321E37F;
            p105.zmag = (float) -2.9845517E38F;
            p105.abs_pressure = (float)2.582958E38F;
            p105.diff_pressure = (float) -1.6841942E38F;
            p105.pressure_alt = (float) -1.9945908E38F;
            p105.temperature = (float) -6.601499E36F;
            p105.fields_updated = (ushort)(ushort)45374;
            CommunicationChannel.instance.send(p105); //===============================
            OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.time_usec = (ulong)7384751072478395061L;
            p106.sensor_id = (byte)(byte)115;
            p106.integration_time_us = (uint)4272167051U;
            p106.integrated_x = (float) -1.6219959E38F;
            p106.integrated_y = (float) -1.8729552E37F;
            p106.integrated_xgyro = (float) -2.913541E38F;
            p106.integrated_ygyro = (float)1.7786006E38F;
            p106.integrated_zgyro = (float) -1.6210523E38F;
            p106.temperature = (short)(short)14707;
            p106.quality = (byte)(byte)117;
            p106.time_delta_distance_us = (uint)1588802814U;
            p106.distance = (float) -2.9334907E38F;
            CommunicationChannel.instance.send(p106); //===============================
            HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.time_usec = (ulong)1659306086004764049L;
            p107.xacc = (float) -8.765112E37F;
            p107.yacc = (float) -2.7312443E38F;
            p107.zacc = (float)2.4428773E38F;
            p107.xgyro = (float)1.7488887E38F;
            p107.ygyro = (float)4.290385E37F;
            p107.zgyro = (float)1.5280961E38F;
            p107.xmag = (float) -1.7674923E38F;
            p107.ymag = (float) -1.7311282E38F;
            p107.zmag = (float)1.130993E38F;
            p107.abs_pressure = (float) -1.4355077E38F;
            p107.diff_pressure = (float)2.9132002E38F;
            p107.pressure_alt = (float)2.1397196E38F;
            p107.temperature = (float)2.8291037E38F;
            p107.fields_updated = (uint)1195401890U;
            CommunicationChannel.instance.send(p107); //===============================
            SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.q1 = (float)1.4584344E38F;
            p108.q2 = (float)1.8494976E38F;
            p108.q3 = (float)1.8626282E38F;
            p108.q4 = (float) -1.1569691E38F;
            p108.roll = (float) -1.9591806E38F;
            p108.pitch = (float)3.1270027E37F;
            p108.yaw = (float) -9.182315E37F;
            p108.xacc = (float)1.0073141E38F;
            p108.yacc = (float)3.3347184E38F;
            p108.zacc = (float)3.0480669E38F;
            p108.xgyro = (float) -1.3724796E38F;
            p108.ygyro = (float)3.0876571E38F;
            p108.zgyro = (float) -2.5699882E38F;
            p108.lat = (float)3.1009249E38F;
            p108.lon = (float) -3.0551584E38F;
            p108.alt = (float) -2.2670002E38F;
            p108.std_dev_horz = (float) -7.4917755E37F;
            p108.std_dev_vert = (float)2.0505267E38F;
            p108.vn = (float)3.0262919E38F;
            p108.ve = (float)1.3027364E37F;
            p108.vd = (float)2.951554E38F;
            CommunicationChannel.instance.send(p108); //===============================
            RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.rssi = (byte)(byte)160;
            p109.remrssi = (byte)(byte)151;
            p109.txbuf = (byte)(byte)116;
            p109.noise = (byte)(byte)200;
            p109.remnoise = (byte)(byte)191;
            p109.rxerrors = (ushort)(ushort)29577;
            p109.fixed_ = (ushort)(ushort)41295;
            CommunicationChannel.instance.send(p109); //===============================
            FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_network = (byte)(byte)127;
            p110.target_system = (byte)(byte)173;
            p110.target_component = (byte)(byte)207;
            p110.payload_SET(new byte[251], 0);
            CommunicationChannel.instance.send(p110); //===============================
            TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.tc1 = (long) -3265214951851145236L;
            p111.ts1 = (long) -306691037631856990L;
            CommunicationChannel.instance.send(p111); //===============================
            CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.time_usec = (ulong)7302097212938468550L;
            p112.seq = (uint)2990063173U;
            CommunicationChannel.instance.send(p112); //===============================
            HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.time_usec = (ulong)2202263701984141523L;
            p113.fix_type = (byte)(byte)66;
            p113.lat = (int)656322585;
            p113.lon = (int) -981299700;
            p113.alt = (int) -1073811776;
            p113.eph = (ushort)(ushort)22928;
            p113.epv = (ushort)(ushort)5992;
            p113.vel = (ushort)(ushort)8494;
            p113.vn = (short)(short)4643;
            p113.ve = (short)(short) -6609;
            p113.vd = (short)(short) -14920;
            p113.cog = (ushort)(ushort)48859;
            p113.satellites_visible = (byte)(byte)130;
            CommunicationChannel.instance.send(p113); //===============================
            HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.time_usec = (ulong)1883710232627981776L;
            p114.sensor_id = (byte)(byte)2;
            p114.integration_time_us = (uint)1773701545U;
            p114.integrated_x = (float)2.7682516E38F;
            p114.integrated_y = (float)1.1821623E38F;
            p114.integrated_xgyro = (float)1.6286967E38F;
            p114.integrated_ygyro = (float) -9.045095E37F;
            p114.integrated_zgyro = (float)2.7763319E38F;
            p114.temperature = (short)(short)27030;
            p114.quality = (byte)(byte)119;
            p114.time_delta_distance_us = (uint)518969861U;
            p114.distance = (float)1.1039792E38F;
            CommunicationChannel.instance.send(p114); //===============================
            HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.time_usec = (ulong)992417310270233726L;
            p115.attitude_quaternion_SET(new float[4], 0);
            p115.rollspeed = (float) -7.307015E37F;
            p115.pitchspeed = (float) -2.9503432E38F;
            p115.yawspeed = (float)2.443237E38F;
            p115.lat = (int)1376862787;
            p115.lon = (int) -1400144341;
            p115.alt = (int) -2015910722;
            p115.vx = (short)(short) -7240;
            p115.vy = (short)(short) -16051;
            p115.vz = (short)(short)2208;
            p115.ind_airspeed = (ushort)(ushort)40868;
            p115.true_airspeed = (ushort)(ushort)59193;
            p115.xacc = (short)(short)16000;
            p115.yacc = (short)(short) -30213;
            p115.zacc = (short)(short) -28252;
            CommunicationChannel.instance.send(p115); //===============================
            SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.time_boot_ms = (uint)3705086519U;
            p116.xacc = (short)(short)19054;
            p116.yacc = (short)(short) -8885;
            p116.zacc = (short)(short) -13713;
            p116.xgyro = (short)(short)32015;
            p116.ygyro = (short)(short)910;
            p116.zgyro = (short)(short) -28494;
            p116.xmag = (short)(short) -28776;
            p116.ymag = (short)(short) -27002;
            p116.zmag = (short)(short)19303;
            CommunicationChannel.instance.send(p116); //===============================
            LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.target_system = (byte)(byte)1;
            p117.target_component = (byte)(byte)135;
            p117.start = (ushort)(ushort)17784;
            p117.end = (ushort)(ushort)2782;
            CommunicationChannel.instance.send(p117); //===============================
            LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.id = (ushort)(ushort)29707;
            p118.num_logs = (ushort)(ushort)41850;
            p118.last_log_num = (ushort)(ushort)36715;
            p118.time_utc = (uint)2434336928U;
            p118.size = (uint)2455573898U;
            CommunicationChannel.instance.send(p118); //===============================
            LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.target_system = (byte)(byte)0;
            p119.target_component = (byte)(byte)135;
            p119.id = (ushort)(ushort)34853;
            p119.ofs = (uint)3668039269U;
            p119.count = (uint)606162421U;
            CommunicationChannel.instance.send(p119); //===============================
            LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.id = (ushort)(ushort)23594;
            p120.ofs = (uint)3760717284U;
            p120.count = (byte)(byte)219;
            p120.data__SET(new byte[90], 0);
            CommunicationChannel.instance.send(p120); //===============================
            LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_system = (byte)(byte)188;
            p121.target_component = (byte)(byte)75;
            CommunicationChannel.instance.send(p121); //===============================
            LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_system = (byte)(byte)40;
            p122.target_component = (byte)(byte)242;
            CommunicationChannel.instance.send(p122); //===============================
            GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.target_system = (byte)(byte)112;
            p123.target_component = (byte)(byte)104;
            p123.len = (byte)(byte)142;
            p123.data__SET(new byte[110], 0);
            CommunicationChannel.instance.send(p123); //===============================
            GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.time_usec = (ulong)5022984272100868642L;
            p124.fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED;
            p124.lat = (int) -1745356628;
            p124.lon = (int) -1153760848;
            p124.alt = (int)1946416981;
            p124.eph = (ushort)(ushort)7363;
            p124.epv = (ushort)(ushort)59430;
            p124.vel = (ushort)(ushort)20260;
            p124.cog = (ushort)(ushort)19894;
            p124.satellites_visible = (byte)(byte)186;
            p124.dgps_numch = (byte)(byte)222;
            p124.dgps_age = (uint)1113562362U;
            CommunicationChannel.instance.send(p124); //===============================
            POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vcc = (ushort)(ushort)9199;
            p125.Vservo = (ushort)(ushort)55585;
            p125.flags = (MAV_POWER_STATUS)MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT;
            CommunicationChannel.instance.send(p125); //===============================
            SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.device = (SERIAL_CONTROL_DEV)SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2;
            p126.flags = (SERIAL_CONTROL_FLAG)SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE;
            p126.timeout = (ushort)(ushort)42437;
            p126.baudrate = (uint)3073575858U;
            p126.count = (byte)(byte)151;
            p126.data__SET(new byte[70], 0);
            CommunicationChannel.instance.send(p126); //===============================
            GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.time_last_baseline_ms = (uint)2982048161U;
            p127.rtk_receiver_id = (byte)(byte)13;
            p127.wn = (ushort)(ushort)59215;
            p127.tow = (uint)3901126249U;
            p127.rtk_health = (byte)(byte)250;
            p127.rtk_rate = (byte)(byte)1;
            p127.nsats = (byte)(byte)211;
            p127.baseline_coords_type = (byte)(byte)244;
            p127.baseline_a_mm = (int)1839677434;
            p127.baseline_b_mm = (int) -1316822958;
            p127.baseline_c_mm = (int)1750654688;
            p127.accuracy = (uint)2586798302U;
            p127.iar_num_hypotheses = (int) -2029891411;
            CommunicationChannel.instance.send(p127); //===============================
            GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.time_last_baseline_ms = (uint)883316919U;
            p128.rtk_receiver_id = (byte)(byte)121;
            p128.wn = (ushort)(ushort)34810;
            p128.tow = (uint)2742904882U;
            p128.rtk_health = (byte)(byte)3;
            p128.rtk_rate = (byte)(byte)254;
            p128.nsats = (byte)(byte)155;
            p128.baseline_coords_type = (byte)(byte)107;
            p128.baseline_a_mm = (int)1954806799;
            p128.baseline_b_mm = (int) -15762953;
            p128.baseline_c_mm = (int) -51485056;
            p128.accuracy = (uint)3999248579U;
            p128.iar_num_hypotheses = (int)2116842940;
            CommunicationChannel.instance.send(p128); //===============================
            SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.time_boot_ms = (uint)1049508687U;
            p129.xacc = (short)(short)15314;
            p129.yacc = (short)(short)28077;
            p129.zacc = (short)(short)13550;
            p129.xgyro = (short)(short) -25975;
            p129.ygyro = (short)(short)21860;
            p129.zgyro = (short)(short)2166;
            p129.xmag = (short)(short) -22045;
            p129.ymag = (short)(short)2900;
            p129.zmag = (short)(short) -30742;
            CommunicationChannel.instance.send(p129); //===============================
            DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.type = (byte)(byte)201;
            p130.size = (uint)865726505U;
            p130.width = (ushort)(ushort)65303;
            p130.height = (ushort)(ushort)8368;
            p130.packets = (ushort)(ushort)21259;
            p130.payload = (byte)(byte)1;
            p130.jpg_quality = (byte)(byte)21;
            CommunicationChannel.instance.send(p130); //===============================
            ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.seqnr = (ushort)(ushort)47807;
            p131.data__SET(new byte[253], 0);
            CommunicationChannel.instance.send(p131); //===============================
            DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.time_boot_ms = (uint)1120809373U;
            p132.min_distance = (ushort)(ushort)26744;
            p132.max_distance = (ushort)(ushort)57330;
            p132.current_distance = (ushort)(ushort)56120;
            p132.type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN;
            p132.id = (byte)(byte)116;
            p132.orientation = (MAV_SENSOR_ORIENTATION)MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_YAW_225;
            p132.covariance = (byte)(byte)18;
            CommunicationChannel.instance.send(p132); //===============================
            TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.lat = (int)1025426156;
            p133.lon = (int)378199584;
            p133.grid_spacing = (ushort)(ushort)51615;
            p133.mask = (ulong)8145213311453963813L;
            CommunicationChannel.instance.send(p133); //===============================
            TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.lat = (int) -1153524938;
            p134.lon = (int)827430979;
            p134.grid_spacing = (ushort)(ushort)62896;
            p134.gridbit = (byte)(byte)223;
            p134.data__SET(new short[16], 0);
            CommunicationChannel.instance.send(p134); //===============================
            TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lat = (int)1592667974;
            p135.lon = (int) -1111021364;
            CommunicationChannel.instance.send(p135); //===============================
            TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.lat = (int)1062957302;
            p136.lon = (int)1402445607;
            p136.spacing = (ushort)(ushort)31401;
            p136.terrain_height = (float)3.0979215E38F;
            p136.current_height = (float) -1.9550503E38F;
            p136.pending = (ushort)(ushort)51795;
            p136.loaded = (ushort)(ushort)13790;
            CommunicationChannel.instance.send(p136); //===============================
            SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.time_boot_ms = (uint)2367192120U;
            p137.press_abs = (float)2.550308E38F;
            p137.press_diff = (float) -7.710463E36F;
            p137.temperature = (short)(short) -887;
            CommunicationChannel.instance.send(p137); //===============================
            ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.time_usec = (ulong)2319278448113738951L;
            p138.q_SET(new float[4], 0);
            p138.x = (float) -5.649911E37F;
            p138.y = (float) -1.3958953E37F;
            p138.z = (float)1.714439E38F;
            CommunicationChannel.instance.send(p138); //===============================
            SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.time_usec = (ulong)32648249584740549L;
            p139.group_mlx = (byte)(byte)192;
            p139.target_system = (byte)(byte)245;
            p139.target_component = (byte)(byte)222;
            p139.controls_SET(new float[8], 0);
            CommunicationChannel.instance.send(p139); //===============================
            ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.time_usec = (ulong)2351125958078592908L;
            p140.group_mlx = (byte)(byte)71;
            p140.controls_SET(new float[8], 0);
            CommunicationChannel.instance.send(p140); //===============================
            ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.time_usec = (ulong)511283696595068214L;
            p141.altitude_monotonic = (float)2.6161852E38F;
            p141.altitude_amsl = (float) -3.1210558E38F;
            p141.altitude_local = (float) -2.9270588E38F;
            p141.altitude_relative = (float)7.486872E37F;
            p141.altitude_terrain = (float)2.5635522E38F;
            p141.bottom_clearance = (float) -3.2036013E38F;
            CommunicationChannel.instance.send(p141); //===============================
            RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.request_id = (byte)(byte)20;
            p142.uri_type = (byte)(byte)147;
            p142.uri_SET(new byte[120], 0);
            p142.transfer_type = (byte)(byte)16;
            p142.storage_SET(new byte[120], 0);
            CommunicationChannel.instance.send(p142); //===============================
            SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.time_boot_ms = (uint)2406050030U;
            p143.press_abs = (float) -1.8627489E38F;
            p143.press_diff = (float)3.3123623E38F;
            p143.temperature = (short)(short) -26979;
            CommunicationChannel.instance.send(p143); //===============================
            FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.timestamp = (ulong)8719063071733359424L;
            p144.est_capabilities = (byte)(byte)135;
            p144.lat = (int) -283550354;
            p144.lon = (int) -1073138373;
            p144.alt = (float)1.0928639E38F;
            p144.vel_SET(new float[3], 0);
            p144.acc_SET(new float[3], 0);
            p144.attitude_q_SET(new float[4], 0);
            p144.rates_SET(new float[3], 0);
            p144.position_cov_SET(new float[3], 0);
            p144.custom_state = (ulong)2457044765018543981L;
            CommunicationChannel.instance.send(p144); //===============================
            CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.time_usec = (ulong)5735250274407101343L;
            p146.x_acc = (float) -2.9603566E38F;
            p146.y_acc = (float)2.7222928E38F;
            p146.z_acc = (float) -1.5924416E38F;
            p146.x_vel = (float) -3.9251723E37F;
            p146.y_vel = (float)2.5094622E38F;
            p146.z_vel = (float) -3.2930997E38F;
            p146.x_pos = (float) -1.3666094E38F;
            p146.y_pos = (float)1.2794486E38F;
            p146.z_pos = (float) -2.3535455E38F;
            p146.airspeed = (float) -2.958974E38F;
            p146.vel_variance_SET(new float[3], 0);
            p146.pos_variance_SET(new float[3], 0);
            p146.q_SET(new float[4], 0);
            p146.roll_rate = (float) -2.7907691E38F;
            p146.pitch_rate = (float)1.1209899E38F;
            p146.yaw_rate = (float)5.983652E37F;
            CommunicationChannel.instance.send(p146); //===============================
            BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.id = (byte)(byte)147;
            p147.battery_function = (MAV_BATTERY_FUNCTION)MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_PROPULSION;
            p147.type = (MAV_BATTERY_TYPE)MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIPO;
            p147.temperature = (short)(short)19808;
            p147.voltages_SET(new ushort[10], 0);
            p147.current_battery = (short)(short) -16236;
            p147.current_consumed = (int) -826925671;
            p147.energy_consumed = (int) -2104887809;
            p147.battery_remaining = (sbyte)(sbyte) - 49;
            CommunicationChannel.instance.send(p147); //===============================
            AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY)MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION;
            p148.flight_sw_version = (uint)3644875853U;
            p148.middleware_sw_version = (uint)1325473578U;
            p148.os_sw_version = (uint)4105271666U;
            p148.board_version = (uint)2531437835U;
            p148.flight_custom_version_SET(new byte[8], 0);
            p148.middleware_custom_version_SET(new byte[8], 0);
            p148.os_custom_version_SET(new byte[8], 0);
            p148.vendor_id = (ushort)(ushort)53084;
            p148.product_id = (ushort)(ushort)64762;
            p148.uid = (ulong)947732642157292507L;
            p148.uid2_SET(new byte[18], 0, PH);
            CommunicationChannel.instance.send(p148); //===============================
            LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.time_usec = (ulong)6663845470928060845L;
            p149.target_num = (byte)(byte)133;
            p149.frame = (MAV_FRAME)MAV_FRAME.MAV_FRAME_GLOBAL;
            p149.angle_x = (float) -1.3366871E38F;
            p149.angle_y = (float) -2.370807E38F;
            p149.distance = (float) -3.3658273E38F;
            p149.size_x = (float) -6.675603E37F;
            p149.size_y = (float) -1.2879297E38F;
            p149.x_SET((float)3.7848487E37F, PH);
            p149.y_SET((float) -2.1755764E38F, PH);
            p149.z_SET((float)2.6085095E38F, PH);
            p149.q_SET(new float[4], 0, PH);
            p149.type = (LANDING_TARGET_TYPE)LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_FIDUCIAL;
            p149.position_valid_SET((byte)(byte)103, PH);
            CommunicationChannel.instance.send(p149); //===============================
            CPU_LOAD p170 = CommunicationChannel.new_CPU_LOAD();
            PH.setPack(p170);
            p170.sensLoad = (byte)(byte)58;
            p170.ctrlLoad = (byte)(byte)46;
            p170.batVolt = (ushort)(ushort)29934;
            CommunicationChannel.instance.send(p170); //===============================
            SENSOR_BIAS p172 = CommunicationChannel.new_SENSOR_BIAS();
            PH.setPack(p172);
            p172.axBias = (float) -3.8765972E37F;
            p172.ayBias = (float) -3.174539E38F;
            p172.azBias = (float)2.3278472E38F;
            p172.gxBias = (float)1.0845705E38F;
            p172.gyBias = (float)1.6682005E38F;
            p172.gzBias = (float) -1.9772656E38F;
            CommunicationChannel.instance.send(p172); //===============================
            DIAGNOSTIC p173 = CommunicationChannel.new_DIAGNOSTIC();
            PH.setPack(p173);
            p173.diagFl1 = (float) -2.4319631E38F;
            p173.diagFl2 = (float) -7.466686E36F;
            p173.diagFl3 = (float) -1.8618506E38F;
            p173.diagSh1 = (short)(short) -12807;
            p173.diagSh2 = (short)(short)9241;
            p173.diagSh3 = (short)(short)24059;
            CommunicationChannel.instance.send(p173); //===============================
            SLUGS_NAVIGATION p176 = CommunicationChannel.new_SLUGS_NAVIGATION();
            PH.setPack(p176);
            p176.u_m = (float) -1.3705837E37F;
            p176.phi_c = (float)2.8535233E38F;
            p176.theta_c = (float) -8.73887E37F;
            p176.psiDot_c = (float) -2.2974567E38F;
            p176.ay_body = (float) -2.4132175E38F;
            p176.totalDist = (float) -2.2178584E38F;
            p176.dist2Go = (float) -3.3824077E37F;
            p176.fromWP = (byte)(byte)151;
            p176.toWP = (byte)(byte)73;
            p176.h_c = (ushort)(ushort)34092;
            CommunicationChannel.instance.send(p176); //===============================
            DATA_LOG p177 = CommunicationChannel.new_DATA_LOG();
            PH.setPack(p177);
            p177.fl_1 = (float) -2.2917099E38F;
            p177.fl_2 = (float)2.4844806E38F;
            p177.fl_3 = (float)3.4346696E37F;
            p177.fl_4 = (float) -1.7675692E38F;
            p177.fl_5 = (float) -3.0274583E38F;
            p177.fl_6 = (float)2.4667667E38F;
            CommunicationChannel.instance.send(p177); //===============================
            GPS_DATE_TIME p179 = CommunicationChannel.new_GPS_DATE_TIME();
            PH.setPack(p179);
            p179.year = (byte)(byte)19;
            p179.month = (byte)(byte)178;
            p179.day = (byte)(byte)98;
            p179.hour = (byte)(byte)100;
            p179.min = (byte)(byte)167;
            p179.sec = (byte)(byte)103;
            p179.clockStat = (byte)(byte)155;
            p179.visSat = (byte)(byte)180;
            p179.useSat = (byte)(byte)215;
            p179.GppGl = (byte)(byte)118;
            p179.sigUsedMask = (byte)(byte)232;
            p179.percentUsed = (byte)(byte)63;
            CommunicationChannel.instance.send(p179); //===============================
            MID_LVL_CMDS p180 = CommunicationChannel.new_MID_LVL_CMDS();
            PH.setPack(p180);
            p180.target = (byte)(byte)29;
            p180.hCommand = (float)2.1878278E38F;
            p180.uCommand = (float)2.4420035E38F;
            p180.rCommand = (float) -3.0255589E38F;
            CommunicationChannel.instance.send(p180); //===============================
            CTRL_SRFC_PT p181 = CommunicationChannel.new_CTRL_SRFC_PT();
            PH.setPack(p181);
            p181.target = (byte)(byte)127;
            p181.bitfieldPt = (ushort)(ushort)50719;
            CommunicationChannel.instance.send(p181); //===============================
            SLUGS_CAMERA_ORDER p184 = CommunicationChannel.new_SLUGS_CAMERA_ORDER();
            PH.setPack(p184);
            p184.target = (byte)(byte)10;
            p184.pan = (sbyte)(sbyte) - 81;
            p184.tilt = (sbyte)(sbyte) - 13;
            p184.zoom = (sbyte)(sbyte) - 50;
            p184.moveHome = (sbyte)(sbyte) - 46;
            CommunicationChannel.instance.send(p184); //===============================
            CONTROL_SURFACE p185 = CommunicationChannel.new_CONTROL_SURFACE();
            PH.setPack(p185);
            p185.target = (byte)(byte)64;
            p185.idSurface = (byte)(byte)6;
            p185.mControl = (float) -7.2101725E37F;
            p185.bControl = (float)3.2135578E38F;
            CommunicationChannel.instance.send(p185); //===============================
            SLUGS_MOBILE_LOCATION p186 = CommunicationChannel.new_SLUGS_MOBILE_LOCATION();
            PH.setPack(p186);
            p186.target = (byte)(byte)77;
            p186.latitude = (float)8.2879813E37F;
            p186.longitude = (float) -3.455315E37F;
            CommunicationChannel.instance.send(p186); //===============================
            SLUGS_CONFIGURATION_CAMERA p188 = CommunicationChannel.new_SLUGS_CONFIGURATION_CAMERA();
            PH.setPack(p188);
            p188.target = (byte)(byte)53;
            p188.idOrder = (byte)(byte)128;
            p188.order = (byte)(byte)229;
            CommunicationChannel.instance.send(p188); //===============================
            ISR_LOCATION p189 = CommunicationChannel.new_ISR_LOCATION();
            PH.setPack(p189);
            p189.target = (byte)(byte)154;
            p189.latitude = (float) -8.1174386E36F;
            p189.longitude = (float) -1.1654106E38F;
            p189.height = (float) -1.8674246E38F;
            p189.option1 = (byte)(byte)233;
            p189.option2 = (byte)(byte)198;
            p189.option3 = (byte)(byte)77;
            CommunicationChannel.instance.send(p189); //===============================
            VOLT_SENSOR p191 = CommunicationChannel.new_VOLT_SENSOR();
            PH.setPack(p191);
            p191.r2Type = (byte)(byte)90;
            p191.voltage = (ushort)(ushort)32808;
            p191.reading2 = (ushort)(ushort)1545;
            CommunicationChannel.instance.send(p191); //===============================
            PTZ_STATUS p192 = CommunicationChannel.new_PTZ_STATUS();
            PH.setPack(p192);
            p192.zoom = (byte)(byte)12;
            p192.pan = (short)(short) -7012;
            p192.tilt = (short)(short)13951;
            CommunicationChannel.instance.send(p192); //===============================
            UAV_STATUS p193 = CommunicationChannel.new_UAV_STATUS();
            PH.setPack(p193);
            p193.target = (byte)(byte)82;
            p193.latitude = (float)3.2060642E38F;
            p193.longitude = (float) -7.22972E37F;
            p193.altitude = (float) -1.667551E38F;
            p193.speed = (float)3.2909518E38F;
            p193.course = (float) -2.6412093E38F;
            CommunicationChannel.instance.send(p193); //===============================
            STATUS_GPS p194 = CommunicationChannel.new_STATUS_GPS();
            PH.setPack(p194);
            p194.csFails = (ushort)(ushort)14275;
            p194.gpsQuality = (byte)(byte)181;
            p194.msgsType = (byte)(byte)208;
            p194.posStatus = (byte)(byte)99;
            p194.magVar = (float) -3.3129169E37F;
            p194.magDir = (sbyte)(sbyte)104;
            p194.modeInd = (byte)(byte)237;
            CommunicationChannel.instance.send(p194); //===============================
            NOVATEL_DIAG p195 = CommunicationChannel.new_NOVATEL_DIAG();
            PH.setPack(p195);
            p195.timeStatus = (byte)(byte)172;
            p195.receiverStatus = (uint)473423688U;
            p195.solStatus = (byte)(byte)199;
            p195.posType = (byte)(byte)205;
            p195.velType = (byte)(byte)54;
            p195.posSolAge = (float) -1.087558E37F;
            p195.csFails = (ushort)(ushort)9343;
            CommunicationChannel.instance.send(p195); //===============================
            SENSOR_DIAG p196 = CommunicationChannel.new_SENSOR_DIAG();
            PH.setPack(p196);
            p196.float1 = (float) -2.3687304E37F;
            p196.float2 = (float)2.6984121E38F;
            p196.int1 = (short)(short) -29860;
            p196.char1 = (sbyte)(sbyte) - 45;
            CommunicationChannel.instance.send(p196); //===============================
            BOOT p197 = CommunicationChannel.new_BOOT();
            PH.setPack(p197);
            p197.version = (uint)3212354422U;
            CommunicationChannel.instance.send(p197); //===============================
            ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.time_usec = (ulong)5578168204114550228L;
            p230.flags = (ESTIMATOR_STATUS_FLAGS)ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT;
            p230.vel_ratio = (float) -2.2067207E38F;
            p230.pos_horiz_ratio = (float) -2.9111294E38F;
            p230.pos_vert_ratio = (float)2.2539408E38F;
            p230.mag_ratio = (float) -5.0966643E37F;
            p230.hagl_ratio = (float) -2.001315E38F;
            p230.tas_ratio = (float) -9.623585E37F;
            p230.pos_horiz_accuracy = (float)2.3779403E38F;
            p230.pos_vert_accuracy = (float)2.2847686E38F;
            CommunicationChannel.instance.send(p230); //===============================
            WIND_COV p231 = CommunicationChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.time_usec = (ulong)3502467268213649084L;
            p231.wind_x = (float) -9.830352E37F;
            p231.wind_y = (float)1.7921515E38F;
            p231.wind_z = (float) -1.7017147E38F;
            p231.var_horiz = (float) -1.1086734E38F;
            p231.var_vert = (float) -7.8111423E37F;
            p231.wind_alt = (float)2.5971435E38F;
            p231.horiz_accuracy = (float) -2.6794104E38F;
            p231.vert_accuracy = (float) -3.0541512E38F;
            CommunicationChannel.instance.send(p231); //===============================
            GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.time_usec = (ulong)625015850663531904L;
            p232.gps_id = (byte)(byte)157;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS)GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY;
            p232.time_week_ms = (uint)3217342332U;
            p232.time_week = (ushort)(ushort)64276;
            p232.fix_type = (byte)(byte)78;
            p232.lat = (int)1220834832;
            p232.lon = (int)642704332;
            p232.alt = (float)3.2675035E38F;
            p232.hdop = (float)2.2432801E38F;
            p232.vdop = (float) -3.1177325E38F;
            p232.vn = (float) -3.2955279E38F;
            p232.ve = (float) -1.810754E38F;
            p232.vd = (float) -1.756351E38F;
            p232.speed_accuracy = (float) -1.2440837E38F;
            p232.horiz_accuracy = (float) -3.1685338E38F;
            p232.vert_accuracy = (float)3.1600113E37F;
            p232.satellites_visible = (byte)(byte)47;
            CommunicationChannel.instance.send(p232); //===============================
            GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.flags = (byte)(byte)232;
            p233.len = (byte)(byte)161;
            p233.data__SET(new byte[180], 0);
            CommunicationChannel.instance.send(p233); //===============================
            HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.base_mode = (MAV_MODE_FLAG)MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
            p234.custom_mode = (uint)1771549368U;
            p234.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF;
            p234.roll = (short)(short) -16123;
            p234.pitch = (short)(short)27615;
            p234.heading = (ushort)(ushort)58545;
            p234.throttle = (sbyte)(sbyte) - 120;
            p234.heading_sp = (short)(short) -20878;
            p234.latitude = (int)821506787;
            p234.longitude = (int) -1455800495;
            p234.altitude_amsl = (short)(short)26321;
            p234.altitude_sp = (short)(short)26079;
            p234.airspeed = (byte)(byte)44;
            p234.airspeed_sp = (byte)(byte)238;
            p234.groundspeed = (byte)(byte)35;
            p234.climb_rate = (sbyte)(sbyte)43;
            p234.gps_nsat = (byte)(byte)1;
            p234.gps_fix_type = (GPS_FIX_TYPE)GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX;
            p234.battery_remaining = (byte)(byte)75;
            p234.temperature = (sbyte)(sbyte)22;
            p234.temperature_air = (sbyte)(sbyte)54;
            p234.failsafe = (byte)(byte)15;
            p234.wp_num = (byte)(byte)142;
            p234.wp_distance = (ushort)(ushort)34108;
            CommunicationChannel.instance.send(p234); //===============================
            VIBRATION p241 = CommunicationChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.time_usec = (ulong)5260364622628931822L;
            p241.vibration_x = (float)1.3232383E38F;
            p241.vibration_y = (float) -1.5023642E38F;
            p241.vibration_z = (float) -3.2963972E38F;
            p241.clipping_0 = (uint)3469748461U;
            p241.clipping_1 = (uint)3591648741U;
            p241.clipping_2 = (uint)221080886U;
            CommunicationChannel.instance.send(p241); //===============================
            HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.latitude = (int)970730105;
            p242.longitude = (int) -225475551;
            p242.altitude = (int)276332802;
            p242.x = (float)5.799102E37F;
            p242.y = (float) -1.7092378E38F;
            p242.z = (float) -9.28114E37F;
            p242.q_SET(new float[4], 0);
            p242.approach_x = (float)1.4308195E38F;
            p242.approach_y = (float)2.4109595E38F;
            p242.approach_z = (float) -1.0073565E38F;
            p242.time_usec_SET((ulong)1524485096312045867L, PH);
            CommunicationChannel.instance.send(p242); //===============================
            SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.target_system = (byte)(byte)154;
            p243.latitude = (int)1456294647;
            p243.longitude = (int)1624411268;
            p243.altitude = (int)1810120229;
            p243.x = (float) -1.3335813E38F;
            p243.y = (float) -2.3853475E38F;
            p243.z = (float)2.05946E38F;
            p243.q_SET(new float[4], 0);
            p243.approach_x = (float)5.5378067E37F;
            p243.approach_y = (float) -1.3739017E38F;
            p243.approach_z = (float)3.1702378E38F;
            p243.time_usec_SET((ulong)5960543551388628737L, PH);
            CommunicationChannel.instance.send(p243); //===============================
            MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.message_id = (ushort)(ushort)8833;
            p244.interval_us = (int) -2044880404;
            CommunicationChannel.instance.send(p244); //===============================
            EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.vtol_state = (MAV_VTOL_STATE)MAV_VTOL_STATE.MAV_VTOL_STATE_FW;
            p245.landed_state = (MAV_LANDED_STATE)MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING;
            CommunicationChannel.instance.send(p245); //===============================
            ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.ICAO_address = (uint)3583727854U;
            p246.lat = (int) -271315753;
            p246.lon = (int)1715678602;
            p246.altitude_type = (ADSB_ALTITUDE_TYPE)ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
            p246.altitude = (int) -1295511073;
            p246.heading = (ushort)(ushort)3118;
            p246.hor_velocity = (ushort)(ushort)50598;
            p246.ver_velocity = (short)(short) -31161;
            p246.callsign_SET("DEMO", PH);
            p246.emitter_type = (ADSB_EMITTER_TYPE)ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE;
            p246.tslc = (byte)(byte)165;
            p246.flags = (ADSB_FLAGS)ADSB_FLAGS.ADSB_FLAGS_SIMULATED;
            p246.squawk = (ushort)(ushort)3942;
            CommunicationChannel.instance.send(p246); //===============================
            COLLISION p247 = CommunicationChannel.new_COLLISION();
            PH.setPack(p247);
            p247.src_ = (MAV_COLLISION_SRC)MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT;
            p247.id = (uint)1559586461U;
            p247.action = (MAV_COLLISION_ACTION)MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_ASCEND_OR_DESCEND;
            p247.threat_level = (MAV_COLLISION_THREAT_LEVEL)MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE;
            p247.time_to_minimum_delta = (float) -5.1862623E37F;
            p247.altitude_minimum_delta = (float) -1.2928974E38F;
            p247.horizontal_minimum_delta = (float)1.4689473E38F;
            CommunicationChannel.instance.send(p247); //===============================
            V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.target_network = (byte)(byte)118;
            p248.target_system = (byte)(byte)57;
            p248.target_component = (byte)(byte)234;
            p248.message_type = (ushort)(ushort)1425;
            p248.payload_SET(new byte[249], 0);
            CommunicationChannel.instance.send(p248); //===============================
            MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.address = (ushort)(ushort)53215;
            p249.ver = (byte)(byte)153;
            p249.type = (byte)(byte)62;
            p249.value_SET(new sbyte[32], 0);
            CommunicationChannel.instance.send(p249); //===============================
            DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.name_SET("DEMO", PH);
            p250.time_usec = (ulong)502246898987242981L;
            p250.x = (float) -1.4770497E38F;
            p250.y = (float) -7.2655065E37F;
            p250.z = (float) -2.784231E38F;
            CommunicationChannel.instance.send(p250); //===============================
            NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.time_boot_ms = (uint)3941366039U;
            p251.name_SET("DEMO", PH);
            p251.value = (float)1.7338876E38F;
            CommunicationChannel.instance.send(p251); //===============================
            NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.time_boot_ms = (uint)3575034033U;
            p252.name_SET("DEMO", PH);
            p252.value = (int) -563285144;
            CommunicationChannel.instance.send(p252); //===============================
            STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.severity = (MAV_SEVERITY)MAV_SEVERITY.MAV_SEVERITY_WARNING;
            p253.text_SET("DEMO", PH);
            CommunicationChannel.instance.send(p253); //===============================
            DEBUG p254 = CommunicationChannel.new_DEBUG();
            PH.setPack(p254);
            p254.time_boot_ms = (uint)3652627999U;
            p254.ind = (byte)(byte)158;
            p254.value = (float) -4.5456865E37F;
            CommunicationChannel.instance.send(p254); //===============================
            SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.target_system = (byte)(byte)90;
            p256.target_component = (byte)(byte)144;
            p256.secret_key_SET(new byte[32], 0);
            p256.initial_timestamp = (ulong)2096997915305346165L;
            CommunicationChannel.instance.send(p256); //===============================
            BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.time_boot_ms = (uint)3996509797U;
            p257.last_change_ms = (uint)4221838090U;
            p257.state = (byte)(byte)48;
            CommunicationChannel.instance.send(p257); //===============================
            PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_system = (byte)(byte)65;
            p258.target_component = (byte)(byte)220;
            p258.tune_SET("DEMO", PH);
            CommunicationChannel.instance.send(p258); //===============================
            CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.time_boot_ms = (uint)82425516U;
            p259.vendor_name_SET(new byte[32], 0);
            p259.model_name_SET(new byte[32], 0);
            p259.firmware_version = (uint)4228817236U;
            p259.focal_length = (float) -1.8103413E37F;
            p259.sensor_size_h = (float)1.6616297E38F;
            p259.sensor_size_v = (float)1.6532879E38F;
            p259.resolution_h = (ushort)(ushort)20344;
            p259.resolution_v = (ushort)(ushort)21282;
            p259.lens_id = (byte)(byte)213;
            p259.flags = (CAMERA_CAP_FLAGS)CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE;
            p259.cam_definition_version = (ushort)(ushort)53233;
            p259.cam_definition_uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p259); //===============================
            CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.time_boot_ms = (uint)4285458981U;
            p260.mode_id = (CAMERA_MODE)CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY;
            CommunicationChannel.instance.send(p260); //===============================
            STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.time_boot_ms = (uint)1687684591U;
            p261.storage_id = (byte)(byte)97;
            p261.storage_count = (byte)(byte)133;
            p261.status = (byte)(byte)40;
            p261.total_capacity = (float)2.842317E38F;
            p261.used_capacity = (float)1.4798386E38F;
            p261.available_capacity = (float)8.1394146E37F;
            p261.read_speed = (float) -1.5277585E38F;
            p261.write_speed = (float) -2.7724746E38F;
            CommunicationChannel.instance.send(p261); //===============================
            CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.time_boot_ms = (uint)1583492410U;
            p262.image_status = (byte)(byte)210;
            p262.video_status = (byte)(byte)217;
            p262.image_interval = (float)3.041881E36F;
            p262.recording_time_ms = (uint)2150309069U;
            p262.available_capacity = (float)2.6972225E37F;
            CommunicationChannel.instance.send(p262); //===============================
            CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.time_boot_ms = (uint)1211547990U;
            p263.time_utc = (ulong)8397818773625309942L;
            p263.camera_id = (byte)(byte)100;
            p263.lat = (int)1193944851;
            p263.lon = (int) -234122477;
            p263.alt = (int)1904435287;
            p263.relative_alt = (int) -1494030804;
            p263.q_SET(new float[4], 0);
            p263.image_index = (int) -1526331791;
            p263.capture_result = (sbyte)(sbyte)81;
            p263.file_url_SET("DEMO", PH);
            CommunicationChannel.instance.send(p263); //===============================
            FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.time_boot_ms = (uint)78663566U;
            p264.arming_time_utc = (ulong)3504533569272113013L;
            p264.takeoff_time_utc = (ulong)504617048324091185L;
            p264.flight_uuid = (ulong)3008936246192599215L;
            CommunicationChannel.instance.send(p264); //===============================
            MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.time_boot_ms = (uint)706192017U;
            p265.roll = (float)3.2338712E38F;
            p265.pitch = (float)2.9053586E38F;
            p265.yaw = (float) -2.2341243E38F;
            CommunicationChannel.instance.send(p265); //===============================
            LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.target_system = (byte)(byte)204;
            p266.target_component = (byte)(byte)225;
            p266.sequence = (ushort)(ushort)1957;
            p266.length = (byte)(byte)148;
            p266.first_message_offset = (byte)(byte)43;
            p266.data__SET(new byte[249], 0);
            CommunicationChannel.instance.send(p266); //===============================
            LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.target_system = (byte)(byte)176;
            p267.target_component = (byte)(byte)73;
            p267.sequence = (ushort)(ushort)34310;
            p267.length = (byte)(byte)77;
            p267.first_message_offset = (byte)(byte)59;
            p267.data__SET(new byte[249], 0);
            CommunicationChannel.instance.send(p267); //===============================
            LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_system = (byte)(byte)138;
            p268.target_component = (byte)(byte)48;
            p268.sequence = (ushort)(ushort)24141;
            CommunicationChannel.instance.send(p268); //===============================
            VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.camera_id = (byte)(byte)230;
            p269.status = (byte)(byte)176;
            p269.framerate = (float)1.6120449E38F;
            p269.resolution_h = (ushort)(ushort)51377;
            p269.resolution_v = (ushort)(ushort)39858;
            p269.bitrate = (uint)2585571050U;
            p269.rotation = (ushort)(ushort)31468;
            p269.uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p269); //===============================
            SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.target_system = (byte)(byte)169;
            p270.target_component = (byte)(byte)95;
            p270.camera_id = (byte)(byte)177;
            p270.framerate = (float)6.3508986E37F;
            p270.resolution_h = (ushort)(ushort)48893;
            p270.resolution_v = (ushort)(ushort)34841;
            p270.bitrate = (uint)3341109894U;
            p270.rotation = (ushort)(ushort)5930;
            p270.uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p270); //===============================
            WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.ssid_SET("DEMO", PH);
            p299.password_SET("DEMO", PH);
            CommunicationChannel.instance.send(p299); //===============================
            PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.version = (ushort)(ushort)29588;
            p300.min_version = (ushort)(ushort)7405;
            p300.max_version = (ushort)(ushort)18713;
            p300.spec_version_hash_SET(new byte[8], 0);
            p300.library_version_hash_SET(new byte[8], 0);
            CommunicationChannel.instance.send(p300); //===============================
            UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.time_usec = (ulong)2286933712063468746L;
            p310.uptime_sec = (uint)2345401238U;
            p310.health = (UAVCAN_NODE_HEALTH)UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR;
            p310.mode = (UAVCAN_NODE_MODE)UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE;
            p310.sub_mode = (byte)(byte)28;
            p310.vendor_specific_status_code = (ushort)(ushort)43369;
            CommunicationChannel.instance.send(p310); //===============================
            UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.time_usec = (ulong)3684189897441424737L;
            p311.uptime_sec = (uint)2787271334U;
            p311.name_SET("DEMO", PH);
            p311.hw_version_major = (byte)(byte)164;
            p311.hw_version_minor = (byte)(byte)85;
            p311.hw_unique_id_SET(new byte[16], 0);
            p311.sw_version_major = (byte)(byte)166;
            p311.sw_version_minor = (byte)(byte)84;
            p311.sw_vcs_commit = (uint)2996819881U;
            CommunicationChannel.instance.send(p311); //===============================
            PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.target_system = (byte)(byte)26;
            p320.target_component = (byte)(byte)159;
            p320.param_id_SET("DEMO", PH);
            p320.param_index = (short)(short)26610;
            CommunicationChannel.instance.send(p320); //===============================
            PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_system = (byte)(byte)100;
            p321.target_component = (byte)(byte)1;
            CommunicationChannel.instance.send(p321); //===============================
            PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_id_SET("DEMO", PH);
            p322.param_value_SET("DEMO", PH);
            p322.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64;
            p322.param_count = (ushort)(ushort)54440;
            p322.param_index = (ushort)(ushort)32371;
            CommunicationChannel.instance.send(p322); //===============================
            PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.target_system = (byte)(byte)165;
            p323.target_component = (byte)(byte)73;
            p323.param_id_SET("DEMO", PH);
            p323.param_value_SET("DEMO", PH);
            p323.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32;
            CommunicationChannel.instance.send(p323); //===============================
            PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_id_SET("DEMO", PH);
            p324.param_value_SET("DEMO", PH);
            p324.param_type = (MAV_PARAM_EXT_TYPE)MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM;
            p324.param_result = (PARAM_ACK)PARAM_ACK.PARAM_ACK_FAILED;
            CommunicationChannel.instance.send(p324); //===============================
            OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.time_usec = (ulong)1409032714567247565L;
            p330.sensor_type = (MAV_DISTANCE_SENSOR)MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER;
            p330.distances_SET(new ushort[72], 0);
            p330.increment = (byte)(byte)244;
            p330.min_distance = (ushort)(ushort)26137;
            p330.max_distance = (ushort)(ushort)38237;
            CommunicationChannel.instance.send(p330); //===============================
        }
    }
}


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
            POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.time_boot_ms = (uint)4243146465U;
            p3.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_INT;
            p3.type_mask = (ushort)(ushort)20403;
            p3.x = (float) -2.2027453E38F;
            p3.y = (float)1.6964395E38F;
            p3.z = (float)3.2690474E38F;
            p3.vx = (float) -1.1805975E38F;
            p3.vy = (float)2.4843224E38F;
            p3.vz = (float)2.1961617E38F;
            p3.afx = (float) -1.0731334E38F;
            p3.afy = (float) -4.901269E37F;
            p3.afz = (float)3.4587842E37F;
            p3.yaw = (float) -3.2693455E38F;
            p3.yaw_rate = (float) -1.1011723E38F;
            CommunicationChannel.instance.send(p3); //===============================
            COMMAND_INT p75 = CommunicationChannel.new_COMMAND_INT();
            PH.setPack(p75);
            p75.target_system = (byte)(byte)72;
            p75.target_component = (byte)(byte)52;
            p75.frame = MAV_FRAME.MAV_FRAME_MISSION;
            p75.command = MAV_CMD.MAV_CMD_DO_SET_MODE;
            p75.current = (byte)(byte)115;
            p75.autocontinue = (byte)(byte)252;
            p75.param1 = (float) -1.349579E38F;
            p75.param2 = (float)2.4627748E38F;
            p75.param3 = (float) -1.9719007E38F;
            p75.param4 = (float)3.8418834E37F;
            p75.x = (int)1839175094;
            p75.y = (int) -244965175;
            p75.z = (float) -1.3319898E38F;
            CommunicationChannel.instance.send(p75); //===============================
            COMMAND_LONG p76 = CommunicationChannel.new_COMMAND_LONG();
            PH.setPack(p76);
            p76.target_system = (byte)(byte)254;
            p76.target_component = (byte)(byte)75;
            p76.command = MAV_CMD.MAV_CMD_REQUEST_CAMERA_INFORMATION;
            p76.confirmation = (byte)(byte)240;
            p76.param1 = (float)2.7541E38F;
            p76.param2 = (float) -6.055791E37F;
            p76.param3 = (float) -1.4154633E38F;
            p76.param4 = (float) -1.5170139E38F;
            p76.param5 = (float) -3.5728798E37F;
            p76.param6 = (float)2.2722998E38F;
            p76.param7 = (float)1.2120871E38F;
            CommunicationChannel.instance.send(p76); //===============================
            COMMAND_ACK p77 = CommunicationChannel.new_COMMAND_ACK();
            PH.setPack(p77);
            p77.command = MAV_CMD.MAV_CMD_NAV_LOITER_TO_ALT;
            p77.result = MAV_RESULT.MAV_RESULT_DENIED;
            p77.progress_SET((byte)(byte)253, PH);
            p77.result_param2_SET((int) -1582299095, PH);
            p77.target_system_SET((byte)(byte)213, PH);
            p77.target_component_SET((byte)(byte)82, PH);
            CommunicationChannel.instance.send(p77); //===============================
            MANUAL_SETPOINT p81 = CommunicationChannel.new_MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.time_boot_ms = (uint)3414682556U;
            p81.roll = (float) -9.08863E37F;
            p81.pitch = (float)1.7432814E38F;
            p81.yaw = (float) -1.2162298E38F;
            p81.thrust = (float)3.2889854E36F;
            p81.mode_switch = (byte)(byte)22;
            p81.manual_override_switch = (byte)(byte)11;
            CommunicationChannel.instance.send(p81); //===============================
            SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.time_boot_ms = (uint)1944056591U;
            p82.target_system = (byte)(byte)0;
            p82.target_component = (byte)(byte)192;
            p82.type_mask = (byte)(byte)210;
            p82.q_SET(new float[4], 0);
            p82.body_roll_rate = (float)1.4131304E38F;
            p82.body_pitch_rate = (float)3.106599E37F;
            p82.body_yaw_rate = (float) -2.0275899E38F;
            p82.thrust = (float) -7.452358E37F;
            CommunicationChannel.instance.send(p82); //===============================
            ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.time_boot_ms = (uint)3789090348U;
            p83.type_mask = (byte)(byte)65;
            p83.q_SET(new float[4], 0);
            p83.body_roll_rate = (float)4.050408E36F;
            p83.body_pitch_rate = (float)2.1560719E37F;
            p83.body_yaw_rate = (float)5.3701716E37F;
            p83.thrust = (float) -2.3041767E38F;
            CommunicationChannel.instance.send(p83); //===============================
            SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.time_boot_ms = (uint)1026892971U;
            p84.target_system = (byte)(byte)246;
            p84.target_component = (byte)(byte)213;
            p84.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
            p84.type_mask = (ushort)(ushort)24080;
            p84.x = (float) -3.1021114E37F;
            p84.y = (float) -1.1591566E37F;
            p84.z = (float)1.2615141E38F;
            p84.vx = (float) -2.035874E38F;
            p84.vy = (float)1.7764718E38F;
            p84.vz = (float) -4.011571E37F;
            p84.afx = (float)7.456414E37F;
            p84.afy = (float) -8.644232E37F;
            p84.afz = (float) -3.1993163E38F;
            p84.yaw = (float)2.3773864E38F;
            p84.yaw_rate = (float) -4.533497E37F;
            CommunicationChannel.instance.send(p84); //===============================
            SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.time_boot_ms = (uint)726106832U;
            p86.target_system = (byte)(byte)86;
            p86.target_component = (byte)(byte)27;
            p86.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
            p86.type_mask = (ushort)(ushort)9751;
            p86.lat_int = (int)1438936325;
            p86.lon_int = (int) -2013568188;
            p86.alt = (float) -3.2971608E38F;
            p86.vx = (float) -3.2003724E38F;
            p86.vy = (float) -1.6833861E38F;
            p86.vz = (float)1.5897255E38F;
            p86.afx = (float)2.4441744E38F;
            p86.afy = (float) -2.7281066E38F;
            p86.afz = (float)3.1502955E38F;
            p86.yaw = (float) -5.3264533E37F;
            p86.yaw_rate = (float) -3.3194165E38F;
            CommunicationChannel.instance.send(p86); //===============================
            POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.time_boot_ms = (uint)1991593896U;
            p87.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL;
            p87.type_mask = (ushort)(ushort)11328;
            p87.lat_int = (int)540483721;
            p87.lon_int = (int)2098386308;
            p87.alt = (float)2.8761793E38F;
            p87.vx = (float) -3.0291817E38F;
            p87.vy = (float) -2.9236842E38F;
            p87.vz = (float)2.0382956E38F;
            p87.afx = (float) -2.757849E38F;
            p87.afy = (float)2.9596676E38F;
            p87.afz = (float)2.1977903E38F;
            p87.yaw = (float) -1.9982374E38F;
            p87.yaw_rate = (float)1.2001142E38F;
            CommunicationChannel.instance.send(p87); //===============================
            LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.time_boot_ms = (uint)3106372440U;
            p89.x = (float) -5.438385E36F;
            p89.y = (float)8.327546E37F;
            p89.z = (float)2.0138954E38F;
            p89.roll = (float)1.8039258E38F;
            p89.pitch = (float) -7.846667E36F;
            p89.yaw = (float)4.2095239E37F;
            CommunicationChannel.instance.send(p89); //===============================
            HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.time_usec = (ulong)628184935491661521L;
            p90.roll = (float) -7.2164276E37F;
            p90.pitch = (float)2.4867E38F;
            p90.yaw = (float) -2.363759E38F;
            p90.rollspeed = (float) -2.924368E38F;
            p90.pitchspeed = (float) -2.4605725E38F;
            p90.yawspeed = (float)1.1013888E38F;
            p90.lat = (int)1570478447;
            p90.lon = (int)459082256;
            p90.alt = (int)1995965106;
            p90.vx = (short)(short)19008;
            p90.vy = (short)(short)6530;
            p90.vz = (short)(short) -10321;
            p90.xacc = (short)(short) -21726;
            p90.yacc = (short)(short) -29835;
            p90.zacc = (short)(short) -30684;
            CommunicationChannel.instance.send(p90); //===============================
            HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.time_usec = (ulong)6307286890149864305L;
            p91.roll_ailerons = (float)2.3979053E38F;
            p91.pitch_elevator = (float)3.0627822E38F;
            p91.yaw_rudder = (float)5.438264E37F;
            p91.throttle = (float)9.445136E37F;
            p91.aux1 = (float)7.8654734E36F;
            p91.aux2 = (float)1.8375198E38F;
            p91.aux3 = (float)1.3551797E38F;
            p91.aux4 = (float)1.3584886E38F;
            p91.mode = MAV_MODE.MAV_MODE_GUIDED_ARMED;
            p91.nav_mode = (byte)(byte)29;
            CommunicationChannel.instance.send(p91); //===============================
            HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.time_usec = (ulong)1255181073265656749L;
            p92.chan1_raw = (ushort)(ushort)53071;
            p92.chan2_raw = (ushort)(ushort)1089;
            p92.chan3_raw = (ushort)(ushort)39261;
            p92.chan4_raw = (ushort)(ushort)46077;
            p92.chan5_raw = (ushort)(ushort)54271;
            p92.chan6_raw = (ushort)(ushort)14290;
            p92.chan7_raw = (ushort)(ushort)59015;
            p92.chan8_raw = (ushort)(ushort)63724;
            p92.chan9_raw = (ushort)(ushort)57024;
            p92.chan10_raw = (ushort)(ushort)51813;
            p92.chan11_raw = (ushort)(ushort)28100;
            p92.chan12_raw = (ushort)(ushort)20101;
            p92.rssi = (byte)(byte)234;
            CommunicationChannel.instance.send(p92); //===============================
            HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.time_usec = (ulong)2942488205805084550L;
            p93.controls_SET(new float[16], 0);
            p93.mode = MAV_MODE.MAV_MODE_MANUAL_ARMED;
            p93.flags = (ulong)4061296870360679478L;
            CommunicationChannel.instance.send(p93); //===============================
            OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.time_usec = (ulong)6158285589656627691L;
            p100.sensor_id = (byte)(byte)47;
            p100.flow_x = (short)(short)18002;
            p100.flow_y = (short)(short)12866;
            p100.flow_comp_m_x = (float)3.0488508E38F;
            p100.flow_comp_m_y = (float)3.2652844E38F;
            p100.quality = (byte)(byte)161;
            p100.ground_distance = (float) -3.2240768E38F;
            p100.flow_rate_x_SET((float)2.5538416E38F, PH);
            p100.flow_rate_y_SET((float) -2.7149782E38F, PH);
            CommunicationChannel.instance.send(p100); //===============================
            GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.usec = (ulong)2870969727514852206L;
            p101.x = (float) -8.741766E37F;
            p101.y = (float) -3.4428369E37F;
            p101.z = (float)8.605066E37F;
            p101.roll = (float)2.9495856E37F;
            p101.pitch = (float)2.3014333E38F;
            p101.yaw = (float)3.8283132E37F;
            CommunicationChannel.instance.send(p101); //===============================
            VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.usec = (ulong)39226672744937565L;
            p102.x = (float) -2.1641361E37F;
            p102.y = (float)2.4914573E38F;
            p102.z = (float) -2.5490367E38F;
            p102.roll = (float)5.8167E37F;
            p102.pitch = (float)3.124098E38F;
            p102.yaw = (float)1.1815064E38F;
            CommunicationChannel.instance.send(p102); //===============================
            VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.usec = (ulong)7421769611358802819L;
            p103.x = (float)1.1038236E38F;
            p103.y = (float) -4.157773E37F;
            p103.z = (float)2.6330135E38F;
            CommunicationChannel.instance.send(p103); //===============================
            VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.usec = (ulong)2393435320257968416L;
            p104.x = (float) -1.2524334E38F;
            p104.y = (float) -2.6448124E38F;
            p104.z = (float) -2.9533336E38F;
            p104.roll = (float)3.2345726E38F;
            p104.pitch = (float) -3.3473383E38F;
            p104.yaw = (float)3.3910092E38F;
            CommunicationChannel.instance.send(p104); //===============================
            HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.time_usec = (ulong)3070332755417554563L;
            p105.xacc = (float)1.5485777E36F;
            p105.yacc = (float)2.6021725E38F;
            p105.zacc = (float) -1.6791485E38F;
            p105.xgyro = (float)2.2019653E38F;
            p105.ygyro = (float) -2.8322426E38F;
            p105.zgyro = (float) -2.6476528E38F;
            p105.xmag = (float) -9.69329E36F;
            p105.ymag = (float)2.918985E38F;
            p105.zmag = (float)2.9656163E38F;
            p105.abs_pressure = (float)2.1832065E38F;
            p105.diff_pressure = (float) -1.0868406E38F;
            p105.pressure_alt = (float) -4.0103866E37F;
            p105.temperature = (float) -1.7036605E38F;
            p105.fields_updated = (ushort)(ushort)13257;
            CommunicationChannel.instance.send(p105); //===============================
            OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.time_usec = (ulong)3610614216914153200L;
            p106.sensor_id = (byte)(byte)122;
            p106.integration_time_us = (uint)2015989390U;
            p106.integrated_x = (float) -1.2292386E38F;
            p106.integrated_y = (float) -1.3775085E38F;
            p106.integrated_xgyro = (float)1.5321458E38F;
            p106.integrated_ygyro = (float)3.1482105E38F;
            p106.integrated_zgyro = (float)2.2990633E38F;
            p106.temperature = (short)(short)2691;
            p106.quality = (byte)(byte)58;
            p106.time_delta_distance_us = (uint)34517998U;
            p106.distance = (float) -8.571201E36F;
            CommunicationChannel.instance.send(p106); //===============================
            HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.time_usec = (ulong)5823156549609264265L;
            p107.xacc = (float)9.489426E37F;
            p107.yacc = (float) -1.8521261E37F;
            p107.zacc = (float)3.2937009E38F;
            p107.xgyro = (float) -1.9814075E38F;
            p107.ygyro = (float)3.2052576E38F;
            p107.zgyro = (float) -1.9580985E38F;
            p107.xmag = (float) -5.924902E37F;
            p107.ymag = (float) -1.286155E37F;
            p107.zmag = (float)4.795754E37F;
            p107.abs_pressure = (float) -1.3642322E38F;
            p107.diff_pressure = (float)2.805982E38F;
            p107.pressure_alt = (float)5.4834394E36F;
            p107.temperature = (float) -1.9414262E38F;
            p107.fields_updated = (uint)1655773263U;
            CommunicationChannel.instance.send(p107); //===============================
            SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.q1 = (float)5.109362E37F;
            p108.q2 = (float)2.2017983E36F;
            p108.q3 = (float)2.492441E38F;
            p108.q4 = (float) -9.8552E37F;
            p108.roll = (float) -1.4584639E38F;
            p108.pitch = (float)2.1283582E38F;
            p108.yaw = (float)2.9944416E38F;
            p108.xacc = (float)1.8688042E38F;
            p108.yacc = (float) -2.7686625E38F;
            p108.zacc = (float)1.1168508E37F;
            p108.xgyro = (float) -4.0997768E37F;
            p108.ygyro = (float) -1.04092096E37F;
            p108.zgyro = (float)2.9044763E38F;
            p108.lat = (float) -3.1101343E38F;
            p108.lon = (float) -2.6660793E38F;
            p108.alt = (float)1.1036166E38F;
            p108.std_dev_horz = (float) -4.2769264E35F;
            p108.std_dev_vert = (float) -2.4456258E38F;
            p108.vn = (float) -1.0746775E38F;
            p108.ve = (float) -2.5605068E38F;
            p108.vd = (float) -9.556294E37F;
            CommunicationChannel.instance.send(p108); //===============================
            RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.rssi = (byte)(byte)174;
            p109.remrssi = (byte)(byte)79;
            p109.txbuf = (byte)(byte)127;
            p109.noise = (byte)(byte)193;
            p109.remnoise = (byte)(byte)118;
            p109.rxerrors = (ushort)(ushort)35977;
            p109.fixed_ = (ushort)(ushort)737;
            CommunicationChannel.instance.send(p109); //===============================
            FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_network = (byte)(byte)210;
            p110.target_system = (byte)(byte)181;
            p110.target_component = (byte)(byte)16;
            p110.payload_SET(new byte[251], 0);
            CommunicationChannel.instance.send(p110); //===============================
            TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.tc1 = (long) -343978419073866428L;
            p111.ts1 = (long)7491397853656152659L;
            CommunicationChannel.instance.send(p111); //===============================
            CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.time_usec = (ulong)8259829458685364368L;
            p112.seq = (uint)144469890U;
            CommunicationChannel.instance.send(p112); //===============================
            HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.time_usec = (ulong)972834647252162879L;
            p113.fix_type = (byte)(byte)219;
            p113.lat = (int) -748171842;
            p113.lon = (int)1170335384;
            p113.alt = (int)2021828924;
            p113.eph = (ushort)(ushort)19258;
            p113.epv = (ushort)(ushort)2765;
            p113.vel = (ushort)(ushort)9502;
            p113.vn = (short)(short)18754;
            p113.ve = (short)(short) -17438;
            p113.vd = (short)(short)17566;
            p113.cog = (ushort)(ushort)1566;
            p113.satellites_visible = (byte)(byte)152;
            CommunicationChannel.instance.send(p113); //===============================
            HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.time_usec = (ulong)1687823221077746936L;
            p114.sensor_id = (byte)(byte)0;
            p114.integration_time_us = (uint)1657218891U;
            p114.integrated_x = (float) -2.2311446E38F;
            p114.integrated_y = (float)2.4535045E37F;
            p114.integrated_xgyro = (float)5.09878E37F;
            p114.integrated_ygyro = (float)2.980405E38F;
            p114.integrated_zgyro = (float)1.3793874E38F;
            p114.temperature = (short)(short) -24736;
            p114.quality = (byte)(byte)38;
            p114.time_delta_distance_us = (uint)1107382380U;
            p114.distance = (float) -4.5832947E37F;
            CommunicationChannel.instance.send(p114); //===============================
            HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.time_usec = (ulong)3818531707596820944L;
            p115.attitude_quaternion_SET(new float[4], 0);
            p115.rollspeed = (float)7.732961E37F;
            p115.pitchspeed = (float) -1.1462841E38F;
            p115.yawspeed = (float) -2.5332415E38F;
            p115.lat = (int)837617543;
            p115.lon = (int) -507566914;
            p115.alt = (int) -113070842;
            p115.vx = (short)(short) -20144;
            p115.vy = (short)(short)7039;
            p115.vz = (short)(short) -14344;
            p115.ind_airspeed = (ushort)(ushort)42102;
            p115.true_airspeed = (ushort)(ushort)16281;
            p115.xacc = (short)(short) -1300;
            p115.yacc = (short)(short) -10967;
            p115.zacc = (short)(short) -10511;
            CommunicationChannel.instance.send(p115); //===============================
            SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.time_boot_ms = (uint)2064288137U;
            p116.xacc = (short)(short) -2230;
            p116.yacc = (short)(short)25720;
            p116.zacc = (short)(short)31787;
            p116.xgyro = (short)(short)28087;
            p116.ygyro = (short)(short)24184;
            p116.zgyro = (short)(short) -19871;
            p116.xmag = (short)(short)18359;
            p116.ymag = (short)(short) -12837;
            p116.zmag = (short)(short)14038;
            CommunicationChannel.instance.send(p116); //===============================
            LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.target_system = (byte)(byte)14;
            p117.target_component = (byte)(byte)198;
            p117.start = (ushort)(ushort)11909;
            p117.end = (ushort)(ushort)45870;
            CommunicationChannel.instance.send(p117); //===============================
            LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.id = (ushort)(ushort)17422;
            p118.num_logs = (ushort)(ushort)4337;
            p118.last_log_num = (ushort)(ushort)22572;
            p118.time_utc = (uint)696252037U;
            p118.size = (uint)241268624U;
            CommunicationChannel.instance.send(p118); //===============================
            LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.target_system = (byte)(byte)175;
            p119.target_component = (byte)(byte)191;
            p119.id = (ushort)(ushort)54957;
            p119.ofs = (uint)2071196846U;
            p119.count = (uint)2162952501U;
            CommunicationChannel.instance.send(p119); //===============================
            LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.id = (ushort)(ushort)33061;
            p120.ofs = (uint)1250008656U;
            p120.count = (byte)(byte)206;
            p120.data__SET(new byte[90], 0);
            CommunicationChannel.instance.send(p120); //===============================
            LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_system = (byte)(byte)40;
            p121.target_component = (byte)(byte)16;
            CommunicationChannel.instance.send(p121); //===============================
            LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_system = (byte)(byte)222;
            p122.target_component = (byte)(byte)202;
            CommunicationChannel.instance.send(p122); //===============================
            GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.target_system = (byte)(byte)171;
            p123.target_component = (byte)(byte)124;
            p123.len = (byte)(byte)6;
            p123.data__SET(new byte[110], 0);
            CommunicationChannel.instance.send(p123); //===============================
            GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.time_usec = (ulong)122965501906722932L;
            p124.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS;
            p124.lat = (int) -672733553;
            p124.lon = (int)30335277;
            p124.alt = (int)127094316;
            p124.eph = (ushort)(ushort)5228;
            p124.epv = (ushort)(ushort)31470;
            p124.vel = (ushort)(ushort)11414;
            p124.cog = (ushort)(ushort)2471;
            p124.satellites_visible = (byte)(byte)206;
            p124.dgps_numch = (byte)(byte)139;
            p124.dgps_age = (uint)824034376U;
            CommunicationChannel.instance.send(p124); //===============================
            POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vcc = (ushort)(ushort)51164;
            p125.Vservo = (ushort)(ushort)38876;
            p125.flags = (MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT);
            CommunicationChannel.instance.send(p125); //===============================
            SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.device = SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1;
            p126.flags = (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND);
            p126.timeout = (ushort)(ushort)45125;
            p126.baudrate = (uint)3216136942U;
            p126.count = (byte)(byte)84;
            p126.data__SET(new byte[70], 0);
            CommunicationChannel.instance.send(p126); //===============================
            GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.time_last_baseline_ms = (uint)638880209U;
            p127.rtk_receiver_id = (byte)(byte)212;
            p127.wn = (ushort)(ushort)65302;
            p127.tow = (uint)526534550U;
            p127.rtk_health = (byte)(byte)185;
            p127.rtk_rate = (byte)(byte)245;
            p127.nsats = (byte)(byte)171;
            p127.baseline_coords_type = (byte)(byte)206;
            p127.baseline_a_mm = (int)1090360607;
            p127.baseline_b_mm = (int) -354019069;
            p127.baseline_c_mm = (int) -1418989216;
            p127.accuracy = (uint)2987658386U;
            p127.iar_num_hypotheses = (int)1111112055;
            CommunicationChannel.instance.send(p127); //===============================
            GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.time_last_baseline_ms = (uint)782980009U;
            p128.rtk_receiver_id = (byte)(byte)208;
            p128.wn = (ushort)(ushort)4694;
            p128.tow = (uint)3690493050U;
            p128.rtk_health = (byte)(byte)83;
            p128.rtk_rate = (byte)(byte)26;
            p128.nsats = (byte)(byte)212;
            p128.baseline_coords_type = (byte)(byte)178;
            p128.baseline_a_mm = (int)336851009;
            p128.baseline_b_mm = (int) -1487244196;
            p128.baseline_c_mm = (int)814443193;
            p128.accuracy = (uint)2205967012U;
            p128.iar_num_hypotheses = (int)1853450565;
            CommunicationChannel.instance.send(p128); //===============================
            SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.time_boot_ms = (uint)4182552318U;
            p129.xacc = (short)(short) -15523;
            p129.yacc = (short)(short)26817;
            p129.zacc = (short)(short) -23212;
            p129.xgyro = (short)(short) -12055;
            p129.ygyro = (short)(short)6599;
            p129.zgyro = (short)(short)14827;
            p129.xmag = (short)(short) -17068;
            p129.ymag = (short)(short) -6962;
            p129.zmag = (short)(short)22029;
            CommunicationChannel.instance.send(p129); //===============================
            DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.type = (byte)(byte)146;
            p130.size = (uint)3281151530U;
            p130.width = (ushort)(ushort)59510;
            p130.height = (ushort)(ushort)5800;
            p130.packets = (ushort)(ushort)63117;
            p130.payload = (byte)(byte)71;
            p130.jpg_quality = (byte)(byte)222;
            CommunicationChannel.instance.send(p130); //===============================
            ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.seqnr = (ushort)(ushort)61975;
            p131.data__SET(new byte[253], 0);
            CommunicationChannel.instance.send(p131); //===============================
            DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.time_boot_ms = (uint)2566870699U;
            p132.min_distance = (ushort)(ushort)10159;
            p132.max_distance = (ushort)(ushort)2831;
            p132.current_distance = (ushort)(ushort)63670;
            p132.type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR;
            p132.id = (byte)(byte)53;
            p132.orientation = MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_270_PITCH_180;
            p132.covariance = (byte)(byte)153;
            CommunicationChannel.instance.send(p132); //===============================
            TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.lat = (int)2036569233;
            p133.lon = (int) -1996398335;
            p133.grid_spacing = (ushort)(ushort)27719;
            p133.mask = (ulong)604549224425373099L;
            CommunicationChannel.instance.send(p133); //===============================
            TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.lat = (int)1918553647;
            p134.lon = (int) -599756878;
            p134.grid_spacing = (ushort)(ushort)16069;
            p134.gridbit = (byte)(byte)50;
            p134.data__SET(new short[16], 0);
            CommunicationChannel.instance.send(p134); //===============================
            TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lat = (int) -626702273;
            p135.lon = (int) -1125613496;
            CommunicationChannel.instance.send(p135); //===============================
            TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.lat = (int) -430417777;
            p136.lon = (int)1585110652;
            p136.spacing = (ushort)(ushort)8015;
            p136.terrain_height = (float) -3.317121E38F;
            p136.current_height = (float)2.5616996E38F;
            p136.pending = (ushort)(ushort)24369;
            p136.loaded = (ushort)(ushort)56468;
            CommunicationChannel.instance.send(p136); //===============================
            SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.time_boot_ms = (uint)2387027754U;
            p137.press_abs = (float)6.2250356E37F;
            p137.press_diff = (float)3.6993293E36F;
            p137.temperature = (short)(short)26592;
            CommunicationChannel.instance.send(p137); //===============================
            ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.time_usec = (ulong)9212019794354586788L;
            p138.q_SET(new float[4], 0);
            p138.x = (float)3.0567853E38F;
            p138.y = (float)3.1465828E38F;
            p138.z = (float) -3.0536097E38F;
            CommunicationChannel.instance.send(p138); //===============================
            SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.time_usec = (ulong)6764440212404966960L;
            p139.group_mlx = (byte)(byte)152;
            p139.target_system = (byte)(byte)226;
            p139.target_component = (byte)(byte)169;
            p139.controls_SET(new float[8], 0);
            CommunicationChannel.instance.send(p139); //===============================
            ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.time_usec = (ulong)6821466856726037918L;
            p140.group_mlx = (byte)(byte)204;
            p140.controls_SET(new float[8], 0);
            CommunicationChannel.instance.send(p140); //===============================
            ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.time_usec = (ulong)976640362891641301L;
            p141.altitude_monotonic = (float) -1.440749E38F;
            p141.altitude_amsl = (float) -2.9531845E37F;
            p141.altitude_local = (float) -8.818932E37F;
            p141.altitude_relative = (float) -9.827738E37F;
            p141.altitude_terrain = (float) -2.4343514E38F;
            p141.bottom_clearance = (float)9.715137E37F;
            CommunicationChannel.instance.send(p141); //===============================
            RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.request_id = (byte)(byte)67;
            p142.uri_type = (byte)(byte)37;
            p142.uri_SET(new byte[120], 0);
            p142.transfer_type = (byte)(byte)62;
            p142.storage_SET(new byte[120], 0);
            CommunicationChannel.instance.send(p142); //===============================
            SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.time_boot_ms = (uint)6554949U;
            p143.press_abs = (float)2.8873266E38F;
            p143.press_diff = (float)6.218096E37F;
            p143.temperature = (short)(short) -24872;
            CommunicationChannel.instance.send(p143); //===============================
            FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.timestamp = (ulong)8355154649935828945L;
            p144.est_capabilities = (byte)(byte)244;
            p144.lat = (int) -1749259776;
            p144.lon = (int)1372812380;
            p144.alt = (float)4.3753133E37F;
            p144.vel_SET(new float[3], 0);
            p144.acc_SET(new float[3], 0);
            p144.attitude_q_SET(new float[4], 0);
            p144.rates_SET(new float[3], 0);
            p144.position_cov_SET(new float[3], 0);
            p144.custom_state = (ulong)2217876383548997196L;
            CommunicationChannel.instance.send(p144); //===============================
            CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.time_usec = (ulong)1644409450236022400L;
            p146.x_acc = (float) -1.817511E38F;
            p146.y_acc = (float) -8.495769E37F;
            p146.z_acc = (float) -2.5708495E36F;
            p146.x_vel = (float)1.3341073E38F;
            p146.y_vel = (float) -9.16162E37F;
            p146.z_vel = (float) -2.687347E38F;
            p146.x_pos = (float)3.3637837E38F;
            p146.y_pos = (float)1.9769591E38F;
            p146.z_pos = (float)2.4425682E38F;
            p146.airspeed = (float) -6.518117E37F;
            p146.vel_variance_SET(new float[3], 0);
            p146.pos_variance_SET(new float[3], 0);
            p146.q_SET(new float[4], 0);
            p146.roll_rate = (float)3.0031888E38F;
            p146.pitch_rate = (float) -2.7606276E38F;
            p146.yaw_rate = (float)8.2039867E37F;
            CommunicationChannel.instance.send(p146); //===============================
            BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.id = (byte)(byte)153;
            p147.battery_function = MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_ALL;
            p147.type = MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_NIMH;
            p147.temperature = (short)(short)4241;
            p147.voltages_SET(new ushort[10], 0);
            p147.current_battery = (short)(short)11426;
            p147.current_consumed = (int)734553183;
            p147.energy_consumed = (int)702246106;
            p147.battery_remaining = (sbyte)(sbyte)9;
            CommunicationChannel.instance.send(p147); //===============================
            AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION);
            p148.flight_sw_version = (uint)312836843U;
            p148.middleware_sw_version = (uint)21752669U;
            p148.os_sw_version = (uint)798608989U;
            p148.board_version = (uint)4022443806U;
            p148.flight_custom_version_SET(new byte[8], 0);
            p148.middleware_custom_version_SET(new byte[8], 0);
            p148.os_custom_version_SET(new byte[8], 0);
            p148.vendor_id = (ushort)(ushort)1723;
            p148.product_id = (ushort)(ushort)7282;
            p148.uid = (ulong)4626088429346761383L;
            p148.uid2_SET(new byte[18], 0, PH);
            CommunicationChannel.instance.send(p148); //===============================
            LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.time_usec = (ulong)2313717602991219287L;
            p149.target_num = (byte)(byte)45;
            p149.frame = MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p149.angle_x = (float)2.6054205E38F;
            p149.angle_y = (float)1.2070421E38F;
            p149.distance = (float)1.0657946E38F;
            p149.size_x = (float)1.0008783E38F;
            p149.size_y = (float)5.14836E37F;
            p149.x_SET((float)3.1474412E38F, PH);
            p149.y_SET((float)8.2259597E37F, PH);
            p149.z_SET((float)3.5729017E36F, PH);
            p149.q_SET(new float[4], 0, PH);
            p149.type = LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_RADIO_BEACON;
            p149.position_valid_SET((byte)(byte)108, PH);
            CommunicationChannel.instance.send(p149); //===============================
            ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.time_usec = (ulong)7330020391185065982L;
            p230.flags = (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL);
            p230.vel_ratio = (float)5.283282E37F;
            p230.pos_horiz_ratio = (float) -1.0374075E37F;
            p230.pos_vert_ratio = (float) -1.934696E38F;
            p230.mag_ratio = (float) -1.0120973E38F;
            p230.hagl_ratio = (float)3.4479215E34F;
            p230.tas_ratio = (float) -1.4150068E38F;
            p230.pos_horiz_accuracy = (float) -1.2637603E38F;
            p230.pos_vert_accuracy = (float)1.9974038E38F;
            CommunicationChannel.instance.send(p230); //===============================
            WIND_COV p231 = CommunicationChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.time_usec = (ulong)6866399472617738092L;
            p231.wind_x = (float)1.8465643E38F;
            p231.wind_y = (float)1.5999077E37F;
            p231.wind_z = (float)3.3204091E38F;
            p231.var_horiz = (float) -1.4216123E37F;
            p231.var_vert = (float) -1.4158565E38F;
            p231.wind_alt = (float) -3.2822095E38F;
            p231.horiz_accuracy = (float) -3.1066713E38F;
            p231.vert_accuracy = (float) -8.507577E37F;
            CommunicationChannel.instance.send(p231); //===============================
            GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.time_usec = (ulong)3564077093422944838L;
            p232.gps_id = (byte)(byte)27;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY);
            p232.time_week_ms = (uint)332574755U;
            p232.time_week = (ushort)(ushort)55500;
            p232.fix_type = (byte)(byte)161;
            p232.lat = (int)144926860;
            p232.lon = (int) -928461397;
            p232.alt = (float) -1.878927E35F;
            p232.hdop = (float) -3.0187397E38F;
            p232.vdop = (float) -1.8166498E38F;
            p232.vn = (float) -5.834751E37F;
            p232.ve = (float)1.0930899E37F;
            p232.vd = (float) -2.927242E38F;
            p232.speed_accuracy = (float)3.220986E38F;
            p232.horiz_accuracy = (float)2.3856312E38F;
            p232.vert_accuracy = (float) -2.566314E38F;
            p232.satellites_visible = (byte)(byte)75;
            CommunicationChannel.instance.send(p232); //===============================
            GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.flags = (byte)(byte)165;
            p233.len = (byte)(byte)144;
            p233.data__SET(new byte[180], 0);
            CommunicationChannel.instance.send(p233); //===============================
            HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED);
            p234.custom_mode = (uint)34778874U;
            p234.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED;
            p234.roll = (short)(short)13776;
            p234.pitch = (short)(short) -21128;
            p234.heading = (ushort)(ushort)58395;
            p234.throttle = (sbyte)(sbyte)21;
            p234.heading_sp = (short)(short) -507;
            p234.latitude = (int)690650275;
            p234.longitude = (int)1411796725;
            p234.altitude_amsl = (short)(short) -6038;
            p234.altitude_sp = (short)(short)11904;
            p234.airspeed = (byte)(byte)64;
            p234.airspeed_sp = (byte)(byte)110;
            p234.groundspeed = (byte)(byte)223;
            p234.climb_rate = (sbyte)(sbyte) - 115;
            p234.gps_nsat = (byte)(byte)229;
            p234.gps_fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED;
            p234.battery_remaining = (byte)(byte)244;
            p234.temperature = (sbyte)(sbyte) - 79;
            p234.temperature_air = (sbyte)(sbyte) - 30;
            p234.failsafe = (byte)(byte)67;
            p234.wp_num = (byte)(byte)215;
            p234.wp_distance = (ushort)(ushort)44378;
            CommunicationChannel.instance.send(p234); //===============================
            VIBRATION p241 = CommunicationChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.time_usec = (ulong)40944306715816323L;
            p241.vibration_x = (float) -9.139148E37F;
            p241.vibration_y = (float) -1.1882746E38F;
            p241.vibration_z = (float)3.152021E38F;
            p241.clipping_0 = (uint)3879905288U;
            p241.clipping_1 = (uint)1021593281U;
            p241.clipping_2 = (uint)1864608320U;
            CommunicationChannel.instance.send(p241); //===============================
            HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.latitude = (int)290720418;
            p242.longitude = (int)1832204842;
            p242.altitude = (int) -782942062;
            p242.x = (float) -2.202546E38F;
            p242.y = (float)1.5585607E38F;
            p242.z = (float) -1.1803889E38F;
            p242.q_SET(new float[4], 0);
            p242.approach_x = (float) -2.4935265E38F;
            p242.approach_y = (float) -4.719332E37F;
            p242.approach_z = (float) -9.149173E37F;
            p242.time_usec_SET((ulong)8663107061056898883L, PH);
            CommunicationChannel.instance.send(p242); //===============================
            SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.target_system = (byte)(byte)134;
            p243.latitude = (int) -1112957760;
            p243.longitude = (int) -1540554420;
            p243.altitude = (int) -8856809;
            p243.x = (float)2.7516068E38F;
            p243.y = (float) -1.2449963E38F;
            p243.z = (float)2.423341E38F;
            p243.q_SET(new float[4], 0);
            p243.approach_x = (float) -1.2140841E38F;
            p243.approach_y = (float) -4.9524193E36F;
            p243.approach_z = (float)2.2302381E38F;
            p243.time_usec_SET((ulong)7265238185469645193L, PH);
            CommunicationChannel.instance.send(p243); //===============================
            MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.message_id = (ushort)(ushort)36317;
            p244.interval_us = (int)154657113;
            CommunicationChannel.instance.send(p244); //===============================
            EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.vtol_state = MAV_VTOL_STATE.MAV_VTOL_STATE_FW;
            p245.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING;
            CommunicationChannel.instance.send(p245); //===============================
            ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.ICAO_address = (uint)3505187874U;
            p246.lat = (int)311526399;
            p246.lon = (int) -1238916292;
            p246.altitude_type = ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC;
            p246.altitude = (int) -1098772093;
            p246.heading = (ushort)(ushort)7840;
            p246.hor_velocity = (ushort)(ushort)32515;
            p246.ver_velocity = (short)(short)20529;
            p246.callsign_SET("DEMO", PH);
            p246.emitter_type = ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_ROTOCRAFT;
            p246.tslc = (byte)(byte)125;
            p246.flags = (ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY |
                          ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK |
                          ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN |
                          ADSB_FLAGS.ADSB_FLAGS_SIMULATED);
            p246.squawk = (ushort)(ushort)48559;
            CommunicationChannel.instance.send(p246); //===============================
            COLLISION p247 = CommunicationChannel.new_COLLISION();
            PH.setPack(p247);
            p247.src_ = MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT;
            p247.id = (uint)868641214U;
            p247.action = MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_ASCEND_OR_DESCEND;
            p247.threat_level = MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH;
            p247.time_to_minimum_delta = (float)1.8916769E38F;
            p247.altitude_minimum_delta = (float)2.6799255E38F;
            p247.horizontal_minimum_delta = (float) -1.9529877E38F;
            CommunicationChannel.instance.send(p247); //===============================
            V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.target_network = (byte)(byte)122;
            p248.target_system = (byte)(byte)225;
            p248.target_component = (byte)(byte)28;
            p248.message_type = (ushort)(ushort)38888;
            p248.payload_SET(new byte[249], 0);
            CommunicationChannel.instance.send(p248); //===============================
            MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.address = (ushort)(ushort)60826;
            p249.ver = (byte)(byte)63;
            p249.type = (byte)(byte)126;
            p249.value_SET(new sbyte[32], 0);
            CommunicationChannel.instance.send(p249); //===============================
            DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.name_SET("DEMO", PH);
            p250.time_usec = (ulong)8255115531818229810L;
            p250.x = (float) -1.1233766E38F;
            p250.y = (float)1.0046768E38F;
            p250.z = (float) -3.4446813E37F;
            CommunicationChannel.instance.send(p250); //===============================
            NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.time_boot_ms = (uint)1153102715U;
            p251.name_SET("DEMO", PH);
            p251.value = (float)3.3052925E38F;
            CommunicationChannel.instance.send(p251); //===============================
            NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.time_boot_ms = (uint)3771807250U;
            p252.name_SET("DEMO", PH);
            p252.value = (int)1749072685;
            CommunicationChannel.instance.send(p252); //===============================
            STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.severity = MAV_SEVERITY.MAV_SEVERITY_NOTICE;
            p253.text_SET("DEMO", PH);
            CommunicationChannel.instance.send(p253); //===============================
            DEBUG p254 = CommunicationChannel.new_DEBUG();
            PH.setPack(p254);
            p254.time_boot_ms = (uint)2139880404U;
            p254.ind = (byte)(byte)207;
            p254.value = (float)1.2056795E38F;
            CommunicationChannel.instance.send(p254); //===============================
            SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.target_system = (byte)(byte)163;
            p256.target_component = (byte)(byte)125;
            p256.secret_key_SET(new byte[32], 0);
            p256.initial_timestamp = (ulong)4887090147417634248L;
            CommunicationChannel.instance.send(p256); //===============================
            BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.time_boot_ms = (uint)1376053794U;
            p257.last_change_ms = (uint)312855021U;
            p257.state = (byte)(byte)226;
            CommunicationChannel.instance.send(p257); //===============================
            PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_system = (byte)(byte)189;
            p258.target_component = (byte)(byte)13;
            p258.tune_SET("DEMO", PH);
            CommunicationChannel.instance.send(p258); //===============================
            CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.time_boot_ms = (uint)3711300174U;
            p259.vendor_name_SET(new byte[32], 0);
            p259.model_name_SET(new byte[32], 0);
            p259.firmware_version = (uint)919618383U;
            p259.focal_length = (float)2.4214372E38F;
            p259.sensor_size_h = (float)7.46906E37F;
            p259.sensor_size_v = (float) -1.7236854E38F;
            p259.resolution_h = (ushort)(ushort)62491;
            p259.resolution_v = (ushort)(ushort)4048;
            p259.lens_id = (byte)(byte)4;
            p259.flags = (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
                          CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_MODES);
            p259.cam_definition_version = (ushort)(ushort)32791;
            p259.cam_definition_uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p259); //===============================
            CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.time_boot_ms = (uint)2822279606U;
            p260.mode_id = CAMERA_MODE.CAMERA_MODE_IMAGE;
            CommunicationChannel.instance.send(p260); //===============================
            STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.time_boot_ms = (uint)4233714726U;
            p261.storage_id = (byte)(byte)53;
            p261.storage_count = (byte)(byte)136;
            p261.status = (byte)(byte)48;
            p261.total_capacity = (float)2.9813507E38F;
            p261.used_capacity = (float) -3.815445E37F;
            p261.available_capacity = (float)3.02433E38F;
            p261.read_speed = (float)2.9521007E38F;
            p261.write_speed = (float)2.702103E38F;
            CommunicationChannel.instance.send(p261); //===============================
            CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.time_boot_ms = (uint)1936452000U;
            p262.image_status = (byte)(byte)153;
            p262.video_status = (byte)(byte)143;
            p262.image_interval = (float)6.458886E37F;
            p262.recording_time_ms = (uint)1315261395U;
            p262.available_capacity = (float)3.6089937E37F;
            CommunicationChannel.instance.send(p262); //===============================
            CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.time_boot_ms = (uint)4140980658U;
            p263.time_utc = (ulong)728272311845747418L;
            p263.camera_id = (byte)(byte)139;
            p263.lat = (int) -332082154;
            p263.lon = (int)255055404;
            p263.alt = (int) -2116463877;
            p263.relative_alt = (int)1107062500;
            p263.q_SET(new float[4], 0);
            p263.image_index = (int) -798948275;
            p263.capture_result = (sbyte)(sbyte)30;
            p263.file_url_SET("DEMO", PH);
            CommunicationChannel.instance.send(p263); //===============================
            FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.time_boot_ms = (uint)1219994539U;
            p264.arming_time_utc = (ulong)2814643980926514027L;
            p264.takeoff_time_utc = (ulong)1912789186496398370L;
            p264.flight_uuid = (ulong)7075264734948294250L;
            CommunicationChannel.instance.send(p264); //===============================
            MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.time_boot_ms = (uint)3347872730U;
            p265.roll = (float)7.6537503E37F;
            p265.pitch = (float)1.3805599E35F;
            p265.yaw = (float)2.6003986E37F;
            CommunicationChannel.instance.send(p265); //===============================
            LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.target_system = (byte)(byte)80;
            p266.target_component = (byte)(byte)221;
            p266.sequence = (ushort)(ushort)12547;
            p266.length = (byte)(byte)146;
            p266.first_message_offset = (byte)(byte)26;
            p266.data__SET(new byte[249], 0);
            CommunicationChannel.instance.send(p266); //===============================
            LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.target_system = (byte)(byte)199;
            p267.target_component = (byte)(byte)207;
            p267.sequence = (ushort)(ushort)64204;
            p267.length = (byte)(byte)223;
            p267.first_message_offset = (byte)(byte)203;
            p267.data__SET(new byte[249], 0);
            CommunicationChannel.instance.send(p267); //===============================
            LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_system = (byte)(byte)39;
            p268.target_component = (byte)(byte)26;
            p268.sequence = (ushort)(ushort)54125;
            CommunicationChannel.instance.send(p268); //===============================
            VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.camera_id = (byte)(byte)28;
            p269.status = (byte)(byte)206;
            p269.framerate = (float)1.276991E38F;
            p269.resolution_h = (ushort)(ushort)572;
            p269.resolution_v = (ushort)(ushort)62416;
            p269.bitrate = (uint)3514856848U;
            p269.rotation = (ushort)(ushort)13246;
            p269.uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p269); //===============================
            SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.target_system = (byte)(byte)97;
            p270.target_component = (byte)(byte)92;
            p270.camera_id = (byte)(byte)69;
            p270.framerate = (float) -2.070405E38F;
            p270.resolution_h = (ushort)(ushort)60204;
            p270.resolution_v = (ushort)(ushort)15516;
            p270.bitrate = (uint)152387204U;
            p270.rotation = (ushort)(ushort)36729;
            p270.uri_SET("DEMO", PH);
            CommunicationChannel.instance.send(p270); //===============================
            WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.ssid_SET("DEMO", PH);
            p299.password_SET("DEMO", PH);
            CommunicationChannel.instance.send(p299); //===============================
            PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.version = (ushort)(ushort)52365;
            p300.min_version = (ushort)(ushort)58422;
            p300.max_version = (ushort)(ushort)38872;
            p300.spec_version_hash_SET(new byte[8], 0);
            p300.library_version_hash_SET(new byte[8], 0);
            CommunicationChannel.instance.send(p300); //===============================
            UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.time_usec = (ulong)2419926363997756936L;
            p310.uptime_sec = (uint)1540151941U;
            p310.health = UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL;
            p310.mode = UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION;
            p310.sub_mode = (byte)(byte)222;
            p310.vendor_specific_status_code = (ushort)(ushort)44178;
            CommunicationChannel.instance.send(p310); //===============================
            UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.time_usec = (ulong)2238271014770497823L;
            p311.uptime_sec = (uint)2117321728U;
            p311.name_SET("DEMO", PH);
            p311.hw_version_major = (byte)(byte)247;
            p311.hw_version_minor = (byte)(byte)94;
            p311.hw_unique_id_SET(new byte[16], 0);
            p311.sw_version_major = (byte)(byte)145;
            p311.sw_version_minor = (byte)(byte)186;
            p311.sw_vcs_commit = (uint)777920612U;
            CommunicationChannel.instance.send(p311); //===============================
            PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.target_system = (byte)(byte)190;
            p320.target_component = (byte)(byte)178;
            p320.param_id_SET("DEMO", PH);
            p320.param_index = (short)(short)4949;
            CommunicationChannel.instance.send(p320); //===============================
            PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_system = (byte)(byte)235;
            p321.target_component = (byte)(byte)143;
            CommunicationChannel.instance.send(p321); //===============================
            PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_id_SET("DEMO", PH);
            p322.param_value_SET("DEMO", PH);
            p322.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT64;
            p322.param_count = (ushort)(ushort)11717;
            p322.param_index = (ushort)(ushort)37213;
            CommunicationChannel.instance.send(p322); //===============================
            PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.target_system = (byte)(byte)250;
            p323.target_component = (byte)(byte)63;
            p323.param_id_SET("DEMO", PH);
            p323.param_value_SET("DEMO", PH);
            p323.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16;
            CommunicationChannel.instance.send(p323); //===============================
            PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_id_SET("DEMO", PH);
            p324.param_value_SET("DEMO", PH);
            p324.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT8;
            p324.param_result = PARAM_ACK.PARAM_ACK_ACCEPTED;
            CommunicationChannel.instance.send(p324); //===============================
            OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.time_usec = (ulong)2014705819966236745L;
            p330.sensor_type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER;
            p330.distances_SET(new ushort[72], 0);
            p330.increment = (byte)(byte)209;
            p330.min_distance = (ushort)(ushort)45524;
            p330.max_distance = (ushort)(ushort)34029;
            CommunicationChannel.instance.send(p330); //===============================
        }
    }
}

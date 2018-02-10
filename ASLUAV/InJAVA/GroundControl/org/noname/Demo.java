package org.noname;
import org.unirail.BlackBox.Host.Pack.Meta.Field.Bounds;

public class Demo extends  GroundControl
{
    public static void main(String[] args)
    {
        final Bounds.Inside PH = new Bounds.Inside();
        CommunicationChannel.instance.on_HEARTBEAT.add((src, ph, pack) ->
        {
            @MAV_TYPE int  type = pack.type_GET();
            @MAV_AUTOPILOT int  autopilot = pack.autopilot_GET();
            @MAV_MODE_FLAG int  base_mode = pack.base_mode_GET();
            long  custom_mode = pack.custom_mode_GET();
            @MAV_STATE int  system_status = pack.system_status_GET();
            char  mavlink_version = pack.mavlink_version_GET();
        });
        CommunicationChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
        {
            @MAV_SYS_STATUS_SENSOR int  onboard_control_sensors_present = pack.onboard_control_sensors_present_GET();
            @MAV_SYS_STATUS_SENSOR int  onboard_control_sensors_enabled = pack.onboard_control_sensors_enabled_GET();
            @MAV_SYS_STATUS_SENSOR int  onboard_control_sensors_health = pack.onboard_control_sensors_health_GET();
            char  load = pack.load_GET();
            char  voltage_battery = pack.voltage_battery_GET();
            short  current_battery = pack.current_battery_GET();
            byte  battery_remaining = pack.battery_remaining_GET();
            char  drop_rate_comm = pack.drop_rate_comm_GET();
            char  errors_comm = pack.errors_comm_GET();
            char  errors_count1 = pack.errors_count1_GET();
            char  errors_count2 = pack.errors_count2_GET();
            char  errors_count3 = pack.errors_count3_GET();
            char  errors_count4 = pack.errors_count4_GET();
        });
        CommunicationChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
        {
            long  time_unix_usec = pack.time_unix_usec_GET();
            long  time_boot_ms = pack.time_boot_ms_GET();
        });
        CommunicationChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            @MAV_FRAME int  coordinate_frame = pack.coordinate_frame_GET();
            char  type_mask = pack.type_mask_GET();
            float  x = pack.x_GET();
            float  y = pack.y_GET();
            float  z = pack.z_GET();
            float  vx = pack.vx_GET();
            float  vy = pack.vy_GET();
            float  vz = pack.vz_GET();
            float  afx = pack.afx_GET();
            float  afy = pack.afy_GET();
            float  afz = pack.afz_GET();
            float  yaw = pack.yaw_GET();
            float  yaw_rate = pack.yaw_rate_GET();
        });
        CommunicationChannel.instance.on_PING.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            long  seq = pack.seq_GET();
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
        });
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  control_request = pack.control_request_GET();
            char  version = pack.version_GET();
            String passkey = pack.passkey_TRY(ph);
        });
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
        {
            char  gcs_system_id = pack.gcs_system_id_GET();
            char  control_request = pack.control_request_GET();
            char  ack = pack.ack_GET();
        });
        CommunicationChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
        {
            String key = pack.key_TRY(ph);
        });
        CommunicationChannel.instance.on_SET_MODE.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            @MAV_MODE int  base_mode = pack.base_mode_GET();
            long  custom_mode = pack.custom_mode_GET();
        });
        CommunicationChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            String param_id = pack.param_id_TRY(ph);
            short  param_index = pack.param_index_GET();
        });
        CommunicationChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
        });
        CommunicationChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
        {
            String param_id = pack.param_id_TRY(ph);
            float  param_value = pack.param_value_GET();
            @MAV_PARAM_TYPE int  param_type = pack.param_type_GET();
            char  param_count = pack.param_count_GET();
            char  param_index = pack.param_index_GET();
        });
        CommunicationChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            String param_id = pack.param_id_TRY(ph);
            float  param_value = pack.param_value_GET();
            @MAV_PARAM_TYPE int  param_type = pack.param_type_GET();
        });
        CommunicationChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            @GPS_FIX_TYPE int  fix_type = pack.fix_type_GET();
            int  lat = pack.lat_GET();
            int  lon = pack.lon_GET();
            int  alt = pack.alt_GET();
            char  eph = pack.eph_GET();
            char  epv = pack.epv_GET();
            char  vel = pack.vel_GET();
            char  cog = pack.cog_GET();
            char  satellites_visible = pack.satellites_visible_GET();
            int  alt_ellipsoid = pack.alt_ellipsoid_TRY(ph);
            long  h_acc = pack.h_acc_TRY(ph);
            long  v_acc = pack.v_acc_TRY(ph);
            long  vel_acc = pack.vel_acc_TRY(ph);
            long  hdg_acc = pack.hdg_acc_TRY(ph);
        });
        CommunicationChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
        {
            char  satellites_visible = pack.satellites_visible_GET();
            char[]  satellite_prn = pack.satellite_prn_GET();
            char[]  satellite_used = pack.satellite_used_GET();
            char[]  satellite_elevation = pack.satellite_elevation_GET();
            char[]  satellite_azimuth = pack.satellite_azimuth_GET();
            char[]  satellite_snr = pack.satellite_snr_GET();
        });
        CommunicationChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            short  xacc = pack.xacc_GET();
            short  yacc = pack.yacc_GET();
            short  zacc = pack.zacc_GET();
            short  xgyro = pack.xgyro_GET();
            short  ygyro = pack.ygyro_GET();
            short  zgyro = pack.zgyro_GET();
            short  xmag = pack.xmag_GET();
            short  ymag = pack.ymag_GET();
            short  zmag = pack.zmag_GET();
        });
        CommunicationChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            short  xacc = pack.xacc_GET();
            short  yacc = pack.yacc_GET();
            short  zacc = pack.zacc_GET();
            short  xgyro = pack.xgyro_GET();
            short  ygyro = pack.ygyro_GET();
            short  zgyro = pack.zgyro_GET();
            short  xmag = pack.xmag_GET();
            short  ymag = pack.ymag_GET();
            short  zmag = pack.zmag_GET();
        });
        CommunicationChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            short  press_abs = pack.press_abs_GET();
            short  press_diff1 = pack.press_diff1_GET();
            short  press_diff2 = pack.press_diff2_GET();
            short  temperature = pack.temperature_GET();
        });
        CommunicationChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            float  press_abs = pack.press_abs_GET();
            float  press_diff = pack.press_diff_GET();
            short  temperature = pack.temperature_GET();
        });
        CommunicationChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            float  roll = pack.roll_GET();
            float  pitch = pack.pitch_GET();
            float  yaw = pack.yaw_GET();
            float  rollspeed = pack.rollspeed_GET();
            float  pitchspeed = pack.pitchspeed_GET();
            float  yawspeed = pack.yawspeed_GET();
        });
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            float  q1 = pack.q1_GET();
            float  q2 = pack.q2_GET();
            float  q3 = pack.q3_GET();
            float  q4 = pack.q4_GET();
            float  rollspeed = pack.rollspeed_GET();
            float  pitchspeed = pack.pitchspeed_GET();
            float  yawspeed = pack.yawspeed_GET();
        });
        CommunicationChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            float  x = pack.x_GET();
            float  y = pack.y_GET();
            float  z = pack.z_GET();
            float  vx = pack.vx_GET();
            float  vy = pack.vy_GET();
            float  vz = pack.vz_GET();
        });
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            int  lat = pack.lat_GET();
            int  lon = pack.lon_GET();
            int  alt = pack.alt_GET();
            int  relative_alt = pack.relative_alt_GET();
            short  vx = pack.vx_GET();
            short  vy = pack.vy_GET();
            short  vz = pack.vz_GET();
            char  hdg = pack.hdg_GET();
        });
        CommunicationChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            char  port = pack.port_GET();
            short  chan1_scaled = pack.chan1_scaled_GET();
            short  chan2_scaled = pack.chan2_scaled_GET();
            short  chan3_scaled = pack.chan3_scaled_GET();
            short  chan4_scaled = pack.chan4_scaled_GET();
            short  chan5_scaled = pack.chan5_scaled_GET();
            short  chan6_scaled = pack.chan6_scaled_GET();
            short  chan7_scaled = pack.chan7_scaled_GET();
            short  chan8_scaled = pack.chan8_scaled_GET();
            char  rssi = pack.rssi_GET();
        });
        CommunicationChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            char  port = pack.port_GET();
            char  chan1_raw = pack.chan1_raw_GET();
            char  chan2_raw = pack.chan2_raw_GET();
            char  chan3_raw = pack.chan3_raw_GET();
            char  chan4_raw = pack.chan4_raw_GET();
            char  chan5_raw = pack.chan5_raw_GET();
            char  chan6_raw = pack.chan6_raw_GET();
            char  chan7_raw = pack.chan7_raw_GET();
            char  chan8_raw = pack.chan8_raw_GET();
            char  rssi = pack.rssi_GET();
        });
        CommunicationChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            char  port = pack.port_GET();
            char  servo1_raw = pack.servo1_raw_GET();
            char  servo2_raw = pack.servo2_raw_GET();
            char  servo3_raw = pack.servo3_raw_GET();
            char  servo4_raw = pack.servo4_raw_GET();
            char  servo5_raw = pack.servo5_raw_GET();
            char  servo6_raw = pack.servo6_raw_GET();
            char  servo7_raw = pack.servo7_raw_GET();
            char  servo8_raw = pack.servo8_raw_GET();
            char  servo9_raw = pack.servo9_raw_TRY(ph);
            char  servo10_raw = pack.servo10_raw_TRY(ph);
            char  servo11_raw = pack.servo11_raw_TRY(ph);
            char  servo12_raw = pack.servo12_raw_TRY(ph);
            char  servo13_raw = pack.servo13_raw_TRY(ph);
            char  servo14_raw = pack.servo14_raw_TRY(ph);
            char  servo15_raw = pack.servo15_raw_TRY(ph);
            char  servo16_raw = pack.servo16_raw_TRY(ph);
        });
        CommunicationChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            short  start_index = pack.start_index_GET();
            short  end_index = pack.end_index_GET();
            @MAV_MISSION_TYPE int  mission_type = pack.mission_type_GET();
        });
        CommunicationChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            short  start_index = pack.start_index_GET();
            short  end_index = pack.end_index_GET();
            @MAV_MISSION_TYPE int  mission_type = pack.mission_type_GET();
        });
        CommunicationChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  seq = pack.seq_GET();
            @MAV_FRAME int  frame = pack.frame_GET();
            @MAV_CMD int  command = pack.command_GET();
            char  current = pack.current_GET();
            char  autocontinue = pack.autocontinue_GET();
            float  param1 = pack.param1_GET();
            float  param2 = pack.param2_GET();
            float  param3 = pack.param3_GET();
            float  param4 = pack.param4_GET();
            float  x = pack.x_GET();
            float  y = pack.y_GET();
            float  z = pack.z_GET();
            @MAV_MISSION_TYPE int  mission_type = pack.mission_type_GET();
        });
        CommunicationChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  seq = pack.seq_GET();
            @MAV_MISSION_TYPE int  mission_type = pack.mission_type_GET();
        });
        CommunicationChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  seq = pack.seq_GET();
        });
        CommunicationChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
        {
            char  seq = pack.seq_GET();
        });
        CommunicationChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            @MAV_MISSION_TYPE int  mission_type = pack.mission_type_GET();
        });
        CommunicationChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  count = pack.count_GET();
            @MAV_MISSION_TYPE int  mission_type = pack.mission_type_GET();
        });
        CommunicationChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            @MAV_MISSION_TYPE int  mission_type = pack.mission_type_GET();
        });
        CommunicationChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
        {
            char  seq = pack.seq_GET();
        });
        CommunicationChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            @MAV_MISSION_RESULT int  type = pack.type_GET();
            @MAV_MISSION_TYPE int  mission_type = pack.mission_type_GET();
        });
        CommunicationChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            int  latitude = pack.latitude_GET();
            int  longitude = pack.longitude_GET();
            int  altitude = pack.altitude_GET();
            long  time_usec = pack.time_usec_TRY(ph);
        });
        CommunicationChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            int  latitude = pack.latitude_GET();
            int  longitude = pack.longitude_GET();
            int  altitude = pack.altitude_GET();
            long  time_usec = pack.time_usec_TRY(ph);
        });
        CommunicationChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            String param_id = pack.param_id_TRY(ph);
            short  param_index = pack.param_index_GET();
            char  parameter_rc_channel_index = pack.parameter_rc_channel_index_GET();
            float  param_value0 = pack.param_value0_GET();
            float  scale = pack.scale_GET();
            float  param_value_min = pack.param_value_min_GET();
            float  param_value_max = pack.param_value_max_GET();
        });
        CommunicationChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  seq = pack.seq_GET();
            @MAV_MISSION_TYPE int  mission_type = pack.mission_type_GET();
        });
        CommunicationChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            @MAV_FRAME int  frame = pack.frame_GET();
            float  p1x = pack.p1x_GET();
            float  p1y = pack.p1y_GET();
            float  p1z = pack.p1z_GET();
            float  p2x = pack.p2x_GET();
            float  p2y = pack.p2y_GET();
            float  p2z = pack.p2z_GET();
        });
        CommunicationChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
        {
            @MAV_FRAME int  frame = pack.frame_GET();
            float  p1x = pack.p1x_GET();
            float  p1y = pack.p1y_GET();
            float  p1z = pack.p1z_GET();
            float  p2x = pack.p2x_GET();
            float  p2y = pack.p2y_GET();
            float  p2z = pack.p2z_GET();
        });
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            float[]  q = pack.q_GET();
            float  rollspeed = pack.rollspeed_GET();
            float  pitchspeed = pack.pitchspeed_GET();
            float  yawspeed = pack.yawspeed_GET();
            float[]  covariance = pack.covariance_GET();
        });
        CommunicationChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
        {
            float  nav_roll = pack.nav_roll_GET();
            float  nav_pitch = pack.nav_pitch_GET();
            short  nav_bearing = pack.nav_bearing_GET();
            short  target_bearing = pack.target_bearing_GET();
            char  wp_dist = pack.wp_dist_GET();
            float  alt_error = pack.alt_error_GET();
            float  aspd_error = pack.aspd_error_GET();
            float  xtrack_error = pack.xtrack_error_GET();
        });
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            @MAV_ESTIMATOR_TYPE int  estimator_type = pack.estimator_type_GET();
            int  lat = pack.lat_GET();
            int  lon = pack.lon_GET();
            int  alt = pack.alt_GET();
            int  relative_alt = pack.relative_alt_GET();
            float  vx = pack.vx_GET();
            float  vy = pack.vy_GET();
            float  vz = pack.vz_GET();
            float[]  covariance = pack.covariance_GET();
        });
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            @MAV_ESTIMATOR_TYPE int  estimator_type = pack.estimator_type_GET();
            float  x = pack.x_GET();
            float  y = pack.y_GET();
            float  z = pack.z_GET();
            float  vx = pack.vx_GET();
            float  vy = pack.vy_GET();
            float  vz = pack.vz_GET();
            float  ax = pack.ax_GET();
            float  ay = pack.ay_GET();
            float  az = pack.az_GET();
            float[]  covariance = pack.covariance_GET();
        });
        CommunicationChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            char  chancount = pack.chancount_GET();
            char  chan1_raw = pack.chan1_raw_GET();
            char  chan2_raw = pack.chan2_raw_GET();
            char  chan3_raw = pack.chan3_raw_GET();
            char  chan4_raw = pack.chan4_raw_GET();
            char  chan5_raw = pack.chan5_raw_GET();
            char  chan6_raw = pack.chan6_raw_GET();
            char  chan7_raw = pack.chan7_raw_GET();
            char  chan8_raw = pack.chan8_raw_GET();
            char  chan9_raw = pack.chan9_raw_GET();
            char  chan10_raw = pack.chan10_raw_GET();
            char  chan11_raw = pack.chan11_raw_GET();
            char  chan12_raw = pack.chan12_raw_GET();
            char  chan13_raw = pack.chan13_raw_GET();
            char  chan14_raw = pack.chan14_raw_GET();
            char  chan15_raw = pack.chan15_raw_GET();
            char  chan16_raw = pack.chan16_raw_GET();
            char  chan17_raw = pack.chan17_raw_GET();
            char  chan18_raw = pack.chan18_raw_GET();
            char  rssi = pack.rssi_GET();
        });
        CommunicationChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  req_stream_id = pack.req_stream_id_GET();
            char  req_message_rate = pack.req_message_rate_GET();
            char  start_stop = pack.start_stop_GET();
        });
        CommunicationChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
        {
            char  stream_id = pack.stream_id_GET();
            char  message_rate = pack.message_rate_GET();
            char  on_off = pack.on_off_GET();
        });
        CommunicationChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
        {
            char  target = pack.target_GET();
            short  x = pack.x_GET();
            short  y = pack.y_GET();
            short  z = pack.z_GET();
            short  r = pack.r_GET();
            char  buttons = pack.buttons_GET();
        });
        CommunicationChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  chan1_raw = pack.chan1_raw_GET();
            char  chan2_raw = pack.chan2_raw_GET();
            char  chan3_raw = pack.chan3_raw_GET();
            char  chan4_raw = pack.chan4_raw_GET();
            char  chan5_raw = pack.chan5_raw_GET();
            char  chan6_raw = pack.chan6_raw_GET();
            char  chan7_raw = pack.chan7_raw_GET();
            char  chan8_raw = pack.chan8_raw_GET();
        });
        CommunicationChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  seq = pack.seq_GET();
            @MAV_FRAME int  frame = pack.frame_GET();
            @MAV_CMD int  command = pack.command_GET();
            char  current = pack.current_GET();
            char  autocontinue = pack.autocontinue_GET();
            float  param1 = pack.param1_GET();
            float  param2 = pack.param2_GET();
            float  param3 = pack.param3_GET();
            float  param4 = pack.param4_GET();
            int  x = pack.x_GET();
            int  y = pack.y_GET();
            float  z = pack.z_GET();
            @MAV_MISSION_TYPE int  mission_type = pack.mission_type_GET();
        });
        CommunicationChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
        {
            float  airspeed = pack.airspeed_GET();
            float  groundspeed = pack.groundspeed_GET();
            short  heading = pack.heading_GET();
            char  throttle = pack.throttle_GET();
            float  alt = pack.alt_GET();
            float  climb = pack.climb_GET();
        });
        CommunicationChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            @MAV_FRAME int  frame = pack.frame_GET();
            @MAV_CMD int  command = pack.command_GET();
            char  current = pack.current_GET();
            char  autocontinue = pack.autocontinue_GET();
            float  param1 = pack.param1_GET();
            float  param2 = pack.param2_GET();
            float  param3 = pack.param3_GET();
            float  param4 = pack.param4_GET();
            int  x = pack.x_GET();
            int  y = pack.y_GET();
            float  z = pack.z_GET();
        });
        CommunicationChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            @MAV_CMD int  command = pack.command_GET();
            char  confirmation = pack.confirmation_GET();
            float  param1 = pack.param1_GET();
            float  param2 = pack.param2_GET();
            float  param3 = pack.param3_GET();
            float  param4 = pack.param4_GET();
            float  param5 = pack.param5_GET();
            float  param6 = pack.param6_GET();
            float  param7 = pack.param7_GET();
        });
        CommunicationChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
        {
            @MAV_CMD int  command = pack.command_GET();
            @MAV_RESULT int  result = pack.result_GET();
            char  progress = pack.progress_TRY(ph);
            int  result_param2 = pack.result_param2_TRY(ph);
            char  target_system = pack.target_system_TRY(ph);
            char  target_component = pack.target_component_TRY(ph);
        });
        CommunicationChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            float  roll = pack.roll_GET();
            float  pitch = pack.pitch_GET();
            float  yaw = pack.yaw_GET();
            float  thrust = pack.thrust_GET();
            char  mode_switch = pack.mode_switch_GET();
            char  manual_override_switch = pack.manual_override_switch_GET();
        });
        CommunicationChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  type_mask = pack.type_mask_GET();
            float[]  q = pack.q_GET();
            float  body_roll_rate = pack.body_roll_rate_GET();
            float  body_pitch_rate = pack.body_pitch_rate_GET();
            float  body_yaw_rate = pack.body_yaw_rate_GET();
            float  thrust = pack.thrust_GET();
        });
        CommunicationChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            char  type_mask = pack.type_mask_GET();
            float[]  q = pack.q_GET();
            float  body_roll_rate = pack.body_roll_rate_GET();
            float  body_pitch_rate = pack.body_pitch_rate_GET();
            float  body_yaw_rate = pack.body_yaw_rate_GET();
            float  thrust = pack.thrust_GET();
        });
        CommunicationChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            @MAV_FRAME int  coordinate_frame = pack.coordinate_frame_GET();
            char  type_mask = pack.type_mask_GET();
            float  x = pack.x_GET();
            float  y = pack.y_GET();
            float  z = pack.z_GET();
            float  vx = pack.vx_GET();
            float  vy = pack.vy_GET();
            float  vz = pack.vz_GET();
            float  afx = pack.afx_GET();
            float  afy = pack.afy_GET();
            float  afz = pack.afz_GET();
            float  yaw = pack.yaw_GET();
            float  yaw_rate = pack.yaw_rate_GET();
        });
        CommunicationChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            @MAV_FRAME int  coordinate_frame = pack.coordinate_frame_GET();
            char  type_mask = pack.type_mask_GET();
            int  lat_int = pack.lat_int_GET();
            int  lon_int = pack.lon_int_GET();
            float  alt = pack.alt_GET();
            float  vx = pack.vx_GET();
            float  vy = pack.vy_GET();
            float  vz = pack.vz_GET();
            float  afx = pack.afx_GET();
            float  afy = pack.afy_GET();
            float  afz = pack.afz_GET();
            float  yaw = pack.yaw_GET();
            float  yaw_rate = pack.yaw_rate_GET();
        });
        CommunicationChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            @MAV_FRAME int  coordinate_frame = pack.coordinate_frame_GET();
            char  type_mask = pack.type_mask_GET();
            int  lat_int = pack.lat_int_GET();
            int  lon_int = pack.lon_int_GET();
            float  alt = pack.alt_GET();
            float  vx = pack.vx_GET();
            float  vy = pack.vy_GET();
            float  vz = pack.vz_GET();
            float  afx = pack.afx_GET();
            float  afy = pack.afy_GET();
            float  afz = pack.afz_GET();
            float  yaw = pack.yaw_GET();
            float  yaw_rate = pack.yaw_rate_GET();
        });
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            float  x = pack.x_GET();
            float  y = pack.y_GET();
            float  z = pack.z_GET();
            float  roll = pack.roll_GET();
            float  pitch = pack.pitch_GET();
            float  yaw = pack.yaw_GET();
        });
        CommunicationChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            float  roll = pack.roll_GET();
            float  pitch = pack.pitch_GET();
            float  yaw = pack.yaw_GET();
            float  rollspeed = pack.rollspeed_GET();
            float  pitchspeed = pack.pitchspeed_GET();
            float  yawspeed = pack.yawspeed_GET();
            int  lat = pack.lat_GET();
            int  lon = pack.lon_GET();
            int  alt = pack.alt_GET();
            short  vx = pack.vx_GET();
            short  vy = pack.vy_GET();
            short  vz = pack.vz_GET();
            short  xacc = pack.xacc_GET();
            short  yacc = pack.yacc_GET();
            short  zacc = pack.zacc_GET();
        });
        CommunicationChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            float  roll_ailerons = pack.roll_ailerons_GET();
            float  pitch_elevator = pack.pitch_elevator_GET();
            float  yaw_rudder = pack.yaw_rudder_GET();
            float  throttle = pack.throttle_GET();
            float  aux1 = pack.aux1_GET();
            float  aux2 = pack.aux2_GET();
            float  aux3 = pack.aux3_GET();
            float  aux4 = pack.aux4_GET();
            @MAV_MODE int  mode = pack.mode_GET();
            char  nav_mode = pack.nav_mode_GET();
        });
        CommunicationChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            char  chan1_raw = pack.chan1_raw_GET();
            char  chan2_raw = pack.chan2_raw_GET();
            char  chan3_raw = pack.chan3_raw_GET();
            char  chan4_raw = pack.chan4_raw_GET();
            char  chan5_raw = pack.chan5_raw_GET();
            char  chan6_raw = pack.chan6_raw_GET();
            char  chan7_raw = pack.chan7_raw_GET();
            char  chan8_raw = pack.chan8_raw_GET();
            char  chan9_raw = pack.chan9_raw_GET();
            char  chan10_raw = pack.chan10_raw_GET();
            char  chan11_raw = pack.chan11_raw_GET();
            char  chan12_raw = pack.chan12_raw_GET();
            char  rssi = pack.rssi_GET();
        });
        CommunicationChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            float[]  controls = pack.controls_GET();
            @MAV_MODE int  mode = pack.mode_GET();
            long  flags = pack.flags_GET();
        });
        CommunicationChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            char  sensor_id = pack.sensor_id_GET();
            short  flow_x = pack.flow_x_GET();
            short  flow_y = pack.flow_y_GET();
            float  flow_comp_m_x = pack.flow_comp_m_x_GET();
            float  flow_comp_m_y = pack.flow_comp_m_y_GET();
            char  quality = pack.quality_GET();
            float  ground_distance = pack.ground_distance_GET();
            float  flow_rate_x = pack.flow_rate_x_TRY(ph);
            float  flow_rate_y = pack.flow_rate_y_TRY(ph);
        });
        CommunicationChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            long  usec = pack.usec_GET();
            float  x = pack.x_GET();
            float  y = pack.y_GET();
            float  z = pack.z_GET();
            float  roll = pack.roll_GET();
            float  pitch = pack.pitch_GET();
            float  yaw = pack.yaw_GET();
        });
        CommunicationChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            long  usec = pack.usec_GET();
            float  x = pack.x_GET();
            float  y = pack.y_GET();
            float  z = pack.z_GET();
            float  roll = pack.roll_GET();
            float  pitch = pack.pitch_GET();
            float  yaw = pack.yaw_GET();
        });
        CommunicationChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
        {
            long  usec = pack.usec_GET();
            float  x = pack.x_GET();
            float  y = pack.y_GET();
            float  z = pack.z_GET();
        });
        CommunicationChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            long  usec = pack.usec_GET();
            float  x = pack.x_GET();
            float  y = pack.y_GET();
            float  z = pack.z_GET();
            float  roll = pack.roll_GET();
            float  pitch = pack.pitch_GET();
            float  yaw = pack.yaw_GET();
        });
        CommunicationChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            float  xacc = pack.xacc_GET();
            float  yacc = pack.yacc_GET();
            float  zacc = pack.zacc_GET();
            float  xgyro = pack.xgyro_GET();
            float  ygyro = pack.ygyro_GET();
            float  zgyro = pack.zgyro_GET();
            float  xmag = pack.xmag_GET();
            float  ymag = pack.ymag_GET();
            float  zmag = pack.zmag_GET();
            float  abs_pressure = pack.abs_pressure_GET();
            float  diff_pressure = pack.diff_pressure_GET();
            float  pressure_alt = pack.pressure_alt_GET();
            float  temperature = pack.temperature_GET();
            char  fields_updated = pack.fields_updated_GET();
        });
        CommunicationChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            char  sensor_id = pack.sensor_id_GET();
            long  integration_time_us = pack.integration_time_us_GET();
            float  integrated_x = pack.integrated_x_GET();
            float  integrated_y = pack.integrated_y_GET();
            float  integrated_xgyro = pack.integrated_xgyro_GET();
            float  integrated_ygyro = pack.integrated_ygyro_GET();
            float  integrated_zgyro = pack.integrated_zgyro_GET();
            short  temperature = pack.temperature_GET();
            char  quality = pack.quality_GET();
            long  time_delta_distance_us = pack.time_delta_distance_us_GET();
            float  distance = pack.distance_GET();
        });
        CommunicationChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            float  xacc = pack.xacc_GET();
            float  yacc = pack.yacc_GET();
            float  zacc = pack.zacc_GET();
            float  xgyro = pack.xgyro_GET();
            float  ygyro = pack.ygyro_GET();
            float  zgyro = pack.zgyro_GET();
            float  xmag = pack.xmag_GET();
            float  ymag = pack.ymag_GET();
            float  zmag = pack.zmag_GET();
            float  abs_pressure = pack.abs_pressure_GET();
            float  diff_pressure = pack.diff_pressure_GET();
            float  pressure_alt = pack.pressure_alt_GET();
            float  temperature = pack.temperature_GET();
            long  fields_updated = pack.fields_updated_GET();
        });
        CommunicationChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
        {
            float  q1 = pack.q1_GET();
            float  q2 = pack.q2_GET();
            float  q3 = pack.q3_GET();
            float  q4 = pack.q4_GET();
            float  roll = pack.roll_GET();
            float  pitch = pack.pitch_GET();
            float  yaw = pack.yaw_GET();
            float  xacc = pack.xacc_GET();
            float  yacc = pack.yacc_GET();
            float  zacc = pack.zacc_GET();
            float  xgyro = pack.xgyro_GET();
            float  ygyro = pack.ygyro_GET();
            float  zgyro = pack.zgyro_GET();
            float  lat = pack.lat_GET();
            float  lon = pack.lon_GET();
            float  alt = pack.alt_GET();
            float  std_dev_horz = pack.std_dev_horz_GET();
            float  std_dev_vert = pack.std_dev_vert_GET();
            float  vn = pack.vn_GET();
            float  ve = pack.ve_GET();
            float  vd = pack.vd_GET();
        });
        CommunicationChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
        {
            char  rssi = pack.rssi_GET();
            char  remrssi = pack.remrssi_GET();
            char  txbuf = pack.txbuf_GET();
            char  noise = pack.noise_GET();
            char  remnoise = pack.remnoise_GET();
            char  rxerrors = pack.rxerrors_GET();
            char  fixed_ = pack.fixed__GET();
        });
        CommunicationChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
        {
            char  target_network = pack.target_network_GET();
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char[]  payload = pack.payload_GET();
        });
        CommunicationChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
        {
            long  tc1 = pack.tc1_GET();
            long  ts1 = pack.ts1_GET();
        });
        CommunicationChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            long  seq = pack.seq_GET();
        });
        CommunicationChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            char  fix_type = pack.fix_type_GET();
            int  lat = pack.lat_GET();
            int  lon = pack.lon_GET();
            int  alt = pack.alt_GET();
            char  eph = pack.eph_GET();
            char  epv = pack.epv_GET();
            char  vel = pack.vel_GET();
            short  vn = pack.vn_GET();
            short  ve = pack.ve_GET();
            short  vd = pack.vd_GET();
            char  cog = pack.cog_GET();
            char  satellites_visible = pack.satellites_visible_GET();
        });
        CommunicationChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            char  sensor_id = pack.sensor_id_GET();
            long  integration_time_us = pack.integration_time_us_GET();
            float  integrated_x = pack.integrated_x_GET();
            float  integrated_y = pack.integrated_y_GET();
            float  integrated_xgyro = pack.integrated_xgyro_GET();
            float  integrated_ygyro = pack.integrated_ygyro_GET();
            float  integrated_zgyro = pack.integrated_zgyro_GET();
            short  temperature = pack.temperature_GET();
            char  quality = pack.quality_GET();
            long  time_delta_distance_us = pack.time_delta_distance_us_GET();
            float  distance = pack.distance_GET();
        });
        CommunicationChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            float[]  attitude_quaternion = pack.attitude_quaternion_GET();
            float  rollspeed = pack.rollspeed_GET();
            float  pitchspeed = pack.pitchspeed_GET();
            float  yawspeed = pack.yawspeed_GET();
            int  lat = pack.lat_GET();
            int  lon = pack.lon_GET();
            int  alt = pack.alt_GET();
            short  vx = pack.vx_GET();
            short  vy = pack.vy_GET();
            short  vz = pack.vz_GET();
            char  ind_airspeed = pack.ind_airspeed_GET();
            char  true_airspeed = pack.true_airspeed_GET();
            short  xacc = pack.xacc_GET();
            short  yacc = pack.yacc_GET();
            short  zacc = pack.zacc_GET();
        });
        CommunicationChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            short  xacc = pack.xacc_GET();
            short  yacc = pack.yacc_GET();
            short  zacc = pack.zacc_GET();
            short  xgyro = pack.xgyro_GET();
            short  ygyro = pack.ygyro_GET();
            short  zgyro = pack.zgyro_GET();
            short  xmag = pack.xmag_GET();
            short  ymag = pack.ymag_GET();
            short  zmag = pack.zmag_GET();
        });
        CommunicationChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  start = pack.start_GET();
            char  end = pack.end_GET();
        });
        CommunicationChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
        {
            char  id = pack.id_GET();
            char  num_logs = pack.num_logs_GET();
            char  last_log_num = pack.last_log_num_GET();
            long  time_utc = pack.time_utc_GET();
            long  size = pack.size_GET();
        });
        CommunicationChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  id = pack.id_GET();
            long  ofs = pack.ofs_GET();
            long  count = pack.count_GET();
        });
        CommunicationChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
        {
            char  id = pack.id_GET();
            long  ofs = pack.ofs_GET();
            char  count = pack.count_GET();
            char[]  data_ = pack.data__GET();
        });
        CommunicationChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
        });
        CommunicationChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
        });
        CommunicationChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  len = pack.len_GET();
            char[]  data_ = pack.data__GET();
        });
        CommunicationChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            @GPS_FIX_TYPE int  fix_type = pack.fix_type_GET();
            int  lat = pack.lat_GET();
            int  lon = pack.lon_GET();
            int  alt = pack.alt_GET();
            char  eph = pack.eph_GET();
            char  epv = pack.epv_GET();
            char  vel = pack.vel_GET();
            char  cog = pack.cog_GET();
            char  satellites_visible = pack.satellites_visible_GET();
            char  dgps_numch = pack.dgps_numch_GET();
            long  dgps_age = pack.dgps_age_GET();
        });
        CommunicationChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
        {
            char  Vcc = pack.Vcc_GET();
            char  Vservo = pack.Vservo_GET();
            @MAV_POWER_STATUS int  flags = pack.flags_GET();
        });
        CommunicationChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
        {
            @SERIAL_CONTROL_DEV int  device = pack.device_GET();
            @SERIAL_CONTROL_FLAG int  flags = pack.flags_GET();
            char  timeout = pack.timeout_GET();
            long  baudrate = pack.baudrate_GET();
            char  count = pack.count_GET();
            char[]  data_ = pack.data__GET();
        });
        CommunicationChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
        {
            long  time_last_baseline_ms = pack.time_last_baseline_ms_GET();
            char  rtk_receiver_id = pack.rtk_receiver_id_GET();
            char  wn = pack.wn_GET();
            long  tow = pack.tow_GET();
            char  rtk_health = pack.rtk_health_GET();
            char  rtk_rate = pack.rtk_rate_GET();
            char  nsats = pack.nsats_GET();
            char  baseline_coords_type = pack.baseline_coords_type_GET();
            int  baseline_a_mm = pack.baseline_a_mm_GET();
            int  baseline_b_mm = pack.baseline_b_mm_GET();
            int  baseline_c_mm = pack.baseline_c_mm_GET();
            long  accuracy = pack.accuracy_GET();
            int  iar_num_hypotheses = pack.iar_num_hypotheses_GET();
        });
        CommunicationChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
        {
            long  time_last_baseline_ms = pack.time_last_baseline_ms_GET();
            char  rtk_receiver_id = pack.rtk_receiver_id_GET();
            char  wn = pack.wn_GET();
            long  tow = pack.tow_GET();
            char  rtk_health = pack.rtk_health_GET();
            char  rtk_rate = pack.rtk_rate_GET();
            char  nsats = pack.nsats_GET();
            char  baseline_coords_type = pack.baseline_coords_type_GET();
            int  baseline_a_mm = pack.baseline_a_mm_GET();
            int  baseline_b_mm = pack.baseline_b_mm_GET();
            int  baseline_c_mm = pack.baseline_c_mm_GET();
            long  accuracy = pack.accuracy_GET();
            int  iar_num_hypotheses = pack.iar_num_hypotheses_GET();
        });
        CommunicationChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            short  xacc = pack.xacc_GET();
            short  yacc = pack.yacc_GET();
            short  zacc = pack.zacc_GET();
            short  xgyro = pack.xgyro_GET();
            short  ygyro = pack.ygyro_GET();
            short  zgyro = pack.zgyro_GET();
            short  xmag = pack.xmag_GET();
            short  ymag = pack.ymag_GET();
            short  zmag = pack.zmag_GET();
        });
        CommunicationChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
        {
            char  type = pack.type_GET();
            long  size = pack.size_GET();
            char  width = pack.width_GET();
            char  height = pack.height_GET();
            char  packets = pack.packets_GET();
            char  payload = pack.payload_GET();
            char  jpg_quality = pack.jpg_quality_GET();
        });
        CommunicationChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
        {
            char  seqnr = pack.seqnr_GET();
            char[]  data_ = pack.data__GET();
        });
        CommunicationChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            char  min_distance = pack.min_distance_GET();
            char  max_distance = pack.max_distance_GET();
            char  current_distance = pack.current_distance_GET();
            @MAV_DISTANCE_SENSOR int  type = pack.type_GET();
            char  id = pack.id_GET();
            @MAV_SENSOR_ORIENTATION int  orientation = pack.orientation_GET();
            char  covariance = pack.covariance_GET();
        });
        CommunicationChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
        {
            int  lat = pack.lat_GET();
            int  lon = pack.lon_GET();
            char  grid_spacing = pack.grid_spacing_GET();
            long  mask = pack.mask_GET();
        });
        CommunicationChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
        {
            int  lat = pack.lat_GET();
            int  lon = pack.lon_GET();
            char  grid_spacing = pack.grid_spacing_GET();
            char  gridbit = pack.gridbit_GET();
            short[]  data_ = pack.data__GET();
        });
        CommunicationChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
        {
            int  lat = pack.lat_GET();
            int  lon = pack.lon_GET();
        });
        CommunicationChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
        {
            int  lat = pack.lat_GET();
            int  lon = pack.lon_GET();
            char  spacing = pack.spacing_GET();
            float  terrain_height = pack.terrain_height_GET();
            float  current_height = pack.current_height_GET();
            char  pending = pack.pending_GET();
            char  loaded = pack.loaded_GET();
        });
        CommunicationChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            float  press_abs = pack.press_abs_GET();
            float  press_diff = pack.press_diff_GET();
            short  temperature = pack.temperature_GET();
        });
        CommunicationChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            float[]  q = pack.q_GET();
            float  x = pack.x_GET();
            float  y = pack.y_GET();
            float  z = pack.z_GET();
        });
        CommunicationChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            char  group_mlx = pack.group_mlx_GET();
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            float[]  controls = pack.controls_GET();
        });
        CommunicationChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            char  group_mlx = pack.group_mlx_GET();
            float[]  controls = pack.controls_GET();
        });
        CommunicationChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            float  altitude_monotonic = pack.altitude_monotonic_GET();
            float  altitude_amsl = pack.altitude_amsl_GET();
            float  altitude_local = pack.altitude_local_GET();
            float  altitude_relative = pack.altitude_relative_GET();
            float  altitude_terrain = pack.altitude_terrain_GET();
            float  bottom_clearance = pack.bottom_clearance_GET();
        });
        CommunicationChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
        {
            char  request_id = pack.request_id_GET();
            char  uri_type = pack.uri_type_GET();
            char[]  uri = pack.uri_GET();
            char  transfer_type = pack.transfer_type_GET();
            char[]  storage = pack.storage_GET();
        });
        CommunicationChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            float  press_abs = pack.press_abs_GET();
            float  press_diff = pack.press_diff_GET();
            short  temperature = pack.temperature_GET();
        });
        CommunicationChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
        {
            long  timestamp = pack.timestamp_GET();
            char  est_capabilities = pack.est_capabilities_GET();
            int  lat = pack.lat_GET();
            int  lon = pack.lon_GET();
            float  alt = pack.alt_GET();
            float[]  vel = pack.vel_GET();
            float[]  acc = pack.acc_GET();
            float[]  attitude_q = pack.attitude_q_GET();
            float[]  rates = pack.rates_GET();
            float[]  position_cov = pack.position_cov_GET();
            long  custom_state = pack.custom_state_GET();
        });
        CommunicationChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            float  x_acc = pack.x_acc_GET();
            float  y_acc = pack.y_acc_GET();
            float  z_acc = pack.z_acc_GET();
            float  x_vel = pack.x_vel_GET();
            float  y_vel = pack.y_vel_GET();
            float  z_vel = pack.z_vel_GET();
            float  x_pos = pack.x_pos_GET();
            float  y_pos = pack.y_pos_GET();
            float  z_pos = pack.z_pos_GET();
            float  airspeed = pack.airspeed_GET();
            float[]  vel_variance = pack.vel_variance_GET();
            float[]  pos_variance = pack.pos_variance_GET();
            float[]  q = pack.q_GET();
            float  roll_rate = pack.roll_rate_GET();
            float  pitch_rate = pack.pitch_rate_GET();
            float  yaw_rate = pack.yaw_rate_GET();
        });
        CommunicationChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
        {
            char  id = pack.id_GET();
            @MAV_BATTERY_FUNCTION int  battery_function = pack.battery_function_GET();
            @MAV_BATTERY_TYPE int  type = pack.type_GET();
            short  temperature = pack.temperature_GET();
            char[]  voltages = pack.voltages_GET();
            short  current_battery = pack.current_battery_GET();
            int  current_consumed = pack.current_consumed_GET();
            int  energy_consumed = pack.energy_consumed_GET();
            byte  battery_remaining = pack.battery_remaining_GET();
        });
        POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.instance.new_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.time_boot_ms_SET(395135768L);
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED);
        p3.type_mask_SET((char)54303);
        p3.x_SET(6.8106866E37F);
        p3.y_SET(2.4201685E38F);
        p3.z_SET(-2.6355304E38F);
        p3.vx_SET(9.372832E37F);
        p3.vy_SET(1.4086798E38F);
        p3.vz_SET(5.7078184E37F);
        p3.afx_SET(1.2476712E38F);
        p3.afy_SET(1.3006773E38F);
        p3.afz_SET(6.1782126E37F);
        p3.yaw_SET(-1.7553044E38F);
        p3.yaw_rate_SET(1.8531109E38F);
        CommunicationChannel.instance.send(p3); //===============================
        SET_ATTITUDE_TARGET p82 = CommunicationChannel.instance.new_SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.time_boot_ms_SET(3498064032L);
        p82.target_system_SET((char)82);
        p82.target_component_SET((char)241);
        p82.type_mask_SET((char)104);
        p82.q_SET(new float[4], 0);
        p82.body_roll_rate_SET(-3.0600968E38F);
        p82.body_pitch_rate_SET(-5.4907403E37F);
        p82.body_yaw_rate_SET(-2.9688121E38F);
        p82.thrust_SET(2.4496697E38F);
        CommunicationChannel.instance.send(p82); //===============================
        ATTITUDE_TARGET p83 = CommunicationChannel.instance.new_ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.time_boot_ms_SET(53031779L);
        p83.type_mask_SET((char)78);
        p83.q_SET(new float[4], 0);
        p83.body_roll_rate_SET(-3.318537E38F);
        p83.body_pitch_rate_SET(-1.4421806E38F);
        p83.body_yaw_rate_SET(2.5354827E38F);
        p83.thrust_SET(-2.1080628E38F);
        CommunicationChannel.instance.send(p83); //===============================
        SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.instance.new_SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.time_boot_ms_SET(168885496L);
        p84.target_system_SET((char)186);
        p84.target_component_SET((char)125);
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
        p84.type_mask_SET((char)2071);
        p84.x_SET(-2.7657266E38F);
        p84.y_SET(1.0334381E38F);
        p84.z_SET(-8.147422E37F);
        p84.vx_SET(3.938854E37F);
        p84.vy_SET(-3.1949065E38F);
        p84.vz_SET(2.4788735E38F);
        p84.afx_SET(2.727967E38F);
        p84.afy_SET(2.547735E38F);
        p84.afz_SET(3.0915455E38F);
        p84.yaw_SET(-6.985552E37F);
        p84.yaw_rate_SET(-4.924992E37F);
        CommunicationChannel.instance.send(p84); //===============================
        SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.instance.new_SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.time_boot_ms_SET(2721326349L);
        p86.target_system_SET((char)128);
        p86.target_component_SET((char)40);
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED);
        p86.type_mask_SET((char)61094);
        p86.lat_int_SET(87490048);
        p86.lon_int_SET(252907250);
        p86.alt_SET(-2.6079187E38F);
        p86.vx_SET(-5.345457E37F);
        p86.vy_SET(-2.7083422E38F);
        p86.vz_SET(-1.6930159E38F);
        p86.afx_SET(-1.5744863E38F);
        p86.afy_SET(-3.0529582E38F);
        p86.afz_SET(-3.2490556E38F);
        p86.yaw_SET(1.2687479E38F);
        p86.yaw_rate_SET(-2.476627E37F);
        CommunicationChannel.instance.send(p86); //===============================
        POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.instance.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.time_boot_ms_SET(1686973126L);
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
        p87.type_mask_SET((char)14201);
        p87.lat_int_SET(-1950359826);
        p87.lon_int_SET(-1092418361);
        p87.alt_SET(-5.986808E35F);
        p87.vx_SET(-2.4293053E38F);
        p87.vy_SET(1.6005165E38F);
        p87.vz_SET(9.381194E37F);
        p87.afx_SET(2.5539154E38F);
        p87.afy_SET(-2.544256E38F);
        p87.afz_SET(-2.2505528E38F);
        p87.yaw_SET(2.5391824E38F);
        p87.yaw_rate_SET(-2.02708E38F);
        CommunicationChannel.instance.send(p87); //===============================
        LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.instance.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.time_boot_ms_SET(4251613260L);
        p89.x_SET(-1.341871E38F);
        p89.y_SET(2.1075795E38F);
        p89.z_SET(-1.0799639E38F);
        p89.roll_SET(-8.4806814E37F);
        p89.pitch_SET(-1.5163313E37F);
        p89.yaw_SET(1.7714644E38F);
        CommunicationChannel.instance.send(p89); //===============================
        HIL_STATE p90 = CommunicationChannel.instance.new_HIL_STATE();
        PH.setPack(p90);
        p90.time_usec_SET(5828812421972695925L);
        p90.roll_SET(3.0935155E38F);
        p90.pitch_SET(1.0945164E38F);
        p90.yaw_SET(-2.5550816E37F);
        p90.rollspeed_SET(-3.73664E37F);
        p90.pitchspeed_SET(1.7494185E38F);
        p90.yawspeed_SET(8.490881E37F);
        p90.lat_SET(-580335318);
        p90.lon_SET(1509610192);
        p90.alt_SET(-1369809764);
        p90.vx_SET((short) -8898);
        p90.vy_SET((short) -23109);
        p90.vz_SET((short) -9604);
        p90.xacc_SET((short) -24003);
        p90.yacc_SET((short)20184);
        p90.zacc_SET((short) -26933);
        CommunicationChannel.instance.send(p90); //===============================
        HIL_CONTROLS p91 = CommunicationChannel.instance.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.time_usec_SET(5211215020268181650L);
        p91.roll_ailerons_SET(5.348794E37F);
        p91.pitch_elevator_SET(-1.923008E38F);
        p91.yaw_rudder_SET(2.5380525E38F);
        p91.throttle_SET(-9.262702E37F);
        p91.aux1_SET(4.375151E37F);
        p91.aux2_SET(7.019293E37F);
        p91.aux3_SET(2.7483028E38F);
        p91.aux4_SET(3.1514092E38F);
        p91.mode_SET(MAV_MODE.MAV_MODE_AUTO_ARMED);
        p91.nav_mode_SET((char)116);
        CommunicationChannel.instance.send(p91); //===============================
        HIL_RC_INPUTS_RAW p92 = CommunicationChannel.instance.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.time_usec_SET(2848978460337672915L);
        p92.chan1_raw_SET((char)33906);
        p92.chan2_raw_SET((char)62824);
        p92.chan3_raw_SET((char)16259);
        p92.chan4_raw_SET((char)20796);
        p92.chan5_raw_SET((char)28052);
        p92.chan6_raw_SET((char)22461);
        p92.chan7_raw_SET((char)28563);
        p92.chan8_raw_SET((char)54360);
        p92.chan9_raw_SET((char)38241);
        p92.chan10_raw_SET((char)3845);
        p92.chan11_raw_SET((char)21490);
        p92.chan12_raw_SET((char)27411);
        p92.rssi_SET((char)241);
        CommunicationChannel.instance.send(p92); //===============================
        HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.instance.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.time_usec_SET(6900179356019170319L);
        p93.controls_SET(new float[16], 0);
        p93.mode_SET(MAV_MODE.MAV_MODE_AUTO_DISARMED);
        p93.flags_SET(5517517464149283803L);
        CommunicationChannel.instance.send(p93); //===============================
        OPTICAL_FLOW p100 = CommunicationChannel.instance.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.time_usec_SET(6100026083993990441L);
        p100.sensor_id_SET((char)93);
        p100.flow_x_SET((short)9334);
        p100.flow_y_SET((short)3565);
        p100.flow_comp_m_x_SET(-1.4149343E38F);
        p100.flow_comp_m_y_SET(2.5454538E37F);
        p100.quality_SET((char)21);
        p100.ground_distance_SET(1.662417E38F);
        p100.flow_rate_x_SET(-3.0502955E38F, PH);
        p100.flow_rate_y_SET(6.4428403E37F, PH);
        CommunicationChannel.instance.send(p100); //===============================
        GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.instance.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.usec_SET(5597195682825007587L);
        p101.x_SET(-2.8802539E38F);
        p101.y_SET(1.1019583E38F);
        p101.z_SET(2.5139907E37F);
        p101.roll_SET(1.4042271E38F);
        p101.pitch_SET(1.5441667E38F);
        p101.yaw_SET(-3.1856003E38F);
        CommunicationChannel.instance.send(p101); //===============================
        VISION_POSITION_ESTIMATE p102 = CommunicationChannel.instance.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.usec_SET(1211092058523036558L);
        p102.x_SET(-6.204839E37F);
        p102.y_SET(2.500985E37F);
        p102.z_SET(1.9179525E38F);
        p102.roll_SET(2.7602528E38F);
        p102.pitch_SET(-1.1666821E38F);
        p102.yaw_SET(-3.6979733E37F);
        CommunicationChannel.instance.send(p102); //===============================
        VISION_SPEED_ESTIMATE p103 = CommunicationChannel.instance.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.usec_SET(2084472935113866370L);
        p103.x_SET(-6.2290824E37F);
        p103.y_SET(-4.832474E37F);
        p103.z_SET(3.1920219E38F);
        CommunicationChannel.instance.send(p103); //===============================
        VICON_POSITION_ESTIMATE p104 = CommunicationChannel.instance.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.usec_SET(1822516593925364646L);
        p104.x_SET(-3.2696875E38F);
        p104.y_SET(2.3420725E38F);
        p104.z_SET(-2.0850145E38F);
        p104.roll_SET(6.861049E37F);
        p104.pitch_SET(-1.1061474E38F);
        p104.yaw_SET(-2.8270835E38F);
        CommunicationChannel.instance.send(p104); //===============================
        HIGHRES_IMU p105 = CommunicationChannel.instance.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.time_usec_SET(8912223162836096010L);
        p105.xacc_SET(2.60998E36F);
        p105.yacc_SET(-2.4226302E38F);
        p105.zacc_SET(-2.0909014E38F);
        p105.xgyro_SET(-5.144195E37F);
        p105.ygyro_SET(-2.1757209E38F);
        p105.zgyro_SET(3.1015699E38F);
        p105.xmag_SET(1.788178E38F);
        p105.ymag_SET(9.715245E37F);
        p105.zmag_SET(-3.349311E38F);
        p105.abs_pressure_SET(-6.8075814E37F);
        p105.diff_pressure_SET(-6.824454E37F);
        p105.pressure_alt_SET(2.45943E38F);
        p105.temperature_SET(3.3299857E38F);
        p105.fields_updated_SET((char)19118);
        CommunicationChannel.instance.send(p105); //===============================
        OPTICAL_FLOW_RAD p106 = CommunicationChannel.instance.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.time_usec_SET(7685585502418019997L);
        p106.sensor_id_SET((char)73);
        p106.integration_time_us_SET(514546723L);
        p106.integrated_x_SET(1.0409611E38F);
        p106.integrated_y_SET(2.556317E37F);
        p106.integrated_xgyro_SET(-3.2947308E38F);
        p106.integrated_ygyro_SET(3.1458066E38F);
        p106.integrated_zgyro_SET(2.4256539E38F);
        p106.temperature_SET((short)27181);
        p106.quality_SET((char)244);
        p106.time_delta_distance_us_SET(1821772035L);
        p106.distance_SET(-2.9953827E38F);
        CommunicationChannel.instance.send(p106); //===============================
        HIL_SENSOR p107 = CommunicationChannel.instance.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.time_usec_SET(572601528431927836L);
        p107.xacc_SET(2.7918808E38F);
        p107.yacc_SET(2.0789182E38F);
        p107.zacc_SET(9.983557E37F);
        p107.xgyro_SET(-2.3867336E38F);
        p107.ygyro_SET(2.1972287E38F);
        p107.zgyro_SET(5.3911066E37F);
        p107.xmag_SET(1.7565324E37F);
        p107.ymag_SET(2.0976052E38F);
        p107.zmag_SET(1.3686025E38F);
        p107.abs_pressure_SET(1.7273413E38F);
        p107.diff_pressure_SET(-2.0402806E38F);
        p107.pressure_alt_SET(2.9571936E38F);
        p107.temperature_SET(1.6260099E38F);
        p107.fields_updated_SET(1262114145L);
        CommunicationChannel.instance.send(p107); //===============================
        SIM_STATE p108 = CommunicationChannel.instance.new_SIM_STATE();
        PH.setPack(p108);
        p108.q1_SET(-9.552708E37F);
        p108.q2_SET(-1.2784204E38F);
        p108.q3_SET(1.9742273E38F);
        p108.q4_SET(-1.4278479E38F);
        p108.roll_SET(9.566818E37F);
        p108.pitch_SET(1.2813356E38F);
        p108.yaw_SET(-2.2527139E38F);
        p108.xacc_SET(-1.7652838E38F);
        p108.yacc_SET(4.7847127E37F);
        p108.zacc_SET(-7.1812275E37F);
        p108.xgyro_SET(1.2377035E38F);
        p108.ygyro_SET(-4.87661E37F);
        p108.zgyro_SET(-2.772068E38F);
        p108.lat_SET(2.7523828E38F);
        p108.lon_SET(-2.425111E38F);
        p108.alt_SET(-1.2747562E38F);
        p108.std_dev_horz_SET(-5.971409E37F);
        p108.std_dev_vert_SET(-6.210152E36F);
        p108.vn_SET(2.9985433E38F);
        p108.ve_SET(2.3868958E38F);
        p108.vd_SET(1.8779651E38F);
        CommunicationChannel.instance.send(p108); //===============================
        RADIO_STATUS p109 = CommunicationChannel.instance.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.rssi_SET((char)138);
        p109.remrssi_SET((char)142);
        p109.txbuf_SET((char)127);
        p109.noise_SET((char)248);
        p109.remnoise_SET((char)83);
        p109.rxerrors_SET((char)51207);
        p109.fixed__SET((char)15991);
        CommunicationChannel.instance.send(p109); //===============================
        FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.instance.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.target_network_SET((char)51);
        p110.target_system_SET((char)107);
        p110.target_component_SET((char)232);
        p110.payload_SET(new char[251], 0);
        CommunicationChannel.instance.send(p110); //===============================
        TIMESYNC p111 = CommunicationChannel.instance.new_TIMESYNC();
        PH.setPack(p111);
        p111.tc1_SET(6068401979282861325L);
        p111.ts1_SET(-8090384915912584126L);
        CommunicationChannel.instance.send(p111); //===============================
        CAMERA_TRIGGER p112 = CommunicationChannel.instance.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.time_usec_SET(2327845352631192333L);
        p112.seq_SET(2489422282L);
        CommunicationChannel.instance.send(p112); //===============================
        HIL_GPS p113 = CommunicationChannel.instance.new_HIL_GPS();
        PH.setPack(p113);
        p113.time_usec_SET(2630235119102051759L);
        p113.fix_type_SET((char)113);
        p113.lat_SET(123459960);
        p113.lon_SET(238454841);
        p113.alt_SET(-1269863350);
        p113.eph_SET((char)30821);
        p113.epv_SET((char)35561);
        p113.vel_SET((char)18231);
        p113.vn_SET((short) -24455);
        p113.ve_SET((short) -8214);
        p113.vd_SET((short) -23462);
        p113.cog_SET((char)32964);
        p113.satellites_visible_SET((char)142);
        CommunicationChannel.instance.send(p113); //===============================
        HIL_OPTICAL_FLOW p114 = CommunicationChannel.instance.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.time_usec_SET(5133660709461952298L);
        p114.sensor_id_SET((char)20);
        p114.integration_time_us_SET(850081730L);
        p114.integrated_x_SET(-3.3249395E37F);
        p114.integrated_y_SET(3.3378212E38F);
        p114.integrated_xgyro_SET(1.5482593E38F);
        p114.integrated_ygyro_SET(-2.8436056E38F);
        p114.integrated_zgyro_SET(2.9198706E37F);
        p114.temperature_SET((short)22053);
        p114.quality_SET((char)250);
        p114.time_delta_distance_us_SET(920241592L);
        p114.distance_SET(-1.547683E38F);
        CommunicationChannel.instance.send(p114); //===============================
        HIL_STATE_QUATERNION p115 = CommunicationChannel.instance.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.time_usec_SET(2877212874877979306L);
        p115.attitude_quaternion_SET(new float[4], 0);
        p115.rollspeed_SET(2.718206E38F);
        p115.pitchspeed_SET(-3.2971888E38F);
        p115.yawspeed_SET(-2.8314757E38F);
        p115.lat_SET(1672427465);
        p115.lon_SET(-2143377768);
        p115.alt_SET(-405831378);
        p115.vx_SET((short) -16447);
        p115.vy_SET((short)27752);
        p115.vz_SET((short)26566);
        p115.ind_airspeed_SET((char)12980);
        p115.true_airspeed_SET((char)63159);
        p115.xacc_SET((short)29569);
        p115.yacc_SET((short) -5424);
        p115.zacc_SET((short)26316);
        CommunicationChannel.instance.send(p115); //===============================
        SCALED_IMU2 p116 = CommunicationChannel.instance.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.time_boot_ms_SET(2060371070L);
        p116.xacc_SET((short) -13157);
        p116.yacc_SET((short)10867);
        p116.zacc_SET((short) -29452);
        p116.xgyro_SET((short)23093);
        p116.ygyro_SET((short)7933);
        p116.zgyro_SET((short) -24519);
        p116.xmag_SET((short) -9997);
        p116.ymag_SET((short) -22300);
        p116.zmag_SET((short) -21692);
        CommunicationChannel.instance.send(p116); //===============================
        LOG_REQUEST_LIST p117 = CommunicationChannel.instance.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.target_system_SET((char)115);
        p117.target_component_SET((char)234);
        p117.start_SET((char)27794);
        p117.end_SET((char)28613);
        CommunicationChannel.instance.send(p117); //===============================
        LOG_ENTRY p118 = CommunicationChannel.instance.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.id_SET((char)43238);
        p118.num_logs_SET((char)1682);
        p118.last_log_num_SET((char)40839);
        p118.time_utc_SET(268723828L);
        p118.size_SET(1858309808L);
        CommunicationChannel.instance.send(p118); //===============================
        LOG_REQUEST_DATA p119 = CommunicationChannel.instance.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.target_system_SET((char)225);
        p119.target_component_SET((char)72);
        p119.id_SET((char)20349);
        p119.ofs_SET(2462525356L);
        p119.count_SET(3470462376L);
        CommunicationChannel.instance.send(p119); //===============================
        LOG_DATA p120 = CommunicationChannel.instance.new_LOG_DATA();
        PH.setPack(p120);
        p120.id_SET((char)19080);
        p120.ofs_SET(1191319670L);
        p120.count_SET((char)58);
        p120.data__SET(new char[90], 0);
        CommunicationChannel.instance.send(p120); //===============================
        LOG_ERASE p121 = CommunicationChannel.instance.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_system_SET((char)75);
        p121.target_component_SET((char)40);
        CommunicationChannel.instance.send(p121); //===============================
        LOG_REQUEST_END p122 = CommunicationChannel.instance.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_system_SET((char)152);
        p122.target_component_SET((char)212);
        CommunicationChannel.instance.send(p122); //===============================
        GPS_INJECT_DATA p123 = CommunicationChannel.instance.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.target_system_SET((char)100);
        p123.target_component_SET((char)252);
        p123.len_SET((char)54);
        p123.data__SET(new char[110], 0);
        CommunicationChannel.instance.send(p123); //===============================
        GPS2_RAW p124 = CommunicationChannel.instance.new_GPS2_RAW();
        PH.setPack(p124);
        p124.time_usec_SET(8807469866526857855L);
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX);
        p124.lat_SET(-2003858319);
        p124.lon_SET(1461311109);
        p124.alt_SET(-1544550288);
        p124.eph_SET((char)5583);
        p124.epv_SET((char)61262);
        p124.vel_SET((char)32404);
        p124.cog_SET((char)55103);
        p124.satellites_visible_SET((char)94);
        p124.dgps_numch_SET((char)97);
        p124.dgps_age_SET(454108842L);
        CommunicationChannel.instance.send(p124); //===============================
        POWER_STATUS p125 = CommunicationChannel.instance.new_POWER_STATUS();
        PH.setPack(p125);
        p125.Vcc_SET((char)38335);
        p125.Vservo_SET((char)51773);
        p125.flags_SET((MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED));
        CommunicationChannel.instance.send(p125); //===============================
        SERIAL_CONTROL p126 = CommunicationChannel.instance.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2);
        p126.flags_SET((SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI |
                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING));
        p126.timeout_SET((char)11066);
        p126.baudrate_SET(1590022562L);
        p126.count_SET((char)168);
        p126.data__SET(new char[70], 0);
        CommunicationChannel.instance.send(p126); //===============================
        GPS_RTK p127 = CommunicationChannel.instance.new_GPS_RTK();
        PH.setPack(p127);
        p127.time_last_baseline_ms_SET(1603210771L);
        p127.rtk_receiver_id_SET((char)180);
        p127.wn_SET((char)42174);
        p127.tow_SET(1449652672L);
        p127.rtk_health_SET((char)154);
        p127.rtk_rate_SET((char)158);
        p127.nsats_SET((char)91);
        p127.baseline_coords_type_SET((char)75);
        p127.baseline_a_mm_SET(1157193709);
        p127.baseline_b_mm_SET(858836570);
        p127.baseline_c_mm_SET(-1300842649);
        p127.accuracy_SET(3828556074L);
        p127.iar_num_hypotheses_SET(-146209952);
        CommunicationChannel.instance.send(p127); //===============================
        GPS2_RTK p128 = CommunicationChannel.instance.new_GPS2_RTK();
        PH.setPack(p128);
        p128.time_last_baseline_ms_SET(4056352343L);
        p128.rtk_receiver_id_SET((char)244);
        p128.wn_SET((char)31768);
        p128.tow_SET(827371019L);
        p128.rtk_health_SET((char)200);
        p128.rtk_rate_SET((char)103);
        p128.nsats_SET((char)181);
        p128.baseline_coords_type_SET((char)243);
        p128.baseline_a_mm_SET(2121333354);
        p128.baseline_b_mm_SET(1923325390);
        p128.baseline_c_mm_SET(246060150);
        p128.accuracy_SET(3970520188L);
        p128.iar_num_hypotheses_SET(-1521763094);
        CommunicationChannel.instance.send(p128); //===============================
        SCALED_IMU3 p129 = CommunicationChannel.instance.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.time_boot_ms_SET(2740981284L);
        p129.xacc_SET((short)4397);
        p129.yacc_SET((short)6056);
        p129.zacc_SET((short) -22874);
        p129.xgyro_SET((short)20188);
        p129.ygyro_SET((short)2558);
        p129.zgyro_SET((short) -5684);
        p129.xmag_SET((short)13405);
        p129.ymag_SET((short)28030);
        p129.zmag_SET((short) -29072);
        CommunicationChannel.instance.send(p129); //===============================
        DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.instance.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.type_SET((char)119);
        p130.size_SET(2953439236L);
        p130.width_SET((char)3217);
        p130.height_SET((char)10746);
        p130.packets_SET((char)54159);
        p130.payload_SET((char)51);
        p130.jpg_quality_SET((char)193);
        CommunicationChannel.instance.send(p130); //===============================
        ENCAPSULATED_DATA p131 = CommunicationChannel.instance.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.seqnr_SET((char)35272);
        p131.data__SET(new char[253], 0);
        CommunicationChannel.instance.send(p131); //===============================
        DISTANCE_SENSOR p132 = CommunicationChannel.instance.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.time_boot_ms_SET(1056448489L);
        p132.min_distance_SET((char)20504);
        p132.max_distance_SET((char)6497);
        p132.current_distance_SET((char)30512);
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER);
        p132.id_SET((char)142);
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_225);
        p132.covariance_SET((char)65);
        CommunicationChannel.instance.send(p132); //===============================
        TERRAIN_REQUEST p133 = CommunicationChannel.instance.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.lat_SET(925460509);
        p133.lon_SET(898721273);
        p133.grid_spacing_SET((char)63317);
        p133.mask_SET(8970514439941801266L);
        CommunicationChannel.instance.send(p133); //===============================
        TERRAIN_DATA p134 = CommunicationChannel.instance.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.lat_SET(1699193475);
        p134.lon_SET(1925055341);
        p134.grid_spacing_SET((char)62423);
        p134.gridbit_SET((char)45);
        p134.data__SET(new short[16], 0);
        CommunicationChannel.instance.send(p134); //===============================
        TERRAIN_CHECK p135 = CommunicationChannel.instance.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lat_SET(65664277);
        p135.lon_SET(1339656435);
        CommunicationChannel.instance.send(p135); //===============================
        TERRAIN_REPORT p136 = CommunicationChannel.instance.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.lat_SET(-1941403680);
        p136.lon_SET(-2064141615);
        p136.spacing_SET((char)28039);
        p136.terrain_height_SET(-2.6678447E38F);
        p136.current_height_SET(-1.2227857E38F);
        p136.pending_SET((char)3562);
        p136.loaded_SET((char)15519);
        CommunicationChannel.instance.send(p136); //===============================
        SCALED_PRESSURE2 p137 = CommunicationChannel.instance.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.time_boot_ms_SET(1557976688L);
        p137.press_abs_SET(-1.4974478E38F);
        p137.press_diff_SET(-1.4496271E38F);
        p137.temperature_SET((short) -11755);
        CommunicationChannel.instance.send(p137); //===============================
        ATT_POS_MOCAP p138 = CommunicationChannel.instance.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.time_usec_SET(5912661625366844428L);
        p138.q_SET(new float[4], 0);
        p138.x_SET(-3.3916832E38F);
        p138.y_SET(7.113535E36F);
        p138.z_SET(3.0921744E38F);
        CommunicationChannel.instance.send(p138); //===============================
        SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.instance.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.time_usec_SET(5074012716078455051L);
        p139.group_mlx_SET((char)74);
        p139.target_system_SET((char)164);
        p139.target_component_SET((char)178);
        p139.controls_SET(new float[8], 0);
        CommunicationChannel.instance.send(p139); //===============================
        ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.instance.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.time_usec_SET(5942097114798295791L);
        p140.group_mlx_SET((char)100);
        p140.controls_SET(new float[8], 0);
        CommunicationChannel.instance.send(p140); //===============================
        ALTITUDE p141 = CommunicationChannel.instance.new_ALTITUDE();
        PH.setPack(p141);
        p141.time_usec_SET(8521378797617220803L);
        p141.altitude_monotonic_SET(8.512712E37F);
        p141.altitude_amsl_SET(-1.6263018E38F);
        p141.altitude_local_SET(2.2454804E38F);
        p141.altitude_relative_SET(-5.417122E36F);
        p141.altitude_terrain_SET(-2.6921493E38F);
        p141.bottom_clearance_SET(1.8014824E38F);
        CommunicationChannel.instance.send(p141); //===============================
        RESOURCE_REQUEST p142 = CommunicationChannel.instance.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.request_id_SET((char)33);
        p142.uri_type_SET((char)60);
        p142.uri_SET(new char[120], 0);
        p142.transfer_type_SET((char)234);
        p142.storage_SET(new char[120], 0);
        CommunicationChannel.instance.send(p142); //===============================
        SCALED_PRESSURE3 p143 = CommunicationChannel.instance.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.time_boot_ms_SET(2886926915L);
        p143.press_abs_SET(-6.4777884E36F);
        p143.press_diff_SET(1.7731828E38F);
        p143.temperature_SET((short)3159);
        CommunicationChannel.instance.send(p143); //===============================
        FOLLOW_TARGET p144 = CommunicationChannel.instance.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.timestamp_SET(4290683208399898957L);
        p144.est_capabilities_SET((char)74);
        p144.lat_SET(-1738350926);
        p144.lon_SET(581520076);
        p144.alt_SET(-2.7320834E38F);
        p144.vel_SET(new float[3], 0);
        p144.acc_SET(new float[3], 0);
        p144.attitude_q_SET(new float[4], 0);
        p144.rates_SET(new float[3], 0);
        p144.position_cov_SET(new float[3], 0);
        p144.custom_state_SET(606056809021422942L);
        CommunicationChannel.instance.send(p144); //===============================
        CONTROL_SYSTEM_STATE p146 = CommunicationChannel.instance.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.time_usec_SET(6831453613659582319L);
        p146.x_acc_SET(2.4151667E38F);
        p146.y_acc_SET(2.3640272E38F);
        p146.z_acc_SET(2.7637513E38F);
        p146.x_vel_SET(9.310534E37F);
        p146.y_vel_SET(-2.2412247E38F);
        p146.z_vel_SET(9.541572E37F);
        p146.x_pos_SET(-2.3284036E38F);
        p146.y_pos_SET(-6.639862E37F);
        p146.z_pos_SET(-2.9385178E37F);
        p146.airspeed_SET(2.1550322E38F);
        p146.vel_variance_SET(new float[3], 0);
        p146.pos_variance_SET(new float[3], 0);
        p146.q_SET(new float[4], 0);
        p146.roll_rate_SET(-1.2081177E38F);
        p146.pitch_rate_SET(-3.0565322E38F);
        p146.yaw_rate_SET(-1.7388217E38F);
        CommunicationChannel.instance.send(p146); //===============================
        BATTERY_STATUS p147 = CommunicationChannel.instance.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.id_SET((char)244);
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD);
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LION);
        p147.temperature_SET((short)10746);
        p147.voltages_SET(new char[10], 0);
        p147.current_battery_SET((short) -11778);
        p147.current_consumed_SET(1671614925);
        p147.energy_consumed_SET(1988465139);
        p147.battery_remaining_SET((byte)49);
        CommunicationChannel.instance.send(p147); //===============================
        AUTOPILOT_VERSION p148 = CommunicationChannel.instance.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.capabilities_SET((MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT));
        p148.flight_sw_version_SET(398323736L);
        p148.middleware_sw_version_SET(724916225L);
        p148.os_sw_version_SET(1204093550L);
        p148.board_version_SET(1644752345L);
        p148.flight_custom_version_SET(new char[8], 0);
        p148.middleware_custom_version_SET(new char[8], 0);
        p148.os_custom_version_SET(new char[8], 0);
        p148.vendor_id_SET((char)47281);
        p148.product_id_SET((char)41710);
        p148.uid_SET(9023479076141966781L);
        p148.uid2_SET(new char[18], 0, PH);
        CommunicationChannel.instance.send(p148); //===============================
        LANDING_TARGET p149 = CommunicationChannel.instance.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.time_usec_SET(2147963853668837150L);
        p149.target_num_SET((char)160);
        p149.frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED);
        p149.angle_x_SET(-3.550912E37F);
        p149.angle_y_SET(-2.3293842E38F);
        p149.distance_SET(1.4826919E37F);
        p149.size_x_SET(-7.5815074E37F);
        p149.size_y_SET(1.9420833E38F);
        p149.x_SET(3.2521146E38F, PH);
        p149.y_SET(-2.8210755E38F, PH);
        p149.z_SET(2.5938537E38F, PH);
        p149.q_SET(new float[4], 0, PH);
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON);
        p149.position_valid_SET((char)11, PH);
        CommunicationChannel.instance.send(p149); //===============================
        SENS_POWER p201 = CommunicationChannel.instance.new_SENS_POWER();
        PH.setPack(p201);
        p201.adc121_vspb_volt_SET(2.502787E38F);
        p201.adc121_cspb_amp_SET(2.6902087E38F);
        p201.adc121_cs1_amp_SET(-2.5710293E38F);
        p201.adc121_cs2_amp_SET(-2.8241446E38F);
        CommunicationChannel.instance.send(p201); //===============================
        SENS_MPPT p202 = CommunicationChannel.instance.new_SENS_MPPT();
        PH.setPack(p202);
        p202.mppt_timestamp_SET(6730817560011246443L);
        p202.mppt1_volt_SET(-1.1310216E38F);
        p202.mppt1_amp_SET(1.2748526E38F);
        p202.mppt1_pwm_SET((char)21040);
        p202.mppt1_status_SET((char)38);
        p202.mppt2_volt_SET(2.6565529E38F);
        p202.mppt2_amp_SET(-9.840203E37F);
        p202.mppt2_pwm_SET((char)50299);
        p202.mppt2_status_SET((char)74);
        p202.mppt3_volt_SET(3.320312E38F);
        p202.mppt3_amp_SET(1.1839492E38F);
        p202.mppt3_pwm_SET((char)1716);
        p202.mppt3_status_SET((char)182);
        CommunicationChannel.instance.send(p202); //===============================
        ASLCTRL_DATA p203 = CommunicationChannel.instance.new_ASLCTRL_DATA();
        PH.setPack(p203);
        p203.timestamp_SET(2827697605504709355L);
        p203.aslctrl_mode_SET((char)41);
        p203.h_SET(2.4348345E38F);
        p203.hRef_SET(2.7586278E38F);
        p203.hRef_t_SET(-3.6469796E37F);
        p203.PitchAngle_SET(-1.2692991E38F);
        p203.PitchAngleRef_SET(4.4692355E37F);
        p203.q_SET(9.397512E36F);
        p203.qRef_SET(2.8941895E38F);
        p203.uElev_SET(2.0424603E38F);
        p203.uThrot_SET(1.4121519E38F);
        p203.uThrot2_SET(-2.7057022E38F);
        p203.nZ_SET(-1.725998E38F);
        p203.AirspeedRef_SET(-2.7022623E38F);
        p203.SpoilersEngaged_SET((char)133);
        p203.YawAngle_SET(2.3312403E38F);
        p203.YawAngleRef_SET(-6.0419174E37F);
        p203.RollAngle_SET(1.1196145E38F);
        p203.RollAngleRef_SET(-2.837604E38F);
        p203.p_SET(2.4015089E38F);
        p203.pRef_SET(-4.72921E37F);
        p203.r_SET(1.8568414E38F);
        p203.rRef_SET(-2.390207E38F);
        p203.uAil_SET(2.9373503E37F);
        p203.uRud_SET(-4.233138E37F);
        CommunicationChannel.instance.send(p203); //===============================
        ASLCTRL_DEBUG p204 = CommunicationChannel.instance.new_ASLCTRL_DEBUG();
        PH.setPack(p204);
        p204.i32_1_SET(2480876151L);
        p204.i8_1_SET((char)244);
        p204.i8_2_SET((char)157);
        p204.f_1_SET(9.774386E37F);
        p204.f_2_SET(-1.0226812E38F);
        p204.f_3_SET(1.9501121E38F);
        p204.f_4_SET(-1.7729016E38F);
        p204.f_5_SET(2.4735796E38F);
        p204.f_6_SET(-5.8059855E36F);
        p204.f_7_SET(-2.4320169E38F);
        p204.f_8_SET(-2.9946816E38F);
        CommunicationChannel.instance.send(p204); //===============================
        ASLUAV_STATUS p205 = CommunicationChannel.instance.new_ASLUAV_STATUS();
        PH.setPack(p205);
        p205.LED_status_SET((char)190);
        p205.SATCOM_status_SET((char)40);
        p205.Servo_status_SET(new char[8], 0);
        p205.Motor_rpm_SET(2.553366E38F);
        CommunicationChannel.instance.send(p205); //===============================
        EKF_EXT p206 = CommunicationChannel.instance.new_EKF_EXT();
        PH.setPack(p206);
        p206.timestamp_SET(3871822518670618163L);
        p206.Windspeed_SET(6.7248776E36F);
        p206.WindDir_SET(9.515851E37F);
        p206.WindZ_SET(-2.3881827E38F);
        p206.Airspeed_SET(1.6581383E38F);
        p206.beta_SET(-1.569041E38F);
        p206.alpha_SET(7.2728735E37F);
        CommunicationChannel.instance.send(p206); //===============================
        ASL_OBCTRL p207 = CommunicationChannel.instance.new_ASL_OBCTRL();
        PH.setPack(p207);
        p207.timestamp_SET(4373043111493537646L);
        p207.uElev_SET(2.7952146E38F);
        p207.uThrot_SET(-2.9598356E38F);
        p207.uThrot2_SET(2.2756397E38F);
        p207.uAilL_SET(1.929883E38F);
        p207.uAilR_SET(3.2603326E38F);
        p207.uRud_SET(-2.616729E38F);
        p207.obctrl_status_SET((char)70);
        CommunicationChannel.instance.send(p207); //===============================
        SENS_ATMOS p208 = CommunicationChannel.instance.new_SENS_ATMOS();
        PH.setPack(p208);
        p208.TempAmbient_SET(-1.4719995E38F);
        p208.Humidity_SET(2.038058E38F);
        CommunicationChannel.instance.send(p208); //===============================
        SENS_BATMON p209 = CommunicationChannel.instance.new_SENS_BATMON();
        PH.setPack(p209);
        p209.temperature_SET(1.4433216E38F);
        p209.voltage_SET((char)26635);
        p209.current_SET((short) -31030);
        p209.SoC_SET((char)88);
        p209.batterystatus_SET((char)12281);
        p209.serialnumber_SET((char)28881);
        p209.hostfetcontrol_SET((char)47866);
        p209.cellvoltage1_SET((char)34603);
        p209.cellvoltage2_SET((char)187);
        p209.cellvoltage3_SET((char)60243);
        p209.cellvoltage4_SET((char)8518);
        p209.cellvoltage5_SET((char)8660);
        p209.cellvoltage6_SET((char)31184);
        CommunicationChannel.instance.send(p209); //===============================
        FW_SOARING_DATA p210 = CommunicationChannel.instance.new_FW_SOARING_DATA();
        PH.setPack(p210);
        p210.timestamp_SET(2061326956783591963L);
        p210.timestampModeChanged_SET(889383436527186012L);
        p210.xW_SET(1.1026571E38F);
        p210.xR_SET(2.2440555E38F);
        p210.xLat_SET(1.1212425E38F);
        p210.xLon_SET(2.2819518E38F);
        p210.VarW_SET(-1.4976089E37F);
        p210.VarR_SET(-2.09973E38F);
        p210.VarLat_SET(-2.1449E38F);
        p210.VarLon_SET(-1.4143701E38F);
        p210.LoiterRadius_SET(1.3913749E38F);
        p210.LoiterDirection_SET(2.4209938E38F);
        p210.DistToSoarPoint_SET(-2.455981E38F);
        p210.vSinkExp_SET(-2.36886E38F);
        p210.z1_LocalUpdraftSpeed_SET(-2.1709877E38F);
        p210.z2_DeltaRoll_SET(5.1969812E35F);
        p210.z1_exp_SET(2.0140946E38F);
        p210.z2_exp_SET(-2.1563148E38F);
        p210.ThermalGSNorth_SET(3.061072E38F);
        p210.ThermalGSEast_SET(5.54694E37F);
        p210.TSE_dot_SET(1.4063555E38F);
        p210.DebugVar1_SET(1.0563633E38F);
        p210.DebugVar2_SET(-1.6755619E38F);
        p210.ControlMode_SET((char)138);
        p210.valid_SET((char)150);
        CommunicationChannel.instance.send(p210); //===============================
        SENSORPOD_STATUS p211 = CommunicationChannel.instance.new_SENSORPOD_STATUS();
        PH.setPack(p211);
        p211.timestamp_SET(3426264838210566550L);
        p211.visensor_rate_1_SET((char)34);
        p211.visensor_rate_2_SET((char)18);
        p211.visensor_rate_3_SET((char)52);
        p211.visensor_rate_4_SET((char)247);
        p211.recording_nodes_count_SET((char)11);
        p211.cpu_temp_SET((char)252);
        p211.free_space_SET((char)64454);
        CommunicationChannel.instance.send(p211); //===============================
        SENS_POWER_BOARD p212 = CommunicationChannel.instance.new_SENS_POWER_BOARD();
        PH.setPack(p212);
        p212.timestamp_SET(3057494503295546271L);
        p212.pwr_brd_status_SET((char)107);
        p212.pwr_brd_led_status_SET((char)199);
        p212.pwr_brd_system_volt_SET(-2.7048745E38F);
        p212.pwr_brd_servo_volt_SET(1.1789122E38F);
        p212.pwr_brd_mot_l_amp_SET(2.7018145E38F);
        p212.pwr_brd_mot_r_amp_SET(-2.5632439E38F);
        p212.pwr_brd_servo_1_amp_SET(-2.4546267E38F);
        p212.pwr_brd_servo_2_amp_SET(-2.685315E38F);
        p212.pwr_brd_servo_3_amp_SET(1.9840734E38F);
        p212.pwr_brd_servo_4_amp_SET(-2.7223352E38F);
        p212.pwr_brd_aux_amp_SET(-3.1222835E38F);
        CommunicationChannel.instance.send(p212); //===============================
        ESTIMATOR_STATUS p230 = CommunicationChannel.instance.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.time_usec_SET(2333140766762434475L);
        p230.flags_SET((ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS));
        p230.vel_ratio_SET(-1.2813467E38F);
        p230.pos_horiz_ratio_SET(1.114477E38F);
        p230.pos_vert_ratio_SET(-1.4345661E37F);
        p230.mag_ratio_SET(-3.0999848E38F);
        p230.hagl_ratio_SET(1.4298526E37F);
        p230.tas_ratio_SET(2.9326874E38F);
        p230.pos_horiz_accuracy_SET(-2.5192824E38F);
        p230.pos_vert_accuracy_SET(-2.972689E36F);
        CommunicationChannel.instance.send(p230); //===============================
        WIND_COV p231 = CommunicationChannel.instance.new_WIND_COV();
        PH.setPack(p231);
        p231.time_usec_SET(2315884616477470415L);
        p231.wind_x_SET(1.6527959E38F);
        p231.wind_y_SET(2.8904693E38F);
        p231.wind_z_SET(-6.157618E36F);
        p231.var_horiz_SET(-2.0703235E38F);
        p231.var_vert_SET(-4.1467916E37F);
        p231.wind_alt_SET(7.2795505E37F);
        p231.horiz_accuracy_SET(3.0264122E38F);
        p231.vert_accuracy_SET(-3.1717539E38F);
        CommunicationChannel.instance.send(p231); //===============================
        GPS_INPUT p232 = CommunicationChannel.instance.new_GPS_INPUT();
        PH.setPack(p232);
        p232.time_usec_SET(8345931304793644525L);
        p232.gps_id_SET((char)56);
        p232.ignore_flags_SET((GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP));
        p232.time_week_ms_SET(228521094L);
        p232.time_week_SET((char)5012);
        p232.fix_type_SET((char)213);
        p232.lat_SET(-2076396420);
        p232.lon_SET(1874591314);
        p232.alt_SET(-2.9762912E37F);
        p232.hdop_SET(-1.3823063E38F);
        p232.vdop_SET(-2.4436288E38F);
        p232.vn_SET(3.1695259E38F);
        p232.ve_SET(-1.0749788E38F);
        p232.vd_SET(-2.3264185E38F);
        p232.speed_accuracy_SET(8.364194E37F);
        p232.horiz_accuracy_SET(3.0984324E38F);
        p232.vert_accuracy_SET(-3.3873825E38F);
        p232.satellites_visible_SET((char)11);
        CommunicationChannel.instance.send(p232); //===============================
        GPS_RTCM_DATA p233 = CommunicationChannel.instance.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.flags_SET((char)60);
        p233.len_SET((char)136);
        p233.data__SET(new char[180], 0);
        CommunicationChannel.instance.send(p233); //===============================
        HIGH_LATENCY p234 = CommunicationChannel.instance.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED));
        p234.custom_mode_SET(3821169046L);
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF);
        p234.roll_SET((short)28887);
        p234.pitch_SET((short) -25180);
        p234.heading_SET((char)64585);
        p234.throttle_SET((byte)122);
        p234.heading_sp_SET((short) -31335);
        p234.latitude_SET(5014693);
        p234.longitude_SET(-200299127);
        p234.altitude_amsl_SET((short) -160);
        p234.altitude_sp_SET((short) -2236);
        p234.airspeed_SET((char)106);
        p234.airspeed_sp_SET((char)33);
        p234.groundspeed_SET((char)182);
        p234.climb_rate_SET((byte) - 17);
        p234.gps_nsat_SET((char)172);
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS);
        p234.battery_remaining_SET((char)26);
        p234.temperature_SET((byte)33);
        p234.temperature_air_SET((byte) - 51);
        p234.failsafe_SET((char)168);
        p234.wp_num_SET((char)151);
        p234.wp_distance_SET((char)12011);
        CommunicationChannel.instance.send(p234); //===============================
        VIBRATION p241 = CommunicationChannel.instance.new_VIBRATION();
        PH.setPack(p241);
        p241.time_usec_SET(2849469162245205847L);
        p241.vibration_x_SET(8.2621226E36F);
        p241.vibration_y_SET(-2.1002958E38F);
        p241.vibration_z_SET(-1.0706713E38F);
        p241.clipping_0_SET(3284651187L);
        p241.clipping_1_SET(4085164200L);
        p241.clipping_2_SET(3263003180L);
        CommunicationChannel.instance.send(p241); //===============================
        HOME_POSITION p242 = CommunicationChannel.instance.new_HOME_POSITION();
        PH.setPack(p242);
        p242.latitude_SET(-1609809300);
        p242.longitude_SET(-1800135937);
        p242.altitude_SET(-768437312);
        p242.x_SET(-1.1557594E38F);
        p242.y_SET(3.2618417E38F);
        p242.z_SET(-2.4726851E38F);
        p242.q_SET(new float[4], 0);
        p242.approach_x_SET(1.6630534E38F);
        p242.approach_y_SET(-5.2413483E37F);
        p242.approach_z_SET(-1.675494E38F);
        p242.time_usec_SET(2916371935919561943L, PH);
        CommunicationChannel.instance.send(p242); //===============================
        SET_HOME_POSITION p243 = CommunicationChannel.instance.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.target_system_SET((char)8);
        p243.latitude_SET(389647441);
        p243.longitude_SET(143671703);
        p243.altitude_SET(-605404795);
        p243.x_SET(-3.030476E37F);
        p243.y_SET(-6.8993836E37F);
        p243.z_SET(1.0731422E38F);
        p243.q_SET(new float[4], 0);
        p243.approach_x_SET(2.7815806E38F);
        p243.approach_y_SET(-2.7823288E38F);
        p243.approach_z_SET(-4.6939834E37F);
        p243.time_usec_SET(4224080110976673379L, PH);
        CommunicationChannel.instance.send(p243); //===============================
        MESSAGE_INTERVAL p244 = CommunicationChannel.instance.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.message_id_SET((char)45938);
        p244.interval_us_SET(578892900);
        CommunicationChannel.instance.send(p244); //===============================
        EXTENDED_SYS_STATE p245 = CommunicationChannel.instance.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_FW);
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING);
        CommunicationChannel.instance.send(p245); //===============================
        ADSB_VEHICLE p246 = CommunicationChannel.instance.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.ICAO_address_SET(1870118201L);
        p246.lat_SET(-581366977);
        p246.lon_SET(565823151);
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
        p246.altitude_SET(-100785220);
        p246.heading_SET((char)21525);
        p246.hor_velocity_SET((char)50580);
        p246.ver_velocity_SET((short)25485);
        p246.callsign_SET("DEMO", PH);
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_GLIDER);
        p246.tslc_SET((char)61);
        p246.flags_SET((ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS |
                        ADSB_FLAGS.ADSB_FLAGS_SIMULATED));
        p246.squawk_SET((char)23168);
        CommunicationChannel.instance.send(p246); //===============================
        COLLISION p247 = CommunicationChannel.instance.new_COLLISION();
        PH.setPack(p247);
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
        p247.id_SET(1265895370L);
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_RTL);
        p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW);
        p247.time_to_minimum_delta_SET(-3.3735096E37F);
        p247.altitude_minimum_delta_SET(-4.423348E37F);
        p247.horizontal_minimum_delta_SET(-2.5427947E38F);
        CommunicationChannel.instance.send(p247); //===============================
        V2_EXTENSION p248 = CommunicationChannel.instance.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.target_network_SET((char)15);
        p248.target_system_SET((char)105);
        p248.target_component_SET((char)66);
        p248.message_type_SET((char)15207);
        p248.payload_SET(new char[249], 0);
        CommunicationChannel.instance.send(p248); //===============================
        MEMORY_VECT p249 = CommunicationChannel.instance.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.address_SET((char)20159);
        p249.ver_SET((char)132);
        p249.type_SET((char)189);
        p249.value_SET(new byte[32], 0);
        CommunicationChannel.instance.send(p249); //===============================
        DEBUG_VECT p250 = CommunicationChannel.instance.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.name_SET("DEMO", PH);
        p250.time_usec_SET(6433460022528508711L);
        p250.x_SET(-9.710837E37F);
        p250.y_SET(6.6062333E37F);
        p250.z_SET(2.1021584E38F);
        CommunicationChannel.instance.send(p250); //===============================
        NAMED_VALUE_FLOAT p251 = CommunicationChannel.instance.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.time_boot_ms_SET(1521898395L);
        p251.name_SET("DEMO", PH);
        p251.value_SET(1.261563E38F);
        CommunicationChannel.instance.send(p251); //===============================
        NAMED_VALUE_INT p252 = CommunicationChannel.instance.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.time_boot_ms_SET(434667074L);
        p252.name_SET("DEMO", PH);
        p252.value_SET(629609297);
        CommunicationChannel.instance.send(p252); //===============================
        STATUSTEXT p253 = CommunicationChannel.instance.new_STATUSTEXT();
        PH.setPack(p253);
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_EMERGENCY);
        p253.text_SET("DEMO", PH);
        CommunicationChannel.instance.send(p253); //===============================
        DEBUG p254 = CommunicationChannel.instance.new_DEBUG();
        PH.setPack(p254);
        p254.time_boot_ms_SET(47022485L);
        p254.ind_SET((char)243);
        p254.value_SET(2.8147104E37F);
        CommunicationChannel.instance.send(p254); //===============================
        SETUP_SIGNING p256 = CommunicationChannel.instance.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.target_system_SET((char)10);
        p256.target_component_SET((char)82);
        p256.secret_key_SET(new char[32], 0);
        p256.initial_timestamp_SET(2316397473800215295L);
        CommunicationChannel.instance.send(p256); //===============================
        BUTTON_CHANGE p257 = CommunicationChannel.instance.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.time_boot_ms_SET(3669114759L);
        p257.last_change_ms_SET(3340329780L);
        p257.state_SET((char)58);
        CommunicationChannel.instance.send(p257); //===============================
        PLAY_TUNE p258 = CommunicationChannel.instance.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.target_system_SET((char)243);
        p258.target_component_SET((char)222);
        p258.tune_SET("DEMO", PH);
        CommunicationChannel.instance.send(p258); //===============================
        CAMERA_INFORMATION p259 = CommunicationChannel.instance.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.time_boot_ms_SET(796611972L);
        p259.vendor_name_SET(new char[32], 0);
        p259.model_name_SET(new char[32], 0);
        p259.firmware_version_SET(1346289340L);
        p259.focal_length_SET(2.9480606E38F);
        p259.sensor_size_h_SET(-4.7712837E37F);
        p259.sensor_size_v_SET(5.4983163E37F);
        p259.resolution_h_SET((char)39875);
        p259.resolution_v_SET((char)6552);
        p259.lens_id_SET((char)251);
        p259.flags_SET((CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE |
                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE |
                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE));
        p259.cam_definition_version_SET((char)21535);
        p259.cam_definition_uri_SET("DEMO", PH);
        CommunicationChannel.instance.send(p259); //===============================
        CAMERA_SETTINGS p260 = CommunicationChannel.instance.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.time_boot_ms_SET(2509026010L);
        p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY);
        CommunicationChannel.instance.send(p260); //===============================
        STORAGE_INFORMATION p261 = CommunicationChannel.instance.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.time_boot_ms_SET(3588584485L);
        p261.storage_id_SET((char)56);
        p261.storage_count_SET((char)186);
        p261.status_SET((char)223);
        p261.total_capacity_SET(-1.5032601E38F);
        p261.used_capacity_SET(-1.2382367E38F);
        p261.available_capacity_SET(-2.1253517E38F);
        p261.read_speed_SET(2.4775606E38F);
        p261.write_speed_SET(-2.385145E38F);
        CommunicationChannel.instance.send(p261); //===============================
        CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.instance.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.time_boot_ms_SET(2088162337L);
        p262.image_status_SET((char)12);
        p262.video_status_SET((char)145);
        p262.image_interval_SET(-2.3398047E38F);
        p262.recording_time_ms_SET(2322704121L);
        p262.available_capacity_SET(-1.2133955E38F);
        CommunicationChannel.instance.send(p262); //===============================
        CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.instance.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.time_boot_ms_SET(2001625248L);
        p263.time_utc_SET(3649809420816639987L);
        p263.camera_id_SET((char)100);
        p263.lat_SET(-45801407);
        p263.lon_SET(958917173);
        p263.alt_SET(-1426293454);
        p263.relative_alt_SET(-1689061709);
        p263.q_SET(new float[4], 0);
        p263.image_index_SET(-1522359826);
        p263.capture_result_SET((byte) - 124);
        p263.file_url_SET("DEMO", PH);
        CommunicationChannel.instance.send(p263); //===============================
        FLIGHT_INFORMATION p264 = CommunicationChannel.instance.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.time_boot_ms_SET(1036822373L);
        p264.arming_time_utc_SET(3383125806376561184L);
        p264.takeoff_time_utc_SET(3773743610887790L);
        p264.flight_uuid_SET(5792381613448413921L);
        CommunicationChannel.instance.send(p264); //===============================
        MOUNT_ORIENTATION p265 = CommunicationChannel.instance.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.time_boot_ms_SET(709036090L);
        p265.roll_SET(2.4212565E38F);
        p265.pitch_SET(1.95941E38F);
        p265.yaw_SET(-2.1280647E38F);
        CommunicationChannel.instance.send(p265); //===============================
        LOGGING_DATA p266 = CommunicationChannel.instance.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.target_system_SET((char)162);
        p266.target_component_SET((char)220);
        p266.sequence_SET((char)8790);
        p266.length_SET((char)50);
        p266.first_message_offset_SET((char)93);
        p266.data__SET(new char[249], 0);
        CommunicationChannel.instance.send(p266); //===============================
        LOGGING_DATA_ACKED p267 = CommunicationChannel.instance.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.target_system_SET((char)207);
        p267.target_component_SET((char)119);
        p267.sequence_SET((char)36802);
        p267.length_SET((char)87);
        p267.first_message_offset_SET((char)148);
        p267.data__SET(new char[249], 0);
        CommunicationChannel.instance.send(p267); //===============================
        LOGGING_ACK p268 = CommunicationChannel.instance.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.target_system_SET((char)39);
        p268.target_component_SET((char)139);
        p268.sequence_SET((char)6873);
        CommunicationChannel.instance.send(p268); //===============================
        VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.instance.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.camera_id_SET((char)146);
        p269.status_SET((char)55);
        p269.framerate_SET(-3.2262655E38F);
        p269.resolution_h_SET((char)61464);
        p269.resolution_v_SET((char)16677);
        p269.bitrate_SET(3150865679L);
        p269.rotation_SET((char)53004);
        p269.uri_SET("DEMO", PH);
        CommunicationChannel.instance.send(p269); //===============================
        SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.instance.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.target_system_SET((char)52);
        p270.target_component_SET((char)98);
        p270.camera_id_SET((char)140);
        p270.framerate_SET(1.9884864E38F);
        p270.resolution_h_SET((char)42190);
        p270.resolution_v_SET((char)7097);
        p270.bitrate_SET(806119387L);
        p270.rotation_SET((char)33518);
        p270.uri_SET("DEMO", PH);
        CommunicationChannel.instance.send(p270); //===============================
        WIFI_CONFIG_AP p299 = CommunicationChannel.instance.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.ssid_SET("DEMO", PH);
        p299.password_SET("DEMO", PH);
        CommunicationChannel.instance.send(p299); //===============================
        PROTOCOL_VERSION p300 = CommunicationChannel.instance.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.version_SET((char)32369);
        p300.min_version_SET((char)40159);
        p300.max_version_SET((char)21571);
        p300.spec_version_hash_SET(new char[8], 0);
        p300.library_version_hash_SET(new char[8], 0);
        CommunicationChannel.instance.send(p300); //===============================
        UAVCAN_NODE_STATUS p310 = CommunicationChannel.instance.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.time_usec_SET(1965064191222022245L);
        p310.uptime_sec_SET(1776811107L);
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR);
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE);
        p310.sub_mode_SET((char)109);
        p310.vendor_specific_status_code_SET((char)22419);
        CommunicationChannel.instance.send(p310); //===============================
        UAVCAN_NODE_INFO p311 = CommunicationChannel.instance.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.time_usec_SET(6236390647133088536L);
        p311.uptime_sec_SET(1594750613L);
        p311.name_SET("DEMO", PH);
        p311.hw_version_major_SET((char)209);
        p311.hw_version_minor_SET((char)221);
        p311.hw_unique_id_SET(new char[16], 0);
        p311.sw_version_major_SET((char)149);
        p311.sw_version_minor_SET((char)63);
        p311.sw_vcs_commit_SET(1185622828L);
        CommunicationChannel.instance.send(p311); //===============================
        PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.instance.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.target_system_SET((char)183);
        p320.target_component_SET((char)37);
        p320.param_id_SET("DEMO", PH);
        p320.param_index_SET((short) -3704);
        CommunicationChannel.instance.send(p320); //===============================
        PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.instance.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_system_SET((char)242);
        p321.target_component_SET((char)26);
        CommunicationChannel.instance.send(p321); //===============================
        PARAM_EXT_VALUE p322 = CommunicationChannel.instance.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_id_SET("DEMO", PH);
        p322.param_value_SET("DEMO", PH);
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64);
        p322.param_count_SET((char)46833);
        p322.param_index_SET((char)46785);
        CommunicationChannel.instance.send(p322); //===============================
        PARAM_EXT_SET p323 = CommunicationChannel.instance.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.target_system_SET((char)255);
        p323.target_component_SET((char)131);
        p323.param_id_SET("DEMO", PH);
        p323.param_value_SET("DEMO", PH);
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32);
        CommunicationChannel.instance.send(p323); //===============================
        PARAM_EXT_ACK p324 = CommunicationChannel.instance.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_id_SET("DEMO", PH);
        p324.param_value_SET("DEMO", PH);
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM);
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_ACCEPTED);
        CommunicationChannel.instance.send(p324); //===============================
        OBSTACLE_DISTANCE p330 = CommunicationChannel.instance.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.time_usec_SET(1069053047273162478L);
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN);
        p330.distances_SET(new char[72], 0);
        p330.increment_SET((char)8);
        p330.min_distance_SET((char)58633);
        p330.max_distance_SET((char)4352);
        CommunicationChannel.instance.send(p330); //===============================
    }
}

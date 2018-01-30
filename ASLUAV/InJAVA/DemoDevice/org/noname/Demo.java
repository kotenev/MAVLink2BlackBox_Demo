package org.noname;
import org.unirail.BlackBox.Host.Pack.Meta.Field.Bounds;

public class Demo extends  DemoDevice
{
    public static void main(String[] args)
    {
        final Bounds.Inside PH = new Bounds.Inside();
        LoopBackDemoChannel.instance.on_HEARTBEAT.add((src, ph, pack) ->
        {
            @MAV_TYPE int  type = pack.type_GET();
            @MAV_AUTOPILOT int  autopilot = pack.autopilot_GET();
            @MAV_MODE_FLAG int  base_mode = pack.base_mode_GET();
            long  custom_mode = pack.custom_mode_GET();
            @MAV_STATE int  system_status = pack.system_status_GET();
            char  mavlink_version = pack.mavlink_version_GET();
        });
        LoopBackDemoChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
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
        LoopBackDemoChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
        {
            long  time_unix_usec = pack.time_unix_usec_GET();
            long  time_boot_ms = pack.time_boot_ms_GET();
        });
        LoopBackDemoChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
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
        LoopBackDemoChannel.instance.on_PING.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            long  seq = pack.seq_GET();
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
        });
        LoopBackDemoChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  control_request = pack.control_request_GET();
            char  version = pack.version_GET();
            String passkey = pack.passkey_TRY(ph);
        });
        LoopBackDemoChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
        {
            char  gcs_system_id = pack.gcs_system_id_GET();
            char  control_request = pack.control_request_GET();
            char  ack = pack.ack_GET();
        });
        LoopBackDemoChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
        {
            String key = pack.key_TRY(ph);
        });
        LoopBackDemoChannel.instance.on_SET_MODE.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            @MAV_MODE int  base_mode = pack.base_mode_GET();
            long  custom_mode = pack.custom_mode_GET();
        });
        LoopBackDemoChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            String param_id = pack.param_id_TRY(ph);
            short  param_index = pack.param_index_GET();
        });
        LoopBackDemoChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
        });
        LoopBackDemoChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
        {
            String param_id = pack.param_id_TRY(ph);
            float  param_value = pack.param_value_GET();
            @MAV_PARAM_TYPE int  param_type = pack.param_type_GET();
            char  param_count = pack.param_count_GET();
            char  param_index = pack.param_index_GET();
        });
        LoopBackDemoChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            String param_id = pack.param_id_TRY(ph);
            float  param_value = pack.param_value_GET();
            @MAV_PARAM_TYPE int  param_type = pack.param_type_GET();
        });
        LoopBackDemoChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
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
        LoopBackDemoChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
        {
            char  satellites_visible = pack.satellites_visible_GET();
            char[]  satellite_prn = pack.satellite_prn_GET();
            char[]  satellite_used = pack.satellite_used_GET();
            char[]  satellite_elevation = pack.satellite_elevation_GET();
            char[]  satellite_azimuth = pack.satellite_azimuth_GET();
            char[]  satellite_snr = pack.satellite_snr_GET();
        });
        LoopBackDemoChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
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
        LoopBackDemoChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
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
        LoopBackDemoChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            short  press_abs = pack.press_abs_GET();
            short  press_diff1 = pack.press_diff1_GET();
            short  press_diff2 = pack.press_diff2_GET();
            short  temperature = pack.temperature_GET();
        });
        LoopBackDemoChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            float  press_abs = pack.press_abs_GET();
            float  press_diff = pack.press_diff_GET();
            short  temperature = pack.temperature_GET();
        });
        LoopBackDemoChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            float  roll = pack.roll_GET();
            float  pitch = pack.pitch_GET();
            float  yaw = pack.yaw_GET();
            float  rollspeed = pack.rollspeed_GET();
            float  pitchspeed = pack.pitchspeed_GET();
            float  yawspeed = pack.yawspeed_GET();
        });
        LoopBackDemoChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
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
        LoopBackDemoChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            float  x = pack.x_GET();
            float  y = pack.y_GET();
            float  z = pack.z_GET();
            float  vx = pack.vx_GET();
            float  vy = pack.vy_GET();
            float  vz = pack.vz_GET();
        });
        LoopBackDemoChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
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
        LoopBackDemoChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
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
        LoopBackDemoChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
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
        LoopBackDemoChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
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
        LoopBackDemoChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            short  start_index = pack.start_index_GET();
            short  end_index = pack.end_index_GET();
            @MAV_MISSION_TYPE int  mission_type = pack.mission_type_GET();
        });
        LoopBackDemoChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            short  start_index = pack.start_index_GET();
            short  end_index = pack.end_index_GET();
            @MAV_MISSION_TYPE int  mission_type = pack.mission_type_GET();
        });
        LoopBackDemoChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
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
        LoopBackDemoChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  seq = pack.seq_GET();
            @MAV_MISSION_TYPE int  mission_type = pack.mission_type_GET();
        });
        LoopBackDemoChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  seq = pack.seq_GET();
        });
        LoopBackDemoChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
        {
            char  seq = pack.seq_GET();
        });
        LoopBackDemoChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            @MAV_MISSION_TYPE int  mission_type = pack.mission_type_GET();
        });
        LoopBackDemoChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  count = pack.count_GET();
            @MAV_MISSION_TYPE int  mission_type = pack.mission_type_GET();
        });
        LoopBackDemoChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            @MAV_MISSION_TYPE int  mission_type = pack.mission_type_GET();
        });
        LoopBackDemoChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
        {
            char  seq = pack.seq_GET();
        });
        LoopBackDemoChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            @MAV_MISSION_RESULT int  type = pack.type_GET();
            @MAV_MISSION_TYPE int  mission_type = pack.mission_type_GET();
        });
        LoopBackDemoChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            int  latitude = pack.latitude_GET();
            int  longitude = pack.longitude_GET();
            int  altitude = pack.altitude_GET();
            long  time_usec = pack.time_usec_TRY(ph);
        });
        LoopBackDemoChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            int  latitude = pack.latitude_GET();
            int  longitude = pack.longitude_GET();
            int  altitude = pack.altitude_GET();
            long  time_usec = pack.time_usec_TRY(ph);
        });
        LoopBackDemoChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
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
        LoopBackDemoChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  seq = pack.seq_GET();
            @MAV_MISSION_TYPE int  mission_type = pack.mission_type_GET();
        });
        LoopBackDemoChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
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
        LoopBackDemoChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
        {
            @MAV_FRAME int  frame = pack.frame_GET();
            float  p1x = pack.p1x_GET();
            float  p1y = pack.p1y_GET();
            float  p1z = pack.p1z_GET();
            float  p2x = pack.p2x_GET();
            float  p2y = pack.p2y_GET();
            float  p2z = pack.p2z_GET();
        });
        LoopBackDemoChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            float[]  q = pack.q_GET();
            float  rollspeed = pack.rollspeed_GET();
            float  pitchspeed = pack.pitchspeed_GET();
            float  yawspeed = pack.yawspeed_GET();
            float[]  covariance = pack.covariance_GET();
        });
        LoopBackDemoChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
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
        LoopBackDemoChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
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
        LoopBackDemoChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
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
        LoopBackDemoChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
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
        LoopBackDemoChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  req_stream_id = pack.req_stream_id_GET();
            char  req_message_rate = pack.req_message_rate_GET();
            char  start_stop = pack.start_stop_GET();
        });
        LoopBackDemoChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
        {
            char  stream_id = pack.stream_id_GET();
            char  message_rate = pack.message_rate_GET();
            char  on_off = pack.on_off_GET();
        });
        LoopBackDemoChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
        {
            char  target = pack.target_GET();
            short  x = pack.x_GET();
            short  y = pack.y_GET();
            short  z = pack.z_GET();
            short  r = pack.r_GET();
            char  buttons = pack.buttons_GET();
        });
        LoopBackDemoChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
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
        LoopBackDemoChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
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
        LoopBackDemoChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
        {
            float  airspeed = pack.airspeed_GET();
            float  groundspeed = pack.groundspeed_GET();
            short  heading = pack.heading_GET();
            char  throttle = pack.throttle_GET();
            float  alt = pack.alt_GET();
            float  climb = pack.climb_GET();
        });
        LoopBackDemoChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
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
        LoopBackDemoChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
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
        LoopBackDemoChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
        {
            @MAV_CMD int  command = pack.command_GET();
            @MAV_RESULT int  result = pack.result_GET();
            char  progress = pack.progress_TRY(ph);
            int  result_param2 = pack.result_param2_TRY(ph);
            char  target_system = pack.target_system_TRY(ph);
            char  target_component = pack.target_component_TRY(ph);
        });
        LoopBackDemoChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            float  roll = pack.roll_GET();
            float  pitch = pack.pitch_GET();
            float  yaw = pack.yaw_GET();
            float  thrust = pack.thrust_GET();
            char  mode_switch = pack.mode_switch_GET();
            char  manual_override_switch = pack.manual_override_switch_GET();
        });
        LoopBackDemoChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
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
        LoopBackDemoChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            char  type_mask = pack.type_mask_GET();
            float[]  q = pack.q_GET();
            float  body_roll_rate = pack.body_roll_rate_GET();
            float  body_pitch_rate = pack.body_pitch_rate_GET();
            float  body_yaw_rate = pack.body_yaw_rate_GET();
            float  thrust = pack.thrust_GET();
        });
        LoopBackDemoChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
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
        LoopBackDemoChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
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
        LoopBackDemoChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
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
        LoopBackDemoChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            float  x = pack.x_GET();
            float  y = pack.y_GET();
            float  z = pack.z_GET();
            float  roll = pack.roll_GET();
            float  pitch = pack.pitch_GET();
            float  yaw = pack.yaw_GET();
        });
        LoopBackDemoChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
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
        LoopBackDemoChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
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
        LoopBackDemoChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
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
        LoopBackDemoChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            float[]  controls = pack.controls_GET();
            @MAV_MODE int  mode = pack.mode_GET();
            long  flags = pack.flags_GET();
        });
        LoopBackDemoChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
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
        LoopBackDemoChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            long  usec = pack.usec_GET();
            float  x = pack.x_GET();
            float  y = pack.y_GET();
            float  z = pack.z_GET();
            float  roll = pack.roll_GET();
            float  pitch = pack.pitch_GET();
            float  yaw = pack.yaw_GET();
        });
        LoopBackDemoChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            long  usec = pack.usec_GET();
            float  x = pack.x_GET();
            float  y = pack.y_GET();
            float  z = pack.z_GET();
            float  roll = pack.roll_GET();
            float  pitch = pack.pitch_GET();
            float  yaw = pack.yaw_GET();
        });
        LoopBackDemoChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
        {
            long  usec = pack.usec_GET();
            float  x = pack.x_GET();
            float  y = pack.y_GET();
            float  z = pack.z_GET();
        });
        LoopBackDemoChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            long  usec = pack.usec_GET();
            float  x = pack.x_GET();
            float  y = pack.y_GET();
            float  z = pack.z_GET();
            float  roll = pack.roll_GET();
            float  pitch = pack.pitch_GET();
            float  yaw = pack.yaw_GET();
        });
        LoopBackDemoChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
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
        LoopBackDemoChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
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
        LoopBackDemoChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
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
        LoopBackDemoChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
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
        LoopBackDemoChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
        {
            char  rssi = pack.rssi_GET();
            char  remrssi = pack.remrssi_GET();
            char  txbuf = pack.txbuf_GET();
            char  noise = pack.noise_GET();
            char  remnoise = pack.remnoise_GET();
            char  rxerrors = pack.rxerrors_GET();
            char  fixed_ = pack.fixed__GET();
        });
        LoopBackDemoChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
        {
            char  target_network = pack.target_network_GET();
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char[]  payload = pack.payload_GET();
        });
        LoopBackDemoChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
        {
            long  tc1 = pack.tc1_GET();
            long  ts1 = pack.ts1_GET();
        });
        LoopBackDemoChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            long  seq = pack.seq_GET();
        });
        LoopBackDemoChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
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
        LoopBackDemoChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
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
        LoopBackDemoChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
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
        LoopBackDemoChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
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
        LoopBackDemoChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  start = pack.start_GET();
            char  end = pack.end_GET();
        });
        LoopBackDemoChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
        {
            char  id = pack.id_GET();
            char  num_logs = pack.num_logs_GET();
            char  last_log_num = pack.last_log_num_GET();
            long  time_utc = pack.time_utc_GET();
            long  size = pack.size_GET();
        });
        LoopBackDemoChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  id = pack.id_GET();
            long  ofs = pack.ofs_GET();
            long  count = pack.count_GET();
        });
        LoopBackDemoChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
        {
            char  id = pack.id_GET();
            long  ofs = pack.ofs_GET();
            char  count = pack.count_GET();
            char[]  data_ = pack.data__GET();
        });
        LoopBackDemoChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
        });
        LoopBackDemoChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
        });
        LoopBackDemoChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  len = pack.len_GET();
            char[]  data_ = pack.data__GET();
        });
        LoopBackDemoChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
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
        LoopBackDemoChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
        {
            char  Vcc = pack.Vcc_GET();
            char  Vservo = pack.Vservo_GET();
            @MAV_POWER_STATUS int  flags = pack.flags_GET();
        });
        LoopBackDemoChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
        {
            @SERIAL_CONTROL_DEV int  device = pack.device_GET();
            @SERIAL_CONTROL_FLAG int  flags = pack.flags_GET();
            char  timeout = pack.timeout_GET();
            long  baudrate = pack.baudrate_GET();
            char  count = pack.count_GET();
            char[]  data_ = pack.data__GET();
        });
        LoopBackDemoChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
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
        LoopBackDemoChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
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
        LoopBackDemoChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
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
        LoopBackDemoChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
        {
            char  type = pack.type_GET();
            long  size = pack.size_GET();
            char  width = pack.width_GET();
            char  height = pack.height_GET();
            char  packets = pack.packets_GET();
            char  payload = pack.payload_GET();
            char  jpg_quality = pack.jpg_quality_GET();
        });
        LoopBackDemoChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
        {
            char  seqnr = pack.seqnr_GET();
            char[]  data_ = pack.data__GET();
        });
        LoopBackDemoChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
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
        LoopBackDemoChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
        {
            int  lat = pack.lat_GET();
            int  lon = pack.lon_GET();
            char  grid_spacing = pack.grid_spacing_GET();
            long  mask = pack.mask_GET();
        });
        LoopBackDemoChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
        {
            int  lat = pack.lat_GET();
            int  lon = pack.lon_GET();
            char  grid_spacing = pack.grid_spacing_GET();
            char  gridbit = pack.gridbit_GET();
            short[]  data_ = pack.data__GET();
        });
        LoopBackDemoChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
        {
            int  lat = pack.lat_GET();
            int  lon = pack.lon_GET();
        });
        LoopBackDemoChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
        {
            int  lat = pack.lat_GET();
            int  lon = pack.lon_GET();
            char  spacing = pack.spacing_GET();
            float  terrain_height = pack.terrain_height_GET();
            float  current_height = pack.current_height_GET();
            char  pending = pack.pending_GET();
            char  loaded = pack.loaded_GET();
        });
        LoopBackDemoChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            float  press_abs = pack.press_abs_GET();
            float  press_diff = pack.press_diff_GET();
            short  temperature = pack.temperature_GET();
        });
        LoopBackDemoChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            float[]  q = pack.q_GET();
            float  x = pack.x_GET();
            float  y = pack.y_GET();
            float  z = pack.z_GET();
        });
        LoopBackDemoChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            char  group_mlx = pack.group_mlx_GET();
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            float[]  controls = pack.controls_GET();
        });
        LoopBackDemoChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            char  group_mlx = pack.group_mlx_GET();
            float[]  controls = pack.controls_GET();
        });
        LoopBackDemoChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            float  altitude_monotonic = pack.altitude_monotonic_GET();
            float  altitude_amsl = pack.altitude_amsl_GET();
            float  altitude_local = pack.altitude_local_GET();
            float  altitude_relative = pack.altitude_relative_GET();
            float  altitude_terrain = pack.altitude_terrain_GET();
            float  bottom_clearance = pack.bottom_clearance_GET();
        });
        LoopBackDemoChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
        {
            char  request_id = pack.request_id_GET();
            char  uri_type = pack.uri_type_GET();
            char[]  uri = pack.uri_GET();
            char  transfer_type = pack.transfer_type_GET();
            char[]  storage = pack.storage_GET();
        });
        LoopBackDemoChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            float  press_abs = pack.press_abs_GET();
            float  press_diff = pack.press_diff_GET();
            short  temperature = pack.temperature_GET();
        });
        LoopBackDemoChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
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
        LoopBackDemoChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
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
        LoopBackDemoChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
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
        LoopBackDemoChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
        {
            @MAV_PROTOCOL_CAPABILITY int  capabilities = pack.capabilities_GET();
            long  flight_sw_version = pack.flight_sw_version_GET();
            long  middleware_sw_version = pack.middleware_sw_version_GET();
            long  os_sw_version = pack.os_sw_version_GET();
            long  board_version = pack.board_version_GET();
            char[]  flight_custom_version = pack.flight_custom_version_GET();
            char[]  middleware_custom_version = pack.middleware_custom_version_GET();
            char[]  os_custom_version = pack.os_custom_version_GET();
            char  vendor_id = pack.vendor_id_GET();
            char  product_id = pack.product_id_GET();
            long  uid = pack.uid_GET();
            char[]  uid2 = pack.uid2_TRY(ph);
        });
        LoopBackDemoChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            char  target_num = pack.target_num_GET();
            @MAV_FRAME int  frame = pack.frame_GET();
            float  angle_x = pack.angle_x_GET();
            float  angle_y = pack.angle_y_GET();
            float  distance = pack.distance_GET();
            float  size_x = pack.size_x_GET();
            float  size_y = pack.size_y_GET();
            float  x = pack.x_TRY(ph);
            float  y = pack.y_TRY(ph);
            float  z = pack.z_TRY(ph);
            float[]  q = pack.q_TRY(ph);
            @LANDING_TARGET_TYPE int  type = pack.type_GET();
            char  position_valid = pack.position_valid_TRY(ph);
        });
        LoopBackDemoChannel.instance.on_SENS_POWER.add((src, ph, pack) ->
        {
            float  adc121_vspb_volt = pack.adc121_vspb_volt_GET();
            float  adc121_cspb_amp = pack.adc121_cspb_amp_GET();
            float  adc121_cs1_amp = pack.adc121_cs1_amp_GET();
            float  adc121_cs2_amp = pack.adc121_cs2_amp_GET();
        });
        LoopBackDemoChannel.instance.on_SENS_MPPT.add((src, ph, pack) ->
        {
            long  mppt_timestamp = pack.mppt_timestamp_GET();
            float  mppt1_volt = pack.mppt1_volt_GET();
            float  mppt1_amp = pack.mppt1_amp_GET();
            char  mppt1_pwm = pack.mppt1_pwm_GET();
            char  mppt1_status = pack.mppt1_status_GET();
            float  mppt2_volt = pack.mppt2_volt_GET();
            float  mppt2_amp = pack.mppt2_amp_GET();
            char  mppt2_pwm = pack.mppt2_pwm_GET();
            char  mppt2_status = pack.mppt2_status_GET();
            float  mppt3_volt = pack.mppt3_volt_GET();
            float  mppt3_amp = pack.mppt3_amp_GET();
            char  mppt3_pwm = pack.mppt3_pwm_GET();
            char  mppt3_status = pack.mppt3_status_GET();
        });
        LoopBackDemoChannel.instance.on_ASLCTRL_DATA.add((src, ph, pack) ->
        {
            long  timestamp = pack.timestamp_GET();
            char  aslctrl_mode = pack.aslctrl_mode_GET();
            float  h = pack.h_GET();
            float  hRef = pack.hRef_GET();
            float  hRef_t = pack.hRef_t_GET();
            float  PitchAngle = pack.PitchAngle_GET();
            float  PitchAngleRef = pack.PitchAngleRef_GET();
            float  q = pack.q_GET();
            float  qRef = pack.qRef_GET();
            float  uElev = pack.uElev_GET();
            float  uThrot = pack.uThrot_GET();
            float  uThrot2 = pack.uThrot2_GET();
            float  nZ = pack.nZ_GET();
            float  AirspeedRef = pack.AirspeedRef_GET();
            char  SpoilersEngaged = pack.SpoilersEngaged_GET();
            float  YawAngle = pack.YawAngle_GET();
            float  YawAngleRef = pack.YawAngleRef_GET();
            float  RollAngle = pack.RollAngle_GET();
            float  RollAngleRef = pack.RollAngleRef_GET();
            float  p = pack.p_GET();
            float  pRef = pack.pRef_GET();
            float  r = pack.r_GET();
            float  rRef = pack.rRef_GET();
            float  uAil = pack.uAil_GET();
            float  uRud = pack.uRud_GET();
        });
        LoopBackDemoChannel.instance.on_ASLCTRL_DEBUG.add((src, ph, pack) ->
        {
            long  i32_1 = pack.i32_1_GET();
            char  i8_1 = pack.i8_1_GET();
            char  i8_2 = pack.i8_2_GET();
            float  f_1 = pack.f_1_GET();
            float  f_2 = pack.f_2_GET();
            float  f_3 = pack.f_3_GET();
            float  f_4 = pack.f_4_GET();
            float  f_5 = pack.f_5_GET();
            float  f_6 = pack.f_6_GET();
            float  f_7 = pack.f_7_GET();
            float  f_8 = pack.f_8_GET();
        });
        LoopBackDemoChannel.instance.on_ASLUAV_STATUS.add((src, ph, pack) ->
        {
            char  LED_status = pack.LED_status_GET();
            char  SATCOM_status = pack.SATCOM_status_GET();
            char[]  Servo_status = pack.Servo_status_GET();
            float  Motor_rpm = pack.Motor_rpm_GET();
        });
        LoopBackDemoChannel.instance.on_EKF_EXT.add((src, ph, pack) ->
        {
            long  timestamp = pack.timestamp_GET();
            float  Windspeed = pack.Windspeed_GET();
            float  WindDir = pack.WindDir_GET();
            float  WindZ = pack.WindZ_GET();
            float  Airspeed = pack.Airspeed_GET();
            float  beta = pack.beta_GET();
            float  alpha = pack.alpha_GET();
        });
        LoopBackDemoChannel.instance.on_ASL_OBCTRL.add((src, ph, pack) ->
        {
            long  timestamp = pack.timestamp_GET();
            float  uElev = pack.uElev_GET();
            float  uThrot = pack.uThrot_GET();
            float  uThrot2 = pack.uThrot2_GET();
            float  uAilL = pack.uAilL_GET();
            float  uAilR = pack.uAilR_GET();
            float  uRud = pack.uRud_GET();
            char  obctrl_status = pack.obctrl_status_GET();
        });
        LoopBackDemoChannel.instance.on_SENS_ATMOS.add((src, ph, pack) ->
        {
            float  TempAmbient = pack.TempAmbient_GET();
            float  Humidity = pack.Humidity_GET();
        });
        LoopBackDemoChannel.instance.on_SENS_BATMON.add((src, ph, pack) ->
        {
            float  temperature = pack.temperature_GET();
            char  voltage = pack.voltage_GET();
            short  current = pack.current_GET();
            char  SoC = pack.SoC_GET();
            char  batterystatus = pack.batterystatus_GET();
            char  serialnumber = pack.serialnumber_GET();
            char  hostfetcontrol = pack.hostfetcontrol_GET();
            char  cellvoltage1 = pack.cellvoltage1_GET();
            char  cellvoltage2 = pack.cellvoltage2_GET();
            char  cellvoltage3 = pack.cellvoltage3_GET();
            char  cellvoltage4 = pack.cellvoltage4_GET();
            char  cellvoltage5 = pack.cellvoltage5_GET();
            char  cellvoltage6 = pack.cellvoltage6_GET();
        });
        LoopBackDemoChannel.instance.on_FW_SOARING_DATA.add((src, ph, pack) ->
        {
            long  timestamp = pack.timestamp_GET();
            long  timestampModeChanged = pack.timestampModeChanged_GET();
            float  xW = pack.xW_GET();
            float  xR = pack.xR_GET();
            float  xLat = pack.xLat_GET();
            float  xLon = pack.xLon_GET();
            float  VarW = pack.VarW_GET();
            float  VarR = pack.VarR_GET();
            float  VarLat = pack.VarLat_GET();
            float  VarLon = pack.VarLon_GET();
            float  LoiterRadius = pack.LoiterRadius_GET();
            float  LoiterDirection = pack.LoiterDirection_GET();
            float  DistToSoarPoint = pack.DistToSoarPoint_GET();
            float  vSinkExp = pack.vSinkExp_GET();
            float  z1_LocalUpdraftSpeed = pack.z1_LocalUpdraftSpeed_GET();
            float  z2_DeltaRoll = pack.z2_DeltaRoll_GET();
            float  z1_exp = pack.z1_exp_GET();
            float  z2_exp = pack.z2_exp_GET();
            float  ThermalGSNorth = pack.ThermalGSNorth_GET();
            float  ThermalGSEast = pack.ThermalGSEast_GET();
            float  TSE_dot = pack.TSE_dot_GET();
            float  DebugVar1 = pack.DebugVar1_GET();
            float  DebugVar2 = pack.DebugVar2_GET();
            char  ControlMode = pack.ControlMode_GET();
            char  valid = pack.valid_GET();
        });
        LoopBackDemoChannel.instance.on_SENSORPOD_STATUS.add((src, ph, pack) ->
        {
            long  timestamp = pack.timestamp_GET();
            char  visensor_rate_1 = pack.visensor_rate_1_GET();
            char  visensor_rate_2 = pack.visensor_rate_2_GET();
            char  visensor_rate_3 = pack.visensor_rate_3_GET();
            char  visensor_rate_4 = pack.visensor_rate_4_GET();
            char  recording_nodes_count = pack.recording_nodes_count_GET();
            char  cpu_temp = pack.cpu_temp_GET();
            char  free_space = pack.free_space_GET();
        });
        LoopBackDemoChannel.instance.on_SENS_POWER_BOARD.add((src, ph, pack) ->
        {
            long  timestamp = pack.timestamp_GET();
            char  pwr_brd_status = pack.pwr_brd_status_GET();
            char  pwr_brd_led_status = pack.pwr_brd_led_status_GET();
            float  pwr_brd_system_volt = pack.pwr_brd_system_volt_GET();
            float  pwr_brd_servo_volt = pack.pwr_brd_servo_volt_GET();
            float  pwr_brd_mot_l_amp = pack.pwr_brd_mot_l_amp_GET();
            float  pwr_brd_mot_r_amp = pack.pwr_brd_mot_r_amp_GET();
            float  pwr_brd_servo_1_amp = pack.pwr_brd_servo_1_amp_GET();
            float  pwr_brd_servo_2_amp = pack.pwr_brd_servo_2_amp_GET();
            float  pwr_brd_servo_3_amp = pack.pwr_brd_servo_3_amp_GET();
            float  pwr_brd_servo_4_amp = pack.pwr_brd_servo_4_amp_GET();
            float  pwr_brd_aux_amp = pack.pwr_brd_aux_amp_GET();
        });
        LoopBackDemoChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            @ESTIMATOR_STATUS_FLAGS int  flags = pack.flags_GET();
            float  vel_ratio = pack.vel_ratio_GET();
            float  pos_horiz_ratio = pack.pos_horiz_ratio_GET();
            float  pos_vert_ratio = pack.pos_vert_ratio_GET();
            float  mag_ratio = pack.mag_ratio_GET();
            float  hagl_ratio = pack.hagl_ratio_GET();
            float  tas_ratio = pack.tas_ratio_GET();
            float  pos_horiz_accuracy = pack.pos_horiz_accuracy_GET();
            float  pos_vert_accuracy = pack.pos_vert_accuracy_GET();
        });
        LoopBackDemoChannel.instance.on_WIND_COV.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            float  wind_x = pack.wind_x_GET();
            float  wind_y = pack.wind_y_GET();
            float  wind_z = pack.wind_z_GET();
            float  var_horiz = pack.var_horiz_GET();
            float  var_vert = pack.var_vert_GET();
            float  wind_alt = pack.wind_alt_GET();
            float  horiz_accuracy = pack.horiz_accuracy_GET();
            float  vert_accuracy = pack.vert_accuracy_GET();
        });
        LoopBackDemoChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            char  gps_id = pack.gps_id_GET();
            @GPS_INPUT_IGNORE_FLAGS int  ignore_flags = pack.ignore_flags_GET();
            long  time_week_ms = pack.time_week_ms_GET();
            char  time_week = pack.time_week_GET();
            char  fix_type = pack.fix_type_GET();
            int  lat = pack.lat_GET();
            int  lon = pack.lon_GET();
            float  alt = pack.alt_GET();
            float  hdop = pack.hdop_GET();
            float  vdop = pack.vdop_GET();
            float  vn = pack.vn_GET();
            float  ve = pack.ve_GET();
            float  vd = pack.vd_GET();
            float  speed_accuracy = pack.speed_accuracy_GET();
            float  horiz_accuracy = pack.horiz_accuracy_GET();
            float  vert_accuracy = pack.vert_accuracy_GET();
            char  satellites_visible = pack.satellites_visible_GET();
        });
        LoopBackDemoChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
        {
            char  flags = pack.flags_GET();
            char  len = pack.len_GET();
            char[]  data_ = pack.data__GET();
        });
        LoopBackDemoChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
        {
            @MAV_MODE_FLAG int  base_mode = pack.base_mode_GET();
            long  custom_mode = pack.custom_mode_GET();
            @MAV_LANDED_STATE int  landed_state = pack.landed_state_GET();
            short  roll = pack.roll_GET();
            short  pitch = pack.pitch_GET();
            char  heading = pack.heading_GET();
            byte  throttle = pack.throttle_GET();
            short  heading_sp = pack.heading_sp_GET();
            int  latitude = pack.latitude_GET();
            int  longitude = pack.longitude_GET();
            short  altitude_amsl = pack.altitude_amsl_GET();
            short  altitude_sp = pack.altitude_sp_GET();
            char  airspeed = pack.airspeed_GET();
            char  airspeed_sp = pack.airspeed_sp_GET();
            char  groundspeed = pack.groundspeed_GET();
            byte  climb_rate = pack.climb_rate_GET();
            char  gps_nsat = pack.gps_nsat_GET();
            @GPS_FIX_TYPE int  gps_fix_type = pack.gps_fix_type_GET();
            char  battery_remaining = pack.battery_remaining_GET();
            byte  temperature = pack.temperature_GET();
            byte  temperature_air = pack.temperature_air_GET();
            char  failsafe = pack.failsafe_GET();
            char  wp_num = pack.wp_num_GET();
            char  wp_distance = pack.wp_distance_GET();
        });
        LoopBackDemoChannel.instance.on_VIBRATION.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            float  vibration_x = pack.vibration_x_GET();
            float  vibration_y = pack.vibration_y_GET();
            float  vibration_z = pack.vibration_z_GET();
            long  clipping_0 = pack.clipping_0_GET();
            long  clipping_1 = pack.clipping_1_GET();
            long  clipping_2 = pack.clipping_2_GET();
        });
        LoopBackDemoChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
        {
            int  latitude = pack.latitude_GET();
            int  longitude = pack.longitude_GET();
            int  altitude = pack.altitude_GET();
            float  x = pack.x_GET();
            float  y = pack.y_GET();
            float  z = pack.z_GET();
            float[]  q = pack.q_GET();
            float  approach_x = pack.approach_x_GET();
            float  approach_y = pack.approach_y_GET();
            float  approach_z = pack.approach_z_GET();
            long  time_usec = pack.time_usec_TRY(ph);
        });
        LoopBackDemoChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            int  latitude = pack.latitude_GET();
            int  longitude = pack.longitude_GET();
            int  altitude = pack.altitude_GET();
            float  x = pack.x_GET();
            float  y = pack.y_GET();
            float  z = pack.z_GET();
            float[]  q = pack.q_GET();
            float  approach_x = pack.approach_x_GET();
            float  approach_y = pack.approach_y_GET();
            float  approach_z = pack.approach_z_GET();
            long  time_usec = pack.time_usec_TRY(ph);
        });
        LoopBackDemoChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
        {
            char  message_id = pack.message_id_GET();
            int  interval_us = pack.interval_us_GET();
        });
        LoopBackDemoChannel.instance.on_EXTENDED_SYS_STATE.add((src, ph, pack) ->
        {
            @MAV_VTOL_STATE int  vtol_state = pack.vtol_state_GET();
            @MAV_LANDED_STATE int  landed_state = pack.landed_state_GET();
        });
        LoopBackDemoChannel.instance.on_ADSB_VEHICLE.add((src, ph, pack) ->
        {
            long  ICAO_address = pack.ICAO_address_GET();
            int  lat = pack.lat_GET();
            int  lon = pack.lon_GET();
            @ADSB_ALTITUDE_TYPE int  altitude_type = pack.altitude_type_GET();
            int  altitude = pack.altitude_GET();
            char  heading = pack.heading_GET();
            char  hor_velocity = pack.hor_velocity_GET();
            short  ver_velocity = pack.ver_velocity_GET();
            String callsign = pack.callsign_TRY(ph);
            @ADSB_EMITTER_TYPE int  emitter_type = pack.emitter_type_GET();
            char  tslc = pack.tslc_GET();
            @ADSB_FLAGS int  flags = pack.flags_GET();
            char  squawk = pack.squawk_GET();
        });
        LoopBackDemoChannel.instance.on_COLLISION.add((src, ph, pack) ->
        {
            @MAV_COLLISION_SRC int  src_ = pack.src__GET();
            long  id = pack.id_GET();
            @MAV_COLLISION_ACTION int  action = pack.action_GET();
            @MAV_COLLISION_THREAT_LEVEL int  threat_level = pack.threat_level_GET();
            float  time_to_minimum_delta = pack.time_to_minimum_delta_GET();
            float  altitude_minimum_delta = pack.altitude_minimum_delta_GET();
            float  horizontal_minimum_delta = pack.horizontal_minimum_delta_GET();
        });
        LoopBackDemoChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
        {
            char  target_network = pack.target_network_GET();
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  message_type = pack.message_type_GET();
            char[]  payload = pack.payload_GET();
        });
        LoopBackDemoChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
        {
            char  address = pack.address_GET();
            char  ver = pack.ver_GET();
            char  type = pack.type_GET();
            byte[]  value = pack.value_GET();
        });
        LoopBackDemoChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
        {
            String name = pack.name_TRY(ph);
            long  time_usec = pack.time_usec_GET();
            float  x = pack.x_GET();
            float  y = pack.y_GET();
            float  z = pack.z_GET();
        });
        LoopBackDemoChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            String name = pack.name_TRY(ph);
            float  value = pack.value_GET();
        });
        LoopBackDemoChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            String name = pack.name_TRY(ph);
            int  value = pack.value_GET();
        });
        LoopBackDemoChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
        {
            @MAV_SEVERITY int  severity = pack.severity_GET();
            String text = pack.text_TRY(ph);
        });
        LoopBackDemoChannel.instance.on_DEBUG.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            char  ind = pack.ind_GET();
            float  value = pack.value_GET();
        });
        LoopBackDemoChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char[]  secret_key = pack.secret_key_GET();
            long  initial_timestamp = pack.initial_timestamp_GET();
        });
        LoopBackDemoChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            long  last_change_ms = pack.last_change_ms_GET();
            char  state = pack.state_GET();
        });
        LoopBackDemoChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            String tune = pack.tune_TRY(ph);
        });
        LoopBackDemoChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            char[]  vendor_name = pack.vendor_name_GET();
            char[]  model_name = pack.model_name_GET();
            long  firmware_version = pack.firmware_version_GET();
            float  focal_length = pack.focal_length_GET();
            float  sensor_size_h = pack.sensor_size_h_GET();
            float  sensor_size_v = pack.sensor_size_v_GET();
            char  resolution_h = pack.resolution_h_GET();
            char  resolution_v = pack.resolution_v_GET();
            char  lens_id = pack.lens_id_GET();
            @CAMERA_CAP_FLAGS int  flags = pack.flags_GET();
            char  cam_definition_version = pack.cam_definition_version_GET();
            String cam_definition_uri = pack.cam_definition_uri_TRY(ph);
        });
        LoopBackDemoChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            @CAMERA_MODE int  mode_id = pack.mode_id_GET();
        });
        LoopBackDemoChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            char  storage_id = pack.storage_id_GET();
            char  storage_count = pack.storage_count_GET();
            char  status = pack.status_GET();
            float  total_capacity = pack.total_capacity_GET();
            float  used_capacity = pack.used_capacity_GET();
            float  available_capacity = pack.available_capacity_GET();
            float  read_speed = pack.read_speed_GET();
            float  write_speed = pack.write_speed_GET();
        });
        LoopBackDemoChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            char  image_status = pack.image_status_GET();
            char  video_status = pack.video_status_GET();
            float  image_interval = pack.image_interval_GET();
            long  recording_time_ms = pack.recording_time_ms_GET();
            float  available_capacity = pack.available_capacity_GET();
        });
        LoopBackDemoChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            long  time_utc = pack.time_utc_GET();
            char  camera_id = pack.camera_id_GET();
            int  lat = pack.lat_GET();
            int  lon = pack.lon_GET();
            int  alt = pack.alt_GET();
            int  relative_alt = pack.relative_alt_GET();
            float[]  q = pack.q_GET();
            int  image_index = pack.image_index_GET();
            byte  capture_result = pack.capture_result_GET();
            String file_url = pack.file_url_TRY(ph);
        });
        LoopBackDemoChannel.instance.on_FLIGHT_INFORMATION.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            long  arming_time_utc = pack.arming_time_utc_GET();
            long  takeoff_time_utc = pack.takeoff_time_utc_GET();
            long  flight_uuid = pack.flight_uuid_GET();
        });
        LoopBackDemoChannel.instance.on_MOUNT_ORIENTATION.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            float  roll = pack.roll_GET();
            float  pitch = pack.pitch_GET();
            float  yaw = pack.yaw_GET();
        });
        LoopBackDemoChannel.instance.on_LOGGING_DATA.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  sequence = pack.sequence_GET();
            char  length = pack.length_GET();
            char  first_message_offset = pack.first_message_offset_GET();
            char[]  data_ = pack.data__GET();
        });
        LoopBackDemoChannel.instance.on_LOGGING_DATA_ACKED.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  sequence = pack.sequence_GET();
            char  length = pack.length_GET();
            char  first_message_offset = pack.first_message_offset_GET();
            char[]  data_ = pack.data__GET();
        });
        LoopBackDemoChannel.instance.on_LOGGING_ACK.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  sequence = pack.sequence_GET();
        });
        LoopBackDemoChannel.instance.on_VIDEO_STREAM_INFORMATION.add((src, ph, pack) ->
        {
            char  camera_id = pack.camera_id_GET();
            char  status = pack.status_GET();
            float  framerate = pack.framerate_GET();
            char  resolution_h = pack.resolution_h_GET();
            char  resolution_v = pack.resolution_v_GET();
            long  bitrate = pack.bitrate_GET();
            char  rotation = pack.rotation_GET();
            String uri = pack.uri_TRY(ph);
        });
        LoopBackDemoChannel.instance.on_SET_VIDEO_STREAM_SETTINGS.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  camera_id = pack.camera_id_GET();
            float  framerate = pack.framerate_GET();
            char  resolution_h = pack.resolution_h_GET();
            char  resolution_v = pack.resolution_v_GET();
            long  bitrate = pack.bitrate_GET();
            char  rotation = pack.rotation_GET();
            String uri = pack.uri_TRY(ph);
        });
        LoopBackDemoChannel.instance.on_WIFI_CONFIG_AP.add((src, ph, pack) ->
        {
            String ssid = pack.ssid_TRY(ph);
            String password = pack.password_TRY(ph);
        });
        LoopBackDemoChannel.instance.on_PROTOCOL_VERSION.add((src, ph, pack) ->
        {
            char  version = pack.version_GET();
            char  min_version = pack.min_version_GET();
            char  max_version = pack.max_version_GET();
            char[]  spec_version_hash = pack.spec_version_hash_GET();
            char[]  library_version_hash = pack.library_version_hash_GET();
        });
        LoopBackDemoChannel.instance.on_UAVCAN_NODE_STATUS.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            long  uptime_sec = pack.uptime_sec_GET();
            @UAVCAN_NODE_HEALTH int  health = pack.health_GET();
            @UAVCAN_NODE_MODE int  mode = pack.mode_GET();
            char  sub_mode = pack.sub_mode_GET();
            char  vendor_specific_status_code = pack.vendor_specific_status_code_GET();
        });
        LoopBackDemoChannel.instance.on_UAVCAN_NODE_INFO.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            long  uptime_sec = pack.uptime_sec_GET();
            String name = pack.name_TRY(ph);
            char  hw_version_major = pack.hw_version_major_GET();
            char  hw_version_minor = pack.hw_version_minor_GET();
            char[]  hw_unique_id = pack.hw_unique_id_GET();
            char  sw_version_major = pack.sw_version_major_GET();
            char  sw_version_minor = pack.sw_version_minor_GET();
            long  sw_vcs_commit = pack.sw_vcs_commit_GET();
        });
        LoopBackDemoChannel.instance.on_PARAM_EXT_REQUEST_READ.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            String param_id = pack.param_id_TRY(ph);
            short  param_index = pack.param_index_GET();
        });
        LoopBackDemoChannel.instance.on_PARAM_EXT_REQUEST_LIST.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
        });
        LoopBackDemoChannel.instance.on_PARAM_EXT_VALUE.add((src, ph, pack) ->
        {
            String param_id = pack.param_id_TRY(ph);
            String param_value = pack.param_value_TRY(ph);
            @MAV_PARAM_EXT_TYPE int  param_type = pack.param_type_GET();
            char  param_count = pack.param_count_GET();
            char  param_index = pack.param_index_GET();
        });
        LoopBackDemoChannel.instance.on_PARAM_EXT_SET.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            String param_id = pack.param_id_TRY(ph);
            String param_value = pack.param_value_TRY(ph);
            @MAV_PARAM_EXT_TYPE int  param_type = pack.param_type_GET();
        });
        LoopBackDemoChannel.instance.on_PARAM_EXT_ACK.add((src, ph, pack) ->
        {
            String param_id = pack.param_id_TRY(ph);
            String param_value = pack.param_value_TRY(ph);
            @MAV_PARAM_EXT_TYPE int  param_type = pack.param_type_GET();
            @PARAM_ACK int  param_result = pack.param_result_GET();
        });
        LoopBackDemoChannel.instance.on_OBSTACLE_DISTANCE.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            @MAV_DISTANCE_SENSOR int  sensor_type = pack.sensor_type_GET();
            char[]  distances = pack.distances_GET();
            char  increment = pack.increment_GET();
            char  min_distance = pack.min_distance_GET();
            char  max_distance = pack.max_distance_GET();
        });
        HEARTBEAT p0 = LoopBackDemoChannel.instance.new_HEARTBEAT();
        PH.setPack(p0);
        p0.type_SET(MAV_TYPE.MAV_TYPE_VTOL_RESERVED5);
        p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_ARDUPILOTMEGA);
        p0.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED);
        p0.custom_mode_SET(2435138756L);
        p0.system_status_SET(MAV_STATE.MAV_STATE_CALIBRATING);
        p0.mavlink_version_SET((char)217);
        LoopBackDemoChannel.instance.send(p0); //===============================
        SYS_STATUS p1 = LoopBackDemoChannel.instance.new_SYS_STATUS();
        PH.setPack(p1);
        p1.onboard_control_sensors_present_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE);
        p1.onboard_control_sensors_enabled_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR);
        p1.onboard_control_sensors_health_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH);
        p1.load_SET((char)26430);
        p1.voltage_battery_SET((char)55498);
        p1.current_battery_SET((short) -1710);
        p1.battery_remaining_SET((byte)89);
        p1.drop_rate_comm_SET((char)62894);
        p1.errors_comm_SET((char)27170);
        p1.errors_count1_SET((char)51817);
        p1.errors_count2_SET((char)3158);
        p1.errors_count3_SET((char)45086);
        p1.errors_count4_SET((char)23809);
        LoopBackDemoChannel.instance.send(p1); //===============================
        SYSTEM_TIME p2 = LoopBackDemoChannel.instance.new_SYSTEM_TIME();
        PH.setPack(p2);
        p2.time_unix_usec_SET(4576391919017669450L);
        p2.time_boot_ms_SET(1592421247L);
        LoopBackDemoChannel.instance.send(p2); //===============================
        POSITION_TARGET_LOCAL_NED p3 = LoopBackDemoChannel.instance.new_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.time_boot_ms_SET(2610776439L);
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
        p3.type_mask_SET((char)65439);
        p3.x_SET(-2.8620376E38F);
        p3.y_SET(-2.5537513E38F);
        p3.z_SET(1.3922677E38F);
        p3.vx_SET(-2.6450274E38F);
        p3.vy_SET(1.2984408E38F);
        p3.vz_SET(2.0455617E38F);
        p3.afx_SET(2.7715837E37F);
        p3.afy_SET(1.4968202E38F);
        p3.afz_SET(-3.1613291E38F);
        p3.yaw_SET(-1.3405855E38F);
        p3.yaw_rate_SET(8.0224125E37F);
        LoopBackDemoChannel.instance.send(p3); //===============================
        PING p4 = LoopBackDemoChannel.instance.new_PING();
        PH.setPack(p4);
        p4.time_usec_SET(9035583133639376339L);
        p4.seq_SET(1092899706L);
        p4.target_system_SET((char)21);
        p4.target_component_SET((char)130);
        LoopBackDemoChannel.instance.send(p4); //===============================
        CHANGE_OPERATOR_CONTROL p5 = LoopBackDemoChannel.instance.new_CHANGE_OPERATOR_CONTROL();
        PH.setPack(p5);
        p5.target_system_SET((char)70);
        p5.control_request_SET((char)109);
        p5.version_SET((char)93);
        p5.passkey_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p5); //===============================
        CHANGE_OPERATOR_CONTROL_ACK p6 = LoopBackDemoChannel.instance.new_CHANGE_OPERATOR_CONTROL_ACK();
        PH.setPack(p6);
        p6.gcs_system_id_SET((char)226);
        p6.control_request_SET((char)179);
        p6.ack_SET((char)224);
        LoopBackDemoChannel.instance.send(p6); //===============================
        AUTH_KEY p7 = LoopBackDemoChannel.instance.new_AUTH_KEY();
        PH.setPack(p7);
        p7.key_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p7); //===============================
        SET_MODE p11 = LoopBackDemoChannel.instance.new_SET_MODE();
        PH.setPack(p11);
        p11.target_system_SET((char)219);
        p11.base_mode_SET(MAV_MODE.MAV_MODE_GUIDED_DISARMED);
        p11.custom_mode_SET(2328431941L);
        LoopBackDemoChannel.instance.send(p11); //===============================
        PARAM_REQUEST_READ p20 = LoopBackDemoChannel.instance.new_PARAM_REQUEST_READ();
        PH.setPack(p20);
        p20.target_system_SET((char)100);
        p20.target_component_SET((char)103);
        p20.param_id_SET("DEMO", PH);
        p20.param_index_SET((short) -13814);
        LoopBackDemoChannel.instance.send(p20); //===============================
        PARAM_REQUEST_LIST p21 = LoopBackDemoChannel.instance.new_PARAM_REQUEST_LIST();
        PH.setPack(p21);
        p21.target_system_SET((char)67);
        p21.target_component_SET((char)235);
        LoopBackDemoChannel.instance.send(p21); //===============================
        PARAM_VALUE p22 = LoopBackDemoChannel.instance.new_PARAM_VALUE();
        PH.setPack(p22);
        p22.param_id_SET("DEMO", PH);
        p22.param_value_SET(-3.0569232E38F);
        p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16);
        p22.param_count_SET((char)18417);
        p22.param_index_SET((char)24251);
        LoopBackDemoChannel.instance.send(p22); //===============================
        PARAM_SET p23 = LoopBackDemoChannel.instance.new_PARAM_SET();
        PH.setPack(p23);
        p23.target_system_SET((char)162);
        p23.target_component_SET((char)222);
        p23.param_id_SET("DEMO", PH);
        p23.param_value_SET(9.304673E36F);
        p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT32);
        LoopBackDemoChannel.instance.send(p23); //===============================
        GPS_RAW_INT p24 = LoopBackDemoChannel.instance.new_GPS_RAW_INT();
        PH.setPack(p24);
        p24.time_usec_SET(1695182827215903001L);
        p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_PPP);
        p24.lat_SET(1241639588);
        p24.lon_SET(-2130252064);
        p24.alt_SET(934447695);
        p24.eph_SET((char)40328);
        p24.epv_SET((char)28357);
        p24.vel_SET((char)51415);
        p24.cog_SET((char)23162);
        p24.satellites_visible_SET((char)195);
        p24.alt_ellipsoid_SET(839491141, PH);
        p24.h_acc_SET(2211777575L, PH);
        p24.v_acc_SET(653929511L, PH);
        p24.vel_acc_SET(2053966630L, PH);
        p24.hdg_acc_SET(2906753896L, PH);
        LoopBackDemoChannel.instance.send(p24); //===============================
        GPS_STATUS p25 = LoopBackDemoChannel.instance.new_GPS_STATUS();
        PH.setPack(p25);
        p25.satellites_visible_SET((char)194);
        p25.satellite_prn_SET(new char[20], 0);
        p25.satellite_used_SET(new char[20], 0);
        p25.satellite_elevation_SET(new char[20], 0);
        p25.satellite_azimuth_SET(new char[20], 0);
        p25.satellite_snr_SET(new char[20], 0);
        LoopBackDemoChannel.instance.send(p25); //===============================
        SCALED_IMU p26 = LoopBackDemoChannel.instance.new_SCALED_IMU();
        PH.setPack(p26);
        p26.time_boot_ms_SET(675428542L);
        p26.xacc_SET((short)20331);
        p26.yacc_SET((short)1018);
        p26.zacc_SET((short)30163);
        p26.xgyro_SET((short) -15877);
        p26.ygyro_SET((short) -10253);
        p26.zgyro_SET((short)20526);
        p26.xmag_SET((short)11505);
        p26.ymag_SET((short)21539);
        p26.zmag_SET((short)6247);
        LoopBackDemoChannel.instance.send(p26); //===============================
        RAW_IMU p27 = LoopBackDemoChannel.instance.new_RAW_IMU();
        PH.setPack(p27);
        p27.time_usec_SET(5167459011390939045L);
        p27.xacc_SET((short)28720);
        p27.yacc_SET((short) -2060);
        p27.zacc_SET((short)16817);
        p27.xgyro_SET((short) -18709);
        p27.ygyro_SET((short)20730);
        p27.zgyro_SET((short) -30080);
        p27.xmag_SET((short) -16016);
        p27.ymag_SET((short) -29337);
        p27.zmag_SET((short) -31770);
        LoopBackDemoChannel.instance.send(p27); //===============================
        RAW_PRESSURE p28 = LoopBackDemoChannel.instance.new_RAW_PRESSURE();
        PH.setPack(p28);
        p28.time_usec_SET(3555954722031107681L);
        p28.press_abs_SET((short)17203);
        p28.press_diff1_SET((short)2816);
        p28.press_diff2_SET((short)27035);
        p28.temperature_SET((short) -711);
        LoopBackDemoChannel.instance.send(p28); //===============================
        SCALED_PRESSURE p29 = LoopBackDemoChannel.instance.new_SCALED_PRESSURE();
        PH.setPack(p29);
        p29.time_boot_ms_SET(1953592283L);
        p29.press_abs_SET(4.6052494E37F);
        p29.press_diff_SET(-4.9678857E37F);
        p29.temperature_SET((short)14704);
        LoopBackDemoChannel.instance.send(p29); //===============================
        ATTITUDE p30 = LoopBackDemoChannel.instance.new_ATTITUDE();
        PH.setPack(p30);
        p30.time_boot_ms_SET(2399546802L);
        p30.roll_SET(-2.8198048E38F);
        p30.pitch_SET(1.9384148E38F);
        p30.yaw_SET(-5.8472146E36F);
        p30.rollspeed_SET(-3.013519E38F);
        p30.pitchspeed_SET(3.252095E38F);
        p30.yawspeed_SET(-3.2263126E38F);
        LoopBackDemoChannel.instance.send(p30); //===============================
        ATTITUDE_QUATERNION p31 = LoopBackDemoChannel.instance.new_ATTITUDE_QUATERNION();
        PH.setPack(p31);
        p31.time_boot_ms_SET(3991538399L);
        p31.q1_SET(-1.0810008E38F);
        p31.q2_SET(-4.932627E37F);
        p31.q3_SET(3.1211945E38F);
        p31.q4_SET(-2.3009432E38F);
        p31.rollspeed_SET(-2.6777028E38F);
        p31.pitchspeed_SET(1.1942581E38F);
        p31.yawspeed_SET(1.6170703E38F);
        LoopBackDemoChannel.instance.send(p31); //===============================
        LOCAL_POSITION_NED p32 = LoopBackDemoChannel.instance.new_LOCAL_POSITION_NED();
        PH.setPack(p32);
        p32.time_boot_ms_SET(1087610724L);
        p32.x_SET(-1.6555497E38F);
        p32.y_SET(-3.357497E38F);
        p32.z_SET(-5.4547365E37F);
        p32.vx_SET(2.970716E38F);
        p32.vy_SET(1.0790065E38F);
        p32.vz_SET(-1.7268679E38F);
        LoopBackDemoChannel.instance.send(p32); //===============================
        GLOBAL_POSITION_INT p33 = LoopBackDemoChannel.instance.new_GLOBAL_POSITION_INT();
        PH.setPack(p33);
        p33.time_boot_ms_SET(3685904086L);
        p33.lat_SET(765065534);
        p33.lon_SET(2030234898);
        p33.alt_SET(1439102598);
        p33.relative_alt_SET(-1937184264);
        p33.vx_SET((short) -3206);
        p33.vy_SET((short)12695);
        p33.vz_SET((short) -3994);
        p33.hdg_SET((char)64067);
        LoopBackDemoChannel.instance.send(p33); //===============================
        RC_CHANNELS_SCALED p34 = LoopBackDemoChannel.instance.new_RC_CHANNELS_SCALED();
        PH.setPack(p34);
        p34.time_boot_ms_SET(3160619887L);
        p34.port_SET((char)176);
        p34.chan1_scaled_SET((short)7022);
        p34.chan2_scaled_SET((short) -23074);
        p34.chan3_scaled_SET((short) -10636);
        p34.chan4_scaled_SET((short)11372);
        p34.chan5_scaled_SET((short) -25570);
        p34.chan6_scaled_SET((short) -27896);
        p34.chan7_scaled_SET((short) -12467);
        p34.chan8_scaled_SET((short) -30895);
        p34.rssi_SET((char)84);
        LoopBackDemoChannel.instance.send(p34); //===============================
        RC_CHANNELS_RAW p35 = LoopBackDemoChannel.instance.new_RC_CHANNELS_RAW();
        PH.setPack(p35);
        p35.time_boot_ms_SET(2785842189L);
        p35.port_SET((char)239);
        p35.chan1_raw_SET((char)23184);
        p35.chan2_raw_SET((char)55360);
        p35.chan3_raw_SET((char)30656);
        p35.chan4_raw_SET((char)13293);
        p35.chan5_raw_SET((char)36333);
        p35.chan6_raw_SET((char)31451);
        p35.chan7_raw_SET((char)27200);
        p35.chan8_raw_SET((char)1283);
        p35.rssi_SET((char)212);
        LoopBackDemoChannel.instance.send(p35); //===============================
        SERVO_OUTPUT_RAW p36 = LoopBackDemoChannel.instance.new_SERVO_OUTPUT_RAW();
        PH.setPack(p36);
        p36.time_usec_SET(1659712933L);
        p36.port_SET((char)100);
        p36.servo1_raw_SET((char)14272);
        p36.servo2_raw_SET((char)6639);
        p36.servo3_raw_SET((char)51013);
        p36.servo4_raw_SET((char)61780);
        p36.servo5_raw_SET((char)26025);
        p36.servo6_raw_SET((char)20002);
        p36.servo7_raw_SET((char)48094);
        p36.servo8_raw_SET((char)13567);
        p36.servo9_raw_SET((char)31897, PH);
        p36.servo10_raw_SET((char)35784, PH);
        p36.servo11_raw_SET((char)61960, PH);
        p36.servo12_raw_SET((char)2171, PH);
        p36.servo13_raw_SET((char)56328, PH);
        p36.servo14_raw_SET((char)7341, PH);
        p36.servo15_raw_SET((char)24125, PH);
        p36.servo16_raw_SET((char)24023, PH);
        LoopBackDemoChannel.instance.send(p36); //===============================
        MISSION_REQUEST_PARTIAL_LIST p37 = LoopBackDemoChannel.instance.new_MISSION_REQUEST_PARTIAL_LIST();
        PH.setPack(p37);
        p37.target_system_SET((char)179);
        p37.target_component_SET((char)235);
        p37.start_index_SET((short)12275);
        p37.end_index_SET((short) -32311);
        p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
        LoopBackDemoChannel.instance.send(p37); //===============================
        MISSION_WRITE_PARTIAL_LIST p38 = LoopBackDemoChannel.instance.new_MISSION_WRITE_PARTIAL_LIST();
        PH.setPack(p38);
        p38.target_system_SET((char)47);
        p38.target_component_SET((char)211);
        p38.start_index_SET((short)21213);
        p38.end_index_SET((short) -17550);
        p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
        LoopBackDemoChannel.instance.send(p38); //===============================
        MISSION_ITEM p39 = LoopBackDemoChannel.instance.new_MISSION_ITEM();
        PH.setPack(p39);
        p39.target_system_SET((char)81);
        p39.target_component_SET((char)32);
        p39.seq_SET((char)1159);
        p39.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
        p39.command_SET(MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY);
        p39.current_SET((char)234);
        p39.autocontinue_SET((char)175);
        p39.param1_SET(-7.183372E37F);
        p39.param2_SET(-1.1782443E38F);
        p39.param3_SET(-3.2612928E38F);
        p39.param4_SET(-3.0101565E36F);
        p39.x_SET(-8.659937E37F);
        p39.y_SET(-2.6085211E38F);
        p39.z_SET(3.3329781E38F);
        p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
        LoopBackDemoChannel.instance.send(p39); //===============================
        MISSION_REQUEST p40 = LoopBackDemoChannel.instance.new_MISSION_REQUEST();
        PH.setPack(p40);
        p40.target_system_SET((char)244);
        p40.target_component_SET((char)178);
        p40.seq_SET((char)14576);
        p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
        LoopBackDemoChannel.instance.send(p40); //===============================
        MISSION_SET_CURRENT p41 = LoopBackDemoChannel.instance.new_MISSION_SET_CURRENT();
        PH.setPack(p41);
        p41.target_system_SET((char)157);
        p41.target_component_SET((char)167);
        p41.seq_SET((char)65072);
        LoopBackDemoChannel.instance.send(p41); //===============================
        MISSION_CURRENT p42 = LoopBackDemoChannel.instance.new_MISSION_CURRENT();
        PH.setPack(p42);
        p42.seq_SET((char)24488);
        LoopBackDemoChannel.instance.send(p42); //===============================
        MISSION_REQUEST_LIST p43 = LoopBackDemoChannel.instance.new_MISSION_REQUEST_LIST();
        PH.setPack(p43);
        p43.target_system_SET((char)157);
        p43.target_component_SET((char)86);
        p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
        LoopBackDemoChannel.instance.send(p43); //===============================
        MISSION_COUNT p44 = LoopBackDemoChannel.instance.new_MISSION_COUNT();
        PH.setPack(p44);
        p44.target_system_SET((char)233);
        p44.target_component_SET((char)233);
        p44.count_SET((char)22354);
        p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
        LoopBackDemoChannel.instance.send(p44); //===============================
        MISSION_CLEAR_ALL p45 = LoopBackDemoChannel.instance.new_MISSION_CLEAR_ALL();
        PH.setPack(p45);
        p45.target_system_SET((char)104);
        p45.target_component_SET((char)236);
        p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
        LoopBackDemoChannel.instance.send(p45); //===============================
        MISSION_ITEM_REACHED p46 = LoopBackDemoChannel.instance.new_MISSION_ITEM_REACHED();
        PH.setPack(p46);
        p46.seq_SET((char)10257);
        LoopBackDemoChannel.instance.send(p46); //===============================
        MISSION_ACK p47 = LoopBackDemoChannel.instance.new_MISSION_ACK();
        PH.setPack(p47);
        p47.target_system_SET((char)252);
        p47.target_component_SET((char)239);
        p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_NO_SPACE);
        p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
        LoopBackDemoChannel.instance.send(p47); //===============================
        SET_GPS_GLOBAL_ORIGIN p48 = LoopBackDemoChannel.instance.new_SET_GPS_GLOBAL_ORIGIN();
        PH.setPack(p48);
        p48.target_system_SET((char)119);
        p48.latitude_SET(1467372682);
        p48.longitude_SET(-4190983);
        p48.altitude_SET(878784183);
        p48.time_usec_SET(8612861944409361242L, PH);
        LoopBackDemoChannel.instance.send(p48); //===============================
        GPS_GLOBAL_ORIGIN p49 = LoopBackDemoChannel.instance.new_GPS_GLOBAL_ORIGIN();
        PH.setPack(p49);
        p49.latitude_SET(1612697400);
        p49.longitude_SET(-1316034217);
        p49.altitude_SET(1261876458);
        p49.time_usec_SET(2770783033376186221L, PH);
        LoopBackDemoChannel.instance.send(p49); //===============================
        PARAM_MAP_RC p50 = LoopBackDemoChannel.instance.new_PARAM_MAP_RC();
        PH.setPack(p50);
        p50.target_system_SET((char)113);
        p50.target_component_SET((char)136);
        p50.param_id_SET("DEMO", PH);
        p50.param_index_SET((short)13647);
        p50.parameter_rc_channel_index_SET((char)203);
        p50.param_value0_SET(2.82549E38F);
        p50.scale_SET(1.3183348E38F);
        p50.param_value_min_SET(-2.9767912E38F);
        p50.param_value_max_SET(-1.9138685E38F);
        LoopBackDemoChannel.instance.send(p50); //===============================
        MISSION_REQUEST_INT p51 = LoopBackDemoChannel.instance.new_MISSION_REQUEST_INT();
        PH.setPack(p51);
        p51.target_system_SET((char)44);
        p51.target_component_SET((char)32);
        p51.seq_SET((char)31360);
        p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
        LoopBackDemoChannel.instance.send(p51); //===============================
        SAFETY_SET_ALLOWED_AREA p54 = LoopBackDemoChannel.instance.new_SAFETY_SET_ALLOWED_AREA();
        PH.setPack(p54);
        p54.target_system_SET((char)187);
        p54.target_component_SET((char)61);
        p54.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
        p54.p1x_SET(2.727215E38F);
        p54.p1y_SET(1.4887357E38F);
        p54.p1z_SET(-7.184989E37F);
        p54.p2x_SET(-2.4259174E38F);
        p54.p2y_SET(-1.4257767E38F);
        p54.p2z_SET(-2.8735739E38F);
        LoopBackDemoChannel.instance.send(p54); //===============================
        SAFETY_ALLOWED_AREA p55 = LoopBackDemoChannel.instance.new_SAFETY_ALLOWED_AREA();
        PH.setPack(p55);
        p55.frame_SET(MAV_FRAME.MAV_FRAME_MISSION);
        p55.p1x_SET(8.4033136E37F);
        p55.p1y_SET(-2.7602475E38F);
        p55.p1z_SET(2.1294603E38F);
        p55.p2x_SET(-3.404432E36F);
        p55.p2y_SET(-1.3236385E38F);
        p55.p2z_SET(-6.953576E37F);
        LoopBackDemoChannel.instance.send(p55); //===============================
        ATTITUDE_QUATERNION_COV p61 = LoopBackDemoChannel.instance.new_ATTITUDE_QUATERNION_COV();
        PH.setPack(p61);
        p61.time_usec_SET(6300352214560263807L);
        p61.q_SET(new float[4], 0);
        p61.rollspeed_SET(2.19179E38F);
        p61.pitchspeed_SET(3.1218038E37F);
        p61.yawspeed_SET(-3.0949742E38F);
        p61.covariance_SET(new float[9], 0);
        LoopBackDemoChannel.instance.send(p61); //===============================
        NAV_CONTROLLER_OUTPUT p62 = LoopBackDemoChannel.instance.new_NAV_CONTROLLER_OUTPUT();
        PH.setPack(p62);
        p62.nav_roll_SET(2.7998579E37F);
        p62.nav_pitch_SET(-1.5357087E38F);
        p62.nav_bearing_SET((short)17888);
        p62.target_bearing_SET((short)25541);
        p62.wp_dist_SET((char)16485);
        p62.alt_error_SET(-1.0561781E38F);
        p62.aspd_error_SET(3.0007888E38F);
        p62.xtrack_error_SET(1.9993274E38F);
        LoopBackDemoChannel.instance.send(p62); //===============================
        GLOBAL_POSITION_INT_COV p63 = LoopBackDemoChannel.instance.new_GLOBAL_POSITION_INT_COV();
        PH.setPack(p63);
        p63.time_usec_SET(5662921457029554702L);
        p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION);
        p63.lat_SET(-937941362);
        p63.lon_SET(1181963368);
        p63.alt_SET(-1605217036);
        p63.relative_alt_SET(-898797853);
        p63.vx_SET(7.0401106E37F);
        p63.vy_SET(-2.6957188E38F);
        p63.vz_SET(8.328136E35F);
        p63.covariance_SET(new float[36], 0);
        LoopBackDemoChannel.instance.send(p63); //===============================
        LOCAL_POSITION_NED_COV p64 = LoopBackDemoChannel.instance.new_LOCAL_POSITION_NED_COV();
        PH.setPack(p64);
        p64.time_usec_SET(6422274707065332038L);
        p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO);
        p64.x_SET(9.598331E37F);
        p64.y_SET(2.6292311E36F);
        p64.z_SET(-1.8246461E38F);
        p64.vx_SET(-2.6669592E38F);
        p64.vy_SET(2.908215E38F);
        p64.vz_SET(3.0450296E38F);
        p64.ax_SET(-8.3524403E37F);
        p64.ay_SET(1.1070982E38F);
        p64.az_SET(2.5269402E38F);
        p64.covariance_SET(new float[45], 0);
        LoopBackDemoChannel.instance.send(p64); //===============================
        RC_CHANNELS p65 = LoopBackDemoChannel.instance.new_RC_CHANNELS();
        PH.setPack(p65);
        p65.time_boot_ms_SET(1063250958L);
        p65.chancount_SET((char)11);
        p65.chan1_raw_SET((char)61527);
        p65.chan2_raw_SET((char)56786);
        p65.chan3_raw_SET((char)14291);
        p65.chan4_raw_SET((char)20461);
        p65.chan5_raw_SET((char)15942);
        p65.chan6_raw_SET((char)57635);
        p65.chan7_raw_SET((char)48011);
        p65.chan8_raw_SET((char)20800);
        p65.chan9_raw_SET((char)33676);
        p65.chan10_raw_SET((char)6710);
        p65.chan11_raw_SET((char)21777);
        p65.chan12_raw_SET((char)27744);
        p65.chan13_raw_SET((char)54252);
        p65.chan14_raw_SET((char)24959);
        p65.chan15_raw_SET((char)7993);
        p65.chan16_raw_SET((char)41105);
        p65.chan17_raw_SET((char)14199);
        p65.chan18_raw_SET((char)28870);
        p65.rssi_SET((char)22);
        LoopBackDemoChannel.instance.send(p65); //===============================
        REQUEST_DATA_STREAM p66 = LoopBackDemoChannel.instance.new_REQUEST_DATA_STREAM();
        PH.setPack(p66);
        p66.target_system_SET((char)36);
        p66.target_component_SET((char)241);
        p66.req_stream_id_SET((char)88);
        p66.req_message_rate_SET((char)38641);
        p66.start_stop_SET((char)213);
        LoopBackDemoChannel.instance.send(p66); //===============================
        DATA_STREAM p67 = LoopBackDemoChannel.instance.new_DATA_STREAM();
        PH.setPack(p67);
        p67.stream_id_SET((char)151);
        p67.message_rate_SET((char)10574);
        p67.on_off_SET((char)153);
        LoopBackDemoChannel.instance.send(p67); //===============================
        MANUAL_CONTROL p69 = LoopBackDemoChannel.instance.new_MANUAL_CONTROL();
        PH.setPack(p69);
        p69.target_SET((char)236);
        p69.x_SET((short)10632);
        p69.y_SET((short) -3624);
        p69.z_SET((short)21215);
        p69.r_SET((short) -12334);
        p69.buttons_SET((char)60708);
        LoopBackDemoChannel.instance.send(p69); //===============================
        RC_CHANNELS_OVERRIDE p70 = LoopBackDemoChannel.instance.new_RC_CHANNELS_OVERRIDE();
        PH.setPack(p70);
        p70.target_system_SET((char)156);
        p70.target_component_SET((char)25);
        p70.chan1_raw_SET((char)51707);
        p70.chan2_raw_SET((char)55994);
        p70.chan3_raw_SET((char)1949);
        p70.chan4_raw_SET((char)58768);
        p70.chan5_raw_SET((char)55872);
        p70.chan6_raw_SET((char)39988);
        p70.chan7_raw_SET((char)22884);
        p70.chan8_raw_SET((char)54686);
        LoopBackDemoChannel.instance.send(p70); //===============================
        MISSION_ITEM_INT p73 = LoopBackDemoChannel.instance.new_MISSION_ITEM_INT();
        PH.setPack(p73);
        p73.target_system_SET((char)165);
        p73.target_component_SET((char)94);
        p73.seq_SET((char)46482);
        p73.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL);
        p73.command_SET(MAV_CMD.MAV_CMD_PREFLIGHT_UAVCAN);
        p73.current_SET((char)64);
        p73.autocontinue_SET((char)161);
        p73.param1_SET(-7.08377E37F);
        p73.param2_SET(2.1499036E38F);
        p73.param3_SET(2.998761E37F);
        p73.param4_SET(1.8350331E37F);
        p73.x_SET(-1227172699);
        p73.y_SET(1524030909);
        p73.z_SET(-7.282844E37F);
        p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
        LoopBackDemoChannel.instance.send(p73); //===============================
        VFR_HUD p74 = LoopBackDemoChannel.instance.new_VFR_HUD();
        PH.setPack(p74);
        p74.airspeed_SET(2.2047446E38F);
        p74.groundspeed_SET(-1.8398527E38F);
        p74.heading_SET((short)16056);
        p74.throttle_SET((char)21865);
        p74.alt_SET(3.0703124E38F);
        p74.climb_SET(2.3485707E37F);
        LoopBackDemoChannel.instance.send(p74); //===============================
        COMMAND_INT p75 = LoopBackDemoChannel.instance.new_COMMAND_INT();
        PH.setPack(p75);
        p75.target_system_SET((char)188);
        p75.target_component_SET((char)204);
        p75.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_ENU);
        p75.command_SET(MAV_CMD.MAV_CMD_CONDITION_LAST);
        p75.current_SET((char)251);
        p75.autocontinue_SET((char)64);
        p75.param1_SET(-9.541264E37F);
        p75.param2_SET(-1.0501377E38F);
        p75.param3_SET(2.3198284E38F);
        p75.param4_SET(3.1081629E38F);
        p75.x_SET(-1614211716);
        p75.y_SET(1552175219);
        p75.z_SET(-9.899917E37F);
        LoopBackDemoChannel.instance.send(p75); //===============================
        COMMAND_LONG p76 = LoopBackDemoChannel.instance.new_COMMAND_LONG();
        PH.setPack(p76);
        p76.target_system_SET((char)174);
        p76.target_component_SET((char)132);
        p76.command_SET(MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE);
        p76.confirmation_SET((char)53);
        p76.param1_SET(-4.8974175E37F);
        p76.param2_SET(3.1343588E38F);
        p76.param3_SET(-2.9672228E38F);
        p76.param4_SET(1.5682164E37F);
        p76.param5_SET(-3.2037857E38F);
        p76.param6_SET(2.6602605E38F);
        p76.param7_SET(-2.0718846E38F);
        LoopBackDemoChannel.instance.send(p76); //===============================
        COMMAND_ACK p77 = LoopBackDemoChannel.instance.new_COMMAND_ACK();
        PH.setPack(p77);
        p77.command_SET(MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION);
        p77.result_SET(MAV_RESULT.MAV_RESULT_ACCEPTED);
        p77.progress_SET((char)149, PH);
        p77.result_param2_SET(1875099036, PH);
        p77.target_system_SET((char)96, PH);
        p77.target_component_SET((char)97, PH);
        LoopBackDemoChannel.instance.send(p77); //===============================
        MANUAL_SETPOINT p81 = LoopBackDemoChannel.instance.new_MANUAL_SETPOINT();
        PH.setPack(p81);
        p81.time_boot_ms_SET(2873844825L);
        p81.roll_SET(2.3161125E37F);
        p81.pitch_SET(-3.1073055E38F);
        p81.yaw_SET(1.4092305E38F);
        p81.thrust_SET(-2.4233608E38F);
        p81.mode_switch_SET((char)95);
        p81.manual_override_switch_SET((char)233);
        LoopBackDemoChannel.instance.send(p81); //===============================
        SET_ATTITUDE_TARGET p82 = LoopBackDemoChannel.instance.new_SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.time_boot_ms_SET(3486991789L);
        p82.target_system_SET((char)39);
        p82.target_component_SET((char)89);
        p82.type_mask_SET((char)180);
        p82.q_SET(new float[4], 0);
        p82.body_roll_rate_SET(-1.4195945E37F);
        p82.body_pitch_rate_SET(1.9229319E38F);
        p82.body_yaw_rate_SET(-1.1073386E38F);
        p82.thrust_SET(-8.2665975E37F);
        LoopBackDemoChannel.instance.send(p82); //===============================
        ATTITUDE_TARGET p83 = LoopBackDemoChannel.instance.new_ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.time_boot_ms_SET(2073626903L);
        p83.type_mask_SET((char)5);
        p83.q_SET(new float[4], 0);
        p83.body_roll_rate_SET(-1.5121649E38F);
        p83.body_pitch_rate_SET(-9.337227E37F);
        p83.body_yaw_rate_SET(-9.218034E37F);
        p83.thrust_SET(4.561281E37F);
        LoopBackDemoChannel.instance.send(p83); //===============================
        SET_POSITION_TARGET_LOCAL_NED p84 = LoopBackDemoChannel.instance.new_SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.time_boot_ms_SET(938324552L);
        p84.target_system_SET((char)254);
        p84.target_component_SET((char)75);
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
        p84.type_mask_SET((char)8603);
        p84.x_SET(3.1474223E38F);
        p84.y_SET(1.1318541E38F);
        p84.z_SET(1.4168782E38F);
        p84.vx_SET(-1.1791892E38F);
        p84.vy_SET(-3.2090705E38F);
        p84.vz_SET(1.010974E38F);
        p84.afx_SET(3.090997E37F);
        p84.afy_SET(-1.0802049E38F);
        p84.afz_SET(-2.311183E38F);
        p84.yaw_SET(-3.0229875E38F);
        p84.yaw_rate_SET(3.5969E36F);
        LoopBackDemoChannel.instance.send(p84); //===============================
        SET_POSITION_TARGET_GLOBAL_INT p86 = LoopBackDemoChannel.instance.new_SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.time_boot_ms_SET(2543510616L);
        p86.target_system_SET((char)41);
        p86.target_component_SET((char)187);
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
        p86.type_mask_SET((char)582);
        p86.lat_int_SET(1242859804);
        p86.lon_int_SET(1684399100);
        p86.alt_SET(-1.4854667E38F);
        p86.vx_SET(-1.6766915E37F);
        p86.vy_SET(2.5482665E37F);
        p86.vz_SET(-3.312535E37F);
        p86.afx_SET(2.479607E38F);
        p86.afy_SET(1.8557591E38F);
        p86.afz_SET(2.2725905E38F);
        p86.yaw_SET(-2.982675E38F);
        p86.yaw_rate_SET(-2.1005656E38F);
        LoopBackDemoChannel.instance.send(p86); //===============================
        POSITION_TARGET_GLOBAL_INT p87 = LoopBackDemoChannel.instance.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.time_boot_ms_SET(3469045232L);
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_ENU);
        p87.type_mask_SET((char)9643);
        p87.lat_int_SET(1201308417);
        p87.lon_int_SET(-102156626);
        p87.alt_SET(-7.8393323E37F);
        p87.vx_SET(-1.369373E38F);
        p87.vy_SET(-2.7415814E38F);
        p87.vz_SET(-2.4161747E38F);
        p87.afx_SET(-1.6964035E38F);
        p87.afy_SET(-4.1774792E37F);
        p87.afz_SET(2.26591E38F);
        p87.yaw_SET(2.5376801E38F);
        p87.yaw_rate_SET(2.195101E37F);
        LoopBackDemoChannel.instance.send(p87); //===============================
        LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = LoopBackDemoChannel.instance.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.time_boot_ms_SET(618105477L);
        p89.x_SET(9.172006E37F);
        p89.y_SET(1.9248719E38F);
        p89.z_SET(-1.5026314E38F);
        p89.roll_SET(1.7886637E38F);
        p89.pitch_SET(2.4283992E37F);
        p89.yaw_SET(2.3696093E38F);
        LoopBackDemoChannel.instance.send(p89); //===============================
        HIL_STATE p90 = LoopBackDemoChannel.instance.new_HIL_STATE();
        PH.setPack(p90);
        p90.time_usec_SET(5860749933996520105L);
        p90.roll_SET(3.6747794E37F);
        p90.pitch_SET(2.2150365E38F);
        p90.yaw_SET(2.235917E38F);
        p90.rollspeed_SET(1.2897974E38F);
        p90.pitchspeed_SET(1.8937688E38F);
        p90.yawspeed_SET(-2.1264697E38F);
        p90.lat_SET(-66367800);
        p90.lon_SET(-341627400);
        p90.alt_SET(-2111853773);
        p90.vx_SET((short) -19573);
        p90.vy_SET((short) -20262);
        p90.vz_SET((short)8641);
        p90.xacc_SET((short) -27813);
        p90.yacc_SET((short)2167);
        p90.zacc_SET((short)15935);
        LoopBackDemoChannel.instance.send(p90); //===============================
        HIL_CONTROLS p91 = LoopBackDemoChannel.instance.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.time_usec_SET(491471476614868854L);
        p91.roll_ailerons_SET(-2.63794E38F);
        p91.pitch_elevator_SET(-3.1365029E38F);
        p91.yaw_rudder_SET(1.9751808E37F);
        p91.throttle_SET(1.3689047E38F);
        p91.aux1_SET(2.5935835E38F);
        p91.aux2_SET(1.369582E38F);
        p91.aux3_SET(-7.3406016E37F);
        p91.aux4_SET(2.1508903E38F);
        p91.mode_SET(MAV_MODE.MAV_MODE_AUTO_DISARMED);
        p91.nav_mode_SET((char)215);
        LoopBackDemoChannel.instance.send(p91); //===============================
        HIL_RC_INPUTS_RAW p92 = LoopBackDemoChannel.instance.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.time_usec_SET(3498879345511262964L);
        p92.chan1_raw_SET((char)4913);
        p92.chan2_raw_SET((char)25741);
        p92.chan3_raw_SET((char)2105);
        p92.chan4_raw_SET((char)42046);
        p92.chan5_raw_SET((char)60551);
        p92.chan6_raw_SET((char)16090);
        p92.chan7_raw_SET((char)59826);
        p92.chan8_raw_SET((char)43258);
        p92.chan9_raw_SET((char)38568);
        p92.chan10_raw_SET((char)12136);
        p92.chan11_raw_SET((char)50543);
        p92.chan12_raw_SET((char)18391);
        p92.rssi_SET((char)102);
        LoopBackDemoChannel.instance.send(p92); //===============================
        HIL_ACTUATOR_CONTROLS p93 = LoopBackDemoChannel.instance.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.time_usec_SET(2778675391949933672L);
        p93.controls_SET(new float[16], 0);
        p93.mode_SET(MAV_MODE.MAV_MODE_MANUAL_DISARMED);
        p93.flags_SET(2207718247008242434L);
        LoopBackDemoChannel.instance.send(p93); //===============================
        OPTICAL_FLOW p100 = LoopBackDemoChannel.instance.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.time_usec_SET(1152441838468591263L);
        p100.sensor_id_SET((char)87);
        p100.flow_x_SET((short)23651);
        p100.flow_y_SET((short)12826);
        p100.flow_comp_m_x_SET(1.5081677E37F);
        p100.flow_comp_m_y_SET(-9.987262E37F);
        p100.quality_SET((char)152);
        p100.ground_distance_SET(-9.039513E35F);
        p100.flow_rate_x_SET(2.7897822E38F, PH);
        p100.flow_rate_y_SET(5.2515114E37F, PH);
        LoopBackDemoChannel.instance.send(p100); //===============================
        GLOBAL_VISION_POSITION_ESTIMATE p101 = LoopBackDemoChannel.instance.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.usec_SET(8948415299590319281L);
        p101.x_SET(-2.1833128E38F);
        p101.y_SET(2.9640766E38F);
        p101.z_SET(-8.1400226E37F);
        p101.roll_SET(2.570847E38F);
        p101.pitch_SET(1.6402963E37F);
        p101.yaw_SET(-2.0521014E38F);
        LoopBackDemoChannel.instance.send(p101); //===============================
        VISION_POSITION_ESTIMATE p102 = LoopBackDemoChannel.instance.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.usec_SET(574577571632053433L);
        p102.x_SET(-1.9949778E37F);
        p102.y_SET(-2.813701E38F);
        p102.z_SET(1.2877094E37F);
        p102.roll_SET(-1.2695686E38F);
        p102.pitch_SET(2.1366023E37F);
        p102.yaw_SET(-3.018839E38F);
        LoopBackDemoChannel.instance.send(p102); //===============================
        VISION_SPEED_ESTIMATE p103 = LoopBackDemoChannel.instance.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.usec_SET(7546820854807816733L);
        p103.x_SET(-5.9351604E37F);
        p103.y_SET(-1.664908E38F);
        p103.z_SET(-3.2601684E38F);
        LoopBackDemoChannel.instance.send(p103); //===============================
        VICON_POSITION_ESTIMATE p104 = LoopBackDemoChannel.instance.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.usec_SET(3096613083114169949L);
        p104.x_SET(2.4996837E38F);
        p104.y_SET(2.3003408E38F);
        p104.z_SET(2.6859011E38F);
        p104.roll_SET(8.724375E37F);
        p104.pitch_SET(-1.4920144E38F);
        p104.yaw_SET(-2.4389364E38F);
        LoopBackDemoChannel.instance.send(p104); //===============================
        HIGHRES_IMU p105 = LoopBackDemoChannel.instance.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.time_usec_SET(7727397101766801461L);
        p105.xacc_SET(-3.317807E38F);
        p105.yacc_SET(-2.1744723E37F);
        p105.zacc_SET(3.0278601E38F);
        p105.xgyro_SET(-3.2810627E38F);
        p105.ygyro_SET(-1.5874279E38F);
        p105.zgyro_SET(-2.8718586E38F);
        p105.xmag_SET(5.653954E37F);
        p105.ymag_SET(2.3277306E38F);
        p105.zmag_SET(7.0252243E37F);
        p105.abs_pressure_SET(3.0091843E38F);
        p105.diff_pressure_SET(1.0219946E38F);
        p105.pressure_alt_SET(-2.591404E38F);
        p105.temperature_SET(-1.0351589E38F);
        p105.fields_updated_SET((char)58881);
        LoopBackDemoChannel.instance.send(p105); //===============================
        OPTICAL_FLOW_RAD p106 = LoopBackDemoChannel.instance.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.time_usec_SET(7568487115776654753L);
        p106.sensor_id_SET((char)107);
        p106.integration_time_us_SET(1855113984L);
        p106.integrated_x_SET(-3.3651927E38F);
        p106.integrated_y_SET(-1.7144607E38F);
        p106.integrated_xgyro_SET(-7.932606E37F);
        p106.integrated_ygyro_SET(1.351257E38F);
        p106.integrated_zgyro_SET(-1.9648842E38F);
        p106.temperature_SET((short) -20438);
        p106.quality_SET((char)77);
        p106.time_delta_distance_us_SET(3680156425L);
        p106.distance_SET(-2.0985722E38F);
        LoopBackDemoChannel.instance.send(p106); //===============================
        HIL_SENSOR p107 = LoopBackDemoChannel.instance.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.time_usec_SET(6331252656778580177L);
        p107.xacc_SET(1.5486435E38F);
        p107.yacc_SET(8.3689415E37F);
        p107.zacc_SET(-2.535768E38F);
        p107.xgyro_SET(-3.591653E37F);
        p107.ygyro_SET(2.7654457E38F);
        p107.zgyro_SET(-1.8125783E38F);
        p107.xmag_SET(-2.3962156E38F);
        p107.ymag_SET(6.913145E37F);
        p107.zmag_SET(-1.6085394E38F);
        p107.abs_pressure_SET(3.5061707E37F);
        p107.diff_pressure_SET(1.3572041E38F);
        p107.pressure_alt_SET(-2.6945088E38F);
        p107.temperature_SET(2.0675726E38F);
        p107.fields_updated_SET(1349880913L);
        LoopBackDemoChannel.instance.send(p107); //===============================
        SIM_STATE p108 = LoopBackDemoChannel.instance.new_SIM_STATE();
        PH.setPack(p108);
        p108.q1_SET(1.3833233E38F);
        p108.q2_SET(4.0369733E37F);
        p108.q3_SET(-3.3970397E38F);
        p108.q4_SET(-3.2834234E38F);
        p108.roll_SET(2.2954159E38F);
        p108.pitch_SET(3.1765586E38F);
        p108.yaw_SET(-1.4057846E38F);
        p108.xacc_SET(3.6895916E37F);
        p108.yacc_SET(3.0783944E38F);
        p108.zacc_SET(-2.7784082E38F);
        p108.xgyro_SET(-2.7637245E37F);
        p108.ygyro_SET(2.0647146E38F);
        p108.zgyro_SET(-2.4175687E38F);
        p108.lat_SET(8.577218E37F);
        p108.lon_SET(8.4436067E37F);
        p108.alt_SET(3.7348747E37F);
        p108.std_dev_horz_SET(-5.084995E37F);
        p108.std_dev_vert_SET(-1.2787957E38F);
        p108.vn_SET(-1.7016065E38F);
        p108.ve_SET(-1.9830203E38F);
        p108.vd_SET(2.7150539E38F);
        LoopBackDemoChannel.instance.send(p108); //===============================
        RADIO_STATUS p109 = LoopBackDemoChannel.instance.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.rssi_SET((char)249);
        p109.remrssi_SET((char)80);
        p109.txbuf_SET((char)211);
        p109.noise_SET((char)133);
        p109.remnoise_SET((char)59);
        p109.rxerrors_SET((char)10776);
        p109.fixed__SET((char)37417);
        LoopBackDemoChannel.instance.send(p109); //===============================
        FILE_TRANSFER_PROTOCOL p110 = LoopBackDemoChannel.instance.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.target_network_SET((char)223);
        p110.target_system_SET((char)61);
        p110.target_component_SET((char)202);
        p110.payload_SET(new char[251], 0);
        LoopBackDemoChannel.instance.send(p110); //===============================
        TIMESYNC p111 = LoopBackDemoChannel.instance.new_TIMESYNC();
        PH.setPack(p111);
        p111.tc1_SET(-7737586132568980078L);
        p111.ts1_SET(-294728153319661554L);
        LoopBackDemoChannel.instance.send(p111); //===============================
        CAMERA_TRIGGER p112 = LoopBackDemoChannel.instance.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.time_usec_SET(4610201130541848733L);
        p112.seq_SET(4291883135L);
        LoopBackDemoChannel.instance.send(p112); //===============================
        HIL_GPS p113 = LoopBackDemoChannel.instance.new_HIL_GPS();
        PH.setPack(p113);
        p113.time_usec_SET(4137620870310266769L);
        p113.fix_type_SET((char)77);
        p113.lat_SET(1671880426);
        p113.lon_SET(1633371816);
        p113.alt_SET(-1992214144);
        p113.eph_SET((char)23642);
        p113.epv_SET((char)9245);
        p113.vel_SET((char)52418);
        p113.vn_SET((short) -2310);
        p113.ve_SET((short) -22901);
        p113.vd_SET((short)15583);
        p113.cog_SET((char)43957);
        p113.satellites_visible_SET((char)221);
        LoopBackDemoChannel.instance.send(p113); //===============================
        HIL_OPTICAL_FLOW p114 = LoopBackDemoChannel.instance.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.time_usec_SET(3919306086958424916L);
        p114.sensor_id_SET((char)96);
        p114.integration_time_us_SET(848061292L);
        p114.integrated_x_SET(1.2555016E38F);
        p114.integrated_y_SET(-2.2004707E38F);
        p114.integrated_xgyro_SET(9.016165E37F);
        p114.integrated_ygyro_SET(3.3891827E38F);
        p114.integrated_zgyro_SET(-2.2481028E37F);
        p114.temperature_SET((short)12015);
        p114.quality_SET((char)43);
        p114.time_delta_distance_us_SET(103453145L);
        p114.distance_SET(2.4495558E37F);
        LoopBackDemoChannel.instance.send(p114); //===============================
        HIL_STATE_QUATERNION p115 = LoopBackDemoChannel.instance.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.time_usec_SET(8397256220196219399L);
        p115.attitude_quaternion_SET(new float[4], 0);
        p115.rollspeed_SET(-9.194367E37F);
        p115.pitchspeed_SET(-1.293249E38F);
        p115.yawspeed_SET(-1.4691707E38F);
        p115.lat_SET(-1108125915);
        p115.lon_SET(1754171229);
        p115.alt_SET(-1959721907);
        p115.vx_SET((short) -14794);
        p115.vy_SET((short)2406);
        p115.vz_SET((short)4758);
        p115.ind_airspeed_SET((char)21169);
        p115.true_airspeed_SET((char)41819);
        p115.xacc_SET((short) -3332);
        p115.yacc_SET((short) -30602);
        p115.zacc_SET((short) -25320);
        LoopBackDemoChannel.instance.send(p115); //===============================
        SCALED_IMU2 p116 = LoopBackDemoChannel.instance.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.time_boot_ms_SET(2754590893L);
        p116.xacc_SET((short)4895);
        p116.yacc_SET((short)28185);
        p116.zacc_SET((short)7134);
        p116.xgyro_SET((short)9262);
        p116.ygyro_SET((short) -7436);
        p116.zgyro_SET((short)1068);
        p116.xmag_SET((short)31191);
        p116.ymag_SET((short) -30968);
        p116.zmag_SET((short) -10828);
        LoopBackDemoChannel.instance.send(p116); //===============================
        LOG_REQUEST_LIST p117 = LoopBackDemoChannel.instance.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.target_system_SET((char)58);
        p117.target_component_SET((char)216);
        p117.start_SET((char)27769);
        p117.end_SET((char)41567);
        LoopBackDemoChannel.instance.send(p117); //===============================
        LOG_ENTRY p118 = LoopBackDemoChannel.instance.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.id_SET((char)27289);
        p118.num_logs_SET((char)28288);
        p118.last_log_num_SET((char)42655);
        p118.time_utc_SET(3986054244L);
        p118.size_SET(3465376130L);
        LoopBackDemoChannel.instance.send(p118); //===============================
        LOG_REQUEST_DATA p119 = LoopBackDemoChannel.instance.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.target_system_SET((char)210);
        p119.target_component_SET((char)79);
        p119.id_SET((char)5698);
        p119.ofs_SET(189672322L);
        p119.count_SET(3281165604L);
        LoopBackDemoChannel.instance.send(p119); //===============================
        LOG_DATA p120 = LoopBackDemoChannel.instance.new_LOG_DATA();
        PH.setPack(p120);
        p120.id_SET((char)40942);
        p120.ofs_SET(3963277599L);
        p120.count_SET((char)23);
        p120.data__SET(new char[90], 0);
        LoopBackDemoChannel.instance.send(p120); //===============================
        LOG_ERASE p121 = LoopBackDemoChannel.instance.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_system_SET((char)190);
        p121.target_component_SET((char)216);
        LoopBackDemoChannel.instance.send(p121); //===============================
        LOG_REQUEST_END p122 = LoopBackDemoChannel.instance.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_system_SET((char)109);
        p122.target_component_SET((char)155);
        LoopBackDemoChannel.instance.send(p122); //===============================
        GPS_INJECT_DATA p123 = LoopBackDemoChannel.instance.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.target_system_SET((char)237);
        p123.target_component_SET((char)1);
        p123.len_SET((char)157);
        p123.data__SET(new char[110], 0);
        LoopBackDemoChannel.instance.send(p123); //===============================
        GPS2_RAW p124 = LoopBackDemoChannel.instance.new_GPS2_RAW();
        PH.setPack(p124);
        p124.time_usec_SET(3826573865230788752L);
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT);
        p124.lat_SET(-1600119760);
        p124.lon_SET(30379844);
        p124.alt_SET(966672653);
        p124.eph_SET((char)23751);
        p124.epv_SET((char)29067);
        p124.vel_SET((char)61576);
        p124.cog_SET((char)28099);
        p124.satellites_visible_SET((char)216);
        p124.dgps_numch_SET((char)41);
        p124.dgps_age_SET(3174567006L);
        LoopBackDemoChannel.instance.send(p124); //===============================
        POWER_STATUS p125 = LoopBackDemoChannel.instance.new_POWER_STATUS();
        PH.setPack(p125);
        p125.Vcc_SET((char)21998);
        p125.Vservo_SET((char)9833);
        p125.flags_SET(MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID);
        LoopBackDemoChannel.instance.send(p125); //===============================
        SERIAL_CONTROL p126 = LoopBackDemoChannel.instance.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2);
        p126.flags_SET(SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE);
        p126.timeout_SET((char)13084);
        p126.baudrate_SET(4104081850L);
        p126.count_SET((char)235);
        p126.data__SET(new char[70], 0);
        LoopBackDemoChannel.instance.send(p126); //===============================
        GPS_RTK p127 = LoopBackDemoChannel.instance.new_GPS_RTK();
        PH.setPack(p127);
        p127.time_last_baseline_ms_SET(3794187354L);
        p127.rtk_receiver_id_SET((char)38);
        p127.wn_SET((char)20081);
        p127.tow_SET(4265206494L);
        p127.rtk_health_SET((char)33);
        p127.rtk_rate_SET((char)47);
        p127.nsats_SET((char)201);
        p127.baseline_coords_type_SET((char)36);
        p127.baseline_a_mm_SET(-1247240124);
        p127.baseline_b_mm_SET(-1004366800);
        p127.baseline_c_mm_SET(996484980);
        p127.accuracy_SET(3456692701L);
        p127.iar_num_hypotheses_SET(2055367622);
        LoopBackDemoChannel.instance.send(p127); //===============================
        GPS2_RTK p128 = LoopBackDemoChannel.instance.new_GPS2_RTK();
        PH.setPack(p128);
        p128.time_last_baseline_ms_SET(1041897651L);
        p128.rtk_receiver_id_SET((char)205);
        p128.wn_SET((char)39391);
        p128.tow_SET(2325972257L);
        p128.rtk_health_SET((char)134);
        p128.rtk_rate_SET((char)77);
        p128.nsats_SET((char)247);
        p128.baseline_coords_type_SET((char)138);
        p128.baseline_a_mm_SET(-592921021);
        p128.baseline_b_mm_SET(-1790680823);
        p128.baseline_c_mm_SET(-1046463710);
        p128.accuracy_SET(1324152348L);
        p128.iar_num_hypotheses_SET(-216946864);
        LoopBackDemoChannel.instance.send(p128); //===============================
        SCALED_IMU3 p129 = LoopBackDemoChannel.instance.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.time_boot_ms_SET(3067764280L);
        p129.xacc_SET((short)17979);
        p129.yacc_SET((short) -16257);
        p129.zacc_SET((short) -32729);
        p129.xgyro_SET((short) -10190);
        p129.ygyro_SET((short) -14460);
        p129.zgyro_SET((short) -11264);
        p129.xmag_SET((short)30887);
        p129.ymag_SET((short)30268);
        p129.zmag_SET((short) -17620);
        LoopBackDemoChannel.instance.send(p129); //===============================
        DATA_TRANSMISSION_HANDSHAKE p130 = LoopBackDemoChannel.instance.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.type_SET((char)132);
        p130.size_SET(3592848710L);
        p130.width_SET((char)22996);
        p130.height_SET((char)41870);
        p130.packets_SET((char)48202);
        p130.payload_SET((char)30);
        p130.jpg_quality_SET((char)150);
        LoopBackDemoChannel.instance.send(p130); //===============================
        ENCAPSULATED_DATA p131 = LoopBackDemoChannel.instance.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.seqnr_SET((char)23830);
        p131.data__SET(new char[253], 0);
        LoopBackDemoChannel.instance.send(p131); //===============================
        DISTANCE_SENSOR p132 = LoopBackDemoChannel.instance.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.time_boot_ms_SET(701045963L);
        p132.min_distance_SET((char)42663);
        p132.max_distance_SET((char)29579);
        p132.current_distance_SET((char)23265);
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN);
        p132.id_SET((char)12);
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_270_PITCH_90);
        p132.covariance_SET((char)70);
        LoopBackDemoChannel.instance.send(p132); //===============================
        TERRAIN_REQUEST p133 = LoopBackDemoChannel.instance.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.lat_SET(-2110609032);
        p133.lon_SET(-755170586);
        p133.grid_spacing_SET((char)14513);
        p133.mask_SET(6763696464802733849L);
        LoopBackDemoChannel.instance.send(p133); //===============================
        TERRAIN_DATA p134 = LoopBackDemoChannel.instance.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.lat_SET(-294058059);
        p134.lon_SET(1665397919);
        p134.grid_spacing_SET((char)32693);
        p134.gridbit_SET((char)128);
        p134.data__SET(new short[16], 0);
        LoopBackDemoChannel.instance.send(p134); //===============================
        TERRAIN_CHECK p135 = LoopBackDemoChannel.instance.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lat_SET(622618535);
        p135.lon_SET(-539694485);
        LoopBackDemoChannel.instance.send(p135); //===============================
        TERRAIN_REPORT p136 = LoopBackDemoChannel.instance.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.lat_SET(1792301029);
        p136.lon_SET(1229058317);
        p136.spacing_SET((char)41794);
        p136.terrain_height_SET(6.0082567E37F);
        p136.current_height_SET(1.7345997E38F);
        p136.pending_SET((char)12809);
        p136.loaded_SET((char)54438);
        LoopBackDemoChannel.instance.send(p136); //===============================
        SCALED_PRESSURE2 p137 = LoopBackDemoChannel.instance.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.time_boot_ms_SET(830168362L);
        p137.press_abs_SET(-3.0882346E38F);
        p137.press_diff_SET(-2.8171516E37F);
        p137.temperature_SET((short)6348);
        LoopBackDemoChannel.instance.send(p137); //===============================
        ATT_POS_MOCAP p138 = LoopBackDemoChannel.instance.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.time_usec_SET(8122611824759135802L);
        p138.q_SET(new float[4], 0);
        p138.x_SET(7.722017E37F);
        p138.y_SET(-1.2572944E38F);
        p138.z_SET(-1.5127085E38F);
        LoopBackDemoChannel.instance.send(p138); //===============================
        SET_ACTUATOR_CONTROL_TARGET p139 = LoopBackDemoChannel.instance.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.time_usec_SET(3946768988574843850L);
        p139.group_mlx_SET((char)116);
        p139.target_system_SET((char)241);
        p139.target_component_SET((char)202);
        p139.controls_SET(new float[8], 0);
        LoopBackDemoChannel.instance.send(p139); //===============================
        ACTUATOR_CONTROL_TARGET p140 = LoopBackDemoChannel.instance.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.time_usec_SET(558453746099099968L);
        p140.group_mlx_SET((char)183);
        p140.controls_SET(new float[8], 0);
        LoopBackDemoChannel.instance.send(p140); //===============================
        ALTITUDE p141 = LoopBackDemoChannel.instance.new_ALTITUDE();
        PH.setPack(p141);
        p141.time_usec_SET(158939499929046884L);
        p141.altitude_monotonic_SET(-8.861825E37F);
        p141.altitude_amsl_SET(5.3385584E37F);
        p141.altitude_local_SET(1.0728369E38F);
        p141.altitude_relative_SET(1.4143864E38F);
        p141.altitude_terrain_SET(3.0631704E38F);
        p141.bottom_clearance_SET(-1.8745989E38F);
        LoopBackDemoChannel.instance.send(p141); //===============================
        RESOURCE_REQUEST p142 = LoopBackDemoChannel.instance.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.request_id_SET((char)33);
        p142.uri_type_SET((char)124);
        p142.uri_SET(new char[120], 0);
        p142.transfer_type_SET((char)250);
        p142.storage_SET(new char[120], 0);
        LoopBackDemoChannel.instance.send(p142); //===============================
        SCALED_PRESSURE3 p143 = LoopBackDemoChannel.instance.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.time_boot_ms_SET(2600113304L);
        p143.press_abs_SET(2.318444E38F);
        p143.press_diff_SET(8.578378E37F);
        p143.temperature_SET((short) -27319);
        LoopBackDemoChannel.instance.send(p143); //===============================
        FOLLOW_TARGET p144 = LoopBackDemoChannel.instance.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.timestamp_SET(8829587912731445764L);
        p144.est_capabilities_SET((char)216);
        p144.lat_SET(-1387406561);
        p144.lon_SET(537565291);
        p144.alt_SET(-2.9386313E38F);
        p144.vel_SET(new float[3], 0);
        p144.acc_SET(new float[3], 0);
        p144.attitude_q_SET(new float[4], 0);
        p144.rates_SET(new float[3], 0);
        p144.position_cov_SET(new float[3], 0);
        p144.custom_state_SET(4416670612307028471L);
        LoopBackDemoChannel.instance.send(p144); //===============================
        CONTROL_SYSTEM_STATE p146 = LoopBackDemoChannel.instance.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.time_usec_SET(7106697704023543279L);
        p146.x_acc_SET(3.0240768E38F);
        p146.y_acc_SET(2.0049213E38F);
        p146.z_acc_SET(-1.1604869E38F);
        p146.x_vel_SET(2.4866198E38F);
        p146.y_vel_SET(9.280663E37F);
        p146.z_vel_SET(2.0216686E37F);
        p146.x_pos_SET(3.4879956E37F);
        p146.y_pos_SET(-3.3720751E38F);
        p146.z_pos_SET(-3.25329E38F);
        p146.airspeed_SET(2.2694158E38F);
        p146.vel_variance_SET(new float[3], 0);
        p146.pos_variance_SET(new float[3], 0);
        p146.q_SET(new float[4], 0);
        p146.roll_rate_SET(-8.998844E37F);
        p146.pitch_rate_SET(-2.0324891E38F);
        p146.yaw_rate_SET(-2.5360535E38F);
        LoopBackDemoChannel.instance.send(p146); //===============================
        BATTERY_STATUS p147 = LoopBackDemoChannel.instance.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.id_SET((char)185);
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_PROPULSION);
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_NIMH);
        p147.temperature_SET((short)13483);
        p147.voltages_SET(new char[10], 0);
        p147.current_battery_SET((short) -20284);
        p147.current_consumed_SET(103169241);
        p147.energy_consumed_SET(-985322874);
        p147.battery_remaining_SET((byte)43);
        LoopBackDemoChannel.instance.send(p147); //===============================
        AUTOPILOT_VERSION p148 = LoopBackDemoChannel.instance.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.capabilities_SET(MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY);
        p148.flight_sw_version_SET(787174336L);
        p148.middleware_sw_version_SET(729733625L);
        p148.os_sw_version_SET(3812531338L);
        p148.board_version_SET(1572283180L);
        p148.flight_custom_version_SET(new char[8], 0);
        p148.middleware_custom_version_SET(new char[8], 0);
        p148.os_custom_version_SET(new char[8], 0);
        p148.vendor_id_SET((char)42718);
        p148.product_id_SET((char)39527);
        p148.uid_SET(7897249426527961081L);
        p148.uid2_SET(new char[18], 0, PH);
        LoopBackDemoChannel.instance.send(p148); //===============================
        LANDING_TARGET p149 = LoopBackDemoChannel.instance.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.time_usec_SET(8231736184828780666L);
        p149.target_num_SET((char)22);
        p149.frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED);
        p149.angle_x_SET(2.1526963E38F);
        p149.angle_y_SET(3.7355077E37F);
        p149.distance_SET(1.1558154E38F);
        p149.size_x_SET(1.3933862E38F);
        p149.size_y_SET(3.2467438E38F);
        p149.x_SET(-2.1834994E38F, PH);
        p149.y_SET(-8.4814374E37F, PH);
        p149.z_SET(-1.1543567E38F, PH);
        p149.q_SET(new float[4], 0, PH);
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER);
        p149.position_valid_SET((char)97, PH);
        LoopBackDemoChannel.instance.send(p149); //===============================
        SENS_POWER p201 = LoopBackDemoChannel.instance.new_SENS_POWER();
        PH.setPack(p201);
        p201.adc121_vspb_volt_SET(2.5574871E38F);
        p201.adc121_cspb_amp_SET(2.0896934E38F);
        p201.adc121_cs1_amp_SET(2.4172773E38F);
        p201.adc121_cs2_amp_SET(-3.240892E38F);
        LoopBackDemoChannel.instance.send(p201); //===============================
        SENS_MPPT p202 = LoopBackDemoChannel.instance.new_SENS_MPPT();
        PH.setPack(p202);
        p202.mppt_timestamp_SET(8443072056031525024L);
        p202.mppt1_volt_SET(6.550092E37F);
        p202.mppt1_amp_SET(6.919675E37F);
        p202.mppt1_pwm_SET((char)25255);
        p202.mppt1_status_SET((char)219);
        p202.mppt2_volt_SET(2.3073892E38F);
        p202.mppt2_amp_SET(-2.0547288E38F);
        p202.mppt2_pwm_SET((char)51937);
        p202.mppt2_status_SET((char)6);
        p202.mppt3_volt_SET(2.6039376E37F);
        p202.mppt3_amp_SET(1.4140818E36F);
        p202.mppt3_pwm_SET((char)46385);
        p202.mppt3_status_SET((char)196);
        LoopBackDemoChannel.instance.send(p202); //===============================
        ASLCTRL_DATA p203 = LoopBackDemoChannel.instance.new_ASLCTRL_DATA();
        PH.setPack(p203);
        p203.timestamp_SET(5847981954631090805L);
        p203.aslctrl_mode_SET((char)148);
        p203.h_SET(-2.2754314E38F);
        p203.hRef_SET(-5.5413176E37F);
        p203.hRef_t_SET(-1.9102292E38F);
        p203.PitchAngle_SET(2.7503116E38F);
        p203.PitchAngleRef_SET(2.4291678E38F);
        p203.q_SET(1.7143467E38F);
        p203.qRef_SET(1.371933E38F);
        p203.uElev_SET(7.1991546E37F);
        p203.uThrot_SET(-2.6606763E38F);
        p203.uThrot2_SET(-1.2429255E38F);
        p203.nZ_SET(-1.5799878E37F);
        p203.AirspeedRef_SET(1.7608912E38F);
        p203.SpoilersEngaged_SET((char)64);
        p203.YawAngle_SET(1.0408797E38F);
        p203.YawAngleRef_SET(2.2337342E38F);
        p203.RollAngle_SET(-9.778124E37F);
        p203.RollAngleRef_SET(-8.127517E36F);
        p203.p_SET(1.529012E38F);
        p203.pRef_SET(-2.0714482E38F);
        p203.r_SET(-1.8581857E38F);
        p203.rRef_SET(2.526514E38F);
        p203.uAil_SET(-6.012342E37F);
        p203.uRud_SET(-6.284421E37F);
        LoopBackDemoChannel.instance.send(p203); //===============================
        ASLCTRL_DEBUG p204 = LoopBackDemoChannel.instance.new_ASLCTRL_DEBUG();
        PH.setPack(p204);
        p204.i32_1_SET(992682503L);
        p204.i8_1_SET((char)68);
        p204.i8_2_SET((char)126);
        p204.f_1_SET(-9.164998E37F);
        p204.f_2_SET(-1.0168254E38F);
        p204.f_3_SET(1.9936526E38F);
        p204.f_4_SET(7.1991297E37F);
        p204.f_5_SET(-9.001246E37F);
        p204.f_6_SET(-1.0078447E38F);
        p204.f_7_SET(2.805115E38F);
        p204.f_8_SET(-2.1703746E38F);
        LoopBackDemoChannel.instance.send(p204); //===============================
        ASLUAV_STATUS p205 = LoopBackDemoChannel.instance.new_ASLUAV_STATUS();
        PH.setPack(p205);
        p205.LED_status_SET((char)99);
        p205.SATCOM_status_SET((char)228);
        p205.Servo_status_SET(new char[8], 0);
        p205.Motor_rpm_SET(-5.000824E37F);
        LoopBackDemoChannel.instance.send(p205); //===============================
        EKF_EXT p206 = LoopBackDemoChannel.instance.new_EKF_EXT();
        PH.setPack(p206);
        p206.timestamp_SET(9213975015208035807L);
        p206.Windspeed_SET(-3.3898744E38F);
        p206.WindDir_SET(-2.3978743E38F);
        p206.WindZ_SET(-2.6729898E38F);
        p206.Airspeed_SET(-2.1461678E38F);
        p206.beta_SET(4.665998E37F);
        p206.alpha_SET(-1.7475552E36F);
        LoopBackDemoChannel.instance.send(p206); //===============================
        ASL_OBCTRL p207 = LoopBackDemoChannel.instance.new_ASL_OBCTRL();
        PH.setPack(p207);
        p207.timestamp_SET(1587996424086052887L);
        p207.uElev_SET(-2.713409E38F);
        p207.uThrot_SET(-8.1947983E37F);
        p207.uThrot2_SET(-2.557537E38F);
        p207.uAilL_SET(-2.8260812E38F);
        p207.uAilR_SET(1.759756E38F);
        p207.uRud_SET(9.017644E37F);
        p207.obctrl_status_SET((char)193);
        LoopBackDemoChannel.instance.send(p207); //===============================
        SENS_ATMOS p208 = LoopBackDemoChannel.instance.new_SENS_ATMOS();
        PH.setPack(p208);
        p208.TempAmbient_SET(3.2988433E38F);
        p208.Humidity_SET(-2.1406976E38F);
        LoopBackDemoChannel.instance.send(p208); //===============================
        SENS_BATMON p209 = LoopBackDemoChannel.instance.new_SENS_BATMON();
        PH.setPack(p209);
        p209.temperature_SET(1.6256844E38F);
        p209.voltage_SET((char)6442);
        p209.current_SET((short) -13474);
        p209.SoC_SET((char)160);
        p209.batterystatus_SET((char)44636);
        p209.serialnumber_SET((char)6970);
        p209.hostfetcontrol_SET((char)24359);
        p209.cellvoltage1_SET((char)10097);
        p209.cellvoltage2_SET((char)51086);
        p209.cellvoltage3_SET((char)52219);
        p209.cellvoltage4_SET((char)59799);
        p209.cellvoltage5_SET((char)55781);
        p209.cellvoltage6_SET((char)30972);
        LoopBackDemoChannel.instance.send(p209); //===============================
        FW_SOARING_DATA p210 = LoopBackDemoChannel.instance.new_FW_SOARING_DATA();
        PH.setPack(p210);
        p210.timestamp_SET(5289661631833210790L);
        p210.timestampModeChanged_SET(7971894697679024944L);
        p210.xW_SET(1.5560334E38F);
        p210.xR_SET(-7.0692224E37F);
        p210.xLat_SET(1.6061397E38F);
        p210.xLon_SET(1.1334939E38F);
        p210.VarW_SET(-1.6754649E38F);
        p210.VarR_SET(-3.2306013E38F);
        p210.VarLat_SET(-3.520937E37F);
        p210.VarLon_SET(1.0499412E38F);
        p210.LoiterRadius_SET(2.3642927E38F);
        p210.LoiterDirection_SET(2.0359742E38F);
        p210.DistToSoarPoint_SET(2.7904795E38F);
        p210.vSinkExp_SET(2.362311E37F);
        p210.z1_LocalUpdraftSpeed_SET(-4.2591356E37F);
        p210.z2_DeltaRoll_SET(3.1827575E38F);
        p210.z1_exp_SET(2.3364026E38F);
        p210.z2_exp_SET(2.2812464E38F);
        p210.ThermalGSNorth_SET(-2.3024786E38F);
        p210.ThermalGSEast_SET(2.5219302E38F);
        p210.TSE_dot_SET(1.5509606E38F);
        p210.DebugVar1_SET(-2.6780318E38F);
        p210.DebugVar2_SET(-8.2545863E37F);
        p210.ControlMode_SET((char)172);
        p210.valid_SET((char)252);
        LoopBackDemoChannel.instance.send(p210); //===============================
        SENSORPOD_STATUS p211 = LoopBackDemoChannel.instance.new_SENSORPOD_STATUS();
        PH.setPack(p211);
        p211.timestamp_SET(3974981571961992607L);
        p211.visensor_rate_1_SET((char)243);
        p211.visensor_rate_2_SET((char)32);
        p211.visensor_rate_3_SET((char)86);
        p211.visensor_rate_4_SET((char)93);
        p211.recording_nodes_count_SET((char)212);
        p211.cpu_temp_SET((char)25);
        p211.free_space_SET((char)41669);
        LoopBackDemoChannel.instance.send(p211); //===============================
        SENS_POWER_BOARD p212 = LoopBackDemoChannel.instance.new_SENS_POWER_BOARD();
        PH.setPack(p212);
        p212.timestamp_SET(2319086600286072566L);
        p212.pwr_brd_status_SET((char)149);
        p212.pwr_brd_led_status_SET((char)124);
        p212.pwr_brd_system_volt_SET(-1.5841157E38F);
        p212.pwr_brd_servo_volt_SET(1.7245614E38F);
        p212.pwr_brd_mot_l_amp_SET(-2.725637E38F);
        p212.pwr_brd_mot_r_amp_SET(-3.2296379E38F);
        p212.pwr_brd_servo_1_amp_SET(-3.3681077E36F);
        p212.pwr_brd_servo_2_amp_SET(-1.9767496E37F);
        p212.pwr_brd_servo_3_amp_SET(-2.6571151E38F);
        p212.pwr_brd_servo_4_amp_SET(-2.554914E38F);
        p212.pwr_brd_aux_amp_SET(1.0044796E38F);
        LoopBackDemoChannel.instance.send(p212); //===============================
        ESTIMATOR_STATUS p230 = LoopBackDemoChannel.instance.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.time_usec_SET(4780114691712470511L);
        p230.flags_SET(ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL);
        p230.vel_ratio_SET(-2.9502898E38F);
        p230.pos_horiz_ratio_SET(6.300898E37F);
        p230.pos_vert_ratio_SET(2.5415013E38F);
        p230.mag_ratio_SET(3.3315283E37F);
        p230.hagl_ratio_SET(5.865417E37F);
        p230.tas_ratio_SET(3.5606987E37F);
        p230.pos_horiz_accuracy_SET(-9.165416E37F);
        p230.pos_vert_accuracy_SET(1.1336359E38F);
        LoopBackDemoChannel.instance.send(p230); //===============================
        WIND_COV p231 = LoopBackDemoChannel.instance.new_WIND_COV();
        PH.setPack(p231);
        p231.time_usec_SET(3341652808905046077L);
        p231.wind_x_SET(1.7364734E38F);
        p231.wind_y_SET(-3.586358E37F);
        p231.wind_z_SET(-5.199797E37F);
        p231.var_horiz_SET(2.0506171E38F);
        p231.var_vert_SET(2.273079E38F);
        p231.wind_alt_SET(2.367867E38F);
        p231.horiz_accuracy_SET(-3.2866227E38F);
        p231.vert_accuracy_SET(-4.7219564E37F);
        LoopBackDemoChannel.instance.send(p231); //===============================
        GPS_INPUT p232 = LoopBackDemoChannel.instance.new_GPS_INPUT();
        PH.setPack(p232);
        p232.time_usec_SET(236440055833058757L);
        p232.gps_id_SET((char)8);
        p232.ignore_flags_SET(GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP);
        p232.time_week_ms_SET(2509779386L);
        p232.time_week_SET((char)1792);
        p232.fix_type_SET((char)144);
        p232.lat_SET(-51651649);
        p232.lon_SET(-393635352);
        p232.alt_SET(-8.84566E37F);
        p232.hdop_SET(3.911466E37F);
        p232.vdop_SET(-3.1332538E38F);
        p232.vn_SET(3.2473754E38F);
        p232.ve_SET(7.693276E37F);
        p232.vd_SET(1.4388867E38F);
        p232.speed_accuracy_SET(-2.3164994E38F);
        p232.horiz_accuracy_SET(1.0797641E37F);
        p232.vert_accuracy_SET(-1.0457207E38F);
        p232.satellites_visible_SET((char)99);
        LoopBackDemoChannel.instance.send(p232); //===============================
        GPS_RTCM_DATA p233 = LoopBackDemoChannel.instance.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.flags_SET((char)151);
        p233.len_SET((char)110);
        p233.data__SET(new char[180], 0);
        LoopBackDemoChannel.instance.send(p233); //===============================
        HIGH_LATENCY p234 = LoopBackDemoChannel.instance.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED);
        p234.custom_mode_SET(2511169917L);
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND);
        p234.roll_SET((short) -21378);
        p234.pitch_SET((short) -19853);
        p234.heading_SET((char)52581);
        p234.throttle_SET((byte)54);
        p234.heading_sp_SET((short) -7783);
        p234.latitude_SET(-657551294);
        p234.longitude_SET(-1389059033);
        p234.altitude_amsl_SET((short)18178);
        p234.altitude_sp_SET((short) -24627);
        p234.airspeed_SET((char)41);
        p234.airspeed_sp_SET((char)109);
        p234.groundspeed_SET((char)173);
        p234.climb_rate_SET((byte) - 23);
        p234.gps_nsat_SET((char)14);
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS);
        p234.battery_remaining_SET((char)238);
        p234.temperature_SET((byte) - 53);
        p234.temperature_air_SET((byte)70);
        p234.failsafe_SET((char)87);
        p234.wp_num_SET((char)208);
        p234.wp_distance_SET((char)47378);
        LoopBackDemoChannel.instance.send(p234); //===============================
        VIBRATION p241 = LoopBackDemoChannel.instance.new_VIBRATION();
        PH.setPack(p241);
        p241.time_usec_SET(8743330621217730686L);
        p241.vibration_x_SET(2.0664437E38F);
        p241.vibration_y_SET(-1.0344034E38F);
        p241.vibration_z_SET(5.820904E37F);
        p241.clipping_0_SET(3425725232L);
        p241.clipping_1_SET(1035588642L);
        p241.clipping_2_SET(1525677788L);
        LoopBackDemoChannel.instance.send(p241); //===============================
        HOME_POSITION p242 = LoopBackDemoChannel.instance.new_HOME_POSITION();
        PH.setPack(p242);
        p242.latitude_SET(786399009);
        p242.longitude_SET(-1573131046);
        p242.altitude_SET(-1641732297);
        p242.x_SET(5.2096043E37F);
        p242.y_SET(-4.278449E37F);
        p242.z_SET(-1.2066714E38F);
        p242.q_SET(new float[4], 0);
        p242.approach_x_SET(1.3361796E38F);
        p242.approach_y_SET(-2.6242945E38F);
        p242.approach_z_SET(2.137174E38F);
        p242.time_usec_SET(9050223976010601059L, PH);
        LoopBackDemoChannel.instance.send(p242); //===============================
        SET_HOME_POSITION p243 = LoopBackDemoChannel.instance.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.target_system_SET((char)83);
        p243.latitude_SET(1928036538);
        p243.longitude_SET(-1300601788);
        p243.altitude_SET(-493926166);
        p243.x_SET(-1.3367778E38F);
        p243.y_SET(1.4063291E38F);
        p243.z_SET(2.1014511E38F);
        p243.q_SET(new float[4], 0);
        p243.approach_x_SET(-5.3288654E37F);
        p243.approach_y_SET(-6.0587386E37F);
        p243.approach_z_SET(2.115072E38F);
        p243.time_usec_SET(8406853170085286607L, PH);
        LoopBackDemoChannel.instance.send(p243); //===============================
        MESSAGE_INTERVAL p244 = LoopBackDemoChannel.instance.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.message_id_SET((char)49864);
        p244.interval_us_SET(-410824115);
        LoopBackDemoChannel.instance.send(p244); //===============================
        EXTENDED_SYS_STATE p245 = LoopBackDemoChannel.instance.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_UNDEFINED);
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND);
        LoopBackDemoChannel.instance.send(p245); //===============================
        ADSB_VEHICLE p246 = LoopBackDemoChannel.instance.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.ICAO_address_SET(1553841947L);
        p246.lat_SET(-1384927088);
        p246.lon_SET(-142756141);
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
        p246.altitude_SET(-529791910);
        p246.heading_SET((char)2567);
        p246.hor_velocity_SET((char)31510);
        p246.ver_velocity_SET((short) -25299);
        p246.callsign_SET("DEMO", PH);
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_SERVICE_SURFACE);
        p246.tslc_SET((char)201);
        p246.flags_SET(ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE);
        p246.squawk_SET((char)1053);
        LoopBackDemoChannel.instance.send(p246); //===============================
        COLLISION p247 = LoopBackDemoChannel.instance.new_COLLISION();
        PH.setPack(p247);
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
        p247.id_SET(2376912880L);
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_HORIZONTALLY);
        p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE);
        p247.time_to_minimum_delta_SET(-2.6542131E38F);
        p247.altitude_minimum_delta_SET(3.2505575E38F);
        p247.horizontal_minimum_delta_SET(-7.343423E36F);
        LoopBackDemoChannel.instance.send(p247); //===============================
        V2_EXTENSION p248 = LoopBackDemoChannel.instance.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.target_network_SET((char)177);
        p248.target_system_SET((char)85);
        p248.target_component_SET((char)79);
        p248.message_type_SET((char)19976);
        p248.payload_SET(new char[249], 0);
        LoopBackDemoChannel.instance.send(p248); //===============================
        MEMORY_VECT p249 = LoopBackDemoChannel.instance.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.address_SET((char)58381);
        p249.ver_SET((char)174);
        p249.type_SET((char)111);
        p249.value_SET(new byte[32], 0);
        LoopBackDemoChannel.instance.send(p249); //===============================
        DEBUG_VECT p250 = LoopBackDemoChannel.instance.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.name_SET("DEMO", PH);
        p250.time_usec_SET(9167431606277683898L);
        p250.x_SET(2.7480888E37F);
        p250.y_SET(-1.0582527E38F);
        p250.z_SET(2.1503725E38F);
        LoopBackDemoChannel.instance.send(p250); //===============================
        NAMED_VALUE_FLOAT p251 = LoopBackDemoChannel.instance.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.time_boot_ms_SET(2138523904L);
        p251.name_SET("DEMO", PH);
        p251.value_SET(2.2985083E38F);
        LoopBackDemoChannel.instance.send(p251); //===============================
        NAMED_VALUE_INT p252 = LoopBackDemoChannel.instance.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.time_boot_ms_SET(1025998755L);
        p252.name_SET("DEMO", PH);
        p252.value_SET(-1557729672);
        LoopBackDemoChannel.instance.send(p252); //===============================
        STATUSTEXT p253 = LoopBackDemoChannel.instance.new_STATUSTEXT();
        PH.setPack(p253);
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_WARNING);
        p253.text_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p253); //===============================
        DEBUG p254 = LoopBackDemoChannel.instance.new_DEBUG();
        PH.setPack(p254);
        p254.time_boot_ms_SET(2877674336L);
        p254.ind_SET((char)58);
        p254.value_SET(3.1677039E38F);
        LoopBackDemoChannel.instance.send(p254); //===============================
        SETUP_SIGNING p256 = LoopBackDemoChannel.instance.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.target_system_SET((char)189);
        p256.target_component_SET((char)193);
        p256.secret_key_SET(new char[32], 0);
        p256.initial_timestamp_SET(1243910327462733989L);
        LoopBackDemoChannel.instance.send(p256); //===============================
        BUTTON_CHANGE p257 = LoopBackDemoChannel.instance.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.time_boot_ms_SET(1731379098L);
        p257.last_change_ms_SET(1973753263L);
        p257.state_SET((char)170);
        LoopBackDemoChannel.instance.send(p257); //===============================
        PLAY_TUNE p258 = LoopBackDemoChannel.instance.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.target_system_SET((char)45);
        p258.target_component_SET((char)138);
        p258.tune_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p258); //===============================
        CAMERA_INFORMATION p259 = LoopBackDemoChannel.instance.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.time_boot_ms_SET(222492688L);
        p259.vendor_name_SET(new char[32], 0);
        p259.model_name_SET(new char[32], 0);
        p259.firmware_version_SET(4126354701L);
        p259.focal_length_SET(-3.0057073E38F);
        p259.sensor_size_h_SET(-1.6130957E38F);
        p259.sensor_size_v_SET(1.3405125E38F);
        p259.resolution_h_SET((char)13533);
        p259.resolution_v_SET((char)42243);
        p259.lens_id_SET((char)152);
        p259.flags_SET(CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE);
        p259.cam_definition_version_SET((char)26825);
        p259.cam_definition_uri_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p259); //===============================
        CAMERA_SETTINGS p260 = LoopBackDemoChannel.instance.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.time_boot_ms_SET(2547960111L);
        p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_IMAGE);
        LoopBackDemoChannel.instance.send(p260); //===============================
        STORAGE_INFORMATION p261 = LoopBackDemoChannel.instance.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.time_boot_ms_SET(1324957592L);
        p261.storage_id_SET((char)197);
        p261.storage_count_SET((char)102);
        p261.status_SET((char)187);
        p261.total_capacity_SET(1.3313923E38F);
        p261.used_capacity_SET(2.0732251E38F);
        p261.available_capacity_SET(-3.1862092E38F);
        p261.read_speed_SET(4.6025655E37F);
        p261.write_speed_SET(-1.6385211E38F);
        LoopBackDemoChannel.instance.send(p261); //===============================
        CAMERA_CAPTURE_STATUS p262 = LoopBackDemoChannel.instance.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.time_boot_ms_SET(3770534199L);
        p262.image_status_SET((char)128);
        p262.video_status_SET((char)148);
        p262.image_interval_SET(-2.1095894E37F);
        p262.recording_time_ms_SET(373133406L);
        p262.available_capacity_SET(2.8357752E38F);
        LoopBackDemoChannel.instance.send(p262); //===============================
        CAMERA_IMAGE_CAPTURED p263 = LoopBackDemoChannel.instance.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.time_boot_ms_SET(1345332578L);
        p263.time_utc_SET(8034695393085458614L);
        p263.camera_id_SET((char)144);
        p263.lat_SET(-840715454);
        p263.lon_SET(862747891);
        p263.alt_SET(-2140327418);
        p263.relative_alt_SET(-758684414);
        p263.q_SET(new float[4], 0);
        p263.image_index_SET(413487719);
        p263.capture_result_SET((byte) - 108);
        p263.file_url_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p263); //===============================
        FLIGHT_INFORMATION p264 = LoopBackDemoChannel.instance.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.time_boot_ms_SET(3766399072L);
        p264.arming_time_utc_SET(214986111140966500L);
        p264.takeoff_time_utc_SET(2539183288463568867L);
        p264.flight_uuid_SET(3532585580440776686L);
        LoopBackDemoChannel.instance.send(p264); //===============================
        MOUNT_ORIENTATION p265 = LoopBackDemoChannel.instance.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.time_boot_ms_SET(4287444274L);
        p265.roll_SET(3.2261172E38F);
        p265.pitch_SET(-5.6069677E37F);
        p265.yaw_SET(-2.1784379E38F);
        LoopBackDemoChannel.instance.send(p265); //===============================
        LOGGING_DATA p266 = LoopBackDemoChannel.instance.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.target_system_SET((char)167);
        p266.target_component_SET((char)28);
        p266.sequence_SET((char)61418);
        p266.length_SET((char)180);
        p266.first_message_offset_SET((char)49);
        p266.data__SET(new char[249], 0);
        LoopBackDemoChannel.instance.send(p266); //===============================
        LOGGING_DATA_ACKED p267 = LoopBackDemoChannel.instance.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.target_system_SET((char)166);
        p267.target_component_SET((char)255);
        p267.sequence_SET((char)60394);
        p267.length_SET((char)106);
        p267.first_message_offset_SET((char)228);
        p267.data__SET(new char[249], 0);
        LoopBackDemoChannel.instance.send(p267); //===============================
        LOGGING_ACK p268 = LoopBackDemoChannel.instance.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.target_system_SET((char)130);
        p268.target_component_SET((char)144);
        p268.sequence_SET((char)42589);
        LoopBackDemoChannel.instance.send(p268); //===============================
        VIDEO_STREAM_INFORMATION p269 = LoopBackDemoChannel.instance.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.camera_id_SET((char)24);
        p269.status_SET((char)228);
        p269.framerate_SET(2.0998949E38F);
        p269.resolution_h_SET((char)38302);
        p269.resolution_v_SET((char)49744);
        p269.bitrate_SET(2810563454L);
        p269.rotation_SET((char)6559);
        p269.uri_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p269); //===============================
        SET_VIDEO_STREAM_SETTINGS p270 = LoopBackDemoChannel.instance.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.target_system_SET((char)131);
        p270.target_component_SET((char)59);
        p270.camera_id_SET((char)52);
        p270.framerate_SET(5.274896E37F);
        p270.resolution_h_SET((char)934);
        p270.resolution_v_SET((char)2291);
        p270.bitrate_SET(1217654961L);
        p270.rotation_SET((char)62448);
        p270.uri_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p270); //===============================
        WIFI_CONFIG_AP p299 = LoopBackDemoChannel.instance.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.ssid_SET("DEMO", PH);
        p299.password_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p299); //===============================
        PROTOCOL_VERSION p300 = LoopBackDemoChannel.instance.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.version_SET((char)1600);
        p300.min_version_SET((char)1125);
        p300.max_version_SET((char)61863);
        p300.spec_version_hash_SET(new char[8], 0);
        p300.library_version_hash_SET(new char[8], 0);
        LoopBackDemoChannel.instance.send(p300); //===============================
        UAVCAN_NODE_STATUS p310 = LoopBackDemoChannel.instance.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.time_usec_SET(6775885172439279691L);
        p310.uptime_sec_SET(4247269510L);
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR);
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OPERATIONAL);
        p310.sub_mode_SET((char)43);
        p310.vendor_specific_status_code_SET((char)41482);
        LoopBackDemoChannel.instance.send(p310); //===============================
        UAVCAN_NODE_INFO p311 = LoopBackDemoChannel.instance.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.time_usec_SET(3132367501405499785L);
        p311.uptime_sec_SET(3064046534L);
        p311.name_SET("DEMO", PH);
        p311.hw_version_major_SET((char)176);
        p311.hw_version_minor_SET((char)212);
        p311.hw_unique_id_SET(new char[16], 0);
        p311.sw_version_major_SET((char)76);
        p311.sw_version_minor_SET((char)48);
        p311.sw_vcs_commit_SET(34260084L);
        LoopBackDemoChannel.instance.send(p311); //===============================
        PARAM_EXT_REQUEST_READ p320 = LoopBackDemoChannel.instance.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.target_system_SET((char)218);
        p320.target_component_SET((char)126);
        p320.param_id_SET("DEMO", PH);
        p320.param_index_SET((short)21229);
        LoopBackDemoChannel.instance.send(p320); //===============================
        PARAM_EXT_REQUEST_LIST p321 = LoopBackDemoChannel.instance.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_system_SET((char)186);
        p321.target_component_SET((char)176);
        LoopBackDemoChannel.instance.send(p321); //===============================
        PARAM_EXT_VALUE p322 = LoopBackDemoChannel.instance.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_id_SET("DEMO", PH);
        p322.param_value_SET("DEMO", PH);
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64);
        p322.param_count_SET((char)38527);
        p322.param_index_SET((char)57684);
        LoopBackDemoChannel.instance.send(p322); //===============================
        PARAM_EXT_SET p323 = LoopBackDemoChannel.instance.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.target_system_SET((char)166);
        p323.target_component_SET((char)31);
        p323.param_id_SET("DEMO", PH);
        p323.param_value_SET("DEMO", PH);
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32);
        LoopBackDemoChannel.instance.send(p323); //===============================
        PARAM_EXT_ACK p324 = LoopBackDemoChannel.instance.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_id_SET("DEMO", PH);
        p324.param_value_SET("DEMO", PH);
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32);
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_ACCEPTED);
        LoopBackDemoChannel.instance.send(p324); //===============================
        OBSTACLE_DISTANCE p330 = LoopBackDemoChannel.instance.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.time_usec_SET(65645524011910699L);
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED);
        p330.distances_SET(new char[72], 0);
        p330.increment_SET((char)200);
        p330.min_distance_SET((char)41496);
        p330.max_distance_SET((char)30638);
        LoopBackDemoChannel.instance.send(p330); //===============================
    }
}

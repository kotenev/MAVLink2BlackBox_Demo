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
        LoopBackDemoChannel.instance.on_SENSOR_OFFSETS.add((src, ph, pack) ->
        {
            short  mag_ofs_x = pack.mag_ofs_x_GET();
            short  mag_ofs_y = pack.mag_ofs_y_GET();
            short  mag_ofs_z = pack.mag_ofs_z_GET();
            float  mag_declination = pack.mag_declination_GET();
            int  raw_press = pack.raw_press_GET();
            int  raw_temp = pack.raw_temp_GET();
            float  gyro_cal_x = pack.gyro_cal_x_GET();
            float  gyro_cal_y = pack.gyro_cal_y_GET();
            float  gyro_cal_z = pack.gyro_cal_z_GET();
            float  accel_cal_x = pack.accel_cal_x_GET();
            float  accel_cal_y = pack.accel_cal_y_GET();
            float  accel_cal_z = pack.accel_cal_z_GET();
        });
        LoopBackDemoChannel.instance.on_SET_MAG_OFFSETS.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            short  mag_ofs_x = pack.mag_ofs_x_GET();
            short  mag_ofs_y = pack.mag_ofs_y_GET();
            short  mag_ofs_z = pack.mag_ofs_z_GET();
        });
        LoopBackDemoChannel.instance.on_MEMINFO.add((src, ph, pack) ->
        {
            char  brkval = pack.brkval_GET();
            char  freemem = pack.freemem_GET();
            long  freemem32 = pack.freemem32_TRY(ph);
        });
        LoopBackDemoChannel.instance.on_AP_ADC.add((src, ph, pack) ->
        {
            char  adc1 = pack.adc1_GET();
            char  adc2 = pack.adc2_GET();
            char  adc3 = pack.adc3_GET();
            char  adc4 = pack.adc4_GET();
            char  adc5 = pack.adc5_GET();
            char  adc6 = pack.adc6_GET();
        });
        LoopBackDemoChannel.instance.on_DIGICAM_CONFIGURE.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  mode = pack.mode_GET();
            char  shutter_speed = pack.shutter_speed_GET();
            char  aperture = pack.aperture_GET();
            char  iso = pack.iso_GET();
            char  exposure_type = pack.exposure_type_GET();
            char  command_id = pack.command_id_GET();
            char  engine_cut_off = pack.engine_cut_off_GET();
            char  extra_param = pack.extra_param_GET();
            float  extra_value = pack.extra_value_GET();
        });
        LoopBackDemoChannel.instance.on_DIGICAM_CONTROL.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  session = pack.session_GET();
            char  zoom_pos = pack.zoom_pos_GET();
            byte  zoom_step = pack.zoom_step_GET();
            char  focus_lock = pack.focus_lock_GET();
            char  shot = pack.shot_GET();
            char  command_id = pack.command_id_GET();
            char  extra_param = pack.extra_param_GET();
            float  extra_value = pack.extra_value_GET();
        });
        LoopBackDemoChannel.instance.on_MOUNT_CONFIGURE.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            @MAV_MOUNT_MODE int  mount_mode = pack.mount_mode_GET();
            char  stab_roll = pack.stab_roll_GET();
            char  stab_pitch = pack.stab_pitch_GET();
            char  stab_yaw = pack.stab_yaw_GET();
        });
        LoopBackDemoChannel.instance.on_MOUNT_CONTROL.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            int  input_a = pack.input_a_GET();
            int  input_b = pack.input_b_GET();
            int  input_c = pack.input_c_GET();
            char  save_position = pack.save_position_GET();
        });
        LoopBackDemoChannel.instance.on_MOUNT_STATUS.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            int  pointing_a = pack.pointing_a_GET();
            int  pointing_b = pack.pointing_b_GET();
            int  pointing_c = pack.pointing_c_GET();
        });
        LoopBackDemoChannel.instance.on_FENCE_POINT.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  idx = pack.idx_GET();
            char  count = pack.count_GET();
            float  lat = pack.lat_GET();
            float  lng = pack.lng_GET();
        });
        LoopBackDemoChannel.instance.on_FENCE_FETCH_POINT.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  idx = pack.idx_GET();
        });
        LoopBackDemoChannel.instance.on_FENCE_STATUS.add((src, ph, pack) ->
        {
            char  breach_status = pack.breach_status_GET();
            char  breach_count = pack.breach_count_GET();
            @FENCE_BREACH int  breach_type = pack.breach_type_GET();
            long  breach_time = pack.breach_time_GET();
        });
        LoopBackDemoChannel.instance.on_AHRS.add((src, ph, pack) ->
        {
            float  omegaIx = pack.omegaIx_GET();
            float  omegaIy = pack.omegaIy_GET();
            float  omegaIz = pack.omegaIz_GET();
            float  accel_weight = pack.accel_weight_GET();
            float  renorm_val = pack.renorm_val_GET();
            float  error_rp = pack.error_rp_GET();
            float  error_yaw = pack.error_yaw_GET();
        });
        LoopBackDemoChannel.instance.on_SIMSTATE.add((src, ph, pack) ->
        {
            float  roll = pack.roll_GET();
            float  pitch = pack.pitch_GET();
            float  yaw = pack.yaw_GET();
            float  xacc = pack.xacc_GET();
            float  yacc = pack.yacc_GET();
            float  zacc = pack.zacc_GET();
            float  xgyro = pack.xgyro_GET();
            float  ygyro = pack.ygyro_GET();
            float  zgyro = pack.zgyro_GET();
            int  lat = pack.lat_GET();
            int  lng = pack.lng_GET();
        });
        LoopBackDemoChannel.instance.on_HWSTATUS.add((src, ph, pack) ->
        {
            char  Vcc = pack.Vcc_GET();
            char  I2Cerr = pack.I2Cerr_GET();
        });
        LoopBackDemoChannel.instance.on_RADIO.add((src, ph, pack) ->
        {
            char  rssi = pack.rssi_GET();
            char  remrssi = pack.remrssi_GET();
            char  txbuf = pack.txbuf_GET();
            char  noise = pack.noise_GET();
            char  remnoise = pack.remnoise_GET();
            char  rxerrors = pack.rxerrors_GET();
            char  fixed_ = pack.fixed__GET();
        });
        LoopBackDemoChannel.instance.on_LIMITS_STATUS.add((src, ph, pack) ->
        {
            @LIMITS_STATE int  limits_state = pack.limits_state_GET();
            long  last_trigger = pack.last_trigger_GET();
            long  last_action = pack.last_action_GET();
            long  last_recovery = pack.last_recovery_GET();
            long  last_clear = pack.last_clear_GET();
            char  breach_count = pack.breach_count_GET();
            @LIMIT_MODULE int  mods_enabled = pack.mods_enabled_GET();
            @LIMIT_MODULE int  mods_required = pack.mods_required_GET();
            @LIMIT_MODULE int  mods_triggered = pack.mods_triggered_GET();
        });
        LoopBackDemoChannel.instance.on_WIND.add((src, ph, pack) ->
        {
            float  direction = pack.direction_GET();
            float  speed = pack.speed_GET();
            float  speed_z = pack.speed_z_GET();
        });
        LoopBackDemoChannel.instance.on_DATA16.add((src, ph, pack) ->
        {
            char  type = pack.type_GET();
            char  len = pack.len_GET();
            char[]  data_ = pack.data__GET();
        });
        LoopBackDemoChannel.instance.on_DATA32.add((src, ph, pack) ->
        {
            char  type = pack.type_GET();
            char  len = pack.len_GET();
            char[]  data_ = pack.data__GET();
        });
        LoopBackDemoChannel.instance.on_DATA64.add((src, ph, pack) ->
        {
            char  type = pack.type_GET();
            char  len = pack.len_GET();
            char[]  data_ = pack.data__GET();
        });
        LoopBackDemoChannel.instance.on_DATA96.add((src, ph, pack) ->
        {
            char  type = pack.type_GET();
            char  len = pack.len_GET();
            char[]  data_ = pack.data__GET();
        });
        LoopBackDemoChannel.instance.on_RANGEFINDER.add((src, ph, pack) ->
        {
            float  distance = pack.distance_GET();
            float  voltage = pack.voltage_GET();
        });
        LoopBackDemoChannel.instance.on_AIRSPEED_AUTOCAL.add((src, ph, pack) ->
        {
            float  vx = pack.vx_GET();
            float  vy = pack.vy_GET();
            float  vz = pack.vz_GET();
            float  diff_pressure = pack.diff_pressure_GET();
            float  EAS2TAS = pack.EAS2TAS_GET();
            float  ratio = pack.ratio_GET();
            float  state_x = pack.state_x_GET();
            float  state_y = pack.state_y_GET();
            float  state_z = pack.state_z_GET();
            float  Pax = pack.Pax_GET();
            float  Pby = pack.Pby_GET();
            float  Pcz = pack.Pcz_GET();
        });
        LoopBackDemoChannel.instance.on_RALLY_POINT.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  idx = pack.idx_GET();
            char  count = pack.count_GET();
            int  lat = pack.lat_GET();
            int  lng = pack.lng_GET();
            short  alt = pack.alt_GET();
            short  break_alt = pack.break_alt_GET();
            char  land_dir = pack.land_dir_GET();
            @RALLY_FLAGS int  flags = pack.flags_GET();
        });
        LoopBackDemoChannel.instance.on_RALLY_FETCH_POINT.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  idx = pack.idx_GET();
        });
        LoopBackDemoChannel.instance.on_COMPASSMOT_STATUS.add((src, ph, pack) ->
        {
            char  throttle = pack.throttle_GET();
            float  current = pack.current_GET();
            char  interference = pack.interference_GET();
            float  CompensationX = pack.CompensationX_GET();
            float  CompensationY = pack.CompensationY_GET();
            float  CompensationZ = pack.CompensationZ_GET();
        });
        LoopBackDemoChannel.instance.on_AHRS2.add((src, ph, pack) ->
        {
            float  roll = pack.roll_GET();
            float  pitch = pack.pitch_GET();
            float  yaw = pack.yaw_GET();
            float  altitude = pack.altitude_GET();
            int  lat = pack.lat_GET();
            int  lng = pack.lng_GET();
        });
        LoopBackDemoChannel.instance.on_CAMERA_STATUS.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            char  target_system = pack.target_system_GET();
            char  cam_idx = pack.cam_idx_GET();
            char  img_idx = pack.img_idx_GET();
            @CAMERA_STATUS_TYPES int  event_id = pack.event_id_GET();
            float  p1 = pack.p1_GET();
            float  p2 = pack.p2_GET();
            float  p3 = pack.p3_GET();
            float  p4 = pack.p4_GET();
        });
        LoopBackDemoChannel.instance.on_CAMERA_FEEDBACK.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            char  target_system = pack.target_system_GET();
            char  cam_idx = pack.cam_idx_GET();
            char  img_idx = pack.img_idx_GET();
            int  lat = pack.lat_GET();
            int  lng = pack.lng_GET();
            float  alt_msl = pack.alt_msl_GET();
            float  alt_rel = pack.alt_rel_GET();
            float  roll = pack.roll_GET();
            float  pitch = pack.pitch_GET();
            float  yaw = pack.yaw_GET();
            float  foc_len = pack.foc_len_GET();
            @CAMERA_FEEDBACK_FLAGS int  flags = pack.flags_GET();
        });
        LoopBackDemoChannel.instance.on_BATTERY2.add((src, ph, pack) ->
        {
            char  voltage = pack.voltage_GET();
            short  current_battery = pack.current_battery_GET();
        });
        LoopBackDemoChannel.instance.on_AHRS3.add((src, ph, pack) ->
        {
            float  roll = pack.roll_GET();
            float  pitch = pack.pitch_GET();
            float  yaw = pack.yaw_GET();
            float  altitude = pack.altitude_GET();
            int  lat = pack.lat_GET();
            int  lng = pack.lng_GET();
            float  v1 = pack.v1_GET();
            float  v2 = pack.v2_GET();
            float  v3 = pack.v3_GET();
            float  v4 = pack.v4_GET();
        });
        LoopBackDemoChannel.instance.on_AUTOPILOT_VERSION_REQUEST.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
        });
        LoopBackDemoChannel.instance.on_REMOTE_LOG_DATA_BLOCK.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            @MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS int  seqno = pack.seqno_GET();
            char[]  data_ = pack.data__GET();
        });
        LoopBackDemoChannel.instance.on_REMOTE_LOG_BLOCK_STATUS.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            long  seqno = pack.seqno_GET();
            @MAV_REMOTE_LOG_DATA_BLOCK_STATUSES int  status = pack.status_GET();
        });
        LoopBackDemoChannel.instance.on_LED_CONTROL.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  instance = pack.instance_GET();
            char  pattern = pack.pattern_GET();
            char  custom_len = pack.custom_len_GET();
            char[]  custom_bytes = pack.custom_bytes_GET();
        });
        LoopBackDemoChannel.instance.on_MAG_CAL_PROGRESS.add((src, ph, pack) ->
        {
            char  compass_id = pack.compass_id_GET();
            char  cal_mask = pack.cal_mask_GET();
            @MAG_CAL_STATUS int  cal_status = pack.cal_status_GET();
            char  attempt = pack.attempt_GET();
            char  completion_pct = pack.completion_pct_GET();
            char[]  completion_mask = pack.completion_mask_GET();
            float  direction_x = pack.direction_x_GET();
            float  direction_y = pack.direction_y_GET();
            float  direction_z = pack.direction_z_GET();
        });
        LoopBackDemoChannel.instance.on_MAG_CAL_REPORT.add((src, ph, pack) ->
        {
            char  compass_id = pack.compass_id_GET();
            char  cal_mask = pack.cal_mask_GET();
            @MAG_CAL_STATUS int  cal_status = pack.cal_status_GET();
            char  autosaved = pack.autosaved_GET();
            float  fitness = pack.fitness_GET();
            float  ofs_x = pack.ofs_x_GET();
            float  ofs_y = pack.ofs_y_GET();
            float  ofs_z = pack.ofs_z_GET();
            float  diag_x = pack.diag_x_GET();
            float  diag_y = pack.diag_y_GET();
            float  diag_z = pack.diag_z_GET();
            float  offdiag_x = pack.offdiag_x_GET();
            float  offdiag_y = pack.offdiag_y_GET();
            float  offdiag_z = pack.offdiag_z_GET();
        });
        LoopBackDemoChannel.instance.on_EKF_STATUS_REPORT.add((src, ph, pack) ->
        {
            @EKF_STATUS_FLAGS int  flags = pack.flags_GET();
            float  velocity_variance = pack.velocity_variance_GET();
            float  pos_horiz_variance = pack.pos_horiz_variance_GET();
            float  pos_vert_variance = pack.pos_vert_variance_GET();
            float  compass_variance = pack.compass_variance_GET();
            float  terrain_alt_variance = pack.terrain_alt_variance_GET();
        });
        LoopBackDemoChannel.instance.on_PID_TUNING.add((src, ph, pack) ->
        {
            @PID_TUNING_AXIS int  axis = pack.axis_GET();
            float  desired = pack.desired_GET();
            float  achieved = pack.achieved_GET();
            float  FF = pack.FF_GET();
            float  P = pack.P_GET();
            float  I = pack.I_GET();
            float  D = pack.D_GET();
        });
        LoopBackDemoChannel.instance.on_GIMBAL_REPORT.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            float  delta_time = pack.delta_time_GET();
            float  delta_angle_x = pack.delta_angle_x_GET();
            float  delta_angle_y = pack.delta_angle_y_GET();
            float  delta_angle_z = pack.delta_angle_z_GET();
            float  delta_velocity_x = pack.delta_velocity_x_GET();
            float  delta_velocity_y = pack.delta_velocity_y_GET();
            float  delta_velocity_z = pack.delta_velocity_z_GET();
            float  joint_roll = pack.joint_roll_GET();
            float  joint_el = pack.joint_el_GET();
            float  joint_az = pack.joint_az_GET();
        });
        LoopBackDemoChannel.instance.on_GIMBAL_CONTROL.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            float  demanded_rate_x = pack.demanded_rate_x_GET();
            float  demanded_rate_y = pack.demanded_rate_y_GET();
            float  demanded_rate_z = pack.demanded_rate_z_GET();
        });
        LoopBackDemoChannel.instance.on_GIMBAL_TORQUE_CMD_REPORT.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            short  rl_torque_cmd = pack.rl_torque_cmd_GET();
            short  el_torque_cmd = pack.el_torque_cmd_GET();
            short  az_torque_cmd = pack.az_torque_cmd_GET();
        });
        LoopBackDemoChannel.instance.on_GOPRO_HEARTBEAT.add((src, ph, pack) ->
        {
            @GOPRO_HEARTBEAT_STATUS int  status = pack.status_GET();
            @GOPRO_CAPTURE_MODE int  capture_mode = pack.capture_mode_GET();
            @GOPRO_HEARTBEAT_FLAGS int  flags = pack.flags_GET();
        });
        LoopBackDemoChannel.instance.on_GOPRO_GET_REQUEST.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            @GOPRO_COMMAND int  cmd_id = pack.cmd_id_GET();
        });
        LoopBackDemoChannel.instance.on_GOPRO_GET_RESPONSE.add((src, ph, pack) ->
        {
            @GOPRO_COMMAND int  cmd_id = pack.cmd_id_GET();
            @GOPRO_REQUEST_STATUS int  status = pack.status_GET();
            char[]  value = pack.value_GET();
        });
        LoopBackDemoChannel.instance.on_GOPRO_SET_REQUEST.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            @GOPRO_COMMAND int  cmd_id = pack.cmd_id_GET();
            char[]  value = pack.value_GET();
        });
        LoopBackDemoChannel.instance.on_GOPRO_SET_RESPONSE.add((src, ph, pack) ->
        {
            @GOPRO_COMMAND int  cmd_id = pack.cmd_id_GET();
            @GOPRO_REQUEST_STATUS int  status = pack.status_GET();
        });
        LoopBackDemoChannel.instance.on_RPM.add((src, ph, pack) ->
        {
            float  rpm1 = pack.rpm1_GET();
            float  rpm2 = pack.rpm2_GET();
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
        LoopBackDemoChannel.instance.on_UAVIONIX_ADSB_OUT_CFG.add((src, ph, pack) ->
        {
            long  ICAO = pack.ICAO_GET();
            String callsign = pack.callsign_TRY(ph);
            @ADSB_EMITTER_TYPE int  emitterType = pack.emitterType_GET();
            @UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE int  aircraftSize = pack.aircraftSize_GET();
            @UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT int  gpsOffsetLat = pack.gpsOffsetLat_GET();
            @UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON int  gpsOffsetLon = pack.gpsOffsetLon_GET();
            char  stallSpeed = pack.stallSpeed_GET();
            @UAVIONIX_ADSB_OUT_RF_SELECT int  rfSelect = pack.rfSelect_GET();
        });
        LoopBackDemoChannel.instance.on_UAVIONIX_ADSB_OUT_DYNAMIC.add((src, ph, pack) ->
        {
            long  utcTime = pack.utcTime_GET();
            int  gpsLat = pack.gpsLat_GET();
            int  gpsLon = pack.gpsLon_GET();
            int  gpsAlt = pack.gpsAlt_GET();
            @UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX int  gpsFix = pack.gpsFix_GET();
            char  numSats = pack.numSats_GET();
            int  baroAltMSL = pack.baroAltMSL_GET();
            long  accuracyHor = pack.accuracyHor_GET();
            char  accuracyVert = pack.accuracyVert_GET();
            char  accuracyVel = pack.accuracyVel_GET();
            short  velVert = pack.velVert_GET();
            short  velNS = pack.velNS_GET();
            short  VelEW = pack.VelEW_GET();
            @UAVIONIX_ADSB_EMERGENCY_STATUS int  emergencyStatus = pack.emergencyStatus_GET();
            @UAVIONIX_ADSB_OUT_DYNAMIC_STATE int  state = pack.state_GET();
            char  squawk = pack.squawk_GET();
        });
        LoopBackDemoChannel.instance.on_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT.add((src, ph, pack) ->
        {
            @UAVIONIX_ADSB_RF_HEALTH int  rfHealth = pack.rfHealth_GET();
        });
        LoopBackDemoChannel.instance.on_DEVICE_OP_READ.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            long  request_id = pack.request_id_GET();
            @DEVICE_OP_BUSTYPE int  bustype = pack.bustype_GET();
            char  bus = pack.bus_GET();
            char  address = pack.address_GET();
            String busname = pack.busname_TRY(ph);
            char  regstart = pack.regstart_GET();
            char  count = pack.count_GET();
        });
        LoopBackDemoChannel.instance.on_DEVICE_OP_READ_REPLY.add((src, ph, pack) ->
        {
            long  request_id = pack.request_id_GET();
            char  result = pack.result_GET();
            char  regstart = pack.regstart_GET();
            char  count = pack.count_GET();
            char[]  data_ = pack.data__GET();
        });
        LoopBackDemoChannel.instance.on_DEVICE_OP_WRITE.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            long  request_id = pack.request_id_GET();
            @DEVICE_OP_BUSTYPE int  bustype = pack.bustype_GET();
            char  bus = pack.bus_GET();
            char  address = pack.address_GET();
            String busname = pack.busname_TRY(ph);
            char  regstart = pack.regstart_GET();
            char  count = pack.count_GET();
            char[]  data_ = pack.data__GET();
        });
        LoopBackDemoChannel.instance.on_DEVICE_OP_WRITE_REPLY.add((src, ph, pack) ->
        {
            long  request_id = pack.request_id_GET();
            char  result = pack.result_GET();
        });
        LoopBackDemoChannel.instance.on_ADAP_TUNING.add((src, ph, pack) ->
        {
            @PID_TUNING_AXIS int  axis = pack.axis_GET();
            float  desired = pack.desired_GET();
            float  achieved = pack.achieved_GET();
            float  error = pack.error_GET();
            float  theta = pack.theta_GET();
            float  omega = pack.omega_GET();
            float  sigma = pack.sigma_GET();
            float  theta_dot = pack.theta_dot_GET();
            float  omega_dot = pack.omega_dot_GET();
            float  sigma_dot = pack.sigma_dot_GET();
            float  f = pack.f_GET();
            float  f_dot = pack.f_dot_GET();
            float  u = pack.u_GET();
        });
        LoopBackDemoChannel.instance.on_VISION_POSITION_DELTA.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            long  time_delta_usec = pack.time_delta_usec_GET();
            float[]  angle_delta = pack.angle_delta_GET();
            float[]  position_delta = pack.position_delta_GET();
            float  confidence = pack.confidence_GET();
        });
        HEARTBEAT p0 = LoopBackDemoChannel.instance.new_HEARTBEAT();
        PH.setPack(p0);
        p0.type_SET(MAV_TYPE.MAV_TYPE_QUADROTOR);
        p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_FP);
        p0.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED);
        p0.custom_mode_SET(3470698428L);
        p0.system_status_SET(MAV_STATE.MAV_STATE_CRITICAL);
        p0.mavlink_version_SET((char)77);
        LoopBackDemoChannel.instance.send(p0); //===============================
        SYS_STATUS p1 = LoopBackDemoChannel.instance.new_SYS_STATUS();
        PH.setPack(p1);
        p1.onboard_control_sensors_present_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION);
        p1.onboard_control_sensors_enabled_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY);
        p1.onboard_control_sensors_health_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION);
        p1.load_SET((char)13013);
        p1.voltage_battery_SET((char)15401);
        p1.current_battery_SET((short)28705);
        p1.battery_remaining_SET((byte)83);
        p1.drop_rate_comm_SET((char)32600);
        p1.errors_comm_SET((char)41838);
        p1.errors_count1_SET((char)40731);
        p1.errors_count2_SET((char)10443);
        p1.errors_count3_SET((char)3288);
        p1.errors_count4_SET((char)47950);
        LoopBackDemoChannel.instance.send(p1); //===============================
        SYSTEM_TIME p2 = LoopBackDemoChannel.instance.new_SYSTEM_TIME();
        PH.setPack(p2);
        p2.time_unix_usec_SET(431983937855100462L);
        p2.time_boot_ms_SET(1621749292L);
        LoopBackDemoChannel.instance.send(p2); //===============================
        POSITION_TARGET_LOCAL_NED p3 = LoopBackDemoChannel.instance.new_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.time_boot_ms_SET(1070556793L);
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_ENU);
        p3.type_mask_SET((char)60007);
        p3.x_SET(1.2735183E38F);
        p3.y_SET(1.9205777E38F);
        p3.z_SET(-7.5747554E37F);
        p3.vx_SET(4.4159547E37F);
        p3.vy_SET(-2.538044E38F);
        p3.vz_SET(1.7838012E37F);
        p3.afx_SET(8.7572E37F);
        p3.afy_SET(1.4333427E38F);
        p3.afz_SET(3.9738075E37F);
        p3.yaw_SET(-1.8037757E38F);
        p3.yaw_rate_SET(-1.2292978E38F);
        LoopBackDemoChannel.instance.send(p3); //===============================
        PING p4 = LoopBackDemoChannel.instance.new_PING();
        PH.setPack(p4);
        p4.time_usec_SET(8656846016117788499L);
        p4.seq_SET(1164389215L);
        p4.target_system_SET((char)50);
        p4.target_component_SET((char)116);
        LoopBackDemoChannel.instance.send(p4); //===============================
        CHANGE_OPERATOR_CONTROL p5 = LoopBackDemoChannel.instance.new_CHANGE_OPERATOR_CONTROL();
        PH.setPack(p5);
        p5.target_system_SET((char)150);
        p5.control_request_SET((char)134);
        p5.version_SET((char)196);
        p5.passkey_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p5); //===============================
        CHANGE_OPERATOR_CONTROL_ACK p6 = LoopBackDemoChannel.instance.new_CHANGE_OPERATOR_CONTROL_ACK();
        PH.setPack(p6);
        p6.gcs_system_id_SET((char)183);
        p6.control_request_SET((char)92);
        p6.ack_SET((char)78);
        LoopBackDemoChannel.instance.send(p6); //===============================
        AUTH_KEY p7 = LoopBackDemoChannel.instance.new_AUTH_KEY();
        PH.setPack(p7);
        p7.key_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p7); //===============================
        SET_MODE p11 = LoopBackDemoChannel.instance.new_SET_MODE();
        PH.setPack(p11);
        p11.target_system_SET((char)109);
        p11.base_mode_SET(MAV_MODE.MAV_MODE_GUIDED_DISARMED);
        p11.custom_mode_SET(920476418L);
        LoopBackDemoChannel.instance.send(p11); //===============================
        PARAM_REQUEST_READ p20 = LoopBackDemoChannel.instance.new_PARAM_REQUEST_READ();
        PH.setPack(p20);
        p20.target_system_SET((char)234);
        p20.target_component_SET((char)224);
        p20.param_id_SET("DEMO", PH);
        p20.param_index_SET((short) -27402);
        LoopBackDemoChannel.instance.send(p20); //===============================
        PARAM_REQUEST_LIST p21 = LoopBackDemoChannel.instance.new_PARAM_REQUEST_LIST();
        PH.setPack(p21);
        p21.target_system_SET((char)142);
        p21.target_component_SET((char)6);
        LoopBackDemoChannel.instance.send(p21); //===============================
        PARAM_VALUE p22 = LoopBackDemoChannel.instance.new_PARAM_VALUE();
        PH.setPack(p22);
        p22.param_id_SET("DEMO", PH);
        p22.param_value_SET(2.9826786E38F);
        p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16);
        p22.param_count_SET((char)11497);
        p22.param_index_SET((char)37824);
        LoopBackDemoChannel.instance.send(p22); //===============================
        PARAM_SET p23 = LoopBackDemoChannel.instance.new_PARAM_SET();
        PH.setPack(p23);
        p23.target_system_SET((char)228);
        p23.target_component_SET((char)18);
        p23.param_id_SET("DEMO", PH);
        p23.param_value_SET(-2.9398274E38F);
        p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT64);
        LoopBackDemoChannel.instance.send(p23); //===============================
        GPS_RAW_INT p24 = LoopBackDemoChannel.instance.new_GPS_RAW_INT();
        PH.setPack(p24);
        p24.time_usec_SET(4136959530368407208L);
        p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS);
        p24.lat_SET(1033771763);
        p24.lon_SET(302785631);
        p24.alt_SET(713524541);
        p24.eph_SET((char)21648);
        p24.epv_SET((char)39567);
        p24.vel_SET((char)55897);
        p24.cog_SET((char)36512);
        p24.satellites_visible_SET((char)244);
        p24.alt_ellipsoid_SET(1249855214, PH);
        p24.h_acc_SET(1692536487L, PH);
        p24.v_acc_SET(3445063577L, PH);
        p24.vel_acc_SET(691998077L, PH);
        p24.hdg_acc_SET(618036723L, PH);
        LoopBackDemoChannel.instance.send(p24); //===============================
        GPS_STATUS p25 = LoopBackDemoChannel.instance.new_GPS_STATUS();
        PH.setPack(p25);
        p25.satellites_visible_SET((char)238);
        p25.satellite_prn_SET(new char[20], 0);
        p25.satellite_used_SET(new char[20], 0);
        p25.satellite_elevation_SET(new char[20], 0);
        p25.satellite_azimuth_SET(new char[20], 0);
        p25.satellite_snr_SET(new char[20], 0);
        LoopBackDemoChannel.instance.send(p25); //===============================
        SCALED_IMU p26 = LoopBackDemoChannel.instance.new_SCALED_IMU();
        PH.setPack(p26);
        p26.time_boot_ms_SET(2433541098L);
        p26.xacc_SET((short) -18837);
        p26.yacc_SET((short) -13145);
        p26.zacc_SET((short)24461);
        p26.xgyro_SET((short)11085);
        p26.ygyro_SET((short) -18129);
        p26.zgyro_SET((short) -15762);
        p26.xmag_SET((short)25357);
        p26.ymag_SET((short)3256);
        p26.zmag_SET((short)23274);
        LoopBackDemoChannel.instance.send(p26); //===============================
        RAW_IMU p27 = LoopBackDemoChannel.instance.new_RAW_IMU();
        PH.setPack(p27);
        p27.time_usec_SET(2220134821959876841L);
        p27.xacc_SET((short)23009);
        p27.yacc_SET((short) -30374);
        p27.zacc_SET((short)24054);
        p27.xgyro_SET((short)27474);
        p27.ygyro_SET((short)6169);
        p27.zgyro_SET((short) -15795);
        p27.xmag_SET((short) -29353);
        p27.ymag_SET((short) -21499);
        p27.zmag_SET((short)13956);
        LoopBackDemoChannel.instance.send(p27); //===============================
        RAW_PRESSURE p28 = LoopBackDemoChannel.instance.new_RAW_PRESSURE();
        PH.setPack(p28);
        p28.time_usec_SET(347445848457324936L);
        p28.press_abs_SET((short)22219);
        p28.press_diff1_SET((short) -13341);
        p28.press_diff2_SET((short) -17568);
        p28.temperature_SET((short)9429);
        LoopBackDemoChannel.instance.send(p28); //===============================
        SCALED_PRESSURE p29 = LoopBackDemoChannel.instance.new_SCALED_PRESSURE();
        PH.setPack(p29);
        p29.time_boot_ms_SET(1770533687L);
        p29.press_abs_SET(1.9670487E38F);
        p29.press_diff_SET(-1.5872561E38F);
        p29.temperature_SET((short)18642);
        LoopBackDemoChannel.instance.send(p29); //===============================
        ATTITUDE p30 = LoopBackDemoChannel.instance.new_ATTITUDE();
        PH.setPack(p30);
        p30.time_boot_ms_SET(4148474242L);
        p30.roll_SET(9.194948E37F);
        p30.pitch_SET(-2.0434801E38F);
        p30.yaw_SET(-3.4720623E37F);
        p30.rollspeed_SET(1.052897E38F);
        p30.pitchspeed_SET(2.834118E38F);
        p30.yawspeed_SET(-9.288267E37F);
        LoopBackDemoChannel.instance.send(p30); //===============================
        ATTITUDE_QUATERNION p31 = LoopBackDemoChannel.instance.new_ATTITUDE_QUATERNION();
        PH.setPack(p31);
        p31.time_boot_ms_SET(67741960L);
        p31.q1_SET(1.7273764E38F);
        p31.q2_SET(1.0179571E38F);
        p31.q3_SET(-3.382245E38F);
        p31.q4_SET(-1.7239914E38F);
        p31.rollspeed_SET(9.930715E37F);
        p31.pitchspeed_SET(-2.1667988E38F);
        p31.yawspeed_SET(2.43798E38F);
        LoopBackDemoChannel.instance.send(p31); //===============================
        LOCAL_POSITION_NED p32 = LoopBackDemoChannel.instance.new_LOCAL_POSITION_NED();
        PH.setPack(p32);
        p32.time_boot_ms_SET(1190918138L);
        p32.x_SET(1.2532125E38F);
        p32.y_SET(6.0867577E37F);
        p32.z_SET(-1.2059806E38F);
        p32.vx_SET(1.416051E38F);
        p32.vy_SET(-1.3641433E38F);
        p32.vz_SET(-1.0745102E38F);
        LoopBackDemoChannel.instance.send(p32); //===============================
        GLOBAL_POSITION_INT p33 = LoopBackDemoChannel.instance.new_GLOBAL_POSITION_INT();
        PH.setPack(p33);
        p33.time_boot_ms_SET(516237176L);
        p33.lat_SET(151873051);
        p33.lon_SET(1376995743);
        p33.alt_SET(1253802378);
        p33.relative_alt_SET(232456196);
        p33.vx_SET((short) -18189);
        p33.vy_SET((short)9516);
        p33.vz_SET((short)21557);
        p33.hdg_SET((char)46369);
        LoopBackDemoChannel.instance.send(p33); //===============================
        RC_CHANNELS_SCALED p34 = LoopBackDemoChannel.instance.new_RC_CHANNELS_SCALED();
        PH.setPack(p34);
        p34.time_boot_ms_SET(1838097547L);
        p34.port_SET((char)83);
        p34.chan1_scaled_SET((short) -25815);
        p34.chan2_scaled_SET((short) -1910);
        p34.chan3_scaled_SET((short) -32124);
        p34.chan4_scaled_SET((short) -24978);
        p34.chan5_scaled_SET((short)2533);
        p34.chan6_scaled_SET((short) -22414);
        p34.chan7_scaled_SET((short)3144);
        p34.chan8_scaled_SET((short)5519);
        p34.rssi_SET((char)54);
        LoopBackDemoChannel.instance.send(p34); //===============================
        RC_CHANNELS_RAW p35 = LoopBackDemoChannel.instance.new_RC_CHANNELS_RAW();
        PH.setPack(p35);
        p35.time_boot_ms_SET(1177603023L);
        p35.port_SET((char)11);
        p35.chan1_raw_SET((char)54064);
        p35.chan2_raw_SET((char)8730);
        p35.chan3_raw_SET((char)41820);
        p35.chan4_raw_SET((char)21032);
        p35.chan5_raw_SET((char)766);
        p35.chan6_raw_SET((char)60700);
        p35.chan7_raw_SET((char)13920);
        p35.chan8_raw_SET((char)10661);
        p35.rssi_SET((char)230);
        LoopBackDemoChannel.instance.send(p35); //===============================
        SERVO_OUTPUT_RAW p36 = LoopBackDemoChannel.instance.new_SERVO_OUTPUT_RAW();
        PH.setPack(p36);
        p36.time_usec_SET(2729011374L);
        p36.port_SET((char)34);
        p36.servo1_raw_SET((char)32293);
        p36.servo2_raw_SET((char)25810);
        p36.servo3_raw_SET((char)4876);
        p36.servo4_raw_SET((char)30816);
        p36.servo5_raw_SET((char)17213);
        p36.servo6_raw_SET((char)33617);
        p36.servo7_raw_SET((char)63663);
        p36.servo8_raw_SET((char)54492);
        p36.servo9_raw_SET((char)42311, PH);
        p36.servo10_raw_SET((char)15334, PH);
        p36.servo11_raw_SET((char)55953, PH);
        p36.servo12_raw_SET((char)18972, PH);
        p36.servo13_raw_SET((char)46790, PH);
        p36.servo14_raw_SET((char)8711, PH);
        p36.servo15_raw_SET((char)59602, PH);
        p36.servo16_raw_SET((char)42626, PH);
        LoopBackDemoChannel.instance.send(p36); //===============================
        MISSION_REQUEST_PARTIAL_LIST p37 = LoopBackDemoChannel.instance.new_MISSION_REQUEST_PARTIAL_LIST();
        PH.setPack(p37);
        p37.target_system_SET((char)58);
        p37.target_component_SET((char)71);
        p37.start_index_SET((short)4488);
        p37.end_index_SET((short)20102);
        p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
        LoopBackDemoChannel.instance.send(p37); //===============================
        MISSION_WRITE_PARTIAL_LIST p38 = LoopBackDemoChannel.instance.new_MISSION_WRITE_PARTIAL_LIST();
        PH.setPack(p38);
        p38.target_system_SET((char)0);
        p38.target_component_SET((char)4);
        p38.start_index_SET((short) -10017);
        p38.end_index_SET((short)12402);
        p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
        LoopBackDemoChannel.instance.send(p38); //===============================
        MISSION_ITEM p39 = LoopBackDemoChannel.instance.new_MISSION_ITEM();
        PH.setPack(p39);
        p39.target_system_SET((char)207);
        p39.target_component_SET((char)179);
        p39.seq_SET((char)62063);
        p39.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_ENU);
        p39.command_SET(MAV_CMD.MAV_CMD_DO_SET_REVERSE);
        p39.current_SET((char)6);
        p39.autocontinue_SET((char)168);
        p39.param1_SET(-2.8732E38F);
        p39.param2_SET(1.5390691E38F);
        p39.param3_SET(-2.3481077E38F);
        p39.param4_SET(1.4414591E38F);
        p39.x_SET(7.043932E37F);
        p39.y_SET(2.1062907E38F);
        p39.z_SET(1.1129364E38F);
        p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
        LoopBackDemoChannel.instance.send(p39); //===============================
        MISSION_REQUEST p40 = LoopBackDemoChannel.instance.new_MISSION_REQUEST();
        PH.setPack(p40);
        p40.target_system_SET((char)83);
        p40.target_component_SET((char)185);
        p40.seq_SET((char)44029);
        p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
        LoopBackDemoChannel.instance.send(p40); //===============================
        MISSION_SET_CURRENT p41 = LoopBackDemoChannel.instance.new_MISSION_SET_CURRENT();
        PH.setPack(p41);
        p41.target_system_SET((char)40);
        p41.target_component_SET((char)48);
        p41.seq_SET((char)38543);
        LoopBackDemoChannel.instance.send(p41); //===============================
        MISSION_CURRENT p42 = LoopBackDemoChannel.instance.new_MISSION_CURRENT();
        PH.setPack(p42);
        p42.seq_SET((char)39896);
        LoopBackDemoChannel.instance.send(p42); //===============================
        MISSION_REQUEST_LIST p43 = LoopBackDemoChannel.instance.new_MISSION_REQUEST_LIST();
        PH.setPack(p43);
        p43.target_system_SET((char)80);
        p43.target_component_SET((char)212);
        p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
        LoopBackDemoChannel.instance.send(p43); //===============================
        MISSION_COUNT p44 = LoopBackDemoChannel.instance.new_MISSION_COUNT();
        PH.setPack(p44);
        p44.target_system_SET((char)255);
        p44.target_component_SET((char)203);
        p44.count_SET((char)11513);
        p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
        LoopBackDemoChannel.instance.send(p44); //===============================
        MISSION_CLEAR_ALL p45 = LoopBackDemoChannel.instance.new_MISSION_CLEAR_ALL();
        PH.setPack(p45);
        p45.target_system_SET((char)138);
        p45.target_component_SET((char)109);
        p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
        LoopBackDemoChannel.instance.send(p45); //===============================
        MISSION_ITEM_REACHED p46 = LoopBackDemoChannel.instance.new_MISSION_ITEM_REACHED();
        PH.setPack(p46);
        p46.seq_SET((char)57640);
        LoopBackDemoChannel.instance.send(p46); //===============================
        MISSION_ACK p47 = LoopBackDemoChannel.instance.new_MISSION_ACK();
        PH.setPack(p47);
        p47.target_system_SET((char)211);
        p47.target_component_SET((char)232);
        p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_INVALID_SEQUENCE);
        p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
        LoopBackDemoChannel.instance.send(p47); //===============================
        SET_GPS_GLOBAL_ORIGIN p48 = LoopBackDemoChannel.instance.new_SET_GPS_GLOBAL_ORIGIN();
        PH.setPack(p48);
        p48.target_system_SET((char)198);
        p48.latitude_SET(-971473418);
        p48.longitude_SET(472083830);
        p48.altitude_SET(311067648);
        p48.time_usec_SET(8762445144172428187L, PH);
        LoopBackDemoChannel.instance.send(p48); //===============================
        GPS_GLOBAL_ORIGIN p49 = LoopBackDemoChannel.instance.new_GPS_GLOBAL_ORIGIN();
        PH.setPack(p49);
        p49.latitude_SET(168002651);
        p49.longitude_SET(-936624666);
        p49.altitude_SET(1370837029);
        p49.time_usec_SET(4398002251260824470L, PH);
        LoopBackDemoChannel.instance.send(p49); //===============================
        PARAM_MAP_RC p50 = LoopBackDemoChannel.instance.new_PARAM_MAP_RC();
        PH.setPack(p50);
        p50.target_system_SET((char)211);
        p50.target_component_SET((char)115);
        p50.param_id_SET("DEMO", PH);
        p50.param_index_SET((short)18882);
        p50.parameter_rc_channel_index_SET((char)41);
        p50.param_value0_SET(-1.0633843E38F);
        p50.scale_SET(-2.9380103E38F);
        p50.param_value_min_SET(-8.727103E37F);
        p50.param_value_max_SET(-1.4451703E38F);
        LoopBackDemoChannel.instance.send(p50); //===============================
        MISSION_REQUEST_INT p51 = LoopBackDemoChannel.instance.new_MISSION_REQUEST_INT();
        PH.setPack(p51);
        p51.target_system_SET((char)146);
        p51.target_component_SET((char)111);
        p51.seq_SET((char)56858);
        p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
        LoopBackDemoChannel.instance.send(p51); //===============================
        SAFETY_SET_ALLOWED_AREA p54 = LoopBackDemoChannel.instance.new_SAFETY_SET_ALLOWED_AREA();
        PH.setPack(p54);
        p54.target_system_SET((char)34);
        p54.target_component_SET((char)9);
        p54.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_ENU);
        p54.p1x_SET(-2.497015E38F);
        p54.p1y_SET(6.1771275E37F);
        p54.p1z_SET(-3.0414627E38F);
        p54.p2x_SET(2.4371043E38F);
        p54.p2y_SET(3.3975752E38F);
        p54.p2z_SET(5.2307163E37F);
        LoopBackDemoChannel.instance.send(p54); //===============================
        SAFETY_ALLOWED_AREA p55 = LoopBackDemoChannel.instance.new_SAFETY_ALLOWED_AREA();
        PH.setPack(p55);
        p55.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
        p55.p1x_SET(-1.4065339E38F);
        p55.p1y_SET(6.1143053E37F);
        p55.p1z_SET(-2.9275567E38F);
        p55.p2x_SET(-1.6905037E37F);
        p55.p2y_SET(2.1844575E38F);
        p55.p2z_SET(3.009934E38F);
        LoopBackDemoChannel.instance.send(p55); //===============================
        ATTITUDE_QUATERNION_COV p61 = LoopBackDemoChannel.instance.new_ATTITUDE_QUATERNION_COV();
        PH.setPack(p61);
        p61.time_usec_SET(671166435793952793L);
        p61.q_SET(new float[4], 0);
        p61.rollspeed_SET(3.7524803E37F);
        p61.pitchspeed_SET(9.337123E37F);
        p61.yawspeed_SET(8.688238E37F);
        p61.covariance_SET(new float[9], 0);
        LoopBackDemoChannel.instance.send(p61); //===============================
        NAV_CONTROLLER_OUTPUT p62 = LoopBackDemoChannel.instance.new_NAV_CONTROLLER_OUTPUT();
        PH.setPack(p62);
        p62.nav_roll_SET(-7.2784497E37F);
        p62.nav_pitch_SET(2.4134617E38F);
        p62.nav_bearing_SET((short) -19549);
        p62.target_bearing_SET((short)20059);
        p62.wp_dist_SET((char)12126);
        p62.alt_error_SET(-3.1288197E38F);
        p62.aspd_error_SET(5.2572127E37F);
        p62.xtrack_error_SET(-1.9625744E38F);
        LoopBackDemoChannel.instance.send(p62); //===============================
        GLOBAL_POSITION_INT_COV p63 = LoopBackDemoChannel.instance.new_GLOBAL_POSITION_INT_COV();
        PH.setPack(p63);
        p63.time_usec_SET(3250043485103405609L);
        p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS);
        p63.lat_SET(-335101323);
        p63.lon_SET(1080348994);
        p63.alt_SET(-1989310801);
        p63.relative_alt_SET(33984040);
        p63.vx_SET(8.653359E37F);
        p63.vy_SET(-2.6856048E38F);
        p63.vz_SET(2.698608E38F);
        p63.covariance_SET(new float[36], 0);
        LoopBackDemoChannel.instance.send(p63); //===============================
        LOCAL_POSITION_NED_COV p64 = LoopBackDemoChannel.instance.new_LOCAL_POSITION_NED_COV();
        PH.setPack(p64);
        p64.time_usec_SET(3612301027920182859L);
        p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION);
        p64.x_SET(2.3467133E38F);
        p64.y_SET(-7.7396265E37F);
        p64.z_SET(1.2862845E38F);
        p64.vx_SET(3.924304E37F);
        p64.vy_SET(-1.9858187E38F);
        p64.vz_SET(2.8461161E38F);
        p64.ax_SET(-1.5405509E38F);
        p64.ay_SET(6.9814437E37F);
        p64.az_SET(-3.3372206E38F);
        p64.covariance_SET(new float[45], 0);
        LoopBackDemoChannel.instance.send(p64); //===============================
        RC_CHANNELS p65 = LoopBackDemoChannel.instance.new_RC_CHANNELS();
        PH.setPack(p65);
        p65.time_boot_ms_SET(274863044L);
        p65.chancount_SET((char)118);
        p65.chan1_raw_SET((char)46602);
        p65.chan2_raw_SET((char)50968);
        p65.chan3_raw_SET((char)61685);
        p65.chan4_raw_SET((char)21434);
        p65.chan5_raw_SET((char)22802);
        p65.chan6_raw_SET((char)36544);
        p65.chan7_raw_SET((char)59344);
        p65.chan8_raw_SET((char)60750);
        p65.chan9_raw_SET((char)58519);
        p65.chan10_raw_SET((char)26434);
        p65.chan11_raw_SET((char)14609);
        p65.chan12_raw_SET((char)12554);
        p65.chan13_raw_SET((char)31488);
        p65.chan14_raw_SET((char)1150);
        p65.chan15_raw_SET((char)38846);
        p65.chan16_raw_SET((char)9854);
        p65.chan17_raw_SET((char)30406);
        p65.chan18_raw_SET((char)58481);
        p65.rssi_SET((char)121);
        LoopBackDemoChannel.instance.send(p65); //===============================
        REQUEST_DATA_STREAM p66 = LoopBackDemoChannel.instance.new_REQUEST_DATA_STREAM();
        PH.setPack(p66);
        p66.target_system_SET((char)90);
        p66.target_component_SET((char)3);
        p66.req_stream_id_SET((char)98);
        p66.req_message_rate_SET((char)9548);
        p66.start_stop_SET((char)185);
        LoopBackDemoChannel.instance.send(p66); //===============================
        DATA_STREAM p67 = LoopBackDemoChannel.instance.new_DATA_STREAM();
        PH.setPack(p67);
        p67.stream_id_SET((char)71);
        p67.message_rate_SET((char)11453);
        p67.on_off_SET((char)209);
        LoopBackDemoChannel.instance.send(p67); //===============================
        MANUAL_CONTROL p69 = LoopBackDemoChannel.instance.new_MANUAL_CONTROL();
        PH.setPack(p69);
        p69.target_SET((char)196);
        p69.x_SET((short) -18271);
        p69.y_SET((short) -10835);
        p69.z_SET((short) -2221);
        p69.r_SET((short) -5126);
        p69.buttons_SET((char)48360);
        LoopBackDemoChannel.instance.send(p69); //===============================
        RC_CHANNELS_OVERRIDE p70 = LoopBackDemoChannel.instance.new_RC_CHANNELS_OVERRIDE();
        PH.setPack(p70);
        p70.target_system_SET((char)69);
        p70.target_component_SET((char)112);
        p70.chan1_raw_SET((char)28648);
        p70.chan2_raw_SET((char)30914);
        p70.chan3_raw_SET((char)46033);
        p70.chan4_raw_SET((char)54910);
        p70.chan5_raw_SET((char)61566);
        p70.chan6_raw_SET((char)46964);
        p70.chan7_raw_SET((char)39063);
        p70.chan8_raw_SET((char)23788);
        LoopBackDemoChannel.instance.send(p70); //===============================
        MISSION_ITEM_INT p73 = LoopBackDemoChannel.instance.new_MISSION_ITEM_INT();
        PH.setPack(p73);
        p73.target_system_SET((char)110);
        p73.target_component_SET((char)161);
        p73.seq_SET((char)57995);
        p73.frame_SET(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
        p73.command_SET(MAV_CMD.MAV_CMD_SET_FACTORY_TEST_MODE);
        p73.current_SET((char)99);
        p73.autocontinue_SET((char)187);
        p73.param1_SET(-2.2862947E38F);
        p73.param2_SET(-3.2143774E38F);
        p73.param3_SET(1.9002036E37F);
        p73.param4_SET(-1.720527E38F);
        p73.x_SET(1720778067);
        p73.y_SET(-1678710811);
        p73.z_SET(1.6085233E38F);
        p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
        LoopBackDemoChannel.instance.send(p73); //===============================
        VFR_HUD p74 = LoopBackDemoChannel.instance.new_VFR_HUD();
        PH.setPack(p74);
        p74.airspeed_SET(-7.1952477E37F);
        p74.groundspeed_SET(2.3417591E38F);
        p74.heading_SET((short)14033);
        p74.throttle_SET((char)11986);
        p74.alt_SET(-1.9830234E37F);
        p74.climb_SET(2.2057617E38F);
        LoopBackDemoChannel.instance.send(p74); //===============================
        COMMAND_INT p75 = LoopBackDemoChannel.instance.new_COMMAND_INT();
        PH.setPack(p75);
        p75.target_system_SET((char)200);
        p75.target_component_SET((char)32);
        p75.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL);
        p75.command_SET(MAV_CMD.MAV_CMD_NAV_VTOL_LAND);
        p75.current_SET((char)39);
        p75.autocontinue_SET((char)100);
        p75.param1_SET(4.847099E37F);
        p75.param2_SET(-4.53639E37F);
        p75.param3_SET(4.0580863E37F);
        p75.param4_SET(-2.912348E38F);
        p75.x_SET(1599774435);
        p75.y_SET(928192436);
        p75.z_SET(-1.1265476E38F);
        LoopBackDemoChannel.instance.send(p75); //===============================
        COMMAND_LONG p76 = LoopBackDemoChannel.instance.new_COMMAND_LONG();
        PH.setPack(p76);
        p76.target_system_SET((char)73);
        p76.target_component_SET((char)213);
        p76.command_SET(MAV_CMD.MAV_CMD_DO_RALLY_LAND);
        p76.confirmation_SET((char)70);
        p76.param1_SET(-3.7830707E37F);
        p76.param2_SET(3.2447428E38F);
        p76.param3_SET(-1.2412482E38F);
        p76.param4_SET(2.0972336E38F);
        p76.param5_SET(1.413851E38F);
        p76.param6_SET(-9.064453E37F);
        p76.param7_SET(8.888399E37F);
        LoopBackDemoChannel.instance.send(p76); //===============================
        COMMAND_ACK p77 = LoopBackDemoChannel.instance.new_COMMAND_ACK();
        PH.setPack(p77);
        p77.command_SET(MAV_CMD.MAV_CMD_PREFLIGHT_UAVCAN);
        p77.result_SET(MAV_RESULT.MAV_RESULT_UNSUPPORTED);
        p77.progress_SET((char)240, PH);
        p77.result_param2_SET(1330570888, PH);
        p77.target_system_SET((char)171, PH);
        p77.target_component_SET((char)248, PH);
        LoopBackDemoChannel.instance.send(p77); //===============================
        MANUAL_SETPOINT p81 = LoopBackDemoChannel.instance.new_MANUAL_SETPOINT();
        PH.setPack(p81);
        p81.time_boot_ms_SET(4193475087L);
        p81.roll_SET(-3.454693E37F);
        p81.pitch_SET(1.4207736E38F);
        p81.yaw_SET(2.171698E38F);
        p81.thrust_SET(3.1515005E38F);
        p81.mode_switch_SET((char)152);
        p81.manual_override_switch_SET((char)180);
        LoopBackDemoChannel.instance.send(p81); //===============================
        SET_ATTITUDE_TARGET p82 = LoopBackDemoChannel.instance.new_SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.time_boot_ms_SET(909050764L);
        p82.target_system_SET((char)186);
        p82.target_component_SET((char)190);
        p82.type_mask_SET((char)169);
        p82.q_SET(new float[4], 0);
        p82.body_roll_rate_SET(4.246562E37F);
        p82.body_pitch_rate_SET(-1.873924E38F);
        p82.body_yaw_rate_SET(1.4496503E38F);
        p82.thrust_SET(1.9727127E37F);
        LoopBackDemoChannel.instance.send(p82); //===============================
        ATTITUDE_TARGET p83 = LoopBackDemoChannel.instance.new_ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.time_boot_ms_SET(2615914491L);
        p83.type_mask_SET((char)54);
        p83.q_SET(new float[4], 0);
        p83.body_roll_rate_SET(2.1540075E38F);
        p83.body_pitch_rate_SET(-2.0095335E38F);
        p83.body_yaw_rate_SET(-2.3594322E38F);
        p83.thrust_SET(-3.072124E38F);
        LoopBackDemoChannel.instance.send(p83); //===============================
        SET_POSITION_TARGET_LOCAL_NED p84 = LoopBackDemoChannel.instance.new_SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.time_boot_ms_SET(2799732997L);
        p84.target_system_SET((char)228);
        p84.target_component_SET((char)12);
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
        p84.type_mask_SET((char)43220);
        p84.x_SET(-9.6119264E36F);
        p84.y_SET(9.482494E37F);
        p84.z_SET(-2.6488417E38F);
        p84.vx_SET(2.4827917E38F);
        p84.vy_SET(-3.370034E38F);
        p84.vz_SET(-1.8570578E38F);
        p84.afx_SET(-1.3170747E38F);
        p84.afy_SET(-1.7684462E38F);
        p84.afz_SET(-1.5503964E38F);
        p84.yaw_SET(-7.412986E37F);
        p84.yaw_rate_SET(5.55244E37F);
        LoopBackDemoChannel.instance.send(p84); //===============================
        SET_POSITION_TARGET_GLOBAL_INT p86 = LoopBackDemoChannel.instance.new_SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.time_boot_ms_SET(3472015246L);
        p86.target_system_SET((char)71);
        p86.target_component_SET((char)101);
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_INT);
        p86.type_mask_SET((char)35806);
        p86.lat_int_SET(1127656702);
        p86.lon_int_SET(-1221320048);
        p86.alt_SET(1.6363993E38F);
        p86.vx_SET(-1.8011729E38F);
        p86.vy_SET(-2.0778343E38F);
        p86.vz_SET(-1.731631E38F);
        p86.afx_SET(-1.4600845E37F);
        p86.afy_SET(-3.2320227E38F);
        p86.afz_SET(-2.5055053E38F);
        p86.yaw_SET(1.8567485E38F);
        p86.yaw_rate_SET(-4.260515E37F);
        LoopBackDemoChannel.instance.send(p86); //===============================
        POSITION_TARGET_GLOBAL_INT p87 = LoopBackDemoChannel.instance.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.time_boot_ms_SET(780123206L);
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_MISSION);
        p87.type_mask_SET((char)359);
        p87.lat_int_SET(813411610);
        p87.lon_int_SET(318270173);
        p87.alt_SET(1.3319105E38F);
        p87.vx_SET(-6.524959E37F);
        p87.vy_SET(1.7837584E38F);
        p87.vz_SET(3.3062502E38F);
        p87.afx_SET(-1.1321723E38F);
        p87.afy_SET(3.022586E38F);
        p87.afz_SET(-2.0479697E38F);
        p87.yaw_SET(1.2904982E38F);
        p87.yaw_rate_SET(1.9793514E38F);
        LoopBackDemoChannel.instance.send(p87); //===============================
        LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = LoopBackDemoChannel.instance.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.time_boot_ms_SET(3549042725L);
        p89.x_SET(2.542675E38F);
        p89.y_SET(2.3384503E38F);
        p89.z_SET(-1.7500178E38F);
        p89.roll_SET(2.7406089E38F);
        p89.pitch_SET(-6.9663227E37F);
        p89.yaw_SET(-2.2394533E38F);
        LoopBackDemoChannel.instance.send(p89); //===============================
        HIL_STATE p90 = LoopBackDemoChannel.instance.new_HIL_STATE();
        PH.setPack(p90);
        p90.time_usec_SET(1976140904259480915L);
        p90.roll_SET(2.8043839E38F);
        p90.pitch_SET(1.1438475E38F);
        p90.yaw_SET(-4.0245703E37F);
        p90.rollspeed_SET(1.37776E38F);
        p90.pitchspeed_SET(2.9056164E38F);
        p90.yawspeed_SET(1.299262E38F);
        p90.lat_SET(-1704650041);
        p90.lon_SET(1286537587);
        p90.alt_SET(346458856);
        p90.vx_SET((short) -9050);
        p90.vy_SET((short) -31499);
        p90.vz_SET((short) -374);
        p90.xacc_SET((short) -28765);
        p90.yacc_SET((short)26133);
        p90.zacc_SET((short) -16746);
        LoopBackDemoChannel.instance.send(p90); //===============================
        HIL_CONTROLS p91 = LoopBackDemoChannel.instance.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.time_usec_SET(4360419503015059954L);
        p91.roll_ailerons_SET(3.92573E37F);
        p91.pitch_elevator_SET(1.8641522E38F);
        p91.yaw_rudder_SET(2.019301E38F);
        p91.throttle_SET(-2.5673963E38F);
        p91.aux1_SET(2.7867112E38F);
        p91.aux2_SET(9.769081E37F);
        p91.aux3_SET(1.0639468E38F);
        p91.aux4_SET(-1.1525052E38F);
        p91.mode_SET(MAV_MODE.MAV_MODE_PREFLIGHT);
        p91.nav_mode_SET((char)101);
        LoopBackDemoChannel.instance.send(p91); //===============================
        HIL_RC_INPUTS_RAW p92 = LoopBackDemoChannel.instance.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.time_usec_SET(438520256067010510L);
        p92.chan1_raw_SET((char)27179);
        p92.chan2_raw_SET((char)38270);
        p92.chan3_raw_SET((char)54671);
        p92.chan4_raw_SET((char)23375);
        p92.chan5_raw_SET((char)32675);
        p92.chan6_raw_SET((char)46326);
        p92.chan7_raw_SET((char)63359);
        p92.chan8_raw_SET((char)9449);
        p92.chan9_raw_SET((char)37730);
        p92.chan10_raw_SET((char)53872);
        p92.chan11_raw_SET((char)25638);
        p92.chan12_raw_SET((char)45141);
        p92.rssi_SET((char)61);
        LoopBackDemoChannel.instance.send(p92); //===============================
        HIL_ACTUATOR_CONTROLS p93 = LoopBackDemoChannel.instance.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.time_usec_SET(623843136463331835L);
        p93.controls_SET(new float[16], 0);
        p93.mode_SET(MAV_MODE.MAV_MODE_MANUAL_DISARMED);
        p93.flags_SET(8256337350486690063L);
        LoopBackDemoChannel.instance.send(p93); //===============================
        OPTICAL_FLOW p100 = LoopBackDemoChannel.instance.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.time_usec_SET(7931968758024001756L);
        p100.sensor_id_SET((char)145);
        p100.flow_x_SET((short) -14130);
        p100.flow_y_SET((short) -7551);
        p100.flow_comp_m_x_SET(-3.011657E38F);
        p100.flow_comp_m_y_SET(3.4598926E37F);
        p100.quality_SET((char)157);
        p100.ground_distance_SET(2.824219E38F);
        p100.flow_rate_x_SET(5.7979327E37F, PH);
        p100.flow_rate_y_SET(1.1404126E38F, PH);
        LoopBackDemoChannel.instance.send(p100); //===============================
        GLOBAL_VISION_POSITION_ESTIMATE p101 = LoopBackDemoChannel.instance.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.usec_SET(1276433857947408339L);
        p101.x_SET(1.014553E38F);
        p101.y_SET(-2.9104943E38F);
        p101.z_SET(2.3930436E38F);
        p101.roll_SET(-1.9889446E38F);
        p101.pitch_SET(-4.1384583E37F);
        p101.yaw_SET(3.3054872E38F);
        LoopBackDemoChannel.instance.send(p101); //===============================
        VISION_POSITION_ESTIMATE p102 = LoopBackDemoChannel.instance.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.usec_SET(900013880239952253L);
        p102.x_SET(-3.103139E38F);
        p102.y_SET(6.0909247E37F);
        p102.z_SET(-2.9859406E37F);
        p102.roll_SET(1.2885852E38F);
        p102.pitch_SET(-2.4771323E38F);
        p102.yaw_SET(6.668577E36F);
        LoopBackDemoChannel.instance.send(p102); //===============================
        VISION_SPEED_ESTIMATE p103 = LoopBackDemoChannel.instance.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.usec_SET(1926695117953358697L);
        p103.x_SET(2.5372248E38F);
        p103.y_SET(3.0702775E38F);
        p103.z_SET(3.0801636E38F);
        LoopBackDemoChannel.instance.send(p103); //===============================
        VICON_POSITION_ESTIMATE p104 = LoopBackDemoChannel.instance.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.usec_SET(4055315689741400154L);
        p104.x_SET(-3.373843E38F);
        p104.y_SET(-2.5764771E38F);
        p104.z_SET(3.0880569E38F);
        p104.roll_SET(1.3338017E38F);
        p104.pitch_SET(3.8658724E37F);
        p104.yaw_SET(2.7855456E38F);
        LoopBackDemoChannel.instance.send(p104); //===============================
        HIGHRES_IMU p105 = LoopBackDemoChannel.instance.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.time_usec_SET(9201295100933342331L);
        p105.xacc_SET(-5.412647E37F);
        p105.yacc_SET(-2.3991276E37F);
        p105.zacc_SET(-1.1390964E38F);
        p105.xgyro_SET(-1.7441607E38F);
        p105.ygyro_SET(-1.2138327E38F);
        p105.zgyro_SET(-2.594349E38F);
        p105.xmag_SET(3.0873943E38F);
        p105.ymag_SET(3.1274242E38F);
        p105.zmag_SET(-2.3300184E37F);
        p105.abs_pressure_SET(-7.6469496E37F);
        p105.diff_pressure_SET(-9.358862E37F);
        p105.pressure_alt_SET(2.096332E38F);
        p105.temperature_SET(3.3691161E38F);
        p105.fields_updated_SET((char)33606);
        LoopBackDemoChannel.instance.send(p105); //===============================
        OPTICAL_FLOW_RAD p106 = LoopBackDemoChannel.instance.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.time_usec_SET(3064433296163647510L);
        p106.sensor_id_SET((char)82);
        p106.integration_time_us_SET(3546482308L);
        p106.integrated_x_SET(2.101158E38F);
        p106.integrated_y_SET(7.8056336E37F);
        p106.integrated_xgyro_SET(5.5287146E37F);
        p106.integrated_ygyro_SET(6.4185014E37F);
        p106.integrated_zgyro_SET(6.7697694E37F);
        p106.temperature_SET((short) -23878);
        p106.quality_SET((char)93);
        p106.time_delta_distance_us_SET(543608991L);
        p106.distance_SET(-2.9200563E38F);
        LoopBackDemoChannel.instance.send(p106); //===============================
        HIL_SENSOR p107 = LoopBackDemoChannel.instance.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.time_usec_SET(4758016502499026414L);
        p107.xacc_SET(1.6511538E38F);
        p107.yacc_SET(8.211331E37F);
        p107.zacc_SET(2.1080376E38F);
        p107.xgyro_SET(-9.993682E37F);
        p107.ygyro_SET(1.0835036E37F);
        p107.zgyro_SET(3.1024775E38F);
        p107.xmag_SET(2.3763956E38F);
        p107.ymag_SET(-7.114594E37F);
        p107.zmag_SET(1.014276E38F);
        p107.abs_pressure_SET(2.974249E38F);
        p107.diff_pressure_SET(2.623944E38F);
        p107.pressure_alt_SET(-2.0463173E38F);
        p107.temperature_SET(7.27576E37F);
        p107.fields_updated_SET(2448180622L);
        LoopBackDemoChannel.instance.send(p107); //===============================
        SIM_STATE p108 = LoopBackDemoChannel.instance.new_SIM_STATE();
        PH.setPack(p108);
        p108.q1_SET(2.8812414E38F);
        p108.q2_SET(-1.941738E38F);
        p108.q3_SET(-1.5956838E38F);
        p108.q4_SET(-9.219214E37F);
        p108.roll_SET(-6.5549396E37F);
        p108.pitch_SET(-6.135685E37F);
        p108.yaw_SET(1.1137842E38F);
        p108.xacc_SET(2.5612726E38F);
        p108.yacc_SET(-2.4227464E38F);
        p108.zacc_SET(-9.710816E37F);
        p108.xgyro_SET(-2.3342804E38F);
        p108.ygyro_SET(-6.0783846E37F);
        p108.zgyro_SET(2.237653E38F);
        p108.lat_SET(-2.1292478E38F);
        p108.lon_SET(-8.856841E37F);
        p108.alt_SET(1.0905186E38F);
        p108.std_dev_horz_SET(1.0350086E38F);
        p108.std_dev_vert_SET(-1.0454941E38F);
        p108.vn_SET(5.931815E37F);
        p108.ve_SET(3.0515031E38F);
        p108.vd_SET(-2.298468E38F);
        LoopBackDemoChannel.instance.send(p108); //===============================
        RADIO_STATUS p109 = LoopBackDemoChannel.instance.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.rssi_SET((char)179);
        p109.remrssi_SET((char)107);
        p109.txbuf_SET((char)1);
        p109.noise_SET((char)239);
        p109.remnoise_SET((char)12);
        p109.rxerrors_SET((char)13262);
        p109.fixed__SET((char)25620);
        LoopBackDemoChannel.instance.send(p109); //===============================
        FILE_TRANSFER_PROTOCOL p110 = LoopBackDemoChannel.instance.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.target_network_SET((char)60);
        p110.target_system_SET((char)147);
        p110.target_component_SET((char)25);
        p110.payload_SET(new char[251], 0);
        LoopBackDemoChannel.instance.send(p110); //===============================
        TIMESYNC p111 = LoopBackDemoChannel.instance.new_TIMESYNC();
        PH.setPack(p111);
        p111.tc1_SET(-8946711925485958651L);
        p111.ts1_SET(-4620222160328258982L);
        LoopBackDemoChannel.instance.send(p111); //===============================
        CAMERA_TRIGGER p112 = LoopBackDemoChannel.instance.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.time_usec_SET(2677802177969425988L);
        p112.seq_SET(3318731406L);
        LoopBackDemoChannel.instance.send(p112); //===============================
        HIL_GPS p113 = LoopBackDemoChannel.instance.new_HIL_GPS();
        PH.setPack(p113);
        p113.time_usec_SET(2569757952027318670L);
        p113.fix_type_SET((char)3);
        p113.lat_SET(-348176177);
        p113.lon_SET(-2013884090);
        p113.alt_SET(-1459847602);
        p113.eph_SET((char)51281);
        p113.epv_SET((char)57333);
        p113.vel_SET((char)45220);
        p113.vn_SET((short) -27657);
        p113.ve_SET((short)20877);
        p113.vd_SET((short) -3411);
        p113.cog_SET((char)43770);
        p113.satellites_visible_SET((char)62);
        LoopBackDemoChannel.instance.send(p113); //===============================
        HIL_OPTICAL_FLOW p114 = LoopBackDemoChannel.instance.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.time_usec_SET(2749083608729018140L);
        p114.sensor_id_SET((char)176);
        p114.integration_time_us_SET(2840230981L);
        p114.integrated_x_SET(3.0949695E38F);
        p114.integrated_y_SET(7.2957886E37F);
        p114.integrated_xgyro_SET(2.6398087E38F);
        p114.integrated_ygyro_SET(-2.5760234E38F);
        p114.integrated_zgyro_SET(1.9626762E37F);
        p114.temperature_SET((short) -11384);
        p114.quality_SET((char)111);
        p114.time_delta_distance_us_SET(2327738754L);
        p114.distance_SET(-1.7399444E38F);
        LoopBackDemoChannel.instance.send(p114); //===============================
        HIL_STATE_QUATERNION p115 = LoopBackDemoChannel.instance.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.time_usec_SET(6363873088141570273L);
        p115.attitude_quaternion_SET(new float[4], 0);
        p115.rollspeed_SET(-2.5340717E38F);
        p115.pitchspeed_SET(6.406201E37F);
        p115.yawspeed_SET(3.2018347E38F);
        p115.lat_SET(-141246294);
        p115.lon_SET(2019887991);
        p115.alt_SET(-1683858516);
        p115.vx_SET((short) -5214);
        p115.vy_SET((short) -15630);
        p115.vz_SET((short) -5007);
        p115.ind_airspeed_SET((char)65428);
        p115.true_airspeed_SET((char)30302);
        p115.xacc_SET((short)19543);
        p115.yacc_SET((short) -27681);
        p115.zacc_SET((short) -17255);
        LoopBackDemoChannel.instance.send(p115); //===============================
        SCALED_IMU2 p116 = LoopBackDemoChannel.instance.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.time_boot_ms_SET(2604848719L);
        p116.xacc_SET((short) -20059);
        p116.yacc_SET((short)24123);
        p116.zacc_SET((short)12781);
        p116.xgyro_SET((short)4918);
        p116.ygyro_SET((short)5921);
        p116.zgyro_SET((short) -25808);
        p116.xmag_SET((short)13414);
        p116.ymag_SET((short) -10279);
        p116.zmag_SET((short)20898);
        LoopBackDemoChannel.instance.send(p116); //===============================
        LOG_REQUEST_LIST p117 = LoopBackDemoChannel.instance.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.target_system_SET((char)170);
        p117.target_component_SET((char)9);
        p117.start_SET((char)12146);
        p117.end_SET((char)45715);
        LoopBackDemoChannel.instance.send(p117); //===============================
        LOG_ENTRY p118 = LoopBackDemoChannel.instance.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.id_SET((char)51470);
        p118.num_logs_SET((char)13687);
        p118.last_log_num_SET((char)2890);
        p118.time_utc_SET(3930146161L);
        p118.size_SET(1437971981L);
        LoopBackDemoChannel.instance.send(p118); //===============================
        LOG_REQUEST_DATA p119 = LoopBackDemoChannel.instance.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.target_system_SET((char)66);
        p119.target_component_SET((char)66);
        p119.id_SET((char)8543);
        p119.ofs_SET(1749186930L);
        p119.count_SET(2050167639L);
        LoopBackDemoChannel.instance.send(p119); //===============================
        LOG_DATA p120 = LoopBackDemoChannel.instance.new_LOG_DATA();
        PH.setPack(p120);
        p120.id_SET((char)16218);
        p120.ofs_SET(2525714004L);
        p120.count_SET((char)154);
        p120.data__SET(new char[90], 0);
        LoopBackDemoChannel.instance.send(p120); //===============================
        LOG_ERASE p121 = LoopBackDemoChannel.instance.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_system_SET((char)200);
        p121.target_component_SET((char)177);
        LoopBackDemoChannel.instance.send(p121); //===============================
        LOG_REQUEST_END p122 = LoopBackDemoChannel.instance.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_system_SET((char)72);
        p122.target_component_SET((char)3);
        LoopBackDemoChannel.instance.send(p122); //===============================
        GPS_INJECT_DATA p123 = LoopBackDemoChannel.instance.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.target_system_SET((char)0);
        p123.target_component_SET((char)141);
        p123.len_SET((char)45);
        p123.data__SET(new char[110], 0);
        LoopBackDemoChannel.instance.send(p123); //===============================
        GPS2_RAW p124 = LoopBackDemoChannel.instance.new_GPS2_RAW();
        PH.setPack(p124);
        p124.time_usec_SET(1682751350649138664L);
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC);
        p124.lat_SET(728849312);
        p124.lon_SET(-1323779800);
        p124.alt_SET(892899094);
        p124.eph_SET((char)31877);
        p124.epv_SET((char)42885);
        p124.vel_SET((char)26168);
        p124.cog_SET((char)54243);
        p124.satellites_visible_SET((char)243);
        p124.dgps_numch_SET((char)205);
        p124.dgps_age_SET(3680323291L);
        LoopBackDemoChannel.instance.send(p124); //===============================
        POWER_STATUS p125 = LoopBackDemoChannel.instance.new_POWER_STATUS();
        PH.setPack(p125);
        p125.Vcc_SET((char)9722);
        p125.Vservo_SET((char)58751);
        p125.flags_SET(MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID);
        LoopBackDemoChannel.instance.send(p125); //===============================
        SERIAL_CONTROL p126 = LoopBackDemoChannel.instance.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_SHELL);
        p126.flags_SET(SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY);
        p126.timeout_SET((char)10127);
        p126.baudrate_SET(2807226633L);
        p126.count_SET((char)113);
        p126.data__SET(new char[70], 0);
        LoopBackDemoChannel.instance.send(p126); //===============================
        GPS_RTK p127 = LoopBackDemoChannel.instance.new_GPS_RTK();
        PH.setPack(p127);
        p127.time_last_baseline_ms_SET(259481835L);
        p127.rtk_receiver_id_SET((char)207);
        p127.wn_SET((char)55349);
        p127.tow_SET(194068289L);
        p127.rtk_health_SET((char)228);
        p127.rtk_rate_SET((char)150);
        p127.nsats_SET((char)44);
        p127.baseline_coords_type_SET((char)109);
        p127.baseline_a_mm_SET(-972408633);
        p127.baseline_b_mm_SET(897634526);
        p127.baseline_c_mm_SET(-780576875);
        p127.accuracy_SET(1620457811L);
        p127.iar_num_hypotheses_SET(-1474733694);
        LoopBackDemoChannel.instance.send(p127); //===============================
        GPS2_RTK p128 = LoopBackDemoChannel.instance.new_GPS2_RTK();
        PH.setPack(p128);
        p128.time_last_baseline_ms_SET(2862130360L);
        p128.rtk_receiver_id_SET((char)163);
        p128.wn_SET((char)16139);
        p128.tow_SET(3443256468L);
        p128.rtk_health_SET((char)224);
        p128.rtk_rate_SET((char)35);
        p128.nsats_SET((char)133);
        p128.baseline_coords_type_SET((char)1);
        p128.baseline_a_mm_SET(-147796441);
        p128.baseline_b_mm_SET(2114303590);
        p128.baseline_c_mm_SET(2032187526);
        p128.accuracy_SET(3226392479L);
        p128.iar_num_hypotheses_SET(752919682);
        LoopBackDemoChannel.instance.send(p128); //===============================
        SCALED_IMU3 p129 = LoopBackDemoChannel.instance.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.time_boot_ms_SET(836861882L);
        p129.xacc_SET((short) -14187);
        p129.yacc_SET((short) -13300);
        p129.zacc_SET((short)758);
        p129.xgyro_SET((short)14864);
        p129.ygyro_SET((short)22023);
        p129.zgyro_SET((short) -4015);
        p129.xmag_SET((short)29494);
        p129.ymag_SET((short)23955);
        p129.zmag_SET((short) -24839);
        LoopBackDemoChannel.instance.send(p129); //===============================
        DATA_TRANSMISSION_HANDSHAKE p130 = LoopBackDemoChannel.instance.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.type_SET((char)115);
        p130.size_SET(908388164L);
        p130.width_SET((char)35920);
        p130.height_SET((char)1259);
        p130.packets_SET((char)29566);
        p130.payload_SET((char)229);
        p130.jpg_quality_SET((char)90);
        LoopBackDemoChannel.instance.send(p130); //===============================
        ENCAPSULATED_DATA p131 = LoopBackDemoChannel.instance.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.seqnr_SET((char)63652);
        p131.data__SET(new char[253], 0);
        LoopBackDemoChannel.instance.send(p131); //===============================
        DISTANCE_SENSOR p132 = LoopBackDemoChannel.instance.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.time_boot_ms_SET(648531154L);
        p132.min_distance_SET((char)27338);
        p132.max_distance_SET((char)738);
        p132.current_distance_SET((char)3320);
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
        p132.id_SET((char)56);
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_90_YAW_45);
        p132.covariance_SET((char)42);
        LoopBackDemoChannel.instance.send(p132); //===============================
        TERRAIN_REQUEST p133 = LoopBackDemoChannel.instance.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.lat_SET(-1512457365);
        p133.lon_SET(-211656462);
        p133.grid_spacing_SET((char)17291);
        p133.mask_SET(4428457211388491391L);
        LoopBackDemoChannel.instance.send(p133); //===============================
        TERRAIN_DATA p134 = LoopBackDemoChannel.instance.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.lat_SET(-824253921);
        p134.lon_SET(-759541030);
        p134.grid_spacing_SET((char)53552);
        p134.gridbit_SET((char)244);
        p134.data__SET(new short[16], 0);
        LoopBackDemoChannel.instance.send(p134); //===============================
        TERRAIN_CHECK p135 = LoopBackDemoChannel.instance.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lat_SET(606247732);
        p135.lon_SET(-635694808);
        LoopBackDemoChannel.instance.send(p135); //===============================
        TERRAIN_REPORT p136 = LoopBackDemoChannel.instance.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.lat_SET(-1521567690);
        p136.lon_SET(-773715983);
        p136.spacing_SET((char)44749);
        p136.terrain_height_SET(-2.8667642E38F);
        p136.current_height_SET(1.1839929E38F);
        p136.pending_SET((char)49562);
        p136.loaded_SET((char)51124);
        LoopBackDemoChannel.instance.send(p136); //===============================
        SCALED_PRESSURE2 p137 = LoopBackDemoChannel.instance.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.time_boot_ms_SET(3204482400L);
        p137.press_abs_SET(2.7335358E38F);
        p137.press_diff_SET(2.9219462E38F);
        p137.temperature_SET((short) -15385);
        LoopBackDemoChannel.instance.send(p137); //===============================
        ATT_POS_MOCAP p138 = LoopBackDemoChannel.instance.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.time_usec_SET(380032076451427598L);
        p138.q_SET(new float[4], 0);
        p138.x_SET(3.2890312E38F);
        p138.y_SET(-3.3728907E38F);
        p138.z_SET(2.2263338E38F);
        LoopBackDemoChannel.instance.send(p138); //===============================
        SET_ACTUATOR_CONTROL_TARGET p139 = LoopBackDemoChannel.instance.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.time_usec_SET(5013724109226988349L);
        p139.group_mlx_SET((char)128);
        p139.target_system_SET((char)106);
        p139.target_component_SET((char)129);
        p139.controls_SET(new float[8], 0);
        LoopBackDemoChannel.instance.send(p139); //===============================
        ACTUATOR_CONTROL_TARGET p140 = LoopBackDemoChannel.instance.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.time_usec_SET(989953545376261355L);
        p140.group_mlx_SET((char)200);
        p140.controls_SET(new float[8], 0);
        LoopBackDemoChannel.instance.send(p140); //===============================
        ALTITUDE p141 = LoopBackDemoChannel.instance.new_ALTITUDE();
        PH.setPack(p141);
        p141.time_usec_SET(4846902331042742783L);
        p141.altitude_monotonic_SET(7.357308E36F);
        p141.altitude_amsl_SET(-8.723355E36F);
        p141.altitude_local_SET(-7.5079345E37F);
        p141.altitude_relative_SET(-2.6759283E38F);
        p141.altitude_terrain_SET(4.2919607E37F);
        p141.bottom_clearance_SET(1.0649365E38F);
        LoopBackDemoChannel.instance.send(p141); //===============================
        RESOURCE_REQUEST p142 = LoopBackDemoChannel.instance.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.request_id_SET((char)205);
        p142.uri_type_SET((char)238);
        p142.uri_SET(new char[120], 0);
        p142.transfer_type_SET((char)85);
        p142.storage_SET(new char[120], 0);
        LoopBackDemoChannel.instance.send(p142); //===============================
        SCALED_PRESSURE3 p143 = LoopBackDemoChannel.instance.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.time_boot_ms_SET(4275849371L);
        p143.press_abs_SET(-9.95171E37F);
        p143.press_diff_SET(-2.0555232E38F);
        p143.temperature_SET((short) -4981);
        LoopBackDemoChannel.instance.send(p143); //===============================
        FOLLOW_TARGET p144 = LoopBackDemoChannel.instance.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.timestamp_SET(1577045258724297084L);
        p144.est_capabilities_SET((char)236);
        p144.lat_SET(-316625137);
        p144.lon_SET(52981430);
        p144.alt_SET(-3.9566605E37F);
        p144.vel_SET(new float[3], 0);
        p144.acc_SET(new float[3], 0);
        p144.attitude_q_SET(new float[4], 0);
        p144.rates_SET(new float[3], 0);
        p144.position_cov_SET(new float[3], 0);
        p144.custom_state_SET(4854435111078923574L);
        LoopBackDemoChannel.instance.send(p144); //===============================
        CONTROL_SYSTEM_STATE p146 = LoopBackDemoChannel.instance.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.time_usec_SET(5033639092450781391L);
        p146.x_acc_SET(2.358616E38F);
        p146.y_acc_SET(1.5578036E38F);
        p146.z_acc_SET(-1.6004409E36F);
        p146.x_vel_SET(-2.3954428E38F);
        p146.y_vel_SET(2.032834E37F);
        p146.z_vel_SET(3.2935033E38F);
        p146.x_pos_SET(-3.3664892E38F);
        p146.y_pos_SET(1.775059E38F);
        p146.z_pos_SET(2.0626257E38F);
        p146.airspeed_SET(1.0460951E38F);
        p146.vel_variance_SET(new float[3], 0);
        p146.pos_variance_SET(new float[3], 0);
        p146.q_SET(new float[4], 0);
        p146.roll_rate_SET(2.310098E38F);
        p146.pitch_rate_SET(4.1909936E37F);
        p146.yaw_rate_SET(-1.0639559E38F);
        LoopBackDemoChannel.instance.send(p146); //===============================
        BATTERY_STATUS p147 = LoopBackDemoChannel.instance.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.id_SET((char)0);
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD);
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIFE);
        p147.temperature_SET((short)6891);
        p147.voltages_SET(new char[10], 0);
        p147.current_battery_SET((short)32615);
        p147.current_consumed_SET(-1323006136);
        p147.energy_consumed_SET(1603131532);
        p147.battery_remaining_SET((byte) - 107);
        LoopBackDemoChannel.instance.send(p147); //===============================
        AUTOPILOT_VERSION p148 = LoopBackDemoChannel.instance.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.capabilities_SET(MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMMAND_INT);
        p148.flight_sw_version_SET(1384885849L);
        p148.middleware_sw_version_SET(3619028795L);
        p148.os_sw_version_SET(914944890L);
        p148.board_version_SET(2733465139L);
        p148.flight_custom_version_SET(new char[8], 0);
        p148.middleware_custom_version_SET(new char[8], 0);
        p148.os_custom_version_SET(new char[8], 0);
        p148.vendor_id_SET((char)9039);
        p148.product_id_SET((char)56165);
        p148.uid_SET(5599120819600202108L);
        p148.uid2_SET(new char[18], 0, PH);
        LoopBackDemoChannel.instance.send(p148); //===============================
        LANDING_TARGET p149 = LoopBackDemoChannel.instance.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.time_usec_SET(8419489947990280173L);
        p149.target_num_SET((char)58);
        p149.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_NED);
        p149.angle_x_SET(-2.026441E38F);
        p149.angle_y_SET(2.3657005E37F);
        p149.distance_SET(-1.321421E38F);
        p149.size_x_SET(9.022679E37F);
        p149.size_y_SET(1.585113E38F);
        p149.x_SET(4.6140307E37F, PH);
        p149.y_SET(2.5828604E38F, PH);
        p149.z_SET(-2.6233717E38F, PH);
        p149.q_SET(new float[4], 0, PH);
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_RADIO_BEACON);
        p149.position_valid_SET((char)19, PH);
        LoopBackDemoChannel.instance.send(p149); //===============================
        SENSOR_OFFSETS p150 = LoopBackDemoChannel.instance.new_SENSOR_OFFSETS();
        PH.setPack(p150);
        p150.mag_ofs_x_SET((short) -30968);
        p150.mag_ofs_y_SET((short) -16747);
        p150.mag_ofs_z_SET((short)23699);
        p150.mag_declination_SET(-9.158523E37F);
        p150.raw_press_SET(-1027306808);
        p150.raw_temp_SET(-774552003);
        p150.gyro_cal_x_SET(-2.0293865E38F);
        p150.gyro_cal_y_SET(-1.3330851E38F);
        p150.gyro_cal_z_SET(4.640769E37F);
        p150.accel_cal_x_SET(-2.7259952E38F);
        p150.accel_cal_y_SET(6.4185536E37F);
        p150.accel_cal_z_SET(1.1406399E37F);
        LoopBackDemoChannel.instance.send(p150); //===============================
        SET_MAG_OFFSETS p151 = LoopBackDemoChannel.instance.new_SET_MAG_OFFSETS();
        PH.setPack(p151);
        p151.target_system_SET((char)106);
        p151.target_component_SET((char)78);
        p151.mag_ofs_x_SET((short)5938);
        p151.mag_ofs_y_SET((short) -4691);
        p151.mag_ofs_z_SET((short)22764);
        LoopBackDemoChannel.instance.send(p151); //===============================
        MEMINFO p152 = LoopBackDemoChannel.instance.new_MEMINFO();
        PH.setPack(p152);
        p152.brkval_SET((char)2138);
        p152.freemem_SET((char)34930);
        p152.freemem32_SET(4129327261L, PH);
        LoopBackDemoChannel.instance.send(p152); //===============================
        AP_ADC p153 = LoopBackDemoChannel.instance.new_AP_ADC();
        PH.setPack(p153);
        p153.adc1_SET((char)14177);
        p153.adc2_SET((char)47271);
        p153.adc3_SET((char)49837);
        p153.adc4_SET((char)43010);
        p153.adc5_SET((char)6327);
        p153.adc6_SET((char)47253);
        LoopBackDemoChannel.instance.send(p153); //===============================
        DIGICAM_CONFIGURE p154 = LoopBackDemoChannel.instance.new_DIGICAM_CONFIGURE();
        PH.setPack(p154);
        p154.target_system_SET((char)122);
        p154.target_component_SET((char)39);
        p154.mode_SET((char)89);
        p154.shutter_speed_SET((char)54626);
        p154.aperture_SET((char)211);
        p154.iso_SET((char)59);
        p154.exposure_type_SET((char)155);
        p154.command_id_SET((char)241);
        p154.engine_cut_off_SET((char)4);
        p154.extra_param_SET((char)90);
        p154.extra_value_SET(1.5576685E38F);
        LoopBackDemoChannel.instance.send(p154); //===============================
        DIGICAM_CONTROL p155 = LoopBackDemoChannel.instance.new_DIGICAM_CONTROL();
        PH.setPack(p155);
        p155.target_system_SET((char)107);
        p155.target_component_SET((char)215);
        p155.session_SET((char)189);
        p155.zoom_pos_SET((char)237);
        p155.zoom_step_SET((byte)8);
        p155.focus_lock_SET((char)98);
        p155.shot_SET((char)130);
        p155.command_id_SET((char)240);
        p155.extra_param_SET((char)157);
        p155.extra_value_SET(-1.5346153E38F);
        LoopBackDemoChannel.instance.send(p155); //===============================
        MOUNT_CONFIGURE p156 = LoopBackDemoChannel.instance.new_MOUNT_CONFIGURE();
        PH.setPack(p156);
        p156.target_system_SET((char)199);
        p156.target_component_SET((char)1);
        p156.mount_mode_SET(MAV_MOUNT_MODE.MAV_MOUNT_MODE_RETRACT);
        p156.stab_roll_SET((char)75);
        p156.stab_pitch_SET((char)131);
        p156.stab_yaw_SET((char)142);
        LoopBackDemoChannel.instance.send(p156); //===============================
        MOUNT_CONTROL p157 = LoopBackDemoChannel.instance.new_MOUNT_CONTROL();
        PH.setPack(p157);
        p157.target_system_SET((char)206);
        p157.target_component_SET((char)155);
        p157.input_a_SET(-152837798);
        p157.input_b_SET(737856361);
        p157.input_c_SET(-511801076);
        p157.save_position_SET((char)19);
        LoopBackDemoChannel.instance.send(p157); //===============================
        MOUNT_STATUS p158 = LoopBackDemoChannel.instance.new_MOUNT_STATUS();
        PH.setPack(p158);
        p158.target_system_SET((char)176);
        p158.target_component_SET((char)59);
        p158.pointing_a_SET(1492011586);
        p158.pointing_b_SET(1101027355);
        p158.pointing_c_SET(-1731733736);
        LoopBackDemoChannel.instance.send(p158); //===============================
        FENCE_POINT p160 = LoopBackDemoChannel.instance.new_FENCE_POINT();
        PH.setPack(p160);
        p160.target_system_SET((char)242);
        p160.target_component_SET((char)112);
        p160.idx_SET((char)8);
        p160.count_SET((char)176);
        p160.lat_SET(8.129695E36F);
        p160.lng_SET(-7.4806846E37F);
        LoopBackDemoChannel.instance.send(p160); //===============================
        FENCE_FETCH_POINT p161 = LoopBackDemoChannel.instance.new_FENCE_FETCH_POINT();
        PH.setPack(p161);
        p161.target_system_SET((char)220);
        p161.target_component_SET((char)113);
        p161.idx_SET((char)30);
        LoopBackDemoChannel.instance.send(p161); //===============================
        FENCE_STATUS p162 = LoopBackDemoChannel.instance.new_FENCE_STATUS();
        PH.setPack(p162);
        p162.breach_status_SET((char)229);
        p162.breach_count_SET((char)10341);
        p162.breach_type_SET(FENCE_BREACH.FENCE_BREACH_NONE);
        p162.breach_time_SET(3836153072L);
        LoopBackDemoChannel.instance.send(p162); //===============================
        AHRS p163 = LoopBackDemoChannel.instance.new_AHRS();
        PH.setPack(p163);
        p163.omegaIx_SET(1.0620153E38F);
        p163.omegaIy_SET(2.7933433E38F);
        p163.omegaIz_SET(-2.8279644E38F);
        p163.accel_weight_SET(-4.0621747E37F);
        p163.renorm_val_SET(3.1518341E38F);
        p163.error_rp_SET(1.5476315E37F);
        p163.error_yaw_SET(-1.6243682E38F);
        LoopBackDemoChannel.instance.send(p163); //===============================
        SIMSTATE p164 = LoopBackDemoChannel.instance.new_SIMSTATE();
        PH.setPack(p164);
        p164.roll_SET(-1.7592341E38F);
        p164.pitch_SET(-1.8359288E38F);
        p164.yaw_SET(9.416584E37F);
        p164.xacc_SET(-9.206626E37F);
        p164.yacc_SET(-2.7504075E38F);
        p164.zacc_SET(-1.8788557E38F);
        p164.xgyro_SET(-1.427477E38F);
        p164.ygyro_SET(1.0778857E38F);
        p164.zgyro_SET(3.3454005E38F);
        p164.lat_SET(-334854369);
        p164.lng_SET(1601242336);
        LoopBackDemoChannel.instance.send(p164); //===============================
        HWSTATUS p165 = LoopBackDemoChannel.instance.new_HWSTATUS();
        PH.setPack(p165);
        p165.Vcc_SET((char)58073);
        p165.I2Cerr_SET((char)196);
        LoopBackDemoChannel.instance.send(p165); //===============================
        RADIO p166 = LoopBackDemoChannel.instance.new_RADIO();
        PH.setPack(p166);
        p166.rssi_SET((char)146);
        p166.remrssi_SET((char)226);
        p166.txbuf_SET((char)138);
        p166.noise_SET((char)64);
        p166.remnoise_SET((char)85);
        p166.rxerrors_SET((char)46129);
        p166.fixed__SET((char)59503);
        LoopBackDemoChannel.instance.send(p166); //===============================
        LIMITS_STATUS p167 = LoopBackDemoChannel.instance.new_LIMITS_STATUS();
        PH.setPack(p167);
        p167.limits_state_SET(LIMITS_STATE.LIMITS_TRIGGERED);
        p167.last_trigger_SET(4204659842L);
        p167.last_action_SET(391530090L);
        p167.last_recovery_SET(2010467277L);
        p167.last_clear_SET(2270411565L);
        p167.breach_count_SET((char)27904);
        p167.mods_enabled_SET(LIMIT_MODULE.LIMIT_GPSLOCK);
        p167.mods_required_SET(LIMIT_MODULE.LIMIT_ALTITUDE);
        p167.mods_triggered_SET(LIMIT_MODULE.LIMIT_ALTITUDE);
        LoopBackDemoChannel.instance.send(p167); //===============================
        WIND p168 = LoopBackDemoChannel.instance.new_WIND();
        PH.setPack(p168);
        p168.direction_SET(-2.5115468E38F);
        p168.speed_SET(2.1956666E38F);
        p168.speed_z_SET(9.05539E37F);
        LoopBackDemoChannel.instance.send(p168); //===============================
        DATA16 p169 = LoopBackDemoChannel.instance.new_DATA16();
        PH.setPack(p169);
        p169.type_SET((char)59);
        p169.len_SET((char)109);
        p169.data__SET(new char[16], 0);
        LoopBackDemoChannel.instance.send(p169); //===============================
        DATA32 p170 = LoopBackDemoChannel.instance.new_DATA32();
        PH.setPack(p170);
        p170.type_SET((char)78);
        p170.len_SET((char)188);
        p170.data__SET(new char[32], 0);
        LoopBackDemoChannel.instance.send(p170); //===============================
        DATA64 p171 = LoopBackDemoChannel.instance.new_DATA64();
        PH.setPack(p171);
        p171.type_SET((char)246);
        p171.len_SET((char)241);
        p171.data__SET(new char[64], 0);
        LoopBackDemoChannel.instance.send(p171); //===============================
        DATA96 p172 = LoopBackDemoChannel.instance.new_DATA96();
        PH.setPack(p172);
        p172.type_SET((char)99);
        p172.len_SET((char)16);
        p172.data__SET(new char[96], 0);
        LoopBackDemoChannel.instance.send(p172); //===============================
        RANGEFINDER p173 = LoopBackDemoChannel.instance.new_RANGEFINDER();
        PH.setPack(p173);
        p173.distance_SET(-2.1082E38F);
        p173.voltage_SET(-2.1223085E38F);
        LoopBackDemoChannel.instance.send(p173); //===============================
        AIRSPEED_AUTOCAL p174 = LoopBackDemoChannel.instance.new_AIRSPEED_AUTOCAL();
        PH.setPack(p174);
        p174.vx_SET(-3.2100755E38F);
        p174.vy_SET(3.0958354E38F);
        p174.vz_SET(1.3521087E38F);
        p174.diff_pressure_SET(-1.1499572E38F);
        p174.EAS2TAS_SET(-2.8869724E38F);
        p174.ratio_SET(3.391687E38F);
        p174.state_x_SET(2.5612572E38F);
        p174.state_y_SET(5.9014054E37F);
        p174.state_z_SET(1.4260324E38F);
        p174.Pax_SET(2.6986659E38F);
        p174.Pby_SET(3.0545735E38F);
        p174.Pcz_SET(2.8698504E38F);
        LoopBackDemoChannel.instance.send(p174); //===============================
        RALLY_POINT p175 = LoopBackDemoChannel.instance.new_RALLY_POINT();
        PH.setPack(p175);
        p175.target_system_SET((char)235);
        p175.target_component_SET((char)82);
        p175.idx_SET((char)74);
        p175.count_SET((char)122);
        p175.lat_SET(-795450972);
        p175.lng_SET(-552637790);
        p175.alt_SET((short)27727);
        p175.break_alt_SET((short)22309);
        p175.land_dir_SET((char)56194);
        p175.flags_SET(RALLY_FLAGS.LAND_IMMEDIATELY);
        LoopBackDemoChannel.instance.send(p175); //===============================
        RALLY_FETCH_POINT p176 = LoopBackDemoChannel.instance.new_RALLY_FETCH_POINT();
        PH.setPack(p176);
        p176.target_system_SET((char)26);
        p176.target_component_SET((char)173);
        p176.idx_SET((char)48);
        LoopBackDemoChannel.instance.send(p176); //===============================
        COMPASSMOT_STATUS p177 = LoopBackDemoChannel.instance.new_COMPASSMOT_STATUS();
        PH.setPack(p177);
        p177.throttle_SET((char)37217);
        p177.current_SET(1.5249592E38F);
        p177.interference_SET((char)22013);
        p177.CompensationX_SET(1.9202856E38F);
        p177.CompensationY_SET(3.1494647E38F);
        p177.CompensationZ_SET(-2.8507906E38F);
        LoopBackDemoChannel.instance.send(p177); //===============================
        AHRS2 p178 = LoopBackDemoChannel.instance.new_AHRS2();
        PH.setPack(p178);
        p178.roll_SET(1.7095712E38F);
        p178.pitch_SET(2.8522502E38F);
        p178.yaw_SET(-5.8122617E37F);
        p178.altitude_SET(1.6351484E38F);
        p178.lat_SET(-993690067);
        p178.lng_SET(1534415341);
        LoopBackDemoChannel.instance.send(p178); //===============================
        CAMERA_STATUS p179 = LoopBackDemoChannel.instance.new_CAMERA_STATUS();
        PH.setPack(p179);
        p179.time_usec_SET(8794900865822737515L);
        p179.target_system_SET((char)19);
        p179.cam_idx_SET((char)49);
        p179.img_idx_SET((char)35883);
        p179.event_id_SET(CAMERA_STATUS_TYPES.CAMERA_STATUS_TYPE_LOWSTOREV);
        p179.p1_SET(1.0309893E38F);
        p179.p2_SET(-1.0793909E38F);
        p179.p3_SET(1.2121946E38F);
        p179.p4_SET(2.1603508E38F);
        LoopBackDemoChannel.instance.send(p179); //===============================
        CAMERA_FEEDBACK p180 = LoopBackDemoChannel.instance.new_CAMERA_FEEDBACK();
        PH.setPack(p180);
        p180.time_usec_SET(5813058649675451048L);
        p180.target_system_SET((char)41);
        p180.cam_idx_SET((char)138);
        p180.img_idx_SET((char)29175);
        p180.lat_SET(703911633);
        p180.lng_SET(273278497);
        p180.alt_msl_SET(1.0153092E38F);
        p180.alt_rel_SET(-1.6615672E38F);
        p180.roll_SET(-2.5341611E38F);
        p180.pitch_SET(2.5038397E38F);
        p180.yaw_SET(-1.7903023E38F);
        p180.foc_len_SET(1.3111462E38F);
        p180.flags_SET(CAMERA_FEEDBACK_FLAGS.CAMERA_FEEDBACK_PHOTO);
        LoopBackDemoChannel.instance.send(p180); //===============================
        BATTERY2 p181 = LoopBackDemoChannel.instance.new_BATTERY2();
        PH.setPack(p181);
        p181.voltage_SET((char)21532);
        p181.current_battery_SET((short)27980);
        LoopBackDemoChannel.instance.send(p181); //===============================
        AHRS3 p182 = LoopBackDemoChannel.instance.new_AHRS3();
        PH.setPack(p182);
        p182.roll_SET(3.3097544E38F);
        p182.pitch_SET(-2.3489073E38F);
        p182.yaw_SET(-9.851971E37F);
        p182.altitude_SET(-4.9175992E36F);
        p182.lat_SET(-1650562122);
        p182.lng_SET(-1539029767);
        p182.v1_SET(-2.8697172E38F);
        p182.v2_SET(3.0250072E38F);
        p182.v3_SET(1.1035104E38F);
        p182.v4_SET(-4.707686E37F);
        LoopBackDemoChannel.instance.send(p182); //===============================
        AUTOPILOT_VERSION_REQUEST p183 = LoopBackDemoChannel.instance.new_AUTOPILOT_VERSION_REQUEST();
        PH.setPack(p183);
        p183.target_system_SET((char)69);
        p183.target_component_SET((char)53);
        LoopBackDemoChannel.instance.send(p183); //===============================
        REMOTE_LOG_DATA_BLOCK p184 = LoopBackDemoChannel.instance.new_REMOTE_LOG_DATA_BLOCK();
        PH.setPack(p184);
        p184.target_system_SET((char)167);
        p184.target_component_SET((char)176);
        p184.seqno_SET(MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS.MAV_REMOTE_LOG_DATA_BLOCK_START);
        p184.data__SET(new char[200], 0);
        LoopBackDemoChannel.instance.send(p184); //===============================
        REMOTE_LOG_BLOCK_STATUS p185 = LoopBackDemoChannel.instance.new_REMOTE_LOG_BLOCK_STATUS();
        PH.setPack(p185);
        p185.target_system_SET((char)122);
        p185.target_component_SET((char)183);
        p185.seqno_SET(1628799310L);
        p185.status_SET(MAV_REMOTE_LOG_DATA_BLOCK_STATUSES.MAV_REMOTE_LOG_DATA_BLOCK_ACK);
        LoopBackDemoChannel.instance.send(p185); //===============================
        LED_CONTROL p186 = LoopBackDemoChannel.instance.new_LED_CONTROL();
        PH.setPack(p186);
        p186.target_system_SET((char)205);
        p186.target_component_SET((char)203);
        p186.instance_SET((char)89);
        p186.pattern_SET((char)104);
        p186.custom_len_SET((char)122);
        p186.custom_bytes_SET(new char[24], 0);
        LoopBackDemoChannel.instance.send(p186); //===============================
        MAG_CAL_PROGRESS p191 = LoopBackDemoChannel.instance.new_MAG_CAL_PROGRESS();
        PH.setPack(p191);
        p191.compass_id_SET((char)97);
        p191.cal_mask_SET((char)51);
        p191.cal_status_SET(MAG_CAL_STATUS.MAG_CAL_RUNNING_STEP_ONE);
        p191.attempt_SET((char)203);
        p191.completion_pct_SET((char)228);
        p191.completion_mask_SET(new char[10], 0);
        p191.direction_x_SET(2.450538E38F);
        p191.direction_y_SET(1.7352413E38F);
        p191.direction_z_SET(-1.2409827E38F);
        LoopBackDemoChannel.instance.send(p191); //===============================
        MAG_CAL_REPORT p192 = LoopBackDemoChannel.instance.new_MAG_CAL_REPORT();
        PH.setPack(p192);
        p192.compass_id_SET((char)97);
        p192.cal_mask_SET((char)32);
        p192.cal_status_SET(MAG_CAL_STATUS.MAG_CAL_SUCCESS);
        p192.autosaved_SET((char)235);
        p192.fitness_SET(-2.8207565E36F);
        p192.ofs_x_SET(3.3885643E38F);
        p192.ofs_y_SET(1.7442223E38F);
        p192.ofs_z_SET(-1.4555319E38F);
        p192.diag_x_SET(3.3548168E38F);
        p192.diag_y_SET(-2.6566937E37F);
        p192.diag_z_SET(3.3770904E38F);
        p192.offdiag_x_SET(-5.474547E37F);
        p192.offdiag_y_SET(1.7762639E38F);
        p192.offdiag_z_SET(1.9966024E38F);
        LoopBackDemoChannel.instance.send(p192); //===============================
        EKF_STATUS_REPORT p193 = LoopBackDemoChannel.instance.new_EKF_STATUS_REPORT();
        PH.setPack(p193);
        p193.flags_SET(EKF_STATUS_FLAGS.EKF_PRED_POS_HORIZ_ABS);
        p193.velocity_variance_SET(2.8707236E38F);
        p193.pos_horiz_variance_SET(-3.3261034E38F);
        p193.pos_vert_variance_SET(-2.8393457E38F);
        p193.compass_variance_SET(1.26602E38F);
        p193.terrain_alt_variance_SET(2.554404E38F);
        LoopBackDemoChannel.instance.send(p193); //===============================
        PID_TUNING p194 = LoopBackDemoChannel.instance.new_PID_TUNING();
        PH.setPack(p194);
        p194.axis_SET(PID_TUNING_AXIS.PID_TUNING_ROLL);
        p194.desired_SET(-2.7599784E38F);
        p194.achieved_SET(6.704618E37F);
        p194.FF_SET(-7.0550263E37F);
        p194.P_SET(1.0813902E38F);
        p194.I_SET(-2.109767E38F);
        p194.D_SET(2.7366556E38F);
        LoopBackDemoChannel.instance.send(p194); //===============================
        GIMBAL_REPORT p200 = LoopBackDemoChannel.instance.new_GIMBAL_REPORT();
        PH.setPack(p200);
        p200.target_system_SET((char)161);
        p200.target_component_SET((char)199);
        p200.delta_time_SET(1.9463177E38F);
        p200.delta_angle_x_SET(-1.5545595E38F);
        p200.delta_angle_y_SET(3.040276E38F);
        p200.delta_angle_z_SET(1.6479916E38F);
        p200.delta_velocity_x_SET(1.6592642E38F);
        p200.delta_velocity_y_SET(-1.8388033E38F);
        p200.delta_velocity_z_SET(8.1961846E37F);
        p200.joint_roll_SET(1.3218182E38F);
        p200.joint_el_SET(2.4822128E38F);
        p200.joint_az_SET(1.7867888E38F);
        LoopBackDemoChannel.instance.send(p200); //===============================
        GIMBAL_CONTROL p201 = LoopBackDemoChannel.instance.new_GIMBAL_CONTROL();
        PH.setPack(p201);
        p201.target_system_SET((char)207);
        p201.target_component_SET((char)23);
        p201.demanded_rate_x_SET(1.480926E38F);
        p201.demanded_rate_y_SET(1.9760825E38F);
        p201.demanded_rate_z_SET(1.6862001E36F);
        LoopBackDemoChannel.instance.send(p201); //===============================
        GIMBAL_TORQUE_CMD_REPORT p214 = LoopBackDemoChannel.instance.new_GIMBAL_TORQUE_CMD_REPORT();
        PH.setPack(p214);
        p214.target_system_SET((char)12);
        p214.target_component_SET((char)234);
        p214.rl_torque_cmd_SET((short)12503);
        p214.el_torque_cmd_SET((short)28580);
        p214.az_torque_cmd_SET((short) -2656);
        LoopBackDemoChannel.instance.send(p214); //===============================
        GOPRO_HEARTBEAT p215 = LoopBackDemoChannel.instance.new_GOPRO_HEARTBEAT();
        PH.setPack(p215);
        p215.status_SET(GOPRO_HEARTBEAT_STATUS.GOPRO_HEARTBEAT_STATUS_DISCONNECTED);
        p215.capture_mode_SET(GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_BURST);
        p215.flags_SET(GOPRO_HEARTBEAT_FLAGS.GOPRO_FLAG_RECORDING);
        LoopBackDemoChannel.instance.send(p215); //===============================
        GOPRO_GET_REQUEST p216 = LoopBackDemoChannel.instance.new_GOPRO_GET_REQUEST();
        PH.setPack(p216);
        p216.target_system_SET((char)20);
        p216.target_component_SET((char)194);
        p216.cmd_id_SET(GOPRO_COMMAND.GOPRO_COMMAND_PROTUNE_GAIN);
        LoopBackDemoChannel.instance.send(p216); //===============================
        GOPRO_GET_RESPONSE p217 = LoopBackDemoChannel.instance.new_GOPRO_GET_RESPONSE();
        PH.setPack(p217);
        p217.cmd_id_SET(GOPRO_COMMAND.GOPRO_COMMAND_PHOTO_RESOLUTION);
        p217.status_SET(GOPRO_REQUEST_STATUS.GOPRO_REQUEST_SUCCESS);
        p217.value_SET(new char[4], 0);
        LoopBackDemoChannel.instance.send(p217); //===============================
        GOPRO_SET_REQUEST p218 = LoopBackDemoChannel.instance.new_GOPRO_SET_REQUEST();
        PH.setPack(p218);
        p218.target_system_SET((char)97);
        p218.target_component_SET((char)95);
        p218.cmd_id_SET(GOPRO_COMMAND.GOPRO_COMMAND_PHOTO_BURST_RATE);
        p218.value_SET(new char[4], 0);
        LoopBackDemoChannel.instance.send(p218); //===============================
        GOPRO_SET_RESPONSE p219 = LoopBackDemoChannel.instance.new_GOPRO_SET_RESPONSE();
        PH.setPack(p219);
        p219.cmd_id_SET(GOPRO_COMMAND.GOPRO_COMMAND_SHUTTER);
        p219.status_SET(GOPRO_REQUEST_STATUS.GOPRO_REQUEST_SUCCESS);
        LoopBackDemoChannel.instance.send(p219); //===============================
        RPM p226 = LoopBackDemoChannel.instance.new_RPM();
        PH.setPack(p226);
        p226.rpm1_SET(1.6476309E38F);
        p226.rpm2_SET(2.1173365E38F);
        LoopBackDemoChannel.instance.send(p226); //===============================
        ESTIMATOR_STATUS p230 = LoopBackDemoChannel.instance.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.time_usec_SET(2742497332719586178L);
        p230.flags_SET(ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL);
        p230.vel_ratio_SET(-2.7313773E38F);
        p230.pos_horiz_ratio_SET(-6.0777386E37F);
        p230.pos_vert_ratio_SET(3.3549365E38F);
        p230.mag_ratio_SET(-2.2206502E38F);
        p230.hagl_ratio_SET(-2.6109408E38F);
        p230.tas_ratio_SET(-1.7480072E38F);
        p230.pos_horiz_accuracy_SET(-1.6732387E38F);
        p230.pos_vert_accuracy_SET(1.3410761E38F);
        LoopBackDemoChannel.instance.send(p230); //===============================
        WIND_COV p231 = LoopBackDemoChannel.instance.new_WIND_COV();
        PH.setPack(p231);
        p231.time_usec_SET(246094309798562544L);
        p231.wind_x_SET(2.0836466E38F);
        p231.wind_y_SET(-1.3968024E38F);
        p231.wind_z_SET(1.7537346E38F);
        p231.var_horiz_SET(2.764805E38F);
        p231.var_vert_SET(-1.3019873E38F);
        p231.wind_alt_SET(-3.5733968E37F);
        p231.horiz_accuracy_SET(-5.3741844E37F);
        p231.vert_accuracy_SET(-2.4740996E38F);
        LoopBackDemoChannel.instance.send(p231); //===============================
        GPS_INPUT p232 = LoopBackDemoChannel.instance.new_GPS_INPUT();
        PH.setPack(p232);
        p232.time_usec_SET(7372467955730687576L);
        p232.gps_id_SET((char)71);
        p232.ignore_flags_SET(GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY);
        p232.time_week_ms_SET(1294109101L);
        p232.time_week_SET((char)26255);
        p232.fix_type_SET((char)142);
        p232.lat_SET(-810939299);
        p232.lon_SET(-114245835);
        p232.alt_SET(-1.1113992E38F);
        p232.hdop_SET(-9.21298E37F);
        p232.vdop_SET(-1.591003E38F);
        p232.vn_SET(3.2401514E38F);
        p232.ve_SET(-2.6153033E38F);
        p232.vd_SET(5.259069E37F);
        p232.speed_accuracy_SET(-1.4577936E38F);
        p232.horiz_accuracy_SET(6.975799E37F);
        p232.vert_accuracy_SET(-3.1390142E37F);
        p232.satellites_visible_SET((char)120);
        LoopBackDemoChannel.instance.send(p232); //===============================
        GPS_RTCM_DATA p233 = LoopBackDemoChannel.instance.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.flags_SET((char)159);
        p233.len_SET((char)87);
        p233.data__SET(new char[180], 0);
        LoopBackDemoChannel.instance.send(p233); //===============================
        HIGH_LATENCY p234 = LoopBackDemoChannel.instance.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED);
        p234.custom_mode_SET(1085611306L);
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR);
        p234.roll_SET((short)19085);
        p234.pitch_SET((short) -14536);
        p234.heading_SET((char)4451);
        p234.throttle_SET((byte)51);
        p234.heading_sp_SET((short) -10796);
        p234.latitude_SET(-1664844897);
        p234.longitude_SET(324120986);
        p234.altitude_amsl_SET((short) -17324);
        p234.altitude_sp_SET((short)31837);
        p234.airspeed_SET((char)244);
        p234.airspeed_sp_SET((char)16);
        p234.groundspeed_SET((char)145);
        p234.climb_rate_SET((byte)98);
        p234.gps_nsat_SET((char)2);
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS);
        p234.battery_remaining_SET((char)24);
        p234.temperature_SET((byte)104);
        p234.temperature_air_SET((byte) - 55);
        p234.failsafe_SET((char)153);
        p234.wp_num_SET((char)231);
        p234.wp_distance_SET((char)2204);
        LoopBackDemoChannel.instance.send(p234); //===============================
        VIBRATION p241 = LoopBackDemoChannel.instance.new_VIBRATION();
        PH.setPack(p241);
        p241.time_usec_SET(4420400637382446242L);
        p241.vibration_x_SET(-1.8723315E38F);
        p241.vibration_y_SET(1.6916979E38F);
        p241.vibration_z_SET(-1.8987763E38F);
        p241.clipping_0_SET(1524081199L);
        p241.clipping_1_SET(982714754L);
        p241.clipping_2_SET(2226516837L);
        LoopBackDemoChannel.instance.send(p241); //===============================
        HOME_POSITION p242 = LoopBackDemoChannel.instance.new_HOME_POSITION();
        PH.setPack(p242);
        p242.latitude_SET(-539121015);
        p242.longitude_SET(-1459435437);
        p242.altitude_SET(1325349438);
        p242.x_SET(-2.812411E38F);
        p242.y_SET(-8.91871E37F);
        p242.z_SET(1.7161744E38F);
        p242.q_SET(new float[4], 0);
        p242.approach_x_SET(-2.0136038E38F);
        p242.approach_y_SET(-1.1493E38F);
        p242.approach_z_SET(1.746492E38F);
        p242.time_usec_SET(7875560993653061819L, PH);
        LoopBackDemoChannel.instance.send(p242); //===============================
        SET_HOME_POSITION p243 = LoopBackDemoChannel.instance.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.target_system_SET((char)220);
        p243.latitude_SET(2102155691);
        p243.longitude_SET(-290194876);
        p243.altitude_SET(164841846);
        p243.x_SET(-1.8182547E38F);
        p243.y_SET(-1.9308734E38F);
        p243.z_SET(3.2891038E38F);
        p243.q_SET(new float[4], 0);
        p243.approach_x_SET(7.705672E37F);
        p243.approach_y_SET(-1.5040226E38F);
        p243.approach_z_SET(-2.199684E38F);
        p243.time_usec_SET(4043231298303863175L, PH);
        LoopBackDemoChannel.instance.send(p243); //===============================
        MESSAGE_INTERVAL p244 = LoopBackDemoChannel.instance.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.message_id_SET((char)30172);
        p244.interval_us_SET(1319400467);
        LoopBackDemoChannel.instance.send(p244); //===============================
        EXTENDED_SYS_STATE p245 = LoopBackDemoChannel.instance.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_MC);
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND);
        LoopBackDemoChannel.instance.send(p245); //===============================
        ADSB_VEHICLE p246 = LoopBackDemoChannel.instance.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.ICAO_address_SET(3383816551L);
        p246.lat_SET(-1860007947);
        p246.lon_SET(603469586);
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
        p246.altitude_SET(-720296751);
        p246.heading_SET((char)60887);
        p246.hor_velocity_SET((char)11618);
        p246.ver_velocity_SET((short) -10253);
        p246.callsign_SET("DEMO", PH);
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_SERVICE_SURFACE);
        p246.tslc_SET((char)96);
        p246.flags_SET(ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE);
        p246.squawk_SET((char)1313);
        LoopBackDemoChannel.instance.send(p246); //===============================
        COLLISION p247 = LoopBackDemoChannel.instance.new_COLLISION();
        PH.setPack(p247);
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
        p247.id_SET(1990082390L);
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_HORIZONTALLY);
        p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW);
        p247.time_to_minimum_delta_SET(-2.3658964E37F);
        p247.altitude_minimum_delta_SET(-3.2282422E38F);
        p247.horizontal_minimum_delta_SET(-2.557875E38F);
        LoopBackDemoChannel.instance.send(p247); //===============================
        V2_EXTENSION p248 = LoopBackDemoChannel.instance.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.target_network_SET((char)156);
        p248.target_system_SET((char)11);
        p248.target_component_SET((char)178);
        p248.message_type_SET((char)51430);
        p248.payload_SET(new char[249], 0);
        LoopBackDemoChannel.instance.send(p248); //===============================
        MEMORY_VECT p249 = LoopBackDemoChannel.instance.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.address_SET((char)16649);
        p249.ver_SET((char)32);
        p249.type_SET((char)42);
        p249.value_SET(new byte[32], 0);
        LoopBackDemoChannel.instance.send(p249); //===============================
        DEBUG_VECT p250 = LoopBackDemoChannel.instance.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.name_SET("DEMO", PH);
        p250.time_usec_SET(1059747569937782383L);
        p250.x_SET(2.6151356E38F);
        p250.y_SET(-2.304916E38F);
        p250.z_SET(-2.5877073E38F);
        LoopBackDemoChannel.instance.send(p250); //===============================
        NAMED_VALUE_FLOAT p251 = LoopBackDemoChannel.instance.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.time_boot_ms_SET(2176127445L);
        p251.name_SET("DEMO", PH);
        p251.value_SET(1.2953862E38F);
        LoopBackDemoChannel.instance.send(p251); //===============================
        NAMED_VALUE_INT p252 = LoopBackDemoChannel.instance.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.time_boot_ms_SET(3060110295L);
        p252.name_SET("DEMO", PH);
        p252.value_SET(83330624);
        LoopBackDemoChannel.instance.send(p252); //===============================
        STATUSTEXT p253 = LoopBackDemoChannel.instance.new_STATUSTEXT();
        PH.setPack(p253);
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_ALERT);
        p253.text_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p253); //===============================
        DEBUG p254 = LoopBackDemoChannel.instance.new_DEBUG();
        PH.setPack(p254);
        p254.time_boot_ms_SET(655871166L);
        p254.ind_SET((char)35);
        p254.value_SET(-1.493188E38F);
        LoopBackDemoChannel.instance.send(p254); //===============================
        SETUP_SIGNING p256 = LoopBackDemoChannel.instance.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.target_system_SET((char)168);
        p256.target_component_SET((char)145);
        p256.secret_key_SET(new char[32], 0);
        p256.initial_timestamp_SET(1127695298634478733L);
        LoopBackDemoChannel.instance.send(p256); //===============================
        BUTTON_CHANGE p257 = LoopBackDemoChannel.instance.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.time_boot_ms_SET(2863415907L);
        p257.last_change_ms_SET(2475145433L);
        p257.state_SET((char)116);
        LoopBackDemoChannel.instance.send(p257); //===============================
        PLAY_TUNE p258 = LoopBackDemoChannel.instance.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.target_system_SET((char)77);
        p258.target_component_SET((char)214);
        p258.tune_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p258); //===============================
        CAMERA_INFORMATION p259 = LoopBackDemoChannel.instance.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.time_boot_ms_SET(3981778246L);
        p259.vendor_name_SET(new char[32], 0);
        p259.model_name_SET(new char[32], 0);
        p259.firmware_version_SET(3847064949L);
        p259.focal_length_SET(2.8018751E38F);
        p259.sensor_size_h_SET(9.255516E37F);
        p259.sensor_size_v_SET(1.2174713E38F);
        p259.resolution_h_SET((char)38164);
        p259.resolution_v_SET((char)59367);
        p259.lens_id_SET((char)100);
        p259.flags_SET(CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_MODES);
        p259.cam_definition_version_SET((char)47204);
        p259.cam_definition_uri_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p259); //===============================
        CAMERA_SETTINGS p260 = LoopBackDemoChannel.instance.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.time_boot_ms_SET(3998516955L);
        p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_IMAGE);
        LoopBackDemoChannel.instance.send(p260); //===============================
        STORAGE_INFORMATION p261 = LoopBackDemoChannel.instance.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.time_boot_ms_SET(420211096L);
        p261.storage_id_SET((char)82);
        p261.storage_count_SET((char)90);
        p261.status_SET((char)143);
        p261.total_capacity_SET(-2.0949528E38F);
        p261.used_capacity_SET(2.4213088E38F);
        p261.available_capacity_SET(1.6937784E37F);
        p261.read_speed_SET(-3.2116713E38F);
        p261.write_speed_SET(3.1814174E38F);
        LoopBackDemoChannel.instance.send(p261); //===============================
        CAMERA_CAPTURE_STATUS p262 = LoopBackDemoChannel.instance.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.time_boot_ms_SET(2442408618L);
        p262.image_status_SET((char)59);
        p262.video_status_SET((char)151);
        p262.image_interval_SET(-2.4297244E38F);
        p262.recording_time_ms_SET(2693587756L);
        p262.available_capacity_SET(3.1509923E37F);
        LoopBackDemoChannel.instance.send(p262); //===============================
        CAMERA_IMAGE_CAPTURED p263 = LoopBackDemoChannel.instance.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.time_boot_ms_SET(3252033958L);
        p263.time_utc_SET(3571439411564948768L);
        p263.camera_id_SET((char)100);
        p263.lat_SET(-1783445951);
        p263.lon_SET(-1828564289);
        p263.alt_SET(-2008717073);
        p263.relative_alt_SET(-1833101897);
        p263.q_SET(new float[4], 0);
        p263.image_index_SET(346721163);
        p263.capture_result_SET((byte)72);
        p263.file_url_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p263); //===============================
        FLIGHT_INFORMATION p264 = LoopBackDemoChannel.instance.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.time_boot_ms_SET(182177234L);
        p264.arming_time_utc_SET(3520316661188552119L);
        p264.takeoff_time_utc_SET(8356585943140620256L);
        p264.flight_uuid_SET(865735175674091856L);
        LoopBackDemoChannel.instance.send(p264); //===============================
        MOUNT_ORIENTATION p265 = LoopBackDemoChannel.instance.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.time_boot_ms_SET(2468076659L);
        p265.roll_SET(-5.409072E37F);
        p265.pitch_SET(3.2742038E38F);
        p265.yaw_SET(1.4894829E38F);
        LoopBackDemoChannel.instance.send(p265); //===============================
        LOGGING_DATA p266 = LoopBackDemoChannel.instance.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.target_system_SET((char)213);
        p266.target_component_SET((char)119);
        p266.sequence_SET((char)63616);
        p266.length_SET((char)65);
        p266.first_message_offset_SET((char)96);
        p266.data__SET(new char[249], 0);
        LoopBackDemoChannel.instance.send(p266); //===============================
        LOGGING_DATA_ACKED p267 = LoopBackDemoChannel.instance.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.target_system_SET((char)181);
        p267.target_component_SET((char)14);
        p267.sequence_SET((char)52081);
        p267.length_SET((char)154);
        p267.first_message_offset_SET((char)239);
        p267.data__SET(new char[249], 0);
        LoopBackDemoChannel.instance.send(p267); //===============================
        LOGGING_ACK p268 = LoopBackDemoChannel.instance.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.target_system_SET((char)174);
        p268.target_component_SET((char)40);
        p268.sequence_SET((char)21387);
        LoopBackDemoChannel.instance.send(p268); //===============================
        VIDEO_STREAM_INFORMATION p269 = LoopBackDemoChannel.instance.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.camera_id_SET((char)132);
        p269.status_SET((char)208);
        p269.framerate_SET(-3.0981427E38F);
        p269.resolution_h_SET((char)25071);
        p269.resolution_v_SET((char)17002);
        p269.bitrate_SET(225423444L);
        p269.rotation_SET((char)42024);
        p269.uri_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p269); //===============================
        SET_VIDEO_STREAM_SETTINGS p270 = LoopBackDemoChannel.instance.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.target_system_SET((char)59);
        p270.target_component_SET((char)26);
        p270.camera_id_SET((char)60);
        p270.framerate_SET(-5.66303E37F);
        p270.resolution_h_SET((char)20069);
        p270.resolution_v_SET((char)52007);
        p270.bitrate_SET(1294726664L);
        p270.rotation_SET((char)64589);
        p270.uri_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p270); //===============================
        WIFI_CONFIG_AP p299 = LoopBackDemoChannel.instance.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.ssid_SET("DEMO", PH);
        p299.password_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p299); //===============================
        PROTOCOL_VERSION p300 = LoopBackDemoChannel.instance.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.version_SET((char)9919);
        p300.min_version_SET((char)36255);
        p300.max_version_SET((char)32863);
        p300.spec_version_hash_SET(new char[8], 0);
        p300.library_version_hash_SET(new char[8], 0);
        LoopBackDemoChannel.instance.send(p300); //===============================
        UAVCAN_NODE_STATUS p310 = LoopBackDemoChannel.instance.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.time_usec_SET(6635921508488113908L);
        p310.uptime_sec_SET(2929606122L);
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK);
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION);
        p310.sub_mode_SET((char)180);
        p310.vendor_specific_status_code_SET((char)62008);
        LoopBackDemoChannel.instance.send(p310); //===============================
        UAVCAN_NODE_INFO p311 = LoopBackDemoChannel.instance.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.time_usec_SET(2118542073194084706L);
        p311.uptime_sec_SET(1241088101L);
        p311.name_SET("DEMO", PH);
        p311.hw_version_major_SET((char)63);
        p311.hw_version_minor_SET((char)167);
        p311.hw_unique_id_SET(new char[16], 0);
        p311.sw_version_major_SET((char)125);
        p311.sw_version_minor_SET((char)247);
        p311.sw_vcs_commit_SET(2732044467L);
        LoopBackDemoChannel.instance.send(p311); //===============================
        PARAM_EXT_REQUEST_READ p320 = LoopBackDemoChannel.instance.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.target_system_SET((char)9);
        p320.target_component_SET((char)239);
        p320.param_id_SET("DEMO", PH);
        p320.param_index_SET((short) -9510);
        LoopBackDemoChannel.instance.send(p320); //===============================
        PARAM_EXT_REQUEST_LIST p321 = LoopBackDemoChannel.instance.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_system_SET((char)152);
        p321.target_component_SET((char)28);
        LoopBackDemoChannel.instance.send(p321); //===============================
        PARAM_EXT_VALUE p322 = LoopBackDemoChannel.instance.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_id_SET("DEMO", PH);
        p322.param_value_SET("DEMO", PH);
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64);
        p322.param_count_SET((char)633);
        p322.param_index_SET((char)8462);
        LoopBackDemoChannel.instance.send(p322); //===============================
        PARAM_EXT_SET p323 = LoopBackDemoChannel.instance.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.target_system_SET((char)198);
        p323.target_component_SET((char)95);
        p323.param_id_SET("DEMO", PH);
        p323.param_value_SET("DEMO", PH);
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT8);
        LoopBackDemoChannel.instance.send(p323); //===============================
        PARAM_EXT_ACK p324 = LoopBackDemoChannel.instance.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_id_SET("DEMO", PH);
        p324.param_value_SET("DEMO", PH);
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM);
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_ACCEPTED);
        LoopBackDemoChannel.instance.send(p324); //===============================
        OBSTACLE_DISTANCE p330 = LoopBackDemoChannel.instance.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.time_usec_SET(1470330925535412712L);
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND);
        p330.distances_SET(new char[72], 0);
        p330.increment_SET((char)242);
        p330.min_distance_SET((char)61142);
        p330.max_distance_SET((char)40319);
        LoopBackDemoChannel.instance.send(p330); //===============================
        UAVIONIX_ADSB_OUT_CFG p10001 = LoopBackDemoChannel.instance.new_UAVIONIX_ADSB_OUT_CFG();
        PH.setPack(p10001);
        p10001.ICAO_SET(3645463526L);
        p10001.callsign_SET("DEMO", PH);
        p10001.emitterType_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_POINT_OBSTACLE);
        p10001.aircraftSize_SET(UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE.UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L55_45M);
        p10001.gpsOffsetLat_SET(UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_LEFT_2M);
        p10001.gpsOffsetLon_SET(UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_APPLIED_BY_SENSOR);
        p10001.stallSpeed_SET((char)14264);
        p10001.rfSelect_SET(UAVIONIX_ADSB_OUT_RF_SELECT.UAVIONIX_ADSB_OUT_RF_SELECT_STANDBY);
        LoopBackDemoChannel.instance.send(p10001); //===============================
        UAVIONIX_ADSB_OUT_DYNAMIC p10002 = LoopBackDemoChannel.instance.new_UAVIONIX_ADSB_OUT_DYNAMIC();
        PH.setPack(p10002);
        p10002.utcTime_SET(2243142184L);
        p10002.gpsLat_SET(210459673);
        p10002.gpsLon_SET(-1975147982);
        p10002.gpsAlt_SET(-1281113589);
        p10002.gpsFix_SET(UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX.UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_DGPS);
        p10002.numSats_SET((char)147);
        p10002.baroAltMSL_SET(1014465165);
        p10002.accuracyHor_SET(562282593L);
        p10002.accuracyVert_SET((char)11978);
        p10002.accuracyVel_SET((char)37636);
        p10002.velVert_SET((short) -16236);
        p10002.velNS_SET((short)25718);
        p10002.VelEW_SET((short)21602);
        p10002.emergencyStatus_SET(UAVIONIX_ADSB_EMERGENCY_STATUS.UAVIONIX_ADSB_OUT_GENERAL_EMERGENCY);
        p10002.state_SET(UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_ON_GROUND);
        p10002.squawk_SET((char)29793);
        LoopBackDemoChannel.instance.send(p10002); //===============================
        UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT p10003 = LoopBackDemoChannel.instance.new_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT();
        PH.setPack(p10003);
        p10003.rfHealth_SET(UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_FAIL_TX);
        LoopBackDemoChannel.instance.send(p10003); //===============================
        DEVICE_OP_READ p11000 = LoopBackDemoChannel.instance.new_DEVICE_OP_READ();
        PH.setPack(p11000);
        p11000.target_system_SET((char)102);
        p11000.target_component_SET((char)254);
        p11000.request_id_SET(1981435531L);
        p11000.bustype_SET(DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_I2C);
        p11000.bus_SET((char)8);
        p11000.address_SET((char)87);
        p11000.busname_SET("DEMO", PH);
        p11000.regstart_SET((char)138);
        p11000.count_SET((char)58);
        LoopBackDemoChannel.instance.send(p11000); //===============================
        DEVICE_OP_READ_REPLY p11001 = LoopBackDemoChannel.instance.new_DEVICE_OP_READ_REPLY();
        PH.setPack(p11001);
        p11001.request_id_SET(2355724120L);
        p11001.result_SET((char)3);
        p11001.regstart_SET((char)11);
        p11001.count_SET((char)37);
        p11001.data__SET(new char[128], 0);
        LoopBackDemoChannel.instance.send(p11001); //===============================
        DEVICE_OP_WRITE p11002 = LoopBackDemoChannel.instance.new_DEVICE_OP_WRITE();
        PH.setPack(p11002);
        p11002.target_system_SET((char)73);
        p11002.target_component_SET((char)161);
        p11002.request_id_SET(861202190L);
        p11002.bustype_SET(DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_I2C);
        p11002.bus_SET((char)75);
        p11002.address_SET((char)170);
        p11002.busname_SET("DEMO", PH);
        p11002.regstart_SET((char)36);
        p11002.count_SET((char)175);
        p11002.data__SET(new char[128], 0);
        LoopBackDemoChannel.instance.send(p11002); //===============================
        DEVICE_OP_WRITE_REPLY p11003 = LoopBackDemoChannel.instance.new_DEVICE_OP_WRITE_REPLY();
        PH.setPack(p11003);
        p11003.request_id_SET(1072426219L);
        p11003.result_SET((char)216);
        LoopBackDemoChannel.instance.send(p11003); //===============================
        ADAP_TUNING p11010 = LoopBackDemoChannel.instance.new_ADAP_TUNING();
        PH.setPack(p11010);
        p11010.axis_SET(PID_TUNING_AXIS.PID_TUNING_ACCZ);
        p11010.desired_SET(2.7895195E38F);
        p11010.achieved_SET(-2.6404333E37F);
        p11010.error_SET(2.3093199E38F);
        p11010.theta_SET(-1.2473835E38F);
        p11010.omega_SET(1.2482324E38F);
        p11010.sigma_SET(-1.9007269E38F);
        p11010.theta_dot_SET(-7.30173E37F);
        p11010.omega_dot_SET(-3.6337316E37F);
        p11010.sigma_dot_SET(-1.2857122E38F);
        p11010.f_SET(-4.4195325E37F);
        p11010.f_dot_SET(-6.3836557E37F);
        p11010.u_SET(-1.2742304E38F);
        LoopBackDemoChannel.instance.send(p11010); //===============================
        VISION_POSITION_DELTA p11011 = LoopBackDemoChannel.instance.new_VISION_POSITION_DELTA();
        PH.setPack(p11011);
        p11011.time_usec_SET(3929176869124210812L);
        p11011.time_delta_usec_SET(2587811248162669589L);
        p11011.angle_delta_SET(new float[3], 0);
        p11011.position_delta_SET(new float[3], 0);
        p11011.confidence_SET(-9.5031595E36F);
        LoopBackDemoChannel.instance.send(p11011); //===============================
    }
}

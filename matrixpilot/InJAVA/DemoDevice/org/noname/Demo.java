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
        LoopBackDemoChannel.instance.on_FLEXIFUNCTION_SET.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
        });
        LoopBackDemoChannel.instance.on_FLEXIFUNCTION_READ_REQ.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            short  read_req_type = pack.read_req_type_GET();
            short  data_index = pack.data_index_GET();
        });
        LoopBackDemoChannel.instance.on_FLEXIFUNCTION_BUFFER_FUNCTION.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  func_index = pack.func_index_GET();
            char  func_count = pack.func_count_GET();
            char  data_address = pack.data_address_GET();
            char  data_size = pack.data_size_GET();
            byte[]  data_ = pack.data__GET();
        });
        LoopBackDemoChannel.instance.on_FLEXIFUNCTION_BUFFER_FUNCTION_ACK.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  func_index = pack.func_index_GET();
            char  result = pack.result_GET();
        });
        LoopBackDemoChannel.instance.on_FLEXIFUNCTION_DIRECTORY.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  directory_type = pack.directory_type_GET();
            char  start_index = pack.start_index_GET();
            char  count = pack.count_GET();
            byte[]  directory_data = pack.directory_data_GET();
        });
        LoopBackDemoChannel.instance.on_FLEXIFUNCTION_DIRECTORY_ACK.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  directory_type = pack.directory_type_GET();
            char  start_index = pack.start_index_GET();
            char  count = pack.count_GET();
            char  result = pack.result_GET();
        });
        LoopBackDemoChannel.instance.on_FLEXIFUNCTION_COMMAND.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  command_type = pack.command_type_GET();
        });
        LoopBackDemoChannel.instance.on_FLEXIFUNCTION_COMMAND_ACK.add((src, ph, pack) ->
        {
            char  command_type = pack.command_type_GET();
            char  result = pack.result_GET();
        });
        LoopBackDemoChannel.instance.on_SERIAL_UDB_EXTRA_F2_A.add((src, ph, pack) ->
        {
            long  sue_time = pack.sue_time_GET();
            char  sue_status = pack.sue_status_GET();
            int  sue_latitude = pack.sue_latitude_GET();
            int  sue_longitude = pack.sue_longitude_GET();
            int  sue_altitude = pack.sue_altitude_GET();
            char  sue_waypoint_index = pack.sue_waypoint_index_GET();
            short  sue_rmat0 = pack.sue_rmat0_GET();
            short  sue_rmat1 = pack.sue_rmat1_GET();
            short  sue_rmat2 = pack.sue_rmat2_GET();
            short  sue_rmat3 = pack.sue_rmat3_GET();
            short  sue_rmat4 = pack.sue_rmat4_GET();
            short  sue_rmat5 = pack.sue_rmat5_GET();
            short  sue_rmat6 = pack.sue_rmat6_GET();
            short  sue_rmat7 = pack.sue_rmat7_GET();
            short  sue_rmat8 = pack.sue_rmat8_GET();
            char  sue_cog = pack.sue_cog_GET();
            short  sue_sog = pack.sue_sog_GET();
            char  sue_cpu_load = pack.sue_cpu_load_GET();
            char  sue_air_speed_3DIMU = pack.sue_air_speed_3DIMU_GET();
            short  sue_estimated_wind_0 = pack.sue_estimated_wind_0_GET();
            short  sue_estimated_wind_1 = pack.sue_estimated_wind_1_GET();
            short  sue_estimated_wind_2 = pack.sue_estimated_wind_2_GET();
            short  sue_magFieldEarth0 = pack.sue_magFieldEarth0_GET();
            short  sue_magFieldEarth1 = pack.sue_magFieldEarth1_GET();
            short  sue_magFieldEarth2 = pack.sue_magFieldEarth2_GET();
            short  sue_svs = pack.sue_svs_GET();
            short  sue_hdop = pack.sue_hdop_GET();
        });
        LoopBackDemoChannel.instance.on_SERIAL_UDB_EXTRA_F2_B.add((src, ph, pack) ->
        {
            long  sue_time = pack.sue_time_GET();
            short  sue_pwm_input_1 = pack.sue_pwm_input_1_GET();
            short  sue_pwm_input_2 = pack.sue_pwm_input_2_GET();
            short  sue_pwm_input_3 = pack.sue_pwm_input_3_GET();
            short  sue_pwm_input_4 = pack.sue_pwm_input_4_GET();
            short  sue_pwm_input_5 = pack.sue_pwm_input_5_GET();
            short  sue_pwm_input_6 = pack.sue_pwm_input_6_GET();
            short  sue_pwm_input_7 = pack.sue_pwm_input_7_GET();
            short  sue_pwm_input_8 = pack.sue_pwm_input_8_GET();
            short  sue_pwm_input_9 = pack.sue_pwm_input_9_GET();
            short  sue_pwm_input_10 = pack.sue_pwm_input_10_GET();
            short  sue_pwm_input_11 = pack.sue_pwm_input_11_GET();
            short  sue_pwm_input_12 = pack.sue_pwm_input_12_GET();
            short  sue_pwm_output_1 = pack.sue_pwm_output_1_GET();
            short  sue_pwm_output_2 = pack.sue_pwm_output_2_GET();
            short  sue_pwm_output_3 = pack.sue_pwm_output_3_GET();
            short  sue_pwm_output_4 = pack.sue_pwm_output_4_GET();
            short  sue_pwm_output_5 = pack.sue_pwm_output_5_GET();
            short  sue_pwm_output_6 = pack.sue_pwm_output_6_GET();
            short  sue_pwm_output_7 = pack.sue_pwm_output_7_GET();
            short  sue_pwm_output_8 = pack.sue_pwm_output_8_GET();
            short  sue_pwm_output_9 = pack.sue_pwm_output_9_GET();
            short  sue_pwm_output_10 = pack.sue_pwm_output_10_GET();
            short  sue_pwm_output_11 = pack.sue_pwm_output_11_GET();
            short  sue_pwm_output_12 = pack.sue_pwm_output_12_GET();
            short  sue_imu_location_x = pack.sue_imu_location_x_GET();
            short  sue_imu_location_y = pack.sue_imu_location_y_GET();
            short  sue_imu_location_z = pack.sue_imu_location_z_GET();
            short  sue_location_error_earth_x = pack.sue_location_error_earth_x_GET();
            short  sue_location_error_earth_y = pack.sue_location_error_earth_y_GET();
            short  sue_location_error_earth_z = pack.sue_location_error_earth_z_GET();
            long  sue_flags = pack.sue_flags_GET();
            short  sue_osc_fails = pack.sue_osc_fails_GET();
            short  sue_imu_velocity_x = pack.sue_imu_velocity_x_GET();
            short  sue_imu_velocity_y = pack.sue_imu_velocity_y_GET();
            short  sue_imu_velocity_z = pack.sue_imu_velocity_z_GET();
            short  sue_waypoint_goal_x = pack.sue_waypoint_goal_x_GET();
            short  sue_waypoint_goal_y = pack.sue_waypoint_goal_y_GET();
            short  sue_waypoint_goal_z = pack.sue_waypoint_goal_z_GET();
            short  sue_aero_x = pack.sue_aero_x_GET();
            short  sue_aero_y = pack.sue_aero_y_GET();
            short  sue_aero_z = pack.sue_aero_z_GET();
            short  sue_barom_temp = pack.sue_barom_temp_GET();
            int  sue_barom_press = pack.sue_barom_press_GET();
            int  sue_barom_alt = pack.sue_barom_alt_GET();
            short  sue_bat_volt = pack.sue_bat_volt_GET();
            short  sue_bat_amp = pack.sue_bat_amp_GET();
            short  sue_bat_amp_hours = pack.sue_bat_amp_hours_GET();
            short  sue_desired_height = pack.sue_desired_height_GET();
            short  sue_memory_stack_free = pack.sue_memory_stack_free_GET();
        });
        LoopBackDemoChannel.instance.on_SERIAL_UDB_EXTRA_F4.add((src, ph, pack) ->
        {
            char  sue_ROLL_STABILIZATION_AILERONS = pack.sue_ROLL_STABILIZATION_AILERONS_GET();
            char  sue_ROLL_STABILIZATION_RUDDER = pack.sue_ROLL_STABILIZATION_RUDDER_GET();
            char  sue_PITCH_STABILIZATION = pack.sue_PITCH_STABILIZATION_GET();
            char  sue_YAW_STABILIZATION_RUDDER = pack.sue_YAW_STABILIZATION_RUDDER_GET();
            char  sue_YAW_STABILIZATION_AILERON = pack.sue_YAW_STABILIZATION_AILERON_GET();
            char  sue_AILERON_NAVIGATION = pack.sue_AILERON_NAVIGATION_GET();
            char  sue_RUDDER_NAVIGATION = pack.sue_RUDDER_NAVIGATION_GET();
            char  sue_ALTITUDEHOLD_STABILIZED = pack.sue_ALTITUDEHOLD_STABILIZED_GET();
            char  sue_ALTITUDEHOLD_WAYPOINT = pack.sue_ALTITUDEHOLD_WAYPOINT_GET();
            char  sue_RACING_MODE = pack.sue_RACING_MODE_GET();
        });
        LoopBackDemoChannel.instance.on_SERIAL_UDB_EXTRA_F5.add((src, ph, pack) ->
        {
            float  sue_YAWKP_AILERON = pack.sue_YAWKP_AILERON_GET();
            float  sue_YAWKD_AILERON = pack.sue_YAWKD_AILERON_GET();
            float  sue_ROLLKP = pack.sue_ROLLKP_GET();
            float  sue_ROLLKD = pack.sue_ROLLKD_GET();
        });
        LoopBackDemoChannel.instance.on_SERIAL_UDB_EXTRA_F6.add((src, ph, pack) ->
        {
            float  sue_PITCHGAIN = pack.sue_PITCHGAIN_GET();
            float  sue_PITCHKD = pack.sue_PITCHKD_GET();
            float  sue_RUDDER_ELEV_MIX = pack.sue_RUDDER_ELEV_MIX_GET();
            float  sue_ROLL_ELEV_MIX = pack.sue_ROLL_ELEV_MIX_GET();
            float  sue_ELEVATOR_BOOST = pack.sue_ELEVATOR_BOOST_GET();
        });
        LoopBackDemoChannel.instance.on_SERIAL_UDB_EXTRA_F7.add((src, ph, pack) ->
        {
            float  sue_YAWKP_RUDDER = pack.sue_YAWKP_RUDDER_GET();
            float  sue_YAWKD_RUDDER = pack.sue_YAWKD_RUDDER_GET();
            float  sue_ROLLKP_RUDDER = pack.sue_ROLLKP_RUDDER_GET();
            float  sue_ROLLKD_RUDDER = pack.sue_ROLLKD_RUDDER_GET();
            float  sue_RUDDER_BOOST = pack.sue_RUDDER_BOOST_GET();
            float  sue_RTL_PITCH_DOWN = pack.sue_RTL_PITCH_DOWN_GET();
        });
        LoopBackDemoChannel.instance.on_SERIAL_UDB_EXTRA_F8.add((src, ph, pack) ->
        {
            float  sue_HEIGHT_TARGET_MAX = pack.sue_HEIGHT_TARGET_MAX_GET();
            float  sue_HEIGHT_TARGET_MIN = pack.sue_HEIGHT_TARGET_MIN_GET();
            float  sue_ALT_HOLD_THROTTLE_MIN = pack.sue_ALT_HOLD_THROTTLE_MIN_GET();
            float  sue_ALT_HOLD_THROTTLE_MAX = pack.sue_ALT_HOLD_THROTTLE_MAX_GET();
            float  sue_ALT_HOLD_PITCH_MIN = pack.sue_ALT_HOLD_PITCH_MIN_GET();
            float  sue_ALT_HOLD_PITCH_MAX = pack.sue_ALT_HOLD_PITCH_MAX_GET();
            float  sue_ALT_HOLD_PITCH_HIGH = pack.sue_ALT_HOLD_PITCH_HIGH_GET();
        });
        LoopBackDemoChannel.instance.on_SERIAL_UDB_EXTRA_F13.add((src, ph, pack) ->
        {
            short  sue_week_no = pack.sue_week_no_GET();
            int  sue_lat_origin = pack.sue_lat_origin_GET();
            int  sue_lon_origin = pack.sue_lon_origin_GET();
            int  sue_alt_origin = pack.sue_alt_origin_GET();
        });
        LoopBackDemoChannel.instance.on_SERIAL_UDB_EXTRA_F14.add((src, ph, pack) ->
        {
            char  sue_WIND_ESTIMATION = pack.sue_WIND_ESTIMATION_GET();
            char  sue_GPS_TYPE = pack.sue_GPS_TYPE_GET();
            char  sue_DR = pack.sue_DR_GET();
            char  sue_BOARD_TYPE = pack.sue_BOARD_TYPE_GET();
            char  sue_AIRFRAME = pack.sue_AIRFRAME_GET();
            short  sue_RCON = pack.sue_RCON_GET();
            short  sue_TRAP_FLAGS = pack.sue_TRAP_FLAGS_GET();
            long  sue_TRAP_SOURCE = pack.sue_TRAP_SOURCE_GET();
            short  sue_osc_fail_count = pack.sue_osc_fail_count_GET();
            char  sue_CLOCK_CONFIG = pack.sue_CLOCK_CONFIG_GET();
            char  sue_FLIGHT_PLAN_TYPE = pack.sue_FLIGHT_PLAN_TYPE_GET();
        });
        LoopBackDemoChannel.instance.on_SERIAL_UDB_EXTRA_F15.add((src, ph, pack) ->
        {
            char[]  sue_ID_VEHICLE_MODEL_NAME = pack.sue_ID_VEHICLE_MODEL_NAME_GET();
            char[]  sue_ID_VEHICLE_REGISTRATION = pack.sue_ID_VEHICLE_REGISTRATION_GET();
        });
        LoopBackDemoChannel.instance.on_SERIAL_UDB_EXTRA_F16.add((src, ph, pack) ->
        {
            char[]  sue_ID_LEAD_PILOT = pack.sue_ID_LEAD_PILOT_GET();
            char[]  sue_ID_DIY_DRONES_URL = pack.sue_ID_DIY_DRONES_URL_GET();
        });
        LoopBackDemoChannel.instance.on_ALTITUDES.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            int  alt_gps = pack.alt_gps_GET();
            int  alt_imu = pack.alt_imu_GET();
            int  alt_barometric = pack.alt_barometric_GET();
            int  alt_optical_flow = pack.alt_optical_flow_GET();
            int  alt_range_finder = pack.alt_range_finder_GET();
            int  alt_extra = pack.alt_extra_GET();
        });
        LoopBackDemoChannel.instance.on_AIRSPEEDS.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            short  airspeed_imu = pack.airspeed_imu_GET();
            short  airspeed_pitot = pack.airspeed_pitot_GET();
            short  airspeed_hot_wire = pack.airspeed_hot_wire_GET();
            short  airspeed_ultrasonic = pack.airspeed_ultrasonic_GET();
            short  aoa = pack.aoa_GET();
            short  aoy = pack.aoy_GET();
        });
        LoopBackDemoChannel.instance.on_SERIAL_UDB_EXTRA_F17.add((src, ph, pack) ->
        {
            float  sue_feed_forward = pack.sue_feed_forward_GET();
            float  sue_turn_rate_nav = pack.sue_turn_rate_nav_GET();
            float  sue_turn_rate_fbw = pack.sue_turn_rate_fbw_GET();
        });
        LoopBackDemoChannel.instance.on_SERIAL_UDB_EXTRA_F18.add((src, ph, pack) ->
        {
            float  angle_of_attack_normal = pack.angle_of_attack_normal_GET();
            float  angle_of_attack_inverted = pack.angle_of_attack_inverted_GET();
            float  elevator_trim_normal = pack.elevator_trim_normal_GET();
            float  elevator_trim_inverted = pack.elevator_trim_inverted_GET();
            float  reference_speed = pack.reference_speed_GET();
        });
        LoopBackDemoChannel.instance.on_SERIAL_UDB_EXTRA_F19.add((src, ph, pack) ->
        {
            char  sue_aileron_output_channel = pack.sue_aileron_output_channel_GET();
            char  sue_aileron_reversed = pack.sue_aileron_reversed_GET();
            char  sue_elevator_output_channel = pack.sue_elevator_output_channel_GET();
            char  sue_elevator_reversed = pack.sue_elevator_reversed_GET();
            char  sue_throttle_output_channel = pack.sue_throttle_output_channel_GET();
            char  sue_throttle_reversed = pack.sue_throttle_reversed_GET();
            char  sue_rudder_output_channel = pack.sue_rudder_output_channel_GET();
            char  sue_rudder_reversed = pack.sue_rudder_reversed_GET();
        });
        LoopBackDemoChannel.instance.on_SERIAL_UDB_EXTRA_F20.add((src, ph, pack) ->
        {
            char  sue_number_of_inputs = pack.sue_number_of_inputs_GET();
            short  sue_trim_value_input_1 = pack.sue_trim_value_input_1_GET();
            short  sue_trim_value_input_2 = pack.sue_trim_value_input_2_GET();
            short  sue_trim_value_input_3 = pack.sue_trim_value_input_3_GET();
            short  sue_trim_value_input_4 = pack.sue_trim_value_input_4_GET();
            short  sue_trim_value_input_5 = pack.sue_trim_value_input_5_GET();
            short  sue_trim_value_input_6 = pack.sue_trim_value_input_6_GET();
            short  sue_trim_value_input_7 = pack.sue_trim_value_input_7_GET();
            short  sue_trim_value_input_8 = pack.sue_trim_value_input_8_GET();
            short  sue_trim_value_input_9 = pack.sue_trim_value_input_9_GET();
            short  sue_trim_value_input_10 = pack.sue_trim_value_input_10_GET();
            short  sue_trim_value_input_11 = pack.sue_trim_value_input_11_GET();
            short  sue_trim_value_input_12 = pack.sue_trim_value_input_12_GET();
        });
        LoopBackDemoChannel.instance.on_SERIAL_UDB_EXTRA_F21.add((src, ph, pack) ->
        {
            short  sue_accel_x_offset = pack.sue_accel_x_offset_GET();
            short  sue_accel_y_offset = pack.sue_accel_y_offset_GET();
            short  sue_accel_z_offset = pack.sue_accel_z_offset_GET();
            short  sue_gyro_x_offset = pack.sue_gyro_x_offset_GET();
            short  sue_gyro_y_offset = pack.sue_gyro_y_offset_GET();
            short  sue_gyro_z_offset = pack.sue_gyro_z_offset_GET();
        });
        LoopBackDemoChannel.instance.on_SERIAL_UDB_EXTRA_F22.add((src, ph, pack) ->
        {
            short  sue_accel_x_at_calibration = pack.sue_accel_x_at_calibration_GET();
            short  sue_accel_y_at_calibration = pack.sue_accel_y_at_calibration_GET();
            short  sue_accel_z_at_calibration = pack.sue_accel_z_at_calibration_GET();
            short  sue_gyro_x_at_calibration = pack.sue_gyro_x_at_calibration_GET();
            short  sue_gyro_y_at_calibration = pack.sue_gyro_y_at_calibration_GET();
            short  sue_gyro_z_at_calibration = pack.sue_gyro_z_at_calibration_GET();
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
        p0.type_SET(MAV_TYPE.MAV_TYPE_FIXED_WING);
        p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_AEROB);
        p0.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);
        p0.custom_mode_SET(2190230309L);
        p0.system_status_SET(MAV_STATE.MAV_STATE_ACTIVE);
        p0.mavlink_version_SET((char)200);
        LoopBackDemoChannel.instance.send(p0); //===============================
        SYS_STATUS p1 = LoopBackDemoChannel.instance.new_SYS_STATUS();
        PH.setPack(p1);
        p1.onboard_control_sensors_present_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG);
        p1.onboard_control_sensors_enabled_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING);
        p1.onboard_control_sensors_health_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW);
        p1.load_SET((char)11421);
        p1.voltage_battery_SET((char)29374);
        p1.current_battery_SET((short)7839);
        p1.battery_remaining_SET((byte) - 29);
        p1.drop_rate_comm_SET((char)1069);
        p1.errors_comm_SET((char)3139);
        p1.errors_count1_SET((char)59434);
        p1.errors_count2_SET((char)55514);
        p1.errors_count3_SET((char)18084);
        p1.errors_count4_SET((char)58565);
        LoopBackDemoChannel.instance.send(p1); //===============================
        SYSTEM_TIME p2 = LoopBackDemoChannel.instance.new_SYSTEM_TIME();
        PH.setPack(p2);
        p2.time_unix_usec_SET(1758431511549418961L);
        p2.time_boot_ms_SET(311951327L);
        LoopBackDemoChannel.instance.send(p2); //===============================
        POSITION_TARGET_LOCAL_NED p3 = LoopBackDemoChannel.instance.new_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.time_boot_ms_SET(2366834459L);
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_MISSION);
        p3.type_mask_SET((char)32775);
        p3.x_SET(2.1850962E38F);
        p3.y_SET(7.468343E37F);
        p3.z_SET(-3.2826557E38F);
        p3.vx_SET(-5.001502E37F);
        p3.vy_SET(-3.1982648E38F);
        p3.vz_SET(-1.9023428E38F);
        p3.afx_SET(3.3110973E38F);
        p3.afy_SET(9.72974E37F);
        p3.afz_SET(2.6113795E38F);
        p3.yaw_SET(2.941662E38F);
        p3.yaw_rate_SET(2.3621241E38F);
        LoopBackDemoChannel.instance.send(p3); //===============================
        PING p4 = LoopBackDemoChannel.instance.new_PING();
        PH.setPack(p4);
        p4.time_usec_SET(5428680912905619688L);
        p4.seq_SET(2540387477L);
        p4.target_system_SET((char)193);
        p4.target_component_SET((char)185);
        LoopBackDemoChannel.instance.send(p4); //===============================
        CHANGE_OPERATOR_CONTROL p5 = LoopBackDemoChannel.instance.new_CHANGE_OPERATOR_CONTROL();
        PH.setPack(p5);
        p5.target_system_SET((char)11);
        p5.control_request_SET((char)190);
        p5.version_SET((char)146);
        p5.passkey_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p5); //===============================
        CHANGE_OPERATOR_CONTROL_ACK p6 = LoopBackDemoChannel.instance.new_CHANGE_OPERATOR_CONTROL_ACK();
        PH.setPack(p6);
        p6.gcs_system_id_SET((char)67);
        p6.control_request_SET((char)104);
        p6.ack_SET((char)100);
        LoopBackDemoChannel.instance.send(p6); //===============================
        AUTH_KEY p7 = LoopBackDemoChannel.instance.new_AUTH_KEY();
        PH.setPack(p7);
        p7.key_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p7); //===============================
        SET_MODE p11 = LoopBackDemoChannel.instance.new_SET_MODE();
        PH.setPack(p11);
        p11.target_system_SET((char)62);
        p11.base_mode_SET(MAV_MODE.MAV_MODE_AUTO_DISARMED);
        p11.custom_mode_SET(1952607191L);
        LoopBackDemoChannel.instance.send(p11); //===============================
        PARAM_REQUEST_READ p20 = LoopBackDemoChannel.instance.new_PARAM_REQUEST_READ();
        PH.setPack(p20);
        p20.target_system_SET((char)108);
        p20.target_component_SET((char)191);
        p20.param_id_SET("DEMO", PH);
        p20.param_index_SET((short)5681);
        LoopBackDemoChannel.instance.send(p20); //===============================
        PARAM_REQUEST_LIST p21 = LoopBackDemoChannel.instance.new_PARAM_REQUEST_LIST();
        PH.setPack(p21);
        p21.target_system_SET((char)15);
        p21.target_component_SET((char)101);
        LoopBackDemoChannel.instance.send(p21); //===============================
        PARAM_VALUE p22 = LoopBackDemoChannel.instance.new_PARAM_VALUE();
        PH.setPack(p22);
        p22.param_id_SET("DEMO", PH);
        p22.param_value_SET(3.2320732E38F);
        p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT64);
        p22.param_count_SET((char)10565);
        p22.param_index_SET((char)14561);
        LoopBackDemoChannel.instance.send(p22); //===============================
        PARAM_SET p23 = LoopBackDemoChannel.instance.new_PARAM_SET();
        PH.setPack(p23);
        p23.target_system_SET((char)40);
        p23.target_component_SET((char)76);
        p23.param_id_SET("DEMO", PH);
        p23.param_value_SET(7.4511976E36F);
        p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT64);
        LoopBackDemoChannel.instance.send(p23); //===============================
        GPS_RAW_INT p24 = LoopBackDemoChannel.instance.new_GPS_RAW_INT();
        PH.setPack(p24);
        p24.time_usec_SET(5651465526787339229L);
        p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX);
        p24.lat_SET(1887314509);
        p24.lon_SET(1468113483);
        p24.alt_SET(1935278480);
        p24.eph_SET((char)59642);
        p24.epv_SET((char)55718);
        p24.vel_SET((char)41531);
        p24.cog_SET((char)51589);
        p24.satellites_visible_SET((char)178);
        p24.alt_ellipsoid_SET(-231351713, PH);
        p24.h_acc_SET(3935390785L, PH);
        p24.v_acc_SET(3104413598L, PH);
        p24.vel_acc_SET(3200949862L, PH);
        p24.hdg_acc_SET(3799796181L, PH);
        LoopBackDemoChannel.instance.send(p24); //===============================
        GPS_STATUS p25 = LoopBackDemoChannel.instance.new_GPS_STATUS();
        PH.setPack(p25);
        p25.satellites_visible_SET((char)182);
        p25.satellite_prn_SET(new char[20], 0);
        p25.satellite_used_SET(new char[20], 0);
        p25.satellite_elevation_SET(new char[20], 0);
        p25.satellite_azimuth_SET(new char[20], 0);
        p25.satellite_snr_SET(new char[20], 0);
        LoopBackDemoChannel.instance.send(p25); //===============================
        SCALED_IMU p26 = LoopBackDemoChannel.instance.new_SCALED_IMU();
        PH.setPack(p26);
        p26.time_boot_ms_SET(1949519955L);
        p26.xacc_SET((short) -6437);
        p26.yacc_SET((short) -2988);
        p26.zacc_SET((short)2209);
        p26.xgyro_SET((short)12760);
        p26.ygyro_SET((short)21975);
        p26.zgyro_SET((short) -21557);
        p26.xmag_SET((short)20897);
        p26.ymag_SET((short) -26984);
        p26.zmag_SET((short)4248);
        LoopBackDemoChannel.instance.send(p26); //===============================
        RAW_IMU p27 = LoopBackDemoChannel.instance.new_RAW_IMU();
        PH.setPack(p27);
        p27.time_usec_SET(6909103308022696299L);
        p27.xacc_SET((short)11922);
        p27.yacc_SET((short) -30830);
        p27.zacc_SET((short) -17554);
        p27.xgyro_SET((short)16509);
        p27.ygyro_SET((short) -235);
        p27.zgyro_SET((short) -28745);
        p27.xmag_SET((short)29312);
        p27.ymag_SET((short)8359);
        p27.zmag_SET((short)12706);
        LoopBackDemoChannel.instance.send(p27); //===============================
        RAW_PRESSURE p28 = LoopBackDemoChannel.instance.new_RAW_PRESSURE();
        PH.setPack(p28);
        p28.time_usec_SET(5419668405743077828L);
        p28.press_abs_SET((short)9417);
        p28.press_diff1_SET((short) -23417);
        p28.press_diff2_SET((short) -26662);
        p28.temperature_SET((short)5184);
        LoopBackDemoChannel.instance.send(p28); //===============================
        SCALED_PRESSURE p29 = LoopBackDemoChannel.instance.new_SCALED_PRESSURE();
        PH.setPack(p29);
        p29.time_boot_ms_SET(403606604L);
        p29.press_abs_SET(2.4310164E38F);
        p29.press_diff_SET(2.5309932E38F);
        p29.temperature_SET((short) -19477);
        LoopBackDemoChannel.instance.send(p29); //===============================
        ATTITUDE p30 = LoopBackDemoChannel.instance.new_ATTITUDE();
        PH.setPack(p30);
        p30.time_boot_ms_SET(1889644603L);
        p30.roll_SET(-1.0808507E38F);
        p30.pitch_SET(-2.836642E38F);
        p30.yaw_SET(3.102992E38F);
        p30.rollspeed_SET(2.684939E38F);
        p30.pitchspeed_SET(1.3945029E37F);
        p30.yawspeed_SET(-2.6038379E38F);
        LoopBackDemoChannel.instance.send(p30); //===============================
        ATTITUDE_QUATERNION p31 = LoopBackDemoChannel.instance.new_ATTITUDE_QUATERNION();
        PH.setPack(p31);
        p31.time_boot_ms_SET(984400531L);
        p31.q1_SET(2.3381429E36F);
        p31.q2_SET(1.816081E38F);
        p31.q3_SET(-6.1523394E37F);
        p31.q4_SET(2.5370808E38F);
        p31.rollspeed_SET(-5.6490653E37F);
        p31.pitchspeed_SET(5.332475E37F);
        p31.yawspeed_SET(1.7041009E38F);
        LoopBackDemoChannel.instance.send(p31); //===============================
        LOCAL_POSITION_NED p32 = LoopBackDemoChannel.instance.new_LOCAL_POSITION_NED();
        PH.setPack(p32);
        p32.time_boot_ms_SET(749652892L);
        p32.x_SET(5.5187235E37F);
        p32.y_SET(-1.7304159E38F);
        p32.z_SET(3.1144184E38F);
        p32.vx_SET(-1.7082048E38F);
        p32.vy_SET(1.3695445E38F);
        p32.vz_SET(-2.3436276E38F);
        LoopBackDemoChannel.instance.send(p32); //===============================
        GLOBAL_POSITION_INT p33 = LoopBackDemoChannel.instance.new_GLOBAL_POSITION_INT();
        PH.setPack(p33);
        p33.time_boot_ms_SET(81517476L);
        p33.lat_SET(543770551);
        p33.lon_SET(-1043382492);
        p33.alt_SET(734456777);
        p33.relative_alt_SET(-1251067330);
        p33.vx_SET((short) -14651);
        p33.vy_SET((short)1259);
        p33.vz_SET((short)2614);
        p33.hdg_SET((char)52027);
        LoopBackDemoChannel.instance.send(p33); //===============================
        RC_CHANNELS_SCALED p34 = LoopBackDemoChannel.instance.new_RC_CHANNELS_SCALED();
        PH.setPack(p34);
        p34.time_boot_ms_SET(3068358152L);
        p34.port_SET((char)43);
        p34.chan1_scaled_SET((short) -23133);
        p34.chan2_scaled_SET((short)26889);
        p34.chan3_scaled_SET((short)1990);
        p34.chan4_scaled_SET((short) -12426);
        p34.chan5_scaled_SET((short)10666);
        p34.chan6_scaled_SET((short)15899);
        p34.chan7_scaled_SET((short) -1724);
        p34.chan8_scaled_SET((short) -5034);
        p34.rssi_SET((char)208);
        LoopBackDemoChannel.instance.send(p34); //===============================
        RC_CHANNELS_RAW p35 = LoopBackDemoChannel.instance.new_RC_CHANNELS_RAW();
        PH.setPack(p35);
        p35.time_boot_ms_SET(1753994342L);
        p35.port_SET((char)113);
        p35.chan1_raw_SET((char)42714);
        p35.chan2_raw_SET((char)54349);
        p35.chan3_raw_SET((char)24833);
        p35.chan4_raw_SET((char)21407);
        p35.chan5_raw_SET((char)58939);
        p35.chan6_raw_SET((char)53223);
        p35.chan7_raw_SET((char)9112);
        p35.chan8_raw_SET((char)19598);
        p35.rssi_SET((char)226);
        LoopBackDemoChannel.instance.send(p35); //===============================
        SERVO_OUTPUT_RAW p36 = LoopBackDemoChannel.instance.new_SERVO_OUTPUT_RAW();
        PH.setPack(p36);
        p36.time_usec_SET(706364650L);
        p36.port_SET((char)77);
        p36.servo1_raw_SET((char)7194);
        p36.servo2_raw_SET((char)48320);
        p36.servo3_raw_SET((char)60440);
        p36.servo4_raw_SET((char)48874);
        p36.servo5_raw_SET((char)54472);
        p36.servo6_raw_SET((char)51940);
        p36.servo7_raw_SET((char)56593);
        p36.servo8_raw_SET((char)3756);
        p36.servo9_raw_SET((char)37142, PH);
        p36.servo10_raw_SET((char)65042, PH);
        p36.servo11_raw_SET((char)48138, PH);
        p36.servo12_raw_SET((char)13462, PH);
        p36.servo13_raw_SET((char)26059, PH);
        p36.servo14_raw_SET((char)33033, PH);
        p36.servo15_raw_SET((char)56572, PH);
        p36.servo16_raw_SET((char)7726, PH);
        LoopBackDemoChannel.instance.send(p36); //===============================
        MISSION_REQUEST_PARTIAL_LIST p37 = LoopBackDemoChannel.instance.new_MISSION_REQUEST_PARTIAL_LIST();
        PH.setPack(p37);
        p37.target_system_SET((char)235);
        p37.target_component_SET((char)252);
        p37.start_index_SET((short) -30769);
        p37.end_index_SET((short) -7368);
        p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
        LoopBackDemoChannel.instance.send(p37); //===============================
        MISSION_WRITE_PARTIAL_LIST p38 = LoopBackDemoChannel.instance.new_MISSION_WRITE_PARTIAL_LIST();
        PH.setPack(p38);
        p38.target_system_SET((char)107);
        p38.target_component_SET((char)33);
        p38.start_index_SET((short)4956);
        p38.end_index_SET((short)10035);
        p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
        LoopBackDemoChannel.instance.send(p38); //===============================
        MISSION_ITEM p39 = LoopBackDemoChannel.instance.new_MISSION_ITEM();
        PH.setPack(p39);
        p39.target_system_SET((char)128);
        p39.target_component_SET((char)209);
        p39.seq_SET((char)7732);
        p39.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
        p39.command_SET(MAV_CMD.MAV_CMD_DO_SET_ROI);
        p39.current_SET((char)222);
        p39.autocontinue_SET((char)98);
        p39.param1_SET(-1.2969168E37F);
        p39.param2_SET(1.04122386E37F);
        p39.param3_SET(-1.914746E38F);
        p39.param4_SET(1.5413137E38F);
        p39.x_SET(6.3584074E36F);
        p39.y_SET(-2.8533862E37F);
        p39.z_SET(-1.4707964E38F);
        p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
        LoopBackDemoChannel.instance.send(p39); //===============================
        MISSION_REQUEST p40 = LoopBackDemoChannel.instance.new_MISSION_REQUEST();
        PH.setPack(p40);
        p40.target_system_SET((char)134);
        p40.target_component_SET((char)56);
        p40.seq_SET((char)16526);
        p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
        LoopBackDemoChannel.instance.send(p40); //===============================
        MISSION_SET_CURRENT p41 = LoopBackDemoChannel.instance.new_MISSION_SET_CURRENT();
        PH.setPack(p41);
        p41.target_system_SET((char)41);
        p41.target_component_SET((char)86);
        p41.seq_SET((char)31693);
        LoopBackDemoChannel.instance.send(p41); //===============================
        MISSION_CURRENT p42 = LoopBackDemoChannel.instance.new_MISSION_CURRENT();
        PH.setPack(p42);
        p42.seq_SET((char)17024);
        LoopBackDemoChannel.instance.send(p42); //===============================
        MISSION_REQUEST_LIST p43 = LoopBackDemoChannel.instance.new_MISSION_REQUEST_LIST();
        PH.setPack(p43);
        p43.target_system_SET((char)96);
        p43.target_component_SET((char)224);
        p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
        LoopBackDemoChannel.instance.send(p43); //===============================
        MISSION_COUNT p44 = LoopBackDemoChannel.instance.new_MISSION_COUNT();
        PH.setPack(p44);
        p44.target_system_SET((char)145);
        p44.target_component_SET((char)151);
        p44.count_SET((char)40303);
        p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
        LoopBackDemoChannel.instance.send(p44); //===============================
        MISSION_CLEAR_ALL p45 = LoopBackDemoChannel.instance.new_MISSION_CLEAR_ALL();
        PH.setPack(p45);
        p45.target_system_SET((char)32);
        p45.target_component_SET((char)174);
        p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
        LoopBackDemoChannel.instance.send(p45); //===============================
        MISSION_ITEM_REACHED p46 = LoopBackDemoChannel.instance.new_MISSION_ITEM_REACHED();
        PH.setPack(p46);
        p46.seq_SET((char)61679);
        LoopBackDemoChannel.instance.send(p46); //===============================
        MISSION_ACK p47 = LoopBackDemoChannel.instance.new_MISSION_ACK();
        PH.setPack(p47);
        p47.target_system_SET((char)205);
        p47.target_component_SET((char)217);
        p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_ERROR);
        p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
        LoopBackDemoChannel.instance.send(p47); //===============================
        SET_GPS_GLOBAL_ORIGIN p48 = LoopBackDemoChannel.instance.new_SET_GPS_GLOBAL_ORIGIN();
        PH.setPack(p48);
        p48.target_system_SET((char)10);
        p48.latitude_SET(488432737);
        p48.longitude_SET(1627071358);
        p48.altitude_SET(-376568771);
        p48.time_usec_SET(5342034709331090328L, PH);
        LoopBackDemoChannel.instance.send(p48); //===============================
        GPS_GLOBAL_ORIGIN p49 = LoopBackDemoChannel.instance.new_GPS_GLOBAL_ORIGIN();
        PH.setPack(p49);
        p49.latitude_SET(861738168);
        p49.longitude_SET(1261936569);
        p49.altitude_SET(-324332254);
        p49.time_usec_SET(7506595398783094169L, PH);
        LoopBackDemoChannel.instance.send(p49); //===============================
        PARAM_MAP_RC p50 = LoopBackDemoChannel.instance.new_PARAM_MAP_RC();
        PH.setPack(p50);
        p50.target_system_SET((char)224);
        p50.target_component_SET((char)158);
        p50.param_id_SET("DEMO", PH);
        p50.param_index_SET((short) -14483);
        p50.parameter_rc_channel_index_SET((char)166);
        p50.param_value0_SET(-9.127954E37F);
        p50.scale_SET(-1.5482296E38F);
        p50.param_value_min_SET(3.3854716E37F);
        p50.param_value_max_SET(-2.5346735E38F);
        LoopBackDemoChannel.instance.send(p50); //===============================
        MISSION_REQUEST_INT p51 = LoopBackDemoChannel.instance.new_MISSION_REQUEST_INT();
        PH.setPack(p51);
        p51.target_system_SET((char)237);
        p51.target_component_SET((char)99);
        p51.seq_SET((char)45845);
        p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
        LoopBackDemoChannel.instance.send(p51); //===============================
        SAFETY_SET_ALLOWED_AREA p54 = LoopBackDemoChannel.instance.new_SAFETY_SET_ALLOWED_AREA();
        PH.setPack(p54);
        p54.target_system_SET((char)78);
        p54.target_component_SET((char)151);
        p54.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_ENU);
        p54.p1x_SET(6.1993793E37F);
        p54.p1y_SET(-2.5952487E38F);
        p54.p1z_SET(-1.709534E38F);
        p54.p2x_SET(-2.647597E38F);
        p54.p2y_SET(1.4030576E38F);
        p54.p2z_SET(-3.9046313E37F);
        LoopBackDemoChannel.instance.send(p54); //===============================
        SAFETY_ALLOWED_AREA p55 = LoopBackDemoChannel.instance.new_SAFETY_ALLOWED_AREA();
        PH.setPack(p55);
        p55.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_NED);
        p55.p1x_SET(-1.7311886E38F);
        p55.p1y_SET(-1.0861742E38F);
        p55.p1z_SET(-2.3594651E37F);
        p55.p2x_SET(6.7624104E37F);
        p55.p2y_SET(2.6042166E38F);
        p55.p2z_SET(1.9122388E38F);
        LoopBackDemoChannel.instance.send(p55); //===============================
        ATTITUDE_QUATERNION_COV p61 = LoopBackDemoChannel.instance.new_ATTITUDE_QUATERNION_COV();
        PH.setPack(p61);
        p61.time_usec_SET(3394046352020450763L);
        p61.q_SET(new float[4], 0);
        p61.rollspeed_SET(2.921706E38F);
        p61.pitchspeed_SET(2.9230911E38F);
        p61.yawspeed_SET(4.342031E37F);
        p61.covariance_SET(new float[9], 0);
        LoopBackDemoChannel.instance.send(p61); //===============================
        NAV_CONTROLLER_OUTPUT p62 = LoopBackDemoChannel.instance.new_NAV_CONTROLLER_OUTPUT();
        PH.setPack(p62);
        p62.nav_roll_SET(-1.6614635E38F);
        p62.nav_pitch_SET(-7.299806E37F);
        p62.nav_bearing_SET((short) -8067);
        p62.target_bearing_SET((short)14793);
        p62.wp_dist_SET((char)30338);
        p62.alt_error_SET(2.8258433E38F);
        p62.aspd_error_SET(-2.2100618E38F);
        p62.xtrack_error_SET(1.4108147E38F);
        LoopBackDemoChannel.instance.send(p62); //===============================
        GLOBAL_POSITION_INT_COV p63 = LoopBackDemoChannel.instance.new_GLOBAL_POSITION_INT_COV();
        PH.setPack(p63);
        p63.time_usec_SET(2921874320116083251L);
        p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS);
        p63.lat_SET(1187852789);
        p63.lon_SET(1417058883);
        p63.alt_SET(673951492);
        p63.relative_alt_SET(46194861);
        p63.vx_SET(-1.3180425E36F);
        p63.vy_SET(-1.6811141E38F);
        p63.vz_SET(3.0724743E38F);
        p63.covariance_SET(new float[36], 0);
        LoopBackDemoChannel.instance.send(p63); //===============================
        LOCAL_POSITION_NED_COV p64 = LoopBackDemoChannel.instance.new_LOCAL_POSITION_NED_COV();
        PH.setPack(p64);
        p64.time_usec_SET(4341732968000118344L);
        p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE);
        p64.x_SET(-2.4334882E38F);
        p64.y_SET(1.7852816E38F);
        p64.z_SET(-1.8782931E38F);
        p64.vx_SET(-9.08994E37F);
        p64.vy_SET(-1.5124779E38F);
        p64.vz_SET(-1.4624849E38F);
        p64.ax_SET(-1.8168644E38F);
        p64.ay_SET(1.9949902E38F);
        p64.az_SET(-3.1731264E38F);
        p64.covariance_SET(new float[45], 0);
        LoopBackDemoChannel.instance.send(p64); //===============================
        RC_CHANNELS p65 = LoopBackDemoChannel.instance.new_RC_CHANNELS();
        PH.setPack(p65);
        p65.time_boot_ms_SET(845764785L);
        p65.chancount_SET((char)140);
        p65.chan1_raw_SET((char)58731);
        p65.chan2_raw_SET((char)25139);
        p65.chan3_raw_SET((char)24484);
        p65.chan4_raw_SET((char)21967);
        p65.chan5_raw_SET((char)18979);
        p65.chan6_raw_SET((char)45305);
        p65.chan7_raw_SET((char)62383);
        p65.chan8_raw_SET((char)52370);
        p65.chan9_raw_SET((char)40715);
        p65.chan10_raw_SET((char)41144);
        p65.chan11_raw_SET((char)29404);
        p65.chan12_raw_SET((char)55190);
        p65.chan13_raw_SET((char)13338);
        p65.chan14_raw_SET((char)16969);
        p65.chan15_raw_SET((char)39355);
        p65.chan16_raw_SET((char)41475);
        p65.chan17_raw_SET((char)60668);
        p65.chan18_raw_SET((char)24524);
        p65.rssi_SET((char)17);
        LoopBackDemoChannel.instance.send(p65); //===============================
        REQUEST_DATA_STREAM p66 = LoopBackDemoChannel.instance.new_REQUEST_DATA_STREAM();
        PH.setPack(p66);
        p66.target_system_SET((char)205);
        p66.target_component_SET((char)45);
        p66.req_stream_id_SET((char)137);
        p66.req_message_rate_SET((char)34616);
        p66.start_stop_SET((char)17);
        LoopBackDemoChannel.instance.send(p66); //===============================
        DATA_STREAM p67 = LoopBackDemoChannel.instance.new_DATA_STREAM();
        PH.setPack(p67);
        p67.stream_id_SET((char)127);
        p67.message_rate_SET((char)28478);
        p67.on_off_SET((char)27);
        LoopBackDemoChannel.instance.send(p67); //===============================
        MANUAL_CONTROL p69 = LoopBackDemoChannel.instance.new_MANUAL_CONTROL();
        PH.setPack(p69);
        p69.target_SET((char)88);
        p69.x_SET((short) -30766);
        p69.y_SET((short) -29497);
        p69.z_SET((short) -15938);
        p69.r_SET((short)18717);
        p69.buttons_SET((char)52994);
        LoopBackDemoChannel.instance.send(p69); //===============================
        RC_CHANNELS_OVERRIDE p70 = LoopBackDemoChannel.instance.new_RC_CHANNELS_OVERRIDE();
        PH.setPack(p70);
        p70.target_system_SET((char)157);
        p70.target_component_SET((char)205);
        p70.chan1_raw_SET((char)3245);
        p70.chan2_raw_SET((char)16773);
        p70.chan3_raw_SET((char)37240);
        p70.chan4_raw_SET((char)44603);
        p70.chan5_raw_SET((char)61892);
        p70.chan6_raw_SET((char)2969);
        p70.chan7_raw_SET((char)22492);
        p70.chan8_raw_SET((char)39189);
        LoopBackDemoChannel.instance.send(p70); //===============================
        MISSION_ITEM_INT p73 = LoopBackDemoChannel.instance.new_MISSION_ITEM_INT();
        PH.setPack(p73);
        p73.target_system_SET((char)142);
        p73.target_component_SET((char)130);
        p73.seq_SET((char)34941);
        p73.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
        p73.command_SET(MAV_CMD.MAV_CMD_NAV_LOITER_UNLIM);
        p73.current_SET((char)243);
        p73.autocontinue_SET((char)109);
        p73.param1_SET(1.7547059E38F);
        p73.param2_SET(1.7786767E38F);
        p73.param3_SET(-2.269169E38F);
        p73.param4_SET(2.3399986E37F);
        p73.x_SET(-1429656978);
        p73.y_SET(-1237889431);
        p73.z_SET(-2.8671662E38F);
        p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
        LoopBackDemoChannel.instance.send(p73); //===============================
        VFR_HUD p74 = LoopBackDemoChannel.instance.new_VFR_HUD();
        PH.setPack(p74);
        p74.airspeed_SET(1.1521986E38F);
        p74.groundspeed_SET(-1.3792257E38F);
        p74.heading_SET((short)19065);
        p74.throttle_SET((char)21027);
        p74.alt_SET(-1.89957E38F);
        p74.climb_SET(2.085688E38F);
        LoopBackDemoChannel.instance.send(p74); //===============================
        COMMAND_INT p75 = LoopBackDemoChannel.instance.new_COMMAND_INT();
        PH.setPack(p75);
        p75.target_system_SET((char)70);
        p75.target_component_SET((char)100);
        p75.frame_SET(MAV_FRAME.MAV_FRAME_MISSION);
        p75.command_SET(MAV_CMD.MAV_CMD_START_RX_PAIR);
        p75.current_SET((char)209);
        p75.autocontinue_SET((char)44);
        p75.param1_SET(1.2834607E38F);
        p75.param2_SET(1.4587235E38F);
        p75.param3_SET(-2.6855675E38F);
        p75.param4_SET(2.165886E38F);
        p75.x_SET(28790997);
        p75.y_SET(-1045404114);
        p75.z_SET(4.5786583E36F);
        LoopBackDemoChannel.instance.send(p75); //===============================
        COMMAND_LONG p76 = LoopBackDemoChannel.instance.new_COMMAND_LONG();
        PH.setPack(p76);
        p76.target_system_SET((char)195);
        p76.target_component_SET((char)147);
        p76.command_SET(MAV_CMD.MAV_CMD_NAV_VTOL_LAND);
        p76.confirmation_SET((char)134);
        p76.param1_SET(1.173554E38F);
        p76.param2_SET(3.3000111E38F);
        p76.param3_SET(3.2332684E38F);
        p76.param4_SET(8.1899467E37F);
        p76.param5_SET(5.002098E37F);
        p76.param6_SET(5.251714E37F);
        p76.param7_SET(2.0108697E38F);
        LoopBackDemoChannel.instance.send(p76); //===============================
        COMMAND_ACK p77 = LoopBackDemoChannel.instance.new_COMMAND_ACK();
        PH.setPack(p77);
        p77.command_SET(MAV_CMD.MAV_CMD_WAYPOINT_USER_1);
        p77.result_SET(MAV_RESULT.MAV_RESULT_ACCEPTED);
        p77.progress_SET((char)108, PH);
        p77.result_param2_SET(-544413728, PH);
        p77.target_system_SET((char)243, PH);
        p77.target_component_SET((char)37, PH);
        LoopBackDemoChannel.instance.send(p77); //===============================
        MANUAL_SETPOINT p81 = LoopBackDemoChannel.instance.new_MANUAL_SETPOINT();
        PH.setPack(p81);
        p81.time_boot_ms_SET(3065887476L);
        p81.roll_SET(3.5718198E37F);
        p81.pitch_SET(2.1290238E38F);
        p81.yaw_SET(-2.9597388E37F);
        p81.thrust_SET(-2.1811866E38F);
        p81.mode_switch_SET((char)190);
        p81.manual_override_switch_SET((char)172);
        LoopBackDemoChannel.instance.send(p81); //===============================
        SET_ATTITUDE_TARGET p82 = LoopBackDemoChannel.instance.new_SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.time_boot_ms_SET(3800051670L);
        p82.target_system_SET((char)67);
        p82.target_component_SET((char)4);
        p82.type_mask_SET((char)160);
        p82.q_SET(new float[4], 0);
        p82.body_roll_rate_SET(2.0381637E38F);
        p82.body_pitch_rate_SET(1.6725925E37F);
        p82.body_yaw_rate_SET(-1.2830074E38F);
        p82.thrust_SET(-2.0193423E38F);
        LoopBackDemoChannel.instance.send(p82); //===============================
        ATTITUDE_TARGET p83 = LoopBackDemoChannel.instance.new_ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.time_boot_ms_SET(4222317910L);
        p83.type_mask_SET((char)224);
        p83.q_SET(new float[4], 0);
        p83.body_roll_rate_SET(-3.2671986E38F);
        p83.body_pitch_rate_SET(3.21004E38F);
        p83.body_yaw_rate_SET(2.3406136E38F);
        p83.thrust_SET(-6.708635E35F);
        LoopBackDemoChannel.instance.send(p83); //===============================
        SET_POSITION_TARGET_LOCAL_NED p84 = LoopBackDemoChannel.instance.new_SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.time_boot_ms_SET(2020612663L);
        p84.target_system_SET((char)251);
        p84.target_component_SET((char)178);
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
        p84.type_mask_SET((char)45294);
        p84.x_SET(1.4926759E38F);
        p84.y_SET(1.4944462E37F);
        p84.z_SET(-1.769493E38F);
        p84.vx_SET(1.214568E38F);
        p84.vy_SET(1.3988647E37F);
        p84.vz_SET(-3.2714703E38F);
        p84.afx_SET(-4.6341604E37F);
        p84.afy_SET(2.8035286E38F);
        p84.afz_SET(-1.7001826E38F);
        p84.yaw_SET(-1.4671305E37F);
        p84.yaw_rate_SET(1.915584E38F);
        LoopBackDemoChannel.instance.send(p84); //===============================
        SET_POSITION_TARGET_GLOBAL_INT p86 = LoopBackDemoChannel.instance.new_SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.time_boot_ms_SET(653648643L);
        p86.target_system_SET((char)144);
        p86.target_component_SET((char)98);
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
        p86.type_mask_SET((char)32812);
        p86.lat_int_SET(-122975714);
        p86.lon_int_SET(431181022);
        p86.alt_SET(2.6571346E38F);
        p86.vx_SET(-2.2762618E38F);
        p86.vy_SET(1.470751E36F);
        p86.vz_SET(2.8650572E37F);
        p86.afx_SET(1.2463515E38F);
        p86.afy_SET(-3.0115707E38F);
        p86.afz_SET(1.6046985E37F);
        p86.yaw_SET(1.6943584E37F);
        p86.yaw_rate_SET(2.453871E38F);
        LoopBackDemoChannel.instance.send(p86); //===============================
        POSITION_TARGET_GLOBAL_INT p87 = LoopBackDemoChannel.instance.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.time_boot_ms_SET(1564458232L);
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_INT);
        p87.type_mask_SET((char)44368);
        p87.lat_int_SET(938146286);
        p87.lon_int_SET(-1312891286);
        p87.alt_SET(7.381849E37F);
        p87.vx_SET(-2.2493182E38F);
        p87.vy_SET(-6.8021857E37F);
        p87.vz_SET(-1.3188695E38F);
        p87.afx_SET(1.345403E38F);
        p87.afy_SET(8.883469E36F);
        p87.afz_SET(5.4031426E37F);
        p87.yaw_SET(-1.9553981E38F);
        p87.yaw_rate_SET(-3.2285933E38F);
        LoopBackDemoChannel.instance.send(p87); //===============================
        LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = LoopBackDemoChannel.instance.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.time_boot_ms_SET(2309703757L);
        p89.x_SET(1.5370791E38F);
        p89.y_SET(-2.8488776E38F);
        p89.z_SET(1.1920778E38F);
        p89.roll_SET(-1.3139529E38F);
        p89.pitch_SET(9.070389E37F);
        p89.yaw_SET(6.9011355E37F);
        LoopBackDemoChannel.instance.send(p89); //===============================
        HIL_STATE p90 = LoopBackDemoChannel.instance.new_HIL_STATE();
        PH.setPack(p90);
        p90.time_usec_SET(2825164873045589509L);
        p90.roll_SET(-1.6652626E38F);
        p90.pitch_SET(2.966154E38F);
        p90.yaw_SET(1.225897E38F);
        p90.rollspeed_SET(3.339371E38F);
        p90.pitchspeed_SET(-2.3432033E38F);
        p90.yawspeed_SET(-1.416769E38F);
        p90.lat_SET(-311225211);
        p90.lon_SET(604956652);
        p90.alt_SET(2015734340);
        p90.vx_SET((short)32429);
        p90.vy_SET((short)30781);
        p90.vz_SET((short)26812);
        p90.xacc_SET((short)4133);
        p90.yacc_SET((short) -26301);
        p90.zacc_SET((short) -28478);
        LoopBackDemoChannel.instance.send(p90); //===============================
        HIL_CONTROLS p91 = LoopBackDemoChannel.instance.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.time_usec_SET(7907966596844656389L);
        p91.roll_ailerons_SET(2.1433996E38F);
        p91.pitch_elevator_SET(-1.3876399E38F);
        p91.yaw_rudder_SET(2.5557698E38F);
        p91.throttle_SET(-2.0170899E38F);
        p91.aux1_SET(1.1290162E38F);
        p91.aux2_SET(-3.303604E38F);
        p91.aux3_SET(9.283661E37F);
        p91.aux4_SET(-2.9487642E38F);
        p91.mode_SET(MAV_MODE.MAV_MODE_MANUAL_ARMED);
        p91.nav_mode_SET((char)136);
        LoopBackDemoChannel.instance.send(p91); //===============================
        HIL_RC_INPUTS_RAW p92 = LoopBackDemoChannel.instance.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.time_usec_SET(5814683641247034681L);
        p92.chan1_raw_SET((char)63293);
        p92.chan2_raw_SET((char)3772);
        p92.chan3_raw_SET((char)49878);
        p92.chan4_raw_SET((char)64039);
        p92.chan5_raw_SET((char)33201);
        p92.chan6_raw_SET((char)46849);
        p92.chan7_raw_SET((char)8530);
        p92.chan8_raw_SET((char)64060);
        p92.chan9_raw_SET((char)2168);
        p92.chan10_raw_SET((char)47700);
        p92.chan11_raw_SET((char)8407);
        p92.chan12_raw_SET((char)28694);
        p92.rssi_SET((char)114);
        LoopBackDemoChannel.instance.send(p92); //===============================
        HIL_ACTUATOR_CONTROLS p93 = LoopBackDemoChannel.instance.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.time_usec_SET(2224368930660463233L);
        p93.controls_SET(new float[16], 0);
        p93.mode_SET(MAV_MODE.MAV_MODE_GUIDED_ARMED);
        p93.flags_SET(4347719254190214585L);
        LoopBackDemoChannel.instance.send(p93); //===============================
        OPTICAL_FLOW p100 = LoopBackDemoChannel.instance.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.time_usec_SET(1182331772548347359L);
        p100.sensor_id_SET((char)184);
        p100.flow_x_SET((short) -14278);
        p100.flow_y_SET((short)15073);
        p100.flow_comp_m_x_SET(-2.3088102E38F);
        p100.flow_comp_m_y_SET(-3.0661474E37F);
        p100.quality_SET((char)174);
        p100.ground_distance_SET(-1.8015414E38F);
        p100.flow_rate_x_SET(-2.9090636E38F, PH);
        p100.flow_rate_y_SET(2.7963431E38F, PH);
        LoopBackDemoChannel.instance.send(p100); //===============================
        GLOBAL_VISION_POSITION_ESTIMATE p101 = LoopBackDemoChannel.instance.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.usec_SET(3927136452743808710L);
        p101.x_SET(-2.0139508E38F);
        p101.y_SET(-2.486425E38F);
        p101.z_SET(2.7315315E37F);
        p101.roll_SET(-1.2824328E38F);
        p101.pitch_SET(-9.755905E37F);
        p101.yaw_SET(-2.6887257E38F);
        LoopBackDemoChannel.instance.send(p101); //===============================
        VISION_POSITION_ESTIMATE p102 = LoopBackDemoChannel.instance.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.usec_SET(6884511941835215032L);
        p102.x_SET(1.7945596E38F);
        p102.y_SET(-9.232829E37F);
        p102.z_SET(2.4675016E38F);
        p102.roll_SET(-1.6507565E38F);
        p102.pitch_SET(-7.193674E37F);
        p102.yaw_SET(-1.2937132E38F);
        LoopBackDemoChannel.instance.send(p102); //===============================
        VISION_SPEED_ESTIMATE p103 = LoopBackDemoChannel.instance.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.usec_SET(7959216395133687443L);
        p103.x_SET(1.4598596E38F);
        p103.y_SET(2.619373E38F);
        p103.z_SET(-1.832253E37F);
        LoopBackDemoChannel.instance.send(p103); //===============================
        VICON_POSITION_ESTIMATE p104 = LoopBackDemoChannel.instance.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.usec_SET(5158057090452063119L);
        p104.x_SET(-3.2742847E38F);
        p104.y_SET(5.9200693E37F);
        p104.z_SET(1.0224787E38F);
        p104.roll_SET(-3.2493864E38F);
        p104.pitch_SET(-9.9450745E36F);
        p104.yaw_SET(-1.8720589E37F);
        LoopBackDemoChannel.instance.send(p104); //===============================
        HIGHRES_IMU p105 = LoopBackDemoChannel.instance.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.time_usec_SET(3064044848865674340L);
        p105.xacc_SET(-6.618199E37F);
        p105.yacc_SET(2.6929225E36F);
        p105.zacc_SET(1.8897754E38F);
        p105.xgyro_SET(1.0302731E38F);
        p105.ygyro_SET(3.2296383E38F);
        p105.zgyro_SET(1.825904E38F);
        p105.xmag_SET(-1.0863132E38F);
        p105.ymag_SET(2.9229698E38F);
        p105.zmag_SET(8.200036E37F);
        p105.abs_pressure_SET(1.5641269E38F);
        p105.diff_pressure_SET(-6.9808717E37F);
        p105.pressure_alt_SET(-2.2717992E38F);
        p105.temperature_SET(1.3443028E38F);
        p105.fields_updated_SET((char)60757);
        LoopBackDemoChannel.instance.send(p105); //===============================
        OPTICAL_FLOW_RAD p106 = LoopBackDemoChannel.instance.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.time_usec_SET(639281847193974412L);
        p106.sensor_id_SET((char)239);
        p106.integration_time_us_SET(2719106032L);
        p106.integrated_x_SET(6.571935E37F);
        p106.integrated_y_SET(-1.3667489E38F);
        p106.integrated_xgyro_SET(-1.4088364E38F);
        p106.integrated_ygyro_SET(-4.1228493E37F);
        p106.integrated_zgyro_SET(-2.474412E38F);
        p106.temperature_SET((short)9558);
        p106.quality_SET((char)11);
        p106.time_delta_distance_us_SET(1629139408L);
        p106.distance_SET(-3.3929868E37F);
        LoopBackDemoChannel.instance.send(p106); //===============================
        HIL_SENSOR p107 = LoopBackDemoChannel.instance.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.time_usec_SET(391019765211347609L);
        p107.xacc_SET(8.128346E36F);
        p107.yacc_SET(2.4706948E38F);
        p107.zacc_SET(-9.386536E37F);
        p107.xgyro_SET(1.3942006E38F);
        p107.ygyro_SET(3.111145E38F);
        p107.zgyro_SET(-2.335269E38F);
        p107.xmag_SET(2.196789E38F);
        p107.ymag_SET(-2.9083282E38F);
        p107.zmag_SET(1.1963768E38F);
        p107.abs_pressure_SET(3.3802E38F);
        p107.diff_pressure_SET(1.7928176E38F);
        p107.pressure_alt_SET(-9.883794E37F);
        p107.temperature_SET(-4.5936925E37F);
        p107.fields_updated_SET(2075924048L);
        LoopBackDemoChannel.instance.send(p107); //===============================
        SIM_STATE p108 = LoopBackDemoChannel.instance.new_SIM_STATE();
        PH.setPack(p108);
        p108.q1_SET(-4.0178125E37F);
        p108.q2_SET(9.340928E37F);
        p108.q3_SET(-2.144144E38F);
        p108.q4_SET(2.603683E38F);
        p108.roll_SET(-1.8755396E38F);
        p108.pitch_SET(-2.5542302E38F);
        p108.yaw_SET(6.4249725E37F);
        p108.xacc_SET(3.2337897E38F);
        p108.yacc_SET(2.6550447E38F);
        p108.zacc_SET(7.267761E37F);
        p108.xgyro_SET(1.9143926E38F);
        p108.ygyro_SET(2.575961E38F);
        p108.zgyro_SET(-2.3047186E38F);
        p108.lat_SET(-3.2659046E38F);
        p108.lon_SET(1.2235508E38F);
        p108.alt_SET(1.9256479E38F);
        p108.std_dev_horz_SET(1.9657746E38F);
        p108.std_dev_vert_SET(-5.1071954E37F);
        p108.vn_SET(2.2442515E38F);
        p108.ve_SET(4.7191655E37F);
        p108.vd_SET(-2.327138E38F);
        LoopBackDemoChannel.instance.send(p108); //===============================
        RADIO_STATUS p109 = LoopBackDemoChannel.instance.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.rssi_SET((char)180);
        p109.remrssi_SET((char)154);
        p109.txbuf_SET((char)210);
        p109.noise_SET((char)87);
        p109.remnoise_SET((char)221);
        p109.rxerrors_SET((char)9629);
        p109.fixed__SET((char)49385);
        LoopBackDemoChannel.instance.send(p109); //===============================
        FILE_TRANSFER_PROTOCOL p110 = LoopBackDemoChannel.instance.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.target_network_SET((char)96);
        p110.target_system_SET((char)239);
        p110.target_component_SET((char)135);
        p110.payload_SET(new char[251], 0);
        LoopBackDemoChannel.instance.send(p110); //===============================
        TIMESYNC p111 = LoopBackDemoChannel.instance.new_TIMESYNC();
        PH.setPack(p111);
        p111.tc1_SET(3462914207084844849L);
        p111.ts1_SET(-7120963690915149843L);
        LoopBackDemoChannel.instance.send(p111); //===============================
        CAMERA_TRIGGER p112 = LoopBackDemoChannel.instance.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.time_usec_SET(2054431275844733627L);
        p112.seq_SET(1106510390L);
        LoopBackDemoChannel.instance.send(p112); //===============================
        HIL_GPS p113 = LoopBackDemoChannel.instance.new_HIL_GPS();
        PH.setPack(p113);
        p113.time_usec_SET(8031624974968833732L);
        p113.fix_type_SET((char)135);
        p113.lat_SET(2087251504);
        p113.lon_SET(-1329747418);
        p113.alt_SET(1912265233);
        p113.eph_SET((char)61706);
        p113.epv_SET((char)49237);
        p113.vel_SET((char)47045);
        p113.vn_SET((short)10907);
        p113.ve_SET((short) -32377);
        p113.vd_SET((short)21179);
        p113.cog_SET((char)31006);
        p113.satellites_visible_SET((char)31);
        LoopBackDemoChannel.instance.send(p113); //===============================
        HIL_OPTICAL_FLOW p114 = LoopBackDemoChannel.instance.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.time_usec_SET(4949241267029277531L);
        p114.sensor_id_SET((char)146);
        p114.integration_time_us_SET(3288194449L);
        p114.integrated_x_SET(3.2068477E38F);
        p114.integrated_y_SET(-9.1381687E36F);
        p114.integrated_xgyro_SET(-2.1724173E38F);
        p114.integrated_ygyro_SET(1.0872674E38F);
        p114.integrated_zgyro_SET(-1.5790351E38F);
        p114.temperature_SET((short) -15756);
        p114.quality_SET((char)158);
        p114.time_delta_distance_us_SET(3211248475L);
        p114.distance_SET(4.7829426E37F);
        LoopBackDemoChannel.instance.send(p114); //===============================
        HIL_STATE_QUATERNION p115 = LoopBackDemoChannel.instance.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.time_usec_SET(6763134908773523532L);
        p115.attitude_quaternion_SET(new float[4], 0);
        p115.rollspeed_SET(8.974356E37F);
        p115.pitchspeed_SET(-1.0202791E38F);
        p115.yawspeed_SET(-2.8016462E38F);
        p115.lat_SET(-1184140048);
        p115.lon_SET(1588898724);
        p115.alt_SET(127104971);
        p115.vx_SET((short) -20955);
        p115.vy_SET((short) -18376);
        p115.vz_SET((short)10057);
        p115.ind_airspeed_SET((char)39565);
        p115.true_airspeed_SET((char)52721);
        p115.xacc_SET((short) -13253);
        p115.yacc_SET((short)23182);
        p115.zacc_SET((short) -25052);
        LoopBackDemoChannel.instance.send(p115); //===============================
        SCALED_IMU2 p116 = LoopBackDemoChannel.instance.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.time_boot_ms_SET(3256374584L);
        p116.xacc_SET((short) -23420);
        p116.yacc_SET((short) -23707);
        p116.zacc_SET((short) -14178);
        p116.xgyro_SET((short)24206);
        p116.ygyro_SET((short) -19230);
        p116.zgyro_SET((short) -10692);
        p116.xmag_SET((short) -8210);
        p116.ymag_SET((short)2692);
        p116.zmag_SET((short) -26425);
        LoopBackDemoChannel.instance.send(p116); //===============================
        LOG_REQUEST_LIST p117 = LoopBackDemoChannel.instance.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.target_system_SET((char)228);
        p117.target_component_SET((char)108);
        p117.start_SET((char)36808);
        p117.end_SET((char)54240);
        LoopBackDemoChannel.instance.send(p117); //===============================
        LOG_ENTRY p118 = LoopBackDemoChannel.instance.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.id_SET((char)46758);
        p118.num_logs_SET((char)38333);
        p118.last_log_num_SET((char)64871);
        p118.time_utc_SET(555252250L);
        p118.size_SET(3783538612L);
        LoopBackDemoChannel.instance.send(p118); //===============================
        LOG_REQUEST_DATA p119 = LoopBackDemoChannel.instance.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.target_system_SET((char)75);
        p119.target_component_SET((char)21);
        p119.id_SET((char)45956);
        p119.ofs_SET(1533999472L);
        p119.count_SET(147629451L);
        LoopBackDemoChannel.instance.send(p119); //===============================
        LOG_DATA p120 = LoopBackDemoChannel.instance.new_LOG_DATA();
        PH.setPack(p120);
        p120.id_SET((char)63676);
        p120.ofs_SET(2154572165L);
        p120.count_SET((char)43);
        p120.data__SET(new char[90], 0);
        LoopBackDemoChannel.instance.send(p120); //===============================
        LOG_ERASE p121 = LoopBackDemoChannel.instance.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_system_SET((char)149);
        p121.target_component_SET((char)89);
        LoopBackDemoChannel.instance.send(p121); //===============================
        LOG_REQUEST_END p122 = LoopBackDemoChannel.instance.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_system_SET((char)93);
        p122.target_component_SET((char)79);
        LoopBackDemoChannel.instance.send(p122); //===============================
        GPS_INJECT_DATA p123 = LoopBackDemoChannel.instance.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.target_system_SET((char)120);
        p123.target_component_SET((char)70);
        p123.len_SET((char)138);
        p123.data__SET(new char[110], 0);
        LoopBackDemoChannel.instance.send(p123); //===============================
        GPS2_RAW p124 = LoopBackDemoChannel.instance.new_GPS2_RAW();
        PH.setPack(p124);
        p124.time_usec_SET(790505309851565776L);
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS);
        p124.lat_SET(808521496);
        p124.lon_SET(1550693448);
        p124.alt_SET(1041943672);
        p124.eph_SET((char)52028);
        p124.epv_SET((char)5019);
        p124.vel_SET((char)13180);
        p124.cog_SET((char)18611);
        p124.satellites_visible_SET((char)106);
        p124.dgps_numch_SET((char)51);
        p124.dgps_age_SET(1569616401L);
        LoopBackDemoChannel.instance.send(p124); //===============================
        POWER_STATUS p125 = LoopBackDemoChannel.instance.new_POWER_STATUS();
        PH.setPack(p125);
        p125.Vcc_SET((char)55617);
        p125.Vservo_SET((char)12800);
        p125.flags_SET(MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID);
        LoopBackDemoChannel.instance.send(p125); //===============================
        SERIAL_CONTROL p126 = LoopBackDemoChannel.instance.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_SHELL);
        p126.flags_SET(SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE);
        p126.timeout_SET((char)11302);
        p126.baudrate_SET(1681968179L);
        p126.count_SET((char)51);
        p126.data__SET(new char[70], 0);
        LoopBackDemoChannel.instance.send(p126); //===============================
        GPS_RTK p127 = LoopBackDemoChannel.instance.new_GPS_RTK();
        PH.setPack(p127);
        p127.time_last_baseline_ms_SET(96511094L);
        p127.rtk_receiver_id_SET((char)218);
        p127.wn_SET((char)28141);
        p127.tow_SET(3492888843L);
        p127.rtk_health_SET((char)97);
        p127.rtk_rate_SET((char)233);
        p127.nsats_SET((char)213);
        p127.baseline_coords_type_SET((char)53);
        p127.baseline_a_mm_SET(-1431388014);
        p127.baseline_b_mm_SET(-152715539);
        p127.baseline_c_mm_SET(-2053739111);
        p127.accuracy_SET(521671477L);
        p127.iar_num_hypotheses_SET(-310989992);
        LoopBackDemoChannel.instance.send(p127); //===============================
        GPS2_RTK p128 = LoopBackDemoChannel.instance.new_GPS2_RTK();
        PH.setPack(p128);
        p128.time_last_baseline_ms_SET(3389070760L);
        p128.rtk_receiver_id_SET((char)117);
        p128.wn_SET((char)46207);
        p128.tow_SET(1331530648L);
        p128.rtk_health_SET((char)237);
        p128.rtk_rate_SET((char)82);
        p128.nsats_SET((char)13);
        p128.baseline_coords_type_SET((char)19);
        p128.baseline_a_mm_SET(-302579294);
        p128.baseline_b_mm_SET(1476304346);
        p128.baseline_c_mm_SET(441159005);
        p128.accuracy_SET(3697725192L);
        p128.iar_num_hypotheses_SET(-205429681);
        LoopBackDemoChannel.instance.send(p128); //===============================
        SCALED_IMU3 p129 = LoopBackDemoChannel.instance.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.time_boot_ms_SET(4209103192L);
        p129.xacc_SET((short) -14007);
        p129.yacc_SET((short) -12105);
        p129.zacc_SET((short)28966);
        p129.xgyro_SET((short) -32719);
        p129.ygyro_SET((short)2462);
        p129.zgyro_SET((short)32755);
        p129.xmag_SET((short) -28158);
        p129.ymag_SET((short) -20040);
        p129.zmag_SET((short)23268);
        LoopBackDemoChannel.instance.send(p129); //===============================
        DATA_TRANSMISSION_HANDSHAKE p130 = LoopBackDemoChannel.instance.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.type_SET((char)219);
        p130.size_SET(1116713504L);
        p130.width_SET((char)32768);
        p130.height_SET((char)11115);
        p130.packets_SET((char)32977);
        p130.payload_SET((char)11);
        p130.jpg_quality_SET((char)206);
        LoopBackDemoChannel.instance.send(p130); //===============================
        ENCAPSULATED_DATA p131 = LoopBackDemoChannel.instance.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.seqnr_SET((char)7377);
        p131.data__SET(new char[253], 0);
        LoopBackDemoChannel.instance.send(p131); //===============================
        DISTANCE_SENSOR p132 = LoopBackDemoChannel.instance.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.time_boot_ms_SET(1369792716L);
        p132.min_distance_SET((char)64120);
        p132.max_distance_SET((char)17242);
        p132.current_distance_SET((char)41089);
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED);
        p132.id_SET((char)28);
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_90);
        p132.covariance_SET((char)226);
        LoopBackDemoChannel.instance.send(p132); //===============================
        TERRAIN_REQUEST p133 = LoopBackDemoChannel.instance.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.lat_SET(1417959995);
        p133.lon_SET(250948196);
        p133.grid_spacing_SET((char)12202);
        p133.mask_SET(5089889571799683940L);
        LoopBackDemoChannel.instance.send(p133); //===============================
        TERRAIN_DATA p134 = LoopBackDemoChannel.instance.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.lat_SET(388516988);
        p134.lon_SET(-283086945);
        p134.grid_spacing_SET((char)27048);
        p134.gridbit_SET((char)117);
        p134.data__SET(new short[16], 0);
        LoopBackDemoChannel.instance.send(p134); //===============================
        TERRAIN_CHECK p135 = LoopBackDemoChannel.instance.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lat_SET(-1811012325);
        p135.lon_SET(1045026751);
        LoopBackDemoChannel.instance.send(p135); //===============================
        TERRAIN_REPORT p136 = LoopBackDemoChannel.instance.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.lat_SET(771319360);
        p136.lon_SET(-1703402973);
        p136.spacing_SET((char)38001);
        p136.terrain_height_SET(3.381287E38F);
        p136.current_height_SET(1.4620686E38F);
        p136.pending_SET((char)23360);
        p136.loaded_SET((char)34715);
        LoopBackDemoChannel.instance.send(p136); //===============================
        SCALED_PRESSURE2 p137 = LoopBackDemoChannel.instance.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.time_boot_ms_SET(159632651L);
        p137.press_abs_SET(5.942629E37F);
        p137.press_diff_SET(1.5232741E38F);
        p137.temperature_SET((short) -30196);
        LoopBackDemoChannel.instance.send(p137); //===============================
        ATT_POS_MOCAP p138 = LoopBackDemoChannel.instance.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.time_usec_SET(5269382030873864313L);
        p138.q_SET(new float[4], 0);
        p138.x_SET(1.0226132E38F);
        p138.y_SET(-1.5608213E38F);
        p138.z_SET(1.3869303E38F);
        LoopBackDemoChannel.instance.send(p138); //===============================
        SET_ACTUATOR_CONTROL_TARGET p139 = LoopBackDemoChannel.instance.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.time_usec_SET(8967130234052280466L);
        p139.group_mlx_SET((char)184);
        p139.target_system_SET((char)219);
        p139.target_component_SET((char)112);
        p139.controls_SET(new float[8], 0);
        LoopBackDemoChannel.instance.send(p139); //===============================
        ACTUATOR_CONTROL_TARGET p140 = LoopBackDemoChannel.instance.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.time_usec_SET(110551363597803043L);
        p140.group_mlx_SET((char)118);
        p140.controls_SET(new float[8], 0);
        LoopBackDemoChannel.instance.send(p140); //===============================
        ALTITUDE p141 = LoopBackDemoChannel.instance.new_ALTITUDE();
        PH.setPack(p141);
        p141.time_usec_SET(6971052558978169073L);
        p141.altitude_monotonic_SET(7.059443E37F);
        p141.altitude_amsl_SET(1.3338981E38F);
        p141.altitude_local_SET(-1.8783032E38F);
        p141.altitude_relative_SET(2.5616944E37F);
        p141.altitude_terrain_SET(1.1874773E37F);
        p141.bottom_clearance_SET(-2.3315581E38F);
        LoopBackDemoChannel.instance.send(p141); //===============================
        RESOURCE_REQUEST p142 = LoopBackDemoChannel.instance.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.request_id_SET((char)40);
        p142.uri_type_SET((char)92);
        p142.uri_SET(new char[120], 0);
        p142.transfer_type_SET((char)0);
        p142.storage_SET(new char[120], 0);
        LoopBackDemoChannel.instance.send(p142); //===============================
        SCALED_PRESSURE3 p143 = LoopBackDemoChannel.instance.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.time_boot_ms_SET(757464392L);
        p143.press_abs_SET(2.7289978E38F);
        p143.press_diff_SET(-3.3063567E38F);
        p143.temperature_SET((short)3543);
        LoopBackDemoChannel.instance.send(p143); //===============================
        FOLLOW_TARGET p144 = LoopBackDemoChannel.instance.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.timestamp_SET(2671959005883587695L);
        p144.est_capabilities_SET((char)160);
        p144.lat_SET(-1199316599);
        p144.lon_SET(212801345);
        p144.alt_SET(4.5028557E37F);
        p144.vel_SET(new float[3], 0);
        p144.acc_SET(new float[3], 0);
        p144.attitude_q_SET(new float[4], 0);
        p144.rates_SET(new float[3], 0);
        p144.position_cov_SET(new float[3], 0);
        p144.custom_state_SET(7865323592934271278L);
        LoopBackDemoChannel.instance.send(p144); //===============================
        CONTROL_SYSTEM_STATE p146 = LoopBackDemoChannel.instance.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.time_usec_SET(8157953117536730436L);
        p146.x_acc_SET(8.2694695E37F);
        p146.y_acc_SET(-4.489388E37F);
        p146.z_acc_SET(3.2450669E38F);
        p146.x_vel_SET(1.8595883E38F);
        p146.y_vel_SET(3.2340282E38F);
        p146.z_vel_SET(1.1687567E38F);
        p146.x_pos_SET(-1.2168739E38F);
        p146.y_pos_SET(1.3222584E38F);
        p146.z_pos_SET(1.3306837E38F);
        p146.airspeed_SET(2.4679314E38F);
        p146.vel_variance_SET(new float[3], 0);
        p146.pos_variance_SET(new float[3], 0);
        p146.q_SET(new float[4], 0);
        p146.roll_rate_SET(1.2441113E38F);
        p146.pitch_rate_SET(3.7612116E37F);
        p146.yaw_rate_SET(-1.2643338E38F);
        LoopBackDemoChannel.instance.send(p146); //===============================
        BATTERY_STATUS p147 = LoopBackDemoChannel.instance.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.id_SET((char)7);
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_ALL);
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIPO);
        p147.temperature_SET((short) -15405);
        p147.voltages_SET(new char[10], 0);
        p147.current_battery_SET((short) -8118);
        p147.current_consumed_SET(-1717746569);
        p147.energy_consumed_SET(-590640773);
        p147.battery_remaining_SET((byte) - 15);
        LoopBackDemoChannel.instance.send(p147); //===============================
        AUTOPILOT_VERSION p148 = LoopBackDemoChannel.instance.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.capabilities_SET(MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE);
        p148.flight_sw_version_SET(1768802233L);
        p148.middleware_sw_version_SET(115883403L);
        p148.os_sw_version_SET(1734170851L);
        p148.board_version_SET(2974726925L);
        p148.flight_custom_version_SET(new char[8], 0);
        p148.middleware_custom_version_SET(new char[8], 0);
        p148.os_custom_version_SET(new char[8], 0);
        p148.vendor_id_SET((char)6137);
        p148.product_id_SET((char)23849);
        p148.uid_SET(761600870282735637L);
        p148.uid2_SET(new char[18], 0, PH);
        LoopBackDemoChannel.instance.send(p148); //===============================
        LANDING_TARGET p149 = LoopBackDemoChannel.instance.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.time_usec_SET(3389827764886121364L);
        p149.target_num_SET((char)30);
        p149.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_INT);
        p149.angle_x_SET(2.5241037E38F);
        p149.angle_y_SET(-2.40255E38F);
        p149.distance_SET(2.2043878E38F);
        p149.size_x_SET(-2.1559771E38F);
        p149.size_y_SET(5.735546E37F);
        p149.x_SET(1.6251366E38F, PH);
        p149.y_SET(8.446588E37F, PH);
        p149.z_SET(-2.0118556E38F, PH);
        p149.q_SET(new float[4], 0, PH);
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_RADIO_BEACON);
        p149.position_valid_SET((char)142, PH);
        LoopBackDemoChannel.instance.send(p149); //===============================
        FLEXIFUNCTION_SET p150 = LoopBackDemoChannel.instance.new_FLEXIFUNCTION_SET();
        PH.setPack(p150);
        p150.target_system_SET((char)15);
        p150.target_component_SET((char)254);
        LoopBackDemoChannel.instance.send(p150); //===============================
        FLEXIFUNCTION_READ_REQ p151 = LoopBackDemoChannel.instance.new_FLEXIFUNCTION_READ_REQ();
        PH.setPack(p151);
        p151.target_system_SET((char)29);
        p151.target_component_SET((char)105);
        p151.read_req_type_SET((short)22749);
        p151.data_index_SET((short)29077);
        LoopBackDemoChannel.instance.send(p151); //===============================
        FLEXIFUNCTION_BUFFER_FUNCTION p152 = LoopBackDemoChannel.instance.new_FLEXIFUNCTION_BUFFER_FUNCTION();
        PH.setPack(p152);
        p152.target_system_SET((char)102);
        p152.target_component_SET((char)110);
        p152.func_index_SET((char)44524);
        p152.func_count_SET((char)51522);
        p152.data_address_SET((char)53783);
        p152.data_size_SET((char)58001);
        p152.data__SET(new byte[48], 0);
        LoopBackDemoChannel.instance.send(p152); //===============================
        FLEXIFUNCTION_BUFFER_FUNCTION_ACK p153 = LoopBackDemoChannel.instance.new_FLEXIFUNCTION_BUFFER_FUNCTION_ACK();
        PH.setPack(p153);
        p153.target_system_SET((char)224);
        p153.target_component_SET((char)98);
        p153.func_index_SET((char)58746);
        p153.result_SET((char)39227);
        LoopBackDemoChannel.instance.send(p153); //===============================
        FLEXIFUNCTION_DIRECTORY p155 = LoopBackDemoChannel.instance.new_FLEXIFUNCTION_DIRECTORY();
        PH.setPack(p155);
        p155.target_system_SET((char)247);
        p155.target_component_SET((char)92);
        p155.directory_type_SET((char)110);
        p155.start_index_SET((char)252);
        p155.count_SET((char)178);
        p155.directory_data_SET(new byte[48], 0);
        LoopBackDemoChannel.instance.send(p155); //===============================
        FLEXIFUNCTION_DIRECTORY_ACK p156 = LoopBackDemoChannel.instance.new_FLEXIFUNCTION_DIRECTORY_ACK();
        PH.setPack(p156);
        p156.target_system_SET((char)155);
        p156.target_component_SET((char)0);
        p156.directory_type_SET((char)140);
        p156.start_index_SET((char)47);
        p156.count_SET((char)212);
        p156.result_SET((char)43654);
        LoopBackDemoChannel.instance.send(p156); //===============================
        FLEXIFUNCTION_COMMAND p157 = LoopBackDemoChannel.instance.new_FLEXIFUNCTION_COMMAND();
        PH.setPack(p157);
        p157.target_system_SET((char)178);
        p157.target_component_SET((char)51);
        p157.command_type_SET((char)101);
        LoopBackDemoChannel.instance.send(p157); //===============================
        FLEXIFUNCTION_COMMAND_ACK p158 = LoopBackDemoChannel.instance.new_FLEXIFUNCTION_COMMAND_ACK();
        PH.setPack(p158);
        p158.command_type_SET((char)17838);
        p158.result_SET((char)22388);
        LoopBackDemoChannel.instance.send(p158); //===============================
        SERIAL_UDB_EXTRA_F2_A p170 = LoopBackDemoChannel.instance.new_SERIAL_UDB_EXTRA_F2_A();
        PH.setPack(p170);
        p170.sue_time_SET(1485751633L);
        p170.sue_status_SET((char)65);
        p170.sue_latitude_SET(-285277950);
        p170.sue_longitude_SET(1763127112);
        p170.sue_altitude_SET(536930166);
        p170.sue_waypoint_index_SET((char)47080);
        p170.sue_rmat0_SET((short)29495);
        p170.sue_rmat1_SET((short)1060);
        p170.sue_rmat2_SET((short) -32123);
        p170.sue_rmat3_SET((short) -17366);
        p170.sue_rmat4_SET((short) -9903);
        p170.sue_rmat5_SET((short)7497);
        p170.sue_rmat6_SET((short)14839);
        p170.sue_rmat7_SET((short) -21928);
        p170.sue_rmat8_SET((short)1277);
        p170.sue_cog_SET((char)4964);
        p170.sue_sog_SET((short)18917);
        p170.sue_cpu_load_SET((char)39536);
        p170.sue_air_speed_3DIMU_SET((char)11717);
        p170.sue_estimated_wind_0_SET((short)17309);
        p170.sue_estimated_wind_1_SET((short)12226);
        p170.sue_estimated_wind_2_SET((short)307);
        p170.sue_magFieldEarth0_SET((short)24097);
        p170.sue_magFieldEarth1_SET((short) -10714);
        p170.sue_magFieldEarth2_SET((short) -3915);
        p170.sue_svs_SET((short)29322);
        p170.sue_hdop_SET((short) -27969);
        LoopBackDemoChannel.instance.send(p170); //===============================
        SERIAL_UDB_EXTRA_F2_B p171 = LoopBackDemoChannel.instance.new_SERIAL_UDB_EXTRA_F2_B();
        PH.setPack(p171);
        p171.sue_time_SET(232693620L);
        p171.sue_pwm_input_1_SET((short)28886);
        p171.sue_pwm_input_2_SET((short)15200);
        p171.sue_pwm_input_3_SET((short) -16121);
        p171.sue_pwm_input_4_SET((short)15245);
        p171.sue_pwm_input_5_SET((short)28661);
        p171.sue_pwm_input_6_SET((short) -27901);
        p171.sue_pwm_input_7_SET((short) -13591);
        p171.sue_pwm_input_8_SET((short) -21707);
        p171.sue_pwm_input_9_SET((short)19600);
        p171.sue_pwm_input_10_SET((short)27652);
        p171.sue_pwm_input_11_SET((short) -17343);
        p171.sue_pwm_input_12_SET((short)19579);
        p171.sue_pwm_output_1_SET((short) -16184);
        p171.sue_pwm_output_2_SET((short)22900);
        p171.sue_pwm_output_3_SET((short)8778);
        p171.sue_pwm_output_4_SET((short) -15073);
        p171.sue_pwm_output_5_SET((short)24783);
        p171.sue_pwm_output_6_SET((short)12478);
        p171.sue_pwm_output_7_SET((short) -12438);
        p171.sue_pwm_output_8_SET((short) -30194);
        p171.sue_pwm_output_9_SET((short)9939);
        p171.sue_pwm_output_10_SET((short)15338);
        p171.sue_pwm_output_11_SET((short)181);
        p171.sue_pwm_output_12_SET((short) -1265);
        p171.sue_imu_location_x_SET((short)27190);
        p171.sue_imu_location_y_SET((short)18699);
        p171.sue_imu_location_z_SET((short)18675);
        p171.sue_location_error_earth_x_SET((short)13198);
        p171.sue_location_error_earth_y_SET((short) -8163);
        p171.sue_location_error_earth_z_SET((short) -24665);
        p171.sue_flags_SET(3611889297L);
        p171.sue_osc_fails_SET((short) -17946);
        p171.sue_imu_velocity_x_SET((short) -16907);
        p171.sue_imu_velocity_y_SET((short)17917);
        p171.sue_imu_velocity_z_SET((short) -20970);
        p171.sue_waypoint_goal_x_SET((short)14829);
        p171.sue_waypoint_goal_y_SET((short)25262);
        p171.sue_waypoint_goal_z_SET((short) -6631);
        p171.sue_aero_x_SET((short) -20896);
        p171.sue_aero_y_SET((short)22656);
        p171.sue_aero_z_SET((short) -6566);
        p171.sue_barom_temp_SET((short)26077);
        p171.sue_barom_press_SET(1704699626);
        p171.sue_barom_alt_SET(915696237);
        p171.sue_bat_volt_SET((short)21409);
        p171.sue_bat_amp_SET((short) -25080);
        p171.sue_bat_amp_hours_SET((short) -21668);
        p171.sue_desired_height_SET((short)7805);
        p171.sue_memory_stack_free_SET((short) -22951);
        LoopBackDemoChannel.instance.send(p171); //===============================
        SERIAL_UDB_EXTRA_F4 p172 = LoopBackDemoChannel.instance.new_SERIAL_UDB_EXTRA_F4();
        PH.setPack(p172);
        p172.sue_ROLL_STABILIZATION_AILERONS_SET((char)232);
        p172.sue_ROLL_STABILIZATION_RUDDER_SET((char)104);
        p172.sue_PITCH_STABILIZATION_SET((char)51);
        p172.sue_YAW_STABILIZATION_RUDDER_SET((char)109);
        p172.sue_YAW_STABILIZATION_AILERON_SET((char)96);
        p172.sue_AILERON_NAVIGATION_SET((char)207);
        p172.sue_RUDDER_NAVIGATION_SET((char)57);
        p172.sue_ALTITUDEHOLD_STABILIZED_SET((char)84);
        p172.sue_ALTITUDEHOLD_WAYPOINT_SET((char)82);
        p172.sue_RACING_MODE_SET((char)120);
        LoopBackDemoChannel.instance.send(p172); //===============================
        SERIAL_UDB_EXTRA_F5 p173 = LoopBackDemoChannel.instance.new_SERIAL_UDB_EXTRA_F5();
        PH.setPack(p173);
        p173.sue_YAWKP_AILERON_SET(2.9439041E38F);
        p173.sue_YAWKD_AILERON_SET(-3.3503784E38F);
        p173.sue_ROLLKP_SET(-2.0977873E38F);
        p173.sue_ROLLKD_SET(6.05027E36F);
        LoopBackDemoChannel.instance.send(p173); //===============================
        SERIAL_UDB_EXTRA_F6 p174 = LoopBackDemoChannel.instance.new_SERIAL_UDB_EXTRA_F6();
        PH.setPack(p174);
        p174.sue_PITCHGAIN_SET(3.4926758E37F);
        p174.sue_PITCHKD_SET(-2.5208445E38F);
        p174.sue_RUDDER_ELEV_MIX_SET(1.1228212E38F);
        p174.sue_ROLL_ELEV_MIX_SET(2.0467994E38F);
        p174.sue_ELEVATOR_BOOST_SET(-7.3812214E36F);
        LoopBackDemoChannel.instance.send(p174); //===============================
        SERIAL_UDB_EXTRA_F7 p175 = LoopBackDemoChannel.instance.new_SERIAL_UDB_EXTRA_F7();
        PH.setPack(p175);
        p175.sue_YAWKP_RUDDER_SET(-3.941175E36F);
        p175.sue_YAWKD_RUDDER_SET(-2.4320428E38F);
        p175.sue_ROLLKP_RUDDER_SET(-2.971671E38F);
        p175.sue_ROLLKD_RUDDER_SET(2.6167746E37F);
        p175.sue_RUDDER_BOOST_SET(6.3094256E37F);
        p175.sue_RTL_PITCH_DOWN_SET(-2.9938165E38F);
        LoopBackDemoChannel.instance.send(p175); //===============================
        SERIAL_UDB_EXTRA_F8 p176 = LoopBackDemoChannel.instance.new_SERIAL_UDB_EXTRA_F8();
        PH.setPack(p176);
        p176.sue_HEIGHT_TARGET_MAX_SET(-2.9789974E37F);
        p176.sue_HEIGHT_TARGET_MIN_SET(-3.9841427E37F);
        p176.sue_ALT_HOLD_THROTTLE_MIN_SET(-2.985774E38F);
        p176.sue_ALT_HOLD_THROTTLE_MAX_SET(9.639698E37F);
        p176.sue_ALT_HOLD_PITCH_MIN_SET(1.2952505E38F);
        p176.sue_ALT_HOLD_PITCH_MAX_SET(-1.6525929E38F);
        p176.sue_ALT_HOLD_PITCH_HIGH_SET(2.675752E37F);
        LoopBackDemoChannel.instance.send(p176); //===============================
        SERIAL_UDB_EXTRA_F13 p177 = LoopBackDemoChannel.instance.new_SERIAL_UDB_EXTRA_F13();
        PH.setPack(p177);
        p177.sue_week_no_SET((short) -15671);
        p177.sue_lat_origin_SET(-829461804);
        p177.sue_lon_origin_SET(2082977392);
        p177.sue_alt_origin_SET(-64252738);
        LoopBackDemoChannel.instance.send(p177); //===============================
        SERIAL_UDB_EXTRA_F14 p178 = LoopBackDemoChannel.instance.new_SERIAL_UDB_EXTRA_F14();
        PH.setPack(p178);
        p178.sue_WIND_ESTIMATION_SET((char)157);
        p178.sue_GPS_TYPE_SET((char)97);
        p178.sue_DR_SET((char)34);
        p178.sue_BOARD_TYPE_SET((char)160);
        p178.sue_AIRFRAME_SET((char)230);
        p178.sue_RCON_SET((short) -31437);
        p178.sue_TRAP_FLAGS_SET((short) -13343);
        p178.sue_TRAP_SOURCE_SET(353666041L);
        p178.sue_osc_fail_count_SET((short)6549);
        p178.sue_CLOCK_CONFIG_SET((char)61);
        p178.sue_FLIGHT_PLAN_TYPE_SET((char)80);
        LoopBackDemoChannel.instance.send(p178); //===============================
        SERIAL_UDB_EXTRA_F15 p179 = LoopBackDemoChannel.instance.new_SERIAL_UDB_EXTRA_F15();
        PH.setPack(p179);
        p179.sue_ID_VEHICLE_MODEL_NAME_SET(new char[40], 0);
        p179.sue_ID_VEHICLE_REGISTRATION_SET(new char[20], 0);
        LoopBackDemoChannel.instance.send(p179); //===============================
        SERIAL_UDB_EXTRA_F16 p180 = LoopBackDemoChannel.instance.new_SERIAL_UDB_EXTRA_F16();
        PH.setPack(p180);
        p180.sue_ID_LEAD_PILOT_SET(new char[40], 0);
        p180.sue_ID_DIY_DRONES_URL_SET(new char[70], 0);
        LoopBackDemoChannel.instance.send(p180); //===============================
        ALTITUDES p181 = LoopBackDemoChannel.instance.new_ALTITUDES();
        PH.setPack(p181);
        p181.time_boot_ms_SET(2490449531L);
        p181.alt_gps_SET(1967723304);
        p181.alt_imu_SET(1413146294);
        p181.alt_barometric_SET(1040699541);
        p181.alt_optical_flow_SET(-1764218379);
        p181.alt_range_finder_SET(-1982391151);
        p181.alt_extra_SET(-1692358757);
        LoopBackDemoChannel.instance.send(p181); //===============================
        AIRSPEEDS p182 = LoopBackDemoChannel.instance.new_AIRSPEEDS();
        PH.setPack(p182);
        p182.time_boot_ms_SET(2277683318L);
        p182.airspeed_imu_SET((short)15500);
        p182.airspeed_pitot_SET((short)21309);
        p182.airspeed_hot_wire_SET((short)28330);
        p182.airspeed_ultrasonic_SET((short)16732);
        p182.aoa_SET((short)7756);
        p182.aoy_SET((short)18567);
        LoopBackDemoChannel.instance.send(p182); //===============================
        SERIAL_UDB_EXTRA_F17 p183 = LoopBackDemoChannel.instance.new_SERIAL_UDB_EXTRA_F17();
        PH.setPack(p183);
        p183.sue_feed_forward_SET(-8.861728E36F);
        p183.sue_turn_rate_nav_SET(-1.3041741E38F);
        p183.sue_turn_rate_fbw_SET(1.513625E38F);
        LoopBackDemoChannel.instance.send(p183); //===============================
        SERIAL_UDB_EXTRA_F18 p184 = LoopBackDemoChannel.instance.new_SERIAL_UDB_EXTRA_F18();
        PH.setPack(p184);
        p184.angle_of_attack_normal_SET(-1.945438E38F);
        p184.angle_of_attack_inverted_SET(-4.931975E37F);
        p184.elevator_trim_normal_SET(-1.1339322E38F);
        p184.elevator_trim_inverted_SET(2.5448928E38F);
        p184.reference_speed_SET(-2.043891E38F);
        LoopBackDemoChannel.instance.send(p184); //===============================
        SERIAL_UDB_EXTRA_F19 p185 = LoopBackDemoChannel.instance.new_SERIAL_UDB_EXTRA_F19();
        PH.setPack(p185);
        p185.sue_aileron_output_channel_SET((char)18);
        p185.sue_aileron_reversed_SET((char)83);
        p185.sue_elevator_output_channel_SET((char)82);
        p185.sue_elevator_reversed_SET((char)146);
        p185.sue_throttle_output_channel_SET((char)39);
        p185.sue_throttle_reversed_SET((char)77);
        p185.sue_rudder_output_channel_SET((char)80);
        p185.sue_rudder_reversed_SET((char)0);
        LoopBackDemoChannel.instance.send(p185); //===============================
        SERIAL_UDB_EXTRA_F20 p186 = LoopBackDemoChannel.instance.new_SERIAL_UDB_EXTRA_F20();
        PH.setPack(p186);
        p186.sue_number_of_inputs_SET((char)211);
        p186.sue_trim_value_input_1_SET((short)5625);
        p186.sue_trim_value_input_2_SET((short) -5677);
        p186.sue_trim_value_input_3_SET((short)30379);
        p186.sue_trim_value_input_4_SET((short) -22195);
        p186.sue_trim_value_input_5_SET((short)5351);
        p186.sue_trim_value_input_6_SET((short) -8817);
        p186.sue_trim_value_input_7_SET((short) -23498);
        p186.sue_trim_value_input_8_SET((short)2704);
        p186.sue_trim_value_input_9_SET((short) -1595);
        p186.sue_trim_value_input_10_SET((short)27903);
        p186.sue_trim_value_input_11_SET((short)21882);
        p186.sue_trim_value_input_12_SET((short) -21944);
        LoopBackDemoChannel.instance.send(p186); //===============================
        SERIAL_UDB_EXTRA_F21 p187 = LoopBackDemoChannel.instance.new_SERIAL_UDB_EXTRA_F21();
        PH.setPack(p187);
        p187.sue_accel_x_offset_SET((short) -18101);
        p187.sue_accel_y_offset_SET((short)6320);
        p187.sue_accel_z_offset_SET((short) -30967);
        p187.sue_gyro_x_offset_SET((short)8983);
        p187.sue_gyro_y_offset_SET((short)25208);
        p187.sue_gyro_z_offset_SET((short) -2345);
        LoopBackDemoChannel.instance.send(p187); //===============================
        SERIAL_UDB_EXTRA_F22 p188 = LoopBackDemoChannel.instance.new_SERIAL_UDB_EXTRA_F22();
        PH.setPack(p188);
        p188.sue_accel_x_at_calibration_SET((short) -29684);
        p188.sue_accel_y_at_calibration_SET((short) -18485);
        p188.sue_accel_z_at_calibration_SET((short) -24365);
        p188.sue_gyro_x_at_calibration_SET((short)18761);
        p188.sue_gyro_y_at_calibration_SET((short) -222);
        p188.sue_gyro_z_at_calibration_SET((short)2051);
        LoopBackDemoChannel.instance.send(p188); //===============================
        ESTIMATOR_STATUS p230 = LoopBackDemoChannel.instance.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.time_usec_SET(8550053797434440331L);
        p230.flags_SET(ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS);
        p230.vel_ratio_SET(-2.4515987E38F);
        p230.pos_horiz_ratio_SET(-8.530238E37F);
        p230.pos_vert_ratio_SET(-2.0001285E38F);
        p230.mag_ratio_SET(1.1186768E38F);
        p230.hagl_ratio_SET(6.4878135E37F);
        p230.tas_ratio_SET(-1.355491E38F);
        p230.pos_horiz_accuracy_SET(2.3570403E38F);
        p230.pos_vert_accuracy_SET(-3.756161E37F);
        LoopBackDemoChannel.instance.send(p230); //===============================
        WIND_COV p231 = LoopBackDemoChannel.instance.new_WIND_COV();
        PH.setPack(p231);
        p231.time_usec_SET(8576123196173963944L);
        p231.wind_x_SET(3.148576E38F);
        p231.wind_y_SET(-2.0579293E38F);
        p231.wind_z_SET(-8.633847E37F);
        p231.var_horiz_SET(-2.3902812E38F);
        p231.var_vert_SET(-2.071993E38F);
        p231.wind_alt_SET(-2.8457452E38F);
        p231.horiz_accuracy_SET(-9.723093E36F);
        p231.vert_accuracy_SET(1.1350157E38F);
        LoopBackDemoChannel.instance.send(p231); //===============================
        GPS_INPUT p232 = LoopBackDemoChannel.instance.new_GPS_INPUT();
        PH.setPack(p232);
        p232.time_usec_SET(3493791617358122786L);
        p232.gps_id_SET((char)136);
        p232.ignore_flags_SET(GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY);
        p232.time_week_ms_SET(1129123676L);
        p232.time_week_SET((char)38175);
        p232.fix_type_SET((char)101);
        p232.lat_SET(1052991279);
        p232.lon_SET(989819233);
        p232.alt_SET(2.5932416E38F);
        p232.hdop_SET(1.9603482E38F);
        p232.vdop_SET(-1.6491898E38F);
        p232.vn_SET(1.4866774E37F);
        p232.ve_SET(-2.3066046E37F);
        p232.vd_SET(1.3414097E38F);
        p232.speed_accuracy_SET(-3.2602132E38F);
        p232.horiz_accuracy_SET(-3.4461647E37F);
        p232.vert_accuracy_SET(-2.0571101E38F);
        p232.satellites_visible_SET((char)193);
        LoopBackDemoChannel.instance.send(p232); //===============================
        GPS_RTCM_DATA p233 = LoopBackDemoChannel.instance.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.flags_SET((char)251);
        p233.len_SET((char)68);
        p233.data__SET(new char[180], 0);
        LoopBackDemoChannel.instance.send(p233); //===============================
        HIGH_LATENCY p234 = LoopBackDemoChannel.instance.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED);
        p234.custom_mode_SET(2422606093L);
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED);
        p234.roll_SET((short)22456);
        p234.pitch_SET((short) -15375);
        p234.heading_SET((char)35734);
        p234.throttle_SET((byte) - 17);
        p234.heading_sp_SET((short) -2684);
        p234.latitude_SET(-869971907);
        p234.longitude_SET(-1476832839);
        p234.altitude_amsl_SET((short)28047);
        p234.altitude_sp_SET((short)2949);
        p234.airspeed_SET((char)191);
        p234.airspeed_sp_SET((char)58);
        p234.groundspeed_SET((char)42);
        p234.climb_rate_SET((byte) - 37);
        p234.gps_nsat_SET((char)117);
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX);
        p234.battery_remaining_SET((char)115);
        p234.temperature_SET((byte)0);
        p234.temperature_air_SET((byte)54);
        p234.failsafe_SET((char)167);
        p234.wp_num_SET((char)163);
        p234.wp_distance_SET((char)54942);
        LoopBackDemoChannel.instance.send(p234); //===============================
        VIBRATION p241 = LoopBackDemoChannel.instance.new_VIBRATION();
        PH.setPack(p241);
        p241.time_usec_SET(3471274579445384160L);
        p241.vibration_x_SET(3.2306956E38F);
        p241.vibration_y_SET(5.797956E37F);
        p241.vibration_z_SET(9.364708E37F);
        p241.clipping_0_SET(380323358L);
        p241.clipping_1_SET(2759155811L);
        p241.clipping_2_SET(2565942477L);
        LoopBackDemoChannel.instance.send(p241); //===============================
        HOME_POSITION p242 = LoopBackDemoChannel.instance.new_HOME_POSITION();
        PH.setPack(p242);
        p242.latitude_SET(-328028519);
        p242.longitude_SET(575471155);
        p242.altitude_SET(523801456);
        p242.x_SET(3.272948E38F);
        p242.y_SET(-2.041639E38F);
        p242.z_SET(2.4969867E37F);
        p242.q_SET(new float[4], 0);
        p242.approach_x_SET(8.633023E36F);
        p242.approach_y_SET(-3.5653097E37F);
        p242.approach_z_SET(1.0950674E38F);
        p242.time_usec_SET(8402087343254995549L, PH);
        LoopBackDemoChannel.instance.send(p242); //===============================
        SET_HOME_POSITION p243 = LoopBackDemoChannel.instance.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.target_system_SET((char)229);
        p243.latitude_SET(-409820963);
        p243.longitude_SET(405237705);
        p243.altitude_SET(34850550);
        p243.x_SET(3.133159E38F);
        p243.y_SET(2.792752E38F);
        p243.z_SET(2.3514164E38F);
        p243.q_SET(new float[4], 0);
        p243.approach_x_SET(-2.657873E38F);
        p243.approach_y_SET(-1.4648874E38F);
        p243.approach_z_SET(1.0413156E38F);
        p243.time_usec_SET(4407614647749698398L, PH);
        LoopBackDemoChannel.instance.send(p243); //===============================
        MESSAGE_INTERVAL p244 = LoopBackDemoChannel.instance.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.message_id_SET((char)53098);
        p244.interval_us_SET(1425318619);
        LoopBackDemoChannel.instance.send(p244); //===============================
        EXTENDED_SYS_STATE p245 = LoopBackDemoChannel.instance.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_MC);
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING);
        LoopBackDemoChannel.instance.send(p245); //===============================
        ADSB_VEHICLE p246 = LoopBackDemoChannel.instance.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.ICAO_address_SET(1062204677L);
        p246.lat_SET(2059724818);
        p246.lon_SET(-840976118);
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
        p246.altitude_SET(309337921);
        p246.heading_SET((char)33588);
        p246.hor_velocity_SET((char)28427);
        p246.ver_velocity_SET((short)23701);
        p246.callsign_SET("DEMO", PH);
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_LIGHT);
        p246.tslc_SET((char)70);
        p246.flags_SET(ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING);
        p246.squawk_SET((char)45582);
        LoopBackDemoChannel.instance.send(p246); //===============================
        COLLISION p247 = LoopBackDemoChannel.instance.new_COLLISION();
        PH.setPack(p247);
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
        p247.id_SET(1288179891L);
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_HORIZONTALLY);
        p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH);
        p247.time_to_minimum_delta_SET(2.682733E38F);
        p247.altitude_minimum_delta_SET(-2.4988142E38F);
        p247.horizontal_minimum_delta_SET(-1.8338744E38F);
        LoopBackDemoChannel.instance.send(p247); //===============================
        V2_EXTENSION p248 = LoopBackDemoChannel.instance.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.target_network_SET((char)75);
        p248.target_system_SET((char)59);
        p248.target_component_SET((char)64);
        p248.message_type_SET((char)39352);
        p248.payload_SET(new char[249], 0);
        LoopBackDemoChannel.instance.send(p248); //===============================
        MEMORY_VECT p249 = LoopBackDemoChannel.instance.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.address_SET((char)41727);
        p249.ver_SET((char)212);
        p249.type_SET((char)39);
        p249.value_SET(new byte[32], 0);
        LoopBackDemoChannel.instance.send(p249); //===============================
        DEBUG_VECT p250 = LoopBackDemoChannel.instance.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.name_SET("DEMO", PH);
        p250.time_usec_SET(1696741743100294393L);
        p250.x_SET(-3.4452023E37F);
        p250.y_SET(3.176778E38F);
        p250.z_SET(2.1140739E38F);
        LoopBackDemoChannel.instance.send(p250); //===============================
        NAMED_VALUE_FLOAT p251 = LoopBackDemoChannel.instance.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.time_boot_ms_SET(859525392L);
        p251.name_SET("DEMO", PH);
        p251.value_SET(-9.832151E37F);
        LoopBackDemoChannel.instance.send(p251); //===============================
        NAMED_VALUE_INT p252 = LoopBackDemoChannel.instance.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.time_boot_ms_SET(1730954104L);
        p252.name_SET("DEMO", PH);
        p252.value_SET(-2059425355);
        LoopBackDemoChannel.instance.send(p252); //===============================
        STATUSTEXT p253 = LoopBackDemoChannel.instance.new_STATUSTEXT();
        PH.setPack(p253);
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_WARNING);
        p253.text_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p253); //===============================
        DEBUG p254 = LoopBackDemoChannel.instance.new_DEBUG();
        PH.setPack(p254);
        p254.time_boot_ms_SET(2454408489L);
        p254.ind_SET((char)17);
        p254.value_SET(-2.6245412E38F);
        LoopBackDemoChannel.instance.send(p254); //===============================
        SETUP_SIGNING p256 = LoopBackDemoChannel.instance.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.target_system_SET((char)50);
        p256.target_component_SET((char)6);
        p256.secret_key_SET(new char[32], 0);
        p256.initial_timestamp_SET(3753330544872415652L);
        LoopBackDemoChannel.instance.send(p256); //===============================
        BUTTON_CHANGE p257 = LoopBackDemoChannel.instance.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.time_boot_ms_SET(2280157687L);
        p257.last_change_ms_SET(3602202901L);
        p257.state_SET((char)173);
        LoopBackDemoChannel.instance.send(p257); //===============================
        PLAY_TUNE p258 = LoopBackDemoChannel.instance.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.target_system_SET((char)102);
        p258.target_component_SET((char)109);
        p258.tune_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p258); //===============================
        CAMERA_INFORMATION p259 = LoopBackDemoChannel.instance.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.time_boot_ms_SET(1219398926L);
        p259.vendor_name_SET(new char[32], 0);
        p259.model_name_SET(new char[32], 0);
        p259.firmware_version_SET(473990288L);
        p259.focal_length_SET(-8.77932E37F);
        p259.sensor_size_h_SET(-2.0574255E38F);
        p259.sensor_size_v_SET(-2.3628281E38F);
        p259.resolution_h_SET((char)53246);
        p259.resolution_v_SET((char)50467);
        p259.lens_id_SET((char)47);
        p259.flags_SET(CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE);
        p259.cam_definition_version_SET((char)43597);
        p259.cam_definition_uri_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p259); //===============================
        CAMERA_SETTINGS p260 = LoopBackDemoChannel.instance.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.time_boot_ms_SET(4229132258L);
        p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_VIDEO);
        LoopBackDemoChannel.instance.send(p260); //===============================
        STORAGE_INFORMATION p261 = LoopBackDemoChannel.instance.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.time_boot_ms_SET(2523948607L);
        p261.storage_id_SET((char)161);
        p261.storage_count_SET((char)68);
        p261.status_SET((char)44);
        p261.total_capacity_SET(-2.954248E38F);
        p261.used_capacity_SET(1.5761326E38F);
        p261.available_capacity_SET(2.6960649E38F);
        p261.read_speed_SET(-1.817216E38F);
        p261.write_speed_SET(3.1744585E38F);
        LoopBackDemoChannel.instance.send(p261); //===============================
        CAMERA_CAPTURE_STATUS p262 = LoopBackDemoChannel.instance.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.time_boot_ms_SET(2277852315L);
        p262.image_status_SET((char)86);
        p262.video_status_SET((char)171);
        p262.image_interval_SET(-1.6389607E38F);
        p262.recording_time_ms_SET(1742082429L);
        p262.available_capacity_SET(-1.0911167E38F);
        LoopBackDemoChannel.instance.send(p262); //===============================
        CAMERA_IMAGE_CAPTURED p263 = LoopBackDemoChannel.instance.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.time_boot_ms_SET(2324977547L);
        p263.time_utc_SET(5723913942883467725L);
        p263.camera_id_SET((char)227);
        p263.lat_SET(1854708098);
        p263.lon_SET(-1205072272);
        p263.alt_SET(-428115051);
        p263.relative_alt_SET(-1649374115);
        p263.q_SET(new float[4], 0);
        p263.image_index_SET(-1152637993);
        p263.capture_result_SET((byte)2);
        p263.file_url_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p263); //===============================
        FLIGHT_INFORMATION p264 = LoopBackDemoChannel.instance.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.time_boot_ms_SET(1853505144L);
        p264.arming_time_utc_SET(8824318717568443972L);
        p264.takeoff_time_utc_SET(5512334641310956019L);
        p264.flight_uuid_SET(8475281147048555374L);
        LoopBackDemoChannel.instance.send(p264); //===============================
        MOUNT_ORIENTATION p265 = LoopBackDemoChannel.instance.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.time_boot_ms_SET(934350397L);
        p265.roll_SET(1.6221559E38F);
        p265.pitch_SET(-1.8548537E38F);
        p265.yaw_SET(2.1027293E38F);
        LoopBackDemoChannel.instance.send(p265); //===============================
        LOGGING_DATA p266 = LoopBackDemoChannel.instance.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.target_system_SET((char)133);
        p266.target_component_SET((char)142);
        p266.sequence_SET((char)54820);
        p266.length_SET((char)61);
        p266.first_message_offset_SET((char)7);
        p266.data__SET(new char[249], 0);
        LoopBackDemoChannel.instance.send(p266); //===============================
        LOGGING_DATA_ACKED p267 = LoopBackDemoChannel.instance.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.target_system_SET((char)30);
        p267.target_component_SET((char)127);
        p267.sequence_SET((char)34532);
        p267.length_SET((char)40);
        p267.first_message_offset_SET((char)67);
        p267.data__SET(new char[249], 0);
        LoopBackDemoChannel.instance.send(p267); //===============================
        LOGGING_ACK p268 = LoopBackDemoChannel.instance.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.target_system_SET((char)124);
        p268.target_component_SET((char)173);
        p268.sequence_SET((char)6304);
        LoopBackDemoChannel.instance.send(p268); //===============================
        VIDEO_STREAM_INFORMATION p269 = LoopBackDemoChannel.instance.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.camera_id_SET((char)237);
        p269.status_SET((char)5);
        p269.framerate_SET(-1.9138768E38F);
        p269.resolution_h_SET((char)14646);
        p269.resolution_v_SET((char)6170);
        p269.bitrate_SET(365705627L);
        p269.rotation_SET((char)49436);
        p269.uri_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p269); //===============================
        SET_VIDEO_STREAM_SETTINGS p270 = LoopBackDemoChannel.instance.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.target_system_SET((char)34);
        p270.target_component_SET((char)243);
        p270.camera_id_SET((char)139);
        p270.framerate_SET(-1.2504815E37F);
        p270.resolution_h_SET((char)23147);
        p270.resolution_v_SET((char)48857);
        p270.bitrate_SET(1446254447L);
        p270.rotation_SET((char)35830);
        p270.uri_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p270); //===============================
        WIFI_CONFIG_AP p299 = LoopBackDemoChannel.instance.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.ssid_SET("DEMO", PH);
        p299.password_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p299); //===============================
        PROTOCOL_VERSION p300 = LoopBackDemoChannel.instance.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.version_SET((char)16437);
        p300.min_version_SET((char)7754);
        p300.max_version_SET((char)9818);
        p300.spec_version_hash_SET(new char[8], 0);
        p300.library_version_hash_SET(new char[8], 0);
        LoopBackDemoChannel.instance.send(p300); //===============================
        UAVCAN_NODE_STATUS p310 = LoopBackDemoChannel.instance.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.time_usec_SET(854587481364372444L);
        p310.uptime_sec_SET(383884712L);
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR);
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE);
        p310.sub_mode_SET((char)9);
        p310.vendor_specific_status_code_SET((char)45571);
        LoopBackDemoChannel.instance.send(p310); //===============================
        UAVCAN_NODE_INFO p311 = LoopBackDemoChannel.instance.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.time_usec_SET(1022746864343528967L);
        p311.uptime_sec_SET(3612243416L);
        p311.name_SET("DEMO", PH);
        p311.hw_version_major_SET((char)100);
        p311.hw_version_minor_SET((char)18);
        p311.hw_unique_id_SET(new char[16], 0);
        p311.sw_version_major_SET((char)110);
        p311.sw_version_minor_SET((char)30);
        p311.sw_vcs_commit_SET(3362049441L);
        LoopBackDemoChannel.instance.send(p311); //===============================
        PARAM_EXT_REQUEST_READ p320 = LoopBackDemoChannel.instance.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.target_system_SET((char)204);
        p320.target_component_SET((char)112);
        p320.param_id_SET("DEMO", PH);
        p320.param_index_SET((short)2389);
        LoopBackDemoChannel.instance.send(p320); //===============================
        PARAM_EXT_REQUEST_LIST p321 = LoopBackDemoChannel.instance.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_system_SET((char)65);
        p321.target_component_SET((char)199);
        LoopBackDemoChannel.instance.send(p321); //===============================
        PARAM_EXT_VALUE p322 = LoopBackDemoChannel.instance.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_id_SET("DEMO", PH);
        p322.param_value_SET("DEMO", PH);
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32);
        p322.param_count_SET((char)61843);
        p322.param_index_SET((char)29990);
        LoopBackDemoChannel.instance.send(p322); //===============================
        PARAM_EXT_SET p323 = LoopBackDemoChannel.instance.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.target_system_SET((char)170);
        p323.target_component_SET((char)31);
        p323.param_id_SET("DEMO", PH);
        p323.param_value_SET("DEMO", PH);
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16);
        LoopBackDemoChannel.instance.send(p323); //===============================
        PARAM_EXT_ACK p324 = LoopBackDemoChannel.instance.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_id_SET("DEMO", PH);
        p324.param_value_SET("DEMO", PH);
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8);
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_FAILED);
        LoopBackDemoChannel.instance.send(p324); //===============================
        OBSTACLE_DISTANCE p330 = LoopBackDemoChannel.instance.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.time_usec_SET(1767039340111450987L);
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
        p330.distances_SET(new char[72], 0);
        p330.increment_SET((char)80);
        p330.min_distance_SET((char)46166);
        p330.max_distance_SET((char)18569);
        LoopBackDemoChannel.instance.send(p330); //===============================
    }
}

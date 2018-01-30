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
        LoopBackDemoChannel.instance.on_ARRAY_TEST_0.add((src, ph, pack) ->
        {
            char  v1 = pack.v1_GET();
            byte[]  ar_i8 = pack.ar_i8_GET();
            char[]  ar_u8 = pack.ar_u8_GET();
            char[]  ar_u16 = pack.ar_u16_GET();
            long[]  ar_u32 = pack.ar_u32_GET();
        });
        LoopBackDemoChannel.instance.on_ARRAY_TEST_1.add((src, ph, pack) ->
        {
            long[]  ar_u32 = pack.ar_u32_GET();
        });
        LoopBackDemoChannel.instance.on_ARRAY_TEST_3.add((src, ph, pack) ->
        {
            char  v = pack.v_GET();
            long[]  ar_u32 = pack.ar_u32_GET();
        });
        LoopBackDemoChannel.instance.on_ARRAY_TEST_4.add((src, ph, pack) ->
        {
            long[]  ar_u32 = pack.ar_u32_GET();
            char  v = pack.v_GET();
        });
        LoopBackDemoChannel.instance.on_ARRAY_TEST_5.add((src, ph, pack) ->
        {
            String c1 = pack.c1_TRY(ph);
            String c2 = pack.c2_TRY(ph);
        });
        LoopBackDemoChannel.instance.on_ARRAY_TEST_6.add((src, ph, pack) ->
        {
            char  v1 = pack.v1_GET();
            char  v2 = pack.v2_GET();
            long  v3 = pack.v3_GET();
            long[]  ar_u32 = pack.ar_u32_GET();
            int[]  ar_i32 = pack.ar_i32_GET();
            char[]  ar_u16 = pack.ar_u16_GET();
            short[]  ar_i16 = pack.ar_i16_GET();
            char[]  ar_u8 = pack.ar_u8_GET();
            byte[]  ar_i8 = pack.ar_i8_GET();
            String ar_c = pack.ar_c_TRY(ph);
            double[]  ar_d = pack.ar_d_GET();
            float[]  ar_f = pack.ar_f_GET();
        });
        LoopBackDemoChannel.instance.on_ARRAY_TEST_7.add((src, ph, pack) ->
        {
            double[]  ar_d = pack.ar_d_GET();
            float[]  ar_f = pack.ar_f_GET();
            long[]  ar_u32 = pack.ar_u32_GET();
            int[]  ar_i32 = pack.ar_i32_GET();
            char[]  ar_u16 = pack.ar_u16_GET();
            short[]  ar_i16 = pack.ar_i16_GET();
            char[]  ar_u8 = pack.ar_u8_GET();
            byte[]  ar_i8 = pack.ar_i8_GET();
            String ar_c = pack.ar_c_TRY(ph);
        });
        LoopBackDemoChannel.instance.on_ARRAY_TEST_8.add((src, ph, pack) ->
        {
            long  v3 = pack.v3_GET();
            double[]  ar_d = pack.ar_d_GET();
            char[]  ar_u16 = pack.ar_u16_GET();
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
        p0.type_SET(MAV_TYPE.MAV_TYPE_SURFACE_BOAT);
        p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY);
        p0.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED);
        p0.custom_mode_SET(530132522L);
        p0.system_status_SET(MAV_STATE.MAV_STATE_STANDBY);
        p0.mavlink_version_SET((char)106);
        LoopBackDemoChannel.instance.send(p0); //===============================
        SYS_STATUS p1 = LoopBackDemoChannel.instance.new_SYS_STATUS();
        PH.setPack(p1);
        p1.onboard_control_sensors_present_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER);
        p1.onboard_control_sensors_enabled_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER);
        p1.onboard_control_sensors_health_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE);
        p1.load_SET((char)9597);
        p1.voltage_battery_SET((char)46243);
        p1.current_battery_SET((short)6109);
        p1.battery_remaining_SET((byte) - 24);
        p1.drop_rate_comm_SET((char)17433);
        p1.errors_comm_SET((char)14269);
        p1.errors_count1_SET((char)12318);
        p1.errors_count2_SET((char)57622);
        p1.errors_count3_SET((char)2842);
        p1.errors_count4_SET((char)32185);
        LoopBackDemoChannel.instance.send(p1); //===============================
        SYSTEM_TIME p2 = LoopBackDemoChannel.instance.new_SYSTEM_TIME();
        PH.setPack(p2);
        p2.time_unix_usec_SET(171449526190849309L);
        p2.time_boot_ms_SET(1947756564L);
        LoopBackDemoChannel.instance.send(p2); //===============================
        POSITION_TARGET_LOCAL_NED p3 = LoopBackDemoChannel.instance.new_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.time_boot_ms_SET(3414150610L);
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_MISSION);
        p3.type_mask_SET((char)47913);
        p3.x_SET(-4.580567E37F);
        p3.y_SET(6.123135E36F);
        p3.z_SET(3.17521E38F);
        p3.vx_SET(-2.2229486E38F);
        p3.vy_SET(-1.3387142E37F);
        p3.vz_SET(5.105895E36F);
        p3.afx_SET(-1.3519336E38F);
        p3.afy_SET(-1.5338891E38F);
        p3.afz_SET(-2.8057306E38F);
        p3.yaw_SET(1.1865643E38F);
        p3.yaw_rate_SET(1.384052E38F);
        LoopBackDemoChannel.instance.send(p3); //===============================
        PING p4 = LoopBackDemoChannel.instance.new_PING();
        PH.setPack(p4);
        p4.time_usec_SET(5045675895713035878L);
        p4.seq_SET(1435197037L);
        p4.target_system_SET((char)139);
        p4.target_component_SET((char)107);
        LoopBackDemoChannel.instance.send(p4); //===============================
        CHANGE_OPERATOR_CONTROL p5 = LoopBackDemoChannel.instance.new_CHANGE_OPERATOR_CONTROL();
        PH.setPack(p5);
        p5.target_system_SET((char)157);
        p5.control_request_SET((char)24);
        p5.version_SET((char)118);
        p5.passkey_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p5); //===============================
        CHANGE_OPERATOR_CONTROL_ACK p6 = LoopBackDemoChannel.instance.new_CHANGE_OPERATOR_CONTROL_ACK();
        PH.setPack(p6);
        p6.gcs_system_id_SET((char)131);
        p6.control_request_SET((char)12);
        p6.ack_SET((char)93);
        LoopBackDemoChannel.instance.send(p6); //===============================
        AUTH_KEY p7 = LoopBackDemoChannel.instance.new_AUTH_KEY();
        PH.setPack(p7);
        p7.key_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p7); //===============================
        SET_MODE p11 = LoopBackDemoChannel.instance.new_SET_MODE();
        PH.setPack(p11);
        p11.target_system_SET((char)59);
        p11.base_mode_SET(MAV_MODE.MAV_MODE_MANUAL_DISARMED);
        p11.custom_mode_SET(1802352795L);
        LoopBackDemoChannel.instance.send(p11); //===============================
        PARAM_REQUEST_READ p20 = LoopBackDemoChannel.instance.new_PARAM_REQUEST_READ();
        PH.setPack(p20);
        p20.target_system_SET((char)225);
        p20.target_component_SET((char)80);
        p20.param_id_SET("DEMO", PH);
        p20.param_index_SET((short) -15153);
        LoopBackDemoChannel.instance.send(p20); //===============================
        PARAM_REQUEST_LIST p21 = LoopBackDemoChannel.instance.new_PARAM_REQUEST_LIST();
        PH.setPack(p21);
        p21.target_system_SET((char)45);
        p21.target_component_SET((char)214);
        LoopBackDemoChannel.instance.send(p21); //===============================
        PARAM_VALUE p22 = LoopBackDemoChannel.instance.new_PARAM_VALUE();
        PH.setPack(p22);
        p22.param_id_SET("DEMO", PH);
        p22.param_value_SET(2.0792506E38F);
        p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT32);
        p22.param_count_SET((char)52045);
        p22.param_index_SET((char)28411);
        LoopBackDemoChannel.instance.send(p22); //===============================
        PARAM_SET p23 = LoopBackDemoChannel.instance.new_PARAM_SET();
        PH.setPack(p23);
        p23.target_system_SET((char)4);
        p23.target_component_SET((char)88);
        p23.param_id_SET("DEMO", PH);
        p23.param_value_SET(-1.5552604E37F);
        p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT64);
        LoopBackDemoChannel.instance.send(p23); //===============================
        GPS_RAW_INT p24 = LoopBackDemoChannel.instance.new_GPS_RAW_INT();
        PH.setPack(p24);
        p24.time_usec_SET(6298844422695663668L);
        p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS);
        p24.lat_SET(-1655977196);
        p24.lon_SET(-840457152);
        p24.alt_SET(-511900038);
        p24.eph_SET((char)40650);
        p24.epv_SET((char)5050);
        p24.vel_SET((char)28919);
        p24.cog_SET((char)58521);
        p24.satellites_visible_SET((char)15);
        p24.alt_ellipsoid_SET(88904948, PH);
        p24.h_acc_SET(3746565806L, PH);
        p24.v_acc_SET(2755073566L, PH);
        p24.vel_acc_SET(3529701641L, PH);
        p24.hdg_acc_SET(3029218080L, PH);
        LoopBackDemoChannel.instance.send(p24); //===============================
        GPS_STATUS p25 = LoopBackDemoChannel.instance.new_GPS_STATUS();
        PH.setPack(p25);
        p25.satellites_visible_SET((char)176);
        p25.satellite_prn_SET(new char[20], 0);
        p25.satellite_used_SET(new char[20], 0);
        p25.satellite_elevation_SET(new char[20], 0);
        p25.satellite_azimuth_SET(new char[20], 0);
        p25.satellite_snr_SET(new char[20], 0);
        LoopBackDemoChannel.instance.send(p25); //===============================
        SCALED_IMU p26 = LoopBackDemoChannel.instance.new_SCALED_IMU();
        PH.setPack(p26);
        p26.time_boot_ms_SET(3231297935L);
        p26.xacc_SET((short)17959);
        p26.yacc_SET((short)3506);
        p26.zacc_SET((short) -7204);
        p26.xgyro_SET((short) -21786);
        p26.ygyro_SET((short)12113);
        p26.zgyro_SET((short) -19546);
        p26.xmag_SET((short)23566);
        p26.ymag_SET((short) -29494);
        p26.zmag_SET((short) -12641);
        LoopBackDemoChannel.instance.send(p26); //===============================
        RAW_IMU p27 = LoopBackDemoChannel.instance.new_RAW_IMU();
        PH.setPack(p27);
        p27.time_usec_SET(8718140574529363855L);
        p27.xacc_SET((short) -23475);
        p27.yacc_SET((short)4655);
        p27.zacc_SET((short) -25094);
        p27.xgyro_SET((short) -31921);
        p27.ygyro_SET((short)19058);
        p27.zgyro_SET((short)6660);
        p27.xmag_SET((short)2455);
        p27.ymag_SET((short)24314);
        p27.zmag_SET((short) -26694);
        LoopBackDemoChannel.instance.send(p27); //===============================
        RAW_PRESSURE p28 = LoopBackDemoChannel.instance.new_RAW_PRESSURE();
        PH.setPack(p28);
        p28.time_usec_SET(5819399633825741960L);
        p28.press_abs_SET((short)11525);
        p28.press_diff1_SET((short) -30565);
        p28.press_diff2_SET((short)3714);
        p28.temperature_SET((short) -20431);
        LoopBackDemoChannel.instance.send(p28); //===============================
        SCALED_PRESSURE p29 = LoopBackDemoChannel.instance.new_SCALED_PRESSURE();
        PH.setPack(p29);
        p29.time_boot_ms_SET(3340443540L);
        p29.press_abs_SET(-1.1245682E38F);
        p29.press_diff_SET(2.6909675E38F);
        p29.temperature_SET((short)8599);
        LoopBackDemoChannel.instance.send(p29); //===============================
        ATTITUDE p30 = LoopBackDemoChannel.instance.new_ATTITUDE();
        PH.setPack(p30);
        p30.time_boot_ms_SET(3326625960L);
        p30.roll_SET(-3.0290339E38F);
        p30.pitch_SET(2.6988614E38F);
        p30.yaw_SET(2.5802121E38F);
        p30.rollspeed_SET(1.0905479E38F);
        p30.pitchspeed_SET(-2.8379604E38F);
        p30.yawspeed_SET(1.5666334E38F);
        LoopBackDemoChannel.instance.send(p30); //===============================
        ATTITUDE_QUATERNION p31 = LoopBackDemoChannel.instance.new_ATTITUDE_QUATERNION();
        PH.setPack(p31);
        p31.time_boot_ms_SET(3584514582L);
        p31.q1_SET(-2.9491636E38F);
        p31.q2_SET(-2.4476857E38F);
        p31.q3_SET(-9.599796E37F);
        p31.q4_SET(-3.0356425E38F);
        p31.rollspeed_SET(3.1443947E38F);
        p31.pitchspeed_SET(1.879886E38F);
        p31.yawspeed_SET(-1.5433323E38F);
        LoopBackDemoChannel.instance.send(p31); //===============================
        LOCAL_POSITION_NED p32 = LoopBackDemoChannel.instance.new_LOCAL_POSITION_NED();
        PH.setPack(p32);
        p32.time_boot_ms_SET(2211131752L);
        p32.x_SET(-9.361345E37F);
        p32.y_SET(5.519495E37F);
        p32.z_SET(1.659089E38F);
        p32.vx_SET(1.7779353E38F);
        p32.vy_SET(-1.2938865E38F);
        p32.vz_SET(8.635413E37F);
        LoopBackDemoChannel.instance.send(p32); //===============================
        GLOBAL_POSITION_INT p33 = LoopBackDemoChannel.instance.new_GLOBAL_POSITION_INT();
        PH.setPack(p33);
        p33.time_boot_ms_SET(3502866022L);
        p33.lat_SET(-1281995032);
        p33.lon_SET(-433737940);
        p33.alt_SET(967720802);
        p33.relative_alt_SET(-1988016189);
        p33.vx_SET((short) -11136);
        p33.vy_SET((short)16457);
        p33.vz_SET((short)26568);
        p33.hdg_SET((char)59790);
        LoopBackDemoChannel.instance.send(p33); //===============================
        RC_CHANNELS_SCALED p34 = LoopBackDemoChannel.instance.new_RC_CHANNELS_SCALED();
        PH.setPack(p34);
        p34.time_boot_ms_SET(577069522L);
        p34.port_SET((char)155);
        p34.chan1_scaled_SET((short) -19341);
        p34.chan2_scaled_SET((short)22640);
        p34.chan3_scaled_SET((short) -25952);
        p34.chan4_scaled_SET((short)30603);
        p34.chan5_scaled_SET((short)26272);
        p34.chan6_scaled_SET((short) -1010);
        p34.chan7_scaled_SET((short) -610);
        p34.chan8_scaled_SET((short)633);
        p34.rssi_SET((char)86);
        LoopBackDemoChannel.instance.send(p34); //===============================
        RC_CHANNELS_RAW p35 = LoopBackDemoChannel.instance.new_RC_CHANNELS_RAW();
        PH.setPack(p35);
        p35.time_boot_ms_SET(1367311985L);
        p35.port_SET((char)153);
        p35.chan1_raw_SET((char)28406);
        p35.chan2_raw_SET((char)28440);
        p35.chan3_raw_SET((char)41687);
        p35.chan4_raw_SET((char)24095);
        p35.chan5_raw_SET((char)8316);
        p35.chan6_raw_SET((char)44551);
        p35.chan7_raw_SET((char)43654);
        p35.chan8_raw_SET((char)31796);
        p35.rssi_SET((char)238);
        LoopBackDemoChannel.instance.send(p35); //===============================
        SERVO_OUTPUT_RAW p36 = LoopBackDemoChannel.instance.new_SERVO_OUTPUT_RAW();
        PH.setPack(p36);
        p36.time_usec_SET(203695301L);
        p36.port_SET((char)11);
        p36.servo1_raw_SET((char)40434);
        p36.servo2_raw_SET((char)14509);
        p36.servo3_raw_SET((char)7065);
        p36.servo4_raw_SET((char)38101);
        p36.servo5_raw_SET((char)3574);
        p36.servo6_raw_SET((char)12720);
        p36.servo7_raw_SET((char)41131);
        p36.servo8_raw_SET((char)39754);
        p36.servo9_raw_SET((char)20418, PH);
        p36.servo10_raw_SET((char)44205, PH);
        p36.servo11_raw_SET((char)60929, PH);
        p36.servo12_raw_SET((char)42666, PH);
        p36.servo13_raw_SET((char)5374, PH);
        p36.servo14_raw_SET((char)41895, PH);
        p36.servo15_raw_SET((char)1675, PH);
        p36.servo16_raw_SET((char)2375, PH);
        LoopBackDemoChannel.instance.send(p36); //===============================
        MISSION_REQUEST_PARTIAL_LIST p37 = LoopBackDemoChannel.instance.new_MISSION_REQUEST_PARTIAL_LIST();
        PH.setPack(p37);
        p37.target_system_SET((char)51);
        p37.target_component_SET((char)223);
        p37.start_index_SET((short)25559);
        p37.end_index_SET((short) -1591);
        p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
        LoopBackDemoChannel.instance.send(p37); //===============================
        MISSION_WRITE_PARTIAL_LIST p38 = LoopBackDemoChannel.instance.new_MISSION_WRITE_PARTIAL_LIST();
        PH.setPack(p38);
        p38.target_system_SET((char)123);
        p38.target_component_SET((char)112);
        p38.start_index_SET((short) -14783);
        p38.end_index_SET((short) -11738);
        p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
        LoopBackDemoChannel.instance.send(p38); //===============================
        MISSION_ITEM p39 = LoopBackDemoChannel.instance.new_MISSION_ITEM();
        PH.setPack(p39);
        p39.target_system_SET((char)209);
        p39.target_component_SET((char)247);
        p39.seq_SET((char)14639);
        p39.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
        p39.command_SET(MAV_CMD.MAV_CMD_DO_SET_RELAY);
        p39.current_SET((char)176);
        p39.autocontinue_SET((char)178);
        p39.param1_SET(2.7588152E38F);
        p39.param2_SET(-2.9545948E38F);
        p39.param3_SET(-6.443832E37F);
        p39.param4_SET(-1.6777283E38F);
        p39.x_SET(2.4561312E38F);
        p39.y_SET(2.184292E38F);
        p39.z_SET(2.4307967E38F);
        p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
        LoopBackDemoChannel.instance.send(p39); //===============================
        MISSION_REQUEST p40 = LoopBackDemoChannel.instance.new_MISSION_REQUEST();
        PH.setPack(p40);
        p40.target_system_SET((char)75);
        p40.target_component_SET((char)24);
        p40.seq_SET((char)39933);
        p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
        LoopBackDemoChannel.instance.send(p40); //===============================
        MISSION_SET_CURRENT p41 = LoopBackDemoChannel.instance.new_MISSION_SET_CURRENT();
        PH.setPack(p41);
        p41.target_system_SET((char)227);
        p41.target_component_SET((char)89);
        p41.seq_SET((char)18964);
        LoopBackDemoChannel.instance.send(p41); //===============================
        MISSION_CURRENT p42 = LoopBackDemoChannel.instance.new_MISSION_CURRENT();
        PH.setPack(p42);
        p42.seq_SET((char)2093);
        LoopBackDemoChannel.instance.send(p42); //===============================
        MISSION_REQUEST_LIST p43 = LoopBackDemoChannel.instance.new_MISSION_REQUEST_LIST();
        PH.setPack(p43);
        p43.target_system_SET((char)174);
        p43.target_component_SET((char)183);
        p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
        LoopBackDemoChannel.instance.send(p43); //===============================
        MISSION_COUNT p44 = LoopBackDemoChannel.instance.new_MISSION_COUNT();
        PH.setPack(p44);
        p44.target_system_SET((char)31);
        p44.target_component_SET((char)0);
        p44.count_SET((char)50090);
        p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
        LoopBackDemoChannel.instance.send(p44); //===============================
        MISSION_CLEAR_ALL p45 = LoopBackDemoChannel.instance.new_MISSION_CLEAR_ALL();
        PH.setPack(p45);
        p45.target_system_SET((char)162);
        p45.target_component_SET((char)41);
        p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
        LoopBackDemoChannel.instance.send(p45); //===============================
        MISSION_ITEM_REACHED p46 = LoopBackDemoChannel.instance.new_MISSION_ITEM_REACHED();
        PH.setPack(p46);
        p46.seq_SET((char)62557);
        LoopBackDemoChannel.instance.send(p46); //===============================
        MISSION_ACK p47 = LoopBackDemoChannel.instance.new_MISSION_ACK();
        PH.setPack(p47);
        p47.target_system_SET((char)236);
        p47.target_component_SET((char)30);
        p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_ACCEPTED);
        p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
        LoopBackDemoChannel.instance.send(p47); //===============================
        SET_GPS_GLOBAL_ORIGIN p48 = LoopBackDemoChannel.instance.new_SET_GPS_GLOBAL_ORIGIN();
        PH.setPack(p48);
        p48.target_system_SET((char)168);
        p48.latitude_SET(1679203376);
        p48.longitude_SET(758259677);
        p48.altitude_SET(-859557294);
        p48.time_usec_SET(1306320992581271796L, PH);
        LoopBackDemoChannel.instance.send(p48); //===============================
        GPS_GLOBAL_ORIGIN p49 = LoopBackDemoChannel.instance.new_GPS_GLOBAL_ORIGIN();
        PH.setPack(p49);
        p49.latitude_SET(-296723159);
        p49.longitude_SET(1963718405);
        p49.altitude_SET(-790487666);
        p49.time_usec_SET(6561854086031693749L, PH);
        LoopBackDemoChannel.instance.send(p49); //===============================
        PARAM_MAP_RC p50 = LoopBackDemoChannel.instance.new_PARAM_MAP_RC();
        PH.setPack(p50);
        p50.target_system_SET((char)232);
        p50.target_component_SET((char)94);
        p50.param_id_SET("DEMO", PH);
        p50.param_index_SET((short)23069);
        p50.parameter_rc_channel_index_SET((char)166);
        p50.param_value0_SET(1.5008645E38F);
        p50.scale_SET(1.110085E38F);
        p50.param_value_min_SET(1.624064E38F);
        p50.param_value_max_SET(-2.5206975E38F);
        LoopBackDemoChannel.instance.send(p50); //===============================
        MISSION_REQUEST_INT p51 = LoopBackDemoChannel.instance.new_MISSION_REQUEST_INT();
        PH.setPack(p51);
        p51.target_system_SET((char)85);
        p51.target_component_SET((char)212);
        p51.seq_SET((char)9209);
        p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
        LoopBackDemoChannel.instance.send(p51); //===============================
        SAFETY_SET_ALLOWED_AREA p54 = LoopBackDemoChannel.instance.new_SAFETY_SET_ALLOWED_AREA();
        PH.setPack(p54);
        p54.target_system_SET((char)188);
        p54.target_component_SET((char)151);
        p54.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_NED);
        p54.p1x_SET(-8.277163E37F);
        p54.p1y_SET(4.041564E37F);
        p54.p1z_SET(-3.1804106E38F);
        p54.p2x_SET(2.243573E36F);
        p54.p2y_SET(2.889291E38F);
        p54.p2z_SET(1.7970487E38F);
        LoopBackDemoChannel.instance.send(p54); //===============================
        SAFETY_ALLOWED_AREA p55 = LoopBackDemoChannel.instance.new_SAFETY_ALLOWED_AREA();
        PH.setPack(p55);
        p55.frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED);
        p55.p1x_SET(-1.9006238E38F);
        p55.p1y_SET(-1.6916598E38F);
        p55.p1z_SET(-2.5506826E38F);
        p55.p2x_SET(3.2324802E38F);
        p55.p2y_SET(2.98605E38F);
        p55.p2z_SET(1.5597525E38F);
        LoopBackDemoChannel.instance.send(p55); //===============================
        ATTITUDE_QUATERNION_COV p61 = LoopBackDemoChannel.instance.new_ATTITUDE_QUATERNION_COV();
        PH.setPack(p61);
        p61.time_usec_SET(469311529361743115L);
        p61.q_SET(new float[4], 0);
        p61.rollspeed_SET(-1.6084075E38F);
        p61.pitchspeed_SET(-2.4792303E38F);
        p61.yawspeed_SET(-1.4862631E38F);
        p61.covariance_SET(new float[9], 0);
        LoopBackDemoChannel.instance.send(p61); //===============================
        NAV_CONTROLLER_OUTPUT p62 = LoopBackDemoChannel.instance.new_NAV_CONTROLLER_OUTPUT();
        PH.setPack(p62);
        p62.nav_roll_SET(-3.2729668E38F);
        p62.nav_pitch_SET(-4.8352447E36F);
        p62.nav_bearing_SET((short)3057);
        p62.target_bearing_SET((short) -28496);
        p62.wp_dist_SET((char)9918);
        p62.alt_error_SET(-8.819285E37F);
        p62.aspd_error_SET(1.624399E38F);
        p62.xtrack_error_SET(-3.0235014E38F);
        LoopBackDemoChannel.instance.send(p62); //===============================
        GLOBAL_POSITION_INT_COV p63 = LoopBackDemoChannel.instance.new_GLOBAL_POSITION_INT_COV();
        PH.setPack(p63);
        p63.time_usec_SET(267334577424740866L);
        p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS);
        p63.lat_SET(1013070600);
        p63.lon_SET(-598155110);
        p63.alt_SET(-2055040462);
        p63.relative_alt_SET(-1944964068);
        p63.vx_SET(-2.7083438E38F);
        p63.vy_SET(-3.2575907E38F);
        p63.vz_SET(1.4547007E38F);
        p63.covariance_SET(new float[36], 0);
        LoopBackDemoChannel.instance.send(p63); //===============================
        LOCAL_POSITION_NED_COV p64 = LoopBackDemoChannel.instance.new_LOCAL_POSITION_NED_COV();
        PH.setPack(p64);
        p64.time_usec_SET(1399254891996411014L);
        p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION);
        p64.x_SET(-2.5032087E38F);
        p64.y_SET(2.1273157E38F);
        p64.z_SET(-1.6353285E38F);
        p64.vx_SET(2.0176665E38F);
        p64.vy_SET(1.6122716E38F);
        p64.vz_SET(-1.560063E38F);
        p64.ax_SET(2.0368068E38F);
        p64.ay_SET(2.9652905E38F);
        p64.az_SET(-8.160403E37F);
        p64.covariance_SET(new float[45], 0);
        LoopBackDemoChannel.instance.send(p64); //===============================
        RC_CHANNELS p65 = LoopBackDemoChannel.instance.new_RC_CHANNELS();
        PH.setPack(p65);
        p65.time_boot_ms_SET(1791196757L);
        p65.chancount_SET((char)237);
        p65.chan1_raw_SET((char)60696);
        p65.chan2_raw_SET((char)13920);
        p65.chan3_raw_SET((char)36235);
        p65.chan4_raw_SET((char)49180);
        p65.chan5_raw_SET((char)46421);
        p65.chan6_raw_SET((char)46826);
        p65.chan7_raw_SET((char)42366);
        p65.chan8_raw_SET((char)671);
        p65.chan9_raw_SET((char)4791);
        p65.chan10_raw_SET((char)21783);
        p65.chan11_raw_SET((char)50641);
        p65.chan12_raw_SET((char)50309);
        p65.chan13_raw_SET((char)27003);
        p65.chan14_raw_SET((char)53564);
        p65.chan15_raw_SET((char)56435);
        p65.chan16_raw_SET((char)29486);
        p65.chan17_raw_SET((char)65179);
        p65.chan18_raw_SET((char)30310);
        p65.rssi_SET((char)205);
        LoopBackDemoChannel.instance.send(p65); //===============================
        REQUEST_DATA_STREAM p66 = LoopBackDemoChannel.instance.new_REQUEST_DATA_STREAM();
        PH.setPack(p66);
        p66.target_system_SET((char)246);
        p66.target_component_SET((char)197);
        p66.req_stream_id_SET((char)219);
        p66.req_message_rate_SET((char)12029);
        p66.start_stop_SET((char)229);
        LoopBackDemoChannel.instance.send(p66); //===============================
        DATA_STREAM p67 = LoopBackDemoChannel.instance.new_DATA_STREAM();
        PH.setPack(p67);
        p67.stream_id_SET((char)139);
        p67.message_rate_SET((char)13592);
        p67.on_off_SET((char)174);
        LoopBackDemoChannel.instance.send(p67); //===============================
        MANUAL_CONTROL p69 = LoopBackDemoChannel.instance.new_MANUAL_CONTROL();
        PH.setPack(p69);
        p69.target_SET((char)105);
        p69.x_SET((short) -21934);
        p69.y_SET((short)17347);
        p69.z_SET((short)9913);
        p69.r_SET((short)17558);
        p69.buttons_SET((char)47143);
        LoopBackDemoChannel.instance.send(p69); //===============================
        RC_CHANNELS_OVERRIDE p70 = LoopBackDemoChannel.instance.new_RC_CHANNELS_OVERRIDE();
        PH.setPack(p70);
        p70.target_system_SET((char)56);
        p70.target_component_SET((char)152);
        p70.chan1_raw_SET((char)48378);
        p70.chan2_raw_SET((char)56408);
        p70.chan3_raw_SET((char)50600);
        p70.chan4_raw_SET((char)33363);
        p70.chan5_raw_SET((char)33686);
        p70.chan6_raw_SET((char)49520);
        p70.chan7_raw_SET((char)38758);
        p70.chan8_raw_SET((char)39733);
        LoopBackDemoChannel.instance.send(p70); //===============================
        MISSION_ITEM_INT p73 = LoopBackDemoChannel.instance.new_MISSION_ITEM_INT();
        PH.setPack(p73);
        p73.target_system_SET((char)69);
        p73.target_component_SET((char)216);
        p73.seq_SET((char)36635);
        p73.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
        p73.command_SET(MAV_CMD.MAV_CMD_SPATIAL_USER_1);
        p73.current_SET((char)85);
        p73.autocontinue_SET((char)245);
        p73.param1_SET(3.1670504E38F);
        p73.param2_SET(-2.3701582E38F);
        p73.param3_SET(-2.7705206E38F);
        p73.param4_SET(1.2844495E38F);
        p73.x_SET(745071373);
        p73.y_SET(1866400587);
        p73.z_SET(-1.29512E38F);
        p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
        LoopBackDemoChannel.instance.send(p73); //===============================
        VFR_HUD p74 = LoopBackDemoChannel.instance.new_VFR_HUD();
        PH.setPack(p74);
        p74.airspeed_SET(2.0090096E38F);
        p74.groundspeed_SET(-1.0557505E38F);
        p74.heading_SET((short) -30029);
        p74.throttle_SET((char)58824);
        p74.alt_SET(6.2747447E37F);
        p74.climb_SET(-2.9193448E38F);
        LoopBackDemoChannel.instance.send(p74); //===============================
        COMMAND_INT p75 = LoopBackDemoChannel.instance.new_COMMAND_INT();
        PH.setPack(p75);
        p75.target_system_SET((char)45);
        p75.target_component_SET((char)113);
        p75.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
        p75.command_SET(MAV_CMD.MAV_CMD_NAV_LOITER_TURNS);
        p75.current_SET((char)220);
        p75.autocontinue_SET((char)190);
        p75.param1_SET(2.0477082E38F);
        p75.param2_SET(-2.369472E37F);
        p75.param3_SET(-8.751078E37F);
        p75.param4_SET(-6.106586E37F);
        p75.x_SET(1347585608);
        p75.y_SET(1943057067);
        p75.z_SET(-2.0378595E38F);
        LoopBackDemoChannel.instance.send(p75); //===============================
        COMMAND_LONG p76 = LoopBackDemoChannel.instance.new_COMMAND_LONG();
        PH.setPack(p76);
        p76.target_system_SET((char)32);
        p76.target_component_SET((char)220);
        p76.command_SET(MAV_CMD.MAV_CMD_WAYPOINT_USER_1);
        p76.confirmation_SET((char)127);
        p76.param1_SET(-1.2915262E37F);
        p76.param2_SET(1.7209986E38F);
        p76.param3_SET(1.7740105E38F);
        p76.param4_SET(-2.8974735E37F);
        p76.param5_SET(2.7811181E38F);
        p76.param6_SET(-4.0621874E37F);
        p76.param7_SET(5.5698493E37F);
        LoopBackDemoChannel.instance.send(p76); //===============================
        COMMAND_ACK p77 = LoopBackDemoChannel.instance.new_COMMAND_ACK();
        PH.setPack(p77);
        p77.command_SET(MAV_CMD.MAV_CMD_DO_DIGICAM_CONTROL);
        p77.result_SET(MAV_RESULT.MAV_RESULT_ACCEPTED);
        p77.progress_SET((char)7, PH);
        p77.result_param2_SET(-360543587, PH);
        p77.target_system_SET((char)23, PH);
        p77.target_component_SET((char)37, PH);
        LoopBackDemoChannel.instance.send(p77); //===============================
        MANUAL_SETPOINT p81 = LoopBackDemoChannel.instance.new_MANUAL_SETPOINT();
        PH.setPack(p81);
        p81.time_boot_ms_SET(3108239086L);
        p81.roll_SET(2.9075895E38F);
        p81.pitch_SET(2.4934608E38F);
        p81.yaw_SET(-2.2270599E38F);
        p81.thrust_SET(1.0796058E38F);
        p81.mode_switch_SET((char)11);
        p81.manual_override_switch_SET((char)127);
        LoopBackDemoChannel.instance.send(p81); //===============================
        SET_ATTITUDE_TARGET p82 = LoopBackDemoChannel.instance.new_SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.time_boot_ms_SET(2607245579L);
        p82.target_system_SET((char)157);
        p82.target_component_SET((char)37);
        p82.type_mask_SET((char)158);
        p82.q_SET(new float[4], 0);
        p82.body_roll_rate_SET(-2.0053346E38F);
        p82.body_pitch_rate_SET(-4.717766E37F);
        p82.body_yaw_rate_SET(-1.9570503E38F);
        p82.thrust_SET(-3.2905626E38F);
        LoopBackDemoChannel.instance.send(p82); //===============================
        ATTITUDE_TARGET p83 = LoopBackDemoChannel.instance.new_ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.time_boot_ms_SET(1843592854L);
        p83.type_mask_SET((char)190);
        p83.q_SET(new float[4], 0);
        p83.body_roll_rate_SET(2.08666E38F);
        p83.body_pitch_rate_SET(1.5496279E38F);
        p83.body_yaw_rate_SET(3.2940763E38F);
        p83.thrust_SET(-3.103087E38F);
        LoopBackDemoChannel.instance.send(p83); //===============================
        SET_POSITION_TARGET_LOCAL_NED p84 = LoopBackDemoChannel.instance.new_SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.time_boot_ms_SET(3936658695L);
        p84.target_system_SET((char)56);
        p84.target_component_SET((char)9);
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_NED);
        p84.type_mask_SET((char)22537);
        p84.x_SET(2.162367E38F);
        p84.y_SET(-1.1421398E38F);
        p84.z_SET(6.0094153E37F);
        p84.vx_SET(2.0101659E38F);
        p84.vy_SET(2.069809E38F);
        p84.vz_SET(-3.0262008E38F);
        p84.afx_SET(-1.2424858E38F);
        p84.afy_SET(-2.9803526E38F);
        p84.afz_SET(-6.3932984E37F);
        p84.yaw_SET(5.413292E37F);
        p84.yaw_rate_SET(-2.1440079E38F);
        LoopBackDemoChannel.instance.send(p84); //===============================
        SET_POSITION_TARGET_GLOBAL_INT p86 = LoopBackDemoChannel.instance.new_SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.time_boot_ms_SET(624284446L);
        p86.target_system_SET((char)63);
        p86.target_component_SET((char)192);
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
        p86.type_mask_SET((char)13);
        p86.lat_int_SET(1904758316);
        p86.lon_int_SET(-192644792);
        p86.alt_SET(-2.443257E38F);
        p86.vx_SET(1.0942584E38F);
        p86.vy_SET(2.6957302E38F);
        p86.vz_SET(-3.3076576E38F);
        p86.afx_SET(-1.4840157E38F);
        p86.afy_SET(-2.6383014E38F);
        p86.afz_SET(2.8119835E38F);
        p86.yaw_SET(-1.8666443E38F);
        p86.yaw_rate_SET(-2.7838818E38F);
        LoopBackDemoChannel.instance.send(p86); //===============================
        POSITION_TARGET_GLOBAL_INT p87 = LoopBackDemoChannel.instance.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.time_boot_ms_SET(2780126372L);
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
        p87.type_mask_SET((char)30676);
        p87.lat_int_SET(361767318);
        p87.lon_int_SET(1768555602);
        p87.alt_SET(1.7960096E38F);
        p87.vx_SET(3.2337527E38F);
        p87.vy_SET(3.2167283E38F);
        p87.vz_SET(-2.3725476E38F);
        p87.afx_SET(1.7110924E38F);
        p87.afy_SET(3.9651928E37F);
        p87.afz_SET(-7.7262295E37F);
        p87.yaw_SET(-2.9707017E38F);
        p87.yaw_rate_SET(-9.161883E37F);
        LoopBackDemoChannel.instance.send(p87); //===============================
        LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = LoopBackDemoChannel.instance.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.time_boot_ms_SET(3639232148L);
        p89.x_SET(1.8122487E38F);
        p89.y_SET(6.5818985E37F);
        p89.z_SET(-1.2282171E38F);
        p89.roll_SET(-1.203235E38F);
        p89.pitch_SET(-2.4931357E38F);
        p89.yaw_SET(-2.3332985E38F);
        LoopBackDemoChannel.instance.send(p89); //===============================
        HIL_STATE p90 = LoopBackDemoChannel.instance.new_HIL_STATE();
        PH.setPack(p90);
        p90.time_usec_SET(3793585508454927494L);
        p90.roll_SET(6.353233E37F);
        p90.pitch_SET(7.4798885E37F);
        p90.yaw_SET(1.6888902E38F);
        p90.rollspeed_SET(2.68237E38F);
        p90.pitchspeed_SET(6.47999E37F);
        p90.yawspeed_SET(1.8923573E38F);
        p90.lat_SET(518410487);
        p90.lon_SET(-784041300);
        p90.alt_SET(1823332);
        p90.vx_SET((short) -28728);
        p90.vy_SET((short) -24319);
        p90.vz_SET((short) -7313);
        p90.xacc_SET((short)1850);
        p90.yacc_SET((short) -6808);
        p90.zacc_SET((short)15774);
        LoopBackDemoChannel.instance.send(p90); //===============================
        HIL_CONTROLS p91 = LoopBackDemoChannel.instance.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.time_usec_SET(2537834476879582200L);
        p91.roll_ailerons_SET(3.07266E38F);
        p91.pitch_elevator_SET(-3.3515718E38F);
        p91.yaw_rudder_SET(7.2386667E37F);
        p91.throttle_SET(-2.3452869E38F);
        p91.aux1_SET(1.4343088E38F);
        p91.aux2_SET(-2.594061E38F);
        p91.aux3_SET(-1.8198226E38F);
        p91.aux4_SET(-1.1668228E38F);
        p91.mode_SET(MAV_MODE.MAV_MODE_TEST_DISARMED);
        p91.nav_mode_SET((char)64);
        LoopBackDemoChannel.instance.send(p91); //===============================
        HIL_RC_INPUTS_RAW p92 = LoopBackDemoChannel.instance.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.time_usec_SET(3431995524866280052L);
        p92.chan1_raw_SET((char)13653);
        p92.chan2_raw_SET((char)38394);
        p92.chan3_raw_SET((char)16899);
        p92.chan4_raw_SET((char)11089);
        p92.chan5_raw_SET((char)42306);
        p92.chan6_raw_SET((char)58020);
        p92.chan7_raw_SET((char)48805);
        p92.chan8_raw_SET((char)43774);
        p92.chan9_raw_SET((char)52652);
        p92.chan10_raw_SET((char)18953);
        p92.chan11_raw_SET((char)63548);
        p92.chan12_raw_SET((char)18407);
        p92.rssi_SET((char)63);
        LoopBackDemoChannel.instance.send(p92); //===============================
        HIL_ACTUATOR_CONTROLS p93 = LoopBackDemoChannel.instance.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.time_usec_SET(5294827732609016125L);
        p93.controls_SET(new float[16], 0);
        p93.mode_SET(MAV_MODE.MAV_MODE_STABILIZE_ARMED);
        p93.flags_SET(4047521010026909602L);
        LoopBackDemoChannel.instance.send(p93); //===============================
        OPTICAL_FLOW p100 = LoopBackDemoChannel.instance.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.time_usec_SET(4629216413819594850L);
        p100.sensor_id_SET((char)101);
        p100.flow_x_SET((short)6618);
        p100.flow_y_SET((short)7807);
        p100.flow_comp_m_x_SET(-3.7456269E37F);
        p100.flow_comp_m_y_SET(-3.0648964E38F);
        p100.quality_SET((char)154);
        p100.ground_distance_SET(-8.656758E37F);
        p100.flow_rate_x_SET(-3.0554915E38F, PH);
        p100.flow_rate_y_SET(-4.4499505E37F, PH);
        LoopBackDemoChannel.instance.send(p100); //===============================
        GLOBAL_VISION_POSITION_ESTIMATE p101 = LoopBackDemoChannel.instance.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.usec_SET(7810324722590166857L);
        p101.x_SET(7.6509884E37F);
        p101.y_SET(3.053567E38F);
        p101.z_SET(-1.8844332E38F);
        p101.roll_SET(-2.6395832E38F);
        p101.pitch_SET(4.9063494E37F);
        p101.yaw_SET(1.6641735E38F);
        LoopBackDemoChannel.instance.send(p101); //===============================
        VISION_POSITION_ESTIMATE p102 = LoopBackDemoChannel.instance.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.usec_SET(3107851729295301739L);
        p102.x_SET(-1.4245034E38F);
        p102.y_SET(-2.3883367E38F);
        p102.z_SET(2.9972384E38F);
        p102.roll_SET(3.0581606E38F);
        p102.pitch_SET(1.9895322E38F);
        p102.yaw_SET(1.2612004E38F);
        LoopBackDemoChannel.instance.send(p102); //===============================
        VISION_SPEED_ESTIMATE p103 = LoopBackDemoChannel.instance.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.usec_SET(3748004465562172996L);
        p103.x_SET(6.9537187E37F);
        p103.y_SET(3.0447872E38F);
        p103.z_SET(-1.9014974E38F);
        LoopBackDemoChannel.instance.send(p103); //===============================
        VICON_POSITION_ESTIMATE p104 = LoopBackDemoChannel.instance.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.usec_SET(6723248366637241935L);
        p104.x_SET(2.3121645E38F);
        p104.y_SET(-3.2235876E38F);
        p104.z_SET(-3.3676951E38F);
        p104.roll_SET(2.2264768E38F);
        p104.pitch_SET(2.647132E38F);
        p104.yaw_SET(-1.7542067E38F);
        LoopBackDemoChannel.instance.send(p104); //===============================
        HIGHRES_IMU p105 = LoopBackDemoChannel.instance.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.time_usec_SET(8314647882247941106L);
        p105.xacc_SET(3.1732294E37F);
        p105.yacc_SET(-3.4654198E37F);
        p105.zacc_SET(-1.1620152E38F);
        p105.xgyro_SET(3.0949448E38F);
        p105.ygyro_SET(3.1002263E38F);
        p105.zgyro_SET(2.9711168E38F);
        p105.xmag_SET(-3.0531888E38F);
        p105.ymag_SET(-1.7979255E38F);
        p105.zmag_SET(-2.6635989E37F);
        p105.abs_pressure_SET(-9.787779E37F);
        p105.diff_pressure_SET(-3.4448233E37F);
        p105.pressure_alt_SET(1.0167565E38F);
        p105.temperature_SET(-3.1245397E38F);
        p105.fields_updated_SET((char)59701);
        LoopBackDemoChannel.instance.send(p105); //===============================
        OPTICAL_FLOW_RAD p106 = LoopBackDemoChannel.instance.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.time_usec_SET(2991490242333807187L);
        p106.sensor_id_SET((char)3);
        p106.integration_time_us_SET(2260608490L);
        p106.integrated_x_SET(-3.3398123E38F);
        p106.integrated_y_SET(-2.5251225E38F);
        p106.integrated_xgyro_SET(3.2977786E37F);
        p106.integrated_ygyro_SET(-1.1247433E37F);
        p106.integrated_zgyro_SET(-1.2602641E37F);
        p106.temperature_SET((short) -25878);
        p106.quality_SET((char)3);
        p106.time_delta_distance_us_SET(1999139762L);
        p106.distance_SET(-1.3115605E37F);
        LoopBackDemoChannel.instance.send(p106); //===============================
        HIL_SENSOR p107 = LoopBackDemoChannel.instance.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.time_usec_SET(703961206572604471L);
        p107.xacc_SET(-2.1131042E38F);
        p107.yacc_SET(3.136176E38F);
        p107.zacc_SET(-2.0914004E37F);
        p107.xgyro_SET(3.1281684E38F);
        p107.ygyro_SET(4.8196826E37F);
        p107.zgyro_SET(6.9449856E37F);
        p107.xmag_SET(-1.3336802E38F);
        p107.ymag_SET(-6.8927533E37F);
        p107.zmag_SET(9.718945E37F);
        p107.abs_pressure_SET(2.8506758E38F);
        p107.diff_pressure_SET(-1.6271273E37F);
        p107.pressure_alt_SET(-8.582248E37F);
        p107.temperature_SET(3.4022022E38F);
        p107.fields_updated_SET(4238872903L);
        LoopBackDemoChannel.instance.send(p107); //===============================
        SIM_STATE p108 = LoopBackDemoChannel.instance.new_SIM_STATE();
        PH.setPack(p108);
        p108.q1_SET(-2.024427E38F);
        p108.q2_SET(-2.340257E38F);
        p108.q3_SET(1.4927886E38F);
        p108.q4_SET(3.1679053E38F);
        p108.roll_SET(1.4132753E38F);
        p108.pitch_SET(-1.1425481E38F);
        p108.yaw_SET(2.2488E38F);
        p108.xacc_SET(-9.326213E37F);
        p108.yacc_SET(-1.2486463E38F);
        p108.zacc_SET(-3.213176E38F);
        p108.xgyro_SET(-3.0779195E38F);
        p108.ygyro_SET(-2.5002163E38F);
        p108.zgyro_SET(-2.2943099E38F);
        p108.lat_SET(-3.3633902E37F);
        p108.lon_SET(3.0864617E38F);
        p108.alt_SET(-1.8321553E38F);
        p108.std_dev_horz_SET(-2.2001555E38F);
        p108.std_dev_vert_SET(-3.0927705E38F);
        p108.vn_SET(-3.2996759E38F);
        p108.ve_SET(-8.773437E37F);
        p108.vd_SET(2.9131211E38F);
        LoopBackDemoChannel.instance.send(p108); //===============================
        RADIO_STATUS p109 = LoopBackDemoChannel.instance.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.rssi_SET((char)207);
        p109.remrssi_SET((char)243);
        p109.txbuf_SET((char)28);
        p109.noise_SET((char)247);
        p109.remnoise_SET((char)203);
        p109.rxerrors_SET((char)2793);
        p109.fixed__SET((char)36910);
        LoopBackDemoChannel.instance.send(p109); //===============================
        FILE_TRANSFER_PROTOCOL p110 = LoopBackDemoChannel.instance.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.target_network_SET((char)200);
        p110.target_system_SET((char)226);
        p110.target_component_SET((char)232);
        p110.payload_SET(new char[251], 0);
        LoopBackDemoChannel.instance.send(p110); //===============================
        TIMESYNC p111 = LoopBackDemoChannel.instance.new_TIMESYNC();
        PH.setPack(p111);
        p111.tc1_SET(-6837028652259643853L);
        p111.ts1_SET(7224212698768013550L);
        LoopBackDemoChannel.instance.send(p111); //===============================
        CAMERA_TRIGGER p112 = LoopBackDemoChannel.instance.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.time_usec_SET(6410997153556810604L);
        p112.seq_SET(952713739L);
        LoopBackDemoChannel.instance.send(p112); //===============================
        HIL_GPS p113 = LoopBackDemoChannel.instance.new_HIL_GPS();
        PH.setPack(p113);
        p113.time_usec_SET(6694953521889955166L);
        p113.fix_type_SET((char)171);
        p113.lat_SET(-1802516115);
        p113.lon_SET(1521535075);
        p113.alt_SET(-933652531);
        p113.eph_SET((char)24979);
        p113.epv_SET((char)61041);
        p113.vel_SET((char)31770);
        p113.vn_SET((short)16483);
        p113.ve_SET((short) -18621);
        p113.vd_SET((short) -30204);
        p113.cog_SET((char)6193);
        p113.satellites_visible_SET((char)195);
        LoopBackDemoChannel.instance.send(p113); //===============================
        HIL_OPTICAL_FLOW p114 = LoopBackDemoChannel.instance.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.time_usec_SET(7066640334946089907L);
        p114.sensor_id_SET((char)73);
        p114.integration_time_us_SET(3999275582L);
        p114.integrated_x_SET(-3.957148E37F);
        p114.integrated_y_SET(-1.8344133E38F);
        p114.integrated_xgyro_SET(-1.1259277E38F);
        p114.integrated_ygyro_SET(-3.2055113E38F);
        p114.integrated_zgyro_SET(-2.390913E38F);
        p114.temperature_SET((short)28611);
        p114.quality_SET((char)118);
        p114.time_delta_distance_us_SET(459416880L);
        p114.distance_SET(-1.0499071E38F);
        LoopBackDemoChannel.instance.send(p114); //===============================
        HIL_STATE_QUATERNION p115 = LoopBackDemoChannel.instance.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.time_usec_SET(2008504516235075732L);
        p115.attitude_quaternion_SET(new float[4], 0);
        p115.rollspeed_SET(-1.8524695E38F);
        p115.pitchspeed_SET(-1.855861E38F);
        p115.yawspeed_SET(-3.0155684E38F);
        p115.lat_SET(-139636020);
        p115.lon_SET(1421166716);
        p115.alt_SET(-274005206);
        p115.vx_SET((short)3540);
        p115.vy_SET((short)30138);
        p115.vz_SET((short)31841);
        p115.ind_airspeed_SET((char)41936);
        p115.true_airspeed_SET((char)56195);
        p115.xacc_SET((short) -30970);
        p115.yacc_SET((short)28488);
        p115.zacc_SET((short) -7775);
        LoopBackDemoChannel.instance.send(p115); //===============================
        SCALED_IMU2 p116 = LoopBackDemoChannel.instance.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.time_boot_ms_SET(3289894401L);
        p116.xacc_SET((short)859);
        p116.yacc_SET((short)13335);
        p116.zacc_SET((short)3418);
        p116.xgyro_SET((short)7043);
        p116.ygyro_SET((short) -28434);
        p116.zgyro_SET((short) -8339);
        p116.xmag_SET((short) -15922);
        p116.ymag_SET((short)21982);
        p116.zmag_SET((short) -25527);
        LoopBackDemoChannel.instance.send(p116); //===============================
        LOG_REQUEST_LIST p117 = LoopBackDemoChannel.instance.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.target_system_SET((char)14);
        p117.target_component_SET((char)164);
        p117.start_SET((char)52282);
        p117.end_SET((char)48055);
        LoopBackDemoChannel.instance.send(p117); //===============================
        LOG_ENTRY p118 = LoopBackDemoChannel.instance.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.id_SET((char)5539);
        p118.num_logs_SET((char)51994);
        p118.last_log_num_SET((char)25553);
        p118.time_utc_SET(3140801533L);
        p118.size_SET(3087992659L);
        LoopBackDemoChannel.instance.send(p118); //===============================
        LOG_REQUEST_DATA p119 = LoopBackDemoChannel.instance.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.target_system_SET((char)184);
        p119.target_component_SET((char)127);
        p119.id_SET((char)31869);
        p119.ofs_SET(933157014L);
        p119.count_SET(3139188577L);
        LoopBackDemoChannel.instance.send(p119); //===============================
        LOG_DATA p120 = LoopBackDemoChannel.instance.new_LOG_DATA();
        PH.setPack(p120);
        p120.id_SET((char)38828);
        p120.ofs_SET(446953231L);
        p120.count_SET((char)116);
        p120.data__SET(new char[90], 0);
        LoopBackDemoChannel.instance.send(p120); //===============================
        LOG_ERASE p121 = LoopBackDemoChannel.instance.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_system_SET((char)71);
        p121.target_component_SET((char)92);
        LoopBackDemoChannel.instance.send(p121); //===============================
        LOG_REQUEST_END p122 = LoopBackDemoChannel.instance.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_system_SET((char)54);
        p122.target_component_SET((char)226);
        LoopBackDemoChannel.instance.send(p122); //===============================
        GPS_INJECT_DATA p123 = LoopBackDemoChannel.instance.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.target_system_SET((char)138);
        p123.target_component_SET((char)171);
        p123.len_SET((char)98);
        p123.data__SET(new char[110], 0);
        LoopBackDemoChannel.instance.send(p123); //===============================
        GPS2_RAW p124 = LoopBackDemoChannel.instance.new_GPS2_RAW();
        PH.setPack(p124);
        p124.time_usec_SET(6452457335335231662L);
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX);
        p124.lat_SET(1959855607);
        p124.lon_SET(-2110996354);
        p124.alt_SET(-998149357);
        p124.eph_SET((char)58603);
        p124.epv_SET((char)31845);
        p124.vel_SET((char)25434);
        p124.cog_SET((char)56783);
        p124.satellites_visible_SET((char)205);
        p124.dgps_numch_SET((char)169);
        p124.dgps_age_SET(3080764260L);
        LoopBackDemoChannel.instance.send(p124); //===============================
        POWER_STATUS p125 = LoopBackDemoChannel.instance.new_POWER_STATUS();
        PH.setPack(p125);
        p125.Vcc_SET((char)43121);
        p125.Vservo_SET((char)34911);
        p125.flags_SET(MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED);
        LoopBackDemoChannel.instance.send(p125); //===============================
        SERIAL_CONTROL p126 = LoopBackDemoChannel.instance.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_SHELL);
        p126.flags_SET(SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI);
        p126.timeout_SET((char)8218);
        p126.baudrate_SET(3372455529L);
        p126.count_SET((char)221);
        p126.data__SET(new char[70], 0);
        LoopBackDemoChannel.instance.send(p126); //===============================
        GPS_RTK p127 = LoopBackDemoChannel.instance.new_GPS_RTK();
        PH.setPack(p127);
        p127.time_last_baseline_ms_SET(3626153731L);
        p127.rtk_receiver_id_SET((char)96);
        p127.wn_SET((char)9616);
        p127.tow_SET(2119079674L);
        p127.rtk_health_SET((char)65);
        p127.rtk_rate_SET((char)98);
        p127.nsats_SET((char)4);
        p127.baseline_coords_type_SET((char)20);
        p127.baseline_a_mm_SET(992985185);
        p127.baseline_b_mm_SET(654928837);
        p127.baseline_c_mm_SET(1574852792);
        p127.accuracy_SET(701390724L);
        p127.iar_num_hypotheses_SET(2021924252);
        LoopBackDemoChannel.instance.send(p127); //===============================
        GPS2_RTK p128 = LoopBackDemoChannel.instance.new_GPS2_RTK();
        PH.setPack(p128);
        p128.time_last_baseline_ms_SET(852533447L);
        p128.rtk_receiver_id_SET((char)200);
        p128.wn_SET((char)25985);
        p128.tow_SET(2137994243L);
        p128.rtk_health_SET((char)151);
        p128.rtk_rate_SET((char)220);
        p128.nsats_SET((char)41);
        p128.baseline_coords_type_SET((char)140);
        p128.baseline_a_mm_SET(-1160606200);
        p128.baseline_b_mm_SET(1386935207);
        p128.baseline_c_mm_SET(1094776426);
        p128.accuracy_SET(837629826L);
        p128.iar_num_hypotheses_SET(790517292);
        LoopBackDemoChannel.instance.send(p128); //===============================
        SCALED_IMU3 p129 = LoopBackDemoChannel.instance.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.time_boot_ms_SET(805793373L);
        p129.xacc_SET((short) -31476);
        p129.yacc_SET((short) -14174);
        p129.zacc_SET((short) -1560);
        p129.xgyro_SET((short)5688);
        p129.ygyro_SET((short)17717);
        p129.zgyro_SET((short)24854);
        p129.xmag_SET((short) -23726);
        p129.ymag_SET((short) -8173);
        p129.zmag_SET((short) -18000);
        LoopBackDemoChannel.instance.send(p129); //===============================
        DATA_TRANSMISSION_HANDSHAKE p130 = LoopBackDemoChannel.instance.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.type_SET((char)30);
        p130.size_SET(3794832372L);
        p130.width_SET((char)10873);
        p130.height_SET((char)23468);
        p130.packets_SET((char)25544);
        p130.payload_SET((char)199);
        p130.jpg_quality_SET((char)122);
        LoopBackDemoChannel.instance.send(p130); //===============================
        ENCAPSULATED_DATA p131 = LoopBackDemoChannel.instance.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.seqnr_SET((char)776);
        p131.data__SET(new char[253], 0);
        LoopBackDemoChannel.instance.send(p131); //===============================
        DISTANCE_SENSOR p132 = LoopBackDemoChannel.instance.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.time_boot_ms_SET(4217516749L);
        p132.min_distance_SET((char)35853);
        p132.max_distance_SET((char)9462);
        p132.current_distance_SET((char)60408);
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED);
        p132.id_SET((char)82);
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_YAW_45);
        p132.covariance_SET((char)83);
        LoopBackDemoChannel.instance.send(p132); //===============================
        TERRAIN_REQUEST p133 = LoopBackDemoChannel.instance.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.lat_SET(-1014896972);
        p133.lon_SET(-224941338);
        p133.grid_spacing_SET((char)58619);
        p133.mask_SET(6454650288012095261L);
        LoopBackDemoChannel.instance.send(p133); //===============================
        TERRAIN_DATA p134 = LoopBackDemoChannel.instance.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.lat_SET(1147525822);
        p134.lon_SET(2123147319);
        p134.grid_spacing_SET((char)41430);
        p134.gridbit_SET((char)116);
        p134.data__SET(new short[16], 0);
        LoopBackDemoChannel.instance.send(p134); //===============================
        TERRAIN_CHECK p135 = LoopBackDemoChannel.instance.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lat_SET(936949415);
        p135.lon_SET(701429250);
        LoopBackDemoChannel.instance.send(p135); //===============================
        TERRAIN_REPORT p136 = LoopBackDemoChannel.instance.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.lat_SET(1653683649);
        p136.lon_SET(-1875274202);
        p136.spacing_SET((char)32729);
        p136.terrain_height_SET(-2.791221E38F);
        p136.current_height_SET(3.3865886E38F);
        p136.pending_SET((char)53421);
        p136.loaded_SET((char)38779);
        LoopBackDemoChannel.instance.send(p136); //===============================
        SCALED_PRESSURE2 p137 = LoopBackDemoChannel.instance.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.time_boot_ms_SET(3965188220L);
        p137.press_abs_SET(-2.8926592E38F);
        p137.press_diff_SET(-2.2543864E38F);
        p137.temperature_SET((short)23210);
        LoopBackDemoChannel.instance.send(p137); //===============================
        ATT_POS_MOCAP p138 = LoopBackDemoChannel.instance.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.time_usec_SET(1410832182122902918L);
        p138.q_SET(new float[4], 0);
        p138.x_SET(-8.3000153E37F);
        p138.y_SET(4.4383454E37F);
        p138.z_SET(-1.3932605E37F);
        LoopBackDemoChannel.instance.send(p138); //===============================
        SET_ACTUATOR_CONTROL_TARGET p139 = LoopBackDemoChannel.instance.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.time_usec_SET(6508947962158849096L);
        p139.group_mlx_SET((char)102);
        p139.target_system_SET((char)212);
        p139.target_component_SET((char)58);
        p139.controls_SET(new float[8], 0);
        LoopBackDemoChannel.instance.send(p139); //===============================
        ACTUATOR_CONTROL_TARGET p140 = LoopBackDemoChannel.instance.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.time_usec_SET(8548447767527627081L);
        p140.group_mlx_SET((char)198);
        p140.controls_SET(new float[8], 0);
        LoopBackDemoChannel.instance.send(p140); //===============================
        ALTITUDE p141 = LoopBackDemoChannel.instance.new_ALTITUDE();
        PH.setPack(p141);
        p141.time_usec_SET(3910294414835319878L);
        p141.altitude_monotonic_SET(2.3756095E38F);
        p141.altitude_amsl_SET(3.1194356E38F);
        p141.altitude_local_SET(3.0105144E38F);
        p141.altitude_relative_SET(-3.0677844E37F);
        p141.altitude_terrain_SET(1.1974575E38F);
        p141.bottom_clearance_SET(-2.4153147E38F);
        LoopBackDemoChannel.instance.send(p141); //===============================
        RESOURCE_REQUEST p142 = LoopBackDemoChannel.instance.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.request_id_SET((char)122);
        p142.uri_type_SET((char)201);
        p142.uri_SET(new char[120], 0);
        p142.transfer_type_SET((char)190);
        p142.storage_SET(new char[120], 0);
        LoopBackDemoChannel.instance.send(p142); //===============================
        SCALED_PRESSURE3 p143 = LoopBackDemoChannel.instance.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.time_boot_ms_SET(1043605196L);
        p143.press_abs_SET(-3.1796835E38F);
        p143.press_diff_SET(-5.9276184E37F);
        p143.temperature_SET((short)24207);
        LoopBackDemoChannel.instance.send(p143); //===============================
        FOLLOW_TARGET p144 = LoopBackDemoChannel.instance.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.timestamp_SET(6694535760639728067L);
        p144.est_capabilities_SET((char)238);
        p144.lat_SET(-937271598);
        p144.lon_SET(-979052735);
        p144.alt_SET(-1.910223E38F);
        p144.vel_SET(new float[3], 0);
        p144.acc_SET(new float[3], 0);
        p144.attitude_q_SET(new float[4], 0);
        p144.rates_SET(new float[3], 0);
        p144.position_cov_SET(new float[3], 0);
        p144.custom_state_SET(1097858694741770510L);
        LoopBackDemoChannel.instance.send(p144); //===============================
        CONTROL_SYSTEM_STATE p146 = LoopBackDemoChannel.instance.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.time_usec_SET(1591265052032067889L);
        p146.x_acc_SET(3.8006525E37F);
        p146.y_acc_SET(1.9967842E37F);
        p146.z_acc_SET(1.974442E38F);
        p146.x_vel_SET(9.481278E37F);
        p146.y_vel_SET(-1.5987732E38F);
        p146.z_vel_SET(1.2448126E38F);
        p146.x_pos_SET(-5.5872816E37F);
        p146.y_pos_SET(3.1273224E38F);
        p146.z_pos_SET(-1.1994697E38F);
        p146.airspeed_SET(1.902033E38F);
        p146.vel_variance_SET(new float[3], 0);
        p146.pos_variance_SET(new float[3], 0);
        p146.q_SET(new float[4], 0);
        p146.roll_rate_SET(-1.85811E38F);
        p146.pitch_rate_SET(2.7681445E38F);
        p146.yaw_rate_SET(-2.8857985E38F);
        LoopBackDemoChannel.instance.send(p146); //===============================
        BATTERY_STATUS p147 = LoopBackDemoChannel.instance.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.id_SET((char)92);
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_PROPULSION);
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LION);
        p147.temperature_SET((short)32414);
        p147.voltages_SET(new char[10], 0);
        p147.current_battery_SET((short)10795);
        p147.current_consumed_SET(-259672412);
        p147.energy_consumed_SET(-1746630258);
        p147.battery_remaining_SET((byte) - 93);
        LoopBackDemoChannel.instance.send(p147); //===============================
        AUTOPILOT_VERSION p148 = LoopBackDemoChannel.instance.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.capabilities_SET(MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MAVLINK2);
        p148.flight_sw_version_SET(884961292L);
        p148.middleware_sw_version_SET(1379463707L);
        p148.os_sw_version_SET(959570168L);
        p148.board_version_SET(303030460L);
        p148.flight_custom_version_SET(new char[8], 0);
        p148.middleware_custom_version_SET(new char[8], 0);
        p148.os_custom_version_SET(new char[8], 0);
        p148.vendor_id_SET((char)38355);
        p148.product_id_SET((char)18893);
        p148.uid_SET(6545447233628866874L);
        p148.uid2_SET(new char[18], 0, PH);
        LoopBackDemoChannel.instance.send(p148); //===============================
        LANDING_TARGET p149 = LoopBackDemoChannel.instance.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.time_usec_SET(555689480565763765L);
        p149.target_num_SET((char)41);
        p149.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
        p149.angle_x_SET(8.1997756E37F);
        p149.angle_y_SET(-1.7313748E38F);
        p149.distance_SET(3.1818014E38F);
        p149.size_x_SET(3.1378634E38F);
        p149.size_y_SET(1.4063531E38F);
        p149.x_SET(-1.431292E38F, PH);
        p149.y_SET(1.106076E38F, PH);
        p149.z_SET(2.7678778E38F, PH);
        p149.q_SET(new float[4], 0, PH);
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON);
        p149.position_valid_SET((char)235, PH);
        LoopBackDemoChannel.instance.send(p149); //===============================
        ARRAY_TEST_0 p150 = LoopBackDemoChannel.instance.new_ARRAY_TEST_0();
        PH.setPack(p150);
        p150.v1_SET((char)185);
        p150.ar_i8_SET(new byte[4], 0);
        p150.ar_u8_SET(new char[4], 0);
        p150.ar_u16_SET(new char[4], 0);
        p150.ar_u32_SET(new long[4], 0);
        LoopBackDemoChannel.instance.send(p150); //===============================
        ARRAY_TEST_1 p151 = LoopBackDemoChannel.instance.new_ARRAY_TEST_1();
        PH.setPack(p151);
        p151.ar_u32_SET(new long[4], 0);
        LoopBackDemoChannel.instance.send(p151); //===============================
        ARRAY_TEST_3 p153 = LoopBackDemoChannel.instance.new_ARRAY_TEST_3();
        PH.setPack(p153);
        p153.v_SET((char)244);
        p153.ar_u32_SET(new long[4], 0);
        LoopBackDemoChannel.instance.send(p153); //===============================
        ARRAY_TEST_4 p154 = LoopBackDemoChannel.instance.new_ARRAY_TEST_4();
        PH.setPack(p154);
        p154.ar_u32_SET(new long[4], 0);
        p154.v_SET((char)134);
        LoopBackDemoChannel.instance.send(p154); //===============================
        ARRAY_TEST_5 p155 = LoopBackDemoChannel.instance.new_ARRAY_TEST_5();
        PH.setPack(p155);
        p155.c1_SET("DEMO", PH);
        p155.c2_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p155); //===============================
        ARRAY_TEST_6 p156 = LoopBackDemoChannel.instance.new_ARRAY_TEST_6();
        PH.setPack(p156);
        p156.v1_SET((char)254);
        p156.v2_SET((char)33346);
        p156.v3_SET(1893384061L);
        p156.ar_u32_SET(new long[2], 0);
        p156.ar_i32_SET(new int[2], 0);
        p156.ar_u16_SET(new char[2], 0);
        p156.ar_i16_SET(new short[2], 0);
        p156.ar_u8_SET(new char[2], 0);
        p156.ar_i8_SET(new byte[2], 0);
        p156.ar_c_SET("DEMO", PH);
        p156.ar_d_SET(new double[2], 0);
        p156.ar_f_SET(new float[2], 0);
        LoopBackDemoChannel.instance.send(p156); //===============================
        ARRAY_TEST_7 p157 = LoopBackDemoChannel.instance.new_ARRAY_TEST_7();
        PH.setPack(p157);
        p157.ar_d_SET(new double[2], 0);
        p157.ar_f_SET(new float[2], 0);
        p157.ar_u32_SET(new long[2], 0);
        p157.ar_i32_SET(new int[2], 0);
        p157.ar_u16_SET(new char[2], 0);
        p157.ar_i16_SET(new short[2], 0);
        p157.ar_u8_SET(new char[2], 0);
        p157.ar_i8_SET(new byte[2], 0);
        p157.ar_c_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p157); //===============================
        ARRAY_TEST_8 p158 = LoopBackDemoChannel.instance.new_ARRAY_TEST_8();
        PH.setPack(p158);
        p158.v3_SET(4153240964L);
        p158.ar_d_SET(new double[2], 0);
        p158.ar_u16_SET(new char[2], 0);
        LoopBackDemoChannel.instance.send(p158); //===============================
        ESTIMATOR_STATUS p230 = LoopBackDemoChannel.instance.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.time_usec_SET(8290819445924808940L);
        p230.flags_SET(ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL);
        p230.vel_ratio_SET(5.128357E37F);
        p230.pos_horiz_ratio_SET(-1.6268109E38F);
        p230.pos_vert_ratio_SET(-1.9625452E38F);
        p230.mag_ratio_SET(-3.1402066E38F);
        p230.hagl_ratio_SET(-2.8441098E38F);
        p230.tas_ratio_SET(1.8007778E38F);
        p230.pos_horiz_accuracy_SET(-4.2953545E37F);
        p230.pos_vert_accuracy_SET(2.738992E38F);
        LoopBackDemoChannel.instance.send(p230); //===============================
        WIND_COV p231 = LoopBackDemoChannel.instance.new_WIND_COV();
        PH.setPack(p231);
        p231.time_usec_SET(434135087120919152L);
        p231.wind_x_SET(1.6403757E38F);
        p231.wind_y_SET(3.3099824E38F);
        p231.wind_z_SET(1.7278556E38F);
        p231.var_horiz_SET(1.8242273E38F);
        p231.var_vert_SET(1.4793379E38F);
        p231.wind_alt_SET(-2.4516519E38F);
        p231.horiz_accuracy_SET(-2.4681721E38F);
        p231.vert_accuracy_SET(-1.2374846E38F);
        LoopBackDemoChannel.instance.send(p231); //===============================
        GPS_INPUT p232 = LoopBackDemoChannel.instance.new_GPS_INPUT();
        PH.setPack(p232);
        p232.time_usec_SET(2044849950973873501L);
        p232.gps_id_SET((char)28);
        p232.ignore_flags_SET(GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY);
        p232.time_week_ms_SET(3215300602L);
        p232.time_week_SET((char)62421);
        p232.fix_type_SET((char)44);
        p232.lat_SET(1273490659);
        p232.lon_SET(885201716);
        p232.alt_SET(-2.5732622E38F);
        p232.hdop_SET(-3.2791753E37F);
        p232.vdop_SET(3.2238517E38F);
        p232.vn_SET(-1.6625738E38F);
        p232.ve_SET(-4.286522E37F);
        p232.vd_SET(-3.9783934E37F);
        p232.speed_accuracy_SET(-2.2958765E38F);
        p232.horiz_accuracy_SET(-2.905322E38F);
        p232.vert_accuracy_SET(1.3267594E38F);
        p232.satellites_visible_SET((char)82);
        LoopBackDemoChannel.instance.send(p232); //===============================
        GPS_RTCM_DATA p233 = LoopBackDemoChannel.instance.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.flags_SET((char)210);
        p233.len_SET((char)248);
        p233.data__SET(new char[180], 0);
        LoopBackDemoChannel.instance.send(p233); //===============================
        HIGH_LATENCY p234 = LoopBackDemoChannel.instance.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED);
        p234.custom_mode_SET(3642813310L);
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED);
        p234.roll_SET((short)8570);
        p234.pitch_SET((short) -26071);
        p234.heading_SET((char)58064);
        p234.throttle_SET((byte) - 83);
        p234.heading_sp_SET((short) -4490);
        p234.latitude_SET(1111822294);
        p234.longitude_SET(-374382677);
        p234.altitude_amsl_SET((short)18879);
        p234.altitude_sp_SET((short)4430);
        p234.airspeed_SET((char)12);
        p234.airspeed_sp_SET((char)88);
        p234.groundspeed_SET((char)120);
        p234.climb_rate_SET((byte) - 117);
        p234.gps_nsat_SET((char)133);
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX);
        p234.battery_remaining_SET((char)161);
        p234.temperature_SET((byte) - 2);
        p234.temperature_air_SET((byte)36);
        p234.failsafe_SET((char)228);
        p234.wp_num_SET((char)80);
        p234.wp_distance_SET((char)11548);
        LoopBackDemoChannel.instance.send(p234); //===============================
        VIBRATION p241 = LoopBackDemoChannel.instance.new_VIBRATION();
        PH.setPack(p241);
        p241.time_usec_SET(5663715528435500493L);
        p241.vibration_x_SET(-1.8663178E38F);
        p241.vibration_y_SET(9.995235E37F);
        p241.vibration_z_SET(3.2558054E38F);
        p241.clipping_0_SET(4181269126L);
        p241.clipping_1_SET(487326614L);
        p241.clipping_2_SET(3519677062L);
        LoopBackDemoChannel.instance.send(p241); //===============================
        HOME_POSITION p242 = LoopBackDemoChannel.instance.new_HOME_POSITION();
        PH.setPack(p242);
        p242.latitude_SET(-1083059205);
        p242.longitude_SET(-2088455469);
        p242.altitude_SET(1436477312);
        p242.x_SET(1.0351608E38F);
        p242.y_SET(3.1866135E37F);
        p242.z_SET(-3.7879887E37F);
        p242.q_SET(new float[4], 0);
        p242.approach_x_SET(9.159257E37F);
        p242.approach_y_SET(-1.615366E38F);
        p242.approach_z_SET(4.3234672E36F);
        p242.time_usec_SET(6161433644030350887L, PH);
        LoopBackDemoChannel.instance.send(p242); //===============================
        SET_HOME_POSITION p243 = LoopBackDemoChannel.instance.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.target_system_SET((char)243);
        p243.latitude_SET(1350741095);
        p243.longitude_SET(-1519662523);
        p243.altitude_SET(2106809864);
        p243.x_SET(7.5087473E37F);
        p243.y_SET(-1.5244078E38F);
        p243.z_SET(-1.315142E38F);
        p243.q_SET(new float[4], 0);
        p243.approach_x_SET(6.6791633E37F);
        p243.approach_y_SET(2.043913E38F);
        p243.approach_z_SET(1.2917783E38F);
        p243.time_usec_SET(8571718146745670583L, PH);
        LoopBackDemoChannel.instance.send(p243); //===============================
        MESSAGE_INTERVAL p244 = LoopBackDemoChannel.instance.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.message_id_SET((char)30408);
        p244.interval_us_SET(312913835);
        LoopBackDemoChannel.instance.send(p244); //===============================
        EXTENDED_SYS_STATE p245 = LoopBackDemoChannel.instance.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_FW);
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND);
        LoopBackDemoChannel.instance.send(p245); //===============================
        ADSB_VEHICLE p246 = LoopBackDemoChannel.instance.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.ICAO_address_SET(3042393306L);
        p246.lat_SET(-484730692);
        p246.lon_SET(-1322396419);
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
        p246.altitude_SET(-662056536);
        p246.heading_SET((char)44897);
        p246.hor_velocity_SET((char)24154);
        p246.ver_velocity_SET((short) -6806);
        p246.callsign_SET("DEMO", PH);
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_HEAVY);
        p246.tslc_SET((char)56);
        p246.flags_SET(ADSB_FLAGS.ADSB_FLAGS_SIMULATED);
        p246.squawk_SET((char)20893);
        LoopBackDemoChannel.instance.send(p246); //===============================
        COLLISION p247 = LoopBackDemoChannel.instance.new_COLLISION();
        PH.setPack(p247);
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
        p247.id_SET(170338010L);
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_RTL);
        p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH);
        p247.time_to_minimum_delta_SET(2.5313705E38F);
        p247.altitude_minimum_delta_SET(-1.1254149E38F);
        p247.horizontal_minimum_delta_SET(5.48811E37F);
        LoopBackDemoChannel.instance.send(p247); //===============================
        V2_EXTENSION p248 = LoopBackDemoChannel.instance.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.target_network_SET((char)230);
        p248.target_system_SET((char)22);
        p248.target_component_SET((char)152);
        p248.message_type_SET((char)63946);
        p248.payload_SET(new char[249], 0);
        LoopBackDemoChannel.instance.send(p248); //===============================
        MEMORY_VECT p249 = LoopBackDemoChannel.instance.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.address_SET((char)1182);
        p249.ver_SET((char)153);
        p249.type_SET((char)54);
        p249.value_SET(new byte[32], 0);
        LoopBackDemoChannel.instance.send(p249); //===============================
        DEBUG_VECT p250 = LoopBackDemoChannel.instance.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.name_SET("DEMO", PH);
        p250.time_usec_SET(2397140473368534945L);
        p250.x_SET(1.8790306E38F);
        p250.y_SET(-1.4350061E38F);
        p250.z_SET(-1.0307592E38F);
        LoopBackDemoChannel.instance.send(p250); //===============================
        NAMED_VALUE_FLOAT p251 = LoopBackDemoChannel.instance.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.time_boot_ms_SET(3361804703L);
        p251.name_SET("DEMO", PH);
        p251.value_SET(2.3064434E38F);
        LoopBackDemoChannel.instance.send(p251); //===============================
        NAMED_VALUE_INT p252 = LoopBackDemoChannel.instance.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.time_boot_ms_SET(3216786893L);
        p252.name_SET("DEMO", PH);
        p252.value_SET(387093947);
        LoopBackDemoChannel.instance.send(p252); //===============================
        STATUSTEXT p253 = LoopBackDemoChannel.instance.new_STATUSTEXT();
        PH.setPack(p253);
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_WARNING);
        p253.text_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p253); //===============================
        DEBUG p254 = LoopBackDemoChannel.instance.new_DEBUG();
        PH.setPack(p254);
        p254.time_boot_ms_SET(1542738402L);
        p254.ind_SET((char)84);
        p254.value_SET(1.4784915E38F);
        LoopBackDemoChannel.instance.send(p254); //===============================
        SETUP_SIGNING p256 = LoopBackDemoChannel.instance.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.target_system_SET((char)74);
        p256.target_component_SET((char)141);
        p256.secret_key_SET(new char[32], 0);
        p256.initial_timestamp_SET(1703250250721743437L);
        LoopBackDemoChannel.instance.send(p256); //===============================
        BUTTON_CHANGE p257 = LoopBackDemoChannel.instance.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.time_boot_ms_SET(1417313867L);
        p257.last_change_ms_SET(3685844325L);
        p257.state_SET((char)55);
        LoopBackDemoChannel.instance.send(p257); //===============================
        PLAY_TUNE p258 = LoopBackDemoChannel.instance.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.target_system_SET((char)23);
        p258.target_component_SET((char)44);
        p258.tune_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p258); //===============================
        CAMERA_INFORMATION p259 = LoopBackDemoChannel.instance.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.time_boot_ms_SET(1906949138L);
        p259.vendor_name_SET(new char[32], 0);
        p259.model_name_SET(new char[32], 0);
        p259.firmware_version_SET(25762892L);
        p259.focal_length_SET(1.3903532E38F);
        p259.sensor_size_h_SET(-9.077426E37F);
        p259.sensor_size_v_SET(2.397084E38F);
        p259.resolution_h_SET((char)37897);
        p259.resolution_v_SET((char)1126);
        p259.lens_id_SET((char)56);
        p259.flags_SET(CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE);
        p259.cam_definition_version_SET((char)16294);
        p259.cam_definition_uri_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p259); //===============================
        CAMERA_SETTINGS p260 = LoopBackDemoChannel.instance.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.time_boot_ms_SET(227750948L);
        p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY);
        LoopBackDemoChannel.instance.send(p260); //===============================
        STORAGE_INFORMATION p261 = LoopBackDemoChannel.instance.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.time_boot_ms_SET(346401149L);
        p261.storage_id_SET((char)157);
        p261.storage_count_SET((char)218);
        p261.status_SET((char)151);
        p261.total_capacity_SET(-3.0688789E38F);
        p261.used_capacity_SET(1.8523789E38F);
        p261.available_capacity_SET(-1.9591284E38F);
        p261.read_speed_SET(3.3214529E38F);
        p261.write_speed_SET(2.7176388E38F);
        LoopBackDemoChannel.instance.send(p261); //===============================
        CAMERA_CAPTURE_STATUS p262 = LoopBackDemoChannel.instance.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.time_boot_ms_SET(2271231037L);
        p262.image_status_SET((char)39);
        p262.video_status_SET((char)241);
        p262.image_interval_SET(7.0373283E37F);
        p262.recording_time_ms_SET(2031822655L);
        p262.available_capacity_SET(6.911628E37F);
        LoopBackDemoChannel.instance.send(p262); //===============================
        CAMERA_IMAGE_CAPTURED p263 = LoopBackDemoChannel.instance.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.time_boot_ms_SET(3327796393L);
        p263.time_utc_SET(4981603838721544332L);
        p263.camera_id_SET((char)6);
        p263.lat_SET(-1983488697);
        p263.lon_SET(10847072);
        p263.alt_SET(-1763729482);
        p263.relative_alt_SET(-1473408786);
        p263.q_SET(new float[4], 0);
        p263.image_index_SET(-723486811);
        p263.capture_result_SET((byte)45);
        p263.file_url_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p263); //===============================
        FLIGHT_INFORMATION p264 = LoopBackDemoChannel.instance.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.time_boot_ms_SET(2173856445L);
        p264.arming_time_utc_SET(3524227716579643745L);
        p264.takeoff_time_utc_SET(2550565322032069813L);
        p264.flight_uuid_SET(36248331514451856L);
        LoopBackDemoChannel.instance.send(p264); //===============================
        MOUNT_ORIENTATION p265 = LoopBackDemoChannel.instance.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.time_boot_ms_SET(444539197L);
        p265.roll_SET(-1.559162E38F);
        p265.pitch_SET(9.032713E37F);
        p265.yaw_SET(2.3850177E38F);
        LoopBackDemoChannel.instance.send(p265); //===============================
        LOGGING_DATA p266 = LoopBackDemoChannel.instance.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.target_system_SET((char)185);
        p266.target_component_SET((char)188);
        p266.sequence_SET((char)56384);
        p266.length_SET((char)254);
        p266.first_message_offset_SET((char)83);
        p266.data__SET(new char[249], 0);
        LoopBackDemoChannel.instance.send(p266); //===============================
        LOGGING_DATA_ACKED p267 = LoopBackDemoChannel.instance.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.target_system_SET((char)15);
        p267.target_component_SET((char)65);
        p267.sequence_SET((char)23275);
        p267.length_SET((char)33);
        p267.first_message_offset_SET((char)109);
        p267.data__SET(new char[249], 0);
        LoopBackDemoChannel.instance.send(p267); //===============================
        LOGGING_ACK p268 = LoopBackDemoChannel.instance.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.target_system_SET((char)214);
        p268.target_component_SET((char)72);
        p268.sequence_SET((char)33439);
        LoopBackDemoChannel.instance.send(p268); //===============================
        VIDEO_STREAM_INFORMATION p269 = LoopBackDemoChannel.instance.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.camera_id_SET((char)118);
        p269.status_SET((char)193);
        p269.framerate_SET(-7.597814E37F);
        p269.resolution_h_SET((char)29381);
        p269.resolution_v_SET((char)2971);
        p269.bitrate_SET(1433281150L);
        p269.rotation_SET((char)38074);
        p269.uri_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p269); //===============================
        SET_VIDEO_STREAM_SETTINGS p270 = LoopBackDemoChannel.instance.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.target_system_SET((char)80);
        p270.target_component_SET((char)174);
        p270.camera_id_SET((char)188);
        p270.framerate_SET(-2.422929E38F);
        p270.resolution_h_SET((char)53333);
        p270.resolution_v_SET((char)21745);
        p270.bitrate_SET(2510829857L);
        p270.rotation_SET((char)25608);
        p270.uri_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p270); //===============================
        WIFI_CONFIG_AP p299 = LoopBackDemoChannel.instance.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.ssid_SET("DEMO", PH);
        p299.password_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p299); //===============================
        PROTOCOL_VERSION p300 = LoopBackDemoChannel.instance.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.version_SET((char)63184);
        p300.min_version_SET((char)59804);
        p300.max_version_SET((char)62388);
        p300.spec_version_hash_SET(new char[8], 0);
        p300.library_version_hash_SET(new char[8], 0);
        LoopBackDemoChannel.instance.send(p300); //===============================
        UAVCAN_NODE_STATUS p310 = LoopBackDemoChannel.instance.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.time_usec_SET(8576135625861971050L);
        p310.uptime_sec_SET(599667332L);
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR);
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE);
        p310.sub_mode_SET((char)222);
        p310.vendor_specific_status_code_SET((char)10584);
        LoopBackDemoChannel.instance.send(p310); //===============================
        UAVCAN_NODE_INFO p311 = LoopBackDemoChannel.instance.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.time_usec_SET(2786178908959435539L);
        p311.uptime_sec_SET(142863759L);
        p311.name_SET("DEMO", PH);
        p311.hw_version_major_SET((char)191);
        p311.hw_version_minor_SET((char)201);
        p311.hw_unique_id_SET(new char[16], 0);
        p311.sw_version_major_SET((char)58);
        p311.sw_version_minor_SET((char)106);
        p311.sw_vcs_commit_SET(1805357377L);
        LoopBackDemoChannel.instance.send(p311); //===============================
        PARAM_EXT_REQUEST_READ p320 = LoopBackDemoChannel.instance.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.target_system_SET((char)151);
        p320.target_component_SET((char)65);
        p320.param_id_SET("DEMO", PH);
        p320.param_index_SET((short)8347);
        LoopBackDemoChannel.instance.send(p320); //===============================
        PARAM_EXT_REQUEST_LIST p321 = LoopBackDemoChannel.instance.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_system_SET((char)93);
        p321.target_component_SET((char)90);
        LoopBackDemoChannel.instance.send(p321); //===============================
        PARAM_EXT_VALUE p322 = LoopBackDemoChannel.instance.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_id_SET("DEMO", PH);
        p322.param_value_SET("DEMO", PH);
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT64);
        p322.param_count_SET((char)12093);
        p322.param_index_SET((char)49879);
        LoopBackDemoChannel.instance.send(p322); //===============================
        PARAM_EXT_SET p323 = LoopBackDemoChannel.instance.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.target_system_SET((char)109);
        p323.target_component_SET((char)128);
        p323.param_id_SET("DEMO", PH);
        p323.param_value_SET("DEMO", PH);
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32);
        LoopBackDemoChannel.instance.send(p323); //===============================
        PARAM_EXT_ACK p324 = LoopBackDemoChannel.instance.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_id_SET("DEMO", PH);
        p324.param_value_SET("DEMO", PH);
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8);
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_ACCEPTED);
        LoopBackDemoChannel.instance.send(p324); //===============================
        OBSTACLE_DISTANCE p330 = LoopBackDemoChannel.instance.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.time_usec_SET(2070200206021680785L);
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN);
        p330.distances_SET(new char[72], 0);
        p330.increment_SET((char)14);
        p330.min_distance_SET((char)46385);
        p330.max_distance_SET((char)56680);
        LoopBackDemoChannel.instance.send(p330); //===============================
    }
}

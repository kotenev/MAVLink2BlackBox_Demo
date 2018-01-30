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
        LoopBackDemoChannel.instance.on_CPU_LOAD.add((src, ph, pack) ->
        {
            char  sensLoad = pack.sensLoad_GET();
            char  ctrlLoad = pack.ctrlLoad_GET();
            char  batVolt = pack.batVolt_GET();
        });
        LoopBackDemoChannel.instance.on_SENSOR_BIAS.add((src, ph, pack) ->
        {
            float  axBias = pack.axBias_GET();
            float  ayBias = pack.ayBias_GET();
            float  azBias = pack.azBias_GET();
            float  gxBias = pack.gxBias_GET();
            float  gyBias = pack.gyBias_GET();
            float  gzBias = pack.gzBias_GET();
        });
        LoopBackDemoChannel.instance.on_DIAGNOSTIC.add((src, ph, pack) ->
        {
            float  diagFl1 = pack.diagFl1_GET();
            float  diagFl2 = pack.diagFl2_GET();
            float  diagFl3 = pack.diagFl3_GET();
            short  diagSh1 = pack.diagSh1_GET();
            short  diagSh2 = pack.diagSh2_GET();
            short  diagSh3 = pack.diagSh3_GET();
        });
        LoopBackDemoChannel.instance.on_SLUGS_NAVIGATION.add((src, ph, pack) ->
        {
            float  u_m = pack.u_m_GET();
            float  phi_c = pack.phi_c_GET();
            float  theta_c = pack.theta_c_GET();
            float  psiDot_c = pack.psiDot_c_GET();
            float  ay_body = pack.ay_body_GET();
            float  totalDist = pack.totalDist_GET();
            float  dist2Go = pack.dist2Go_GET();
            char  fromWP = pack.fromWP_GET();
            char  toWP = pack.toWP_GET();
            char  h_c = pack.h_c_GET();
        });
        LoopBackDemoChannel.instance.on_DATA_LOG.add((src, ph, pack) ->
        {
            float  fl_1 = pack.fl_1_GET();
            float  fl_2 = pack.fl_2_GET();
            float  fl_3 = pack.fl_3_GET();
            float  fl_4 = pack.fl_4_GET();
            float  fl_5 = pack.fl_5_GET();
            float  fl_6 = pack.fl_6_GET();
        });
        LoopBackDemoChannel.instance.on_GPS_DATE_TIME.add((src, ph, pack) ->
        {
            char  year = pack.year_GET();
            char  month = pack.month_GET();
            char  day = pack.day_GET();
            char  hour = pack.hour_GET();
            char  min = pack.min_GET();
            char  sec = pack.sec_GET();
            char  clockStat = pack.clockStat_GET();
            char  visSat = pack.visSat_GET();
            char  useSat = pack.useSat_GET();
            char  GppGl = pack.GppGl_GET();
            char  sigUsedMask = pack.sigUsedMask_GET();
            char  percentUsed = pack.percentUsed_GET();
        });
        LoopBackDemoChannel.instance.on_MID_LVL_CMDS.add((src, ph, pack) ->
        {
            char  target = pack.target_GET();
            float  hCommand = pack.hCommand_GET();
            float  uCommand = pack.uCommand_GET();
            float  rCommand = pack.rCommand_GET();
        });
        LoopBackDemoChannel.instance.on_CTRL_SRFC_PT.add((src, ph, pack) ->
        {
            char  target = pack.target_GET();
            char  bitfieldPt = pack.bitfieldPt_GET();
        });
        LoopBackDemoChannel.instance.on_SLUGS_CAMERA_ORDER.add((src, ph, pack) ->
        {
            char  target = pack.target_GET();
            byte  pan = pack.pan_GET();
            byte  tilt = pack.tilt_GET();
            byte  zoom = pack.zoom_GET();
            byte  moveHome = pack.moveHome_GET();
        });
        LoopBackDemoChannel.instance.on_CONTROL_SURFACE.add((src, ph, pack) ->
        {
            char  target = pack.target_GET();
            char  idSurface = pack.idSurface_GET();
            float  mControl = pack.mControl_GET();
            float  bControl = pack.bControl_GET();
        });
        LoopBackDemoChannel.instance.on_SLUGS_MOBILE_LOCATION.add((src, ph, pack) ->
        {
            char  target = pack.target_GET();
            float  latitude = pack.latitude_GET();
            float  longitude = pack.longitude_GET();
        });
        LoopBackDemoChannel.instance.on_SLUGS_CONFIGURATION_CAMERA.add((src, ph, pack) ->
        {
            char  target = pack.target_GET();
            char  idOrder = pack.idOrder_GET();
            char  order = pack.order_GET();
        });
        LoopBackDemoChannel.instance.on_ISR_LOCATION.add((src, ph, pack) ->
        {
            char  target = pack.target_GET();
            float  latitude = pack.latitude_GET();
            float  longitude = pack.longitude_GET();
            float  height = pack.height_GET();
            char  option1 = pack.option1_GET();
            char  option2 = pack.option2_GET();
            char  option3 = pack.option3_GET();
        });
        LoopBackDemoChannel.instance.on_VOLT_SENSOR.add((src, ph, pack) ->
        {
            char  r2Type = pack.r2Type_GET();
            char  voltage = pack.voltage_GET();
            char  reading2 = pack.reading2_GET();
        });
        LoopBackDemoChannel.instance.on_PTZ_STATUS.add((src, ph, pack) ->
        {
            char  zoom = pack.zoom_GET();
            short  pan = pack.pan_GET();
            short  tilt = pack.tilt_GET();
        });
        LoopBackDemoChannel.instance.on_UAV_STATUS.add((src, ph, pack) ->
        {
            char  target = pack.target_GET();
            float  latitude = pack.latitude_GET();
            float  longitude = pack.longitude_GET();
            float  altitude = pack.altitude_GET();
            float  speed = pack.speed_GET();
            float  course = pack.course_GET();
        });
        LoopBackDemoChannel.instance.on_STATUS_GPS.add((src, ph, pack) ->
        {
            char  csFails = pack.csFails_GET();
            char  gpsQuality = pack.gpsQuality_GET();
            char  msgsType = pack.msgsType_GET();
            char  posStatus = pack.posStatus_GET();
            float  magVar = pack.magVar_GET();
            byte  magDir = pack.magDir_GET();
            char  modeInd = pack.modeInd_GET();
        });
        LoopBackDemoChannel.instance.on_NOVATEL_DIAG.add((src, ph, pack) ->
        {
            char  timeStatus = pack.timeStatus_GET();
            long  receiverStatus = pack.receiverStatus_GET();
            char  solStatus = pack.solStatus_GET();
            char  posType = pack.posType_GET();
            char  velType = pack.velType_GET();
            float  posSolAge = pack.posSolAge_GET();
            char  csFails = pack.csFails_GET();
        });
        LoopBackDemoChannel.instance.on_SENSOR_DIAG.add((src, ph, pack) ->
        {
            float  float1 = pack.float1_GET();
            float  float2 = pack.float2_GET();
            short  int1 = pack.int1_GET();
            byte  char1 = pack.char1_GET();
        });
        LoopBackDemoChannel.instance.on_BOOT.add((src, ph, pack) ->
        {
            long  version = pack.version_GET();
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
        p0.type_SET(MAV_TYPE.MAV_TYPE_VTOL_TILTROTOR);
        p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_GENERIC_MISSION_FULL);
        p0.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED);
        p0.custom_mode_SET(1932424355L);
        p0.system_status_SET(MAV_STATE.MAV_STATE_FLIGHT_TERMINATION);
        p0.mavlink_version_SET((char)218);
        LoopBackDemoChannel.instance.send(p0); //===============================
        SYS_STATUS p1 = LoopBackDemoChannel.instance.new_SYS_STATUS();
        PH.setPack(p1);
        p1.onboard_control_sensors_present_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING);
        p1.onboard_control_sensors_enabled_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG);
        p1.onboard_control_sensors_health_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS);
        p1.load_SET((char)15985);
        p1.voltage_battery_SET((char)4893);
        p1.current_battery_SET((short)14880);
        p1.battery_remaining_SET((byte) - 1);
        p1.drop_rate_comm_SET((char)15096);
        p1.errors_comm_SET((char)65307);
        p1.errors_count1_SET((char)29804);
        p1.errors_count2_SET((char)62826);
        p1.errors_count3_SET((char)21945);
        p1.errors_count4_SET((char)6514);
        LoopBackDemoChannel.instance.send(p1); //===============================
        SYSTEM_TIME p2 = LoopBackDemoChannel.instance.new_SYSTEM_TIME();
        PH.setPack(p2);
        p2.time_unix_usec_SET(3648422741979458816L);
        p2.time_boot_ms_SET(3466579959L);
        LoopBackDemoChannel.instance.send(p2); //===============================
        POSITION_TARGET_LOCAL_NED p3 = LoopBackDemoChannel.instance.new_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.time_boot_ms_SET(4128926638L);
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_INT);
        p3.type_mask_SET((char)57369);
        p3.x_SET(-6.654988E37F);
        p3.y_SET(1.9479756E37F);
        p3.z_SET(1.3638602E38F);
        p3.vx_SET(2.6285287E38F);
        p3.vy_SET(1.6430673E38F);
        p3.vz_SET(2.1430497E38F);
        p3.afx_SET(-2.0220579E38F);
        p3.afy_SET(-1.8838063E38F);
        p3.afz_SET(2.0039694E38F);
        p3.yaw_SET(4.8503324E37F);
        p3.yaw_rate_SET(-3.0962305E38F);
        LoopBackDemoChannel.instance.send(p3); //===============================
        PING p4 = LoopBackDemoChannel.instance.new_PING();
        PH.setPack(p4);
        p4.time_usec_SET(2194910826571765694L);
        p4.seq_SET(640339082L);
        p4.target_system_SET((char)124);
        p4.target_component_SET((char)49);
        LoopBackDemoChannel.instance.send(p4); //===============================
        CHANGE_OPERATOR_CONTROL p5 = LoopBackDemoChannel.instance.new_CHANGE_OPERATOR_CONTROL();
        PH.setPack(p5);
        p5.target_system_SET((char)82);
        p5.control_request_SET((char)247);
        p5.version_SET((char)56);
        p5.passkey_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p5); //===============================
        CHANGE_OPERATOR_CONTROL_ACK p6 = LoopBackDemoChannel.instance.new_CHANGE_OPERATOR_CONTROL_ACK();
        PH.setPack(p6);
        p6.gcs_system_id_SET((char)227);
        p6.control_request_SET((char)51);
        p6.ack_SET((char)23);
        LoopBackDemoChannel.instance.send(p6); //===============================
        AUTH_KEY p7 = LoopBackDemoChannel.instance.new_AUTH_KEY();
        PH.setPack(p7);
        p7.key_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p7); //===============================
        SET_MODE p11 = LoopBackDemoChannel.instance.new_SET_MODE();
        PH.setPack(p11);
        p11.target_system_SET((char)7);
        p11.base_mode_SET(MAV_MODE.MAV_MODE_STABILIZE_ARMED);
        p11.custom_mode_SET(3224783772L);
        LoopBackDemoChannel.instance.send(p11); //===============================
        PARAM_REQUEST_READ p20 = LoopBackDemoChannel.instance.new_PARAM_REQUEST_READ();
        PH.setPack(p20);
        p20.target_system_SET((char)0);
        p20.target_component_SET((char)105);
        p20.param_id_SET("DEMO", PH);
        p20.param_index_SET((short) -16927);
        LoopBackDemoChannel.instance.send(p20); //===============================
        PARAM_REQUEST_LIST p21 = LoopBackDemoChannel.instance.new_PARAM_REQUEST_LIST();
        PH.setPack(p21);
        p21.target_system_SET((char)118);
        p21.target_component_SET((char)3);
        LoopBackDemoChannel.instance.send(p21); //===============================
        PARAM_VALUE p22 = LoopBackDemoChannel.instance.new_PARAM_VALUE();
        PH.setPack(p22);
        p22.param_id_SET("DEMO", PH);
        p22.param_value_SET(-1.7386974E38F);
        p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT64);
        p22.param_count_SET((char)31166);
        p22.param_index_SET((char)29636);
        LoopBackDemoChannel.instance.send(p22); //===============================
        PARAM_SET p23 = LoopBackDemoChannel.instance.new_PARAM_SET();
        PH.setPack(p23);
        p23.target_system_SET((char)141);
        p23.target_component_SET((char)162);
        p23.param_id_SET("DEMO", PH);
        p23.param_value_SET(3.3818018E38F);
        p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT8);
        LoopBackDemoChannel.instance.send(p23); //===============================
        GPS_RAW_INT p24 = LoopBackDemoChannel.instance.new_GPS_RAW_INT();
        PH.setPack(p24);
        p24.time_usec_SET(659184039635755371L);
        p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS);
        p24.lat_SET(1805293240);
        p24.lon_SET(254446925);
        p24.alt_SET(1438445033);
        p24.eph_SET((char)63054);
        p24.epv_SET((char)8598);
        p24.vel_SET((char)17221);
        p24.cog_SET((char)32556);
        p24.satellites_visible_SET((char)86);
        p24.alt_ellipsoid_SET(1206743628, PH);
        p24.h_acc_SET(560186517L, PH);
        p24.v_acc_SET(3217577220L, PH);
        p24.vel_acc_SET(860148451L, PH);
        p24.hdg_acc_SET(2801407292L, PH);
        LoopBackDemoChannel.instance.send(p24); //===============================
        GPS_STATUS p25 = LoopBackDemoChannel.instance.new_GPS_STATUS();
        PH.setPack(p25);
        p25.satellites_visible_SET((char)7);
        p25.satellite_prn_SET(new char[20], 0);
        p25.satellite_used_SET(new char[20], 0);
        p25.satellite_elevation_SET(new char[20], 0);
        p25.satellite_azimuth_SET(new char[20], 0);
        p25.satellite_snr_SET(new char[20], 0);
        LoopBackDemoChannel.instance.send(p25); //===============================
        SCALED_IMU p26 = LoopBackDemoChannel.instance.new_SCALED_IMU();
        PH.setPack(p26);
        p26.time_boot_ms_SET(3379245782L);
        p26.xacc_SET((short)2543);
        p26.yacc_SET((short) -12084);
        p26.zacc_SET((short)26948);
        p26.xgyro_SET((short)10687);
        p26.ygyro_SET((short)27351);
        p26.zgyro_SET((short)11212);
        p26.xmag_SET((short)22725);
        p26.ymag_SET((short)17024);
        p26.zmag_SET((short)6120);
        LoopBackDemoChannel.instance.send(p26); //===============================
        RAW_IMU p27 = LoopBackDemoChannel.instance.new_RAW_IMU();
        PH.setPack(p27);
        p27.time_usec_SET(794555893517572057L);
        p27.xacc_SET((short)22790);
        p27.yacc_SET((short) -9516);
        p27.zacc_SET((short)12006);
        p27.xgyro_SET((short)5904);
        p27.ygyro_SET((short)1858);
        p27.zgyro_SET((short) -28228);
        p27.xmag_SET((short)9958);
        p27.ymag_SET((short) -18350);
        p27.zmag_SET((short) -16434);
        LoopBackDemoChannel.instance.send(p27); //===============================
        RAW_PRESSURE p28 = LoopBackDemoChannel.instance.new_RAW_PRESSURE();
        PH.setPack(p28);
        p28.time_usec_SET(4621479417789591838L);
        p28.press_abs_SET((short) -1885);
        p28.press_diff1_SET((short)24382);
        p28.press_diff2_SET((short) -21448);
        p28.temperature_SET((short)14277);
        LoopBackDemoChannel.instance.send(p28); //===============================
        SCALED_PRESSURE p29 = LoopBackDemoChannel.instance.new_SCALED_PRESSURE();
        PH.setPack(p29);
        p29.time_boot_ms_SET(3250912801L);
        p29.press_abs_SET(-2.845039E38F);
        p29.press_diff_SET(-1.963239E38F);
        p29.temperature_SET((short) -15749);
        LoopBackDemoChannel.instance.send(p29); //===============================
        ATTITUDE p30 = LoopBackDemoChannel.instance.new_ATTITUDE();
        PH.setPack(p30);
        p30.time_boot_ms_SET(1986805607L);
        p30.roll_SET(-1.177974E38F);
        p30.pitch_SET(-2.5910425E38F);
        p30.yaw_SET(1.1242348E37F);
        p30.rollspeed_SET(-7.6607E37F);
        p30.pitchspeed_SET(1.6584646E38F);
        p30.yawspeed_SET(2.7324734E38F);
        LoopBackDemoChannel.instance.send(p30); //===============================
        ATTITUDE_QUATERNION p31 = LoopBackDemoChannel.instance.new_ATTITUDE_QUATERNION();
        PH.setPack(p31);
        p31.time_boot_ms_SET(1808715877L);
        p31.q1_SET(1.4817913E38F);
        p31.q2_SET(2.2955536E38F);
        p31.q3_SET(-1.6553831E38F);
        p31.q4_SET(-5.0727833E37F);
        p31.rollspeed_SET(2.0450167E38F);
        p31.pitchspeed_SET(7.846838E37F);
        p31.yawspeed_SET(2.8234394E37F);
        LoopBackDemoChannel.instance.send(p31); //===============================
        LOCAL_POSITION_NED p32 = LoopBackDemoChannel.instance.new_LOCAL_POSITION_NED();
        PH.setPack(p32);
        p32.time_boot_ms_SET(2344702740L);
        p32.x_SET(1.0627628E38F);
        p32.y_SET(-5.8693065E37F);
        p32.z_SET(-3.2713935E38F);
        p32.vx_SET(2.8626005E38F);
        p32.vy_SET(-1.964156E38F);
        p32.vz_SET(2.1563343E38F);
        LoopBackDemoChannel.instance.send(p32); //===============================
        GLOBAL_POSITION_INT p33 = LoopBackDemoChannel.instance.new_GLOBAL_POSITION_INT();
        PH.setPack(p33);
        p33.time_boot_ms_SET(118744349L);
        p33.lat_SET(1586291166);
        p33.lon_SET(72419233);
        p33.alt_SET(345371993);
        p33.relative_alt_SET(258055889);
        p33.vx_SET((short)17821);
        p33.vy_SET((short) -27306);
        p33.vz_SET((short)24216);
        p33.hdg_SET((char)17190);
        LoopBackDemoChannel.instance.send(p33); //===============================
        RC_CHANNELS_SCALED p34 = LoopBackDemoChannel.instance.new_RC_CHANNELS_SCALED();
        PH.setPack(p34);
        p34.time_boot_ms_SET(3144045530L);
        p34.port_SET((char)48);
        p34.chan1_scaled_SET((short)4487);
        p34.chan2_scaled_SET((short) -18678);
        p34.chan3_scaled_SET((short)23875);
        p34.chan4_scaled_SET((short) -23448);
        p34.chan5_scaled_SET((short)5677);
        p34.chan6_scaled_SET((short)6967);
        p34.chan7_scaled_SET((short)10977);
        p34.chan8_scaled_SET((short)4572);
        p34.rssi_SET((char)45);
        LoopBackDemoChannel.instance.send(p34); //===============================
        RC_CHANNELS_RAW p35 = LoopBackDemoChannel.instance.new_RC_CHANNELS_RAW();
        PH.setPack(p35);
        p35.time_boot_ms_SET(2086755102L);
        p35.port_SET((char)30);
        p35.chan1_raw_SET((char)6474);
        p35.chan2_raw_SET((char)34437);
        p35.chan3_raw_SET((char)2679);
        p35.chan4_raw_SET((char)49026);
        p35.chan5_raw_SET((char)21580);
        p35.chan6_raw_SET((char)35441);
        p35.chan7_raw_SET((char)55011);
        p35.chan8_raw_SET((char)62954);
        p35.rssi_SET((char)45);
        LoopBackDemoChannel.instance.send(p35); //===============================
        SERVO_OUTPUT_RAW p36 = LoopBackDemoChannel.instance.new_SERVO_OUTPUT_RAW();
        PH.setPack(p36);
        p36.time_usec_SET(820600186L);
        p36.port_SET((char)116);
        p36.servo1_raw_SET((char)53583);
        p36.servo2_raw_SET((char)11054);
        p36.servo3_raw_SET((char)19564);
        p36.servo4_raw_SET((char)18767);
        p36.servo5_raw_SET((char)6978);
        p36.servo6_raw_SET((char)40452);
        p36.servo7_raw_SET((char)16847);
        p36.servo8_raw_SET((char)64936);
        p36.servo9_raw_SET((char)11205, PH);
        p36.servo10_raw_SET((char)20353, PH);
        p36.servo11_raw_SET((char)30885, PH);
        p36.servo12_raw_SET((char)26436, PH);
        p36.servo13_raw_SET((char)61575, PH);
        p36.servo14_raw_SET((char)40865, PH);
        p36.servo15_raw_SET((char)51834, PH);
        p36.servo16_raw_SET((char)45077, PH);
        LoopBackDemoChannel.instance.send(p36); //===============================
        MISSION_REQUEST_PARTIAL_LIST p37 = LoopBackDemoChannel.instance.new_MISSION_REQUEST_PARTIAL_LIST();
        PH.setPack(p37);
        p37.target_system_SET((char)215);
        p37.target_component_SET((char)174);
        p37.start_index_SET((short) -15948);
        p37.end_index_SET((short)24427);
        p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
        LoopBackDemoChannel.instance.send(p37); //===============================
        MISSION_WRITE_PARTIAL_LIST p38 = LoopBackDemoChannel.instance.new_MISSION_WRITE_PARTIAL_LIST();
        PH.setPack(p38);
        p38.target_system_SET((char)210);
        p38.target_component_SET((char)118);
        p38.start_index_SET((short) -20415);
        p38.end_index_SET((short)12313);
        p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
        LoopBackDemoChannel.instance.send(p38); //===============================
        MISSION_ITEM p39 = LoopBackDemoChannel.instance.new_MISSION_ITEM();
        PH.setPack(p39);
        p39.target_system_SET((char)245);
        p39.target_component_SET((char)64);
        p39.seq_SET((char)8192);
        p39.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_INT);
        p39.command_SET(MAV_CMD.MAV_CMD_NAV_TAKEOFF);
        p39.current_SET((char)90);
        p39.autocontinue_SET((char)208);
        p39.param1_SET(1.8422724E38F);
        p39.param2_SET(-1.5372084E38F);
        p39.param3_SET(1.3330669E38F);
        p39.param4_SET(-3.0639606E38F);
        p39.x_SET(2.1089597E38F);
        p39.y_SET(-2.6528775E38F);
        p39.z_SET(2.315291E38F);
        p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
        LoopBackDemoChannel.instance.send(p39); //===============================
        MISSION_REQUEST p40 = LoopBackDemoChannel.instance.new_MISSION_REQUEST();
        PH.setPack(p40);
        p40.target_system_SET((char)9);
        p40.target_component_SET((char)224);
        p40.seq_SET((char)24787);
        p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
        LoopBackDemoChannel.instance.send(p40); //===============================
        MISSION_SET_CURRENT p41 = LoopBackDemoChannel.instance.new_MISSION_SET_CURRENT();
        PH.setPack(p41);
        p41.target_system_SET((char)92);
        p41.target_component_SET((char)133);
        p41.seq_SET((char)31898);
        LoopBackDemoChannel.instance.send(p41); //===============================
        MISSION_CURRENT p42 = LoopBackDemoChannel.instance.new_MISSION_CURRENT();
        PH.setPack(p42);
        p42.seq_SET((char)12402);
        LoopBackDemoChannel.instance.send(p42); //===============================
        MISSION_REQUEST_LIST p43 = LoopBackDemoChannel.instance.new_MISSION_REQUEST_LIST();
        PH.setPack(p43);
        p43.target_system_SET((char)49);
        p43.target_component_SET((char)137);
        p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
        LoopBackDemoChannel.instance.send(p43); //===============================
        MISSION_COUNT p44 = LoopBackDemoChannel.instance.new_MISSION_COUNT();
        PH.setPack(p44);
        p44.target_system_SET((char)10);
        p44.target_component_SET((char)235);
        p44.count_SET((char)39881);
        p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
        LoopBackDemoChannel.instance.send(p44); //===============================
        MISSION_CLEAR_ALL p45 = LoopBackDemoChannel.instance.new_MISSION_CLEAR_ALL();
        PH.setPack(p45);
        p45.target_system_SET((char)135);
        p45.target_component_SET((char)85);
        p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
        LoopBackDemoChannel.instance.send(p45); //===============================
        MISSION_ITEM_REACHED p46 = LoopBackDemoChannel.instance.new_MISSION_ITEM_REACHED();
        PH.setPack(p46);
        p46.seq_SET((char)8245);
        LoopBackDemoChannel.instance.send(p46); //===============================
        MISSION_ACK p47 = LoopBackDemoChannel.instance.new_MISSION_ACK();
        PH.setPack(p47);
        p47.target_system_SET((char)248);
        p47.target_component_SET((char)169);
        p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_NO_SPACE);
        p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
        LoopBackDemoChannel.instance.send(p47); //===============================
        SET_GPS_GLOBAL_ORIGIN p48 = LoopBackDemoChannel.instance.new_SET_GPS_GLOBAL_ORIGIN();
        PH.setPack(p48);
        p48.target_system_SET((char)93);
        p48.latitude_SET(-1184557476);
        p48.longitude_SET(-450722567);
        p48.altitude_SET(1725470516);
        p48.time_usec_SET(8289892445754938389L, PH);
        LoopBackDemoChannel.instance.send(p48); //===============================
        GPS_GLOBAL_ORIGIN p49 = LoopBackDemoChannel.instance.new_GPS_GLOBAL_ORIGIN();
        PH.setPack(p49);
        p49.latitude_SET(2096725771);
        p49.longitude_SET(-1320275169);
        p49.altitude_SET(-1469711888);
        p49.time_usec_SET(7236100365070306810L, PH);
        LoopBackDemoChannel.instance.send(p49); //===============================
        PARAM_MAP_RC p50 = LoopBackDemoChannel.instance.new_PARAM_MAP_RC();
        PH.setPack(p50);
        p50.target_system_SET((char)38);
        p50.target_component_SET((char)123);
        p50.param_id_SET("DEMO", PH);
        p50.param_index_SET((short)25068);
        p50.parameter_rc_channel_index_SET((char)71);
        p50.param_value0_SET(3.0838848E38F);
        p50.scale_SET(-1.6083388E38F);
        p50.param_value_min_SET(-1.6200073E38F);
        p50.param_value_max_SET(-1.2204414E38F);
        LoopBackDemoChannel.instance.send(p50); //===============================
        MISSION_REQUEST_INT p51 = LoopBackDemoChannel.instance.new_MISSION_REQUEST_INT();
        PH.setPack(p51);
        p51.target_system_SET((char)207);
        p51.target_component_SET((char)131);
        p51.seq_SET((char)61358);
        p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
        LoopBackDemoChannel.instance.send(p51); //===============================
        SAFETY_SET_ALLOWED_AREA p54 = LoopBackDemoChannel.instance.new_SAFETY_SET_ALLOWED_AREA();
        PH.setPack(p54);
        p54.target_system_SET((char)68);
        p54.target_component_SET((char)98);
        p54.frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED);
        p54.p1x_SET(-2.6518756E38F);
        p54.p1y_SET(-1.759237E38F);
        p54.p1z_SET(1.8061549E38F);
        p54.p2x_SET(-3.05305E38F);
        p54.p2y_SET(2.9028951E38F);
        p54.p2z_SET(-1.2909841E38F);
        LoopBackDemoChannel.instance.send(p54); //===============================
        SAFETY_ALLOWED_AREA p55 = LoopBackDemoChannel.instance.new_SAFETY_ALLOWED_AREA();
        PH.setPack(p55);
        p55.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL);
        p55.p1x_SET(1.5086761E38F);
        p55.p1y_SET(1.7750735E38F);
        p55.p1z_SET(1.5272679E37F);
        p55.p2x_SET(-5.0198533E37F);
        p55.p2y_SET(1.2321407E38F);
        p55.p2z_SET(-3.3815574E38F);
        LoopBackDemoChannel.instance.send(p55); //===============================
        ATTITUDE_QUATERNION_COV p61 = LoopBackDemoChannel.instance.new_ATTITUDE_QUATERNION_COV();
        PH.setPack(p61);
        p61.time_usec_SET(8530334411648303057L);
        p61.q_SET(new float[4], 0);
        p61.rollspeed_SET(-8.954869E37F);
        p61.pitchspeed_SET(2.1692962E38F);
        p61.yawspeed_SET(-2.6349844E38F);
        p61.covariance_SET(new float[9], 0);
        LoopBackDemoChannel.instance.send(p61); //===============================
        NAV_CONTROLLER_OUTPUT p62 = LoopBackDemoChannel.instance.new_NAV_CONTROLLER_OUTPUT();
        PH.setPack(p62);
        p62.nav_roll_SET(-9.819729E37F);
        p62.nav_pitch_SET(-1.9602294E37F);
        p62.nav_bearing_SET((short) -8622);
        p62.target_bearing_SET((short) -13163);
        p62.wp_dist_SET((char)57025);
        p62.alt_error_SET(-1.7675528E38F);
        p62.aspd_error_SET(2.343717E38F);
        p62.xtrack_error_SET(2.2903323E38F);
        LoopBackDemoChannel.instance.send(p62); //===============================
        GLOBAL_POSITION_INT_COV p63 = LoopBackDemoChannel.instance.new_GLOBAL_POSITION_INT_COV();
        PH.setPack(p63);
        p63.time_usec_SET(5457431927552200407L);
        p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO);
        p63.lat_SET(548237745);
        p63.lon_SET(-1900879755);
        p63.alt_SET(1432385806);
        p63.relative_alt_SET(-1392917710);
        p63.vx_SET(-5.41548E37F);
        p63.vy_SET(1.8700589E38F);
        p63.vz_SET(1.0573602E38F);
        p63.covariance_SET(new float[36], 0);
        LoopBackDemoChannel.instance.send(p63); //===============================
        LOCAL_POSITION_NED_COV p64 = LoopBackDemoChannel.instance.new_LOCAL_POSITION_NED_COV();
        PH.setPack(p64);
        p64.time_usec_SET(7540916961477302174L);
        p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS);
        p64.x_SET(6.9923653E37F);
        p64.y_SET(-2.1903477E38F);
        p64.z_SET(2.9262282E38F);
        p64.vx_SET(-1.6867957E38F);
        p64.vy_SET(2.807988E38F);
        p64.vz_SET(9.961978E37F);
        p64.ax_SET(1.9418103E37F);
        p64.ay_SET(3.1850224E38F);
        p64.az_SET(-3.3313762E38F);
        p64.covariance_SET(new float[45], 0);
        LoopBackDemoChannel.instance.send(p64); //===============================
        RC_CHANNELS p65 = LoopBackDemoChannel.instance.new_RC_CHANNELS();
        PH.setPack(p65);
        p65.time_boot_ms_SET(58244647L);
        p65.chancount_SET((char)17);
        p65.chan1_raw_SET((char)9003);
        p65.chan2_raw_SET((char)26035);
        p65.chan3_raw_SET((char)47205);
        p65.chan4_raw_SET((char)54599);
        p65.chan5_raw_SET((char)34824);
        p65.chan6_raw_SET((char)21559);
        p65.chan7_raw_SET((char)32143);
        p65.chan8_raw_SET((char)57252);
        p65.chan9_raw_SET((char)24585);
        p65.chan10_raw_SET((char)40163);
        p65.chan11_raw_SET((char)32001);
        p65.chan12_raw_SET((char)13256);
        p65.chan13_raw_SET((char)47964);
        p65.chan14_raw_SET((char)1595);
        p65.chan15_raw_SET((char)55838);
        p65.chan16_raw_SET((char)25259);
        p65.chan17_raw_SET((char)14804);
        p65.chan18_raw_SET((char)43740);
        p65.rssi_SET((char)200);
        LoopBackDemoChannel.instance.send(p65); //===============================
        REQUEST_DATA_STREAM p66 = LoopBackDemoChannel.instance.new_REQUEST_DATA_STREAM();
        PH.setPack(p66);
        p66.target_system_SET((char)234);
        p66.target_component_SET((char)202);
        p66.req_stream_id_SET((char)177);
        p66.req_message_rate_SET((char)20408);
        p66.start_stop_SET((char)34);
        LoopBackDemoChannel.instance.send(p66); //===============================
        DATA_STREAM p67 = LoopBackDemoChannel.instance.new_DATA_STREAM();
        PH.setPack(p67);
        p67.stream_id_SET((char)163);
        p67.message_rate_SET((char)41593);
        p67.on_off_SET((char)189);
        LoopBackDemoChannel.instance.send(p67); //===============================
        MANUAL_CONTROL p69 = LoopBackDemoChannel.instance.new_MANUAL_CONTROL();
        PH.setPack(p69);
        p69.target_SET((char)128);
        p69.x_SET((short) -25235);
        p69.y_SET((short) -8787);
        p69.z_SET((short)24806);
        p69.r_SET((short)13007);
        p69.buttons_SET((char)58821);
        LoopBackDemoChannel.instance.send(p69); //===============================
        RC_CHANNELS_OVERRIDE p70 = LoopBackDemoChannel.instance.new_RC_CHANNELS_OVERRIDE();
        PH.setPack(p70);
        p70.target_system_SET((char)241);
        p70.target_component_SET((char)201);
        p70.chan1_raw_SET((char)21380);
        p70.chan2_raw_SET((char)44391);
        p70.chan3_raw_SET((char)57779);
        p70.chan4_raw_SET((char)41449);
        p70.chan5_raw_SET((char)61816);
        p70.chan6_raw_SET((char)42424);
        p70.chan7_raw_SET((char)23032);
        p70.chan8_raw_SET((char)42996);
        LoopBackDemoChannel.instance.send(p70); //===============================
        MISSION_ITEM_INT p73 = LoopBackDemoChannel.instance.new_MISSION_ITEM_INT();
        PH.setPack(p73);
        p73.target_system_SET((char)122);
        p73.target_component_SET((char)252);
        p73.seq_SET((char)39618);
        p73.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
        p73.command_SET(MAV_CMD.MAV_CMD_TURN_LIGHT);
        p73.current_SET((char)208);
        p73.autocontinue_SET((char)210);
        p73.param1_SET(2.1814875E37F);
        p73.param2_SET(-1.68059E38F);
        p73.param3_SET(-1.8326807E36F);
        p73.param4_SET(-1.2554496E38F);
        p73.x_SET(901665231);
        p73.y_SET(-1524525972);
        p73.z_SET(3.2629134E37F);
        p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
        LoopBackDemoChannel.instance.send(p73); //===============================
        VFR_HUD p74 = LoopBackDemoChannel.instance.new_VFR_HUD();
        PH.setPack(p74);
        p74.airspeed_SET(2.0983276E38F);
        p74.groundspeed_SET(2.7814805E37F);
        p74.heading_SET((short)16564);
        p74.throttle_SET((char)8804);
        p74.alt_SET(-7.2723736E37F);
        p74.climb_SET(-3.1039445E38F);
        LoopBackDemoChannel.instance.send(p74); //===============================
        COMMAND_INT p75 = LoopBackDemoChannel.instance.new_COMMAND_INT();
        PH.setPack(p75);
        p75.target_system_SET((char)96);
        p75.target_component_SET((char)76);
        p75.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
        p75.command_SET(MAV_CMD.MAV_CMD_DO_SET_PARAMETER);
        p75.current_SET((char)87);
        p75.autocontinue_SET((char)189);
        p75.param1_SET(-3.3422598E38F);
        p75.param2_SET(6.2576375E37F);
        p75.param3_SET(-5.8341205E37F);
        p75.param4_SET(3.2125008E38F);
        p75.x_SET(-880266748);
        p75.y_SET(1778282726);
        p75.z_SET(1.4800948E38F);
        LoopBackDemoChannel.instance.send(p75); //===============================
        COMMAND_LONG p76 = LoopBackDemoChannel.instance.new_COMMAND_LONG();
        PH.setPack(p76);
        p76.target_system_SET((char)39);
        p76.target_component_SET((char)254);
        p76.command_SET(MAV_CMD.MAV_CMD_OVERRIDE_GOTO);
        p76.confirmation_SET((char)166);
        p76.param1_SET(-1.034286E38F);
        p76.param2_SET(-9.5687167E36F);
        p76.param3_SET(1.7013053E38F);
        p76.param4_SET(-3.3878153E38F);
        p76.param5_SET(2.3888705E38F);
        p76.param6_SET(2.4175462E38F);
        p76.param7_SET(-2.0873025E38F);
        LoopBackDemoChannel.instance.send(p76); //===============================
        COMMAND_ACK p77 = LoopBackDemoChannel.instance.new_COMMAND_ACK();
        PH.setPack(p77);
        p77.command_SET(MAV_CMD.MAV_CMD_DO_NOTHING);
        p77.result_SET(MAV_RESULT.MAV_RESULT_DENIED);
        p77.progress_SET((char)232, PH);
        p77.result_param2_SET(-248054876, PH);
        p77.target_system_SET((char)118, PH);
        p77.target_component_SET((char)43, PH);
        LoopBackDemoChannel.instance.send(p77); //===============================
        MANUAL_SETPOINT p81 = LoopBackDemoChannel.instance.new_MANUAL_SETPOINT();
        PH.setPack(p81);
        p81.time_boot_ms_SET(602445283L);
        p81.roll_SET(2.9333269E38F);
        p81.pitch_SET(1.4326806E38F);
        p81.yaw_SET(8.251739E37F);
        p81.thrust_SET(-8.463773E36F);
        p81.mode_switch_SET((char)230);
        p81.manual_override_switch_SET((char)130);
        LoopBackDemoChannel.instance.send(p81); //===============================
        SET_ATTITUDE_TARGET p82 = LoopBackDemoChannel.instance.new_SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.time_boot_ms_SET(3047483306L);
        p82.target_system_SET((char)90);
        p82.target_component_SET((char)164);
        p82.type_mask_SET((char)221);
        p82.q_SET(new float[4], 0);
        p82.body_roll_rate_SET(1.3866642E38F);
        p82.body_pitch_rate_SET(-3.2880832E38F);
        p82.body_yaw_rate_SET(-1.704638E38F);
        p82.thrust_SET(2.1036086E38F);
        LoopBackDemoChannel.instance.send(p82); //===============================
        ATTITUDE_TARGET p83 = LoopBackDemoChannel.instance.new_ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.time_boot_ms_SET(571102008L);
        p83.type_mask_SET((char)215);
        p83.q_SET(new float[4], 0);
        p83.body_roll_rate_SET(3.3804621E38F);
        p83.body_pitch_rate_SET(-1.0154519E38F);
        p83.body_yaw_rate_SET(-4.882293E37F);
        p83.thrust_SET(-1.5248957E38F);
        LoopBackDemoChannel.instance.send(p83); //===============================
        SET_POSITION_TARGET_LOCAL_NED p84 = LoopBackDemoChannel.instance.new_SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.time_boot_ms_SET(3764119956L);
        p84.target_system_SET((char)48);
        p84.target_component_SET((char)210);
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_INT);
        p84.type_mask_SET((char)13002);
        p84.x_SET(1.1225118E38F);
        p84.y_SET(-2.9365729E38F);
        p84.z_SET(2.53209E38F);
        p84.vx_SET(-2.2895699E38F);
        p84.vy_SET(3.1547551E37F);
        p84.vz_SET(-2.0264377E38F);
        p84.afx_SET(-2.2694483E38F);
        p84.afy_SET(-1.9959086E38F);
        p84.afz_SET(2.0674517E38F);
        p84.yaw_SET(-2.6516316E38F);
        p84.yaw_rate_SET(-7.7802715E37F);
        LoopBackDemoChannel.instance.send(p84); //===============================
        SET_POSITION_TARGET_GLOBAL_INT p86 = LoopBackDemoChannel.instance.new_SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.time_boot_ms_SET(685886062L);
        p86.target_system_SET((char)173);
        p86.target_component_SET((char)25);
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL);
        p86.type_mask_SET((char)56768);
        p86.lat_int_SET(-1522204252);
        p86.lon_int_SET(613746918);
        p86.alt_SET(-1.901008E38F);
        p86.vx_SET(3.2231956E38F);
        p86.vy_SET(2.6207907E38F);
        p86.vz_SET(2.1981754E37F);
        p86.afx_SET(7.6471834E37F);
        p86.afy_SET(2.3833069E38F);
        p86.afz_SET(-6.6136065E37F);
        p86.yaw_SET(-2.3367202E37F);
        p86.yaw_rate_SET(-6.368005E37F);
        LoopBackDemoChannel.instance.send(p86); //===============================
        POSITION_TARGET_GLOBAL_INT p87 = LoopBackDemoChannel.instance.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.time_boot_ms_SET(1543447534L);
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
        p87.type_mask_SET((char)12710);
        p87.lat_int_SET(-90106787);
        p87.lon_int_SET(1477762934);
        p87.alt_SET(8.612918E37F);
        p87.vx_SET(-3.825895E37F);
        p87.vy_SET(-1.2761965E38F);
        p87.vz_SET(-5.900475E37F);
        p87.afx_SET(-2.3742134E38F);
        p87.afy_SET(-7.5974727E37F);
        p87.afz_SET(-1.4415289E37F);
        p87.yaw_SET(-1.0891839E38F);
        p87.yaw_rate_SET(7.121191E37F);
        LoopBackDemoChannel.instance.send(p87); //===============================
        LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = LoopBackDemoChannel.instance.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.time_boot_ms_SET(3864897187L);
        p89.x_SET(-1.5418141E38F);
        p89.y_SET(-2.1054052E38F);
        p89.z_SET(1.4624106E38F);
        p89.roll_SET(1.6812253E37F);
        p89.pitch_SET(-2.1142942E38F);
        p89.yaw_SET(-2.2818703E38F);
        LoopBackDemoChannel.instance.send(p89); //===============================
        HIL_STATE p90 = LoopBackDemoChannel.instance.new_HIL_STATE();
        PH.setPack(p90);
        p90.time_usec_SET(6067691104083872128L);
        p90.roll_SET(4.3421186E37F);
        p90.pitch_SET(3.2509597E38F);
        p90.yaw_SET(-3.3519278E37F);
        p90.rollspeed_SET(2.4747207E37F);
        p90.pitchspeed_SET(1.4178448E38F);
        p90.yawspeed_SET(-1.022497E38F);
        p90.lat_SET(2098943617);
        p90.lon_SET(-163441638);
        p90.alt_SET(2005911567);
        p90.vx_SET((short) -11347);
        p90.vy_SET((short)16901);
        p90.vz_SET((short) -16651);
        p90.xacc_SET((short)11839);
        p90.yacc_SET((short)17630);
        p90.zacc_SET((short) -26044);
        LoopBackDemoChannel.instance.send(p90); //===============================
        HIL_CONTROLS p91 = LoopBackDemoChannel.instance.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.time_usec_SET(7241562600162842815L);
        p91.roll_ailerons_SET(-1.7179007E37F);
        p91.pitch_elevator_SET(2.0378976E38F);
        p91.yaw_rudder_SET(2.070673E38F);
        p91.throttle_SET(-6.886034E37F);
        p91.aux1_SET(3.2480507E37F);
        p91.aux2_SET(8.4688943E37F);
        p91.aux3_SET(8.8582885E36F);
        p91.aux4_SET(2.3934635E38F);
        p91.mode_SET(MAV_MODE.MAV_MODE_AUTO_ARMED);
        p91.nav_mode_SET((char)179);
        LoopBackDemoChannel.instance.send(p91); //===============================
        HIL_RC_INPUTS_RAW p92 = LoopBackDemoChannel.instance.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.time_usec_SET(5891455193597833205L);
        p92.chan1_raw_SET((char)36384);
        p92.chan2_raw_SET((char)13631);
        p92.chan3_raw_SET((char)33629);
        p92.chan4_raw_SET((char)52620);
        p92.chan5_raw_SET((char)9031);
        p92.chan6_raw_SET((char)8187);
        p92.chan7_raw_SET((char)46778);
        p92.chan8_raw_SET((char)46243);
        p92.chan9_raw_SET((char)12307);
        p92.chan10_raw_SET((char)24465);
        p92.chan11_raw_SET((char)39085);
        p92.chan12_raw_SET((char)40125);
        p92.rssi_SET((char)124);
        LoopBackDemoChannel.instance.send(p92); //===============================
        HIL_ACTUATOR_CONTROLS p93 = LoopBackDemoChannel.instance.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.time_usec_SET(8575937285871525234L);
        p93.controls_SET(new float[16], 0);
        p93.mode_SET(MAV_MODE.MAV_MODE_TEST_ARMED);
        p93.flags_SET(487937733022297111L);
        LoopBackDemoChannel.instance.send(p93); //===============================
        OPTICAL_FLOW p100 = LoopBackDemoChannel.instance.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.time_usec_SET(2923435370760664119L);
        p100.sensor_id_SET((char)39);
        p100.flow_x_SET((short)18846);
        p100.flow_y_SET((short) -9336);
        p100.flow_comp_m_x_SET(9.291344E36F);
        p100.flow_comp_m_y_SET(-2.7634164E38F);
        p100.quality_SET((char)34);
        p100.ground_distance_SET(2.632802E38F);
        p100.flow_rate_x_SET(1.432355E38F, PH);
        p100.flow_rate_y_SET(1.4245247E38F, PH);
        LoopBackDemoChannel.instance.send(p100); //===============================
        GLOBAL_VISION_POSITION_ESTIMATE p101 = LoopBackDemoChannel.instance.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.usec_SET(2415229429170744745L);
        p101.x_SET(-2.6055292E38F);
        p101.y_SET(1.7380297E38F);
        p101.z_SET(-1.3802128E37F);
        p101.roll_SET(-3.0250605E38F);
        p101.pitch_SET(3.1802396E38F);
        p101.yaw_SET(-7.108983E37F);
        LoopBackDemoChannel.instance.send(p101); //===============================
        VISION_POSITION_ESTIMATE p102 = LoopBackDemoChannel.instance.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.usec_SET(6982695573343178295L);
        p102.x_SET(-7.676338E37F);
        p102.y_SET(5.086772E37F);
        p102.z_SET(-3.3180515E35F);
        p102.roll_SET(4.605669E37F);
        p102.pitch_SET(-8.911697E37F);
        p102.yaw_SET(-2.6279557E38F);
        LoopBackDemoChannel.instance.send(p102); //===============================
        VISION_SPEED_ESTIMATE p103 = LoopBackDemoChannel.instance.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.usec_SET(6984847537819138274L);
        p103.x_SET(3.0650084E38F);
        p103.y_SET(-2.3289516E38F);
        p103.z_SET(2.1500046E37F);
        LoopBackDemoChannel.instance.send(p103); //===============================
        VICON_POSITION_ESTIMATE p104 = LoopBackDemoChannel.instance.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.usec_SET(4360157043355452315L);
        p104.x_SET(-2.5241923E38F);
        p104.y_SET(1.8542226E38F);
        p104.z_SET(-2.8360559E38F);
        p104.roll_SET(2.5593328E38F);
        p104.pitch_SET(-9.844986E37F);
        p104.yaw_SET(-2.5899487E38F);
        LoopBackDemoChannel.instance.send(p104); //===============================
        HIGHRES_IMU p105 = LoopBackDemoChannel.instance.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.time_usec_SET(8750770958047360931L);
        p105.xacc_SET(-1.2462094E38F);
        p105.yacc_SET(-1.268066E38F);
        p105.zacc_SET(2.6357237E38F);
        p105.xgyro_SET(-1.3501388E38F);
        p105.ygyro_SET(2.9594003E38F);
        p105.zgyro_SET(1.4288253E38F);
        p105.xmag_SET(-4.1976526E37F);
        p105.ymag_SET(1.5283995E38F);
        p105.zmag_SET(4.609134E37F);
        p105.abs_pressure_SET(-1.8128292E38F);
        p105.diff_pressure_SET(-3.397748E38F);
        p105.pressure_alt_SET(-2.5177385E38F);
        p105.temperature_SET(3.336296E38F);
        p105.fields_updated_SET((char)24738);
        LoopBackDemoChannel.instance.send(p105); //===============================
        OPTICAL_FLOW_RAD p106 = LoopBackDemoChannel.instance.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.time_usec_SET(7149045055985930384L);
        p106.sensor_id_SET((char)200);
        p106.integration_time_us_SET(4042475072L);
        p106.integrated_x_SET(1.9684985E38F);
        p106.integrated_y_SET(2.468392E37F);
        p106.integrated_xgyro_SET(3.005221E38F);
        p106.integrated_ygyro_SET(-5.2871E37F);
        p106.integrated_zgyro_SET(1.9011133E38F);
        p106.temperature_SET((short)22850);
        p106.quality_SET((char)196);
        p106.time_delta_distance_us_SET(1140709690L);
        p106.distance_SET(-6.2278427E37F);
        LoopBackDemoChannel.instance.send(p106); //===============================
        HIL_SENSOR p107 = LoopBackDemoChannel.instance.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.time_usec_SET(4435292738238916530L);
        p107.xacc_SET(2.5786634E38F);
        p107.yacc_SET(-1.9367336E38F);
        p107.zacc_SET(1.2607204E38F);
        p107.xgyro_SET(1.821069E38F);
        p107.ygyro_SET(1.4570758E38F);
        p107.zgyro_SET(-9.2539844E36F);
        p107.xmag_SET(-3.3194858E37F);
        p107.ymag_SET(1.0682782E38F);
        p107.zmag_SET(2.439053E38F);
        p107.abs_pressure_SET(-8.19795E37F);
        p107.diff_pressure_SET(6.3447515E37F);
        p107.pressure_alt_SET(-2.8005444E37F);
        p107.temperature_SET(-1.2790751E38F);
        p107.fields_updated_SET(2820390496L);
        LoopBackDemoChannel.instance.send(p107); //===============================
        SIM_STATE p108 = LoopBackDemoChannel.instance.new_SIM_STATE();
        PH.setPack(p108);
        p108.q1_SET(-1.4270302E38F);
        p108.q2_SET(-2.9054363E38F);
        p108.q3_SET(-8.376749E37F);
        p108.q4_SET(-1.952882E38F);
        p108.roll_SET(-3.2530926E38F);
        p108.pitch_SET(-1.843219E38F);
        p108.yaw_SET(3.2356763E36F);
        p108.xacc_SET(1.3010194E38F);
        p108.yacc_SET(2.7162087E38F);
        p108.zacc_SET(-1.3186769E38F);
        p108.xgyro_SET(4.6519344E37F);
        p108.ygyro_SET(-2.0988207E38F);
        p108.zgyro_SET(8.571982E37F);
        p108.lat_SET(-3.3587662E38F);
        p108.lon_SET(-2.823029E38F);
        p108.alt_SET(2.3900264E38F);
        p108.std_dev_horz_SET(-1.2417896E38F);
        p108.std_dev_vert_SET(-2.6006742E38F);
        p108.vn_SET(-1.6799415E38F);
        p108.ve_SET(1.3441373E38F);
        p108.vd_SET(-2.3006122E38F);
        LoopBackDemoChannel.instance.send(p108); //===============================
        RADIO_STATUS p109 = LoopBackDemoChannel.instance.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.rssi_SET((char)98);
        p109.remrssi_SET((char)32);
        p109.txbuf_SET((char)66);
        p109.noise_SET((char)153);
        p109.remnoise_SET((char)27);
        p109.rxerrors_SET((char)65164);
        p109.fixed__SET((char)18521);
        LoopBackDemoChannel.instance.send(p109); //===============================
        FILE_TRANSFER_PROTOCOL p110 = LoopBackDemoChannel.instance.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.target_network_SET((char)160);
        p110.target_system_SET((char)217);
        p110.target_component_SET((char)50);
        p110.payload_SET(new char[251], 0);
        LoopBackDemoChannel.instance.send(p110); //===============================
        TIMESYNC p111 = LoopBackDemoChannel.instance.new_TIMESYNC();
        PH.setPack(p111);
        p111.tc1_SET(4142109831302172466L);
        p111.ts1_SET(1725466965885772254L);
        LoopBackDemoChannel.instance.send(p111); //===============================
        CAMERA_TRIGGER p112 = LoopBackDemoChannel.instance.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.time_usec_SET(5825663297709304507L);
        p112.seq_SET(1639203464L);
        LoopBackDemoChannel.instance.send(p112); //===============================
        HIL_GPS p113 = LoopBackDemoChannel.instance.new_HIL_GPS();
        PH.setPack(p113);
        p113.time_usec_SET(1178673837542367586L);
        p113.fix_type_SET((char)45);
        p113.lat_SET(-1158591733);
        p113.lon_SET(-1048176173);
        p113.alt_SET(930457424);
        p113.eph_SET((char)12362);
        p113.epv_SET((char)174);
        p113.vel_SET((char)32013);
        p113.vn_SET((short)18589);
        p113.ve_SET((short) -5730);
        p113.vd_SET((short)14444);
        p113.cog_SET((char)49374);
        p113.satellites_visible_SET((char)58);
        LoopBackDemoChannel.instance.send(p113); //===============================
        HIL_OPTICAL_FLOW p114 = LoopBackDemoChannel.instance.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.time_usec_SET(3404908012756018261L);
        p114.sensor_id_SET((char)24);
        p114.integration_time_us_SET(195405934L);
        p114.integrated_x_SET(1.4675752E38F);
        p114.integrated_y_SET(2.1037363E38F);
        p114.integrated_xgyro_SET(2.3531195E38F);
        p114.integrated_ygyro_SET(-2.741089E38F);
        p114.integrated_zgyro_SET(2.4691672E38F);
        p114.temperature_SET((short)26813);
        p114.quality_SET((char)145);
        p114.time_delta_distance_us_SET(1668444592L);
        p114.distance_SET(-6.2914797E37F);
        LoopBackDemoChannel.instance.send(p114); //===============================
        HIL_STATE_QUATERNION p115 = LoopBackDemoChannel.instance.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.time_usec_SET(2597162835869962196L);
        p115.attitude_quaternion_SET(new float[4], 0);
        p115.rollspeed_SET(-1.9695009E38F);
        p115.pitchspeed_SET(3.100567E38F);
        p115.yawspeed_SET(1.4314776E37F);
        p115.lat_SET(690794607);
        p115.lon_SET(-1731040202);
        p115.alt_SET(2135301302);
        p115.vx_SET((short) -12521);
        p115.vy_SET((short) -6926);
        p115.vz_SET((short) -30453);
        p115.ind_airspeed_SET((char)25824);
        p115.true_airspeed_SET((char)53897);
        p115.xacc_SET((short)2233);
        p115.yacc_SET((short)7346);
        p115.zacc_SET((short) -4865);
        LoopBackDemoChannel.instance.send(p115); //===============================
        SCALED_IMU2 p116 = LoopBackDemoChannel.instance.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.time_boot_ms_SET(1907211053L);
        p116.xacc_SET((short) -23246);
        p116.yacc_SET((short) -15590);
        p116.zacc_SET((short)19868);
        p116.xgyro_SET((short) -7064);
        p116.ygyro_SET((short) -9433);
        p116.zgyro_SET((short)3064);
        p116.xmag_SET((short) -8186);
        p116.ymag_SET((short) -1903);
        p116.zmag_SET((short) -28011);
        LoopBackDemoChannel.instance.send(p116); //===============================
        LOG_REQUEST_LIST p117 = LoopBackDemoChannel.instance.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.target_system_SET((char)170);
        p117.target_component_SET((char)162);
        p117.start_SET((char)26823);
        p117.end_SET((char)21511);
        LoopBackDemoChannel.instance.send(p117); //===============================
        LOG_ENTRY p118 = LoopBackDemoChannel.instance.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.id_SET((char)7459);
        p118.num_logs_SET((char)3684);
        p118.last_log_num_SET((char)33747);
        p118.time_utc_SET(164228303L);
        p118.size_SET(2518141592L);
        LoopBackDemoChannel.instance.send(p118); //===============================
        LOG_REQUEST_DATA p119 = LoopBackDemoChannel.instance.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.target_system_SET((char)33);
        p119.target_component_SET((char)147);
        p119.id_SET((char)54201);
        p119.ofs_SET(231091461L);
        p119.count_SET(548482303L);
        LoopBackDemoChannel.instance.send(p119); //===============================
        LOG_DATA p120 = LoopBackDemoChannel.instance.new_LOG_DATA();
        PH.setPack(p120);
        p120.id_SET((char)53854);
        p120.ofs_SET(1673565238L);
        p120.count_SET((char)215);
        p120.data__SET(new char[90], 0);
        LoopBackDemoChannel.instance.send(p120); //===============================
        LOG_ERASE p121 = LoopBackDemoChannel.instance.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_system_SET((char)18);
        p121.target_component_SET((char)107);
        LoopBackDemoChannel.instance.send(p121); //===============================
        LOG_REQUEST_END p122 = LoopBackDemoChannel.instance.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_system_SET((char)104);
        p122.target_component_SET((char)29);
        LoopBackDemoChannel.instance.send(p122); //===============================
        GPS_INJECT_DATA p123 = LoopBackDemoChannel.instance.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.target_system_SET((char)247);
        p123.target_component_SET((char)151);
        p123.len_SET((char)227);
        p123.data__SET(new char[110], 0);
        LoopBackDemoChannel.instance.send(p123); //===============================
        GPS2_RAW p124 = LoopBackDemoChannel.instance.new_GPS2_RAW();
        PH.setPack(p124);
        p124.time_usec_SET(3828613007500787344L);
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX);
        p124.lat_SET(802241309);
        p124.lon_SET(-1402788936);
        p124.alt_SET(-834460609);
        p124.eph_SET((char)55936);
        p124.epv_SET((char)24189);
        p124.vel_SET((char)64634);
        p124.cog_SET((char)62435);
        p124.satellites_visible_SET((char)94);
        p124.dgps_numch_SET((char)175);
        p124.dgps_age_SET(29025490L);
        LoopBackDemoChannel.instance.send(p124); //===============================
        POWER_STATUS p125 = LoopBackDemoChannel.instance.new_POWER_STATUS();
        PH.setPack(p125);
        p125.Vcc_SET((char)41136);
        p125.Vservo_SET((char)33713);
        p125.flags_SET(MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT);
        LoopBackDemoChannel.instance.send(p125); //===============================
        SERIAL_CONTROL p126 = LoopBackDemoChannel.instance.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS2);
        p126.flags_SET(SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE);
        p126.timeout_SET((char)1298);
        p126.baudrate_SET(361682848L);
        p126.count_SET((char)187);
        p126.data__SET(new char[70], 0);
        LoopBackDemoChannel.instance.send(p126); //===============================
        GPS_RTK p127 = LoopBackDemoChannel.instance.new_GPS_RTK();
        PH.setPack(p127);
        p127.time_last_baseline_ms_SET(2424652931L);
        p127.rtk_receiver_id_SET((char)162);
        p127.wn_SET((char)10043);
        p127.tow_SET(3721433858L);
        p127.rtk_health_SET((char)33);
        p127.rtk_rate_SET((char)94);
        p127.nsats_SET((char)209);
        p127.baseline_coords_type_SET((char)9);
        p127.baseline_a_mm_SET(-1077162314);
        p127.baseline_b_mm_SET(955446124);
        p127.baseline_c_mm_SET(-419218308);
        p127.accuracy_SET(246809996L);
        p127.iar_num_hypotheses_SET(-40121369);
        LoopBackDemoChannel.instance.send(p127); //===============================
        GPS2_RTK p128 = LoopBackDemoChannel.instance.new_GPS2_RTK();
        PH.setPack(p128);
        p128.time_last_baseline_ms_SET(3606850913L);
        p128.rtk_receiver_id_SET((char)127);
        p128.wn_SET((char)57613);
        p128.tow_SET(3203961657L);
        p128.rtk_health_SET((char)247);
        p128.rtk_rate_SET((char)65);
        p128.nsats_SET((char)57);
        p128.baseline_coords_type_SET((char)29);
        p128.baseline_a_mm_SET(-1003679682);
        p128.baseline_b_mm_SET(1862275822);
        p128.baseline_c_mm_SET(-1388267762);
        p128.accuracy_SET(2072345309L);
        p128.iar_num_hypotheses_SET(-244133057);
        LoopBackDemoChannel.instance.send(p128); //===============================
        SCALED_IMU3 p129 = LoopBackDemoChannel.instance.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.time_boot_ms_SET(1488465253L);
        p129.xacc_SET((short)24619);
        p129.yacc_SET((short) -10468);
        p129.zacc_SET((short)32197);
        p129.xgyro_SET((short) -32137);
        p129.ygyro_SET((short)28578);
        p129.zgyro_SET((short)31769);
        p129.xmag_SET((short) -12468);
        p129.ymag_SET((short)24880);
        p129.zmag_SET((short) -28972);
        LoopBackDemoChannel.instance.send(p129); //===============================
        DATA_TRANSMISSION_HANDSHAKE p130 = LoopBackDemoChannel.instance.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.type_SET((char)17);
        p130.size_SET(355130235L);
        p130.width_SET((char)34939);
        p130.height_SET((char)20336);
        p130.packets_SET((char)55357);
        p130.payload_SET((char)118);
        p130.jpg_quality_SET((char)126);
        LoopBackDemoChannel.instance.send(p130); //===============================
        ENCAPSULATED_DATA p131 = LoopBackDemoChannel.instance.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.seqnr_SET((char)58586);
        p131.data__SET(new char[253], 0);
        LoopBackDemoChannel.instance.send(p131); //===============================
        DISTANCE_SENSOR p132 = LoopBackDemoChannel.instance.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.time_boot_ms_SET(124186991L);
        p132.min_distance_SET((char)31627);
        p132.max_distance_SET((char)1150);
        p132.current_distance_SET((char)53324);
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN);
        p132.id_SET((char)86);
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_90_YAW_270);
        p132.covariance_SET((char)72);
        LoopBackDemoChannel.instance.send(p132); //===============================
        TERRAIN_REQUEST p133 = LoopBackDemoChannel.instance.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.lat_SET(-645091004);
        p133.lon_SET(219853487);
        p133.grid_spacing_SET((char)31880);
        p133.mask_SET(7195589248122358949L);
        LoopBackDemoChannel.instance.send(p133); //===============================
        TERRAIN_DATA p134 = LoopBackDemoChannel.instance.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.lat_SET(175276333);
        p134.lon_SET(509476101);
        p134.grid_spacing_SET((char)8137);
        p134.gridbit_SET((char)89);
        p134.data__SET(new short[16], 0);
        LoopBackDemoChannel.instance.send(p134); //===============================
        TERRAIN_CHECK p135 = LoopBackDemoChannel.instance.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lat_SET(541110997);
        p135.lon_SET(-1648960858);
        LoopBackDemoChannel.instance.send(p135); //===============================
        TERRAIN_REPORT p136 = LoopBackDemoChannel.instance.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.lat_SET(413008940);
        p136.lon_SET(1818682488);
        p136.spacing_SET((char)32538);
        p136.terrain_height_SET(2.367644E38F);
        p136.current_height_SET(-1.8430398E38F);
        p136.pending_SET((char)57917);
        p136.loaded_SET((char)29240);
        LoopBackDemoChannel.instance.send(p136); //===============================
        SCALED_PRESSURE2 p137 = LoopBackDemoChannel.instance.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.time_boot_ms_SET(1262862275L);
        p137.press_abs_SET(-3.336889E38F);
        p137.press_diff_SET(2.2727383E38F);
        p137.temperature_SET((short) -14145);
        LoopBackDemoChannel.instance.send(p137); //===============================
        ATT_POS_MOCAP p138 = LoopBackDemoChannel.instance.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.time_usec_SET(4674056951479133562L);
        p138.q_SET(new float[4], 0);
        p138.x_SET(-2.6473386E38F);
        p138.y_SET(3.216011E38F);
        p138.z_SET(2.5579074E38F);
        LoopBackDemoChannel.instance.send(p138); //===============================
        SET_ACTUATOR_CONTROL_TARGET p139 = LoopBackDemoChannel.instance.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.time_usec_SET(3449778076492003944L);
        p139.group_mlx_SET((char)139);
        p139.target_system_SET((char)229);
        p139.target_component_SET((char)133);
        p139.controls_SET(new float[8], 0);
        LoopBackDemoChannel.instance.send(p139); //===============================
        ACTUATOR_CONTROL_TARGET p140 = LoopBackDemoChannel.instance.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.time_usec_SET(3107159322140467800L);
        p140.group_mlx_SET((char)223);
        p140.controls_SET(new float[8], 0);
        LoopBackDemoChannel.instance.send(p140); //===============================
        ALTITUDE p141 = LoopBackDemoChannel.instance.new_ALTITUDE();
        PH.setPack(p141);
        p141.time_usec_SET(5507109111286749006L);
        p141.altitude_monotonic_SET(2.6997285E38F);
        p141.altitude_amsl_SET(2.4620833E38F);
        p141.altitude_local_SET(-2.3041185E37F);
        p141.altitude_relative_SET(3.3179176E38F);
        p141.altitude_terrain_SET(3.2596921E38F);
        p141.bottom_clearance_SET(3.2524973E38F);
        LoopBackDemoChannel.instance.send(p141); //===============================
        RESOURCE_REQUEST p142 = LoopBackDemoChannel.instance.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.request_id_SET((char)185);
        p142.uri_type_SET((char)8);
        p142.uri_SET(new char[120], 0);
        p142.transfer_type_SET((char)166);
        p142.storage_SET(new char[120], 0);
        LoopBackDemoChannel.instance.send(p142); //===============================
        SCALED_PRESSURE3 p143 = LoopBackDemoChannel.instance.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.time_boot_ms_SET(4106070296L);
        p143.press_abs_SET(-3.2709865E37F);
        p143.press_diff_SET(1.3401444E38F);
        p143.temperature_SET((short) -26525);
        LoopBackDemoChannel.instance.send(p143); //===============================
        FOLLOW_TARGET p144 = LoopBackDemoChannel.instance.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.timestamp_SET(692849988271083840L);
        p144.est_capabilities_SET((char)2);
        p144.lat_SET(-1145772336);
        p144.lon_SET(334191312);
        p144.alt_SET(6.358022E37F);
        p144.vel_SET(new float[3], 0);
        p144.acc_SET(new float[3], 0);
        p144.attitude_q_SET(new float[4], 0);
        p144.rates_SET(new float[3], 0);
        p144.position_cov_SET(new float[3], 0);
        p144.custom_state_SET(7161427585712834785L);
        LoopBackDemoChannel.instance.send(p144); //===============================
        CONTROL_SYSTEM_STATE p146 = LoopBackDemoChannel.instance.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.time_usec_SET(8757128563775169704L);
        p146.x_acc_SET(-2.2181164E38F);
        p146.y_acc_SET(-4.9990527E37F);
        p146.z_acc_SET(1.4948361E38F);
        p146.x_vel_SET(-1.7700704E38F);
        p146.y_vel_SET(-1.7886384E38F);
        p146.z_vel_SET(2.070729E38F);
        p146.x_pos_SET(1.5315211E37F);
        p146.y_pos_SET(1.7342069E38F);
        p146.z_pos_SET(-2.1775262E38F);
        p146.airspeed_SET(-3.1584657E38F);
        p146.vel_variance_SET(new float[3], 0);
        p146.pos_variance_SET(new float[3], 0);
        p146.q_SET(new float[4], 0);
        p146.roll_rate_SET(-2.8631493E38F);
        p146.pitch_rate_SET(3.3280067E38F);
        p146.yaw_rate_SET(-2.304721E38F);
        LoopBackDemoChannel.instance.send(p146); //===============================
        BATTERY_STATUS p147 = LoopBackDemoChannel.instance.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.id_SET((char)188);
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD);
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIFE);
        p147.temperature_SET((short) -15463);
        p147.voltages_SET(new char[10], 0);
        p147.current_battery_SET((short) -29241);
        p147.current_consumed_SET(-1154978539);
        p147.energy_consumed_SET(1174084137);
        p147.battery_remaining_SET((byte) - 79);
        LoopBackDemoChannel.instance.send(p147); //===============================
        AUTOPILOT_VERSION p148 = LoopBackDemoChannel.instance.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.capabilities_SET(MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP);
        p148.flight_sw_version_SET(947491936L);
        p148.middleware_sw_version_SET(3943714025L);
        p148.os_sw_version_SET(1117255766L);
        p148.board_version_SET(818053305L);
        p148.flight_custom_version_SET(new char[8], 0);
        p148.middleware_custom_version_SET(new char[8], 0);
        p148.os_custom_version_SET(new char[8], 0);
        p148.vendor_id_SET((char)31193);
        p148.product_id_SET((char)61733);
        p148.uid_SET(5153808066425839926L);
        p148.uid2_SET(new char[18], 0, PH);
        LoopBackDemoChannel.instance.send(p148); //===============================
        LANDING_TARGET p149 = LoopBackDemoChannel.instance.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.time_usec_SET(101148598817880198L);
        p149.target_num_SET((char)167);
        p149.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
        p149.angle_x_SET(3.3030652E38F);
        p149.angle_y_SET(4.486568E37F);
        p149.distance_SET(6.5132456E37F);
        p149.size_x_SET(-2.4848246E38F);
        p149.size_y_SET(6.2875E37F);
        p149.x_SET(1.4965109E38F, PH);
        p149.y_SET(2.278877E38F, PH);
        p149.z_SET(-6.5793323E37F, PH);
        p149.q_SET(new float[4], 0, PH);
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER);
        p149.position_valid_SET((char)105, PH);
        LoopBackDemoChannel.instance.send(p149); //===============================
        CPU_LOAD p170 = LoopBackDemoChannel.instance.new_CPU_LOAD();
        PH.setPack(p170);
        p170.sensLoad_SET((char)252);
        p170.ctrlLoad_SET((char)38);
        p170.batVolt_SET((char)8440);
        LoopBackDemoChannel.instance.send(p170); //===============================
        SENSOR_BIAS p172 = LoopBackDemoChannel.instance.new_SENSOR_BIAS();
        PH.setPack(p172);
        p172.axBias_SET(3.392314E38F);
        p172.ayBias_SET(-1.84117E38F);
        p172.azBias_SET(-5.7109155E37F);
        p172.gxBias_SET(-1.9154492E37F);
        p172.gyBias_SET(7.419435E36F);
        p172.gzBias_SET(-1.5347166E38F);
        LoopBackDemoChannel.instance.send(p172); //===============================
        DIAGNOSTIC p173 = LoopBackDemoChannel.instance.new_DIAGNOSTIC();
        PH.setPack(p173);
        p173.diagFl1_SET(2.7902971E38F);
        p173.diagFl2_SET(1.785915E38F);
        p173.diagFl3_SET(-2.978072E38F);
        p173.diagSh1_SET((short) -24201);
        p173.diagSh2_SET((short)1520);
        p173.diagSh3_SET((short) -32442);
        LoopBackDemoChannel.instance.send(p173); //===============================
        SLUGS_NAVIGATION p176 = LoopBackDemoChannel.instance.new_SLUGS_NAVIGATION();
        PH.setPack(p176);
        p176.u_m_SET(2.648525E38F);
        p176.phi_c_SET(-5.934059E37F);
        p176.theta_c_SET(-2.4748602E38F);
        p176.psiDot_c_SET(1.1833319E37F);
        p176.ay_body_SET(8.4380757E37F);
        p176.totalDist_SET(-1.9617005E37F);
        p176.dist2Go_SET(3.3366454E38F);
        p176.fromWP_SET((char)68);
        p176.toWP_SET((char)90);
        p176.h_c_SET((char)15435);
        LoopBackDemoChannel.instance.send(p176); //===============================
        DATA_LOG p177 = LoopBackDemoChannel.instance.new_DATA_LOG();
        PH.setPack(p177);
        p177.fl_1_SET(2.6735388E38F);
        p177.fl_2_SET(-2.6541742E38F);
        p177.fl_3_SET(-1.2234595E38F);
        p177.fl_4_SET(1.1353793E38F);
        p177.fl_5_SET(-3.1724092E38F);
        p177.fl_6_SET(-1.254906E38F);
        LoopBackDemoChannel.instance.send(p177); //===============================
        GPS_DATE_TIME p179 = LoopBackDemoChannel.instance.new_GPS_DATE_TIME();
        PH.setPack(p179);
        p179.year_SET((char)235);
        p179.month_SET((char)78);
        p179.day_SET((char)120);
        p179.hour_SET((char)23);
        p179.min_SET((char)233);
        p179.sec_SET((char)88);
        p179.clockStat_SET((char)104);
        p179.visSat_SET((char)134);
        p179.useSat_SET((char)23);
        p179.GppGl_SET((char)168);
        p179.sigUsedMask_SET((char)77);
        p179.percentUsed_SET((char)32);
        LoopBackDemoChannel.instance.send(p179); //===============================
        MID_LVL_CMDS p180 = LoopBackDemoChannel.instance.new_MID_LVL_CMDS();
        PH.setPack(p180);
        p180.target_SET((char)22);
        p180.hCommand_SET(-2.6664004E38F);
        p180.uCommand_SET(2.598176E38F);
        p180.rCommand_SET(1.5538365E38F);
        LoopBackDemoChannel.instance.send(p180); //===============================
        CTRL_SRFC_PT p181 = LoopBackDemoChannel.instance.new_CTRL_SRFC_PT();
        PH.setPack(p181);
        p181.target_SET((char)46);
        p181.bitfieldPt_SET((char)43068);
        LoopBackDemoChannel.instance.send(p181); //===============================
        SLUGS_CAMERA_ORDER p184 = LoopBackDemoChannel.instance.new_SLUGS_CAMERA_ORDER();
        PH.setPack(p184);
        p184.target_SET((char)147);
        p184.pan_SET((byte) - 59);
        p184.tilt_SET((byte) - 97);
        p184.zoom_SET((byte) - 30);
        p184.moveHome_SET((byte)72);
        LoopBackDemoChannel.instance.send(p184); //===============================
        CONTROL_SURFACE p185 = LoopBackDemoChannel.instance.new_CONTROL_SURFACE();
        PH.setPack(p185);
        p185.target_SET((char)105);
        p185.idSurface_SET((char)147);
        p185.mControl_SET(3.1900667E38F);
        p185.bControl_SET(1.7068023E38F);
        LoopBackDemoChannel.instance.send(p185); //===============================
        SLUGS_MOBILE_LOCATION p186 = LoopBackDemoChannel.instance.new_SLUGS_MOBILE_LOCATION();
        PH.setPack(p186);
        p186.target_SET((char)175);
        p186.latitude_SET(7.521348E37F);
        p186.longitude_SET(-2.8489778E37F);
        LoopBackDemoChannel.instance.send(p186); //===============================
        SLUGS_CONFIGURATION_CAMERA p188 = LoopBackDemoChannel.instance.new_SLUGS_CONFIGURATION_CAMERA();
        PH.setPack(p188);
        p188.target_SET((char)235);
        p188.idOrder_SET((char)80);
        p188.order_SET((char)169);
        LoopBackDemoChannel.instance.send(p188); //===============================
        ISR_LOCATION p189 = LoopBackDemoChannel.instance.new_ISR_LOCATION();
        PH.setPack(p189);
        p189.target_SET((char)122);
        p189.latitude_SET(-3.2628245E38F);
        p189.longitude_SET(1.7300254E38F);
        p189.height_SET(-3.0305157E38F);
        p189.option1_SET((char)244);
        p189.option2_SET((char)38);
        p189.option3_SET((char)64);
        LoopBackDemoChannel.instance.send(p189); //===============================
        VOLT_SENSOR p191 = LoopBackDemoChannel.instance.new_VOLT_SENSOR();
        PH.setPack(p191);
        p191.r2Type_SET((char)80);
        p191.voltage_SET((char)56850);
        p191.reading2_SET((char)8227);
        LoopBackDemoChannel.instance.send(p191); //===============================
        PTZ_STATUS p192 = LoopBackDemoChannel.instance.new_PTZ_STATUS();
        PH.setPack(p192);
        p192.zoom_SET((char)159);
        p192.pan_SET((short)11810);
        p192.tilt_SET((short) -9114);
        LoopBackDemoChannel.instance.send(p192); //===============================
        UAV_STATUS p193 = LoopBackDemoChannel.instance.new_UAV_STATUS();
        PH.setPack(p193);
        p193.target_SET((char)176);
        p193.latitude_SET(5.5721245E37F);
        p193.longitude_SET(-2.4419654E38F);
        p193.altitude_SET(8.210837E36F);
        p193.speed_SET(2.0303144E38F);
        p193.course_SET(-2.2012444E38F);
        LoopBackDemoChannel.instance.send(p193); //===============================
        STATUS_GPS p194 = LoopBackDemoChannel.instance.new_STATUS_GPS();
        PH.setPack(p194);
        p194.csFails_SET((char)25860);
        p194.gpsQuality_SET((char)34);
        p194.msgsType_SET((char)197);
        p194.posStatus_SET((char)152);
        p194.magVar_SET(2.2352138E38F);
        p194.magDir_SET((byte) - 24);
        p194.modeInd_SET((char)147);
        LoopBackDemoChannel.instance.send(p194); //===============================
        NOVATEL_DIAG p195 = LoopBackDemoChannel.instance.new_NOVATEL_DIAG();
        PH.setPack(p195);
        p195.timeStatus_SET((char)250);
        p195.receiverStatus_SET(176440936L);
        p195.solStatus_SET((char)81);
        p195.posType_SET((char)171);
        p195.velType_SET((char)173);
        p195.posSolAge_SET(-3.1587237E38F);
        p195.csFails_SET((char)6269);
        LoopBackDemoChannel.instance.send(p195); //===============================
        SENSOR_DIAG p196 = LoopBackDemoChannel.instance.new_SENSOR_DIAG();
        PH.setPack(p196);
        p196.float1_SET(-2.6339088E38F);
        p196.float2_SET(1.4087831E38F);
        p196.int1_SET((short)15264);
        p196.char1_SET((byte) - 33);
        LoopBackDemoChannel.instance.send(p196); //===============================
        BOOT p197 = LoopBackDemoChannel.instance.new_BOOT();
        PH.setPack(p197);
        p197.version_SET(1823965619L);
        LoopBackDemoChannel.instance.send(p197); //===============================
        ESTIMATOR_STATUS p230 = LoopBackDemoChannel.instance.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.time_usec_SET(1460722430698767161L);
        p230.flags_SET(ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE);
        p230.vel_ratio_SET(2.1742948E36F);
        p230.pos_horiz_ratio_SET(-2.45011E38F);
        p230.pos_vert_ratio_SET(-7.2233287E37F);
        p230.mag_ratio_SET(-9.073234E37F);
        p230.hagl_ratio_SET(-3.2575746E38F);
        p230.tas_ratio_SET(2.8385405E38F);
        p230.pos_horiz_accuracy_SET(-3.533943E37F);
        p230.pos_vert_accuracy_SET(-7.4694E37F);
        LoopBackDemoChannel.instance.send(p230); //===============================
        WIND_COV p231 = LoopBackDemoChannel.instance.new_WIND_COV();
        PH.setPack(p231);
        p231.time_usec_SET(1283059784101332626L);
        p231.wind_x_SET(4.786342E37F);
        p231.wind_y_SET(1.5949331E38F);
        p231.wind_z_SET(-1.4585529E38F);
        p231.var_horiz_SET(-1.2492629E38F);
        p231.var_vert_SET(-9.861742E37F);
        p231.wind_alt_SET(1.6144676E38F);
        p231.horiz_accuracy_SET(1.2769128E38F);
        p231.vert_accuracy_SET(-1.2613842E38F);
        LoopBackDemoChannel.instance.send(p231); //===============================
        GPS_INPUT p232 = LoopBackDemoChannel.instance.new_GPS_INPUT();
        PH.setPack(p232);
        p232.time_usec_SET(1183077060877989223L);
        p232.gps_id_SET((char)183);
        p232.ignore_flags_SET(GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT);
        p232.time_week_ms_SET(2642969681L);
        p232.time_week_SET((char)55815);
        p232.fix_type_SET((char)73);
        p232.lat_SET(-1443417360);
        p232.lon_SET(539366837);
        p232.alt_SET(6.9075305E37F);
        p232.hdop_SET(4.3071517E37F);
        p232.vdop_SET(-1.5827159E37F);
        p232.vn_SET(1.1101804E38F);
        p232.ve_SET(-1.1550634E38F);
        p232.vd_SET(2.4765996E37F);
        p232.speed_accuracy_SET(3.8782393E37F);
        p232.horiz_accuracy_SET(-5.9692836E36F);
        p232.vert_accuracy_SET(2.7374744E38F);
        p232.satellites_visible_SET((char)226);
        LoopBackDemoChannel.instance.send(p232); //===============================
        GPS_RTCM_DATA p233 = LoopBackDemoChannel.instance.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.flags_SET((char)194);
        p233.len_SET((char)156);
        p233.data__SET(new char[180], 0);
        LoopBackDemoChannel.instance.send(p233); //===============================
        HIGH_LATENCY p234 = LoopBackDemoChannel.instance.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED);
        p234.custom_mode_SET(3552527851L);
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING);
        p234.roll_SET((short)18293);
        p234.pitch_SET((short) -21123);
        p234.heading_SET((char)62009);
        p234.throttle_SET((byte)98);
        p234.heading_sp_SET((short)9437);
        p234.latitude_SET(-1642153188);
        p234.longitude_SET(-1769000840);
        p234.altitude_amsl_SET((short)23672);
        p234.altitude_sp_SET((short) -8315);
        p234.airspeed_SET((char)158);
        p234.airspeed_sp_SET((char)143);
        p234.groundspeed_SET((char)108);
        p234.climb_rate_SET((byte)20);
        p234.gps_nsat_SET((char)84);
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED);
        p234.battery_remaining_SET((char)254);
        p234.temperature_SET((byte)71);
        p234.temperature_air_SET((byte)76);
        p234.failsafe_SET((char)74);
        p234.wp_num_SET((char)226);
        p234.wp_distance_SET((char)23821);
        LoopBackDemoChannel.instance.send(p234); //===============================
        VIBRATION p241 = LoopBackDemoChannel.instance.new_VIBRATION();
        PH.setPack(p241);
        p241.time_usec_SET(7461014190168008543L);
        p241.vibration_x_SET(1.0070564E38F);
        p241.vibration_y_SET(5.443688E37F);
        p241.vibration_z_SET(-1.734974E38F);
        p241.clipping_0_SET(328808436L);
        p241.clipping_1_SET(2178683451L);
        p241.clipping_2_SET(2230970167L);
        LoopBackDemoChannel.instance.send(p241); //===============================
        HOME_POSITION p242 = LoopBackDemoChannel.instance.new_HOME_POSITION();
        PH.setPack(p242);
        p242.latitude_SET(632866306);
        p242.longitude_SET(1871753254);
        p242.altitude_SET(1434417694);
        p242.x_SET(-4.3713004E37F);
        p242.y_SET(-2.534787E38F);
        p242.z_SET(3.2027042E38F);
        p242.q_SET(new float[4], 0);
        p242.approach_x_SET(1.4127265E37F);
        p242.approach_y_SET(-3.3573773E38F);
        p242.approach_z_SET(1.5832984E38F);
        p242.time_usec_SET(3928726411134700200L, PH);
        LoopBackDemoChannel.instance.send(p242); //===============================
        SET_HOME_POSITION p243 = LoopBackDemoChannel.instance.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.target_system_SET((char)98);
        p243.latitude_SET(-1849301768);
        p243.longitude_SET(183178360);
        p243.altitude_SET(959946082);
        p243.x_SET(1.2050032E37F);
        p243.y_SET(-8.948224E37F);
        p243.z_SET(1.3699866E38F);
        p243.q_SET(new float[4], 0);
        p243.approach_x_SET(-2.6657984E38F);
        p243.approach_y_SET(-7.6247784E37F);
        p243.approach_z_SET(2.159386E38F);
        p243.time_usec_SET(547810746640654492L, PH);
        LoopBackDemoChannel.instance.send(p243); //===============================
        MESSAGE_INTERVAL p244 = LoopBackDemoChannel.instance.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.message_id_SET((char)39258);
        p244.interval_us_SET(-699004580);
        LoopBackDemoChannel.instance.send(p244); //===============================
        EXTENDED_SYS_STATE p245 = LoopBackDemoChannel.instance.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_MC);
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF);
        LoopBackDemoChannel.instance.send(p245); //===============================
        ADSB_VEHICLE p246 = LoopBackDemoChannel.instance.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.ICAO_address_SET(976762470L);
        p246.lat_SET(359650089);
        p246.lon_SET(-2088245707);
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
        p246.altitude_SET(-130429955);
        p246.heading_SET((char)64127);
        p246.hor_velocity_SET((char)11664);
        p246.ver_velocity_SET((short)22930);
        p246.callsign_SET("DEMO", PH);
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_HEAVY);
        p246.tslc_SET((char)110);
        p246.flags_SET(ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN);
        p246.squawk_SET((char)24753);
        LoopBackDemoChannel.instance.send(p246); //===============================
        COLLISION p247 = LoopBackDemoChannel.instance.new_COLLISION();
        PH.setPack(p247);
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
        p247.id_SET(2466892612L);
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_ASCEND_OR_DESCEND);
        p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH);
        p247.time_to_minimum_delta_SET(3.109372E38F);
        p247.altitude_minimum_delta_SET(1.2542121E38F);
        p247.horizontal_minimum_delta_SET(-1.5459449E38F);
        LoopBackDemoChannel.instance.send(p247); //===============================
        V2_EXTENSION p248 = LoopBackDemoChannel.instance.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.target_network_SET((char)171);
        p248.target_system_SET((char)111);
        p248.target_component_SET((char)177);
        p248.message_type_SET((char)8913);
        p248.payload_SET(new char[249], 0);
        LoopBackDemoChannel.instance.send(p248); //===============================
        MEMORY_VECT p249 = LoopBackDemoChannel.instance.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.address_SET((char)5776);
        p249.ver_SET((char)79);
        p249.type_SET((char)50);
        p249.value_SET(new byte[32], 0);
        LoopBackDemoChannel.instance.send(p249); //===============================
        DEBUG_VECT p250 = LoopBackDemoChannel.instance.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.name_SET("DEMO", PH);
        p250.time_usec_SET(448782407284126643L);
        p250.x_SET(2.7964155E38F);
        p250.y_SET(6.7832105E37F);
        p250.z_SET(-2.655334E38F);
        LoopBackDemoChannel.instance.send(p250); //===============================
        NAMED_VALUE_FLOAT p251 = LoopBackDemoChannel.instance.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.time_boot_ms_SET(3297732361L);
        p251.name_SET("DEMO", PH);
        p251.value_SET(-2.8587426E38F);
        LoopBackDemoChannel.instance.send(p251); //===============================
        NAMED_VALUE_INT p252 = LoopBackDemoChannel.instance.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.time_boot_ms_SET(4280416213L);
        p252.name_SET("DEMO", PH);
        p252.value_SET(1115756816);
        LoopBackDemoChannel.instance.send(p252); //===============================
        STATUSTEXT p253 = LoopBackDemoChannel.instance.new_STATUSTEXT();
        PH.setPack(p253);
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_WARNING);
        p253.text_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p253); //===============================
        DEBUG p254 = LoopBackDemoChannel.instance.new_DEBUG();
        PH.setPack(p254);
        p254.time_boot_ms_SET(3256474437L);
        p254.ind_SET((char)233);
        p254.value_SET(2.0417215E38F);
        LoopBackDemoChannel.instance.send(p254); //===============================
        SETUP_SIGNING p256 = LoopBackDemoChannel.instance.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.target_system_SET((char)136);
        p256.target_component_SET((char)205);
        p256.secret_key_SET(new char[32], 0);
        p256.initial_timestamp_SET(2714481812916175128L);
        LoopBackDemoChannel.instance.send(p256); //===============================
        BUTTON_CHANGE p257 = LoopBackDemoChannel.instance.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.time_boot_ms_SET(3908409077L);
        p257.last_change_ms_SET(1184656336L);
        p257.state_SET((char)132);
        LoopBackDemoChannel.instance.send(p257); //===============================
        PLAY_TUNE p258 = LoopBackDemoChannel.instance.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.target_system_SET((char)95);
        p258.target_component_SET((char)6);
        p258.tune_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p258); //===============================
        CAMERA_INFORMATION p259 = LoopBackDemoChannel.instance.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.time_boot_ms_SET(1433245265L);
        p259.vendor_name_SET(new char[32], 0);
        p259.model_name_SET(new char[32], 0);
        p259.firmware_version_SET(364749032L);
        p259.focal_length_SET(1.862519E38F);
        p259.sensor_size_h_SET(2.4344443E38F);
        p259.sensor_size_v_SET(-2.6401399E36F);
        p259.resolution_h_SET((char)13163);
        p259.resolution_v_SET((char)39811);
        p259.lens_id_SET((char)148);
        p259.flags_SET(CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE);
        p259.cam_definition_version_SET((char)40746);
        p259.cam_definition_uri_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p259); //===============================
        CAMERA_SETTINGS p260 = LoopBackDemoChannel.instance.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.time_boot_ms_SET(3088639464L);
        p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_IMAGE);
        LoopBackDemoChannel.instance.send(p260); //===============================
        STORAGE_INFORMATION p261 = LoopBackDemoChannel.instance.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.time_boot_ms_SET(2428287394L);
        p261.storage_id_SET((char)186);
        p261.storage_count_SET((char)101);
        p261.status_SET((char)87);
        p261.total_capacity_SET(-2.6236802E38F);
        p261.used_capacity_SET(-1.5088867E38F);
        p261.available_capacity_SET(-1.3821969E37F);
        p261.read_speed_SET(-1.6259169E38F);
        p261.write_speed_SET(-2.341183E38F);
        LoopBackDemoChannel.instance.send(p261); //===============================
        CAMERA_CAPTURE_STATUS p262 = LoopBackDemoChannel.instance.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.time_boot_ms_SET(2144982855L);
        p262.image_status_SET((char)101);
        p262.video_status_SET((char)160);
        p262.image_interval_SET(3.2247628E38F);
        p262.recording_time_ms_SET(1580357282L);
        p262.available_capacity_SET(-2.4146413E38F);
        LoopBackDemoChannel.instance.send(p262); //===============================
        CAMERA_IMAGE_CAPTURED p263 = LoopBackDemoChannel.instance.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.time_boot_ms_SET(494313616L);
        p263.time_utc_SET(6217377903081909379L);
        p263.camera_id_SET((char)212);
        p263.lat_SET(854424334);
        p263.lon_SET(-1227468693);
        p263.alt_SET(-1185732273);
        p263.relative_alt_SET(-637956134);
        p263.q_SET(new float[4], 0);
        p263.image_index_SET(-1945192898);
        p263.capture_result_SET((byte) - 40);
        p263.file_url_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p263); //===============================
        FLIGHT_INFORMATION p264 = LoopBackDemoChannel.instance.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.time_boot_ms_SET(3956047074L);
        p264.arming_time_utc_SET(5049136493136030169L);
        p264.takeoff_time_utc_SET(7138758830389043169L);
        p264.flight_uuid_SET(3162981526684568263L);
        LoopBackDemoChannel.instance.send(p264); //===============================
        MOUNT_ORIENTATION p265 = LoopBackDemoChannel.instance.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.time_boot_ms_SET(3853702505L);
        p265.roll_SET(-6.989233E37F);
        p265.pitch_SET(3.2965315E38F);
        p265.yaw_SET(1.4465847E38F);
        LoopBackDemoChannel.instance.send(p265); //===============================
        LOGGING_DATA p266 = LoopBackDemoChannel.instance.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.target_system_SET((char)126);
        p266.target_component_SET((char)125);
        p266.sequence_SET((char)45680);
        p266.length_SET((char)142);
        p266.first_message_offset_SET((char)171);
        p266.data__SET(new char[249], 0);
        LoopBackDemoChannel.instance.send(p266); //===============================
        LOGGING_DATA_ACKED p267 = LoopBackDemoChannel.instance.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.target_system_SET((char)95);
        p267.target_component_SET((char)43);
        p267.sequence_SET((char)13876);
        p267.length_SET((char)43);
        p267.first_message_offset_SET((char)45);
        p267.data__SET(new char[249], 0);
        LoopBackDemoChannel.instance.send(p267); //===============================
        LOGGING_ACK p268 = LoopBackDemoChannel.instance.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.target_system_SET((char)109);
        p268.target_component_SET((char)218);
        p268.sequence_SET((char)26066);
        LoopBackDemoChannel.instance.send(p268); //===============================
        VIDEO_STREAM_INFORMATION p269 = LoopBackDemoChannel.instance.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.camera_id_SET((char)28);
        p269.status_SET((char)110);
        p269.framerate_SET(1.9786158E35F);
        p269.resolution_h_SET((char)15625);
        p269.resolution_v_SET((char)23273);
        p269.bitrate_SET(2378847852L);
        p269.rotation_SET((char)35899);
        p269.uri_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p269); //===============================
        SET_VIDEO_STREAM_SETTINGS p270 = LoopBackDemoChannel.instance.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.target_system_SET((char)165);
        p270.target_component_SET((char)86);
        p270.camera_id_SET((char)184);
        p270.framerate_SET(-1.8451717E38F);
        p270.resolution_h_SET((char)469);
        p270.resolution_v_SET((char)33472);
        p270.bitrate_SET(2746493805L);
        p270.rotation_SET((char)19403);
        p270.uri_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p270); //===============================
        WIFI_CONFIG_AP p299 = LoopBackDemoChannel.instance.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.ssid_SET("DEMO", PH);
        p299.password_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p299); //===============================
        PROTOCOL_VERSION p300 = LoopBackDemoChannel.instance.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.version_SET((char)43840);
        p300.min_version_SET((char)7415);
        p300.max_version_SET((char)48997);
        p300.spec_version_hash_SET(new char[8], 0);
        p300.library_version_hash_SET(new char[8], 0);
        LoopBackDemoChannel.instance.send(p300); //===============================
        UAVCAN_NODE_STATUS p310 = LoopBackDemoChannel.instance.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.time_usec_SET(6541651601779305539L);
        p310.uptime_sec_SET(1446659031L);
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL);
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE);
        p310.sub_mode_SET((char)19);
        p310.vendor_specific_status_code_SET((char)13360);
        LoopBackDemoChannel.instance.send(p310); //===============================
        UAVCAN_NODE_INFO p311 = LoopBackDemoChannel.instance.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.time_usec_SET(5565003787790492498L);
        p311.uptime_sec_SET(590877881L);
        p311.name_SET("DEMO", PH);
        p311.hw_version_major_SET((char)171);
        p311.hw_version_minor_SET((char)227);
        p311.hw_unique_id_SET(new char[16], 0);
        p311.sw_version_major_SET((char)201);
        p311.sw_version_minor_SET((char)81);
        p311.sw_vcs_commit_SET(3843360240L);
        LoopBackDemoChannel.instance.send(p311); //===============================
        PARAM_EXT_REQUEST_READ p320 = LoopBackDemoChannel.instance.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.target_system_SET((char)8);
        p320.target_component_SET((char)68);
        p320.param_id_SET("DEMO", PH);
        p320.param_index_SET((short)30157);
        LoopBackDemoChannel.instance.send(p320); //===============================
        PARAM_EXT_REQUEST_LIST p321 = LoopBackDemoChannel.instance.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_system_SET((char)53);
        p321.target_component_SET((char)104);
        LoopBackDemoChannel.instance.send(p321); //===============================
        PARAM_EXT_VALUE p322 = LoopBackDemoChannel.instance.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_id_SET("DEMO", PH);
        p322.param_value_SET("DEMO", PH);
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64);
        p322.param_count_SET((char)59449);
        p322.param_index_SET((char)61411);
        LoopBackDemoChannel.instance.send(p322); //===============================
        PARAM_EXT_SET p323 = LoopBackDemoChannel.instance.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.target_system_SET((char)13);
        p323.target_component_SET((char)169);
        p323.param_id_SET("DEMO", PH);
        p323.param_value_SET("DEMO", PH);
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT8);
        LoopBackDemoChannel.instance.send(p323); //===============================
        PARAM_EXT_ACK p324 = LoopBackDemoChannel.instance.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_id_SET("DEMO", PH);
        p324.param_value_SET("DEMO", PH);
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16);
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_IN_PROGRESS);
        LoopBackDemoChannel.instance.send(p324); //===============================
        OBSTACLE_DISTANCE p330 = LoopBackDemoChannel.instance.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.time_usec_SET(2701487175925151574L);
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
        p330.distances_SET(new char[72], 0);
        p330.increment_SET((char)109);
        p330.min_distance_SET((char)11986);
        p330.max_distance_SET((char)23630);
        LoopBackDemoChannel.instance.send(p330); //===============================
    }
}

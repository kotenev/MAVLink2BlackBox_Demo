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
        LoopBackDemoChannel.instance.on_NAV_FILTER_BIAS.add((src, ph, pack) ->
        {
            long  usec = pack.usec_GET();
            float  accel_0 = pack.accel_0_GET();
            float  accel_1 = pack.accel_1_GET();
            float  accel_2 = pack.accel_2_GET();
            float  gyro_0 = pack.gyro_0_GET();
            float  gyro_1 = pack.gyro_1_GET();
            float  gyro_2 = pack.gyro_2_GET();
        });
        LoopBackDemoChannel.instance.on_RADIO_CALIBRATION.add((src, ph, pack) ->
        {
            char[]  aileron = pack.aileron_GET();
            char[]  elevator = pack.elevator_GET();
            char[]  rudder = pack.rudder_GET();
            char[]  gyro = pack.gyro_GET();
            char[]  pitch = pack.pitch_GET();
            char[]  throttle = pack.throttle_GET();
        });
        LoopBackDemoChannel.instance.on_UALBERTA_SYS_STATUS.add((src, ph, pack) ->
        {
            char  mode = pack.mode_GET();
            char  nav_mode = pack.nav_mode_GET();
            char  pilot = pack.pilot_GET();
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
        p0.type_SET(MAV_TYPE.MAV_TYPE_FLAPPING_WING);
        p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_UDB);
        p0.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);
        p0.custom_mode_SET(172923116L);
        p0.system_status_SET(MAV_STATE.MAV_STATE_ACTIVE);
        p0.mavlink_version_SET((char)7);
        LoopBackDemoChannel.instance.send(p0); //===============================
        SYS_STATUS p1 = LoopBackDemoChannel.instance.new_SYS_STATUS();
        PH.setPack(p1);
        p1.onboard_control_sensors_present_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL);
        p1.onboard_control_sensors_enabled_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE);
        p1.onboard_control_sensors_health_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION);
        p1.load_SET((char)57806);
        p1.voltage_battery_SET((char)16886);
        p1.current_battery_SET((short) -7603);
        p1.battery_remaining_SET((byte) - 127);
        p1.drop_rate_comm_SET((char)44402);
        p1.errors_comm_SET((char)29631);
        p1.errors_count1_SET((char)60801);
        p1.errors_count2_SET((char)39511);
        p1.errors_count3_SET((char)48073);
        p1.errors_count4_SET((char)2382);
        LoopBackDemoChannel.instance.send(p1); //===============================
        SYSTEM_TIME p2 = LoopBackDemoChannel.instance.new_SYSTEM_TIME();
        PH.setPack(p2);
        p2.time_unix_usec_SET(5443395731119145048L);
        p2.time_boot_ms_SET(1603435996L);
        LoopBackDemoChannel.instance.send(p2); //===============================
        POSITION_TARGET_LOCAL_NED p3 = LoopBackDemoChannel.instance.new_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.time_boot_ms_SET(1895463047L);
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
        p3.type_mask_SET((char)4839);
        p3.x_SET(3.4015534E38F);
        p3.y_SET(-2.7797426E38F);
        p3.z_SET(-2.3121766E38F);
        p3.vx_SET(1.2237649E38F);
        p3.vy_SET(1.7815845E37F);
        p3.vz_SET(-4.1272909E37F);
        p3.afx_SET(1.9306523E38F);
        p3.afy_SET(-2.9463795E37F);
        p3.afz_SET(2.0464523E38F);
        p3.yaw_SET(2.2657103E38F);
        p3.yaw_rate_SET(2.6957975E38F);
        LoopBackDemoChannel.instance.send(p3); //===============================
        PING p4 = LoopBackDemoChannel.instance.new_PING();
        PH.setPack(p4);
        p4.time_usec_SET(6298486419642490054L);
        p4.seq_SET(2394453376L);
        p4.target_system_SET((char)145);
        p4.target_component_SET((char)50);
        LoopBackDemoChannel.instance.send(p4); //===============================
        CHANGE_OPERATOR_CONTROL p5 = LoopBackDemoChannel.instance.new_CHANGE_OPERATOR_CONTROL();
        PH.setPack(p5);
        p5.target_system_SET((char)161);
        p5.control_request_SET((char)184);
        p5.version_SET((char)173);
        p5.passkey_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p5); //===============================
        CHANGE_OPERATOR_CONTROL_ACK p6 = LoopBackDemoChannel.instance.new_CHANGE_OPERATOR_CONTROL_ACK();
        PH.setPack(p6);
        p6.gcs_system_id_SET((char)147);
        p6.control_request_SET((char)31);
        p6.ack_SET((char)57);
        LoopBackDemoChannel.instance.send(p6); //===============================
        AUTH_KEY p7 = LoopBackDemoChannel.instance.new_AUTH_KEY();
        PH.setPack(p7);
        p7.key_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p7); //===============================
        SET_MODE p11 = LoopBackDemoChannel.instance.new_SET_MODE();
        PH.setPack(p11);
        p11.target_system_SET((char)248);
        p11.base_mode_SET(MAV_MODE.MAV_MODE_AUTO_ARMED);
        p11.custom_mode_SET(1099232960L);
        LoopBackDemoChannel.instance.send(p11); //===============================
        PARAM_REQUEST_READ p20 = LoopBackDemoChannel.instance.new_PARAM_REQUEST_READ();
        PH.setPack(p20);
        p20.target_system_SET((char)39);
        p20.target_component_SET((char)153);
        p20.param_id_SET("DEMO", PH);
        p20.param_index_SET((short) -14369);
        LoopBackDemoChannel.instance.send(p20); //===============================
        PARAM_REQUEST_LIST p21 = LoopBackDemoChannel.instance.new_PARAM_REQUEST_LIST();
        PH.setPack(p21);
        p21.target_system_SET((char)19);
        p21.target_component_SET((char)254);
        LoopBackDemoChannel.instance.send(p21); //===============================
        PARAM_VALUE p22 = LoopBackDemoChannel.instance.new_PARAM_VALUE();
        PH.setPack(p22);
        p22.param_id_SET("DEMO", PH);
        p22.param_value_SET(9.161663E37F);
        p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT32);
        p22.param_count_SET((char)38523);
        p22.param_index_SET((char)26021);
        LoopBackDemoChannel.instance.send(p22); //===============================
        PARAM_SET p23 = LoopBackDemoChannel.instance.new_PARAM_SET();
        PH.setPack(p23);
        p23.target_system_SET((char)219);
        p23.target_component_SET((char)182);
        p23.param_id_SET("DEMO", PH);
        p23.param_value_SET(1.332861E38F);
        p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT64);
        LoopBackDemoChannel.instance.send(p23); //===============================
        GPS_RAW_INT p24 = LoopBackDemoChannel.instance.new_GPS_RAW_INT();
        PH.setPack(p24);
        p24.time_usec_SET(935359094120766359L);
        p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC);
        p24.lat_SET(-1162576593);
        p24.lon_SET(2068586803);
        p24.alt_SET(-801456659);
        p24.eph_SET((char)62807);
        p24.epv_SET((char)11093);
        p24.vel_SET((char)5941);
        p24.cog_SET((char)5344);
        p24.satellites_visible_SET((char)41);
        p24.alt_ellipsoid_SET(-1040888161, PH);
        p24.h_acc_SET(258090162L, PH);
        p24.v_acc_SET(3297559275L, PH);
        p24.vel_acc_SET(1352073065L, PH);
        p24.hdg_acc_SET(2835436347L, PH);
        LoopBackDemoChannel.instance.send(p24); //===============================
        GPS_STATUS p25 = LoopBackDemoChannel.instance.new_GPS_STATUS();
        PH.setPack(p25);
        p25.satellites_visible_SET((char)41);
        p25.satellite_prn_SET(new char[20], 0);
        p25.satellite_used_SET(new char[20], 0);
        p25.satellite_elevation_SET(new char[20], 0);
        p25.satellite_azimuth_SET(new char[20], 0);
        p25.satellite_snr_SET(new char[20], 0);
        LoopBackDemoChannel.instance.send(p25); //===============================
        SCALED_IMU p26 = LoopBackDemoChannel.instance.new_SCALED_IMU();
        PH.setPack(p26);
        p26.time_boot_ms_SET(2225293391L);
        p26.xacc_SET((short)28545);
        p26.yacc_SET((short)10799);
        p26.zacc_SET((short)31219);
        p26.xgyro_SET((short)9931);
        p26.ygyro_SET((short) -17172);
        p26.zgyro_SET((short)12408);
        p26.xmag_SET((short) -21194);
        p26.ymag_SET((short) -28089);
        p26.zmag_SET((short)29900);
        LoopBackDemoChannel.instance.send(p26); //===============================
        RAW_IMU p27 = LoopBackDemoChannel.instance.new_RAW_IMU();
        PH.setPack(p27);
        p27.time_usec_SET(6111933908800942247L);
        p27.xacc_SET((short) -22884);
        p27.yacc_SET((short)22471);
        p27.zacc_SET((short)9840);
        p27.xgyro_SET((short) -11146);
        p27.ygyro_SET((short) -6191);
        p27.zgyro_SET((short) -26934);
        p27.xmag_SET((short)3606);
        p27.ymag_SET((short) -14781);
        p27.zmag_SET((short) -8641);
        LoopBackDemoChannel.instance.send(p27); //===============================
        RAW_PRESSURE p28 = LoopBackDemoChannel.instance.new_RAW_PRESSURE();
        PH.setPack(p28);
        p28.time_usec_SET(7710836602000701686L);
        p28.press_abs_SET((short)23569);
        p28.press_diff1_SET((short) -27630);
        p28.press_diff2_SET((short)26818);
        p28.temperature_SET((short) -10690);
        LoopBackDemoChannel.instance.send(p28); //===============================
        SCALED_PRESSURE p29 = LoopBackDemoChannel.instance.new_SCALED_PRESSURE();
        PH.setPack(p29);
        p29.time_boot_ms_SET(313970766L);
        p29.press_abs_SET(-2.6685879E38F);
        p29.press_diff_SET(-1.2597496E38F);
        p29.temperature_SET((short) -32182);
        LoopBackDemoChannel.instance.send(p29); //===============================
        ATTITUDE p30 = LoopBackDemoChannel.instance.new_ATTITUDE();
        PH.setPack(p30);
        p30.time_boot_ms_SET(2648940899L);
        p30.roll_SET(-2.2542937E38F);
        p30.pitch_SET(-3.2225171E38F);
        p30.yaw_SET(3.7011432E37F);
        p30.rollspeed_SET(-1.9431366E38F);
        p30.pitchspeed_SET(1.7849056E38F);
        p30.yawspeed_SET(-2.3654823E38F);
        LoopBackDemoChannel.instance.send(p30); //===============================
        ATTITUDE_QUATERNION p31 = LoopBackDemoChannel.instance.new_ATTITUDE_QUATERNION();
        PH.setPack(p31);
        p31.time_boot_ms_SET(3390564553L);
        p31.q1_SET(1.8607405E38F);
        p31.q2_SET(1.869508E38F);
        p31.q3_SET(1.6735728E38F);
        p31.q4_SET(-1.5954567E38F);
        p31.rollspeed_SET(-3.0030107E38F);
        p31.pitchspeed_SET(2.1378169E38F);
        p31.yawspeed_SET(-3.1526844E38F);
        LoopBackDemoChannel.instance.send(p31); //===============================
        LOCAL_POSITION_NED p32 = LoopBackDemoChannel.instance.new_LOCAL_POSITION_NED();
        PH.setPack(p32);
        p32.time_boot_ms_SET(1033882977L);
        p32.x_SET(7.979891E37F);
        p32.y_SET(1.1118614E38F);
        p32.z_SET(2.2194138E38F);
        p32.vx_SET(-2.5357894E38F);
        p32.vy_SET(-5.8726804E37F);
        p32.vz_SET(5.862581E37F);
        LoopBackDemoChannel.instance.send(p32); //===============================
        GLOBAL_POSITION_INT p33 = LoopBackDemoChannel.instance.new_GLOBAL_POSITION_INT();
        PH.setPack(p33);
        p33.time_boot_ms_SET(3816871691L);
        p33.lat_SET(-669900387);
        p33.lon_SET(-753638317);
        p33.alt_SET(1931360217);
        p33.relative_alt_SET(120457624);
        p33.vx_SET((short)30176);
        p33.vy_SET((short)8252);
        p33.vz_SET((short)22993);
        p33.hdg_SET((char)57100);
        LoopBackDemoChannel.instance.send(p33); //===============================
        RC_CHANNELS_SCALED p34 = LoopBackDemoChannel.instance.new_RC_CHANNELS_SCALED();
        PH.setPack(p34);
        p34.time_boot_ms_SET(691961226L);
        p34.port_SET((char)187);
        p34.chan1_scaled_SET((short)19430);
        p34.chan2_scaled_SET((short)32677);
        p34.chan3_scaled_SET((short)20063);
        p34.chan4_scaled_SET((short) -10455);
        p34.chan5_scaled_SET((short)21661);
        p34.chan6_scaled_SET((short)1872);
        p34.chan7_scaled_SET((short)32644);
        p34.chan8_scaled_SET((short)24622);
        p34.rssi_SET((char)79);
        LoopBackDemoChannel.instance.send(p34); //===============================
        RC_CHANNELS_RAW p35 = LoopBackDemoChannel.instance.new_RC_CHANNELS_RAW();
        PH.setPack(p35);
        p35.time_boot_ms_SET(1531108670L);
        p35.port_SET((char)82);
        p35.chan1_raw_SET((char)58514);
        p35.chan2_raw_SET((char)34251);
        p35.chan3_raw_SET((char)8050);
        p35.chan4_raw_SET((char)6932);
        p35.chan5_raw_SET((char)32089);
        p35.chan6_raw_SET((char)1026);
        p35.chan7_raw_SET((char)11471);
        p35.chan8_raw_SET((char)16718);
        p35.rssi_SET((char)244);
        LoopBackDemoChannel.instance.send(p35); //===============================
        SERVO_OUTPUT_RAW p36 = LoopBackDemoChannel.instance.new_SERVO_OUTPUT_RAW();
        PH.setPack(p36);
        p36.time_usec_SET(289724769L);
        p36.port_SET((char)246);
        p36.servo1_raw_SET((char)64818);
        p36.servo2_raw_SET((char)15685);
        p36.servo3_raw_SET((char)56162);
        p36.servo4_raw_SET((char)60278);
        p36.servo5_raw_SET((char)30788);
        p36.servo6_raw_SET((char)53881);
        p36.servo7_raw_SET((char)4064);
        p36.servo8_raw_SET((char)61452);
        p36.servo9_raw_SET((char)29347, PH);
        p36.servo10_raw_SET((char)65359, PH);
        p36.servo11_raw_SET((char)59173, PH);
        p36.servo12_raw_SET((char)17989, PH);
        p36.servo13_raw_SET((char)61939, PH);
        p36.servo14_raw_SET((char)25582, PH);
        p36.servo15_raw_SET((char)56537, PH);
        p36.servo16_raw_SET((char)31088, PH);
        LoopBackDemoChannel.instance.send(p36); //===============================
        MISSION_REQUEST_PARTIAL_LIST p37 = LoopBackDemoChannel.instance.new_MISSION_REQUEST_PARTIAL_LIST();
        PH.setPack(p37);
        p37.target_system_SET((char)43);
        p37.target_component_SET((char)99);
        p37.start_index_SET((short)13838);
        p37.end_index_SET((short)11116);
        p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
        LoopBackDemoChannel.instance.send(p37); //===============================
        MISSION_WRITE_PARTIAL_LIST p38 = LoopBackDemoChannel.instance.new_MISSION_WRITE_PARTIAL_LIST();
        PH.setPack(p38);
        p38.target_system_SET((char)85);
        p38.target_component_SET((char)150);
        p38.start_index_SET((short)10661);
        p38.end_index_SET((short) -30603);
        p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
        LoopBackDemoChannel.instance.send(p38); //===============================
        MISSION_ITEM p39 = LoopBackDemoChannel.instance.new_MISSION_ITEM();
        PH.setPack(p39);
        p39.target_system_SET((char)154);
        p39.target_component_SET((char)197);
        p39.seq_SET((char)44064);
        p39.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
        p39.command_SET(MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS);
        p39.current_SET((char)47);
        p39.autocontinue_SET((char)193);
        p39.param1_SET(8.945487E37F);
        p39.param2_SET(-1.1446125E38F);
        p39.param3_SET(-3.1805414E38F);
        p39.param4_SET(1.0658945E38F);
        p39.x_SET(1.7221855E38F);
        p39.y_SET(1.6657687E38F);
        p39.z_SET(-1.3855155E38F);
        p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
        LoopBackDemoChannel.instance.send(p39); //===============================
        MISSION_REQUEST p40 = LoopBackDemoChannel.instance.new_MISSION_REQUEST();
        PH.setPack(p40);
        p40.target_system_SET((char)239);
        p40.target_component_SET((char)108);
        p40.seq_SET((char)38976);
        p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
        LoopBackDemoChannel.instance.send(p40); //===============================
        MISSION_SET_CURRENT p41 = LoopBackDemoChannel.instance.new_MISSION_SET_CURRENT();
        PH.setPack(p41);
        p41.target_system_SET((char)112);
        p41.target_component_SET((char)133);
        p41.seq_SET((char)40722);
        LoopBackDemoChannel.instance.send(p41); //===============================
        MISSION_CURRENT p42 = LoopBackDemoChannel.instance.new_MISSION_CURRENT();
        PH.setPack(p42);
        p42.seq_SET((char)34613);
        LoopBackDemoChannel.instance.send(p42); //===============================
        MISSION_REQUEST_LIST p43 = LoopBackDemoChannel.instance.new_MISSION_REQUEST_LIST();
        PH.setPack(p43);
        p43.target_system_SET((char)188);
        p43.target_component_SET((char)150);
        p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
        LoopBackDemoChannel.instance.send(p43); //===============================
        MISSION_COUNT p44 = LoopBackDemoChannel.instance.new_MISSION_COUNT();
        PH.setPack(p44);
        p44.target_system_SET((char)194);
        p44.target_component_SET((char)147);
        p44.count_SET((char)35053);
        p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
        LoopBackDemoChannel.instance.send(p44); //===============================
        MISSION_CLEAR_ALL p45 = LoopBackDemoChannel.instance.new_MISSION_CLEAR_ALL();
        PH.setPack(p45);
        p45.target_system_SET((char)156);
        p45.target_component_SET((char)113);
        p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
        LoopBackDemoChannel.instance.send(p45); //===============================
        MISSION_ITEM_REACHED p46 = LoopBackDemoChannel.instance.new_MISSION_ITEM_REACHED();
        PH.setPack(p46);
        p46.seq_SET((char)51222);
        LoopBackDemoChannel.instance.send(p46); //===============================
        MISSION_ACK p47 = LoopBackDemoChannel.instance.new_MISSION_ACK();
        PH.setPack(p47);
        p47.target_system_SET((char)83);
        p47.target_component_SET((char)120);
        p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM1);
        p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
        LoopBackDemoChannel.instance.send(p47); //===============================
        SET_GPS_GLOBAL_ORIGIN p48 = LoopBackDemoChannel.instance.new_SET_GPS_GLOBAL_ORIGIN();
        PH.setPack(p48);
        p48.target_system_SET((char)242);
        p48.latitude_SET(513290864);
        p48.longitude_SET(941810576);
        p48.altitude_SET(207793401);
        p48.time_usec_SET(5478105643869980384L, PH);
        LoopBackDemoChannel.instance.send(p48); //===============================
        GPS_GLOBAL_ORIGIN p49 = LoopBackDemoChannel.instance.new_GPS_GLOBAL_ORIGIN();
        PH.setPack(p49);
        p49.latitude_SET(-10641921);
        p49.longitude_SET(1546114079);
        p49.altitude_SET(-784005357);
        p49.time_usec_SET(7025535099499833262L, PH);
        LoopBackDemoChannel.instance.send(p49); //===============================
        PARAM_MAP_RC p50 = LoopBackDemoChannel.instance.new_PARAM_MAP_RC();
        PH.setPack(p50);
        p50.target_system_SET((char)209);
        p50.target_component_SET((char)56);
        p50.param_id_SET("DEMO", PH);
        p50.param_index_SET((short) -32540);
        p50.parameter_rc_channel_index_SET((char)156);
        p50.param_value0_SET(-2.1229423E37F);
        p50.scale_SET(-1.6983069E38F);
        p50.param_value_min_SET(-4.300831E37F);
        p50.param_value_max_SET(1.2363802E38F);
        LoopBackDemoChannel.instance.send(p50); //===============================
        MISSION_REQUEST_INT p51 = LoopBackDemoChannel.instance.new_MISSION_REQUEST_INT();
        PH.setPack(p51);
        p51.target_system_SET((char)169);
        p51.target_component_SET((char)11);
        p51.seq_SET((char)45935);
        p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
        LoopBackDemoChannel.instance.send(p51); //===============================
        SAFETY_SET_ALLOWED_AREA p54 = LoopBackDemoChannel.instance.new_SAFETY_SET_ALLOWED_AREA();
        PH.setPack(p54);
        p54.target_system_SET((char)55);
        p54.target_component_SET((char)122);
        p54.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
        p54.p1x_SET(-1.0221438E38F);
        p54.p1y_SET(2.0227028E38F);
        p54.p1z_SET(-2.925543E38F);
        p54.p2x_SET(-2.4359454E38F);
        p54.p2y_SET(4.775132E37F);
        p54.p2z_SET(3.1261538E38F);
        LoopBackDemoChannel.instance.send(p54); //===============================
        SAFETY_ALLOWED_AREA p55 = LoopBackDemoChannel.instance.new_SAFETY_ALLOWED_AREA();
        PH.setPack(p55);
        p55.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
        p55.p1x_SET(-2.2353885E38F);
        p55.p1y_SET(4.2039444E37F);
        p55.p1z_SET(1.6178179E38F);
        p55.p2x_SET(-2.9766132E38F);
        p55.p2y_SET(-2.2631249E38F);
        p55.p2z_SET(-4.1793345E37F);
        LoopBackDemoChannel.instance.send(p55); //===============================
        ATTITUDE_QUATERNION_COV p61 = LoopBackDemoChannel.instance.new_ATTITUDE_QUATERNION_COV();
        PH.setPack(p61);
        p61.time_usec_SET(2980709498162796382L);
        p61.q_SET(new float[4], 0);
        p61.rollspeed_SET(8.581676E37F);
        p61.pitchspeed_SET(-5.8369915E37F);
        p61.yawspeed_SET(-1.2508323E38F);
        p61.covariance_SET(new float[9], 0);
        LoopBackDemoChannel.instance.send(p61); //===============================
        NAV_CONTROLLER_OUTPUT p62 = LoopBackDemoChannel.instance.new_NAV_CONTROLLER_OUTPUT();
        PH.setPack(p62);
        p62.nav_roll_SET(2.8985082E38F);
        p62.nav_pitch_SET(1.844621E35F);
        p62.nav_bearing_SET((short)25037);
        p62.target_bearing_SET((short)574);
        p62.wp_dist_SET((char)39506);
        p62.alt_error_SET(-9.955853E37F);
        p62.aspd_error_SET(-1.287927E38F);
        p62.xtrack_error_SET(2.302464E38F);
        LoopBackDemoChannel.instance.send(p62); //===============================
        GLOBAL_POSITION_INT_COV p63 = LoopBackDemoChannel.instance.new_GLOBAL_POSITION_INT_COV();
        PH.setPack(p63);
        p63.time_usec_SET(7331180567978224858L);
        p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS);
        p63.lat_SET(1984764947);
        p63.lon_SET(1684945096);
        p63.alt_SET(770709198);
        p63.relative_alt_SET(1251210);
        p63.vx_SET(-2.8472589E38F);
        p63.vy_SET(2.0794212E38F);
        p63.vz_SET(2.0461777E38F);
        p63.covariance_SET(new float[36], 0);
        LoopBackDemoChannel.instance.send(p63); //===============================
        LOCAL_POSITION_NED_COV p64 = LoopBackDemoChannel.instance.new_LOCAL_POSITION_NED_COV();
        PH.setPack(p64);
        p64.time_usec_SET(1715863395637580690L);
        p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS);
        p64.x_SET(-2.6035377E38F);
        p64.y_SET(3.327287E38F);
        p64.z_SET(-2.0738514E38F);
        p64.vx_SET(3.8195819E37F);
        p64.vy_SET(-2.8263525E37F);
        p64.vz_SET(-1.9219881E38F);
        p64.ax_SET(1.359446E38F);
        p64.ay_SET(-7.6914827E37F);
        p64.az_SET(2.7555087E38F);
        p64.covariance_SET(new float[45], 0);
        LoopBackDemoChannel.instance.send(p64); //===============================
        RC_CHANNELS p65 = LoopBackDemoChannel.instance.new_RC_CHANNELS();
        PH.setPack(p65);
        p65.time_boot_ms_SET(2760723293L);
        p65.chancount_SET((char)214);
        p65.chan1_raw_SET((char)28713);
        p65.chan2_raw_SET((char)6057);
        p65.chan3_raw_SET((char)19772);
        p65.chan4_raw_SET((char)13731);
        p65.chan5_raw_SET((char)38999);
        p65.chan6_raw_SET((char)12343);
        p65.chan7_raw_SET((char)26684);
        p65.chan8_raw_SET((char)8096);
        p65.chan9_raw_SET((char)56922);
        p65.chan10_raw_SET((char)65472);
        p65.chan11_raw_SET((char)32794);
        p65.chan12_raw_SET((char)1416);
        p65.chan13_raw_SET((char)59797);
        p65.chan14_raw_SET((char)24699);
        p65.chan15_raw_SET((char)39096);
        p65.chan16_raw_SET((char)46672);
        p65.chan17_raw_SET((char)54629);
        p65.chan18_raw_SET((char)32703);
        p65.rssi_SET((char)57);
        LoopBackDemoChannel.instance.send(p65); //===============================
        REQUEST_DATA_STREAM p66 = LoopBackDemoChannel.instance.new_REQUEST_DATA_STREAM();
        PH.setPack(p66);
        p66.target_system_SET((char)201);
        p66.target_component_SET((char)211);
        p66.req_stream_id_SET((char)149);
        p66.req_message_rate_SET((char)28178);
        p66.start_stop_SET((char)16);
        LoopBackDemoChannel.instance.send(p66); //===============================
        DATA_STREAM p67 = LoopBackDemoChannel.instance.new_DATA_STREAM();
        PH.setPack(p67);
        p67.stream_id_SET((char)56);
        p67.message_rate_SET((char)47427);
        p67.on_off_SET((char)52);
        LoopBackDemoChannel.instance.send(p67); //===============================
        MANUAL_CONTROL p69 = LoopBackDemoChannel.instance.new_MANUAL_CONTROL();
        PH.setPack(p69);
        p69.target_SET((char)135);
        p69.x_SET((short)966);
        p69.y_SET((short)30693);
        p69.z_SET((short) -21503);
        p69.r_SET((short) -29513);
        p69.buttons_SET((char)35114);
        LoopBackDemoChannel.instance.send(p69); //===============================
        RC_CHANNELS_OVERRIDE p70 = LoopBackDemoChannel.instance.new_RC_CHANNELS_OVERRIDE();
        PH.setPack(p70);
        p70.target_system_SET((char)213);
        p70.target_component_SET((char)157);
        p70.chan1_raw_SET((char)62365);
        p70.chan2_raw_SET((char)59228);
        p70.chan3_raw_SET((char)28536);
        p70.chan4_raw_SET((char)55101);
        p70.chan5_raw_SET((char)63033);
        p70.chan6_raw_SET((char)53509);
        p70.chan7_raw_SET((char)27866);
        p70.chan8_raw_SET((char)59419);
        LoopBackDemoChannel.instance.send(p70); //===============================
        MISSION_ITEM_INT p73 = LoopBackDemoChannel.instance.new_MISSION_ITEM_INT();
        PH.setPack(p73);
        p73.target_system_SET((char)88);
        p73.target_component_SET((char)248);
        p73.seq_SET((char)61258);
        p73.frame_SET(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
        p73.command_SET(MAV_CMD.MAV_CMD_DO_RALLY_LAND);
        p73.current_SET((char)62);
        p73.autocontinue_SET((char)112);
        p73.param1_SET(6.899734E37F);
        p73.param2_SET(-2.550474E38F);
        p73.param3_SET(2.8755583E38F);
        p73.param4_SET(1.3238091E38F);
        p73.x_SET(-839228633);
        p73.y_SET(-1285508410);
        p73.z_SET(2.171861E38F);
        p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
        LoopBackDemoChannel.instance.send(p73); //===============================
        VFR_HUD p74 = LoopBackDemoChannel.instance.new_VFR_HUD();
        PH.setPack(p74);
        p74.airspeed_SET(2.5074958E38F);
        p74.groundspeed_SET(9.687989E37F);
        p74.heading_SET((short)23033);
        p74.throttle_SET((char)55317);
        p74.alt_SET(-3.0238E38F);
        p74.climb_SET(2.2537672E38F);
        LoopBackDemoChannel.instance.send(p74); //===============================
        COMMAND_INT p75 = LoopBackDemoChannel.instance.new_COMMAND_INT();
        PH.setPack(p75);
        p75.target_system_SET((char)213);
        p75.target_component_SET((char)248);
        p75.frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED);
        p75.command_SET(MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION);
        p75.current_SET((char)57);
        p75.autocontinue_SET((char)229);
        p75.param1_SET(8.1867893E37F);
        p75.param2_SET(1.4434028E38F);
        p75.param3_SET(9.541467E37F);
        p75.param4_SET(3.1704238E38F);
        p75.x_SET(519922316);
        p75.y_SET(1545977952);
        p75.z_SET(-1.1972589E38F);
        LoopBackDemoChannel.instance.send(p75); //===============================
        COMMAND_LONG p76 = LoopBackDemoChannel.instance.new_COMMAND_LONG();
        PH.setPack(p76);
        p76.target_system_SET((char)66);
        p76.target_component_SET((char)101);
        p76.command_SET(MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE);
        p76.confirmation_SET((char)17);
        p76.param1_SET(-5.0502886E37F);
        p76.param2_SET(1.8285077E38F);
        p76.param3_SET(1.2464838E38F);
        p76.param4_SET(-1.7091999E38F);
        p76.param5_SET(-2.774232E38F);
        p76.param6_SET(1.0604539E38F);
        p76.param7_SET(2.5216822E38F);
        LoopBackDemoChannel.instance.send(p76); //===============================
        COMMAND_ACK p77 = LoopBackDemoChannel.instance.new_COMMAND_ACK();
        PH.setPack(p77);
        p77.command_SET(MAV_CMD.MAV_CMD_USER_5);
        p77.result_SET(MAV_RESULT.MAV_RESULT_IN_PROGRESS);
        p77.progress_SET((char)228, PH);
        p77.result_param2_SET(251495140, PH);
        p77.target_system_SET((char)46, PH);
        p77.target_component_SET((char)156, PH);
        LoopBackDemoChannel.instance.send(p77); //===============================
        MANUAL_SETPOINT p81 = LoopBackDemoChannel.instance.new_MANUAL_SETPOINT();
        PH.setPack(p81);
        p81.time_boot_ms_SET(1156749615L);
        p81.roll_SET(-3.0429036E38F);
        p81.pitch_SET(-2.9125891E38F);
        p81.yaw_SET(-1.4263557E38F);
        p81.thrust_SET(-3.306487E37F);
        p81.mode_switch_SET((char)47);
        p81.manual_override_switch_SET((char)87);
        LoopBackDemoChannel.instance.send(p81); //===============================
        SET_ATTITUDE_TARGET p82 = LoopBackDemoChannel.instance.new_SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.time_boot_ms_SET(2620764184L);
        p82.target_system_SET((char)124);
        p82.target_component_SET((char)239);
        p82.type_mask_SET((char)140);
        p82.q_SET(new float[4], 0);
        p82.body_roll_rate_SET(-3.0444801E38F);
        p82.body_pitch_rate_SET(-1.287545E38F);
        p82.body_yaw_rate_SET(1.5150618E38F);
        p82.thrust_SET(3.2429433E38F);
        LoopBackDemoChannel.instance.send(p82); //===============================
        ATTITUDE_TARGET p83 = LoopBackDemoChannel.instance.new_ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.time_boot_ms_SET(142723589L);
        p83.type_mask_SET((char)195);
        p83.q_SET(new float[4], 0);
        p83.body_roll_rate_SET(1.9435781E38F);
        p83.body_pitch_rate_SET(-2.2284959E38F);
        p83.body_yaw_rate_SET(-1.079248E38F);
        p83.thrust_SET(-2.2570428E38F);
        LoopBackDemoChannel.instance.send(p83); //===============================
        SET_POSITION_TARGET_LOCAL_NED p84 = LoopBackDemoChannel.instance.new_SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.time_boot_ms_SET(2972025494L);
        p84.target_system_SET((char)115);
        p84.target_component_SET((char)191);
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
        p84.type_mask_SET((char)7876);
        p84.x_SET(2.8908273E38F);
        p84.y_SET(3.3869582E38F);
        p84.z_SET(3.3204876E38F);
        p84.vx_SET(1.9920054E38F);
        p84.vy_SET(-2.4610875E38F);
        p84.vz_SET(-5.574622E37F);
        p84.afx_SET(-1.729283E38F);
        p84.afy_SET(2.1750503E38F);
        p84.afz_SET(9.971634E37F);
        p84.yaw_SET(1.4918041E38F);
        p84.yaw_rate_SET(2.8154842E38F);
        LoopBackDemoChannel.instance.send(p84); //===============================
        SET_POSITION_TARGET_GLOBAL_INT p86 = LoopBackDemoChannel.instance.new_SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.time_boot_ms_SET(3218545502L);
        p86.target_system_SET((char)126);
        p86.target_component_SET((char)219);
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED);
        p86.type_mask_SET((char)3925);
        p86.lat_int_SET(1660009354);
        p86.lon_int_SET(-2055579285);
        p86.alt_SET(2.0163054E38F);
        p86.vx_SET(4.695171E37F);
        p86.vy_SET(6.0299386E37F);
        p86.vz_SET(-3.2314385E38F);
        p86.afx_SET(-6.749028E37F);
        p86.afy_SET(1.7413008E38F);
        p86.afz_SET(2.9122553E38F);
        p86.yaw_SET(-2.450372E38F);
        p86.yaw_rate_SET(1.7439081E38F);
        LoopBackDemoChannel.instance.send(p86); //===============================
        POSITION_TARGET_GLOBAL_INT p87 = LoopBackDemoChannel.instance.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.time_boot_ms_SET(165049396L);
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_NED);
        p87.type_mask_SET((char)45538);
        p87.lat_int_SET(1189029576);
        p87.lon_int_SET(1804454756);
        p87.alt_SET(3.1410597E38F);
        p87.vx_SET(3.2553787E38F);
        p87.vy_SET(-7.5567233E37F);
        p87.vz_SET(2.736003E38F);
        p87.afx_SET(-2.1807726E36F);
        p87.afy_SET(-1.625414E38F);
        p87.afz_SET(1.9468573E37F);
        p87.yaw_SET(2.5130037E38F);
        p87.yaw_rate_SET(2.635431E38F);
        LoopBackDemoChannel.instance.send(p87); //===============================
        LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = LoopBackDemoChannel.instance.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.time_boot_ms_SET(1335575488L);
        p89.x_SET(-2.54725E38F);
        p89.y_SET(-1.3326267E38F);
        p89.z_SET(2.5108053E38F);
        p89.roll_SET(-2.8533714E38F);
        p89.pitch_SET(2.134642E38F);
        p89.yaw_SET(3.1030188E38F);
        LoopBackDemoChannel.instance.send(p89); //===============================
        HIL_STATE p90 = LoopBackDemoChannel.instance.new_HIL_STATE();
        PH.setPack(p90);
        p90.time_usec_SET(3954450207498329185L);
        p90.roll_SET(-1.2405001E38F);
        p90.pitch_SET(3.3323543E38F);
        p90.yaw_SET(-1.0671321E38F);
        p90.rollspeed_SET(2.796922E38F);
        p90.pitchspeed_SET(1.5221036E38F);
        p90.yawspeed_SET(-2.3149993E38F);
        p90.lat_SET(-1598211780);
        p90.lon_SET(-2050124364);
        p90.alt_SET(-169308580);
        p90.vx_SET((short)28713);
        p90.vy_SET((short)17303);
        p90.vz_SET((short) -30569);
        p90.xacc_SET((short)11367);
        p90.yacc_SET((short)13998);
        p90.zacc_SET((short)30533);
        LoopBackDemoChannel.instance.send(p90); //===============================
        HIL_CONTROLS p91 = LoopBackDemoChannel.instance.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.time_usec_SET(4611090030123911036L);
        p91.roll_ailerons_SET(4.7548555E37F);
        p91.pitch_elevator_SET(-1.3451191E38F);
        p91.yaw_rudder_SET(4.406789E37F);
        p91.throttle_SET(1.6047517E38F);
        p91.aux1_SET(-3.2884396E38F);
        p91.aux2_SET(1.1171855E38F);
        p91.aux3_SET(1.453764E38F);
        p91.aux4_SET(-1.4965484E38F);
        p91.mode_SET(MAV_MODE.MAV_MODE_STABILIZE_DISARMED);
        p91.nav_mode_SET((char)164);
        LoopBackDemoChannel.instance.send(p91); //===============================
        HIL_RC_INPUTS_RAW p92 = LoopBackDemoChannel.instance.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.time_usec_SET(34593058435089034L);
        p92.chan1_raw_SET((char)37071);
        p92.chan2_raw_SET((char)53420);
        p92.chan3_raw_SET((char)37650);
        p92.chan4_raw_SET((char)52686);
        p92.chan5_raw_SET((char)52736);
        p92.chan6_raw_SET((char)8151);
        p92.chan7_raw_SET((char)43186);
        p92.chan8_raw_SET((char)41751);
        p92.chan9_raw_SET((char)37037);
        p92.chan10_raw_SET((char)25336);
        p92.chan11_raw_SET((char)14782);
        p92.chan12_raw_SET((char)7302);
        p92.rssi_SET((char)213);
        LoopBackDemoChannel.instance.send(p92); //===============================
        HIL_ACTUATOR_CONTROLS p93 = LoopBackDemoChannel.instance.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.time_usec_SET(2923990178629567795L);
        p93.controls_SET(new float[16], 0);
        p93.mode_SET(MAV_MODE.MAV_MODE_GUIDED_DISARMED);
        p93.flags_SET(2960602740581948821L);
        LoopBackDemoChannel.instance.send(p93); //===============================
        OPTICAL_FLOW p100 = LoopBackDemoChannel.instance.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.time_usec_SET(5580205009206149166L);
        p100.sensor_id_SET((char)219);
        p100.flow_x_SET((short)8260);
        p100.flow_y_SET((short)9904);
        p100.flow_comp_m_x_SET(1.3461512E38F);
        p100.flow_comp_m_y_SET(1.0000555E38F);
        p100.quality_SET((char)89);
        p100.ground_distance_SET(-7.604061E37F);
        p100.flow_rate_x_SET(-1.96825E38F, PH);
        p100.flow_rate_y_SET(9.683782E37F, PH);
        LoopBackDemoChannel.instance.send(p100); //===============================
        GLOBAL_VISION_POSITION_ESTIMATE p101 = LoopBackDemoChannel.instance.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.usec_SET(8984748825478128834L);
        p101.x_SET(-1.7462112E38F);
        p101.y_SET(9.86684E37F);
        p101.z_SET(-1.9707428E38F);
        p101.roll_SET(-1.701755E38F);
        p101.pitch_SET(-2.0263908E38F);
        p101.yaw_SET(-9.943872E37F);
        LoopBackDemoChannel.instance.send(p101); //===============================
        VISION_POSITION_ESTIMATE p102 = LoopBackDemoChannel.instance.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.usec_SET(2749813883704867806L);
        p102.x_SET(7.8864403E37F);
        p102.y_SET(-2.0995334E38F);
        p102.z_SET(1.9187778E38F);
        p102.roll_SET(1.972926E38F);
        p102.pitch_SET(-2.5278738E38F);
        p102.yaw_SET(-1.7685977E38F);
        LoopBackDemoChannel.instance.send(p102); //===============================
        VISION_SPEED_ESTIMATE p103 = LoopBackDemoChannel.instance.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.usec_SET(7810448096204831261L);
        p103.x_SET(1.1748547E38F);
        p103.y_SET(-3.3337512E37F);
        p103.z_SET(-9.797563E37F);
        LoopBackDemoChannel.instance.send(p103); //===============================
        VICON_POSITION_ESTIMATE p104 = LoopBackDemoChannel.instance.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.usec_SET(470482067356230521L);
        p104.x_SET(-2.7294897E38F);
        p104.y_SET(3.4770023E37F);
        p104.z_SET(-1.623822E38F);
        p104.roll_SET(-2.0364929E38F);
        p104.pitch_SET(1.7692214E38F);
        p104.yaw_SET(-1.6607055E38F);
        LoopBackDemoChannel.instance.send(p104); //===============================
        HIGHRES_IMU p105 = LoopBackDemoChannel.instance.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.time_usec_SET(5039343717872331595L);
        p105.xacc_SET(-1.1287728E38F);
        p105.yacc_SET(-2.396165E38F);
        p105.zacc_SET(-1.5022322E38F);
        p105.xgyro_SET(2.480688E38F);
        p105.ygyro_SET(-2.744835E38F);
        p105.zgyro_SET(1.8315754E38F);
        p105.xmag_SET(-6.4137066E37F);
        p105.ymag_SET(3.476646E37F);
        p105.zmag_SET(-2.4729652E37F);
        p105.abs_pressure_SET(3.3947983E38F);
        p105.diff_pressure_SET(1.9915245E38F);
        p105.pressure_alt_SET(2.601932E38F);
        p105.temperature_SET(-2.2580536E38F);
        p105.fields_updated_SET((char)2061);
        LoopBackDemoChannel.instance.send(p105); //===============================
        OPTICAL_FLOW_RAD p106 = LoopBackDemoChannel.instance.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.time_usec_SET(7070658137128081300L);
        p106.sensor_id_SET((char)4);
        p106.integration_time_us_SET(3531063333L);
        p106.integrated_x_SET(2.9980732E38F);
        p106.integrated_y_SET(1.8682401E38F);
        p106.integrated_xgyro_SET(1.7823433E38F);
        p106.integrated_ygyro_SET(1.6862138E38F);
        p106.integrated_zgyro_SET(3.2909088E38F);
        p106.temperature_SET((short)24807);
        p106.quality_SET((char)190);
        p106.time_delta_distance_us_SET(4230483826L);
        p106.distance_SET(-5.824151E37F);
        LoopBackDemoChannel.instance.send(p106); //===============================
        HIL_SENSOR p107 = LoopBackDemoChannel.instance.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.time_usec_SET(4977386933484804958L);
        p107.xacc_SET(1.1677827E38F);
        p107.yacc_SET(-2.6329288E38F);
        p107.zacc_SET(-1.8505328E38F);
        p107.xgyro_SET(-1.8965838E37F);
        p107.ygyro_SET(2.5165045E38F);
        p107.zgyro_SET(-7.846203E37F);
        p107.xmag_SET(2.238046E38F);
        p107.ymag_SET(2.0012737E38F);
        p107.zmag_SET(-3.2919124E38F);
        p107.abs_pressure_SET(-1.6526722E38F);
        p107.diff_pressure_SET(-3.5571688E37F);
        p107.pressure_alt_SET(6.606678E37F);
        p107.temperature_SET(1.1756246E38F);
        p107.fields_updated_SET(1856020483L);
        LoopBackDemoChannel.instance.send(p107); //===============================
        SIM_STATE p108 = LoopBackDemoChannel.instance.new_SIM_STATE();
        PH.setPack(p108);
        p108.q1_SET(-1.927562E38F);
        p108.q2_SET(7.0129316E37F);
        p108.q3_SET(-7.988197E36F);
        p108.q4_SET(-4.6891613E37F);
        p108.roll_SET(1.6196349E38F);
        p108.pitch_SET(3.0804054E38F);
        p108.yaw_SET(1.1481862E38F);
        p108.xacc_SET(-8.324811E36F);
        p108.yacc_SET(2.3770996E38F);
        p108.zacc_SET(1.23122E38F);
        p108.xgyro_SET(-5.398512E37F);
        p108.ygyro_SET(-1.1022598E38F);
        p108.zgyro_SET(9.571273E36F);
        p108.lat_SET(-2.1905376E38F);
        p108.lon_SET(-1.8306952E37F);
        p108.alt_SET(-2.192945E38F);
        p108.std_dev_horz_SET(2.2620193E38F);
        p108.std_dev_vert_SET(-3.0170338E38F);
        p108.vn_SET(2.9427263E38F);
        p108.ve_SET(1.4374032E38F);
        p108.vd_SET(3.1763614E38F);
        LoopBackDemoChannel.instance.send(p108); //===============================
        RADIO_STATUS p109 = LoopBackDemoChannel.instance.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.rssi_SET((char)50);
        p109.remrssi_SET((char)4);
        p109.txbuf_SET((char)43);
        p109.noise_SET((char)131);
        p109.remnoise_SET((char)152);
        p109.rxerrors_SET((char)47477);
        p109.fixed__SET((char)46225);
        LoopBackDemoChannel.instance.send(p109); //===============================
        FILE_TRANSFER_PROTOCOL p110 = LoopBackDemoChannel.instance.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.target_network_SET((char)111);
        p110.target_system_SET((char)242);
        p110.target_component_SET((char)139);
        p110.payload_SET(new char[251], 0);
        LoopBackDemoChannel.instance.send(p110); //===============================
        TIMESYNC p111 = LoopBackDemoChannel.instance.new_TIMESYNC();
        PH.setPack(p111);
        p111.tc1_SET(2505079145288163357L);
        p111.ts1_SET(6716098076019483224L);
        LoopBackDemoChannel.instance.send(p111); //===============================
        CAMERA_TRIGGER p112 = LoopBackDemoChannel.instance.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.time_usec_SET(8264531125741671001L);
        p112.seq_SET(102212379L);
        LoopBackDemoChannel.instance.send(p112); //===============================
        HIL_GPS p113 = LoopBackDemoChannel.instance.new_HIL_GPS();
        PH.setPack(p113);
        p113.time_usec_SET(5215370593622365394L);
        p113.fix_type_SET((char)19);
        p113.lat_SET(-36039241);
        p113.lon_SET(-407749109);
        p113.alt_SET(-1713124406);
        p113.eph_SET((char)55288);
        p113.epv_SET((char)8629);
        p113.vel_SET((char)59854);
        p113.vn_SET((short)3767);
        p113.ve_SET((short) -27936);
        p113.vd_SET((short) -22945);
        p113.cog_SET((char)14778);
        p113.satellites_visible_SET((char)126);
        LoopBackDemoChannel.instance.send(p113); //===============================
        HIL_OPTICAL_FLOW p114 = LoopBackDemoChannel.instance.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.time_usec_SET(5125339672581547994L);
        p114.sensor_id_SET((char)230);
        p114.integration_time_us_SET(1266189872L);
        p114.integrated_x_SET(-2.9424663E38F);
        p114.integrated_y_SET(5.9786896E36F);
        p114.integrated_xgyro_SET(-4.0811491E37F);
        p114.integrated_ygyro_SET(-1.695536E38F);
        p114.integrated_zgyro_SET(-3.2300163E38F);
        p114.temperature_SET((short)26321);
        p114.quality_SET((char)34);
        p114.time_delta_distance_us_SET(3545031630L);
        p114.distance_SET(-1.4842957E38F);
        LoopBackDemoChannel.instance.send(p114); //===============================
        HIL_STATE_QUATERNION p115 = LoopBackDemoChannel.instance.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.time_usec_SET(375371400321037935L);
        p115.attitude_quaternion_SET(new float[4], 0);
        p115.rollspeed_SET(-1.6132476E38F);
        p115.pitchspeed_SET(3.103728E38F);
        p115.yawspeed_SET(-2.392217E38F);
        p115.lat_SET(-1544943550);
        p115.lon_SET(-1383180490);
        p115.alt_SET(-1570194354);
        p115.vx_SET((short)23205);
        p115.vy_SET((short)9922);
        p115.vz_SET((short) -1641);
        p115.ind_airspeed_SET((char)13320);
        p115.true_airspeed_SET((char)2698);
        p115.xacc_SET((short)31433);
        p115.yacc_SET((short)26209);
        p115.zacc_SET((short)31416);
        LoopBackDemoChannel.instance.send(p115); //===============================
        SCALED_IMU2 p116 = LoopBackDemoChannel.instance.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.time_boot_ms_SET(402497554L);
        p116.xacc_SET((short)24419);
        p116.yacc_SET((short) -9365);
        p116.zacc_SET((short)7285);
        p116.xgyro_SET((short) -18750);
        p116.ygyro_SET((short)12520);
        p116.zgyro_SET((short)25792);
        p116.xmag_SET((short)21791);
        p116.ymag_SET((short)30541);
        p116.zmag_SET((short)15997);
        LoopBackDemoChannel.instance.send(p116); //===============================
        LOG_REQUEST_LIST p117 = LoopBackDemoChannel.instance.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.target_system_SET((char)165);
        p117.target_component_SET((char)207);
        p117.start_SET((char)60977);
        p117.end_SET((char)10193);
        LoopBackDemoChannel.instance.send(p117); //===============================
        LOG_ENTRY p118 = LoopBackDemoChannel.instance.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.id_SET((char)13167);
        p118.num_logs_SET((char)41318);
        p118.last_log_num_SET((char)6649);
        p118.time_utc_SET(3996810561L);
        p118.size_SET(625801346L);
        LoopBackDemoChannel.instance.send(p118); //===============================
        LOG_REQUEST_DATA p119 = LoopBackDemoChannel.instance.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.target_system_SET((char)136);
        p119.target_component_SET((char)217);
        p119.id_SET((char)19608);
        p119.ofs_SET(4004804424L);
        p119.count_SET(3304472166L);
        LoopBackDemoChannel.instance.send(p119); //===============================
        LOG_DATA p120 = LoopBackDemoChannel.instance.new_LOG_DATA();
        PH.setPack(p120);
        p120.id_SET((char)20984);
        p120.ofs_SET(920353362L);
        p120.count_SET((char)162);
        p120.data__SET(new char[90], 0);
        LoopBackDemoChannel.instance.send(p120); //===============================
        LOG_ERASE p121 = LoopBackDemoChannel.instance.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_system_SET((char)27);
        p121.target_component_SET((char)170);
        LoopBackDemoChannel.instance.send(p121); //===============================
        LOG_REQUEST_END p122 = LoopBackDemoChannel.instance.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_system_SET((char)41);
        p122.target_component_SET((char)187);
        LoopBackDemoChannel.instance.send(p122); //===============================
        GPS_INJECT_DATA p123 = LoopBackDemoChannel.instance.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.target_system_SET((char)61);
        p123.target_component_SET((char)52);
        p123.len_SET((char)98);
        p123.data__SET(new char[110], 0);
        LoopBackDemoChannel.instance.send(p123); //===============================
        GPS2_RAW p124 = LoopBackDemoChannel.instance.new_GPS2_RAW();
        PH.setPack(p124);
        p124.time_usec_SET(5174106780700440282L);
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS);
        p124.lat_SET(-1131919111);
        p124.lon_SET(1469588961);
        p124.alt_SET(-123804835);
        p124.eph_SET((char)54722);
        p124.epv_SET((char)34565);
        p124.vel_SET((char)26032);
        p124.cog_SET((char)6262);
        p124.satellites_visible_SET((char)135);
        p124.dgps_numch_SET((char)108);
        p124.dgps_age_SET(1040437505L);
        LoopBackDemoChannel.instance.send(p124); //===============================
        POWER_STATUS p125 = LoopBackDemoChannel.instance.new_POWER_STATUS();
        PH.setPack(p125);
        p125.Vcc_SET((char)2846);
        p125.Vservo_SET((char)5944);
        p125.flags_SET(MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT);
        LoopBackDemoChannel.instance.send(p125); //===============================
        SERIAL_CONTROL p126 = LoopBackDemoChannel.instance.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS2);
        p126.flags_SET(SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE);
        p126.timeout_SET((char)45635);
        p126.baudrate_SET(4086433774L);
        p126.count_SET((char)4);
        p126.data__SET(new char[70], 0);
        LoopBackDemoChannel.instance.send(p126); //===============================
        GPS_RTK p127 = LoopBackDemoChannel.instance.new_GPS_RTK();
        PH.setPack(p127);
        p127.time_last_baseline_ms_SET(2428506131L);
        p127.rtk_receiver_id_SET((char)120);
        p127.wn_SET((char)10615);
        p127.tow_SET(3878374992L);
        p127.rtk_health_SET((char)241);
        p127.rtk_rate_SET((char)0);
        p127.nsats_SET((char)124);
        p127.baseline_coords_type_SET((char)223);
        p127.baseline_a_mm_SET(1510942156);
        p127.baseline_b_mm_SET(151802938);
        p127.baseline_c_mm_SET(-976061184);
        p127.accuracy_SET(239511607L);
        p127.iar_num_hypotheses_SET(-775802491);
        LoopBackDemoChannel.instance.send(p127); //===============================
        GPS2_RTK p128 = LoopBackDemoChannel.instance.new_GPS2_RTK();
        PH.setPack(p128);
        p128.time_last_baseline_ms_SET(3611622743L);
        p128.rtk_receiver_id_SET((char)186);
        p128.wn_SET((char)8247);
        p128.tow_SET(2401000069L);
        p128.rtk_health_SET((char)175);
        p128.rtk_rate_SET((char)119);
        p128.nsats_SET((char)140);
        p128.baseline_coords_type_SET((char)134);
        p128.baseline_a_mm_SET(893842939);
        p128.baseline_b_mm_SET(1639806364);
        p128.baseline_c_mm_SET(1382756636);
        p128.accuracy_SET(2591314935L);
        p128.iar_num_hypotheses_SET(-1040232459);
        LoopBackDemoChannel.instance.send(p128); //===============================
        SCALED_IMU3 p129 = LoopBackDemoChannel.instance.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.time_boot_ms_SET(475858339L);
        p129.xacc_SET((short) -30988);
        p129.yacc_SET((short)5946);
        p129.zacc_SET((short)12591);
        p129.xgyro_SET((short)18399);
        p129.ygyro_SET((short)25749);
        p129.zgyro_SET((short)119);
        p129.xmag_SET((short) -13700);
        p129.ymag_SET((short) -28172);
        p129.zmag_SET((short) -16571);
        LoopBackDemoChannel.instance.send(p129); //===============================
        DATA_TRANSMISSION_HANDSHAKE p130 = LoopBackDemoChannel.instance.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.type_SET((char)214);
        p130.size_SET(201812684L);
        p130.width_SET((char)18932);
        p130.height_SET((char)35851);
        p130.packets_SET((char)61411);
        p130.payload_SET((char)243);
        p130.jpg_quality_SET((char)155);
        LoopBackDemoChannel.instance.send(p130); //===============================
        ENCAPSULATED_DATA p131 = LoopBackDemoChannel.instance.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.seqnr_SET((char)55446);
        p131.data__SET(new char[253], 0);
        LoopBackDemoChannel.instance.send(p131); //===============================
        DISTANCE_SENSOR p132 = LoopBackDemoChannel.instance.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.time_boot_ms_SET(3303072134L);
        p132.min_distance_SET((char)64516);
        p132.max_distance_SET((char)36335);
        p132.current_distance_SET((char)55743);
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND);
        p132.id_SET((char)76);
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_PITCH_90);
        p132.covariance_SET((char)47);
        LoopBackDemoChannel.instance.send(p132); //===============================
        TERRAIN_REQUEST p133 = LoopBackDemoChannel.instance.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.lat_SET(-1382566125);
        p133.lon_SET(-1297935260);
        p133.grid_spacing_SET((char)59130);
        p133.mask_SET(7519584397534814174L);
        LoopBackDemoChannel.instance.send(p133); //===============================
        TERRAIN_DATA p134 = LoopBackDemoChannel.instance.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.lat_SET(1415544107);
        p134.lon_SET(-2087324560);
        p134.grid_spacing_SET((char)49158);
        p134.gridbit_SET((char)225);
        p134.data__SET(new short[16], 0);
        LoopBackDemoChannel.instance.send(p134); //===============================
        TERRAIN_CHECK p135 = LoopBackDemoChannel.instance.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lat_SET(29757373);
        p135.lon_SET(1056170955);
        LoopBackDemoChannel.instance.send(p135); //===============================
        TERRAIN_REPORT p136 = LoopBackDemoChannel.instance.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.lat_SET(-1538498723);
        p136.lon_SET(1249986567);
        p136.spacing_SET((char)1983);
        p136.terrain_height_SET(1.4242853E37F);
        p136.current_height_SET(-2.9933478E38F);
        p136.pending_SET((char)23543);
        p136.loaded_SET((char)361);
        LoopBackDemoChannel.instance.send(p136); //===============================
        SCALED_PRESSURE2 p137 = LoopBackDemoChannel.instance.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.time_boot_ms_SET(3873502177L);
        p137.press_abs_SET(-2.0555188E38F);
        p137.press_diff_SET(-2.9388891E38F);
        p137.temperature_SET((short)23301);
        LoopBackDemoChannel.instance.send(p137); //===============================
        ATT_POS_MOCAP p138 = LoopBackDemoChannel.instance.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.time_usec_SET(6036129018830843392L);
        p138.q_SET(new float[4], 0);
        p138.x_SET(6.105344E37F);
        p138.y_SET(-2.959222E38F);
        p138.z_SET(-1.8030703E38F);
        LoopBackDemoChannel.instance.send(p138); //===============================
        SET_ACTUATOR_CONTROL_TARGET p139 = LoopBackDemoChannel.instance.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.time_usec_SET(6629896183081539680L);
        p139.group_mlx_SET((char)71);
        p139.target_system_SET((char)192);
        p139.target_component_SET((char)88);
        p139.controls_SET(new float[8], 0);
        LoopBackDemoChannel.instance.send(p139); //===============================
        ACTUATOR_CONTROL_TARGET p140 = LoopBackDemoChannel.instance.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.time_usec_SET(8827921958424968011L);
        p140.group_mlx_SET((char)179);
        p140.controls_SET(new float[8], 0);
        LoopBackDemoChannel.instance.send(p140); //===============================
        ALTITUDE p141 = LoopBackDemoChannel.instance.new_ALTITUDE();
        PH.setPack(p141);
        p141.time_usec_SET(4966168471124554417L);
        p141.altitude_monotonic_SET(-6.5930913E37F);
        p141.altitude_amsl_SET(2.1249631E38F);
        p141.altitude_local_SET(1.7655227E38F);
        p141.altitude_relative_SET(1.6234852E38F);
        p141.altitude_terrain_SET(3.0421647E38F);
        p141.bottom_clearance_SET(2.419671E38F);
        LoopBackDemoChannel.instance.send(p141); //===============================
        RESOURCE_REQUEST p142 = LoopBackDemoChannel.instance.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.request_id_SET((char)38);
        p142.uri_type_SET((char)2);
        p142.uri_SET(new char[120], 0);
        p142.transfer_type_SET((char)158);
        p142.storage_SET(new char[120], 0);
        LoopBackDemoChannel.instance.send(p142); //===============================
        SCALED_PRESSURE3 p143 = LoopBackDemoChannel.instance.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.time_boot_ms_SET(4127459548L);
        p143.press_abs_SET(-1.103546E38F);
        p143.press_diff_SET(-1.0731478E38F);
        p143.temperature_SET((short) -26277);
        LoopBackDemoChannel.instance.send(p143); //===============================
        FOLLOW_TARGET p144 = LoopBackDemoChannel.instance.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.timestamp_SET(3354887015345989128L);
        p144.est_capabilities_SET((char)89);
        p144.lat_SET(-1047392961);
        p144.lon_SET(-542664998);
        p144.alt_SET(7.0536836E37F);
        p144.vel_SET(new float[3], 0);
        p144.acc_SET(new float[3], 0);
        p144.attitude_q_SET(new float[4], 0);
        p144.rates_SET(new float[3], 0);
        p144.position_cov_SET(new float[3], 0);
        p144.custom_state_SET(231879622049093164L);
        LoopBackDemoChannel.instance.send(p144); //===============================
        CONTROL_SYSTEM_STATE p146 = LoopBackDemoChannel.instance.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.time_usec_SET(2657380522171428729L);
        p146.x_acc_SET(6.008748E37F);
        p146.y_acc_SET(6.508969E36F);
        p146.z_acc_SET(-1.2984029E38F);
        p146.x_vel_SET(-4.1903177E37F);
        p146.y_vel_SET(3.1086851E38F);
        p146.z_vel_SET(7.7270225E37F);
        p146.x_pos_SET(-3.2541252E38F);
        p146.y_pos_SET(-3.0121942E38F);
        p146.z_pos_SET(3.2128164E38F);
        p146.airspeed_SET(2.3111613E38F);
        p146.vel_variance_SET(new float[3], 0);
        p146.pos_variance_SET(new float[3], 0);
        p146.q_SET(new float[4], 0);
        p146.roll_rate_SET(2.8397627E37F);
        p146.pitch_rate_SET(-3.0452718E38F);
        p146.yaw_rate_SET(-1.5071797E38F);
        LoopBackDemoChannel.instance.send(p146); //===============================
        BATTERY_STATUS p147 = LoopBackDemoChannel.instance.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.id_SET((char)211);
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_AVIONICS);
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_UNKNOWN);
        p147.temperature_SET((short) -1696);
        p147.voltages_SET(new char[10], 0);
        p147.current_battery_SET((short)733);
        p147.current_consumed_SET(2041350275);
        p147.energy_consumed_SET(-1176524412);
        p147.battery_remaining_SET((byte)99);
        LoopBackDemoChannel.instance.send(p147); //===============================
        AUTOPILOT_VERSION p148 = LoopBackDemoChannel.instance.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.capabilities_SET(MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION);
        p148.flight_sw_version_SET(647483581L);
        p148.middleware_sw_version_SET(1526068115L);
        p148.os_sw_version_SET(533391488L);
        p148.board_version_SET(1178035452L);
        p148.flight_custom_version_SET(new char[8], 0);
        p148.middleware_custom_version_SET(new char[8], 0);
        p148.os_custom_version_SET(new char[8], 0);
        p148.vendor_id_SET((char)34741);
        p148.product_id_SET((char)51892);
        p148.uid_SET(8112699593334861277L);
        p148.uid2_SET(new char[18], 0, PH);
        LoopBackDemoChannel.instance.send(p148); //===============================
        LANDING_TARGET p149 = LoopBackDemoChannel.instance.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.time_usec_SET(2251874678842220015L);
        p149.target_num_SET((char)205);
        p149.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL);
        p149.angle_x_SET(-1.0768386E38F);
        p149.angle_y_SET(-1.917397E37F);
        p149.distance_SET(-3.1304015E38F);
        p149.size_x_SET(-2.1875037E38F);
        p149.size_y_SET(-3.0182388E38F);
        p149.x_SET(-2.6834721E38F, PH);
        p149.y_SET(-1.6302113E38F, PH);
        p149.z_SET(6.6720923E37F, PH);
        p149.q_SET(new float[4], 0, PH);
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_RADIO_BEACON);
        p149.position_valid_SET((char)233, PH);
        LoopBackDemoChannel.instance.send(p149); //===============================
        NAV_FILTER_BIAS p220 = LoopBackDemoChannel.instance.new_NAV_FILTER_BIAS();
        PH.setPack(p220);
        p220.usec_SET(6614627102144065524L);
        p220.accel_0_SET(1.797043E38F);
        p220.accel_1_SET(-3.0484476E38F);
        p220.accel_2_SET(6.7624576E37F);
        p220.gyro_0_SET(9.825161E37F);
        p220.gyro_1_SET(-3.7642066E37F);
        p220.gyro_2_SET(-1.94299E38F);
        LoopBackDemoChannel.instance.send(p220); //===============================
        RADIO_CALIBRATION p221 = LoopBackDemoChannel.instance.new_RADIO_CALIBRATION();
        PH.setPack(p221);
        p221.aileron_SET(new char[3], 0);
        p221.elevator_SET(new char[3], 0);
        p221.rudder_SET(new char[3], 0);
        p221.gyro_SET(new char[2], 0);
        p221.pitch_SET(new char[5], 0);
        p221.throttle_SET(new char[5], 0);
        LoopBackDemoChannel.instance.send(p221); //===============================
        UALBERTA_SYS_STATUS p222 = LoopBackDemoChannel.instance.new_UALBERTA_SYS_STATUS();
        PH.setPack(p222);
        p222.mode_SET((char)2);
        p222.nav_mode_SET((char)80);
        p222.pilot_SET((char)56);
        LoopBackDemoChannel.instance.send(p222); //===============================
        ESTIMATOR_STATUS p230 = LoopBackDemoChannel.instance.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.time_usec_SET(4835065012917334676L);
        p230.flags_SET(ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ);
        p230.vel_ratio_SET(1.761037E38F);
        p230.pos_horiz_ratio_SET(2.144872E38F);
        p230.pos_vert_ratio_SET(-1.3536423E38F);
        p230.mag_ratio_SET(-3.350912E38F);
        p230.hagl_ratio_SET(4.0386136E37F);
        p230.tas_ratio_SET(-1.2495281E38F);
        p230.pos_horiz_accuracy_SET(-1.7596806E38F);
        p230.pos_vert_accuracy_SET(7.921254E37F);
        LoopBackDemoChannel.instance.send(p230); //===============================
        WIND_COV p231 = LoopBackDemoChannel.instance.new_WIND_COV();
        PH.setPack(p231);
        p231.time_usec_SET(9189071371955779806L);
        p231.wind_x_SET(-2.4951631E38F);
        p231.wind_y_SET(-5.108088E37F);
        p231.wind_z_SET(3.0210779E38F);
        p231.var_horiz_SET(2.8035243E38F);
        p231.var_vert_SET(2.917582E38F);
        p231.wind_alt_SET(2.795074E38F);
        p231.horiz_accuracy_SET(2.158418E38F);
        p231.vert_accuracy_SET(1.4537195E38F);
        LoopBackDemoChannel.instance.send(p231); //===============================
        GPS_INPUT p232 = LoopBackDemoChannel.instance.new_GPS_INPUT();
        PH.setPack(p232);
        p232.time_usec_SET(4625021474765840085L);
        p232.gps_id_SET((char)48);
        p232.ignore_flags_SET(GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY);
        p232.time_week_ms_SET(670280783L);
        p232.time_week_SET((char)39327);
        p232.fix_type_SET((char)67);
        p232.lat_SET(536389686);
        p232.lon_SET(-1937130340);
        p232.alt_SET(2.8145155E38F);
        p232.hdop_SET(-2.7541415E38F);
        p232.vdop_SET(4.3606273E37F);
        p232.vn_SET(2.9428026E38F);
        p232.ve_SET(-1.3578898E38F);
        p232.vd_SET(-2.5965285E38F);
        p232.speed_accuracy_SET(-1.4993163E38F);
        p232.horiz_accuracy_SET(6.491588E37F);
        p232.vert_accuracy_SET(3.496831E37F);
        p232.satellites_visible_SET((char)214);
        LoopBackDemoChannel.instance.send(p232); //===============================
        GPS_RTCM_DATA p233 = LoopBackDemoChannel.instance.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.flags_SET((char)185);
        p233.len_SET((char)36);
        p233.data__SET(new char[180], 0);
        LoopBackDemoChannel.instance.send(p233); //===============================
        HIGH_LATENCY p234 = LoopBackDemoChannel.instance.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED);
        p234.custom_mode_SET(2600809230L);
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING);
        p234.roll_SET((short)12173);
        p234.pitch_SET((short)29491);
        p234.heading_SET((char)54313);
        p234.throttle_SET((byte) - 54);
        p234.heading_sp_SET((short) -11151);
        p234.latitude_SET(768331704);
        p234.longitude_SET(225963150);
        p234.altitude_amsl_SET((short) -255);
        p234.altitude_sp_SET((short) -3492);
        p234.airspeed_SET((char)102);
        p234.airspeed_sp_SET((char)74);
        p234.groundspeed_SET((char)190);
        p234.climb_rate_SET((byte)38);
        p234.gps_nsat_SET((char)73);
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_PPP);
        p234.battery_remaining_SET((char)35);
        p234.temperature_SET((byte) - 40);
        p234.temperature_air_SET((byte) - 70);
        p234.failsafe_SET((char)109);
        p234.wp_num_SET((char)19);
        p234.wp_distance_SET((char)21123);
        LoopBackDemoChannel.instance.send(p234); //===============================
        VIBRATION p241 = LoopBackDemoChannel.instance.new_VIBRATION();
        PH.setPack(p241);
        p241.time_usec_SET(51979688645645701L);
        p241.vibration_x_SET(7.9310763E37F);
        p241.vibration_y_SET(2.7864456E37F);
        p241.vibration_z_SET(2.2019787E38F);
        p241.clipping_0_SET(3396852303L);
        p241.clipping_1_SET(1233697406L);
        p241.clipping_2_SET(1126136919L);
        LoopBackDemoChannel.instance.send(p241); //===============================
        HOME_POSITION p242 = LoopBackDemoChannel.instance.new_HOME_POSITION();
        PH.setPack(p242);
        p242.latitude_SET(-1678651383);
        p242.longitude_SET(95114190);
        p242.altitude_SET(2091321733);
        p242.x_SET(-1.924515E37F);
        p242.y_SET(9.970756E36F);
        p242.z_SET(2.7324866E38F);
        p242.q_SET(new float[4], 0);
        p242.approach_x_SET(6.70753E37F);
        p242.approach_y_SET(6.481566E37F);
        p242.approach_z_SET(-2.5747779E38F);
        p242.time_usec_SET(3087372922822956442L, PH);
        LoopBackDemoChannel.instance.send(p242); //===============================
        SET_HOME_POSITION p243 = LoopBackDemoChannel.instance.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.target_system_SET((char)140);
        p243.latitude_SET(1277531022);
        p243.longitude_SET(110172616);
        p243.altitude_SET(-1628430478);
        p243.x_SET(-1.4318532E38F);
        p243.y_SET(3.1509011E38F);
        p243.z_SET(4.0909881E37F);
        p243.q_SET(new float[4], 0);
        p243.approach_x_SET(-1.9573012E38F);
        p243.approach_y_SET(3.1965126E38F);
        p243.approach_z_SET(-2.2485373E38F);
        p243.time_usec_SET(5056760304479246940L, PH);
        LoopBackDemoChannel.instance.send(p243); //===============================
        MESSAGE_INTERVAL p244 = LoopBackDemoChannel.instance.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.message_id_SET((char)14994);
        p244.interval_us_SET(1780883993);
        LoopBackDemoChannel.instance.send(p244); //===============================
        EXTENDED_SYS_STATE p245 = LoopBackDemoChannel.instance.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_FW);
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING);
        LoopBackDemoChannel.instance.send(p245); //===============================
        ADSB_VEHICLE p246 = LoopBackDemoChannel.instance.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.ICAO_address_SET(1006931255L);
        p246.lat_SET(274061458);
        p246.lon_SET(-1521004816);
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
        p246.altitude_SET(-100988923);
        p246.heading_SET((char)40068);
        p246.hor_velocity_SET((char)49286);
        p246.ver_velocity_SET((short)15927);
        p246.callsign_SET("DEMO", PH);
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_NO_INFO);
        p246.tslc_SET((char)162);
        p246.flags_SET(ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS);
        p246.squawk_SET((char)57407);
        LoopBackDemoChannel.instance.send(p246); //===============================
        COLLISION p247 = LoopBackDemoChannel.instance.new_COLLISION();
        PH.setPack(p247);
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
        p247.id_SET(1109889656L);
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_PERPENDICULAR);
        p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE);
        p247.time_to_minimum_delta_SET(-1.1541038E38F);
        p247.altitude_minimum_delta_SET(-2.0189952E38F);
        p247.horizontal_minimum_delta_SET(2.1935152E38F);
        LoopBackDemoChannel.instance.send(p247); //===============================
        V2_EXTENSION p248 = LoopBackDemoChannel.instance.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.target_network_SET((char)80);
        p248.target_system_SET((char)19);
        p248.target_component_SET((char)238);
        p248.message_type_SET((char)57768);
        p248.payload_SET(new char[249], 0);
        LoopBackDemoChannel.instance.send(p248); //===============================
        MEMORY_VECT p249 = LoopBackDemoChannel.instance.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.address_SET((char)52491);
        p249.ver_SET((char)203);
        p249.type_SET((char)50);
        p249.value_SET(new byte[32], 0);
        LoopBackDemoChannel.instance.send(p249); //===============================
        DEBUG_VECT p250 = LoopBackDemoChannel.instance.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.name_SET("DEMO", PH);
        p250.time_usec_SET(8723134456988799635L);
        p250.x_SET(8.971539E37F);
        p250.y_SET(-9.384997E37F);
        p250.z_SET(5.764247E36F);
        LoopBackDemoChannel.instance.send(p250); //===============================
        NAMED_VALUE_FLOAT p251 = LoopBackDemoChannel.instance.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.time_boot_ms_SET(4016422871L);
        p251.name_SET("DEMO", PH);
        p251.value_SET(-2.7188156E38F);
        LoopBackDemoChannel.instance.send(p251); //===============================
        NAMED_VALUE_INT p252 = LoopBackDemoChannel.instance.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.time_boot_ms_SET(2769256595L);
        p252.name_SET("DEMO", PH);
        p252.value_SET(1400493417);
        LoopBackDemoChannel.instance.send(p252); //===============================
        STATUSTEXT p253 = LoopBackDemoChannel.instance.new_STATUSTEXT();
        PH.setPack(p253);
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_CRITICAL);
        p253.text_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p253); //===============================
        DEBUG p254 = LoopBackDemoChannel.instance.new_DEBUG();
        PH.setPack(p254);
        p254.time_boot_ms_SET(1764526714L);
        p254.ind_SET((char)82);
        p254.value_SET(2.996161E38F);
        LoopBackDemoChannel.instance.send(p254); //===============================
        SETUP_SIGNING p256 = LoopBackDemoChannel.instance.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.target_system_SET((char)149);
        p256.target_component_SET((char)123);
        p256.secret_key_SET(new char[32], 0);
        p256.initial_timestamp_SET(1070080284835231667L);
        LoopBackDemoChannel.instance.send(p256); //===============================
        BUTTON_CHANGE p257 = LoopBackDemoChannel.instance.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.time_boot_ms_SET(491272110L);
        p257.last_change_ms_SET(2701017126L);
        p257.state_SET((char)99);
        LoopBackDemoChannel.instance.send(p257); //===============================
        PLAY_TUNE p258 = LoopBackDemoChannel.instance.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.target_system_SET((char)147);
        p258.target_component_SET((char)14);
        p258.tune_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p258); //===============================
        CAMERA_INFORMATION p259 = LoopBackDemoChannel.instance.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.time_boot_ms_SET(476488093L);
        p259.vendor_name_SET(new char[32], 0);
        p259.model_name_SET(new char[32], 0);
        p259.firmware_version_SET(3642773363L);
        p259.focal_length_SET(-1.6834092E38F);
        p259.sensor_size_h_SET(3.3863454E38F);
        p259.sensor_size_v_SET(-1.3422769E38F);
        p259.resolution_h_SET((char)54521);
        p259.resolution_v_SET((char)58408);
        p259.lens_id_SET((char)141);
        p259.flags_SET(CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE);
        p259.cam_definition_version_SET((char)40945);
        p259.cam_definition_uri_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p259); //===============================
        CAMERA_SETTINGS p260 = LoopBackDemoChannel.instance.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.time_boot_ms_SET(1266766163L);
        p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_IMAGE);
        LoopBackDemoChannel.instance.send(p260); //===============================
        STORAGE_INFORMATION p261 = LoopBackDemoChannel.instance.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.time_boot_ms_SET(3658001998L);
        p261.storage_id_SET((char)143);
        p261.storage_count_SET((char)31);
        p261.status_SET((char)126);
        p261.total_capacity_SET(8.531338E37F);
        p261.used_capacity_SET(-1.6769652E38F);
        p261.available_capacity_SET(-1.2003506E37F);
        p261.read_speed_SET(1.9306373E38F);
        p261.write_speed_SET(-2.1590828E38F);
        LoopBackDemoChannel.instance.send(p261); //===============================
        CAMERA_CAPTURE_STATUS p262 = LoopBackDemoChannel.instance.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.time_boot_ms_SET(3939373681L);
        p262.image_status_SET((char)94);
        p262.video_status_SET((char)205);
        p262.image_interval_SET(1.87235E38F);
        p262.recording_time_ms_SET(3944226691L);
        p262.available_capacity_SET(-2.017882E38F);
        LoopBackDemoChannel.instance.send(p262); //===============================
        CAMERA_IMAGE_CAPTURED p263 = LoopBackDemoChannel.instance.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.time_boot_ms_SET(2556062685L);
        p263.time_utc_SET(4726491969330341638L);
        p263.camera_id_SET((char)78);
        p263.lat_SET(937801947);
        p263.lon_SET(-1223100099);
        p263.alt_SET(-1135734583);
        p263.relative_alt_SET(-1532590294);
        p263.q_SET(new float[4], 0);
        p263.image_index_SET(-778208014);
        p263.capture_result_SET((byte) - 44);
        p263.file_url_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p263); //===============================
        FLIGHT_INFORMATION p264 = LoopBackDemoChannel.instance.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.time_boot_ms_SET(992477173L);
        p264.arming_time_utc_SET(3789930982405777453L);
        p264.takeoff_time_utc_SET(9053130206029185298L);
        p264.flight_uuid_SET(6198419609319055868L);
        LoopBackDemoChannel.instance.send(p264); //===============================
        MOUNT_ORIENTATION p265 = LoopBackDemoChannel.instance.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.time_boot_ms_SET(2457282400L);
        p265.roll_SET(-2.738306E38F);
        p265.pitch_SET(-5.2849667E37F);
        p265.yaw_SET(-1.1287846E38F);
        LoopBackDemoChannel.instance.send(p265); //===============================
        LOGGING_DATA p266 = LoopBackDemoChannel.instance.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.target_system_SET((char)47);
        p266.target_component_SET((char)69);
        p266.sequence_SET((char)35876);
        p266.length_SET((char)190);
        p266.first_message_offset_SET((char)123);
        p266.data__SET(new char[249], 0);
        LoopBackDemoChannel.instance.send(p266); //===============================
        LOGGING_DATA_ACKED p267 = LoopBackDemoChannel.instance.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.target_system_SET((char)173);
        p267.target_component_SET((char)72);
        p267.sequence_SET((char)41484);
        p267.length_SET((char)7);
        p267.first_message_offset_SET((char)60);
        p267.data__SET(new char[249], 0);
        LoopBackDemoChannel.instance.send(p267); //===============================
        LOGGING_ACK p268 = LoopBackDemoChannel.instance.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.target_system_SET((char)19);
        p268.target_component_SET((char)25);
        p268.sequence_SET((char)52537);
        LoopBackDemoChannel.instance.send(p268); //===============================
        VIDEO_STREAM_INFORMATION p269 = LoopBackDemoChannel.instance.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.camera_id_SET((char)25);
        p269.status_SET((char)235);
        p269.framerate_SET(2.747377E37F);
        p269.resolution_h_SET((char)47522);
        p269.resolution_v_SET((char)50620);
        p269.bitrate_SET(316597242L);
        p269.rotation_SET((char)16299);
        p269.uri_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p269); //===============================
        SET_VIDEO_STREAM_SETTINGS p270 = LoopBackDemoChannel.instance.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.target_system_SET((char)203);
        p270.target_component_SET((char)173);
        p270.camera_id_SET((char)45);
        p270.framerate_SET(-3.061024E38F);
        p270.resolution_h_SET((char)19290);
        p270.resolution_v_SET((char)49020);
        p270.bitrate_SET(2821155869L);
        p270.rotation_SET((char)17377);
        p270.uri_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p270); //===============================
        WIFI_CONFIG_AP p299 = LoopBackDemoChannel.instance.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.ssid_SET("DEMO", PH);
        p299.password_SET("DEMO", PH);
        LoopBackDemoChannel.instance.send(p299); //===============================
        PROTOCOL_VERSION p300 = LoopBackDemoChannel.instance.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.version_SET((char)59063);
        p300.min_version_SET((char)36629);
        p300.max_version_SET((char)687);
        p300.spec_version_hash_SET(new char[8], 0);
        p300.library_version_hash_SET(new char[8], 0);
        LoopBackDemoChannel.instance.send(p300); //===============================
        UAVCAN_NODE_STATUS p310 = LoopBackDemoChannel.instance.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.time_usec_SET(1830927958580517119L);
        p310.uptime_sec_SET(2597368676L);
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING);
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION);
        p310.sub_mode_SET((char)40);
        p310.vendor_specific_status_code_SET((char)62458);
        LoopBackDemoChannel.instance.send(p310); //===============================
        UAVCAN_NODE_INFO p311 = LoopBackDemoChannel.instance.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.time_usec_SET(6572650480906643107L);
        p311.uptime_sec_SET(3312860875L);
        p311.name_SET("DEMO", PH);
        p311.hw_version_major_SET((char)108);
        p311.hw_version_minor_SET((char)217);
        p311.hw_unique_id_SET(new char[16], 0);
        p311.sw_version_major_SET((char)35);
        p311.sw_version_minor_SET((char)201);
        p311.sw_vcs_commit_SET(3784150859L);
        LoopBackDemoChannel.instance.send(p311); //===============================
        PARAM_EXT_REQUEST_READ p320 = LoopBackDemoChannel.instance.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.target_system_SET((char)245);
        p320.target_component_SET((char)213);
        p320.param_id_SET("DEMO", PH);
        p320.param_index_SET((short)22842);
        LoopBackDemoChannel.instance.send(p320); //===============================
        PARAM_EXT_REQUEST_LIST p321 = LoopBackDemoChannel.instance.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_system_SET((char)228);
        p321.target_component_SET((char)42);
        LoopBackDemoChannel.instance.send(p321); //===============================
        PARAM_EXT_VALUE p322 = LoopBackDemoChannel.instance.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_id_SET("DEMO", PH);
        p322.param_value_SET("DEMO", PH);
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32);
        p322.param_count_SET((char)23925);
        p322.param_index_SET((char)15520);
        LoopBackDemoChannel.instance.send(p322); //===============================
        PARAM_EXT_SET p323 = LoopBackDemoChannel.instance.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.target_system_SET((char)128);
        p323.target_component_SET((char)16);
        p323.param_id_SET("DEMO", PH);
        p323.param_value_SET("DEMO", PH);
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64);
        LoopBackDemoChannel.instance.send(p323); //===============================
        PARAM_EXT_ACK p324 = LoopBackDemoChannel.instance.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_id_SET("DEMO", PH);
        p324.param_value_SET("DEMO", PH);
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16);
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED);
        LoopBackDemoChannel.instance.send(p324); //===============================
        OBSTACLE_DISTANCE p330 = LoopBackDemoChannel.instance.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.time_usec_SET(8458618497093633682L);
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN);
        p330.distances_SET(new char[72], 0);
        p330.increment_SET((char)228);
        p330.min_distance_SET((char)38726);
        p330.max_distance_SET((char)33273);
        LoopBackDemoChannel.instance.send(p330); //===============================
    }
}
